/**
  * Copyright (c) 2021, Systems Group, ETH Zurich
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
  * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
  * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
  * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  */

`timescale 1ns / 1ps

import lynxTypes::*;

/**
 * @brief   TLB controller utilizing one of the XDMA channels for fast pull mappings.
 *
 * Pulls the VA -> PA mappings from the host memory.
 *
 *  @param TLB_ORDER    Size of the TLBs
 *  @param N_ASSOC      Set associativity
 */
module tlb_controller #(
  parameter integer TLB_ORDER = 10,
  parameter integer DEF_PG_BITS = 12,
  parameter integer N_ASSOC = 4,
  parameter integer DBG_L = 0,
  parameter integer DBG_S = 0,
  parameter integer ID_REG = 0
) (
  input  logic              aclk,
  input  logic              aresetn,

  output logic [4:0]        pg_bits, // support 4K -> 12; 2M -> 21; 1G -> 30
  AXI4S.s                   s_axis,
  tlbIntf.s                 TLB
);

// -- Brief ---------------------------------------------------------
// ------------------------------------------------------------------
// ------------------- TLB Entry (104 bits) -------------------------
//         | VALID | STRM | PID |  TAG  |  PHY  |  HPID  |
//         |   1   |   2  |  6  |   31  |   32  |   32   | 
// ------------------------------------------------------------------
// Assert: 
//  1. PHY_BITS = PADDR_BITS - pg_bits <= 32
//  2. TAG_BITS = VADDR_BITS - TLB_ORDER - pg_bits <= 31
//  3. TOTAL <= TLB_DATA_BITS = 104
// ------------------------------------------------------------------


// -- Decl ----------------------------------------------------------
// ------------------------------------------------------------------

// Constants
localparam integer N_ASSOC_BITS = $clog2(N_ASSOC);

localparam integer PHY_BITS = 32; // >= PADDR_BITS - pg_bits
localparam integer TAG_BITS = 31; // >= VADDR_BITS - TLB_ORDER - pg_bits
localparam integer TLB_VAL_BIT_OFFS = HPID_BITS + PHY_BITS + TAG_BITS + PID_BITS + STRM_BITS; // = 103

localparam integer TLB_SIZE = 2**TLB_ORDER;
localparam integer TLB_IDX_BITS = $clog2(N_ASSOC);

// -- FSM
typedef enum logic[2:0]  {
    ST_IDLE, ST_WAIT_1, ST_WAIT_2, ST_COMP, 
    ST_UPDATE_PG_BITS, ST_UPDATE_PG_BITS_WAIT_1, ST_UPDATE_PG_BITS_WAIT_2, ST_UPDATE_PG_BITS_WAIT_3} state_t;
logic [2:0] state_C, state_N;

// -- Internal
AXI4S axis_fifo_out ();
AXI4S #(.AXI4S_DATA_BITS(AXI_TLB_BITS)) axis_s0 ();
logic [AXI_TLB_BITS-1:0] data_C, data_N;

logic [N_ASSOC-1:0][TLB_DATA_BITS/8-1:0] tlb_wr_en;
logic [N_ASSOC-1:0][TLB_ORDER-1:0] tlb_addr;
logic [N_ASSOC-1:0][TLB_DATA_BITS-1:0] tlb_data_upd_in;
logic [N_ASSOC-1:0][TLB_DATA_BITS-1:0] tlb_data_upd_out;
logic [N_ASSOC-1:0][TLB_DATA_BITS-1:0] tlb_data_lup;

logic [N_ASSOC-1:0] tag_cmp;
logic [TLB_IDX_BITS-1:0] hit_idx;
logic tlb_hit;

logic [N_ASSOC_BITS-1:0] entry_insert_fe, entry_insert_se;
logic [1:0] entry_insert_min;
logic [1:0] curr_ref;
logic filled;

logic [N_ASSOC-1:0] ref_r_wr_en;
logic [N_ASSOC-1:0][TLB_ORDER-1:0] ref_r_addr;
logic [N_ASSOC-1:0][0:0] ref_r_data_upd_in;
logic [N_ASSOC-1:0][0:0] ref_r_data_upd_out;

logic [N_ASSOC-1:0] ref_m_wr_en;
logic [N_ASSOC-1:0][TLB_ORDER-1:0] ref_m_addr;
logic [N_ASSOC-1:0][0:0] ref_m_data_upd_in;
logic [N_ASSOC-1:0][0:0] ref_m_data_upd_out;

logic [31:0] tmr_clr;
logic tmr_clr_valid;
logic [TLB_ORDER-1:0] ref_tmr_addr;

logic [1:0][TLB_ORDER-1:0] addr_ext;
logic [1:0][TAG_BITS-1:0] tag_ext;
logic [1:0][PID_BITS-1:0] pid_ext;
logic [1:0][STRM_BITS-1:0] strm_ext;
logic [1:0] wr_ext;
logic [1:0] val_ext;

logic [PHY_BITS-1:0] data_C_phy;
logic [HPID_BITS-1:0] data_C_hpid;
logic [TLB_ORDER-1:0] data_C_idx;
logic [TAG_BITS-1:0] data_C_tag;
logic [PID_BITS-1:0] data_C_pid;
logic [STRM_BITS-1:0] data_C_strm;
logic data_C_val;

logic [VADDR_BITS+32-1:0] tlb_addr_extended;
logic [TLB_ORDER-1:0] tlb_addr_idx;
logic [TAG_BITS-1:0] tlb_addr_tag;

logic [4:0] pg_bits_C, pg_bits_N;
logic [4:0] pg_bits_update_pending;
logic [TLB_ORDER-1:0] pg_bits_update_tlb_clear_idx;

// -- Def -----------------------------------------------------------
// ------------------------------------------------------------------

// Queueing
axis_data_fifo_128_tlb inst_data_q (
    .s_axis_aresetn(aresetn),
    .s_axis_aclk(aclk),
    .s_axis_tvalid(s_axis.tvalid),
    .s_axis_tready(s_axis.tready),
    .s_axis_tdata(s_axis.tdata),
    .s_axis_tlast(s_axis.tlast),
    .m_axis_tvalid(axis_s0.tvalid),
    .m_axis_tready(axis_s0.tready),
    .m_axis_tdata(axis_s0.tdata),
    .m_axis_tlast()
);

// TLBs 
for (genvar i = 0; i < N_ASSOC; i++) begin
  // BRAM instantiation
  ram_tp_c #(
      .ADDR_BITS(TLB_ORDER),
      .DATA_BITS(TLB_DATA_BITS)
  ) inst_pt_host (
      .clk       (aclk),
      .a_en      (1'b1),
      .a_we      (tlb_wr_en[i]),
      .a_addr    (tlb_addr[i]),
      .b_en      (1'b1),
      .b_addr    (tlb_addr_idx),
      .a_data_in (tlb_data_upd_in[i]),
      .a_data_out(tlb_data_upd_out[i]),
      .b_data_out(tlb_data_lup[i])
    );

    ram_sp_c #(
      .ADDR_BITS(TLB_ORDER),
      .DATA_BITS(1)
    ) inst_ref_r (
      .clk       (aclk),
      .a_en      (1'b1),
      .a_we      (ref_r_wr_en[i]),
      .a_addr    (ref_r_addr[i]),
      .a_data_in (ref_r_data_upd_in[i]),
      .a_data_out(ref_r_data_upd_out[i])
    );

    ram_sp_c #(
      .ADDR_BITS(TLB_ORDER),
      .DATA_BITS(1)
    ) inst_ref_m (
      .clk       (aclk),
      .a_en      (1'b1),
      .a_we      (ref_m_wr_en[i]),
      .a_addr    (ref_m_addr[i]),
      .a_data_in (ref_m_data_upd_in[i]),
      .a_data_out(ref_m_data_upd_out[i])
    );
end

// REG
always_ff @( posedge aclk ) begin : PROC_LUP
    if(aresetn == 1'b0) begin
        state_C <= ST_IDLE;
        pg_bits_C <= DEF_PG_BITS;

        data_C = 0;

`ifdef EN_NRU
        addr_C <= 0;
        wr_C <= 1'b0;
        val_C <= 0;
        hit_addr_C <= 0;
        hit_wr_C <= 1'b0;
        hit_val_C <= 1'b0;
        hit_idx_C <= 0;  
`endif
    end
    else begin
        state_C <= state_N;
        pg_bits_C <= pg_bits_N;

        data_C  = data_N;

`ifdef EN_NRU
        addr_C <= addr_N;
        wr_C <= wr_N;
        val_C <= val_N;
        hit_addr_C <= hit_addr_N;
        hit_wr_C <= hit_wr_N;
        hit_val_C <= hit_val_N;
        hit_idx_C <= hit_idx_N;   
`endif
    end
end

// NSL
always_comb begin: NSL
	state_N = state_C;

	case(state_C)
		ST_IDLE: 
            state_N = axis_s0.tvalid ? 
              (axis_s0.tdata[127:64] == 64'h8000_0000_0000_0000 ? ST_UPDATE_PG_BITS : ST_WAIT_1) : 
              (ST_IDLE); 
            
        ST_WAIT_1:
            state_N = ST_WAIT_2;

        ST_WAIT_2:
            state_N = ST_COMP;

        ST_COMP:
            state_N = ST_IDLE;

        ST_UPDATE_PG_BITS:
            if (pg_bits_update_tlb_clear_idx != {TLB_ORDER{1'b1}}) begin
                state_N = ST_UPDATE_PG_BITS;
            end else begin
                state_N = ST_UPDATE_PG_BITS_WAIT_1;
            end
        
        ST_UPDATE_PG_BITS_WAIT_1:
            state_N = ST_UPDATE_PG_BITS_WAIT_2;
        
        ST_UPDATE_PG_BITS_WAIT_2:
            state_N = ST_UPDATE_PG_BITS_WAIT_3;

        ST_UPDATE_PG_BITS_WAIT_3:
            state_N = ST_IDLE;

	endcase // state_C
end

// DP 
always_comb begin
    data_N    = data_C;
    pg_bits_N = pg_bits_C;
    pg_bits   = pg_bits_C; // for output

    // Input
    axis_s0.tready = 1'b0;

    // TLB
    for(int i = 0; i < N_ASSOC; i++) begin
        tlb_data_upd_in[i][0+:HPID_BITS] = data_C_hpid;
        tlb_data_upd_in[i][HPID_BITS+:PHY_BITS] = data_C_phy;
        tlb_data_upd_in[i][HPID_BITS+PHY_BITS+:TAG_BITS] = data_C_tag;
        tlb_data_upd_in[i][HPID_BITS+PHY_BITS+TAG_BITS+:PID_BITS] = data_C_pid;
        tlb_data_upd_in[i][HPID_BITS+PHY_BITS+TAG_BITS+PID_BITS+:STRM_BITS] = data_C_strm;
        tlb_data_upd_in[i][TLB_VAL_BIT_OFFS+:1] = data_C_val;
        tlb_addr[i] = data_C_idx;
    end
    tlb_wr_en = 0;

    // Ref
    for(int i = 0; i < N_ASSOC; i++) begin
        ref_r_data_upd_in[i] = 0;
        ref_m_data_upd_in[i] = 0;
        ref_r_addr[i] = data_C_idx;
        ref_m_addr[i] = data_C_idx;
        ref_r_wr_en[i] = 0;
        ref_m_wr_en[i] = 0;
    end

    // Update ref
    if(tmr_clr_valid) begin
        ref_r_wr_en = ~0;
        for(int i = 0; i < N_ASSOC; i++)
            ref_r_addr[i] = ref_tmr_addr;
    end

    if(val_ext[1]) begin
        if(tlb_hit) begin
            ref_r_wr_en = ~0;
            ref_r_addr[hit_idx] = addr_ext[1];
            ref_r_data_upd_in[hit_idx] = 1;

            if(wr_ext[1]) begin
                ref_m_wr_en = ~0;
                ref_m_addr[hit_idx] = addr_ext[1];
                ref_m_data_upd_in[hit_idx] = 1;
            end
        end
    end

    // Main state
    case (state_C)
        ST_IDLE: begin
            axis_s0.tready = 1'b1;
            if(axis_s0.tvalid) begin
                data_N = axis_s0.tdata;
            end
        end

        ST_COMP: begin
            if(data_C_val) begin
                // Insertion
                if(!filled) begin
                    tlb_wr_en[entry_insert_fe] = ~0;    
                end
                else begin
                    tlb_wr_en[entry_insert_se] = ~0;    
                end
            end
            else begin
                // Removal
                for(int i = 0; i < N_ASSOC; i++) begin
                    if((tlb_data_upd_out[i][HPID_BITS+PHY_BITS+:TAG_BITS] == data_C_tag) && // tag
                       (tlb_data_upd_out[i][0+:HPID_BITS] == data_C_hpid) // host pid
                       ) begin
                        tlb_wr_en[i] = ~0;
                        ref_r_wr_en[i] = ~0;
                        ref_m_wr_en[i] = ~0;
                    end
                end
            end
        end

        ST_UPDATE_PG_BITS:
            // Clear TLB
            for (int i = 0; i < N_ASSOC; i++) begin
                tlb_addr[i] = pg_bits_update_tlb_clear_idx;
                tlb_wr_en[i] = ~0;
                tlb_data_upd_in[i] = 0;
                ref_r_addr[i] = pg_bits_update_tlb_clear_idx;
                ref_r_wr_en[i] = ~0;
                ref_r_data_upd_in[i] = 0;
                ref_m_addr[i] = pg_bits_update_tlb_clear_idx;
                ref_m_wr_en[i] = ~0;
                ref_m_data_upd_in[i] = 0;
            end

        ST_UPDATE_PG_BITS_WAIT_3:
            pg_bits_N = pg_bits_update_pending;

    endcase
end

// DP of data_C and tlb_addr selection regaring pg_bits
always_comb begin
    data_C_hpid       = data_C[32 +: HPID_BITS];
    data_C_phy        = 0;
    data_C_tag        = 0;
    data_C_pid        = 0;
    data_C_strm       = 0;
    data_C_val        = 0;
    data_C_idx        = data_C[64 +: TLB_ORDER];

    tlb_addr_extended = {32'b0, TLB.addr};
    tlb_addr_idx      = 0;
    tlb_addr_tag      = 0;

    if (pg_bits_C == 12) begin
        // 4K
        data_C_phy[0 +: (PADDR_BITS - 12)]             = data_C[0 +: (PADDR_BITS - 12)];
        data_C_tag[0 +: (VADDR_BITS - TLB_ORDER - 12)] = data_C[64+TLB_ORDER +: (VADDR_BITS - TLB_ORDER - 12)];
        data_C_pid                                     = data_C[64+TLB_ORDER+(VADDR_BITS - TLB_ORDER - 12) +: PID_BITS];
        data_C_strm                                    = data_C[64+TLB_ORDER+(VADDR_BITS - TLB_ORDER - 12)+PID_BITS +: STRM_BITS];
        data_C_val                                     = data_C[64+TLB_ORDER+(VADDR_BITS - TLB_ORDER - 12)+PID_BITS+STRM_BITS +: 1];

        tlb_addr_idx                                   = tlb_addr_extended[12 +: TLB_ORDER];
        tlb_addr_tag                                   = tlb_addr_extended[12+TLB_ORDER +: TAG_BITS];
    end else if (pg_bits_C == 21) begin
        // 2M
        data_C_phy[0 +: (PADDR_BITS - 21)]             = data_C[0 +: (PADDR_BITS - 21)];
        data_C_tag[0 +: (VADDR_BITS - TLB_ORDER - 21)] = data_C[64+TLB_ORDER +: (VADDR_BITS - TLB_ORDER - 21)];
        data_C_pid                                     = data_C[64+TLB_ORDER+(VADDR_BITS - TLB_ORDER - 21) +: PID_BITS];
        data_C_strm                                    = data_C[64+TLB_ORDER+(VADDR_BITS - TLB_ORDER - 21)+PID_BITS +: STRM_BITS];
        data_C_val                                     = data_C[64+TLB_ORDER+(VADDR_BITS - TLB_ORDER - 21)+PID_BITS+STRM_BITS +: 1];

        tlb_addr_idx                                   = tlb_addr_extended[21 +: TLB_ORDER];
        tlb_addr_tag                                   = tlb_addr_extended[21+TLB_ORDER +: TAG_BITS];
    end else if (pg_bits_C == 30) begin
        // 1G
        data_C_phy[0 +: (PADDR_BITS - 30)]             = data_C[0 +: (PADDR_BITS - 30)];
        data_C_tag[0 +: (VADDR_BITS - TLB_ORDER - 30)] = data_C[64+TLB_ORDER +: (VADDR_BITS - TLB_ORDER - 30)];
        data_C_pid                                     = data_C[64+TLB_ORDER+(VADDR_BITS - TLB_ORDER - 30) +: PID_BITS];
        data_C_strm                                    = data_C[64+TLB_ORDER+(VADDR_BITS - TLB_ORDER - 30)+PID_BITS +: STRM_BITS];
        data_C_val                                     = data_C[64+TLB_ORDER+(VADDR_BITS - TLB_ORDER - 30)+PID_BITS+STRM_BITS +: 1];

        tlb_addr_idx                                   = tlb_addr_extended[30 +: TLB_ORDER];
        tlb_addr_tag                                   = tlb_addr_extended[30+TLB_ORDER +: TAG_BITS];
    end
end

// Find first order
always_comb begin   
    filled = 1'b1;
    entry_insert_fe = 0;

    for(int i = 0; i < N_ASSOC; i++) begin
        if(!tlb_data_upd_out[i][TLB_VAL_BIT_OFFS]) begin
            filled = 1'b0;
            entry_insert_fe = i;
            break;
        end
    end
end

// Find second order
always_comb begin   
    entry_insert_se = 0;
    entry_insert_min = 2'b11;
    curr_ref = 0;

    for(int i = 0; i < N_ASSOC; i++) begin
        curr_ref = {ref_r_data_upd_out[i], ref_m_data_upd_out[i]};
        if(curr_ref <= entry_insert_min) begin
            entry_insert_se = i;
            entry_insert_min = curr_ref;
        end
    end
end

// TLB Reference map timer
always_ff @( posedge aclk ) begin : TLB_CLR
    if(aresetn == 1'b0) begin
        // Timer updates
        tmr_clr_valid <= 1'b0;
        tmr_clr <= 0;
        ref_tmr_addr <= 0;

        // External updates
        addr_ext <= 'X;
        pid_ext <= 'X;
        strm_ext <= 'X;
        wr_ext <= 'X;
        val_ext <= 0;
    end
    else begin
        // Timer updates
        if(tmr_clr_valid && ref_tmr_addr == TLB_SIZE-1) begin
            tmr_clr_valid <= 1'b0;
            tmr_clr <= 0;
            ref_tmr_addr <= 0;
        end
        else begin
            if(tmr_clr == TLB_TMR_REF_CLR) begin
                tmr_clr_valid <= 1'b1;
                ref_tmr_addr <= tmr_clr_valid ? ref_tmr_addr + 1 : ref_tmr_addr;
            end
            else begin
                tmr_clr <= tmr_clr + 1;
            end
        end

        // External updates
        addr_ext[0] <= tlb_addr_idx;
        tag_ext[0] <= tlb_addr_tag;
        pid_ext[0] <= TLB.pid;
        strm_ext[0] <= TLB.strm;
        wr_ext[0] <= TLB.wr;
        val_ext[0] <= TLB.valid;

        addr_ext[1] <= addr_ext[0];
        tag_ext[1] <= tag_ext[0];
        pid_ext[1] <= pid_ext[0];
        strm_ext[1] <= strm_ext[0];
        wr_ext[1] <= wr_ext[0];
        val_ext[1] <= val_ext[0];
    end
end

// Update pg_bits
always_ff @( posedge aclk ) begin
    if(aresetn == 1'b0) begin
        pg_bits_update_pending <= 0;
        pg_bits_update_tlb_clear_idx <= 0;
    end else begin
        pg_bits_update_pending <= pg_bits_update_pending;
        pg_bits_update_tlb_clear_idx <= pg_bits_update_tlb_clear_idx;
        if (state_C == ST_IDLE) begin
            pg_bits_update_pending <= axis_s0.tdata[4:0];
            pg_bits_update_tlb_clear_idx <= 0;
        end else if (state_C == ST_UPDATE_PG_BITS) begin
            pg_bits_update_tlb_clear_idx <= pg_bits_update_tlb_clear_idx + 1;
        end
    end
end

//
// External FSM access
//

// Hit/Miss combinational logic
always_comb begin
    tag_cmp = 0;

	tlb_hit = 1'b0;
	hit_idx = 0;

	// Pages
	for (int i = 0; i < N_ASSOC; i++) begin
        // tag cmp
        tag_cmp[i] = 
        (tlb_data_lup[i][HPID_BITS+PHY_BITS+:TAG_BITS] == tag_ext[1]) && // tag hit
        (tlb_data_lup[i][HPID_BITS+PHY_BITS+TAG_BITS+:PID_BITS] == pid_ext[1]) && // pid hit
`ifdef EN_STRM
`ifdef EN_MEM        
        (tlb_data_lup[i][HPID_BITS+PHY_BITS+TAG_BITS+PID_BITS+:STRM_BITS] == strm_ext[1]) && //TLB.strm) && // strm hit
`endif
`endif
        tlb_data_lup[i][TLB_VAL_BIT_OFFS];

        if(tag_cmp[i]) begin 
            tlb_hit = 1'b1;
            hit_idx = i;
        end
	end
end

// Output
always_ff @( posedge aclk ) begin
    if(~aresetn) begin
        TLB.data <= 0;
        TLB.hit <= 1'b0;
    end
    else begin
        TLB.data <= tlb_data_lup[hit_idx];
        TLB.hit <= tlb_hit;
    end
end

/////////////////////////////////////////////////////////////////////////////
// DEBUG
/////////////////////////////////////////////////////////////////////////////
`define DBG_TLB_CONTROLLER
`ifdef DBG_TLB_CONTROLLER

wire ila_tlb_valid;
wire ila_tlb_wr;
wire [5:0] ila_tlb_pid;
wire [1:0] ila_tlb_strm;
wire ila_tlb_hit;
wire [47:0] ila_tlb_addr;
wire [103:0] ila_tlb_data;
wire ila_axis_s0_tvalid;
wire ila_axis_s0_tready;
wire [127:0] ila_axis_s0_tdata; 
assign ila_tlb_valid = TLB.valid;
assign ila_tlb_wr = TLB.wr;
assign ila_tlb_pid = TLB.pid;
assign ila_tlb_strm = TLB.strm;
assign ila_tlb_hit = TLB.hit;
assign ila_tlb_addr = TLB.addr;
assign ila_tlb_data = TLB.data;
assign ila_axis_s0_tvalid = axis_s0.tvalid;
assign ila_axis_s0_tready = axis_s0.tready;
assign ila_axis_s0_tdata = axis_s0.tdata;

if(DEF_PG_BITS == 21) begin
if(ID_REG == 0) begin
ila_tlb_ctrl inst_ila_tlb_ctrl (
    .clk(aclk),
    .probe0(ila_tlb_valid),
    .probe1(ila_tlb_wr),
    .probe2(ila_tlb_pid), // 6
    .probe3(ila_tlb_strm), // 2
    .probe4(ila_tlb_hit),
    .probe5(ila_tlb_addr), // 48
    .probe6(ila_tlb_data), // 104
    .probe7(state_C), // 3
    .probe8(ila_axis_s0_tvalid),
    .probe9(ila_axis_s0_tready),
    .probe10(ila_axis_s0_tdata), // 128
    .probe11(data_C), // 128
    .probe12(hit_idx), // 2
    .probe13(tlb_hit), 
    .probe14(entry_insert_fe), // 2
    .probe15(entry_insert_se), // 2
    .probe16(curr_ref), // 2
    .probe17(filled),
    .probe18(tmr_clr), // 32
    .probe19(tmr_clr_valid),
    .probe20(ref_tmr_addr), // 10
    .probe21(addr_ext[1]), // 10
    .probe22(wr_ext[1]), 
    .probe23(val_ext[1]),
    .probe24(pg_bits) // 5
);
end
end

`endif

endmodule // tlb_controller
