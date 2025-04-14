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
module stlb_controller #(
  parameter integer TLB_ORDER = 10,
  parameter integer N_ASSOC = 4,
  parameter integer ID_REG = 0
) (
  input  logic              aclk,
  input  logic              aresetn,

  output logic [4:0]        pg_bits, // support 4K -> 12; 2M -> 21; 1G -> 30
  AXI4S.s                   s_axis,
  tlbIntf.s                 TLB
);

// -- Decl ----------------------------------------------------------
// ------------------------------------------------------------------

// Constants
localparam integer N_ASSOC_BITS = $clog2(N_ASSOC);

localparam integer PHY_BITS = 32; // PADDR_BITS - PG_S_BITS
localparam integer VIR_BITS = 36; // VADDR_BITS - PG_S_BITS
localparam integer TAG_BITS = 31;
localparam integer TLB_VAL_BIT_OFFS = HPID_BITS + PHY_BITS + TAG_BITS + PID_BITS + STRM_BITS;

localparam integer TLB_SIZE = 2**TLB_ORDER;
localparam integer TLB_IDX_BITS = $clog2(N_ASSOC);

// -- FSM
typedef enum logic [2:0] {
  ST_IDLE,
  ST_CHECK,
  ST_NEW_ENTRY,
  ST_MODIFY_PT,
  ST_APPEND_PT,
  ST_DEL_ENTRY
} state_t;
logic [2:0] state_C, state_N;

// -- Internal
AXI4S #(.AXI4S_DATA_BITS(AXI_TLB_BITS)) axis_s0 ();
logic [AXI_TLB_BITS-1:0] data_N;
logic [PHY_BITS-1:0]  data_C_paddr;
logic [HPID_BITS-1:0] data_C_hpid;
logic [VIR_BITS-1:0]  data_C_vaddr;
logic [PID_BITS-1:0]  data_C_pid;
logic [STRM_BITS-1:0] data_C_strm;
logic [1-1:0]         data_C_valid;
logic [5-1:0]         data_C_pg_bits;

logic [N_ASSOC-1:0][PHY_BITS/8-1:0] pt_wr_en;        // Paddr Table
logic [N_ASSOC-1:0][TLB_ORDER-1:0]  pt_addr_upd;
logic [N_ASSOC-1:0][PHY_BITS-1:0]   pt_data_upd_in;
logic [N_ASSOC-1:0][PHY_BITS-1:0]   pt_data_upd_out;
logic [N_ASSOC-1:0][TLB_ORDER-1:0]  pt_addr_lup;
logic [N_ASSOC-1:0][PHY_BITS-1:0]   pt_data_lup;

logic [VADDR_BITS-1:0] tlb_addr_s, tlb_addr_l, tlb_addr_h;

logic [N_ASSOC-1:0][VIR_BITS-1:0]   vaddr_diff_upd;
logic [N_ASSOC-1:0][VIR_BITS-1:0]   vaddr_diff_lup;

logic hit_modify_upd, hit_append_upd, hit_delete_upd;
logic [TLB_IDX_BITS-1:0] hit_idx_modify_upd, hit_idx_modify_upd_saved;
logic [TLB_IDX_BITS-1:0] hit_idx_append_upd, hit_idx_append_upd_saved;
logic [TLB_IDX_BITS-1:0] hit_idx_delete_upd, hit_idx_delete_upd_saved;

logic [N_ASSOC_BITS-1:0] entry_insert_fe, entry_insert_se;
logic [7:0] min_ref_cnt;
logic filled;

logic [31:0] tmr_clr;

logic hit_lup;
logic [TLB_IDX_BITS-1:0] hit_idx_lup;
logic [1:0] hit_lup_saved;
logic [1:0][TLB_IDX_BITS-1:0] hit_idx_lup_saved;
logic [1:0] tlb_val_saved;
logic [1:0][HPID_BITS-1:0] hpid_saved;
logic [1:0][5-1:0] pg_bits_saved;

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
// Registers for TLB entries
logic [N_ASSOC-1:0][HPID_BITS-1:0]  entry_hpid;
logic [N_ASSOC-1:0][VIR_BITS-1:0]   entry_vaddr;
logic [N_ASSOC-1:0][PID_BITS-1:0]   entry_pid;
logic [N_ASSOC-1:0][STRM_BITS-1:0]  entry_strm;
logic [N_ASSOC-1:0][TLB_ORDER:0]    entry_valid_sz;
logic [N_ASSOC-1:0][5-1:0]          entry_pg_bits;
logic [N_ASSOC-1:0][7:0]            entry_ref_cnt;
// BRAM for physical addresses
for (genvar i = 0; i < N_ASSOC; i++) begin
  // BRAM instantiation
  ram_tp_c #(
    .ADDR_BITS(TLB_ORDER),
    .DATA_BITS(PHY_BITS)
  ) inst_paddr_table_host (
    .clk       (aclk),
    .a_en      (1'b1),
    .a_we      (pt_wr_en[i]),
    .a_addr    (pt_addr_upd[i]),
    .b_en      (1'b1),
    .b_addr    (pt_addr_lup[i]),
    .a_data_in (pt_data_upd_in[i]),
    .a_data_out(pt_data_upd_out[i]),
    .b_data_out(pt_data_lup[i])
  );
end

// REG for TLB entries
always_ff @(posedge aclk) begin
  if (aresetn == 1'b0) begin
    for (int i = 0; i < N_ASSOC; i++) begin
      entry_hpid[i]     <= 0;
      entry_vaddr[i]    <= 0;
      entry_pid[i]      <= 0;
      entry_strm[i]     <= 0;
      entry_valid_sz[i] <= 0;
      entry_pg_bits[i]  <= 0;
      entry_ref_cnt[i]  <= 0;

      tmr_clr           <= 0;
    end
  end else begin
    tmr_clr <= tmr_clr + 1;
    for (int i = 0; i < N_ASSOC; i++) begin
      entry_hpid[i]     <= entry_hpid[i];
      entry_vaddr[i]    <= entry_vaddr[i];
      entry_pid[i]      <= entry_pid[i];
      entry_strm[i]     <= entry_strm[i];
      entry_valid_sz[i] <= entry_valid_sz[i];
      entry_pg_bits[i]  <= entry_pg_bits[i];
      entry_ref_cnt[i]  <= entry_ref_cnt[i];
    end
    // Halve ref count by timer
    if (tmr_clr == TLB_TMR_REF_CLR) begin
      tmr_clr <= 0;
      for (int i = 0; i < N_ASSOC; i++) begin
        entry_ref_cnt[i] <= entry_ref_cnt[i] >> 1; // divide by 2
      end
    end
    // Add ref count by lookup hit
    if (tlb_val_saved[1] && hit_lup_saved[1]) begin
      entry_ref_cnt[hit_idx_lup_saved[1]] <= entry_ref_cnt[hit_idx_lup_saved[1]] + 1;
      if (tmr_clr == TLB_TMR_REF_CLR) begin
        entry_ref_cnt[hit_idx_lup_saved[1]] <= (entry_ref_cnt[hit_idx_lup_saved[1]] >> 1) + 1;
      end
    end
    // Update TLB entries
    if (state_C == ST_NEW_ENTRY) begin
      if (!filled) begin
        entry_hpid[entry_insert_fe]     <= data_C_hpid;
        entry_vaddr[entry_insert_fe]    <= data_C_vaddr;
        entry_pid[entry_insert_fe]      <= data_C_pid;
        entry_strm[entry_insert_fe]     <= data_C_strm;
        entry_valid_sz[entry_insert_fe] <= 1;
        entry_pg_bits[entry_insert_fe]  <= data_C_pg_bits;
        entry_ref_cnt[entry_insert_fe]  <= 1;
      end else begin
        entry_hpid[entry_insert_se]     <= data_C_hpid;
        entry_vaddr[entry_insert_se]    <= data_C_vaddr;
        entry_pid[entry_insert_se]      <= data_C_pid;
        entry_strm[entry_insert_se]     <= data_C_strm;
        entry_valid_sz[entry_insert_se] <= 1;
        entry_pg_bits[entry_insert_se]  <= data_C_pg_bits;
        entry_ref_cnt[entry_insert_se]  <= 1;
      end
    end else if (state_C == ST_MODIFY_PT) begin
      // nothing to do with tlb entry
    end else if (state_C == ST_APPEND_PT) begin
      entry_valid_sz[hit_idx_append_upd] <= entry_valid_sz[hit_idx_append_upd] + 1;
    end else if (state_C == ST_DEL_ENTRY) begin
      entry_valid_sz[hit_idx_delete_upd] <= 0;
    end
  end
end

// REG
always_ff @(posedge aclk) begin
  if (aresetn == 1'b0) begin
    state_C        <= ST_IDLE;
    data_C_paddr   <= 0;
    data_C_hpid    <= 0;
    data_C_vaddr   <= 0;
    data_C_pid     <= 0;
    data_C_strm    <= 0;
    data_C_valid   <= 0;
    data_C_pg_bits <= 0;
  end else begin
    state_C        <= state_N;
    data_C_paddr   <= data_N[0  +: PHY_BITS];
    data_C_hpid    <= data_N[(PHY_BITS) +: HPID_BITS];
    data_C_vaddr   <= data_N[(PHY_BITS+HPID_BITS) +: VIR_BITS];
    data_C_pid     <= data_N[(PHY_BITS+HPID_BITS+VIR_BITS) +: PID_BITS];
    data_C_strm    <= data_N[(PHY_BITS+HPID_BITS+VIR_BITS+PID_BITS) +: STRM_BITS];
    data_C_valid   <= data_N[(PHY_BITS+HPID_BITS+VIR_BITS+PID_BITS+STRM_BITS) +: 1];
    data_C_pg_bits <= data_N[(PHY_BITS+HPID_BITS+VIR_BITS+PID_BITS+STRM_BITS+1) +: 5];
  end
end

// NSL
always_comb begin
  state_N = state_C;
  
  case (state_N)
    ST_IDLE: begin
      state_N = axis_s0.tvalid ? ST_CHECK : ST_IDLE;
    end
    ST_CHECK: begin
      if (hit_modify_upd && data_C_valid) begin
        state_N = ST_MODIFY_PT;
      end else if (hit_append_upd && data_C_valid) begin
        state_N = ST_APPEND_PT;
      end else if (data_C_valid) begin
        state_N = ST_NEW_ENTRY;
      end else if (hit_delete_upd) begin
        state_N = ST_DEL_ENTRY;
      end else begin
        state_N = ST_IDLE;
      end
    end
    ST_NEW_ENTRY: begin
      state_N = ST_IDLE;
    end
    ST_MODIFY_PT: begin
      state_N = ST_IDLE;
    end
    ST_APPEND_PT: begin
      state_N = ST_IDLE;
    end
    ST_DEL_ENTRY: begin
      state_N = ST_IDLE;
    end
  endcase
end

// DP
always_comb begin
  data_N = {16'b0, data_C_pg_bits, data_C_valid, data_C_strm, data_C_pid, data_C_vaddr, data_C_hpid, data_C_paddr};

  // Input
  axis_s0.tready = 1'b0;

  // Paddr Table
  for (int i = 0; i < N_ASSOC; i++) begin
    pt_wr_en[i]       = 0;
    vaddr_diff_upd[i] = data_C_vaddr - entry_vaddr[i];
    pt_addr_upd[i]    = vaddr_diff_upd[i][0 +: TLB_ORDER];
    pt_data_upd_in[i] = data_C_paddr;
    if (entry_pg_bits[i] == PG_S_BITS) begin
      vaddr_diff_lup[i] = tlb_addr_s - entry_vaddr[i];
    end else if (entry_pg_bits[i] == PG_L_BITS) begin
      vaddr_diff_lup[i] = tlb_addr_l - entry_vaddr[i];
    end else if (entry_pg_bits[i] == 30) begin
      vaddr_diff_lup[i] = tlb_addr_h - entry_vaddr[i];
    end else begin
      vaddr_diff_lup[i] = 0;
    end
    pt_addr_lup[i]    = vaddr_diff_lup[i][0 +: TLB_ORDER];
  end

  // Main State
  // Update Paddr Table
  case (state_C)
    ST_IDLE: begin
      axis_s0.tready = 1'b1;
      if (axis_s0.tvalid) begin
        data_N = axis_s0.tdata;
      end
    end
    ST_CHECK: begin
      // nothing to do with BRAM
    end
    ST_NEW_ENTRY: begin
      if (!filled) begin
        pt_wr_en[entry_insert_fe] = ~0;
        pt_addr_upd[entry_insert_fe] = 0;
      end else begin
        pt_wr_en[entry_insert_se] = ~0;
        pt_addr_upd[entry_insert_se] = 0;
      end
    end
    ST_MODIFY_PT: begin
      pt_wr_en[hit_idx_modify_upd_saved] = ~0;
    end
    ST_APPEND_PT: begin
      pt_wr_en[hit_idx_append_upd_saved] = ~0;
    end
    ST_DEL_ENTRY: begin
      // nothing to do with BRAM
    end
  endcase

end

// Hit dectection for Update logic
always_ff @(posedge aclk) begin
  if (aresetn == 1'b0) begin
    hit_idx_append_upd_saved <= 0;
    hit_idx_modify_upd_saved <= 0;
    hit_idx_delete_upd_saved <= 0;
  end else begin
    hit_idx_append_upd_saved <= hit_idx_append_upd;
    hit_idx_modify_upd_saved <= hit_idx_modify_upd;
    hit_idx_delete_upd_saved <= hit_idx_delete_upd;
  end
end

always_comb begin
  hit_modify_upd = 0;
  hit_append_upd = 0;
  hit_delete_upd = 0;
  hit_idx_modify_upd = 0;
  hit_idx_append_upd = 0;
  hit_idx_delete_upd = 0;

  for (int i = 0; i < N_ASSOC; i++) begin
    if (
      vaddr_diff_upd[i] <= entry_valid_sz[i] && data_C_vaddr >= entry_vaddr[i] && // vaddr match
      data_C_pg_bits == entry_pg_bits[i] // pg_bits match
    ) begin
      if (
        data_C_pid == entry_pid[i] && // pid match
        data_C_strm == entry_strm[i]  // strm match
      ) begin
        // full hit: new tlb
        if (vaddr_diff_upd[i] < entry_valid_sz[i]) begin
          // within the valid range
          hit_modify_upd = 1;
          hit_idx_modify_upd = i;
        end else if (entry_valid_sz[i] != ~0) begin
          // out of valid range by 1: append
          hit_append_upd = 1;
          hit_idx_append_upd = i;
        end
      end
      // partial hit: delete tlb
      if (vaddr_diff_upd[i] < entry_valid_sz[i]) begin
        // within the valid range
        hit_delete_upd = 1;
        hit_idx_delete_upd = i;
      end
    end
  end
end

// Hit dectection for TLB Lookup logic
always_ff @(posedge aclk) begin
  if (aresetn == 1'b0) begin
    hit_lup_saved     <= 0;
    hit_idx_lup_saved <= 0;
    tlb_val_saved     <= 0;
    hpid_saved        <= 0;
    pg_bits_saved     <= 0;
  end else begin
    hit_lup_saved[0]     <= hit_lup;
    hit_idx_lup_saved[0] <= hit_idx_lup;
    tlb_val_saved[0]     <= TLB.valid;
    hpid_saved[0]        <= entry_hpid[hit_idx_lup];
    pg_bits_saved[0]     <= entry_pg_bits[hit_idx_lup];

    hit_lup_saved[1]     <= hit_lup_saved[0];
    hit_idx_lup_saved[1] <= hit_idx_lup_saved[0];
    tlb_val_saved[1]     <= tlb_val_saved[0];
    hpid_saved[1]        <= hpid_saved[0];
    pg_bits_saved[1]     <= pg_bits_saved[0];
  end
end

always_comb begin
  hit_lup = 0;
  hit_idx_lup = 0;

  for (int i = 0; i < N_ASSOC; i++) begin
    if (
      vaddr_diff_lup[i] < entry_valid_sz[i] && tlb_addr_s >= entry_vaddr[i] && // vaddr match
      TLB.pid == entry_pid[i] && // pid match
      TLB.strm == entry_strm[i] // strm match
    ) begin
      hit_lup = 1;
      hit_idx_lup = i;
    end
  end
end

// First Update Order
always_comb begin   
  filled = 1'b1;
  entry_insert_fe = 0;

  for(int i = 0; i < N_ASSOC; i++) begin
    if (entry_valid_sz[i] == 0) begin
      filled = 1'b0;
      entry_insert_fe = i;
      break;
    end
  end
end

// Second Update Order
always_comb begin
  entry_insert_se = 0;
  min_ref_cnt = 8'hFF;

  for(int i = 0; i < N_ASSOC; i++) begin
    if (entry_ref_cnt[i] <= min_ref_cnt) begin
      min_ref_cnt = entry_ref_cnt[i];
      entry_insert_se = i;
    end
  end
end

// TLB Addr selection by page size
always_comb begin
  tlb_addr_s = 0;
  tlb_addr_l = 0;
  tlb_addr_h = 0;
  tlb_addr_s = TLB.addr[VADDR_BITS-1:PG_S_BITS];
  tlb_addr_l = TLB.addr[VADDR_BITS-1:PG_L_BITS];
  tlb_addr_h = TLB.addr[VADDR_BITS-1:30];
end

// Output
always_ff @(posedge aclk) begin
  if (aresetn == 1'b0) begin
    pg_bits <= 0;
    TLB.hit <= 0;
    TLB.data <= 0;
  end else begin
    pg_bits <= pg_bits_saved[1];
    TLB.hit <= hit_lup_saved[1];
    TLB.data <= 0;
    TLB.data[0 +: HPID_BITS] <= hpid_saved[1];
    TLB.data[HPID_BITS +: PHY_BITS] <= pt_data_lup[hit_idx_lup_saved[1]];
    TLB.data[TLB_VAL_BIT_OFFS] <= hit_lup_saved[1];
  end
end

/////////////////////////////////////////////////////////////////////////////
// DEBUG
/////////////////////////////////////////////////////////////////////////////
`define DBG_STLB_CONTROLLER
`ifdef DBG_STLB_CONTROLLER

wire ila_axis_s0_tvalid;
wire ila_axis_s0_tready;
wire [127:0] ila_axis_s0_tdata; 
wire [47:0] ila_tlb_addr;
wire [103:0] ila_tlb_data;
wire ila_tlb_hit;

assign ila_axis_s0_tvalid = axis_s0.tvalid;
assign ila_axis_s0_tready = axis_s0.tready;
assign ila_axis_s0_tdata = axis_s0.tdata;
assign ila_tlb_addr = TLB.addr;
assign ila_tlb_data = TLB.data;
assign ila_tlb_hit = TLB.hit;

if(ID_REG == 0) begin
if(N_ASSOC == 4) begin
ila_stlb_ctrl inst_ila_stlb_ctrl (
    .clk(aclk),
    .probe0(entry_vaddr[0]), // 36
    .probe1(entry_vaddr[1]), // 36
    .probe2(entry_vaddr[2]), // 36
    .probe3(entry_vaddr[3]), // 36
    .probe4(entry_pg_bits[0]), // 5
    .probe5(entry_pg_bits[1]), // 5
    .probe6(entry_pg_bits[2]), // 5
    .probe7(entry_pg_bits[3]), // 5
    .probe8(ila_axis_s0_tvalid),
    .probe9(ila_axis_s0_tready),
    .probe10(ila_axis_s0_tdata), // 128
    .probe11(data_N),            // 128
    .probe12(hit_modify_upd),
    .probe13(hit_idx_modify_upd), // 2
    .probe14(hit_append_upd),
    .probe15(hit_idx_append_upd), // 2
    .probe16({pt_wr_en[3], pt_wr_en[2], pt_wr_en[1], pt_wr_en[0]}), // 16
    .probe17(pt_addr_upd[0]), // 10
    .probe18(pt_addr_upd[1]), // 10
    .probe19(pt_addr_upd[2]), // 10
    .probe20(pt_addr_upd[3]), // 10
    .probe21(pt_data_upd_in[0]), // 32
    .probe22(pt_data_upd_in[1]), // 32
    .probe23(pt_data_upd_in[2]), // 32
    .probe24(pt_data_upd_in[3]), // 32
    .probe25(ila_tlb_addr), // 48
    .probe26(ila_tlb_data), // 104
    .probe27(ila_tlb_hit),
    .probe28(hit_lup),
    .probe29(hit_idx_lup), // 2
    .probe30(pt_addr_lup[0]), // 10
    .probe31(pt_addr_lup[1]), // 10
    .probe32(pt_addr_lup[2]), // 10
    .probe33(pt_addr_lup[3]), // 10
    .probe34(pt_data_lup[0]), // 32
    .probe35(pt_data_lup[1]), // 32
    .probe36(pt_data_lup[2]), // 32
    .probe37(pt_data_lup[3]), // 32
    .probe38(entry_valid_sz[0]), // 11
    .probe39(entry_valid_sz[1]), // 11
    .probe40(entry_valid_sz[2]), // 11
    .probe41(entry_valid_sz[3]), // 11
    .probe42(entry_insert_fe), // 2
    .probe43(entry_insert_se), // 2
    .probe44(state_C) // 3
);
end
end

`endif

endmodule