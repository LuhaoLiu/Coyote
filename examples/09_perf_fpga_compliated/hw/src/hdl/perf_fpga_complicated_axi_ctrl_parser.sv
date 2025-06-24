import lynxTypes::*;
module perf_fpga_complicated_axi_ctrl_parser (
  input  logic                        aclk,
  input  logic                        aresetn,
  
  AXI4L.s                             axi_ctrl,

  output logic                        bench_reset,
  output logic [31:0]                 bench_n_reps,
  input  logic [31:0]                 bench_done,

  output logic [1:0]                  bench_req_ctrl,
  output logic [63:0]                 bench_req_n_beats,
  output logic [LEN_BITS-1:0]         bench_req_len_A,
  output logic [LEN_BITS-1:0]         bench_req_len_B,
  output logic [VADDR_BITS-1:0]       bench_req_vaddr_A,
  output logic [VADDR_BITS-1:0]       bench_req_vaddr_B,
  output logic [PID_BITS-1:0]         bench_req_pid,

  input  logic [63:0]                 bench_timer,
  
  input  logic                        req_accepted
);

/////////////////////////////////////
//          CONSTANTS             //
///////////////////////////////////
localparam integer N_REGS = 12;
localparam integer ADDR_MSB = $clog2(N_REGS);
localparam integer ADDR_LSB = $clog2(AXIL_DATA_BITS/8);
localparam integer AXI_ADDR_BITS = ADDR_LSB + ADDR_MSB;

/////////////////////////////////////
//          REGISTERS             //
///////////////////////////////////
// Internal AXI registers
logic [AXI_ADDR_BITS-1:0] axi_awaddr;
logic axi_awready;
logic [AXI_ADDR_BITS-1:0] axi_araddr;
logic axi_arready;
logic [1:0] axi_bresp;
logic axi_bvalid;
logic axi_wready;
logic [AXIL_DATA_BITS-1:0] axi_rdata;
logic [1:0] axi_rresp;
logic axi_rvalid;
logic aw_en;

// Registers for holding the values read from/to be written to the AXI Lite interface
// These are synchronous but the outputs are combinatorial
logic [N_REGS-1:0][AXIL_DATA_BITS-1:0] ctrl_reg;
logic ctrl_reg_rden;
logic ctrl_reg_wren;

/////////////////////////////////////
//         REGISTER MAP           //
///////////////////////////////////
// 0 (W1S)  : Bench reset
localparam integer BENCH_RESET_REG = 0;

// 1 (WR)   : Number of total requests
localparam integer BENCH_N_REPS_REG = 1;

// 2 (RO)   : Number of completed requests
localparam integer BENCH_DONE_REG = 2;

// 3 (W1S)  : Request type
localparam integer BENCH_REQ_CTRL_REG = 3;

// 4 (WR)   : Request N beats
localparam integer BENCH_REQ_N_BEATS_REG = 4;

// 5 (WR)   : Request length A
localparam integer BENCH_REQ_LEN_A_REG = 5;

// 6 (WR)   : Request length B
localparam integer BENCH_REQ_LEN_B_REG = 6;

// 7 (WR)   : Request virtual address A
localparam integer BENCH_REQ_VADDR_A_REG = 7;

// 8 (WR)   : Request virtual address B
localparam integer BENCH_REQ_VADDR_B_REG = 8;

// 9 (WR)   : Request PID
localparam integer BENCH_REQ_PID_REG = 9;

// 10 (RO)  : Benchmark timer
localparam integer BENCH_TIMER_REG = 10;

/////////////////////////////////////
//         WRITE PROCESS          //
///////////////////////////////////
// Data coming in from host to the vFPGA vie PCIe and XDMA
assign ctrl_reg_wren = axi_wready && axi_ctrl.wvalid && axi_awready && axi_ctrl.awvalid;

always_ff @(posedge aclk) begin
  if (aresetn == 1'b0) begin
    ctrl_reg <= 0;
  end
  else begin
    // Control
    ctrl_reg[BENCH_RESET_REG] <= 0;
    if (req_accepted) begin
      ctrl_reg[BENCH_REQ_CTRL_REG] <= 0;
    end

    if(ctrl_reg_wren) begin
      case (axi_awaddr[ADDR_LSB+:ADDR_MSB])
        BENCH_RESET_REG:     // Benchmark reset
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[BENCH_RESET_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        BENCH_N_REPS_REG:    // Number of total requests
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[BENCH_N_REPS_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        BENCH_REQ_CTRL_REG:  // Request type
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[BENCH_REQ_CTRL_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        BENCH_REQ_N_BEATS_REG: // Request N beats
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[BENCH_REQ_N_BEATS_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        BENCH_REQ_LEN_A_REG: // Request length A
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[BENCH_REQ_LEN_A_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        BENCH_REQ_LEN_B_REG: // Request length B  
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[BENCH_REQ_LEN_B_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        BENCH_REQ_VADDR_A_REG: // Request virtual address A 
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[BENCH_REQ_VADDR_A_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        BENCH_REQ_VADDR_B_REG: // Request virtual address B
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[BENCH_REQ_VADDR_B_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        BENCH_REQ_PID_REG: // Request PID
          for (int i = 0; i < (AXIL_DATA_BITS/8); i++) begin
            if(axi_ctrl.wstrb[i]) begin
              ctrl_reg[BENCH_REQ_PID_REG][(i*8)+:8] <= axi_ctrl.wdata[(i*8)+:8];
            end
          end
        default: ;
      endcase
    end
  end
end    

/////////////////////////////////////
//         READ PROCESS           //
///////////////////////////////////
// Data going to the host from the vFPGA via XDMA and PCIe
assign ctrl_reg_rden = axi_arready & axi_ctrl.arvalid & ~axi_rvalid;

always_ff @(posedge aclk) begin
  if(aresetn == 1'b0) begin
    axi_rdata <= 0;
  end
  else begin
    if(ctrl_reg_rden) begin
      axi_rdata <= 0;

      case (axi_araddr[ADDR_LSB+:ADDR_MSB])
        BENCH_RESET_REG:     // Benchmark reset
          axi_rdata[0] <= ctrl_reg[BENCH_RESET_REG][0];
        BENCH_N_REPS_REG:    // Number of total requests
          axi_rdata[31:0] <= ctrl_reg[BENCH_N_REPS_REG][31:0];
        BENCH_DONE_REG:   // Number of completions
          axi_rdata[31:0] <= bench_done;
        BENCH_REQ_CTRL_REG:  // Request type
          axi_rdata[1:0] <= ctrl_reg[BENCH_REQ_CTRL_REG][1:0];
        BENCH_REQ_N_BEATS_REG: // Request N beats
          axi_rdata[63:0] <= ctrl_reg[BENCH_REQ_N_BEATS_REG][63:0];
        BENCH_REQ_LEN_A_REG: // Request length A
          axi_rdata[LEN_BITS-1:0] <= ctrl_reg[BENCH_REQ_LEN_A_REG][LEN_BITS-1:0];
        BENCH_REQ_LEN_B_REG: // Request length B
          axi_rdata[LEN_BITS-1:0] <= ctrl_reg[BENCH_REQ_LEN_B_REG][LEN_BITS-1:0];
        BENCH_REQ_VADDR_A_REG: // Request virtual address A
          axi_rdata[VADDR_BITS-1:0] <= ctrl_reg[BENCH_REQ_VADDR_A_REG][VADDR_BITS-1:0];
        BENCH_REQ_VADDR_B_REG: // Request virtual address B
          axi_rdata[VADDR_BITS-1:0] <= ctrl_reg[BENCH_REQ_VADDR_B_REG][VADDR_BITS-1:0];
        BENCH_REQ_PID_REG: // Request PID
          axi_rdata[PID_BITS-1:0] <= ctrl_reg[BENCH_REQ_PID_REG][PID_BITS-1:0];
        BENCH_TIMER_REG:  // Benchmark timer
          axi_rdata <= bench_timer;
        default: ;
      endcase
    end
  end 
end

/////////////////////////////////////
//       OUTPUT ASSIGNMENT        //
///////////////////////////////////
always_comb begin
  bench_reset       = ctrl_reg[BENCH_RESET_REG][0];
  bench_n_reps      = ctrl_reg[BENCH_N_REPS_REG][31:0];
  bench_req_ctrl    = ctrl_reg[BENCH_REQ_CTRL_REG][1:0];
  bench_req_n_beats = ctrl_reg[BENCH_REQ_N_BEATS_REG][63:0];
  bench_req_len_A   = ctrl_reg[BENCH_REQ_LEN_A_REG][LEN_BITS-1:0];
  bench_req_len_B   = ctrl_reg[BENCH_REQ_LEN_B_REG][LEN_BITS-1:0];
  bench_req_vaddr_A = ctrl_reg[BENCH_REQ_VADDR_A_REG][VADDR_BITS-1:0];
  bench_req_vaddr_B = ctrl_reg[BENCH_REQ_VADDR_B_REG][VADDR_BITS-1:0];
  bench_req_pid     = ctrl_reg[BENCH_REQ_PID_REG][PID_BITS-1:0];
end

/////////////////////////////////////
//     STANDARD AXI CONTROL       //
///////////////////////////////////
// NOT TO BE EDITED

// I/O
assign axi_ctrl.awready = axi_awready;
assign axi_ctrl.arready = axi_arready;
assign axi_ctrl.bresp = axi_bresp;
assign axi_ctrl.bvalid = axi_bvalid;
assign axi_ctrl.wready = axi_wready;
assign axi_ctrl.rdata = axi_rdata;
assign axi_ctrl.rresp = axi_rresp;
assign axi_ctrl.rvalid = axi_rvalid;

// awready and awaddr
always_ff @(posedge aclk) begin
  if ( aresetn == 1'b0 )
    begin
      axi_awready <= 1'b0;
      axi_awaddr <= 0;
      aw_en <= 1'b1;
    end 
  else
    begin    
      if (~axi_awready && axi_ctrl.awvalid && axi_ctrl.wvalid && aw_en)
        begin
          axi_awready <= 1'b1;
          aw_en <= 1'b0;
          axi_awaddr <= axi_ctrl.awaddr;
        end
      else if (axi_ctrl.bready && axi_bvalid)
        begin
          aw_en <= 1'b1;
          axi_awready <= 1'b0;
        end
      else           
        begin
          axi_awready <= 1'b0;
        end
    end 
end  

// arready and araddr
always_ff @(posedge aclk) begin
  if ( aresetn == 1'b0 )
    begin
      axi_arready <= 1'b0;
      axi_araddr  <= 0;
    end 
  else
    begin    
      if (~axi_arready && axi_ctrl.arvalid)
        begin
          axi_arready <= 1'b1;
          axi_araddr  <= axi_ctrl.araddr;
        end
      else
        begin
          axi_arready <= 1'b0;
        end
    end 
end    

// bvalid and bresp
always_ff @(posedge aclk) begin
  if ( aresetn == 1'b0 )
    begin
      axi_bvalid  <= 0;
      axi_bresp   <= 2'b0;
    end 
  else
    begin    
      if (axi_awready && axi_ctrl.awvalid && ~axi_bvalid && axi_wready && axi_ctrl.wvalid)
        begin
          axi_bvalid <= 1'b1;
          axi_bresp  <= 2'b0;
        end                   
      else
        begin
          if (axi_ctrl.bready && axi_bvalid) 
            begin
              axi_bvalid <= 1'b0; 
            end  
        end
    end
end

// wready
always_ff @(posedge aclk) begin
  if ( aresetn == 1'b0 )
    begin
      axi_wready <= 1'b0;
    end 
  else
    begin    
      if (~axi_wready && axi_ctrl.wvalid && axi_ctrl.awvalid && aw_en )
        begin
          axi_wready <= 1'b1;
        end
      else
        begin
          axi_wready <= 1'b0;
        end
    end 
end  

// rvalid and rresp
always_ff @(posedge aclk) begin
  if ( aresetn == 1'b0 )
    begin
      axi_rvalid <= 0;
      axi_rresp  <= 0;
    end 
  else
    begin    
      if (axi_arready && axi_ctrl.arvalid && ~axi_rvalid)
        begin
          axi_rvalid <= 1'b1;
          axi_rresp  <= 2'b0;
        end   
      else if (axi_rvalid && axi_ctrl.rready)
        begin
          axi_rvalid <= 1'b0;
        end                
    end
end    

endmodule