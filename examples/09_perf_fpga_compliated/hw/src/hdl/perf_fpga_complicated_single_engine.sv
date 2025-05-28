import lynxTypes::*;
module perf_fpga_complicated_single_engine (
  input  logic                        aclk,
  input  logic                        aresetn,

  AXI4SR.s                            axis_in,
  AXI4SR.m                            axis_out,

  input  logic [1:0]                  req_type,
  input  logic [63:0]                 n_beats,

  output logic                        busy,
  output logic                        done
);

logic [1:0] req_type_reg;
logic [63:0] n_beats_reg;

always_comb begin
    axis_in.tready = n_beats_reg != 0 ? 1'b1 : 1'b0;

    axis_out.tdata = n_beats_reg;
    axis_out.tkeep = ~0;
    axis_out.tid   = 0;
    axis_out.tlast = 1'b0;
    axis_out.tvalid = n_beats_reg != 0 ? 1'b1 : 1'b0;
end

always_ff @(posedge aclk) begin
    if(aresetn == 1'b0) begin
        busy <= 1'b0;
        done <= 1'b0;

        req_type_reg <= 2'b0;
        n_beats_reg <= 64'b0;
    end 
    else begin
        busy <= busy;
        done <= 1'b0;

        // Start
        if (!busy && req_type != 0) begin
            busy <= 1'b1;
            req_type_reg <= req_type;
            n_beats_reg <= n_beats;
        end

        // Process
        if (req_type_reg == 2'b01) begin // Read
            // Process read request
            if (axis_in.tvalid && axis_in.tready) begin
                // Read data from input stream
                n_beats_reg <= n_beats_reg - 1;
            end
        end else begin // Write
            // Process write request
            if (axis_out.tvalid && axis_out.tready) begin
                // Write data to output stream
                n_beats_reg <= n_beats_reg - 1;
            end
        end

        // Finish
        if (n_beats_reg == 0) begin
            done <= 1'b1;
            busy <= 1'b0;

            req_type_reg <= 2'b0;
        end

    end
end

endmodule