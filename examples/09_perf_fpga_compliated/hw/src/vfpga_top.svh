// Simple pipeline stages, buffering the input/output signals (not really needed, but nice to have for easier timing closure)
AXI4SR axis_in_int [N_STRM_AXI-1:0] ();
AXI4SR axis_out_int [N_STRM_AXI-1:0] ();

genvar i;
generate
    for (i = 0; i < N_STRM_AXI; i++) begin : io_reg
        axisr_reg inst_reg_in  (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_host_recv[i]), .m_axis(axis_in_int[i]));
        axisr_reg inst_reg_out (.aclk(aclk), .aresetn(aresetn), .s_axis(axis_out_int[i]), .m_axis(axis_host_send[i]));
    end
endgenerate

///////////////////////////////////////
//          BENCH CONTROL           //
/////////////////////////////////////
// Submitting read/write request
logic [1:0] bench_req_ctrl;
// 00 - no request
// 01 - read request
// 10 - write request

logic req_accepted;

///////////////////////////////////////
//       BENCHMARK CONTROL          //
/////////////////////////////////////
// Reset benchmark control register
logic bench_reset;

// Total number of reads/writes to be issued; controlled by the user from software
logic [31:0] bench_n_reps;

// Number of completed reads/writes
logic [31:0] bench_done;

///////////////////////////////////////
//         DATA COMPLETION          //
/////////////////////////////////////
// Number of incoming/outgoing AXI data streams for each read/write request; 
// e.g. for a message of size 4096 bytes, the valid from the AXI stream goes high 8 times
// Set from the software to avoid computing divsion on the FPGA
logic [63:0] bench_req_n_beats;

///////////////////////////////////////
//        READ/WRITE DATA           //
/////////////////////////////////////
// Buffer size in bytes
logic [LEN_BITS-1:0] bench_req_len_A;
logic [LEN_BITS-1:0] bench_req_len_B;

// Virtual address of buffer to be read from / written to
logic [VADDR_BITS-1:0] bench_req_vaddr_A;
logic [VADDR_BITS-1:0] bench_req_vaddr_B;

// Coyote thread ID (obtained in software from coyote_thread.getCtid())
logic [PID_BITS-1:0] bench_req_pid;

///////////////////////////////////////
//           FSM & TIMING           //
/////////////////////////////////////
logic [2:0] state_C;
typedef enum logic[2:0]  {ST_IDLE, ST_RUNNING, ST_REQ_SUBMIT_A, ST_REQ_SUBMIT_B, ST_FINISH} state_t;

// Clock cycle counter
logic [63:0] bench_timer;

// The helper module parses the AXI interface to the target signals
perf_fpga_complicated_axi_ctrl_parser inst_axi_ctrl_parser (
    .aclk(aclk),
    .aresetn(aresetn),
    .axi_ctrl(axi_ctrl),
    .bench_req_ctrl(bench_req_ctrl),
    .bench_reset(bench_reset),
    .bench_n_reps(bench_n_reps),
    .bench_done(bench_done),
    .bench_req_n_beats(bench_req_n_beats),
    .bench_req_len_A(bench_req_len_A),
    .bench_req_len_B(bench_req_len_B),
    .bench_req_vaddr_A(bench_req_vaddr_A),
    .bench_req_vaddr_B(bench_req_vaddr_B),
    .bench_req_pid(bench_req_pid),
    .bench_timer(bench_timer),
    .req_accepted(req_accepted)
);

// Instances for each AXI stream engine
logic [N_STRM_AXI-1:0] engine_busy_array;
logic [N_STRM_AXI-1:0] engine_done_array;
logic [1:0] engine_req_type_array [N_STRM_AXI-1:0];
logic [63:0] engine_n_beats_array [N_STRM_AXI-1:0];
generate 
    for (i = 0; i < N_STRM_AXI; i++) begin : engine_gen
        perf_fpga_complicated_single_engine inst_single_engine (
            .aclk(aclk),
            .aresetn(aresetn),
            .axis_in(axis_in_int[i]),
            .axis_out(axis_out_int[i]),
            .req_type(engine_req_type_array[i]),
            .n_beats(engine_n_beats_array[i]),
            .busy(engine_busy_array[i]),
            .done(engine_done_array[i])
        );
    end
endgenerate

logic next_acceptable; // if any engine is ready to accept a new request
logic [2:0] next_engine; // the next engine id to submit a request to
logic engine_submitted; // if a request was submitted to the next engine
logic [3:0] done_number; // number of engines that have completed their requests
always_comb begin
    next_acceptable = engine_busy_array == ~0 ? 1'b0 : 1'b1;
    for (int j = 0; j < N_STRM_AXI; j++) begin
        if (engine_busy_array[j] == 1'b0) begin
            next_engine = j;
            break;
        end
    end

    for (int j = 0; j < N_STRM_AXI; j++) begin
        engine_req_type_array[j] = 2'b00;
        engine_n_beats_array[j] = 0;
    end
    if (next_acceptable && !engine_submitted) begin
        // If there is an engine that is not busy, we can submit a request
        if (bench_req_ctrl == 2'b01) begin // Read request
            engine_req_type_array[next_engine] = 2'b01;
            engine_n_beats_array[next_engine] = bench_req_n_beats;
        end else if (bench_req_ctrl == 2'b10) begin // Write request
            engine_req_type_array[next_engine] = 2'b10;
            engine_n_beats_array[next_engine] = bench_req_n_beats;
        end
    end

    req_accepted = 1'b0;
    if (state_C == ST_REQ_SUBMIT_B) begin
        if (bench_req_ctrl == 2'b01) begin // Read request
            req_accepted = sq_rd.ready && sq_rd.valid;
        end else if (bench_req_ctrl == 2'b10) begin // Write request
            req_accepted = sq_wr.ready && sq_wr.valid;
        end
    end

    done_number = 0;
    foreach (engine_done_array[j]) begin
        done_number += engine_done_array[j];
    end
end

// State-machine transition and counters
logic [2:0] next_engine_reg;
always_ff @(posedge aclk) begin
    if(aresetn == 1'b0) begin
        state_C <= ST_IDLE;

        bench_done <= 0;
        bench_timer <= 0;

        engine_submitted <= 0;
        next_engine_reg <= 0;
    end 
    else begin
        // Default values
        bench_done <= bench_done + done_number;
        bench_timer <= bench_timer + 1;
        engine_submitted <= engine_submitted;
        next_engine_reg <= next_engine_reg;
        // State machine logic
        case(state_C) 
            ST_IDLE: begin
                bench_done <= 0;
                bench_timer <= 0;
                if (next_acceptable && (bench_req_ctrl == 2'b01 || bench_req_ctrl == 2'b10)) begin
                    state_C <= ST_REQ_SUBMIT_A;
                    engine_submitted <= 1'b1;
                    next_engine_reg <= next_engine;
                end
            end
            ST_REQ_SUBMIT_A: begin
                if (bench_req_ctrl == 2'b01) begin // Read request
                    if (sq_rd.ready && sq_rd.valid) begin
                        state_C <= ST_REQ_SUBMIT_B;
                    end
                end else if (bench_req_ctrl == 2'b10) begin // Write request
                    if (sq_wr.ready && sq_wr.valid) begin
                        state_C <= ST_REQ_SUBMIT_B;
                    end
                end
            end
            ST_REQ_SUBMIT_B: begin
                if (bench_req_ctrl == 2'b01) begin // Read request
                    if (cq_rd.valid && cq_rd.ready) begin
                        state_C <= ST_RUNNING;
                        engine_submitted <= 1'b0; // reset the flag for the next request to the engine
                    end
                end else if (bench_req_ctrl == 2'b10) begin // Write request
                    if (cq_wr.valid && cq_wr.ready) begin
                        state_C <= ST_RUNNING;
                        engine_submitted <= 1'b0;
                    end
                end
            end
            ST_RUNNING: begin
                if (next_acceptable && (bench_req_ctrl == 2'b01 || bench_req_ctrl == 2'b10)) begin
                    // If there is an engine that is not busy, we can submit a request
                    state_C <= ST_REQ_SUBMIT_A;
                    engine_submitted <= 1'b1;
                    next_engine_reg <= next_engine;
                end 
                if (bench_done == bench_n_reps) begin
                    // If all engines are done, we can finish the benchmark
                    state_C <= ST_FINISH;
                end
            end
            ST_FINISH: begin
                bench_timer <= bench_timer;
                if (bench_reset == 1'b1) begin
                    bench_done <= 0;
                    state_C <= ST_IDLE;
                end
            end
        endcase
    end
end

always_comb begin
    ///////////////////////////////
    //          READS           //
    /////////////////////////////
    // Requests
    sq_rd.data = 0;
    sq_rd.data.last = 1'b1;
    sq_rd.data.pid = bench_req_pid;
    sq_rd.data.len = state_C == ST_REQ_SUBMIT_A ? bench_req_len_A : bench_req_len_B;
    sq_rd.data.vaddr = state_C == ST_REQ_SUBMIT_A ? bench_req_vaddr_A : bench_req_vaddr_B;
    sq_rd.data.strm = STRM_HOST;
    sq_rd.data.opcode = LOCAL_READ;
    sq_rd.data.dest = next_engine_reg;
    sq_rd.valid = bench_req_ctrl == 2'b01 && (state_C == ST_REQ_SUBMIT_A || state_C == ST_REQ_SUBMIT_B);

    cq_rd.ready = 1'b1;

    ///////////////////////////////
    //          WRITES          //
    /////////////////////////////
    // Requests
    sq_wr.data = 0;
    sq_wr.data.last = 1'b1;
    sq_wr.data.pid = bench_req_pid;
    sq_wr.data.len = state_C == ST_REQ_SUBMIT_A ? bench_req_len_A : bench_req_len_B;
    sq_wr.data.vaddr = state_C == ST_REQ_SUBMIT_A ? bench_req_vaddr_A : bench_req_vaddr_B;
    sq_wr.data.strm = STRM_HOST;
    sq_wr.data.opcode = LOCAL_WRITE;
    sq_wr.data.dest = next_engine_reg;
    sq_wr.valid = bench_req_ctrl == 2'b10 && (state_C == ST_REQ_SUBMIT_A || state_C == ST_REQ_SUBMIT_B);

    cq_wr.ready = 1'b1;

end

// Tie off unused interfaces
always_comb notify.tie_off_m();