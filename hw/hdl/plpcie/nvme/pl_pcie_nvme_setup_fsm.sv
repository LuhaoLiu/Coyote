`timescale 1ns/1ps

// -----------------------------------------------------------------------------
// nvme_setup_fsm
// -----------------------------------------------------------------------------
// Processor-free NVMe controller/queue initialization sequencer.
//
// This FSM is deliberately separate from pl_pcie_nvme_enum_fsm.  It starts when
// pl_pcie_nvme_enum_top asserts enum_done and assumes enumeration has already:
//
//   * assigned the endpoint BAR and enabled Memory Space + Bus Master,
//   * programmed the PG344 outbound BDF table, and
//   * assigned the Root Port's inbound DMA BAR.
//
// The fixed assumptions come from the measured controller registers:
// Some of them are not relevant to this module
//
//   CAP = 64'h1800_C030_1E02_3FFF
//     MQES   = 16'h3fff  (up to 16384 entries)
//     CQR    = 0         (queues need not be physically contiguous)
//     TO     = 8'h1e
//     DSTRD  = 0         (4-byte doorbell stride)
//     CSS.NVM= 1         (NVM command set supported)
//     MPSMIN = MPSMAX = 0 (4-KiB memory pages)
//
//   VS = 32'h0002_0000   (NVMe 2.0)
//
// Queue depth is a parameter with a default of 64.  At that depth an SQ uses
// exactly one 4-KiB page (64 entries * 64 bytes), while a CQ consumes 1 KiB
// (64 entries * 16 bytes) and is given its own 4-KiB page.
//
// Non-destructive discovery is mandatory.  Identify Controller, Active
// Namespace List, and Identify Namespace reuse one DMA page.  The final
// artifact deliberately contains no NVM read/write test datapath.
// -----------------------------------------------------------------------------
module pl_pcie_nvme_setup_fsm #(
    parameter integer MMIO_ADDR_WIDTH = 64,
    parameter integer RAM_ADDR_WIDTH  = 11,

    parameter logic [MMIO_ADDR_WIDTH-1:0] NVME_MMIO_AXI_BASE =
        64'h0000_0000_8000_0000,
    parameter logic [63:0] RP_DMA_PCIE_BASE =
        64'h0000_1000_0000_0000,
    parameter logic [63:0] RP_DMA_PCIE_SIZE =
        64'h0000_1000_0000_0000,

    parameter logic [63:0] KNOWN_CAP = 64'h1800_C030_1E02_3FFF,
    parameter logic [31:0] KNOWN_VS  = 32'h0002_0000,
    parameter integer QUEUE_DEPTH = 64,

    // The three Identify payloads are sequential and share one 4-KiB page.
    // Admin SQ and CQ have separate pages for an unambiguous 16-KiB layout.
    parameter logic [31:0] DISCOVERY_RAM_OFFSET  = 32'h0000_0000,
    parameter logic [31:0] ADMIN_SQ_RAM_OFFSET   = 32'h0000_2000,
    parameter logic [31:0] ADMIN_CQ_RAM_OFFSET   = 32'h0000_3000,
    parameter logic [63:0] DISCOVERY_PCIE_ADDR   =
        RP_DMA_PCIE_BASE + RP_DMA_PCIE_SIZE - 64'h4000,
    parameter logic [63:0] ADMIN_SQ_PCIE_ADDR    =
        RP_DMA_PCIE_BASE + RP_DMA_PCIE_SIZE - 64'h2000,
    parameter logic [63:0] ADMIN_CQ_PCIE_ADDR    =
        RP_DMA_PCIE_BASE + RP_DMA_PCIE_SIZE - 64'h1000,

    // I/O queues are owned by the surrounding system and are deliberately
    // not backed by this module's RAM.
    parameter logic [63:0] IO_SQ_PCIE_ADDR       =
        64'h0000_1FFF_F401_0000,
    parameter logic [63:0] IO_CQ_PCIE_ADDR       =
        64'h0000_1FFF_F402_0000,

    parameter integer READY_TIMEOUT_POLLS = 1_000_000,
    parameter integer CQ_TIMEOUT_POLLS    = 1_000_000,
    parameter logic [31:0] NVME_NSID      = 32'd1
) (
    input  wire                         clk,
    input  wire                         resetn,

    // Connect directly to pl_pcie_nvme_enum_top.enum_done.
    input  wire                         enum_done,


    // Abstract single-transaction endpoint-MMIO interface.  The standalone
    // wrapper connects this to pl_pcie_nvme_axi4_single_master_64.
    output wire                         mmio_cmd_valid,
    input  wire                         mmio_cmd_ready,
    output wire                         mmio_cmd_write,
    output wire [MMIO_ADDR_WIDTH-1:0]   mmio_cmd_addr,
    output wire [63:0]                  mmio_cmd_wdata,
    output wire [7:0]                   mmio_cmd_wstrb,
    output wire [2:0]                   mmio_cmd_size,

    input  wire                         mmio_rsp_valid,
    output wire                         mmio_rsp_ready,
    input  wire [63:0]                  mmio_rsp_rdata,
    input  wire [1:0]                   mmio_rsp_resp,
    input  wire                         mmio_rsp_timeout,

    // Native 64-bit RAM port.  ram_rdata is valid one clock after ram_en.
    output logic                        ram_en,
    output logic [7:0]                  ram_we,
    output logic [RAM_ADDR_WIDTH-1:0]   ram_addr,
    output logic [63:0]                 ram_wdata,
    input  wire [63:0]                  ram_rdata,

    // High-level status.
    output wire                         setup_started,
    output wire                         setup_busy,
    output wire                         queues_ready,
    output wire                         setup_done,
    output wire                         setup_error,

    // Mandatory, non-destructive discovery results.  These are populated by
    // parsing the Identify Controller, Active Namespace List, and Identify
    // Namespace DMA buffers before any I/O queue is created.
    output logic                        discovery_done,
    output logic                        discovery_valid,
    output logic                        controller_info_valid,
    output logic                        namespace_list_valid,
    output logic                        namespace_info_valid,
    output logic                        target_nsid_found,
    output logic [15:0]                 discovered_controller_vid,
    output logic [15:0]                 discovered_controller_ssvid,
    output logic [31:0]                 discovered_controller_version,
    output logic [31:0]                 discovered_controller_nn,
    output logic [7:0]                  discovered_controller_mdts,
    output logic [7:0]                  discovered_controller_sqes,
    output logic [7:0]                  discovered_controller_cqes,
    output logic [31:0]                 discovered_first_nsid,
    output logic [31:0]                 discovered_active_nsid_count,
    output logic [63:0]                 discovered_nsze,
    output logic [63:0]                 discovered_ncap,
    output logic [63:0]                 discovered_nuse,
    output logic [7:0]                  discovered_nlbaf,
    output logic [7:0]                  discovered_flbas,
    output logic [5:0]                  discovered_lba_format_index,
    output logic [7:0]                  discovered_lbads,
    output logic [15:0]                 discovered_metadata_bytes,
    output logic [31:0]                 discovered_lba_bytes,

    // Stable, compact ILA probes.
    output wire [7:0]                   state_dbg,
    output logic [7:0]                  error_code,
    output logic [7:0]                  last_opcode,
    output logic [15:0]                 last_cid,
    output logic [15:0]                 last_completion_cid,
    output logic [15:0]                 last_completion_status,
    output logic [127:0]                last_cqe,
    output logic [31:0]                 last_mmio_offset,
    output logic [63:0]                 last_mmio_rdata,
    output logic [31:0]                 submitted_command_count,
    output logic [15:0]                 admin_sq_tail_dbg,
    output logic [15:0]                 admin_cq_head_dbg,
    output logic                        admin_cq_phase_dbg,
    output logic [31:0]                 ready_poll_count_dbg,
    output logic [31:0]                 cq_poll_count_dbg,
    output logic [31:0]                 allocated_io_queues
);

    localparam logic [1:0] AXI_OKAY = 2'b00;

    // NVMe controller register offsets.
    localparam logic [31:0] NVME_CC      = 32'h0000_0014;
    localparam logic [31:0] NVME_CSTS    = 32'h0000_001c;
    localparam logic [31:0] NVME_AQA     = 32'h0000_0024;
    localparam logic [31:0] NVME_ASQ     = 32'h0000_0028;
    localparam logic [31:0] NVME_ACQ     = 32'h0000_0030;
    localparam logic [31:0] NVME_DB_BASE = 32'h0000_1000;

    // DSTRD == 0: each successive doorbell is four bytes apart.
    localparam logic [31:0] ADMIN_SQ_TAIL_DB = NVME_DB_BASE + 32'h0;
    localparam logic [31:0] ADMIN_CQ_HEAD_DB = NVME_DB_BASE + 32'h4;

    localparam logic [31:0] AQA_VALUE =
        ((QUEUE_DEPTH - 1) << 16) | (QUEUE_DEPTH - 1);

    // IOSQES=6 (64-byte SQE), IOCQES=4 (16-byte CQE), MPS=0, CSS=0,
    // AMS=0, and EN=1.
    localparam logic [31:0] CC_ENABLE_VALUE = 32'h0046_0001;

    localparam integer RAM_BYTES  = (1 << RAM_ADDR_WIDTH) * 8;
    localparam integer READY_LIMIT =
        (READY_TIMEOUT_POLLS < 1) ? 1 : READY_TIMEOUT_POLLS;
    localparam integer CQ_LIMIT =
        (CQ_TIMEOUT_POLLS < 1) ? 1 : CQ_TIMEOUT_POLLS;

    // Error codes are stable so an ILA capture can be decoded without an HDL
    // waveform database.
    localparam logic [7:0] ERR_NONE              = 8'h00;
    localparam logic [7:0] ERR_ENUM_LOST         = 8'h01;
    localparam logic [7:0] ERR_MMIO_AXI          = 8'h10;
    localparam logic [7:0] ERR_MMIO_TIMEOUT      = 8'h11;
    localparam logic [7:0] ERR_CSTS_CFS          = 8'h20;
    localparam logic [7:0] ERR_RDY0_TIMEOUT      = 8'h21;
    localparam logic [7:0] ERR_RDY1_TIMEOUT      = 8'h22;
    localparam logic [7:0] ERR_CQ_TIMEOUT        = 8'h30;
    localparam logic [7:0] ERR_CQ_CID            = 8'h31;
    localparam logic [7:0] ERR_CQ_SQID           = 8'h32;
    localparam logic [7:0] ERR_CQ_STATUS         = 8'h33;
    localparam logic [7:0] ERR_DISC_CTRL_INVALID = 8'h50;
    localparam logic [7:0] ERR_DISC_NSID_MISSING = 8'h51;
    localparam logic [7:0] ERR_DISC_NS_INVALID   = 8'h52;
    localparam logic [7:0] ERR_DISC_LBA_INVALID  = 8'h53;

    typedef enum logic [3:0] {
        CMD_NONE             = 4'h0,
        CMD_IDENTIFY_CTRL    = 4'h1,
        CMD_IDENTIFY_LIST    = 4'h2,
        CMD_IDENTIFY_NS      = 4'h3,
        CMD_SET_NUM_QUEUES   = 4'h4,
        CMD_CREATE_IO_CQ     = 4'h5,
        CMD_CREATE_IO_SQ     = 4'h6
    } command_t;

    // Explicit encodings reserve groups for controller setup and Admin queue
    // execution, making ILA traces easy to scan.
    typedef enum logic [7:0] {
        ST_IDLE                  = 8'h00,
        ST_CLEAR_RAM             = 8'h01,

        ST_DISABLE_CC            = 8'h10,
        ST_RDY0_READ             = 8'h11,
        ST_RDY0_CHECK            = 8'h12,
        ST_WRITE_AQA             = 8'h13,
        ST_WRITE_ASQ             = 8'h14,
        ST_WRITE_ACQ             = 8'h15,
        ST_ENABLE_CC             = 8'h16,
        ST_RDY1_READ             = 8'h17,
        ST_RDY1_CHECK            = 8'h18,

        ST_PREP_IDENTIFY_CTRL    = 8'h20,
        ST_PREP_IDENTIFY_LIST    = 8'h21,
        ST_PREP_IDENTIFY_NS      = 8'h22,
        ST_PREP_SET_NUM_QUEUES   = 8'h23,
        ST_PREP_CREATE_IO_CQ     = 8'h24,
        ST_PREP_CREATE_IO_SQ     = 8'h25,

        ST_SQE_WRITE             = 8'h40,
        ST_SQ_DOORBELL           = 8'h41,
        ST_CQ_POLL_START         = 8'h42,
        ST_CQ_READ0_REQ          = 8'h43,
        ST_CQ_READ0_WAIT         = 8'h44,
        ST_CQ_READ1_REQ          = 8'h45,
        ST_CQ_READ1_WAIT         = 8'h46,
        ST_CQ_CHECK              = 8'h47,
        ST_CQ_ACK                = 8'h48,
        ST_COMMAND_COMPLETE      = 8'h49,

        // Mandatory discovery parsing.  Keeping these states in a separate
        // range makes the non-destructive phase obvious in an ILA trace.
        ST_DISC_CTRL_W0_REQ       = 8'h80,
        ST_DISC_CTRL_W0_WAIT      = 8'h81,
        ST_DISC_CTRL_W9_REQ       = 8'h82,
        ST_DISC_CTRL_W9_WAIT      = 8'h83,
        ST_DISC_CTRL_W10_REQ      = 8'h84,
        ST_DISC_CTRL_W10_WAIT     = 8'h85,
        ST_DISC_CTRL_W64_REQ      = 8'h86,
        ST_DISC_CTRL_W64_WAIT     = 8'h87,
        ST_DISC_CTRL_CHECK        = 8'h88,
        ST_DISC_LIST_REQ          = 8'h89,
        ST_DISC_LIST_WAIT         = 8'h8a,
        ST_DISC_LIST_CHECK        = 8'h8b,
        ST_DISC_NS_W0_REQ         = 8'h90,
        ST_DISC_NS_W0_WAIT        = 8'h91,
        ST_DISC_NS_W1_REQ         = 8'h92,
        ST_DISC_NS_W1_WAIT        = 8'h93,
        ST_DISC_NS_W2_REQ         = 8'h94,
        ST_DISC_NS_W2_WAIT        = 8'h95,
        ST_DISC_NS_W3_REQ         = 8'h96,
        ST_DISC_NS_W3_WAIT        = 8'h97,
        ST_DISC_LBAF_REQ          = 8'h98,
        ST_DISC_LBAF_WAIT         = 8'h99,
        ST_DISC_NS_CHECK          = 8'h9a,

        ST_MMIO_REQ               = 8'hf0,
        ST_MMIO_RSP               = 8'hf1,
        ST_DONE                   = 8'hfe,
        ST_ERROR                  = 8'hff
    } state_t;

    (* MARK_DEBUG = "TRUE" *) state_t state;
    state_t mmio_return_state;

    command_t pending_command;
    logic [15:0] pending_cid;
    logic [15:0] next_cid;
    logic [2:0]  sqe_word_index;

    logic [15:0] admin_sq_tail;
    logic [15:0] admin_cq_head;
    logic        admin_cq_phase;

    logic [15:0] doorbell_value;
    logic [63:0] cqe_word0;
    logic [63:0] cqe_word1;

    logic [RAM_ADDR_WIDTH-1:0] clear_word_index;
    logic [31:0] ready_poll_count;
    logic [31:0] cq_poll_count;
    logic        started_latch;
    logic        queues_ready_latch;

    logic [8:0]  namespace_list_word_index;
    logic [63:0] namespace_list_read_data;
    logic [31:0] selected_lba_format;

    logic                        mmio_write_reg;
    logic [MMIO_ADDR_WIDTH-1:0]  mmio_addr_reg;
    logic [63:0]                 mmio_wdata_reg;
    logic [7:0]                  mmio_wstrb_reg;
    logic [2:0]                  mmio_size_reg;
    logic [31:0]                 mmio_read32_data;

    wire expected_cq_phase = admin_cq_phase;
    wire [15:0] expected_sqid = 16'd0;

    wire [31:0] namespace_list_nsid0 = namespace_list_read_data[31:0];
    wire [31:0] namespace_list_nsid1 = namespace_list_read_data[63:32];
    wire [1:0] namespace_list_valid_entries =
        {1'b0, (namespace_list_nsid0 != 32'd0)} +
        {1'b0, (namespace_list_nsid1 != 32'd0)};
    wire namespace_list_current_match =
        (namespace_list_nsid0 == NVME_NSID) ||
        (namespace_list_nsid1 == NVME_NSID);

    wire controller_queue_formats_valid =
        (discovered_controller_sqes[3:0] <= 4'd6) &&
        (discovered_controller_sqes[7:4] >= 4'd6) &&
        (discovered_controller_cqes[3:0] <= 4'd4) &&
        (discovered_controller_cqes[7:4] >= 4'd4);

    wire selected_lba_format_index_valid =
        ({2'd0, discovered_lba_format_index} <= discovered_nlbaf);
    wire selected_lbads_valid =
        (selected_lba_format[23:16] >= 8'd9) &&
        (selected_lba_format[23:16] < 8'd32);
    wire [31:0] selected_lba_bytes_value = selected_lbads_valid ?
        (32'd1 << selected_lba_format[23:16]) : 32'd0;
    wire namespace_capacity_valid =
        (discovered_nsze != 64'd0) &&
        (discovered_ncap != 64'd0) &&
        (discovered_ncap <= discovered_nsze) &&
        (discovered_nuse <= discovered_ncap);
    assign mmio_cmd_valid = (state == ST_MMIO_REQ);
    assign mmio_cmd_write = mmio_write_reg;
    assign mmio_cmd_addr  = mmio_addr_reg;
    assign mmio_cmd_wdata = mmio_wdata_reg;
    assign mmio_cmd_wstrb = mmio_wstrb_reg;
    assign mmio_cmd_size  = mmio_size_reg;
    assign mmio_rsp_ready = (state == ST_MMIO_RSP);

    assign setup_started       = started_latch;
    assign setup_busy          = (state != ST_IDLE) &&
                                 (state != ST_DONE) &&
                                 (state != ST_ERROR);
    assign queues_ready        = queues_ready_latch;
    assign setup_done          = (state == ST_DONE);
    assign setup_error         = (state == ST_ERROR);
    assign state_dbg           = state;

    always_comb begin
        admin_sq_tail_dbg   = admin_sq_tail;
        admin_cq_head_dbg   = admin_cq_head;
        admin_cq_phase_dbg  = admin_cq_phase;
        ready_poll_count_dbg= ready_poll_count;
        cq_poll_count_dbg   = cq_poll_count;
    end

    function automatic logic [7:0] command_opcode(input command_t command);
        begin
            case (command)
                CMD_IDENTIFY_CTRL,
                CMD_IDENTIFY_LIST,
                CMD_IDENTIFY_NS:    command_opcode = 8'h06;
                CMD_SET_NUM_QUEUES: command_opcode = 8'h09;
                CMD_CREATE_IO_CQ:   command_opcode = 8'h05;
                CMD_CREATE_IO_SQ:   command_opcode = 8'h01;
                default:            command_opcode = 8'h00;
            endcase
        end
    endfunction

    function automatic logic [63:0] command_prp1(input command_t command);
        begin
            case (command)
                CMD_IDENTIFY_CTRL,
                CMD_IDENTIFY_LIST,
                CMD_IDENTIFY_NS:
                    command_prp1 = DISCOVERY_PCIE_ADDR;
                CMD_CREATE_IO_CQ:
                    command_prp1 = IO_CQ_PCIE_ADDR;
                CMD_CREATE_IO_SQ:
                    command_prp1 = IO_SQ_PCIE_ADDR;
                default:
                    command_prp1 = 64'd0;
            endcase
        end
    endfunction

    // Construct one 64-bit word of a 64-byte NVMe SQ entry.  The explicit
    // DWord packing mirrors the NVMe command diagrams and is convenient for
    // inspecting the queue through ILA/debug RAM reads.
    function automatic logic [63:0] sqe_word(
        input command_t command,
        input logic [15:0] cid,
        input logic [2:0] word_number
    );
        logic [31:0] dw0;
        logic [31:0] dw1;
        logic [31:0] dw10;
        logic [31:0] dw11;
        logic [31:0] dw12;
        logic [63:0] prp1;
        begin
            dw0  = {cid, 8'h00, command_opcode(command)};
            dw1  = (command == CMD_IDENTIFY_NS) ? NVME_NSID : 32'd0;
            dw10 = 32'd0;
            dw11 = 32'd0;
            dw12 = 32'd0;
            prp1 = command_prp1(command);

            case (command)
                CMD_IDENTIFY_CTRL: begin
                    dw10 = 32'd1; // CNS=01h: Identify Controller
                end
                CMD_IDENTIFY_LIST: begin
                    dw10 = 32'd2; // CNS=02h: Active Namespace ID list
                end
                CMD_IDENTIFY_NS: begin
                    dw10 = 32'd0; // CNS=00h: Identify Namespace
                end
                CMD_SET_NUM_QUEUES: begin
                    dw10 = 32'd7; // FID=07h: Number of Queues
                    dw11 = 32'd0; // Request one SQ and one CQ (zero based)
                end
                CMD_CREATE_IO_CQ: begin
                    dw10 = ((QUEUE_DEPTH - 1) << 16) | 16'd1;
                    dw11 = 32'h0000_0001; // PC=1, IEN=0, IV=0
                end
                CMD_CREATE_IO_SQ: begin
                    dw10 = ((QUEUE_DEPTH - 1) << 16) | 16'd1;
                    dw11 = 32'h0001_0001; // CQID=1, QPRIO=0, PC=1
                end
                default: begin
                    dw10 = 32'd0;
                    dw11 = 32'd0;
                    dw12 = 32'd0;
                end
            endcase

            case (word_number)
                3'd0: sqe_word = {dw1, dw0};
                3'd1: sqe_word = 64'd0;       // DW2..DW3
                3'd2: sqe_word = 64'd0;       // MPTR, DW4..DW5
                3'd3: sqe_word = prp1;        // PRP1, DW6..DW7
                3'd4: sqe_word = 64'd0;       // PRP2, DW8..DW9
                3'd5: sqe_word = {dw11, dw10};
                3'd6: sqe_word = {32'd0, dw12};
                default: sqe_word = 64'd0;    // DW14..DW15
            endcase
        end
    endfunction

    // These tasks only load the generic MMIO engine's registers.  All command
    // and response handshakes still occur in ST_MMIO_REQ/ST_MMIO_RSP.
    task automatic launch_write32(
        input logic [MMIO_ADDR_WIDTH-1:0] address,
        input logic [31:0] data,
        input state_t return_state
    );
        integer lane_shift;
        begin
            lane_shift       = address[2:0] * 8;
            mmio_write_reg   <= 1'b1;
            mmio_addr_reg    <= address;
            mmio_wdata_reg   <= ({32'd0, data} << lane_shift);
            mmio_wstrb_reg   <= (8'h0f << address[2:0]);
            mmio_size_reg    <= 3'd2;
            mmio_return_state<= return_state;
            state            <= ST_MMIO_REQ;
        end
    endtask

    task automatic launch_write64(
        input logic [MMIO_ADDR_WIDTH-1:0] address,
        input logic [63:0] data,
        input state_t return_state
    );
        begin
            mmio_write_reg   <= 1'b1;
            mmio_addr_reg    <= address;
            mmio_wdata_reg   <= data;
            mmio_wstrb_reg   <= 8'hff;
            mmio_size_reg    <= 3'd3;
            mmio_return_state<= return_state;
            state            <= ST_MMIO_REQ;
        end
    endtask

    task automatic launch_read32(
        input logic [MMIO_ADDR_WIDTH-1:0] address,
        input state_t return_state
    );
        begin
            mmio_write_reg   <= 1'b0;
            mmio_addr_reg    <= address;
            mmio_wdata_reg   <= 64'd0;
            mmio_wstrb_reg   <= 8'h00;
            mmio_size_reg    <= 3'd2;
            mmio_return_state<= return_state;
            state            <= ST_MMIO_REQ;
        end
    endtask

    task automatic prepare_command(input command_t command);
        begin
            pending_command <= command;
            pending_cid     <= next_cid;
            last_cid        <= next_cid;
            last_opcode     <= command_opcode(command);
            next_cid        <= next_cid + 1'b1;
            sqe_word_index  <= 3'd0;
            state           <= ST_SQE_WRITE;
        end
    endtask

    // Local setup-FSM access to TDP-RAM port B.
    always_comb begin
        ram_en    = 1'b0;
        ram_we    = 8'h00;
        ram_addr  = '0;
        ram_wdata = 64'd0;

        case (state)
            ST_CLEAR_RAM: begin
                ram_en    = 1'b1;
                ram_we    = 8'hff;
                ram_addr  = clear_word_index;
                ram_wdata = 64'd0;
            end

            ST_SQE_WRITE: begin
                ram_en   = 1'b1;
                ram_we   = 8'hff;
                ram_addr = (ADMIN_SQ_RAM_OFFSET >> 3) +
                           (admin_sq_tail * 8) + sqe_word_index;
                ram_wdata = sqe_word(
                    pending_command, pending_cid, sqe_word_index
                );
            end

            ST_CQ_READ0_REQ: begin
                ram_en   = 1'b1;
                ram_we   = 8'h00;
                ram_addr = (ADMIN_CQ_RAM_OFFSET >> 3) +
                           (admin_cq_head * 2);
            end

            ST_CQ_READ1_REQ: begin
                ram_en   = 1'b1;
                ram_we   = 8'h00;
                ram_addr = (ADMIN_CQ_RAM_OFFSET >> 3) +
                           (admin_cq_head * 2) + 1;
            end

            ST_DISC_CTRL_W0_REQ: begin
                ram_en   = 1'b1;
                ram_we   = 8'h00;
                ram_addr = (DISCOVERY_RAM_OFFSET >> 3);
            end

            ST_DISC_CTRL_W9_REQ: begin
                ram_en   = 1'b1;
                ram_we   = 8'h00;
                ram_addr = (DISCOVERY_RAM_OFFSET >> 3) + 9;
            end

            ST_DISC_CTRL_W10_REQ: begin
                ram_en   = 1'b1;
                ram_we   = 8'h00;
                ram_addr = (DISCOVERY_RAM_OFFSET >> 3) + 10;
            end

            ST_DISC_CTRL_W64_REQ: begin
                ram_en   = 1'b1;
                ram_we   = 8'h00;
                ram_addr = (DISCOVERY_RAM_OFFSET >> 3) + 64;
            end

            ST_DISC_LIST_REQ: begin
                ram_en   = 1'b1;
                ram_we   = 8'h00;
                ram_addr = (DISCOVERY_RAM_OFFSET >> 3) +
                           namespace_list_word_index;
            end

            ST_DISC_NS_W0_REQ: begin
                ram_en   = 1'b1;
                ram_we   = 8'h00;
                ram_addr = (DISCOVERY_RAM_OFFSET >> 3);
            end

            ST_DISC_NS_W1_REQ: begin
                ram_en   = 1'b1;
                ram_we   = 8'h00;
                ram_addr = (DISCOVERY_RAM_OFFSET >> 3) + 1;
            end

            ST_DISC_NS_W2_REQ: begin
                ram_en   = 1'b1;
                ram_we   = 8'h00;
                ram_addr = (DISCOVERY_RAM_OFFSET >> 3) + 2;
            end

            ST_DISC_NS_W3_REQ: begin
                ram_en   = 1'b1;
                ram_we   = 8'h00;
                ram_addr = (DISCOVERY_RAM_OFFSET >> 3) + 3;
            end

            ST_DISC_LBAF_REQ: begin
                ram_en   = 1'b1;
                ram_we   = 8'h00;
                // LBA format descriptors start at byte 128 and are four
                // bytes each; two descriptors therefore share one RAM word.
                ram_addr = (DISCOVERY_RAM_OFFSET >> 3) + 16 +
                           (discovered_lba_format_index >> 1);
            end

            default: begin
                ram_en    = 1'b0;
                ram_we    = 8'h00;
                ram_addr  = '0;
                ram_wdata = 64'd0;
            end
        endcase
    end

    always_ff @(posedge clk) begin
        if (!resetn) begin
            state                    <= ST_IDLE;
            mmio_return_state        <= ST_IDLE;
            pending_command          <= CMD_NONE;
            pending_cid              <= 16'd0;
            next_cid                 <= 16'd1;
            sqe_word_index           <= 3'd0;
            admin_sq_tail            <= 16'd0;
            admin_cq_head            <= 16'd0;
            admin_cq_phase           <= 1'b1;
            doorbell_value           <= 16'd0;
            cqe_word0                <= 64'd0;
            cqe_word1                <= 64'd0;
            clear_word_index         <= '0;
            ready_poll_count         <= 32'd0;
            cq_poll_count            <= 32'd0;
            started_latch            <= 1'b0;
            queues_ready_latch       <= 1'b0;
            namespace_list_word_index<= 9'd0;
            namespace_list_read_data <= 64'd0;
            selected_lba_format      <= 32'd0;
            mmio_write_reg           <= 1'b0;
            mmio_addr_reg            <= '0;
            mmio_wdata_reg           <= 64'd0;
            mmio_wstrb_reg           <= 8'h00;
            mmio_size_reg            <= 3'd2;
            mmio_read32_data         <= 32'd0;
            discovery_done           <= 1'b0;
            discovery_valid          <= 1'b0;
            controller_info_valid    <= 1'b0;
            namespace_list_valid     <= 1'b0;
            namespace_info_valid     <= 1'b0;
            target_nsid_found        <= 1'b0;
            discovered_controller_vid<= 16'd0;
            discovered_controller_ssvid <= 16'd0;
            discovered_controller_version <= 32'd0;
            discovered_controller_nn <= 32'd0;
            discovered_controller_mdts <= 8'd0;
            discovered_controller_sqes <= 8'd0;
            discovered_controller_cqes <= 8'd0;
            discovered_first_nsid    <= 32'd0;
            discovered_active_nsid_count <= 32'd0;
            discovered_nsze          <= 64'd0;
            discovered_ncap          <= 64'd0;
            discovered_nuse          <= 64'd0;
            discovered_nlbaf         <= 8'd0;
            discovered_flbas         <= 8'd0;
            discovered_lba_format_index <= 6'd0;
            discovered_lbads         <= 8'd0;
            discovered_metadata_bytes<= 16'd0;
            discovered_lba_bytes     <= 32'd0;
            error_code               <= ERR_NONE;
            last_opcode              <= 8'h00;
            last_cid                 <= 16'd0;
            last_completion_cid      <= 16'd0;
            last_completion_status   <= 16'd0;
            last_cqe                 <= 128'd0;
            last_mmio_offset         <= 32'd0;
            last_mmio_rdata          <= 64'd0;
            submitted_command_count  <= 32'd0;
            allocated_io_queues      <= 32'd0;
        end else begin
            // enum_done is latched by the enumeration FSM.  If it disappears
            // while setup is active, continuing to touch the endpoint is not
            // safe because BAR/BDF state may have been reset.
            if (started_latch &&
                (state != ST_IDLE) &&
                (state != ST_DONE) &&
                (state != ST_ERROR) &&
                !enum_done) begin
                error_code <= ERR_ENUM_LOST;
                state      <= ST_ERROR;
            end else begin
                case (state)
                    ST_IDLE: begin
                        if (enum_done) begin
                            started_latch           <= 1'b1;
                            queues_ready_latch      <= 1'b0;
                            discovery_done          <= 1'b0;
                            discovery_valid         <= 1'b0;
                            controller_info_valid   <= 1'b0;
                            namespace_list_valid    <= 1'b0;
                            namespace_info_valid    <= 1'b0;
                            target_nsid_found       <= 1'b0;
                            discovered_first_nsid   <= 32'd0;
                            discovered_active_nsid_count <= 32'd0;
                            error_code              <= ERR_NONE;
                            submitted_command_count <= 32'd0;
                            clear_word_index        <= '0;
                            state                   <= ST_DISABLE_CC;
                        end
                    end

                    ST_CLEAR_RAM: begin
                        if (&clear_word_index) begin
                            clear_word_index <= '0;
                            state <= ST_WRITE_AQA;
                        end else begin
                            clear_word_index <= clear_word_index + 1'b1;
                        end
                    end

                    // Always request CC.EN=0 first.  This safely recovers from
                    // a prior partially initialized controller, then waits for
                    // CSTS.RDY to deassert before replacing queue registers.
                    ST_DISABLE_CC: begin
                        ready_poll_count <= 32'd0;
                        launch_write32(
                            NVME_MMIO_AXI_BASE + NVME_CC,
                            32'd0,
                            ST_RDY0_READ
                        );
                    end

                    ST_RDY0_READ: begin
                        launch_read32(
                            NVME_MMIO_AXI_BASE + NVME_CSTS,
                            ST_RDY0_CHECK
                        );
                    end

                    ST_RDY0_CHECK: begin
                        if (mmio_read32_data[1]) begin
                            error_code <= ERR_CSTS_CFS;
                            state      <= ST_ERROR;
                        end else if (!mmio_read32_data[0]) begin
                            ready_poll_count <= 32'd0;
                            clear_word_index <= '0;
                            state <= ST_CLEAR_RAM;
                        end else if (ready_poll_count >= (READY_LIMIT - 1)) begin
                            error_code <= ERR_RDY0_TIMEOUT;
                            state      <= ST_ERROR;
                        end else begin
                            ready_poll_count <= ready_poll_count + 1'b1;
                            state <= ST_RDY0_READ;
                        end
                    end

                    ST_WRITE_AQA: begin
                        launch_write32(
                            NVME_MMIO_AXI_BASE + NVME_AQA,
                            AQA_VALUE,
                            ST_WRITE_ASQ
                        );
                    end

                    ST_WRITE_ASQ: begin
                        launch_write64(
                            NVME_MMIO_AXI_BASE + NVME_ASQ,
                            ADMIN_SQ_PCIE_ADDR,
                            ST_WRITE_ACQ
                        );
                    end

                    ST_WRITE_ACQ: begin
                        launch_write64(
                            NVME_MMIO_AXI_BASE + NVME_ACQ,
                            ADMIN_CQ_PCIE_ADDR,
                            ST_ENABLE_CC
                        );
                    end

                    ST_ENABLE_CC: begin
                        launch_write32(
                            NVME_MMIO_AXI_BASE + NVME_CC,
                            CC_ENABLE_VALUE,
                            ST_RDY1_READ
                        );
                    end

                    ST_RDY1_READ: begin
                        launch_read32(
                            NVME_MMIO_AXI_BASE + NVME_CSTS,
                            ST_RDY1_CHECK
                        );
                    end

                    ST_RDY1_CHECK: begin
                        if (mmio_read32_data[1]) begin
                            error_code <= ERR_CSTS_CFS;
                            state      <= ST_ERROR;
                        end else if (mmio_read32_data[0]) begin
                            ready_poll_count <= 32'd0;
                            state <= ST_PREP_IDENTIFY_CTRL;
                        end else if (ready_poll_count >= (READY_LIMIT - 1)) begin
                            error_code <= ERR_RDY1_TIMEOUT;
                            state      <= ST_ERROR;
                        end else begin
                            ready_poll_count <= ready_poll_count + 1'b1;
                            state <= ST_RDY1_READ;
                        end
                    end

                    // ---------------------------------------------------------
                    // Mandatory non-destructive discovery parsing.
                    //
                    // Identify commands transfer complete 4-KiB data
                    // structures into RAM.  Only the fields required for a
                    // safe first I/O are read back through the native port;
                    // the entire pages remain available through debug RAM.
                    // ---------------------------------------------------------
                    ST_DISC_CTRL_W0_REQ:
                        state <= ST_DISC_CTRL_W0_WAIT;

                    ST_DISC_CTRL_W0_WAIT: begin
                        discovered_controller_vid   <= ram_rdata[15:0];
                        discovered_controller_ssvid <= ram_rdata[31:16];
                        state <= ST_DISC_CTRL_W9_REQ;
                    end

                    ST_DISC_CTRL_W9_REQ:
                        state <= ST_DISC_CTRL_W9_WAIT;

                    ST_DISC_CTRL_W9_WAIT: begin
                        // Identify Controller byte 77 is MDTS.
                        discovered_controller_mdts <= ram_rdata[47:40];
                        state <= ST_DISC_CTRL_W10_REQ;
                    end

                    ST_DISC_CTRL_W10_REQ:
                        state <= ST_DISC_CTRL_W10_WAIT;

                    ST_DISC_CTRL_W10_WAIT: begin
                        // Identify Controller bytes 80:83 mirror VS.
                        discovered_controller_version <= ram_rdata[31:0];
                        state <= ST_DISC_CTRL_W64_REQ;
                    end

                    ST_DISC_CTRL_W64_REQ:
                        state <= ST_DISC_CTRL_W64_WAIT;

                    ST_DISC_CTRL_W64_WAIT: begin
                        // Bytes 512, 513, and 516:519 are SQES, CQES, and NN.
                        discovered_controller_sqes <= ram_rdata[7:0];
                        discovered_controller_cqes <= ram_rdata[15:8];
                        discovered_controller_nn   <= ram_rdata[63:32];
                        state <= ST_DISC_CTRL_CHECK;
                    end

                    ST_DISC_CTRL_CHECK: begin
                        if ((discovered_controller_vid == 16'd0) ||
                            (discovered_controller_vid == 16'hffff) ||
                            (discovered_controller_version == 32'd0) ||
                            (discovered_controller_nn == 32'd0) ||
                            !controller_queue_formats_valid) begin
                            discovery_done <= 1'b1;
                            error_code <= ERR_DISC_CTRL_INVALID;
                            state <= ST_ERROR;
                        end else begin
                            controller_info_valid <= 1'b1;
                            state <= ST_PREP_IDENTIFY_LIST;
                        end
                    end

                    ST_DISC_LIST_REQ:
                        state <= ST_DISC_LIST_WAIT;

                    ST_DISC_LIST_WAIT: begin
                        namespace_list_read_data <= ram_rdata;
                        state <= ST_DISC_LIST_CHECK;
                    end

                    ST_DISC_LIST_CHECK: begin
                        if ((namespace_list_word_index == 9'd0) &&
                            (namespace_list_nsid0 != 32'd0))
                            discovered_first_nsid <= namespace_list_nsid0;

                        discovered_active_nsid_count <=
                            discovered_active_nsid_count +
                            namespace_list_valid_entries;

                        if (namespace_list_current_match)
                            target_nsid_found <= 1'b1;

                        // The list is packed with zero termination and holds at
                        // most 1,024 NSIDs (512 native 64-bit RAM words).
                        if ((namespace_list_nsid0 == 32'd0) ||
                            (namespace_list_nsid1 == 32'd0) ||
                            (namespace_list_word_index == 9'd511)) begin
                            if (!(target_nsid_found ||
                                  namespace_list_current_match)) begin
                                discovery_done <= 1'b1;
                                error_code <= ERR_DISC_NSID_MISSING;
                                state <= ST_ERROR;
                            end else begin
                                namespace_list_valid <= 1'b1;
                                state <= ST_PREP_IDENTIFY_NS;
                            end
                        end else begin
                            namespace_list_word_index <=
                                namespace_list_word_index + 1'b1;
                            state <= ST_DISC_LIST_REQ;
                        end
                    end

                    ST_DISC_NS_W0_REQ:
                        state <= ST_DISC_NS_W0_WAIT;

                    ST_DISC_NS_W0_WAIT: begin
                        discovered_nsze <= ram_rdata;
                        state <= ST_DISC_NS_W1_REQ;
                    end

                    ST_DISC_NS_W1_REQ:
                        state <= ST_DISC_NS_W1_WAIT;

                    ST_DISC_NS_W1_WAIT: begin
                        discovered_ncap <= ram_rdata;
                        state <= ST_DISC_NS_W2_REQ;
                    end

                    ST_DISC_NS_W2_REQ:
                        state <= ST_DISC_NS_W2_WAIT;

                    ST_DISC_NS_W2_WAIT: begin
                        discovered_nuse <= ram_rdata;
                        state <= ST_DISC_NS_W3_REQ;
                    end

                    ST_DISC_NS_W3_REQ:
                        state <= ST_DISC_NS_W3_WAIT;

                    ST_DISC_NS_W3_WAIT: begin
                        // Bytes 25 and 26 are NLBAF and FLBAS.  NVMe 2.x may
                        // extend the format index with FLBAS bits 6:5.
                        discovered_nlbaf <= ram_rdata[15:8];
                        discovered_flbas <= ram_rdata[23:16];
                        discovered_lba_format_index <=
                            {ram_rdata[22:21], ram_rdata[19:16]};
                        state <= ST_DISC_LBAF_REQ;
                    end

                    ST_DISC_LBAF_REQ:
                        state <= ST_DISC_LBAF_WAIT;

                    ST_DISC_LBAF_WAIT: begin
                        if (discovered_lba_format_index[0])
                            selected_lba_format <= ram_rdata[63:32];
                        else
                            selected_lba_format <= ram_rdata[31:0];
                        state <= ST_DISC_NS_CHECK;
                    end

                    ST_DISC_NS_CHECK: begin
                        discovered_metadata_bytes <= selected_lba_format[15:0];
                        discovered_lbads <= selected_lba_format[23:16];
                        discovered_lba_bytes <= selected_lba_bytes_value;
                        discovery_done <= 1'b1;

                        if (!namespace_capacity_valid) begin
                            error_code <= ERR_DISC_NS_INVALID;
                            state <= ST_ERROR;
                        end else if (!selected_lba_format_index_valid ||
                                     !selected_lbads_valid) begin
                            error_code <= ERR_DISC_LBA_INVALID;
                            state <= ST_ERROR;
                        end else begin
                            namespace_info_valid <= 1'b1;
                            discovery_valid      <= 1'b1;
                            state <= ST_PREP_SET_NUM_QUEUES;
                        end
                    end

                    ST_PREP_IDENTIFY_CTRL:
                        prepare_command(CMD_IDENTIFY_CTRL);

                    ST_PREP_IDENTIFY_LIST:
                        prepare_command(CMD_IDENTIFY_LIST);

                    ST_PREP_IDENTIFY_NS:
                        prepare_command(CMD_IDENTIFY_NS);

                    ST_PREP_SET_NUM_QUEUES:
                        prepare_command(CMD_SET_NUM_QUEUES);

                    ST_PREP_CREATE_IO_CQ:
                        prepare_command(CMD_CREATE_IO_CQ);

                    ST_PREP_CREATE_IO_SQ:
                        prepare_command(CMD_CREATE_IO_SQ);

                    ST_SQE_WRITE: begin
                        if (sqe_word_index == 3'd7) begin
                            sqe_word_index <= 3'd0;
                            if (admin_sq_tail == (QUEUE_DEPTH - 1)) begin
                                doorbell_value <= 16'd0;
                                admin_sq_tail  <= 16'd0;
                            end else begin
                                doorbell_value <= admin_sq_tail + 1'b1;
                                admin_sq_tail  <= admin_sq_tail + 1'b1;
                            end
                            state <= ST_SQ_DOORBELL;
                        end else begin
                            sqe_word_index <= sqe_word_index + 1'b1;
                        end
                    end

                    ST_SQ_DOORBELL: begin
                        launch_write32(
                            NVME_MMIO_AXI_BASE + ADMIN_SQ_TAIL_DB,
                            {16'd0, doorbell_value},
                            ST_CQ_POLL_START
                        );
                    end

                    ST_CQ_POLL_START: begin
                        cq_poll_count <= 32'd0;
                        submitted_command_count <=
                            submitted_command_count + 1'b1;
                        state <= ST_CQ_READ0_REQ;
                    end

                    ST_CQ_READ0_REQ:
                        state <= ST_CQ_READ0_WAIT;

                    ST_CQ_READ0_WAIT: begin
                        cqe_word0 <= ram_rdata;
                        state <= ST_CQ_READ1_REQ;
                    end

                    ST_CQ_READ1_REQ:
                        state <= ST_CQ_READ1_WAIT;

                    ST_CQ_READ1_WAIT: begin
                        cqe_word1 <= ram_rdata;
                        state <= ST_CQ_CHECK;
                    end

                    ST_CQ_CHECK: begin
                        last_cqe <= {cqe_word1, cqe_word0};

                        // P is bit 16 of CQE DW3, hence bit 48 of the upper
                        // 64-bit RAM word.  A mismatched phase means empty.
                        if (cqe_word1[48] != expected_cq_phase) begin
                            if (cq_poll_count >= (CQ_LIMIT - 1)) begin
                                error_code <= ERR_CQ_TIMEOUT;
                                state      <= ST_ERROR;
                            end else begin
                                cq_poll_count <= cq_poll_count + 1'b1;
                                state <= ST_CQ_READ0_REQ;
                            end
                        end else if (cqe_word1[47:32] != pending_cid) begin
                            last_completion_cid <= cqe_word1[47:32];
                            error_code <= ERR_CQ_CID;
                            state      <= ST_ERROR;
                        end else if (cqe_word1[31:16] != expected_sqid) begin
                            error_code <= ERR_CQ_SQID;
                            state      <= ST_ERROR;
                        end else if (cqe_word1[63:49] != 15'd0) begin
                            last_completion_status <= {1'b0, cqe_word1[63:49]};
                            error_code <= ERR_CQ_STATUS;
                            state      <= ST_ERROR;
                        end else begin
                            last_completion_cid    <= cqe_word1[47:32];
                            last_completion_status <= 16'd0;
                            state <= ST_CQ_ACK;
                        end
                    end

                    ST_CQ_ACK: begin
                        if (admin_cq_head == (QUEUE_DEPTH - 1)) begin
                            admin_cq_head      <= 16'd0;
                            admin_cq_phase     <= ~admin_cq_phase;
                            launch_write32(
                                NVME_MMIO_AXI_BASE + ADMIN_CQ_HEAD_DB,
                                32'd0,
                                ST_COMMAND_COMPLETE
                            );
                        end else begin
                            admin_cq_head      <= admin_cq_head + 1'b1;
                            launch_write32(
                                NVME_MMIO_AXI_BASE + ADMIN_CQ_HEAD_DB,
                                {16'd0, (admin_cq_head + 1'b1)},
                                ST_COMMAND_COMPLETE
                            );
                        end
                    end

                    ST_COMMAND_COMPLETE: begin
                        case (pending_command)
                            CMD_IDENTIFY_CTRL:
                                state <= ST_DISC_CTRL_W0_REQ;
                            CMD_IDENTIFY_LIST: begin
                                namespace_list_word_index    <= 9'd0;
                                namespace_list_read_data     <= 64'd0;
                                discovered_first_nsid        <= 32'd0;
                                discovered_active_nsid_count <= 32'd0;
                                target_nsid_found            <= 1'b0;
                                state <= ST_DISC_LIST_REQ;
                            end
                            CMD_IDENTIFY_NS:
                                state <= ST_DISC_NS_W0_REQ;
                            CMD_SET_NUM_QUEUES: begin
                                allocated_io_queues <= cqe_word0[31:0];
                                state <= ST_PREP_CREATE_IO_CQ;
                            end
                            CMD_CREATE_IO_CQ:
                                state <= ST_PREP_CREATE_IO_SQ;
                            CMD_CREATE_IO_SQ: begin
                                queues_ready_latch <= 1'b1;
                                state <= ST_DONE;
                            end
                            default: begin
                                error_code <= ERR_CQ_STATUS;
                                state <= ST_ERROR;
                            end
                        endcase
                    end

                    ST_MMIO_REQ: begin
                        if (mmio_cmd_valid && mmio_cmd_ready) begin
                            last_mmio_offset <=
                                mmio_addr_reg - NVME_MMIO_AXI_BASE;
                            state <= ST_MMIO_RSP;
                        end
                    end

                    ST_MMIO_RSP: begin
                        if (mmio_rsp_valid) begin
                            last_mmio_rdata <= mmio_rsp_rdata;
                            if (mmio_rsp_timeout) begin
                                error_code <= ERR_MMIO_TIMEOUT;
                                state      <= ST_ERROR;
                            end else if (mmio_rsp_resp != AXI_OKAY) begin
                                error_code <= ERR_MMIO_AXI;
                                state      <= ST_ERROR;
                            end else begin
                                mmio_read32_data <=
                                    mmio_rsp_rdata >>
                                    (mmio_addr_reg[2:0] * 8);
                                state <= mmio_return_state;
                            end
                        end
                    end

                    ST_DONE: begin
                        // Latched until reset for reliable ILA use.
                    end

                    ST_ERROR: begin
                        // Latched until reset; no automatic retry.
                    end

                    default: begin
                        error_code <= ERR_CQ_STATUS;
                        state <= ST_ERROR;
                    end
                endcase
            end
        end
    end

    initial begin
        if ((QUEUE_DEPTH < 2) || (QUEUE_DEPTH > 4096) ||
            ((QUEUE_DEPTH & (QUEUE_DEPTH - 1)) != 0))
            $error("pl_pcie_nvme_setup_fsm: QUEUE_DEPTH must be a power of two in [2,4096]");
        if ((KNOWN_CAP[15:0] + 1) < QUEUE_DEPTH)
            $error("pl_pcie_nvme_setup_fsm: QUEUE_DEPTH exceeds known CAP.MQES");
        if ((KNOWN_CAP[35:32] != 4'd0) ||
            (KNOWN_CAP[51:48] != 4'd0) ||
            (KNOWN_CAP[55:52] != 4'd0) || !KNOWN_CAP[37])
            $error("pl_pcie_nvme_setup_fsm: fixed DSTRD/MPS/CSS assumptions do not match KNOWN_CAP");
        if (KNOWN_VS != 32'h0002_0000)
            $warning("pl_pcie_nvme_setup_fsm: code was validated against NVMe VS 2.0");
        if (NVME_NSID == 32'd0)
            $error("pl_pcie_nvme_setup_fsm: NVME_NSID zero is not usable");
        if ((DISCOVERY_PCIE_ADDR[11:0] != 0) ||
            (ADMIN_SQ_PCIE_ADDR[11:0] != 0) ||
            (ADMIN_CQ_PCIE_ADDR[11:0] != 0) ||
            (IO_SQ_PCIE_ADDR[11:0] != 0) ||
            (IO_CQ_PCIE_ADDR[11:0] != 0))
            $error("pl_pcie_nvme_setup_fsm: queue/PRP addresses must be 4-KiB aligned");
        if ((DISCOVERY_RAM_OFFSET + 4096 > RAM_BYTES) ||
            (ADMIN_SQ_RAM_OFFSET + 4096 > RAM_BYTES) ||
            (ADMIN_CQ_RAM_OFFSET + 4096 > RAM_BYTES))
            $error("pl_pcie_nvme_setup_fsm: local buffers exceed RAM");
    end

endmodule
