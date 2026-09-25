/**
 * This file is part of the Coyote <https://github.com/fpgasystems/Coyote>
 *
 * MIT Licence
 * Copyright (c) 2021-2026, Systems Group, ETH Zurich
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


`timescale 1ns/1ps

import lynxTypes::*;

/**
 * @brief   NVMe info table
 *
 * Stores per-device NVMe info and per-(region, device) permissions.
 * SQ space follows SQHD; CID/PRP ownership follows completion, independently.
 */
module nvme_info_table #(
    parameter int MAX_NVME_DEVICES = 16,
    parameter int MAX_NSID         = 256
) (
    input  logic        aclk,
    input  logic        aresetn,

    metaIntf.s          s_tbl_req,         // nvme_info_req_t (with region_id, naddr, len)
    metaIntf.m          m_tbl_rsp,         // nvme_info_rsp_t (with lba_offset)

    // Internal sidebands; shared request/response types stay unchanged.
    output logic [NVME_QUEUE_BITS-1:0] alloc_cid,
    input logic cpl_valid,
    input logic [N_NVME_BITS-1:0] cpl_dev,
    input logic [NVME_QUEUE_BITS-1:0] cpl_cid,
    input logic [15:0] cpl_sq_head,
    input logic resolve_valid,
    input logic resolve_abort,
    input logic [N_NVME_BITS-1:0] resolve_dev,
    input logic [NVME_QUEUE_BITS-1:0] resolve_cid,
    metaIntf.s          s_update_tbl,      // update_tbl_t (device info)
    metaIntf.s          s_perm_update      // nvme_perm_update_t (region permission)
);

    // Storage arrays
    nvme_info_entry_t nvme_info_tbl [MAX_NVME_DEVICES][MAX_NSID];
    logic [NVME_QUEUE_BITS-1:0] sq_tail [MAX_NVME_DEVICES];
    logic [NVME_QUEUE_BITS-1:0] sq_head [MAX_NVME_DEVICES];
    logic [63:0] cid_owned [MAX_NVME_DEVICES];
    // At most one unpublished SQ slot per device. Tail commits only after
    // SQE storage, so a preparation failure can cancel without an SQ hole.
    logic sq_reserved [MAX_NVME_DEVICES];
    logic [NVME_QUEUE_BITS-1:0] reserved_cid [MAX_NVME_DEVICES];
    nvme_perm_entry_t perm_table    [N_REGIONS][MAX_NVME_DEVICES];

    // Response registers
    nvme_info_rsp_t   rsp_C, rsp_N;
    logic             rsp_valid_C, rsp_valid_N;

    // Temporaries
    logic             rsp_free;
    logic             request_ready;
    logic [NVME_QUEUE_BITS-1:0] rsp_cid_C;
    logic [NVME_QUEUE_BITS-1:0] next_sq_tail;

    // Register the device read, then two levels of eight-way selection, not a
    // 64-way priority chain on request READY. The final check rejects stale choices
    // after allocation; concurrent releases can only make more CIDs free.
    function automatic logic [2:0] first_free8(input logic [7:0] bits_free);
        casez (bits_free)
            8'b???????1: first_free8 = 3'd0;
            8'b??????10: first_free8 = 3'd1;
            8'b?????100: first_free8 = 3'd2;
            8'b????1000: first_free8 = 3'd3;
            8'b???10000: first_free8 = 3'd4;
            8'b??100000: first_free8 = 3'd5;
            8'b?1000000: first_free8 = 3'd6;
            default:     first_free8 = 3'd7;
        endcase
    endfunction

    logic [7:0] free_groups_C;
    logic [7:0][2:0] free_indices_C;
    logic [N_NVME_BITS-1:0] group_dev_C, candidate_dev_C;
    logic group_valid_C, candidate_valid_C;
    logic [5:0] candidate_cid_C;
    logic [63:0] free_bits;
    logic [63:0] free_bits_C;
    logic [N_NVME_BITS-1:0] selected_dev_C;
    logic selected_valid_C;
    logic [2:0] selected_group;
    assign free_bits = (s_tbl_req.data.dev_id < MAX_NVME_DEVICES) ?
                       ~cid_owned[s_tbl_req.data.dev_id] : 64'b0;
    assign selected_group = first_free8(free_groups_C);
    assign alloc_cid = rsp_cid_C;
    assign s_tbl_req.ready = request_ready;
    assign next_sq_tail = sq_tail[s_tbl_req.data.dev_id] + 1'b1;

    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            free_groups_C <= '0;
            free_indices_C <= '0;
            group_dev_C <= '0;
            candidate_dev_C <= '0;
            group_valid_C <= 1'b0;
            candidate_valid_C <= 1'b0;
            candidate_cid_C <= '0;
            free_bits_C <= '0;
            selected_dev_C <= '0;
            selected_valid_C <= 1'b0;
        end else begin
            free_bits_C <= free_bits;
            selected_dev_C <= s_tbl_req.data.dev_id;
            selected_valid_C <= s_tbl_req.valid;
            group_dev_C <= selected_dev_C;
            group_valid_C <= selected_valid_C;
            for (int g = 0; g < 8; g++) begin
                free_groups_C[g] <= |free_bits_C[g*8 +: 8];
                free_indices_C[g] <= first_free8(free_bits_C[g*8 +: 8]);
            end
            candidate_dev_C <= group_dev_C;
            candidate_valid_C <= group_valid_C && (|free_groups_C);
            candidate_cid_C <= {selected_group, free_indices_C[selected_group]};
        end
    end

`ifndef SYNTHESIS
    initial assert (NVME_QUEUE_BITS == 6)
        else $fatal(1, "NVMe CID bitmap/encoder requires 64 command contexts");
`endif

    integer i, j, k;

    // Combinational logic
    always_comb begin
        // Defaults
        s_update_tbl.ready     = 1'b0;
        s_perm_update.ready    = 1'b0;
        request_ready         = 1'b0;

        m_tbl_rsp.valid = rsp_valid_C;
        m_tbl_rsp.data  = rsp_C;

        rsp_N       = rsp_C;
        rsp_valid_N = rsp_valid_C;

        // Response slot is free when empty or being consumed this cycle
        rsp_free = (~rsp_valid_C) || (rsp_valid_C && m_tbl_rsp.ready);

        // Clear valid when consumed
        if (rsp_valid_C && m_tbl_rsp.ready) begin
            rsp_valid_N = 1'b0;
        end

        // One accepted op per cycle when response slot is free
        // Credit returns and reservation resolution below are independent of
        // this response slot, so stalled admission cannot block completions.
        if (rsp_free) begin
            if (s_update_tbl.valid) begin
                // Queue recreation is quiescent; SQHD alone cannot prove that
                // the SSD has finished reading a live command's PRP storage.
                s_update_tbl.ready = !s_update_tbl.data.reset_queue ||
                    (cid_owned[s_update_tbl.data.dev_id] == '0);
            end
            else if (s_perm_update.valid) begin
                s_perm_update.ready = 1'b1;
            end
            else if (s_tbl_req.valid) begin
                request_ready = 1'b1;

                // Generate response based on table lookup
                rsp_N = '0;

                // Bounds check
                if (s_tbl_req.data.dev_id >= MAX_NVME_DEVICES || s_tbl_req.data.nsid >= MAX_NSID) begin
                    rsp_N.error = NVME_NO_DEVICE;
                end
                // Valid check
                else if (!nvme_info_tbl[s_tbl_req.data.dev_id][s_tbl_req.data.nsid].valid) begin
                    rsp_N.error = NVME_NO_DEVICE;
                end
                // Permission check: region allowed for this device?
                else if (!perm_table[s_tbl_req.data.region_id][s_tbl_req.data.dev_id].valid) begin
                    rsp_N.error = NVME_PERMISSION_DENIED;
                end
                // Permission check: offset + len within allowed range?
                else if ((s_tbl_req.data.naddr + s_tbl_req.data.len) >
                         perm_table[s_tbl_req.data.region_id][s_tbl_req.data.dev_id].lba_size) begin
                    rsp_N.error = NVME_PERMISSION_DENIED;
                end
                // Both an SQ slot and an independently owned CID are needed.
                else if (sq_reserved[s_tbl_req.data.dev_id] ||
                         next_sq_tail == sq_head[s_tbl_req.data.dev_id] ||
                         !candidate_valid_C || candidate_dev_C != s_tbl_req.data.dev_id ||
                         cid_owned[s_tbl_req.data.dev_id][candidate_cid_C]) begin
                    request_ready = 1'b0;
                end
                // Success path
                else begin
                    rsp_N.error      = NVME_NO_ERROR;
                    rsp_N.dev_id     = s_tbl_req.data.dev_id;
                    rsp_N.nsid       = s_tbl_req.data.nsid;
                    rsp_N.lbaf       = nvme_info_tbl[s_tbl_req.data.dev_id][s_tbl_req.data.nsid].lbaf;
                    rsp_N.nsze       = nvme_info_tbl[s_tbl_req.data.dev_id][s_tbl_req.data.nsid].nsze;
                    rsp_N.sq_db_addr = nvme_info_tbl[s_tbl_req.data.dev_id][s_tbl_req.data.nsid].sq_db_addr;
                    rsp_N.sq_tail    = sq_tail[s_tbl_req.data.dev_id];
                    rsp_N.lba_offset = perm_table[s_tbl_req.data.region_id][s_tbl_req.data.dev_id].lba_offset;
                end

                // Emit a response only when the request was accepted; a
                // backpressured (SQ-full) request produces no response.
                rsp_valid_N = request_ready;
            end
        end
    end

    // Sequential logic
    always_ff @(posedge aclk) begin
        if (!aresetn) begin
            rsp_C       <= '0;
            rsp_valid_C <= 1'b0;
            rsp_cid_C   <= '0;

            for (i = 0; i < MAX_NVME_DEVICES; i++) begin
                sq_tail[i] <= '0;
                sq_head[i] <= '0;
                cid_owned[i] <= '0;
                sq_reserved[i] <= 1'b0;
                reserved_cid[i] <= '0;
                for (j = 0; j < MAX_NSID; j++) begin
                    nvme_info_tbl[i][j] <= '0;
                end
            end
            for (k = 0; k < N_REGIONS; k++) begin
                for (i = 0; i < MAX_NVME_DEVICES; i++) begin
                    perm_table[k][i] <= '0;
                end
            end
        end
        else begin
            rsp_C       <= rsp_N;
            rsp_valid_C <= rsp_valid_N;

            // CQEs arrive here in CQ-ring order. SQHD frees only SQ entries;
            // only this completed command loses CID/PRP ownership.
            if (cpl_valid) begin
                if (cpl_sq_head < 64)
                    sq_head[cpl_dev] <= cpl_sq_head[NVME_QUEUE_BITS-1:0];
                cid_owned[cpl_dev][cpl_cid] <= 1'b0;
            end
            if (resolve_valid && sq_reserved[resolve_dev] &&
                reserved_cid[resolve_dev] == resolve_cid) begin
                sq_reserved[resolve_dev] <= 1'b0;
                if (resolve_abort)
                    cid_owned[resolve_dev][resolve_cid] <= 1'b0;
                else
                    sq_tail[resolve_dev] <= sq_tail[resolve_dev] + 1'b1;
            end

            // Update device info (highest priority)
            if (s_update_tbl.valid && s_update_tbl.ready) begin
                nvme_info_tbl[s_update_tbl.data.dev_id][s_update_tbl.data.nsid].lbaf       <= s_update_tbl.data.lbaf;
                nvme_info_tbl[s_update_tbl.data.dev_id][s_update_tbl.data.nsid].nsze       <= s_update_tbl.data.nsze;
                nvme_info_tbl[s_update_tbl.data.dev_id][s_update_tbl.data.nsid].valid      <= s_update_tbl.data.valid;
                nvme_info_tbl[s_update_tbl.data.dev_id][s_update_tbl.data.nsid].sq_db_addr <= s_update_tbl.data.sq_db_addr;

                if (s_update_tbl.data.reset_queue) begin
                    sq_tail[s_update_tbl.data.dev_id] <= '0;
                    sq_head[s_update_tbl.data.dev_id] <= '0;
                    cid_owned[s_update_tbl.data.dev_id] <= '0;
                    sq_reserved[s_update_tbl.data.dev_id] <= 1'b0;
                end
            end
            // Update permission entry
            else if (s_perm_update.valid && s_perm_update.ready) begin
                perm_table[s_perm_update.data.region_id][s_perm_update.data.dev_id].lba_offset <= s_perm_update.data.lba_offset;
                perm_table[s_perm_update.data.region_id][s_perm_update.data.dev_id].lba_size   <= s_perm_update.data.lba_size;
                perm_table[s_perm_update.data.region_id][s_perm_update.data.dev_id].valid      <= 1'b1;
            end
            // Reserve a free CID and the current unpublished SQ slot.
            else if (s_tbl_req.valid && s_tbl_req.ready) begin
                if (rsp_N.error == NVME_NO_ERROR) begin
                    cid_owned[s_tbl_req.data.dev_id][candidate_cid_C] <= 1'b1;
                    sq_reserved[s_tbl_req.data.dev_id] <= 1'b1;
                    reserved_cid[s_tbl_req.data.dev_id] <= candidate_cid_C;
                    rsp_cid_C <= candidate_cid_C;
                end
            end
        end
    end

    // ILA Debug
// `define EN_ILA_NVME_INFO_TABLE
`ifdef EN_ILA_NVME_INFO_TABLE
    ila_nvme_info_table inst_ila_nvme_info_table (
        .clk    (aclk),
        // s_tbl_req
        .probe0 (s_tbl_req.valid),                      // 1
        .probe1 (s_tbl_req.ready),                      // 1
        .probe2 (s_tbl_req.data.dev_id),                // N_NVME_BITS (4)
        .probe3 (s_tbl_req.data.region_id),             // REGION_ID_BITS
        .probe4 (s_tbl_req.data.naddr),                 // ADDR_BITS (48)
        .probe5 (s_tbl_req.data.len),                   // LEN_BITS (28)
        // m_tbl_rsp
        .probe6 (m_tbl_rsp.valid),                      // 1
        .probe7 (m_tbl_rsp.ready),                      // 1
        .probe8 (m_tbl_rsp.data.error),                 // 16
        .probe9 (m_tbl_rsp.data.lba_offset),            // 64
        .probe10(m_tbl_rsp.data.sq_tail),               // NVME_QUEUE_BITS (6)
        // Completion/SQ consumption, independent of CQ acknowledgement
        .probe11(cpl_valid),                            // 1
        .probe12(candidate_valid_C),                    // 1
        .probe13(cpl_dev),                              // N_NVME_BITS (4)
        .probe14(cpl_sq_head[NVME_QUEUE_BITS-1:0]),      // NVME_QUEUE_BITS (6)
        // s_update_tbl
        .probe15(s_update_tbl.valid),                   // 1
        .probe16(s_update_tbl.ready),                   // 1
        .probe17(s_update_tbl.data.dev_id),             // N_NVME_BITS (4)
        // s_perm_update
        .probe18(s_perm_update.valid),                  // 1
        .probe19(s_perm_update.ready),                  // 1
        .probe20(s_perm_update.data.region_id),         // REGION_ID_BITS
        .probe21(s_perm_update.data.dev_id),            // N_NVME_BITS (4)
        // Internal
        .probe22(rsp_free),                             // 1
        .probe23(rsp_valid_C)                           // 1
    );
`endif

endmodule
