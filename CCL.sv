// ============================================================================
// Core Complex Level (CCL) - Single Core Complex with Coherent Caches
// ============================================================================
// This module represents one core complex containing:
// - 2 L1 caches (L1a and L1b)
// - 1 L2 cache (shared by both L1s)
// - L1 Snoop controller (for L1a-L1b coherence)
// - Bus arbiter (for L1a/L1b -> L2 arbitration)
//
// The L1 caches are kept coherent via the snoop controller.
// The bus arbiter manages access to the shared L2.
// ============================================================================

module CCL (
    input  logic         clk,
    input  logic         reset,

    // -------------------------
    // CPU Interfaces (2 cores)
    // -------------------------
    // CPU A
    input  logic         cpu_a_req,
    input  logic         cpu_a_we,
    input  logic [31:0]  cpu_a_addr,
    input  logic [31:0]  cpu_a_wdata,
    output logic [31:0]  cpu_a_rdata,
    output logic         cpu_a_ready,
    output logic         cpu_a_miss,

    // CPU B
    input  logic         cpu_b_req,
    input  logic         cpu_b_we,
    input  logic [31:0]  cpu_b_addr,
    input  logic [31:0]  cpu_b_wdata,
    output logic [31:0]  cpu_b_rdata,
    output logic         cpu_b_ready,
    output logic         cpu_b_miss,

    // -------------------------
    // Memory Interface (32-bit blocking bus from L2)
    // -------------------------
    output logic         mem_valid,
    input  logic         mem_ready,
    output logic         mem_we,
    output logic         mem_rwitm,
    output logic [31:0]  mem_addr,
    input  logic [31:0]  mem_rdata,

    // -------------------------
    // L2 Snoop Interface (for multi-complex coherence)
    // -------------------------
    input  logic         l2_snoop_req,
    input  logic [31:0]  l2_snoop_addr,
    output logic         l2_snoop_has_line,
    output logic [31:0]  l2_snoop_data,
    output logic [1:0]   l2_snoop_mesi_state,
    input  logic         l2_coherence_invalidate,
    input  logic         l2_coherence_downgrade,
    input  logic [31:0]  l2_coherence_addr,
    input  logic         l2_shared_copy_exists,
    input  logic         l2_coherence_writeback,  // Cross-CCL dirty intervention: writeback request
    input  logic         l2_snoop_dirty_stall     // Cross-CCL snoop detected dirty: stall fill
);

    // =========================================================================
    // Internal Signals
    // =========================================================================

    // L1a -> Arbiter
    logic         l1a_mem_valid;
    logic         l1a_mem_ready;
    logic         l1a_mem_we;
    logic         l1a_mem_rwitm;
    logic [31:0]  l1a_mem_addr;
    logic [31:0]  l1a_mem_rdata;
    logic [1:0]   l1a_mesi_state;

    // L1b -> Arbiter
    logic         l1b_mem_valid;
    logic         l1b_mem_ready;
    logic         l1b_mem_we;
    logic         l1b_mem_rwitm;
    logic [31:0]  l1b_mem_addr;
    logic [31:0]  l1b_mem_rdata;
    logic [1:0]   l1b_mesi_state;

    // Arbiter -> L2
    logic         arb_l2_valid;
    logic         arb_l2_ready;
    logic         arb_l2_we;
    logic         arb_l2_rwitm;
    logic [31:0]  arb_l2_addr;
    logic [31:0]  arb_l2_wdata;
    logic [31:0]  arb_l2_rdata;
    logic [0:0]   arb_l2_granted_id;  // Which L1 has the bus (0=L1a, 1=L1b)

    // L1 Snoop Controller signals
    logic         l1_snoop_resp_valid;   // Snoop response valid
    logic         l1_snoop_hit;          // Snoop hit
    logic         l1_snoop_dirty;        // Snoop dirty
    logic [31:0]  l1_snoop_resp_data;    // Snoop response data
    
    logic [1:0]   l1_snoop_req;
    logic [31:0]  l1_snoop_addr;
    logic [1:0]   l1_has_line;
    logic [1:0][31:0] l1_snoop_data;
    logic [1:0]   l1_invalidate;
    logic [1:0]   l1_downgrade;
    logic [1:0]   l1_writeback;          // Write-back request per L1
    logic [31:0]  l1_coherence_addr;
    logic         l1_shared_copy_exists;
    logic [1:0][1:0] l1_mesi_states;

    // Combine L1 MESI states for snoop controller
    assign l1_mesi_states[0] = l1a_mesi_state;
    assign l1_mesi_states[1] = l1b_mesi_state;

    // L2 MESI state
    logic [1:0]   l2_mesi_state;
    
    // L2 peer writeback coordination
    logic         l2_wait_for_peer_wb;  // L2 waiting for peer WB (signal to L1s)

    // Cross-CCL L1 snoop query (for reporting L1 dirty state to cross-CCL snoop controller)
    logic        l1a_xcl_has_line, l1b_xcl_has_line;
    logic [1:0]  l1a_xcl_mesi, l1b_xcl_mesi;
    logic [1:0]  l2_raw_snoop_mesi;  // Raw L2 snoop MESI (before L1 override)

    // Cross-CCL writeback trigger for L1s
    logic l1a_xcl_wb, l1b_xcl_wb;
    assign l1a_xcl_wb = l2_coherence_writeback && l1a_xcl_has_line && (l1a_xcl_mesi == 2'b11);
    assign l1b_xcl_wb = l2_coherence_writeback && l1b_xcl_has_line && (l1b_xcl_mesi == 2'b11);

    // Signal to L2: at least one L1 has dirty data pending writeback for coherence
    logic l1_dirty_pending_wb;
    assign l1_dirty_pending_wb = l1a_xcl_wb || l1b_xcl_wb;

    // Combine L1 + L2 MESI state for cross-CCL snoop response
    // If any L1 has Modified, report Modified (L2 may only show Exclusive after silent E→M)
    assign l2_snoop_mesi_state = (l1a_xcl_has_line && l1a_xcl_mesi == 2'b11) ? 2'b11 :
                                  (l1b_xcl_has_line && l1b_xcl_mesi == 2'b11) ? 2'b11 :
                                  l2_raw_snoop_mesi;

    // =========================================================================
    // L1a Cache
    // =========================================================================
    cachel1_top l1a (
        .clk(clk),
        .reset(reset),

        // CPU Interface
        .cpu_req(cpu_a_req),
        .cpu_we(cpu_a_we),
        .cpu_addr(cpu_a_addr),
        .cpu_wdata(cpu_a_wdata),
        .cpu_rdata(cpu_a_rdata),
        .cpu_ready(cpu_a_ready),
        .cpu_miss(cpu_a_miss),

        // Memory Interface (to arbiter)
        .mem_valid(l1a_mem_valid),
        .mem_ready(l1a_mem_ready),
        .mem_we(l1a_mem_we),
        .mem_rwitm(l1a_mem_rwitm),
        .mem_addr(l1a_mem_addr),
        .mem_rdata(l1a_mem_rdata),

        // MESI State output
        .mesi_state_out(l1a_mesi_state),

        // Snoop Interface (from L1 snoop controller + cross-CCL cascade)
        .snoop_req(l1_snoop_req[0]),
        .snoop_addr(l1_snoop_addr),
        .has_line(l1_has_line[0]),
        .snoop_data(l1_snoop_data[0]),
        .invalidate(l1_invalidate[0] | l2_coherence_invalidate),
        .downgrade(l1_downgrade[0] | l2_coherence_downgrade),
        .coherence_addr((l2_coherence_invalidate | l2_coherence_downgrade) ? l2_coherence_addr : l1_coherence_addr),
        .writeback_req(l1_writeback[0] | l1a_xcl_wb),
        .writeback_addr(l1a_xcl_wb ? l2_coherence_addr : l1_coherence_addr),
        
        // Bus snooping (to capture peer writebacks)
        .bus_valid(arb_l2_valid),
        .bus_we(arb_l2_we),
        .bus_addr(arb_l2_addr),
        .bus_wdata(arb_l2_wdata),

        // Coherence information
        .shared_copy_exists(l1_shared_copy_exists | l2_shared_copy_exists),
        .wait_for_peer_wb(l2_wait_for_peer_wb),
        
        // Cross-CCL snoop query
        .xcl_snoop_addr(l2_snoop_addr),
        .xcl_has_line(l1a_xcl_has_line),
        .xcl_mesi_state(l1a_xcl_mesi)
    );

    // =========================================================================
    // L1b Cache
    // =========================================================================
    cachel1_top l1b (
        .clk(clk),
        .reset(reset),

        // CPU Interface
        .cpu_req(cpu_b_req),
        .cpu_we(cpu_b_we),
        .cpu_addr(cpu_b_addr),
        .cpu_wdata(cpu_b_wdata),
        .cpu_rdata(cpu_b_rdata),
        .cpu_ready(cpu_b_ready),
        .cpu_miss(cpu_b_miss),

        // Memory Interface (to arbiter)
        .mem_valid(l1b_mem_valid),
        .mem_ready(l1b_mem_ready),
        .mem_we(l1b_mem_we),
        .mem_rwitm(l1b_mem_rwitm),
        .mem_addr(l1b_mem_addr),
        .mem_rdata(l1b_mem_rdata),

        // MESI State output
        .mesi_state_out(l1b_mesi_state),

        // Snoop Interface (from L1 snoop controller)
        .snoop_req(l1_snoop_req[1]),
        .snoop_addr(l1_snoop_addr),
        .has_line(l1_has_line[1]),
        .snoop_data(l1_snoop_data[1]),
        .invalidate(l1_invalidate[1] | l2_coherence_invalidate),
        .downgrade(l1_downgrade[1] | l2_coherence_downgrade),
        .coherence_addr((l2_coherence_invalidate | l2_coherence_downgrade) ? l2_coherence_addr : l1_coherence_addr),
        .writeback_req(l1_writeback[1] | l1b_xcl_wb),
        .writeback_addr(l1b_xcl_wb ? l2_coherence_addr : l1_coherence_addr),
        
        // Bus snooping (to capture peer writebacks)
        .bus_valid(arb_l2_valid),
        .bus_we(arb_l2_we),
        .bus_addr(arb_l2_addr),
        .bus_wdata(arb_l2_wdata),

        // Coherence information
        .shared_copy_exists(l1_shared_copy_exists | l2_shared_copy_exists),
        .wait_for_peer_wb(l2_wait_for_peer_wb),
        
        // Cross-CCL snoop query
        .xcl_snoop_addr(l2_snoop_addr),
        .xcl_has_line(l1b_xcl_has_line),
        .xcl_mesi_state(l1b_xcl_mesi)
    );

    // =========================================================================
    // L1 Snoop Controller (monitors L1a and L1b for coherence)
    // =========================================================================
    // Monitors L1->L2 arbiter's granted bus transactions to trigger snooping
    // The arbiter ensures only one L1 at a time, so no snoop arbitration needed
    // =========================================================================
    snoop_controller #(
        .NUM_CACHES(2),
        .DATA_WIDTH(32)
    ) l1_snoop (
        .clk(clk),
        .reset(reset),

        // External snoop bus - connected to L1->L2 arbiter's granted transaction
        .snoop_req(arb_l2_valid),        // Snoop when L1->L2 bus transaction is active
        .snoop_addr(arb_l2_addr),        // Address from granted L1 cache
        .snoop_read(!arb_l2_we && !arb_l2_rwitm),  // Read if not a write or RWITM
        .snoop_write(arb_l2_we),         // Write operation (writeback)
        .snoop_rwitm(arb_l2_rwitm),      // Read-With-Intent-To-Modify (write miss)
        .snoop_requester_id(arb_l2_granted_id),  // Which L1 is making the request
        .snoop_resp_valid(l1_snoop_resp_valid),
        .snoop_hit(l1_snoop_hit),
        .snoop_dirty(l1_snoop_dirty),
        .snoop_data(l1_snoop_resp_data),
        .shared_copy_exists(l1_shared_copy_exists),
        .snoop_dirty_stall(),            // Not used at L1 level

        // Peer cache interfaces (L1a and L1b)
        .cache_snoop_req(l1_snoop_req),
        .cache_snoop_addr(l1_snoop_addr),
        .cache_mesi_state(l1_mesi_states),
        .cache_has_line(l1_has_line),
        .cache_snoop_data(l1_snoop_data),
        .cache_invalidate(l1_invalidate),
        .cache_downgrade(l1_downgrade),
        .cache_coherence_addr(l1_coherence_addr),
        
        // Cache-to-cache transfer coordination
        .cache_writeback(l1_writeback),
        
        // Bus monitoring (to detect writebacks)
        .bus_valid(arb_l2_valid),
        .bus_we(arb_l2_we),
        .bus_addr(arb_l2_addr)
    );

    // =========================================================================
    // Bus Arbiter (arbitrates L1a and L1b access to L2)
    // =========================================================================
    bus_arbiter #(
        .NUM_REQUESTORS(2)
    ) l1_l2_arbiter (
        .clk(clk),
        .reset(reset),

        // Requestor interfaces (L1a and L1b)
        .req_valid({l1b_mem_valid, l1a_mem_valid}),
        .req_ready({l1b_mem_ready, l1a_mem_ready}),
        .req_we({l1b_mem_we, l1a_mem_we}),
        .req_rwitm({l1b_mem_rwitm, l1a_mem_rwitm}),
        .req_addr({l1b_mem_addr, l1a_mem_addr}),
        .req_wdata({32'b0, 32'b0}),  // Write data not needed for L1->L2 (writes handled in cache)
        .req_rdata({l1b_mem_rdata, l1a_mem_rdata}),

        // Shared bus to L2
        .bus_valid(arb_l2_valid),
        .bus_ready(arb_l2_ready),
        .bus_we(arb_l2_we),
        .bus_rwitm(arb_l2_rwitm),
        .bus_addr(arb_l2_addr),
        .bus_wdata(arb_l2_wdata),
        .bus_rdata(arb_l2_rdata),
        .granted_id_out(arb_l2_granted_id)
    );

    // =========================================================================
    // L2 Cache
    // =========================================================================
    cachel2_top l2 (
        .clk(clk),
        .reset(reset),

        // CPU Interface (from arbiter)
        .cpu_req(arb_l2_valid),
        .cpu_we(arb_l2_we),
        .cpu_rwitm(arb_l2_rwitm),
        .cpu_addr(arb_l2_addr),
        .cpu_wdata(arb_l2_wdata),
        .cpu_rdata(arb_l2_rdata),
        .cpu_ready(arb_l2_ready),

        // Memory Interface (to next level)
        .mem_valid(mem_valid),
        .mem_ready(mem_ready),
        .mem_we(mem_we),
        .mem_rwitm(mem_rwitm),
        .mem_addr(mem_addr),
        .mem_rdata(mem_rdata),

        // MESI State output
        .mesi_state_out(l2_mesi_state),

        // Snoop Interface (from other L2's snoop controller)
        .snoop_req(l2_snoop_req),
        .snoop_addr(l2_snoop_addr),
        .snoop_has_line(l2_snoop_has_line),
        .snoop_data(l2_snoop_data),
        .snoop_mesi_state(l2_raw_snoop_mesi),

        // Coherence Interface
        .coherence_invalidate(l2_coherence_invalidate),
        .coherence_downgrade(l2_coherence_downgrade),
        .coherence_addr(l2_coherence_addr),

        // Coherence information
        .shared_copy_exists(l2_shared_copy_exists),
        .l1_has_dirty_data(l1_snoop_dirty),  // L1 has Modified copy
        .l1_snoop_data(l1_snoop_resp_data),   // Fresh data from L1 intervention
        .wait_for_peer_wb(l2_wait_for_peer_wb),  // Signal L1s to release bus
        
        // Cross-CCL dirty intervention
        .coherence_writeback(l2_coherence_writeback),
        .snoop_dirty_stall(l2_snoop_dirty_stall),
        .l1_dirty_pending_wb(l1_dirty_pending_wb)
    );

endmodule