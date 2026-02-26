// ============================================================================
// Top Module - Multi-Core Complex System
// ============================================================================
// Instantiates two Core Complexes (CCL0 and CCL1), each with:
//   - 2 L1 caches (one per core)
//   - 1 shared L2 cache
//   - Internal L1 snoop controller
//
// Provides:
//   - L2 snoop controller for inter-complex coherence
//   - Memory arbiter for shared memory access
// ============================================================================

module Top (
    input  logic         clk,
    input  logic         reset,

    // -------------------------
    // CPU Interfaces (4 cores total: 2 per CCL)
    // -------------------------
    // CCL0: Core 0 and Core 1
    input  logic         cpu0_req,
    input  logic         cpu0_we,
    input  logic [31:0]  cpu0_addr,
    input  logic [31:0]  cpu0_wdata,
    output logic [31:0]  cpu0_rdata,
    output logic         cpu0_ready,
    output logic         cpu0_miss,

    input  logic         cpu1_req,
    input  logic         cpu1_we,
    input  logic [31:0]  cpu1_addr,
    input  logic [31:0]  cpu1_wdata,
    output logic [31:0]  cpu1_rdata,
    output logic         cpu1_ready,
    output logic         cpu1_miss,

    // CCL1: Core 2 and Core 3
    input  logic         cpu2_req,
    input  logic         cpu2_we,
    input  logic [31:0]  cpu2_addr,
    input  logic [31:0]  cpu2_wdata,
    output logic [31:0]  cpu2_rdata,
    output logic         cpu2_ready,
    output logic         cpu2_miss,

    input  logic         cpu3_req,
    input  logic         cpu3_we,
    input  logic [31:0]  cpu3_addr,
    input  logic [31:0]  cpu3_wdata,
    output logic [31:0]  cpu3_rdata,
    output logic         cpu3_ready,
    output logic         cpu3_miss,

    // -------------------------
    // Main Memory Interface (32-bit blocking bus)
    // -------------------------
    output logic         mem_valid,
    input  logic         mem_ready,
    output logic         mem_we,
    output logic [31:0]  mem_addr,
    input  logic [31:0]  mem_rdata
);

    // =========================================================================
    // Internal Signals
    // =========================================================================

    // CCL0 -> Memory Arbiter
    logic         ccl0_mem_valid;
    logic         ccl0_mem_ready;
    logic         ccl0_mem_we;
    logic         ccl0_mem_rwitm;
    logic [31:0]  ccl0_mem_addr;
    logic [31:0]  ccl0_mem_rdata;
    logic         ccl0_wait_for_peer_wb;  // L2 waiting for peer WB

    // CCL1 -> Memory Arbiter
    logic         ccl1_mem_valid;
    logic         ccl1_mem_ready;
    logic         ccl1_mem_we;
    logic         ccl1_mem_rwitm;
    logic [31:0]  ccl1_mem_addr;
    logic [31:0]  ccl1_mem_rdata;
    logic         ccl1_wait_for_peer_wb;  // L2 waiting for peer WB

    // L2 Snoop Controller signals
    logic [1:0]         l2_snoop_req;
    logic [31:0]        l2_snoop_addr;
    logic [1:0]         l2_has_line;
    logic [1:0][31:0]   l2_snoop_data;
    logic [1:0]         l2_invalidate;
    logic [1:0]         l2_downgrade;
    logic [31:0]        l2_coherence_addr;
    logic               l2_shared_copy_exists;
    logic [1:0]         l2_writeback;  // Write-back request per L2
    logic               l2_snoop_dirty_stall;  // Stall requesting CCL fill during dirty intervention

    // L2 Snoop response (unused externally)
    logic               l2_snoop_resp_valid;
    logic               l2_snoop_hit;
    logic               l2_snoop_dirty;
    logic [31:0]        l2_snoop_resp_data;
    
    // Memory arbiter internal signals
    logic               mem_rwitm_internal;  // RWITM from arbiter (for snoop controller)
    logic [0:0]         mem_arbiter_granted_id;  // Which CCL has the memory bus (0=CCL0, 1=CCL1)

    // CCL MESI states (driven by L2 snoop interface — actual MESI state of snooped line)
    logic [1:0][1:0]    l2_mesi_states;

    // =========================================================================
    // CCL0 - Core Complex 0 (Cores 0 and 1)
    // =========================================================================
    CCL ccl0 (
        .clk(clk),
        .reset(reset),

        // CPU interfaces
        .cpu_a_req(cpu0_req),
        .cpu_a_we(cpu0_we),
        .cpu_a_addr(cpu0_addr),
        .cpu_a_wdata(cpu0_wdata),
        .cpu_a_rdata(cpu0_rdata),
        .cpu_a_ready(cpu0_ready),
        .cpu_a_miss(cpu0_miss),

        .cpu_b_req(cpu1_req),
        .cpu_b_we(cpu1_we),
        .cpu_b_addr(cpu1_addr),
        .cpu_b_wdata(cpu1_wdata),
        .cpu_b_rdata(cpu1_rdata),
        .cpu_b_ready(cpu1_ready),
        .cpu_b_miss(cpu1_miss),

        // Memory interface (to arbiter)
        .mem_valid(ccl0_mem_valid),
        .mem_ready(ccl0_mem_ready),
        .mem_we(ccl0_mem_we),
        .mem_rwitm(ccl0_mem_rwitm),
        .mem_addr(ccl0_mem_addr),
        .mem_rdata(ccl0_mem_rdata),

        // L2 Snoop interface (from L2 snoop controller)
        .l2_snoop_req(l2_snoop_req[0]),
        .l2_snoop_addr(l2_snoop_addr),
        .l2_snoop_has_line(l2_has_line[0]),
        .l2_snoop_data(l2_snoop_data[0]),
        .l2_snoop_mesi_state(l2_mesi_states[0]),
        .l2_coherence_invalidate(l2_invalidate[0]),
        .l2_coherence_downgrade(l2_downgrade[0]),
        .l2_coherence_addr(l2_coherence_addr),
        .l2_shared_copy_exists(l2_shared_copy_exists),
        .l2_coherence_writeback(l2_writeback[0]),
        .l2_snoop_dirty_stall(l2_snoop_dirty_stall)
    );

    // =========================================================================
    // CCL1 - Core Complex 1 (Cores 2 and 3)
    // =========================================================================
    CCL ccl1 (
        .clk(clk),
        .reset(reset),

        // CPU interfaces
        .cpu_a_req(cpu2_req),
        .cpu_a_we(cpu2_we),
        .cpu_a_addr(cpu2_addr),
        .cpu_a_wdata(cpu2_wdata),
        .cpu_a_rdata(cpu2_rdata),
        .cpu_a_ready(cpu2_ready),
        .cpu_a_miss(cpu2_miss),

        .cpu_b_req(cpu3_req),
        .cpu_b_we(cpu3_we),
        .cpu_b_addr(cpu3_addr),
        .cpu_b_wdata(cpu3_wdata),
        .cpu_b_rdata(cpu3_rdata),
        .cpu_b_ready(cpu3_ready),
        .cpu_b_miss(cpu3_miss),

        // Memory interface (to arbiter)
        .mem_valid(ccl1_mem_valid),
        .mem_ready(ccl1_mem_ready),
        .mem_we(ccl1_mem_we),
        .mem_rwitm(ccl1_mem_rwitm),
        .mem_addr(ccl1_mem_addr),
        .mem_rdata(ccl1_mem_rdata),

        // L2 Snoop interface (from L2 snoop controller)
        .l2_snoop_req(l2_snoop_req[1]),
        .l2_snoop_addr(l2_snoop_addr),
        .l2_snoop_has_line(l2_has_line[1]),
        .l2_snoop_data(l2_snoop_data[1]),
        .l2_snoop_mesi_state(l2_mesi_states[1]),
        .l2_coherence_invalidate(l2_invalidate[1]),
        .l2_coherence_downgrade(l2_downgrade[1]),
        .l2_coherence_addr(l2_coherence_addr),
        .l2_shared_copy_exists(l2_shared_copy_exists),
        .l2_coherence_writeback(l2_writeback[1]),
        .l2_snoop_dirty_stall(l2_snoop_dirty_stall)
    );

    // =========================================================================
    // L2 Snoop Controller (monitors both CCL L2 caches for coherence)
    // =========================================================================
    // Monitors memory arbiter's granted bus transactions to trigger snooping
    // The arbiter ensures only one requestor at a time, so no snoop arbitration needed
    // =========================================================================
    snoop_controller #(
        .NUM_CACHES(2),
        .DATA_WIDTH(32)
    ) l2_snoop (
        .clk(clk),
        .reset(reset),

        // External snoop bus - connected to memory arbiter's granted transaction
        .snoop_req(mem_valid),               // Snoop when memory bus transaction is active
        .snoop_addr(mem_addr),               // Address from granted L2 cache
        .snoop_read(!mem_we && !mem_rwitm_internal),  // Read if not a write or RWITM
        .snoop_write(mem_we),                // Write operation (writeback)
        .snoop_rwitm(mem_rwitm_internal),    // Read-With-Intent-To-Modify (write miss)
        .snoop_requester_id(mem_arbiter_granted_id),  // Which CCL is making the request
        .snoop_resp_valid(l2_snoop_resp_valid),
        .snoop_hit(l2_snoop_hit),
        .snoop_dirty(l2_snoop_dirty),
        .snoop_data(l2_snoop_resp_data),
        .shared_copy_exists(l2_shared_copy_exists),
        .snoop_dirty_stall(l2_snoop_dirty_stall),

        // L2 cache interfaces (CCL0 and CCL1)
        .cache_snoop_req(l2_snoop_req),
        .cache_snoop_addr(l2_snoop_addr),
        .cache_mesi_state(l2_mesi_states),
        .cache_has_line(l2_has_line),
        .cache_snoop_data(l2_snoop_data),
        .cache_invalidate(l2_invalidate),
        .cache_downgrade(l2_downgrade),
        .cache_coherence_addr(l2_coherence_addr),
        
        // Cache-to-cache transfer coordination
        .cache_writeback(l2_writeback),
        
        // Bus monitoring (to detect writebacks)
        .bus_valid(mem_valid),
        .bus_we(mem_we),
        .bus_addr(mem_addr)
    );

    // =========================================================================
    // Memory Arbiter (arbitrates CCL0 and CCL1 access to main memory)
    // =========================================================================
    bus_arbiter #(
        .NUM_REQUESTORS(2)
    ) mem_arbiter (
        .clk(clk),
        .reset(reset),

        // Requestor interfaces (CCL0 and CCL1 L2 caches)
        .req_valid({ccl1_mem_valid, ccl0_mem_valid}),
        .req_ready({ccl1_mem_ready, ccl0_mem_ready}),
        .req_we({ccl1_mem_we, ccl0_mem_we}),
        .req_rwitm({ccl1_mem_rwitm, ccl0_mem_rwitm}),
        .req_addr({ccl1_mem_addr, ccl0_mem_addr}),
        .req_wdata({32'b0, 32'b0}),  // Write data handled by cache controller
        .req_rdata({ccl1_mem_rdata, ccl0_mem_rdata}),

        // Shared memory bus
        .bus_valid(mem_valid),
        .bus_ready(mem_ready),
        .bus_we(mem_we),
        .bus_rwitm(mem_rwitm_internal),
        .bus_addr(mem_addr),
        .bus_wdata(),  // Unused
        .bus_rdata(mem_rdata),
        .granted_id_out(mem_arbiter_granted_id)
    );

endmodule