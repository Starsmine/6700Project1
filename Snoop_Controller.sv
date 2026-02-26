// ============================================================================
// Generic MESI Snoop Controller
// ============================================================================
// This module handles bus snooping for cache coherence at any level of the
// memory hierarchy. It monitors snoop requests and coordinates coherence 
// actions across multiple peer caches (L1s under an L2, or L2s in the system).
//
// Features:
// - Generic: works at any cache level
// - Monitors snoop requests from other caches
// - Checks multiple peer caches for matching addresses
// - Coordinates cache-to-cache transfers
// - Handles invalidations and downgrades
// - Supports MESI protocol state transitions
//
// Typical Usage:
//   - One snoop controller per shared cache level
//   - L1 Snoop Controller: Monitors multiple L1s under an L2
//   - L2 Snoop Controller: Monitors multiple L2s in a multi-complex system
// ============================================================================

module snoop_controller #(
    parameter NUM_CACHES = 2,     // Number of peer caches to monitor
    parameter DATA_WIDTH = 32     // Width of cache-to-cache transfer (32 for L1, 128 for L2)
)(
    input  logic         clk,
    input  logic         reset,

    // -------------------------
    // Snoop Bus Interface (from other caches/cores)
    // -------------------------
    input  logic         snoop_req,          // Snoop request valid
    input  logic [31:0]  snoop_addr,         // Address to snoop
    input  logic         snoop_read,         // Snoop is a read operation
    input  logic         snoop_write,        // Snoop is a write operation (invalidate)
    input  logic         snoop_rwitm,        // Read-with-intent-to-modify
    input  logic [$clog2(NUM_CACHES)-1:0] snoop_requester_id,  // Which cache is making the request
    output logic         snoop_resp_valid,   // Snoop response ready
    output logic         snoop_hit,          // At least one cache has the line
    output logic         snoop_dirty,        // At least one cache has dirty data
    output logic [DATA_WIDTH-1:0] snoop_data, // Data from cache-to-cache transfer
    output logic         shared_copy_exists, // Another cache has a valid copy (for MESI E/S decision)
    output logic         snoop_dirty_stall,  // Stall requesting CCL fill during dirty intervention

    // -------------------------
    // Peer Cache Interfaces (per cache)
    // -------------------------
    // Cache MESI State Queries
    output logic [NUM_CACHES-1:0]         cache_snoop_req,      // Snoop request to each cache
    output logic [31:0]                   cache_snoop_addr,     // Address to check
    input  logic [NUM_CACHES-1:0] [1:0]   cache_mesi_state,     // Current MESI state of line
    input  logic [NUM_CACHES-1:0]         cache_has_line,       // Cache has this line (valid)
    input  logic [NUM_CACHES-1:0] [DATA_WIDTH-1:0] cache_snoop_data, // Data from each cache
    
    // Cache MESI State Updates (broadcast to all caches)
    output logic [NUM_CACHES-1:0]         cache_invalidate,     // Invalidate command
    output logic [NUM_CACHES-1:0]         cache_downgrade,      // Downgrade M/E -> S
    output logic [NUM_CACHES-1:0]         cache_writeback,      // Request write-back from dirty cache
    output logic [31:0]                   cache_coherence_addr, // Address for coherence action
    
    // -------------------------
    // Bus Snooping (to detect write-backs)
    // -------------------------
    input  logic         bus_valid,          // Bus transaction active
    input  logic         bus_we,             // Bus write (write-back)
    input  logic [31:0]  bus_addr            // Bus address
);

    // =========================================================================
    // MESI State Encoding
    // =========================================================================
    typedef enum logic [1:0] {
        MESI_INVALID   = 2'b00,
        MESI_SHARED    = 2'b01,
        MESI_EXCLUSIVE = 2'b10,
        MESI_MODIFIED  = 2'b11
    } mesi_state_t;

    // =========================================================================
    // Snoop Controller FSM
    // =========================================================================
    typedef enum logic [2:0] {
        SNOOP_IDLE,
        SNOOP_CHECK,       // Check all L1 caches
        SNOOP_WAIT_DATA,   // Wait for dirty data from cache
        SNOOP_BROADCAST,   // Broadcast invalidate/downgrade
        SNOOP_RESPOND      // Send response
    } snoop_state_t;

    snoop_state_t state, next_state;

    // =========================================================================
    // Internal Registers
    // =========================================================================
    logic [31:0] pending_snoop_addr;
    logic        pending_snoop_read;
    logic        pending_snoop_write;
    logic        pending_snoop_rwitm;
    logic [$clog2(NUM_CACHES)-1:0] pending_snoop_requester_id;
    
    // Snoop results (latched during CHECK state)
    logic [NUM_CACHES-1:0] latched_cache_hit;
    logic [NUM_CACHES-1:0] latched_cache_dirty;
    logic [NUM_CACHES-1:0] latched_cache_exclusive;
    logic [NUM_CACHES-1:0] latched_cache_shared;
    logic latched_shared_copy_exists;  // Stable version for peer caches to sample
    
    // Request tracking (to prevent re-processing same request)
    logic [31:0] prev_snoop_addr;      // Last processed address
    logic        prev_snoop_req;       // Previous snoop_req value
    
    // Snoop results (combinational, updated every cycle)
    logic [NUM_CACHES-1:0] cache_hit;
    logic [NUM_CACHES-1:0] cache_dirty;
    logic [NUM_CACHES-1:0] cache_exclusive;
    logic [NUM_CACHES-1:0] cache_shared;
    
    // Which cache has dirty data (priority encoder)
    logic [$clog2(NUM_CACHES)-1:0] dirty_cache_id;
    logic has_dirty_data;
    logic has_shared_data;
    logic has_exclusive_data;

    // =========================================================================
    // Snoop Result Aggregation
    // =========================================================================
    
    // Check each peer cache for hits and states
    always_comb begin
        for (int i = 0; i < NUM_CACHES; i++) begin
            cache_hit[i]       = cache_has_line[i];
            cache_dirty[i]     = cache_has_line[i] && (cache_mesi_state[i] == MESI_MODIFIED);
            cache_exclusive[i] = cache_has_line[i] && (cache_mesi_state[i] == MESI_EXCLUSIVE);
            cache_shared[i]    = cache_has_line[i] && (cache_mesi_state[i] == MESI_SHARED);
        end
    end
    
    // Aggregate results (use latched values after CHECK state)
    logic [NUM_CACHES-1:0] active_cache_dirty;
    logic [NUM_CACHES-1:0] active_cache_exclusive;
    
    assign active_cache_dirty = (state == SNOOP_CHECK) ? cache_dirty : latched_cache_dirty;
    assign active_cache_exclusive = (state == SNOOP_CHECK) ? cache_exclusive : latched_cache_exclusive;
    
    // Exclude requester from dirty check (requester won't write back during its own request)
    logic [NUM_CACHES-1:0] requester_mask;
    logic [NUM_CACHES-1:0] dirty_excluding_requester;
    
    assign requester_mask = (1 << pending_snoop_requester_id);
    assign dirty_excluding_requester = active_cache_dirty & ~requester_mask;
    assign has_dirty_data     = |dirty_excluding_requester;  // Only check other caches
    assign has_shared_data    = |latched_cache_shared;  // Always use latched
    assign has_exclusive_data = |active_cache_exclusive;
    assign shared_copy_exists = latched_shared_copy_exists;  // Use stable latched version
    
    // Priority encoder: find first cache with dirty data (exclude requester)
    always_comb begin
        dirty_cache_id = '0;
        for (int i = NUM_CACHES-1; i >= 0; i--) begin
            if (dirty_excluding_requester[i]) begin
                dirty_cache_id = i[$clog2(NUM_CACHES)-1:0];
            end
        end
    end

    // =========================================================================
    // Snoop Response Logic
    // =========================================================================
    
    // Drive snoop_dirty continuously so L2 can make early decisions
    assign snoop_dirty = has_dirty_data;
    
    // Stall requesting CCL while dirty intervention is in progress
    assign snoop_dirty_stall = (state == SNOOP_CHECK && has_dirty_data) || (state == SNOOP_WAIT_DATA);
    
    always_comb begin
        snoop_resp_valid = 1'b0;
        snoop_hit        = 1'b0;
        snoop_data       = '0;
        
        case (state)
            SNOOP_RESPOND: begin
                snoop_resp_valid = 1'b1;
                snoop_hit        = |latched_cache_hit;  // Use latched results
                
                // Provide data from dirty cache if available
                if (has_dirty_data) begin
                    snoop_data = cache_snoop_data[dirty_cache_id];
                end
            end
        endcase
    end

    // =========================================================================
    // Peer Cache Control Signals
    // =========================================================================
    
    always_comb begin
        cache_snoop_req     = '0;
        cache_snoop_addr    = pending_snoop_addr;
        cache_invalidate    = '0;
        cache_downgrade     = '0;
        cache_writeback     = '0;
        cache_coherence_addr = pending_snoop_addr;
        
        case (state)
            SNOOP_CHECK: begin
                // Request all peer caches to check for the address
                cache_snoop_req  = '1;
                cache_snoop_addr = pending_snoop_addr;
            end
            
            SNOOP_WAIT_DATA: begin
                // Request write-back from dirty cache (EXCEPT requester)
                // The dirty cache will assert mem_valid + mem_we to write back
                cache_writeback = latched_cache_dirty;
                cache_writeback[pending_snoop_requester_id] = 1'b0;  // Don't request writeback from requester
`ifdef DEBUG_VERBOSE
                $display("[SNOOP @%0t] TRIGGERING WRITEBACK: addr=%h, dirty_caches=%b, requester=%0d, targets=%b", 
                         $time, pending_snoop_addr, latched_cache_dirty, pending_snoop_requester_id, cache_writeback);
`endif
            end
            
            SNOOP_BROADCAST: begin
                if (pending_snoop_write || pending_snoop_rwitm) begin
                    // Other cache is writing - invalidate all copies EXCEPT the requester
                    cache_invalidate = latched_cache_hit;
                    cache_invalidate[pending_snoop_requester_id] = 1'b0;  // Don't invalidate requester
`ifdef DEBUG_VERBOSE
                    $display("[SNOOP @%0t] BROADCAST INVALIDATE: addr=%h, all_targets=%b, requester=%0d, targets=%b", 
                             $time, pending_snoop_addr, latched_cache_hit, pending_snoop_requester_id, cache_invalidate);
`endif
                    
                end else if (pending_snoop_read) begin
                    // Other cache is reading
                    // Downgrade Modified/Exclusive -> Shared (EXCEPT requester who is reading)
                    cache_downgrade = latched_cache_dirty | latched_cache_exclusive;
                    cache_downgrade[pending_snoop_requester_id] = 1'b0;  // Don't downgrade requester
`ifdef DEBUG_VERBOSE
                    $display("[SNOOP @%0t] BROADCAST DOWNGRADE: addr=%h, all_targets=%b, requester=%0d, targets=%b (dirty=%b, excl=%b)", 
                             $time, pending_snoop_addr, latched_cache_dirty | latched_cache_exclusive, 
                             pending_snoop_requester_id, cache_downgrade, latched_cache_dirty, latched_cache_exclusive);
`endif
                end
            end
        endcase
    end

    // =========================================================================
    // FSM - State Register
    // =========================================================================
    
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state                <= SNOOP_IDLE;
            pending_snoop_addr   <= 32'b0;
            pending_snoop_read   <= 1'b0;
            pending_snoop_write  <= 1'b0;
            pending_snoop_rwitm  <= 1'b0;
            pending_snoop_requester_id <= '0;
            latched_cache_hit    <= '0;
            latched_cache_dirty  <= '0;
            latched_cache_exclusive <= '0;
            latched_cache_shared <= '0;
            latched_shared_copy_exists <= 1'b0;
            prev_snoop_addr      <= 32'hFFFFFFFF;  // Invalid address
            prev_snoop_req       <= 1'b0;
        end else begin
            // Debug snoop_req monitoring
            if (snoop_req) begin
`ifdef DEBUG_VERBOSE
                $display("[SNOOP @%0t] snoop_req=%b, state=%s, addr=%h, read=%b, write=%b, rwitm=%b, prev_addr=%h", 
                         $time, snoop_req, state.name(), snoop_addr, snoop_read, snoop_write, snoop_rwitm, prev_snoop_addr);
`endif
            end
            
            state <= next_state;
            
            // Track previous snoop_req to detect edges
            prev_snoop_req <= snoop_req;
            
            // Track processed address when starting new snoop
            if (state == SNOOP_IDLE && next_state == SNOOP_CHECK) begin
                prev_snoop_addr <= snoop_addr;
`ifdef DEBUG_VERBOSE
                $display("[SNOOP @%0t] Starting new snoop, storing addr=%h", $time, snoop_addr);
`endif
            end else if (!snoop_req) begin
                // Reset tracked address when snoop_req deasserts
                prev_snoop_addr <= 32'hFFFFFFFF;
            end
            
            // Latch snoop results during CHECK state
            if (state == SNOOP_CHECK) begin
                latched_cache_hit       <= cache_hit;
                latched_cache_dirty     <= cache_dirty;
                latched_cache_exclusive <= cache_exclusive;
                latched_cache_shared    <= cache_shared;
                latched_shared_copy_exists <= |cache_hit;  // Latch stable version
                if (|cache_hit) begin
`ifdef DEBUG_VERBOSE
                    $display("[SNOOP @%0t] LATCHING shared_copy_exists=1 (cache_hit=%b)", $time, cache_hit);
`endif
                end
            end
            
            // Clear latched shared_copy_exists when snoop request is fully de-asserted
            // Keep it asserted until requester completes allocation and samples the signal
            if (!snoop_req && latched_shared_copy_exists) begin
`ifdef DEBUG_VERBOSE
                $display("[SNOOP @%0t] CLEARING shared_copy_exists=0 (snoop_req deasserted)", $time);
`endif
                latched_shared_copy_exists <= 1'b0;
            end
            
            // Debug state transitions
            if (state != next_state) begin
`ifdef DEBUG_VERBOSE
                $display("[SNOOP @%0t] STATE: %s -> %s", $time, state.name(), next_state.name());
`endif
            end
            
            // Capture snoop request
            if (snoop_req && state == SNOOP_IDLE) begin
                pending_snoop_addr  <= snoop_addr;
                pending_snoop_read  <= snoop_read;
                pending_snoop_write <= snoop_write;
                pending_snoop_rwitm <= snoop_rwitm;
                pending_snoop_requester_id <= snoop_requester_id;
`ifdef DEBUG_VERBOSE
                $display("[SNOOP @%0t] REQUEST: addr=%h, read=%b, write=%b, rwitm=%b, requester=%0d", 
                         $time, snoop_addr, snoop_read, snoop_write, snoop_rwitm, snoop_requester_id);
`endif
            end
        end
    end

    // =========================================================================
    // FSM - Next State Logic
    // =========================================================================
    
    always_comb begin
        next_state = state;
        
        case (state)
            SNOOP_IDLE: begin
                // Only process if address is new (different from last) or snoop_req rising edge
                if (snoop_req && ((snoop_addr != prev_snoop_addr) || !prev_snoop_req)) begin
                    next_state = SNOOP_CHECK;
                end
            end
            
            SNOOP_CHECK: begin
                // One cycle to check all L1 caches
`ifdef DEBUG_VERBOSE
                $display("[SNOOP @%0t] CHECK: addr=%h, hits=%b, dirty=%b, excl=%b, shared=%b, will_set_shared_exists=%b", 
                         $time, pending_snoop_addr, cache_hit, cache_dirty, cache_exclusive, cache_shared, |cache_hit);
`endif
                if (has_dirty_data) begin
                    next_state = SNOOP_WAIT_DATA;
                end else begin
                    next_state = SNOOP_BROADCAST;
                end
            end
            
            SNOOP_WAIT_DATA: begin
                // Wait for dirty cache to write back to bus
                // Monitor bus_valid && bus_we && address match
                if (bus_valid && bus_we && (bus_addr[31:4] == pending_snoop_addr[31:4])) begin
`ifdef DEBUG_VERBOSE
                    $display("[SNOOP @%0t] WRITEBACK DETECTED on bus: addr=%h", $time, bus_addr);
`endif
                    next_state = SNOOP_BROADCAST;
                end
                // Else stay in WAIT_DATA until write-back appears
            end
            
            SNOOP_BROADCAST: begin
                // Broadcast coherence actions
                next_state = SNOOP_RESPOND;
            end
            
            SNOOP_RESPOND: begin
                // Send response
                next_state = SNOOP_IDLE;
            end
            
            default: begin
                next_state = SNOOP_IDLE;
            end
        endcase
    end

endmodule


// ============================================================================
// Usage Notes
// ============================================================================
//
// 1. Generic Design:
//    - NUM_CACHES: Number of peer caches to monitor (e.g., 2-4 L1s or 2 L2s)
//    - DATA_WIDTH: Always 32 bits (single word from blocking bus protocol)
//
// 2. L1 Snoop Controller Example (multiple L1s under one L2):
//      snoop_controller #(
//          .NUM_CACHES(2),      // 2 L1 caches
//          .DATA_WIDTH(32)      // 32-bit word transfers
//      ) l1_snoop (
//          .cache_snoop_req(l1_snoop_req),
//          .cache_has_line(l1_has_line),
//          .cache_mesi_state(l1_mesi_state),
//          .cache_snoop_data(l1_snoop_data),    // 32-bit data per L1
//          .cache_invalidate(l1_invalidate),
//          .cache_downgrade(l1_downgrade),
//          ...
//      );
//
// 3. L2 Snoop Controller Example (multiple L2s in multi-complex system):
//      snoop_controller #(
//          .NUM_CACHES(2),      // 2 L2 caches (one per complex)
//          .DATA_WIDTH(32)      // 32-bit word transfers (same as L1)
//      ) l2_snoop (
//          .cache_snoop_req(l2_snoop_req),
//          .cache_has_line(l2_has_line),
//          .cache_mesi_state(l2_mesi_state),
//          .cache_snoop_data(l2_snoop_data),    // 32-bit data per L2
//          .cache_invalidate(l2_invalidate),
//          .cache_downgrade(l2_downgrade),
//          ...
//      );
//
// 4. Snoop Operations:
//    - snoop_read: Another cache is reading
//      * If any peer has Modified: writeback + downgrade to Shared
//      * If any peer has Exclusive: downgrade to Shared
//      * Response: hit=1, provide data if dirty
//
//    - snoop_write: Another cache is writing
//      * Invalidate all peer cache copies
//      * If any peer has Modified: writeback first
//      * Response: hit=1, dirty=1 if writeback needed
//
//    - snoop_rwitm (Read-With-Intent-To-Modify): Another cache wants exclusive
//      * Same as snoop_write - invalidate all copies
//
// 5. Cache-to-Cache Transfer:
//    - If snoop finds dirty data, it provides it directly
//    - Requesting cache can get data without going to next level
//
// 6. Multi-Cache Coordination:
//    - Broadcasts invalidate/downgrade to all relevant peer caches
//    - Caches must implement handlers for these coherence signals
//
// ============================================================================
