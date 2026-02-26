// ============================================================================
// L2 Cache Controller with Integrated MESI Protocol Logic
// ============================================================================
// This module combines the cache FSM with MESI state transition logic.
// It handles:
// - Cache hit/miss detection and response (4-way set associative)
// - Memory transactions via blocking 32-bit bus
// - MESI state transitions based on operations and snoop results
// - LRU-based way selection
// ============================================================================

module cachel2_controller (
    input  logic        clk,
    input  logic        reset,

    // CPU side
    input  logic        cpu_req,
    input  logic        cpu_we,
    input  logic        cpu_rwitm,      // CPU Read-With-Intent-To-Modify
    input  logic [31:0] cpu_addr,

    // Datapath status
    input  logic        hit,
    input  logic [1:0]  mesi_state,   // MESI state of target way
    input  logic [19:0] evict_tag,
    input  logic [7:0]  index,
    input  logic [127:0] evict_block,
    input  logic [1:0]  hit_way,
    input  logic [1:0]  lru_way,

    // Memory side (blocking bus)
    input  logic        mem_ready,
    output logic        mem_valid,
    output logic        mem_we,
    output logic        mem_rwitm,         // Read-With-Intent-To-Modify (write miss)
    output logic [31:0] mem_addr,
    input  logic [31:0] mem_rdata,
    output logic [127:0] refill_data,

    // Datapath control
    output logic        write_enable_word,
    output logic        write_enable_line,
    output logic [1:0]  target_way,
    output logic        use_wb_addr,        // Use write-back address for datapath
    output logic [31:0] wb_addr_out,        // Write-back address (for datapath during RECEIVE_WB)
    
    // MESI state control (to datapath)
    output logic [1:0]  mesi_state_next,   // Next MESI state
    output logic        mesi_update,       // Update MESI state
    output logic [31:0] mesi_update_addr,  // Address for MESI update
    
    // Snoop information (from snoop controller)
    input  logic        shared_copy_exists, // Another L2 has this line
    input  logic        l1_has_dirty_data,  // L1 has Modified copy (don't use stale L2 data)
    
    // Cross-CCL dirty intervention
    input  logic        coherence_writeback, // External writeback request (cross-CCL)
    input  logic        snoop_dirty_stall,   // Fill stall: snoop detected dirty peer
    input  logic [31:0] coherence_addr,      // Address for cross-CCL coherence writeback
    input  logic        l1_dirty_pending_wb, // L1 has dirty data pending WB for coherence addr

    // CPU response
    output logic        ready,              // Registered ready (for most states)
    output logic        ready_comb,         // Combinational ready (for ALLOC_DATA pass-through)
    output logic        wait_for_peer_wb,   // Signal L1 to release bus (waiting for peer WB)
    
    // Pass-through control for refill data
    output logic        use_refill_data,    // Signal to bypass cache and use mem_rdata
    output logic        use_cache_forward,  // Signal to forward from cache using word_count
    output logic [1:0]  forward_word_index, // Which word from refill_buffer to forward
    
    // Latched address (for stable datapath addressing after request acceptance)
    output logic [31:0] latched_addr_out,   // Latched address for hit detection
    output logic        addr_latched        // 1 = use latched_addr for datapath
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
    // FSM States
    // =========================================================================
    typedef enum logic [3:0] {
        IDLE,
        COMPARE_TAG,
        WB_ADDR,         // Write-back: send address to memory
        WB_DATA,         // Write-back: send data words to memory
        WB_DONE,         // Write-back: idle cycle 1 (de-assert mem_valid)
        WB_DONE2,        // Write-back: idle cycle 2 (flush stale arbiter mem_ready)
        ALLOC_ADDR,      // Allocate: send address to memory
        ALLOC_DATA,      // Allocate: receive data words from memory
        FORWARD_DATA,    // Forward buffered data to requesting L1
        CACHE_FORWARD,   // Forward data from cache on read hit
        RECEIVE_WB       // Receive write-back data from L1
    } state_t;

    state_t state, next_state;

    // =========================================================================
    // Internal Registers & Signals
    // =========================================================================
    
    // Registered ready output to break combinational loops
    logic ready_next;
    logic wait_for_peer_wb_next;
    
    // Request latch to prevent re-triggering
    logic prev_cpu_req;  // Previous cycle's cpu_req for edge detection
    logic [31:0] prev_cpu_addr;  // Previous cycle's cpu_addr for address change detection
    logic req_latched, req_latched_next;
    logic [31:0] latched_addr, latched_addr_next;  // Latched address
    logic latched_we, latched_we_next;              // Latched write enable
    logic latched_rwitm, latched_rwitm_next;        // Latched RWITM
    logic [1:0] latched_way, latched_way_next;      // Latched hit way for write-backs
    logic [31:0] wb_addr, wb_addr_next;             // Write-back address (saved during RECEIVE_WB)
    
    // Coherence writeback tracking (cross-CCL dirty intervention)
    logic coherence_wb, coherence_wb_next;
    logic coherence_wb_serviced, coherence_wb_serviced_next;  // Prevents re-trigger while signal stays high
    
    // Word transfer counter (0-5: allows distinguishing skip from word3)
    logic [2:0] word_count, word_count_next;
    
    // Temporary buffer for refill data
    logic [127:0] refill_buffer, refill_buffer_next;

    // Block-aligned addresses (use latched address)
    logic [31:0] new_block_addr;
    logic [31:0] evict_block_addr;
    
    // MESI-based checks (replacing valid/dirty)
    logic evict_valid;   // Line is valid (not Invalid)
    logic evict_dirty;   // Line is dirty (Modified)

    assign new_block_addr   = {latched_addr[31:4], 4'b0000};
    assign evict_block_addr = {evict_tag, index, 4'b0000};
    assign evict_valid      = (mesi_state != MESI_INVALID);
    assign evict_dirty      = (mesi_state == MESI_MODIFIED);

    // Target way selection: use hit_way on hit, lru_way on miss
    logic [1:0] selected_way;
    assign selected_way = hit ? hit_way : lru_way;
    
    // Output refill data from buffer
    assign refill_data = refill_buffer;
    
    // Output write-back address
    assign wb_addr_out = wb_addr;
    
    // Output latched address for stable hit detection
    assign latched_addr_out = latched_addr;
    assign addr_latched     = req_latched;

    // =========================================================================
    // MESI State Transition Logic
    // =========================================================================
    always_comb begin
        mesi_state_next  = mesi_state;  // Default: no change
        mesi_update      = 1'b0;
        mesi_update_addr = latched_addr;  // Use latched address
        
        case (state)
            COMPARE_TAG: begin
                if (hit && latched_rwitm && !l1_has_dirty_data) begin
                    // RWITM (L1 write miss): INVALIDATE L2's copy
                    // Only fire when dirty peer has written back (l1_has_dirty_data=0)
                    // L1 will get Modified copy, L2 becomes Invalid
                    mesi_update = 1'b1;
                    mesi_state_next = MESI_INVALID;
                end
                // Read hit, write-back, or RWITM-with-dirty-peer: no state change needed here
            end
            
            ALLOC_DATA: begin
                // After allocation, set initial state (when all words received and registered)
                if (word_count == 3'd4 && (next_state == FORWARD_DATA || next_state == IDLE)) begin
                    mesi_update = 1'b1;
                    if (latched_rwitm) begin
                        // RWITM (write miss): allocate as Modified
                        mesi_state_next = MESI_MODIFIED;
                    end else begin
                        // Read miss: Exclusive if no other copies, Shared otherwise
                        mesi_state_next = shared_copy_exists ? MESI_SHARED : MESI_EXCLUSIVE;
                    end
                end
            end
            
            CACHE_FORWARD: begin
                // When forwarding cached data to another L1, may need to downgrade
                // If currently Exclusive and snoop indicates other L1s have it, downgrade to Shared
                if (mesi_state == MESI_EXCLUSIVE && shared_copy_exists) begin
                    mesi_update = 1'b1;
                    mesi_state_next = MESI_SHARED;
                end
                // If already Shared or Modified, no state change needed
            end
            
            RECEIVE_WB: begin
                // L1 writing back dirty data - mark as Modified in L2
                // Use wb_addr (the original write-back address), not latched_addr (which may be ALLOC addr)
                if (word_count == 3'd7 && (next_state == IDLE || next_state == COMPARE_TAG || next_state == WB_ADDR)) begin
                    mesi_update = 1'b1;
                    mesi_state_next = MESI_MODIFIED;  // Written-back data is dirty
                    mesi_update_addr = wb_addr;  // Use saved write-back address
                end
            end
            
            WB_ADDR: begin
                // When evicting, invalidate the old line; coherence WB → Shared
                mesi_update      = 1'b1;
                mesi_state_next  = coherence_wb ? MESI_SHARED : MESI_INVALID;
                mesi_update_addr = evict_block_addr;
            end
        endcase
    end

    // Expose word index for forwarding
    assign forward_word_index = word_count;
    
    // =========================================================================
    // Output Logic
    // =========================================================================
    always_comb begin
        // Defaults
        mem_valid         = 1'b0;
        mem_we            = 1'b0;
        mem_rwitm         = 1'b0;
        mem_addr          = 32'b0;
        write_enable_word = 1'b0;
        write_enable_line = 1'b0;
        use_refill_data   = 1'b0;
        use_cache_forward = 1'b0;
        use_wb_addr       = 1'b0;
        // ready is now a registered output, driven from next-state logic
        target_way        = selected_way;
        
        // Combinational ready: 
        // - During FORWARD_DATA or CACHE_FORWARD, generate ready pulses for each word
        // - During IDLE, immediately clear ready if new request detected
        // - Otherwise, use registered ready
        if (state == FORWARD_DATA || state == CACHE_FORWARD) begin
            ready_comb = 1'b1;  // Generate ready pulse every cycle during forwarding
        end else if (state == IDLE) begin
            // Clear ready immediately when new request is detected
            if ((cpu_req && !prev_cpu_req) || 
                (cpu_req && prev_cpu_req && (cpu_addr != prev_cpu_addr) && !cpu_we)) begin
                ready_comb = 1'b0;  // New request - clear ready immediately
            end else begin
                ready_comb = ready;  // Use registered ready
            end
        end else begin
            ready_comb = ready;  // Use registered ready for other states
        end

        case (state)

            IDLE: begin
                // ready_next is computed in the next-state logic block
            end

            COMPARE_TAG: begin
                if (hit) begin
                    target_way = hit_way;

                    if (latched_we) begin
                        write_enable_word = 1'b1;
                        // MESI state transition handled above
                    end
                end
                // On miss, transition happens in next-state logic
            end

            WB_ADDR: begin
                // Send write-back address
                target_way = coherence_wb ? latched_way : lru_way;
                use_wb_addr = coherence_wb;  // Use wb_addr for coherence WB addressing
                mem_valid = 1'b1;
                mem_we    = 1'b1;
                mem_addr  = evict_block_addr;
            end

            WB_DATA: begin
                // Send write-back data words sequentially
                target_way = coherence_wb ? latched_way : lru_way;
                use_wb_addr = coherence_wb;  // Use wb_addr for coherence WB addressing
                mem_valid = 1'b1;
                mem_we    = 1'b1;
                // Send current word based on counter
                case (word_count)
                    2'b00: mem_addr = evict_block[31:0];
                    2'b01: mem_addr = evict_block[63:32];
                    2'b10: mem_addr = evict_block[95:64];
                    2'b11: mem_addr = evict_block[127:96];
                endcase
            end

            ALLOC_ADDR: begin
                // Send allocate address
                target_way = lru_way;
                if (snoop_dirty_stall) begin
                    mem_valid = 1'b0;  // Release bus: peer CCL writing back dirty data
                end else begin
                    mem_valid = 1'b1;
                end
                mem_we    = 1'b0;
                mem_rwitm = latched_rwitm;  // Propagate RWITM from L1 to memory bus
                mem_addr  = new_block_addr;
            end

            ALLOC_DATA: begin
                // Receive allocate data words sequentially
                target_way = lru_way;
                mem_valid = 1'b1;  // Keep valid high during data transfer (like L1)
                mem_we    = 1'b0;
                mem_rwitm = latched_rwitm;  // Maintain RWITM during data transfer
                mem_addr  = new_block_addr;  // Keep driving address (not don't care!)
                use_refill_data = 1'b1;  // Pass mem_rdata through to cpu_rdata
                
                // Write complete block when all words received AND registered (count=4)
                if (word_count == 3'd4 && (next_state == FORWARD_DATA || next_state == IDLE)) begin
                    write_enable_line = 1'b1;
                    // MESI state set in MESI transition logic
                    // ready_next set in next-state logic
                end
            end
            
            FORWARD_DATA: begin
                // Forward buffered refill data to L1
                // Provide words from refill_buffer sequentially with ready pulses
                use_refill_data = 1'b1;  // Use refill buffer, not cache read
            end
            
            CACHE_FORWARD: begin
                // Forward data from cache on read hit
                // Provide words from cache sequentially with ready pulses
                use_cache_forward = 1'b1;  // Use cache data with word_count indexing
                target_way = latched_way;  // Read from the way that hit
            end
            
            RECEIVE_WB: begin
                // Receiving write-back data from L1
                // Write to cache when all 4 words received and registered (count=7)
                // Use latched_way (the way that hit for the write-back address)
                target_way = latched_way;
                if (word_count == 3'd7 && (next_state == IDLE || next_state == COMPARE_TAG || next_state == WB_ADDR)) begin
                    // Write complete block to cache
                    use_wb_addr = 1'b1;  // Override datapath address with saved write-back address ONLY during write
                    write_enable_line = 1'b1;
                    use_refill_data = 1'b1;  // Use refill buffer data
                    // MESI state should be set to MODIFIED (updated in MESI logic)
                end
            end
        endcase
    end

    // =========================================================================
    // State Register & Word Counter
    // =========================================================================
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state         <= IDLE;
            word_count    <= 3'd0;
            refill_buffer <= 128'b0;
            req_latched   <= 1'b0;
            latched_addr  <= 32'h0;
            latched_we    <= 1'b0;            latched_rwitm <= 1'b0;            latched_way   <= 2'b0;
            wb_addr       <= 32'h0;
            prev_cpu_req  <= 1'b0;
            prev_cpu_addr <= 32'h0;
            ready         <= 1'b0;
            wait_for_peer_wb <= 1'b0;
            coherence_wb  <= 1'b0;
            coherence_wb_serviced <= 1'b0;
        end else begin
            state         <= next_state;
            word_count    <= word_count_next;
            refill_buffer <= refill_buffer_next;
            req_latched   <= req_latched_next;
            latched_addr  <= latched_addr_next;
            latched_we    <= latched_we_next;
            latched_rwitm <= latched_rwitm_next;
            latched_way   <= latched_way_next;
            wb_addr       <= wb_addr_next;
            
            // Debug cpu_req changes
            if (prev_cpu_req != cpu_req) begin
`ifdef DEBUG_VERBOSE
                $display("[L2 @%0t] CPU_REQ CHANGE: %b -> %b", $time, prev_cpu_req, cpu_req);
`endif
            end
            prev_cpu_req  <= cpu_req;  // Track previous cpu_req
            prev_cpu_addr <= cpu_addr; // Track previous cpu_addr for address change detection
            ready         <= ready_next;  // Register ready output
            wait_for_peer_wb <= wait_for_peer_wb_next;  // Register wait signal
            coherence_wb  <= coherence_wb_next;  // Register coherence writeback flag
            coherence_wb_serviced <= coherence_wb_serviced_next;
            
            // Debug state transitions
            if (state != next_state) begin
`ifdef DEBUG_VERBOSE
                $display("[L2 @%0t] STATE CHANGE: %s -> %s (cpu_req=%b, prev_cpu_req=%b)", 
                         $time, state.name(), next_state.name(), cpu_req, prev_cpu_req);
`endif
            end
        end
    end

    // =========================================================================
    // Next-State Logic & Word Counter Update
    // =========================================================================
    always_comb begin
        next_state         = state;
        word_count_next    = word_count;
        refill_buffer_next = refill_buffer;
        req_latched_next   = req_latched;
        latched_addr_next  = latched_addr;
        latched_we_next    = latched_we;
        latched_rwitm_next = latched_rwitm;
        latched_way_next   = latched_way;  // Hold previous value by default
        wb_addr_next       = wb_addr;  // Hold previous value by default
        ready_next         = ready;  // Hold previous value by default (not 0!)
        wait_for_peer_wb_next = 1'b0;  // Default to not waiting
        coherence_wb_next  = coherence_wb;  // Hold previous value
        coherence_wb_serviced_next = coherence_wb_serviced;
        // Clear serviced flag once external coherence_writeback deasserts
        if (!coherence_writeback)
            coherence_wb_serviced_next = 1'b0;

        case (state)

            IDLE: begin
                // Hold ready high if we just completed a request and cpu_req is still high
                if (req_latched && cpu_req) begin
                    ready_next = 1'b1;
`ifdef DEBUG_VERBOSE
                    $display("[L2 @%0t] IDLE: Holding ready=1 (req_latched=%b, cpu_req=%b)", 
                             $time, req_latched, cpu_req);
`endif
                end else begin
                    // Clear ready when cpu_req is de-asserted
                    ready_next = 1'b0;
                end
                
                // Accept new request on:
                // 1. Rising edge of cpu_req, OR
                // 2. Address change while req is high AND it's a READ (not during write data phase)
                if ((cpu_req && !prev_cpu_req) || 
                    (cpu_req && prev_cpu_req && (cpu_addr != prev_cpu_addr) && !cpu_we)) begin
                    // New request detected (either rising edge or address change during read)
                    next_state        = COMPARE_TAG;
                    word_count_next   = 3'd0;
                    req_latched_next  = 1'b1;
                    latched_addr_next = cpu_addr;
                    latched_we_next   = cpu_we;
                    latched_rwitm_next = cpu_rwitm;
                    ready_next        = 1'b0;  // Clear ready on new request
                    if (!prev_cpu_req) begin
`ifdef DEBUG_VERBOSE
                        $display("[L2 @%0t] RISING EDGE DETECTED! cpu_req=%b, prev_cpu_req=%b, addr=%h", 
                                 $time, cpu_req, prev_cpu_req, cpu_addr);
`endif
                    end else begin
`ifdef DEBUG_VERBOSE
                        $display("[L2 @%0t] ADDRESS CHANGE DETECTED! addr=%h -> %h (cpu_req still high, READ)", 
                                 $time, prev_cpu_addr, cpu_addr);
`endif
                    end
`ifdef DEBUG_VERBOSE
                end else if (cpu_req) begin
                    $display("[L2 @%0t] IDLE with cpu_req=1 but no edge (prev_cpu_req=%b)", 
                             $time, prev_cpu_req);
`endif
                end else if (coherence_writeback && !coherence_wb_serviced && !req_latched) begin
                    // Cross-CCL dirty intervention: L2 has Modified data that must be
                    // written back to memory so the requesting CCL can fill cleanly.
                    // No L1 within this CCL is dirty — the L2 itself holds the dirty line.
                    // Guard: coherence_wb_serviced prevents re-trigger while signal stays high.
                    next_state        = COMPARE_TAG;
                    word_count_next   = 3'd0;
                    req_latched_next  = 1'b1;
                    latched_addr_next = coherence_addr;
                    latched_we_next   = 1'b0;
                    latched_rwitm_next = 1'b0;
                    coherence_wb_next = 1'b1;
                    ready_next        = 1'b0;
`ifdef DEBUG_VERBOSE
                    $display("[L2 @%0t] IDLE: coherence_writeback for addr=%h \u2014 starting autonomous WB", $time, coherence_addr);
`endif
                end
                
                // Note: Do NOT clear req_latched when cpu_req goes away
                // L2 must complete latched requests even if requestor releases bus
            end

            COMPARE_TAG: begin
`ifdef DEBUG_VERBOSE
                $display("[L2] COMPARE_TAG: addr=%h (latched=%h), hit=%b, req_latched=%b, we=%b, rwitm=%b, dirty_l1=%b, coh_wb=%b", 
                         cpu_addr, latched_addr, hit, req_latched, latched_we, latched_rwitm, l1_has_dirty_data, coherence_wb);
`endif
                
                // PRIORITY 0: Autonomous cross-CCL writeback (coherence_wb from IDLE)
                if (coherence_wb && !latched_we) begin
                    if (cpu_req && !prev_cpu_req && cpu_we) begin
                        // L1 dirty WB just arrived on internal bus — accept it first
                        // so L2 cache array gets the fresh dirty data before memory WB
                        next_state = RECEIVE_WB;
                        wb_addr_next = cpu_addr;
                        latched_way_next = hit_way;
                        word_count_next = 3'b000;
                        ready_next = 1'b0;
`ifdef DEBUG_VERBOSE
                        $display("[L2] Coherence WB: L1 dirty WB arrived addr=%h, accepting first", cpu_addr);
`endif
                    end else if (hit && !l1_dirty_pending_wb) begin
                        // No L1 dirty data — safe to WB L2's data to memory
                        next_state = WB_ADDR;
                        latched_way_next = hit_way;
                        wb_addr_next = latched_addr;  // Set wb_addr so dp_addr indexes correct set during WB
                        ready_next = 1'b0;
`ifdef DEBUG_VERBOSE
                        $display("[L2] Coherence WB: hit way=%0d, going to WB_ADDR for addr=%h", hit_way, latched_addr);
`endif
                    end else if (hit) begin
                        // L1 has dirty data — wait for L1 to write back first
                        next_state = COMPARE_TAG;
                        latched_way_next = hit_way;
                        ready_next = 1'b0;
`ifdef DEBUG_VERBOSE
                        $display("[L2] Coherence WB: waiting for L1 dirty WB, addr=%h", latched_addr);
`endif
                    end else begin
                        // Line no longer present (evicted/invalidated) — abort
                        next_state = IDLE;
                        coherence_wb_next = 1'b0;
                        req_latched_next = 1'b0;
                        ready_next = 1'b0;
`ifdef DEBUG_VERBOSE
                        $display("[L2] Coherence WB: miss \u2014 line already gone, aborting");
`endif
                    end
                end
                // PRIORITY 1: Detect incoming writeback while waiting for dirty peer
                // Accept writeback without forgetting pending read request
                else if (cpu_req && !prev_cpu_req && cpu_we && req_latched && !latched_we) begin
                    // NEW writeback arriving while we have pending READ - accept WB
                    // Don't overwrite latched read request - WB uses wb_addr directly
                    next_state = RECEIVE_WB;
                    wb_addr_next = cpu_addr;  // Save WB address separately
                    word_count_next = 3'b000;
                    ready_next = 1'b0;
                    // Keep req_latched=1 and latched_addr unchanged (pending read)
`ifdef DEBUG_VERBOSE
                    $display("[L2] COMPARE_TAG: Accepting WB addr=%h while READ pending for %h", cpu_addr, latched_addr);
`endif
                end
                // PRIORITY 2: Process latched request
                else if (hit) begin
                    // Check operation type
                    if (latched_rwitm && l1_has_dirty_data) begin
                        // RWITM hit BUT peer L1 has dirty copy - L2 data is stale!
                        // Must wait for peer to write back before forwarding
                        next_state = COMPARE_TAG;
                        latched_way_next = hit_way;    // Save way so RECEIVE_WB writes to correct location
                        ready_next = 1'b0;
                        wait_for_peer_wb_next = 1'b1;  // Signal pending L1 to release bus
`ifdef DEBUG_VERBOSE
                        $display("[L2] RWITM hit but dirty peer! Waiting for WB, addr=%h", latched_addr);
`endif
                    end else if (latched_rwitm) begin
                        // RWITM (read with intent to modify) - L1 doing write miss
                        // Forward all 4 words to L1 (like a read hit), then L2 becomes Invalid
                        // (MESI update to Invalid fires in MESI logic for COMPARE_TAG/rwitm)
                        next_state = CACHE_FORWARD;
                        latched_way_next = hit_way;
                        word_count_next = 3'b000;
                        ready_next = 1'b0;
`ifdef DEBUG_VERBOSE
                        $display("[L2] RWITM! Forwarding data then invalidating L2 copy, way=%d", hit_way);
`endif
                    end else if (latched_we) begin
                        // Write-back from L1 - accept the data
                        next_state = RECEIVE_WB;
                        latched_way_next = hit_way;
                        wb_addr_next = latched_addr;  // Save write-back address
                        word_count_next = 3'b000;
                        ready_next = 1'b0;
`ifdef DEBUG_VERBOSE
                        $display("[L2] Write-back hit! Accepting dirty data from L1, way=%d", hit_way);
`endif
                    end else if (l1_has_dirty_data) begin
                        // Read hit BUT peer L1 has dirty copy — L2 data is stale!
                        // Must wait for peer to write back before forwarding
                        next_state = COMPARE_TAG;
                        latched_way_next = hit_way;
                        ready_next = 1'b0;
                        wait_for_peer_wb_next = 1'b1;
`ifdef DEBUG_VERBOSE
                        $display("[L2] Read hit but dirty peer! Waiting for WB, addr=%h", latched_addr);
`endif
                    end else begin
                        // Read - forward data from cache (clean)
                        next_state = CACHE_FORWARD;
                        latched_way_next = hit_way;
                        word_count_next = 3'b000;
                        ready_next = 1'b0;
`ifdef DEBUG_VERBOSE
                        $display("[L2] Read Hit! Going to CACHE_FORWARD, way=%d", hit_way);
`endif
                    end
                end else if (evict_dirty && evict_valid) begin
                    next_state = WB_ADDR;
`ifdef DEBUG_VERBOSE
                    $display("[L2] Miss with dirty eviction");
`endif
                end else if (l1_has_dirty_data && !latched_we) begin
                    // Peer L1 has dirty copy - loop in COMPARE_TAG waiting for WB
                    next_state = COMPARE_TAG;
                    ready_next = 1'b0;
                    wait_for_peer_wb_next = 1'b1;  // Signal L1 to release bus
`ifdef DEBUG_VERBOSE
                    $display("[L2] Miss with dirty peer! Waiting in COMPARE_TAG for WB, addr=%h", latched_addr);
`endif
                end else if (latched_we) begin
                    // Write-back miss: L2's copy was previously invalidated (e.g. by RWITM),
                    // but L1 still has dirty data to write back - accept it into RECEIVE_WB.
                    // Use LRU way since there's no hit way.
                    next_state = RECEIVE_WB;
                    latched_way_next = lru_way;
                    wb_addr_next = latched_addr;
                    word_count_next = 3'b000;
                    ready_next = 1'b0;
`ifdef DEBUG_VERBOSE
                    $display("[L2] WB miss (L2 invalidated) - accepting WB for %h into way %0d", latched_addr, lru_way);
`endif
                end else begin
                    next_state = ALLOC_ADDR;
`ifdef DEBUG_VERBOSE
                    $display("[L2] Miss, going to ALLOC_ADDR for %h", latched_addr);
`endif
                end
            end

            WB_ADDR: begin
`ifdef DEBUG_VERBOSE
                $display("[L2 @%0t] WB_ADDR: mem_ready=%b, evict_addr=%h, coherence_wb=%b",
                         $time, mem_ready, evict_block_addr, coherence_wb);
`endif
                if (mem_ready) begin
                    next_state      = WB_DATA;
                    word_count_next = 3'd0;  // Start with word 0
                end
            end

            WB_DATA: begin
                if (mem_ready) begin
                    if (word_count == 2'b11) begin
                        // All 4 words sent — go to WB_DONE to drop mem_valid for one cycle
                        // before starting the fill (memory model resets on mem_valid=0)
                        next_state      = WB_DONE;
                        word_count_next = 2'b00;
                    end else begin
                        // More words to send
                        word_count_next = word_count + 2'b01;
                    end
                end
            end

            WB_DONE: begin
                // mem_valid is de-asserted this cycle (output logic default = 0)
                // Memory model resets its counter; need a 2nd idle cycle to flush
                // the stale registered mem_ready from the bus arbiter pipeline
                next_state = WB_DONE2;
            end

            WB_DONE2: begin
                // Second idle cycle: arbiter's registered mem_ready is now 0
                if (coherence_wb) begin
                    // Coherence writeback complete: return to IDLE (no new allocation)
                    next_state = IDLE;
                    coherence_wb_next = 1'b0;
                    req_latched_next = 1'b0;
                    ready_next = 1'b0;
                    coherence_wb_serviced_next = 1'b1;  // Prevent re-trigger until signal deasserts
                end else begin
                    // Normal eviction: proceed with fill allocation
                    next_state = ALLOC_ADDR;
                end
            end

            ALLOC_ADDR: begin
                if (mem_ready && !snoop_dirty_stall) begin
                    next_state      = ALLOC_DATA;
                    word_count_next = 2'b00;  // Start receiving word 0
                end
            end

            ALLOC_DATA: begin
                // Receive data from memory and buffer it
                // Don't signal ready to L1 yet - wait until all words received AND registered
                
                if (word_count == 3'd4) begin
                    // Refill buffer now has all 4 words - forward to L1 if still waiting
                    if (cpu_req) begin
                        // L1 still waiting - forward buffered data
                        next_state       = FORWARD_DATA;
                        word_count_next  = 2'b00;  // Start at word 0
                    end else begin
                        // No one waiting - go to IDLE
                        next_state       = IDLE;
                        req_latched_next = 1'b0;
                    end
                end else if (mem_ready) begin
                    // Store received word in buffer
                    case (word_count)
                        3'd0: refill_buffer_next[31:0]   = mem_rdata;
                        3'd1: refill_buffer_next[63:32]  = mem_rdata;
                        3'd2: refill_buffer_next[95:64]  = mem_rdata;
                        3'd3: refill_buffer_next[127:96] = mem_rdata;
                    endcase
                    
                    if (word_count == 3'd3) begin
                        // All 4 words received - wait one cycle for register update
                        word_count_next = 3'd4;
                    end else begin
                        // More words to receive (counts 0, 1, 2)
                        word_count_next = word_count + 3'd1;
                    end
                end
            end
            
            FORWARD_DATA: begin
                // Forward one word per cycle, cycling through refill buffer
                // Stay in this state until cpu_req drops (L1 completes)
                
                if (!cpu_req) begin
                    // L1 de-asserted request - done forwarding
                    next_state       = IDLE;
                    req_latched_next = 1'b0;
                    word_count_next  = 2'b00;
                end else begin
                    // Keep cycling through words while L1 is reading
                    // L1 will receive exactly 4 words due to arbiter delay
                    word_count_next = word_count + 2'b01;
                end
            end
            
            CACHE_FORWARD: begin
                // Forward data from cache on read hit
                // Provide sequential words like FORWARD_DATA
                
                if (!cpu_req) begin
                    // L1 de-asserted request - done forwarding
                    next_state       = IDLE;
                    req_latched_next = 1'b0;
                    word_count_next  = 2'b00;
                end else begin
                    // Keep cycling through words while L1 is reading
                    word_count_next = word_count + 2'b01;
                end
            end
            
            RECEIVE_WB: begin
                // Receive write-back data from L1 (4 words on cpu_addr bus)
                // L1 protocol: WB_ADDR -> (wait for ready) -> WB_DATA(word0,1,2,3) -> possibly ALLOC_ADDR
                // Skip 3 cycles before first data word appears
                // Count: 0(skip WB_ADDR) -> 1(skip) -> 2(skip) -> 3(word0) -> 4(word1) -> 5(word2) -> 6(word3) -> 7(write)
                //
                // IMPORTANT: word_count==7 is checked FIRST (before cpu_req) because all
                // 4 data words have already been captured. L1 may drop cpu_req (mem_valid)
                // in the same delta cycle that word_count reaches 7 — if the completion
                // logic were gated by cpu_req, the coherence writeback path would be lost
                // and the system would deadlock.
                
                if (word_count == 3'd7) begin
                    // All 4 words received — commit to exit path regardless of cpu_req
                    if (coherence_writeback) begin
                        // Cross-CCL dirty intervention: write dirty data to memory
                        next_state = WB_ADDR;
                        coherence_wb_next = 1'b1;
                        word_count_next = 3'd0;
                        ready_next = 1'b0;
`ifdef DEBUG_VERBOSE
                        $display("[L2 @%0t] RECEIVE_WB[7]: Coherence WB -> writing to memory, addr=%h", $time, wb_addr);
`endif
                    end else if (req_latched && !latched_we) begin
                        // We have a pending READ request - return to COMPARE_TAG to service it
                        // Don't re-latch, keep original request
                        next_state = COMPARE_TAG;
                        word_count_next = 3'd0;
                        ready_next = 1'b0;
`ifdef DEBUG_VERBOSE
                        $display("[L2 @%0t] RECEIVE_WB[7]: Complete, returning to service pending READ addr=%h", $time, latched_addr);
`endif
                    end else if (cpu_req && cpu_we) begin
                        // Still in write mode (WB only, no ALLOC following)
                        next_state = IDLE;
                        req_latched_next = 1'b0;
                        word_count_next = 3'd0;
                        ready_next = 1'b0;
`ifdef DEBUG_VERBOSE
                        $display("[L2 @%0t] RECEIVE_WB[7]: Complete (WB only)", $time);
`endif
                    end else if (cpu_req && !cpu_we) begin
                        // Read mode - L1 moved to ALLOC
                        next_state = COMPARE_TAG;
                        latched_addr_next = cpu_addr;
                        latched_we_next = cpu_we;
                        latched_rwitm_next = cpu_rwitm;
                        req_latched_next = 1'b1;
                        word_count_next = 3'd0;
                        ready_next = 1'b0;
`ifdef DEBUG_VERBOSE
                        $display("[L2 @%0t] RECEIVE_WB[7]: ->ALLOC addr=%h", $time, cpu_addr);
`endif
                    end else begin
                        // cpu_req dropped at completion — normal WB done
                        next_state = IDLE;
                        req_latched_next = 1'b0;
                        word_count_next = 3'd0;
                        ready_next = 1'b0;
`ifdef DEBUG_VERBOSE
                        $display("[L2 @%0t] RECEIVE_WB[7]: Complete (cpu_req dropped)", $time);
`endif
                    end
                end else if (cpu_req) begin
                    case (word_count)
                        3'd0: begin
                            // Skip WB_ADDR cycle
                            word_count_next = 3'd1;
                            ready_next = 1'b1;
`ifdef DEBUG_VERBOSE
                            $display("[L2 @%0t] RECEIVE_WB[0]: Skip WB_ADDR", $time);
`endif
                        end
                        
                        3'd1: begin
                            // Skip - L1 waiting for ready
                            word_count_next = 3'd2;
                            ready_next = 1'b1;
`ifdef DEBUG_VERBOSE
                            $display("[L2 @%0t] RECEIVE_WB[1]: Skip", $time);
`endif
                        end
                        
                        3'd2: begin
                            // Skip - L1 transitioning to WB_DATA
                            word_count_next = 3'd3;
                            ready_next = 1'b1;
`ifdef DEBUG_VERBOSE
                            $display("[L2 @%0t] RECEIVE_WB[2]: Skip transition", $time);
`endif
                        end
                        
                        3'd3: begin
                            // Capture word 0
                            refill_buffer_next[31:0] = cpu_addr;
                            word_count_next = 3'd4;
                            ready_next = 1'b1;
`ifdef DEBUG_VERBOSE
                            $display("[L2 @%0t] RECEIVE_WB[3]: Capture word0=%h", $time, cpu_addr);
`endif
                        end
                        
                        3'd4: begin
                            // Capture word 1
                            refill_buffer_next[63:32] = cpu_addr;
                            word_count_next = 3'd5;
                            ready_next = 1'b1;
`ifdef DEBUG_VERBOSE
                            $display("[L2 @%0t] RECEIVE_WB[4]: Capture word1=%h", $time, cpu_addr);
`endif
                        end
                        
                        3'd5: begin
                            // Capture word 2
                            refill_buffer_next[95:64] = cpu_addr;
                            word_count_next = 3'd6;
                            ready_next = 1'b1;
`ifdef DEBUG_VERBOSE
                            $display("[L2 @%0t] RECEIVE_WB[5]: Capture word2=%h", $time, cpu_addr);
`endif
                        end
                        
                        3'd6: begin
                            // Capture word 3
                            refill_buffer_next[127:96] = cpu_addr;
                            word_count_next = 3'd7;
                            ready_next = 1'b0;  // De-assert ready, no more data to send
`ifdef DEBUG_VERBOSE
                            $display("[L2 @%0t] RECEIVE_WB[6]: Capture word3=%h", $time, cpu_addr);
`endif
                        end
                        
                        default: begin
                            // Should not reach here
                            next_state = IDLE;
                            word_count_next = 3'd0;
                        end
                    endcase
                end else begin
                    // cpu_req dropped before all data received
                    next_state = IDLE;
                    req_latched_next = 1'b0;
                    word_count_next = 3'd0;
                    ready_next = 1'b0;
                end
            end

        endcase
    end

endmodule
