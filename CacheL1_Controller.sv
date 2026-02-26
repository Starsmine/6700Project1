// ============================================================================
// L1 Cache Controller with Integrated MESI Protocol Logic
// ============================================================================
// This module combines the cache FSM with MESI state transition logic.
// It handles:
// - Cache hit/miss detection and response
// - Memory transactions via blocking 32-bit bus
// - MESI state transitions based on operations and snoop results
// ============================================================================

module cachel1_controller (
    input  logic        clk,
    input  logic        reset,

    // CPU side
    input  logic        cpu_req,
    input  logic        cpu_we,
    input  logic [31:0] cpu_addr,

    // Datapath status
    input  logic        hit,
    input  logic [1:0]  mesi_state,   // Current MESI state
    input  logic [18:0] evict_tag,
    input  logic [8:0]  index,
    input  logic [127:0] evict_block,

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
    output logic        use_refill_for_cpu,  // Use refill buffer for CPU read data
    
    // MESI state control (to datapath)
    output logic [1:0]  mesi_state_next,   // Next MESI state
    output logic        mesi_update,       // Update MESI state
    output logic [31:0] mesi_update_addr,  // Address for MESI update
    
    // Snoop information (from snoop controller)
    input  logic        shared_copy_exists, // Another L1 has this line
    input  logic        wait_for_peer_wb,   // L2 waiting for peer WB (release bus)
    input  logic        writeback_req,      // Snoop controller requests write-back
    input  logic [31:0] writeback_addr,     // Address to write back
    
    // Bus snooping (to capture write-backs from peer caches)
    input  logic        bus_valid,          // Bus transaction active
    input  logic        bus_we,             // Bus write (write-back)
    input  logic [31:0] bus_addr,           // Bus address
    input  logic [31:0] bus_wdata,          // Bus write data

    // CPU response
    output logic        ready,
    output logic        miss   // High when completing a miss operation
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
    typedef enum logic [2:0] {
        IDLE,
        COMPARE_TAG,
        UPGRADE,         // Upgrade: send invalidate request for write to Shared line
        WB_ADDR,         // Write-back: send address
        WB_DATA,         // Write-back: send data words
        ALLOC_ADDR,      // Allocate: send address
        ALLOC_DATA       // Allocate: receive data words
    } state_t;

    state_t state, next_state;

    // =========================================================================
    // Internal Registers & Signals
    // =========================================================================
    
    // Registered ready output to break combinational loops
    logic ready_next;
    
    // Flag to write cache line in next cycle (after refill_buffer is registered)
    logic write_line_next_cycle, write_line_next_cycle_next;
    
    // Flag for first cycle of ALLOC_DATA (to hold mem_valid for L2 to latch)
    logic alloc_first_cycle, alloc_first_cycle_next;
    
    // Flag to distinguish snoop-triggered WB from eviction WB
    logic snoop_wb, snoop_wb_next;
    
    // Latched shared_copy_exists: sticky during allocation (cross-CCL signal is transient)
    logic latched_shared, latched_shared_next;
    
    // Request latch to prevent re-triggering
    logic prev_cpu_req;  // Previous cycle's cpu_req for edge detection
    logic req_latched, req_latched_next;
    logic [31:0] latched_addr, latched_addr_next;  // Latched address
    logic latched_we, latched_we_next;              // Latched write enable
    
    // Word transfer counter (0-3 for 4 words)
    logic [1:0] word_count, word_count_next;
    
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
    
    // Output refill data from buffer
    assign refill_data = refill_buffer;

    // =========================================================================
    // MESI State Transition Logic
    // =========================================================================
    always_comb begin
        mesi_state_next  = mesi_state;  // Default: no change
        mesi_update      = 1'b0;
        mesi_update_addr = latched_addr;  // Use latched address
        
        case (state)
            COMPARE_TAG: begin
                if (hit && latched_we) begin
                    // Write Hit: Upgrade to Modified
                    mesi_update = 1'b1;
                    case (mesi_state)
                        MESI_EXCLUSIVE: mesi_state_next = MESI_MODIFIED;  // Silent upgrade
                        MESI_SHARED:    mesi_state_next = MESI_MODIFIED;  // Upgrade after invalidation
                        MESI_MODIFIED:  mesi_state_next = MESI_MODIFIED;  // Already Modified
                        default:        mesi_state_next = MESI_INVALID;
                    endcase
                end
                // Read hit: no state change needed
            end
            
            UPGRADE: begin
                // After invalidation completes, upgrade to Modified
                // MESI state was already updated in COMPARE_TAG
                // This state just waits for snoop invalidation to complete
            end
            
            ALLOC_DATA: begin
                // After allocation, set initial state
                if (word_count == 2'b11 && mem_ready) begin
                    mesi_update = 1'b1;
                    if (latched_we) begin
                        // Write miss: allocate as Modified
                        mesi_state_next = MESI_MODIFIED;
                    end else begin
                        // Read miss: Exclusive if no other copies, Shared otherwise
                        // Use latched value since cross-CCL shared_copy_exists is transient
                        mesi_state_next = (shared_copy_exists || latched_shared) ? MESI_SHARED : MESI_EXCLUSIVE;
                    end
                end
            end
            
            WB_ADDR: begin
                // When evicting, invalidate the old line
                // For snoop-triggered intervention WB, downgrade to Shared (keep the line)
                mesi_update      = 1'b1;
                mesi_state_next  = snoop_wb ? MESI_SHARED : MESI_INVALID;
                mesi_update_addr = evict_block_addr;
            end
        endcase
    end

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
        use_refill_for_cpu = 1'b0;  // Default: use cache read
        miss              = 1'b0;    // Default: not a miss completion
        // ready is now registered, computed in next-state logic
        
        // Write cache line if flagged from previous cycle
        if (write_line_next_cycle) begin
            write_enable_line = 1'b1;
            use_refill_for_cpu = 1'b1;  // Provide refill data while cache is being written
            miss = 1'b1;                 // This is a miss completion
            // For write misses (RWITM), also write the CPU's word so it "wins" over refill data
            if (latched_we) begin
                write_enable_word = 1'b1;
            end
        end

        case (state)

            IDLE: begin
                // ready_next computed in next-state logic
            end

            COMPARE_TAG: begin
                if (hit && evict_valid) begin
                    if (latched_we) begin
                        write_enable_word = 1'b1;
                        // MESI state transition handled above
                    end
                end
                // On miss, transition happens in next-state logic
            end
            
            UPGRADE: begin
                // Request invalidation of peer caches
                mem_valid = 1'b1;
                mem_we    = 1'b0;  // Not a write, just invalidate request
                mem_rwitm = 1'b1;  // Signal write intent to trigger snoop invalidation
                mem_addr  = new_block_addr;  // Block-aligned address
                write_enable_word = 1'b1;  // Write the word to our cache
`ifdef DEBUG_VERBOSE
                $display("[L1 @%0t] UPGRADE: Sending invalidate request for addr=%h", $time, new_block_addr);
`endif
            end

            WB_ADDR: begin
                // Send write-back address
                // Keep mem_valid high through WB_ADDR and WB_DATA to prevent glitches
                mem_valid = 1'b1;
                mem_we    = 1'b1;
                mem_addr  = evict_block_addr;
            end

            WB_DATA: begin
                // Send write-back data words sequentially
                // Keep mem_valid high until entire write-back is complete
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
                // Keep mem_valid high through ALLOC_ADDR and ALLOC_DATA to prevent glitches
                mem_valid = 1'b1;  // Stay high until we're done with entire transaction
                mem_we    = 1'b0;
                mem_rwitm = latched_we;  // Signal RWITM for write misses
                mem_addr  = new_block_addr;
            end

            ALLOC_DATA: begin
                // Receive allocate data words sequentially
                // Release bus only when L2 signals it's waiting for peer writeback
                if (wait_for_peer_wb) begin
                    mem_valid = 1'b0;  // L2 waiting for peer WB, release bus
                end else begin
                    mem_valid = 1'b1;  // Keep requesting normally
                end
                mem_we    = 1'b0;
                mem_rwitm = latched_we;  // Maintain RWITM
                mem_addr  = new_block_addr;  // Keep driving address
                
`ifdef DEBUG_VERBOSE
                $display("[L1 @%0t] ALLOC_DATA: mem_valid=%b, mem_ready=%b, word_count=%d, next_state=%s", 
                         $time, mem_valid, mem_ready, word_count, next_state.name());
`endif
                
                // Note: write_enable_line will be asserted next cycle via write_line_next_cycle flag
            end
        endcase
    end

    // =========================================================================
    // State Register & Word Counter
    // =========================================================================
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            state         <= IDLE;
            word_count    <= 2'b00;
            refill_buffer <= 128'b0;
            req_latched   <= 1'b0;
            latched_addr  <= 32'h0;
            latched_we    <= 1'b0;
            prev_cpu_req  <= 1'b0;
            ready         <= 1'b0;
            write_line_next_cycle <= 1'b0;
            snoop_wb      <= 1'b0;
            latched_shared <= 1'b0;
        end else begin
            state         <= next_state;
            word_count    <= word_count_next;
            refill_buffer <= refill_buffer_next;
            req_latched   <= req_latched_next;
            latched_addr  <= latched_addr_next;
            latched_we    <= latched_we_next;
            prev_cpu_req  <= cpu_req;
            ready         <= ready_next;
            write_line_next_cycle <= write_line_next_cycle_next;
            snoop_wb      <= snoop_wb_next;
            latched_shared <= latched_shared_next;
            
            // Debug state transitions
            if (state != next_state) begin
`ifdef DEBUG_VERBOSE
                $display("[L1 @%0t] STATE CHANGE: %s -> %s", 
                         $time, state.name(), next_state.name());
`endif
            end
        end
    end
    
    // Monitor mem_valid changes (combinational output)
    logic prev_mem_valid;
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            prev_mem_valid <= 1'b0;
        end else begin
            if (prev_mem_valid != mem_valid) begin
`ifdef DEBUG_VERBOSE
                $display("[L1 @%0t] MEM_VALID CHANGE: %b -> %b (state=%s)", 
                         $time, prev_mem_valid, mem_valid, state.name());
`endif
            end
            prev_mem_valid <= mem_valid;
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
        ready_next         = ready;  // Hold previous value by default (not 0!)
        write_line_next_cycle_next = 1'b0;  // Default: no write
        snoop_wb_next = snoop_wb;  // Hold previous value
        // Sticky latch: capture shared_copy_exists during alloc, clear on return to IDLE
        if (state == IDLE)
            latched_shared_next = 1'b0;
        else if (shared_copy_exists)
            latched_shared_next = 1'b1;
        else
            latched_shared_next = latched_shared;

        case (state)

            IDLE: begin
                // PRIORITY 1: Snoop-requested write-back (intervention)
                if (writeback_req) begin
                    // Snoop controller requests write-back of dirty line
                    // Save the write-back address and initiate write-back
                    next_state = WB_ADDR;
                    latched_addr_next = writeback_addr;  // Use snoop address for write-back
                    word_count_next = 2'b00;
                    snoop_wb_next = 1'b1;  // Mark as snoop-triggered WB
`ifdef DEBUG_VERBOSE
                    $display("[L1 @%0t] WRITEBACK REQUEST from snoop: addr=%h", $time, writeback_addr);
`endif
                end
                // PRIORITY 2: CPU requests
                // Accept on rising edge OR when cpu_req is already high but we haven't
                // latched it yet (can happen if req arrived during snoop-triggered WB)
                else if (cpu_req && (!prev_cpu_req || !req_latched)) begin
                    // New cpu request - latch it
                    next_state        = COMPARE_TAG;
                    word_count_next   = 2'b00;
                    req_latched_next  = 1'b1;
                    latched_addr_next = cpu_addr;
                    latched_we_next   = cpu_we;
                    ready_next        = 1'b0;  // Clear ready on new request
                end else begin
                    // Hold ready high if we just completed and cpu_req still high
                    if (req_latched && cpu_req) begin
                        ready_next = 1'b1;
                    end else begin
                        // Clear ready when cpu_req is de-asserted
                        ready_next = 1'b0;
                    end
                    
                    if (!cpu_req) begin
                        // Clear latch when request de-asserted
                        req_latched_next = 1'b0;
                        ready_next       = 1'b0;  // Clear ready when request goes away
                    end
                end
            end

            COMPARE_TAG: begin
                if (hit && evict_valid) begin
                    // Check if write to Shared line (needs invalidation)
                    if (latched_we && mesi_state == MESI_SHARED) begin
`ifdef DEBUG_VERBOSE
                        $display("[L1 @%0t] Write to SHARED line - requesting invalidation", $time);
`endif
                        next_state = UPGRADE;  // Go to UPGRADE state to invalidate peers
                        ready_next = 1'b0;     // Not ready yet
                    end else begin
                        next_state = IDLE;
                        ready_next = 1'b1;  // Assert ready on hit
                    end
                    // Don't clear latch here - wait for cpu_req to go low
                end else if (evict_dirty && evict_valid) begin
                    next_state = WB_ADDR;
                end else begin
                    next_state = ALLOC_ADDR;
                end
            end
            
            UPGRADE: begin
                // Wait for invalidation to complete
                if (mem_ready) begin
`ifdef DEBUG_VERBOSE
                    $display("[L1 @%0t] Invalidation complete - write can proceed", $time);
`endif
                    next_state = IDLE;
                    ready_next = 1'b1;  // Assert ready after invalidation
                end
            end

            WB_ADDR: begin
                if (mem_ready) begin
                    next_state      = WB_DATA;
                    word_count_next = 2'b00;  // Start with word 0
                end
            end

            WB_DATA: begin
                if (mem_ready) begin
                    if (word_count == 2'b11) begin
                        // All 4 words sent
                        if (snoop_wb) begin
                            // Snoop-triggered WB: return to IDLE (no re-allocation)
                            next_state  = IDLE;
                            snoop_wb_next = 1'b0;
                        end else begin
                            // Eviction WB: proceed to allocate new line
                            next_state  = ALLOC_ADDR;
                        end
                        word_count_next = 2'b00;
                    end else begin
                        // More words to send
                        word_count_next = word_count + 2'b01;
                    end
                end
            end

            ALLOC_ADDR: begin
                // Always transition to ALLOC_DATA immediately
                // Wait for mem_ready pulses in ALLOC_DATA state
                next_state      = ALLOC_DATA;
                word_count_next = 2'b00;  // Start receiving word 0
            end

            ALLOC_DATA: begin
                // Priority 1: Snoop bus for write-backs from peer caches (cache-to-cache transfer)
                if (bus_valid && bus_we && (bus_addr[31:4] == latched_addr[31:4])) begin
                    // Peer cache is writing back the line we're allocating!
                    // Capture data from bus instead of waiting for memory
                    case (bus_addr[3:2])  // Which word is being written
                        2'b00: refill_buffer_next[31:0]   = bus_wdata;
                        2'b01: refill_buffer_next[63:32]  = bus_wdata;
                        2'b10: refill_buffer_next[95:64]  = bus_wdata;
                        2'b11: refill_buffer_next[127:96] = bus_wdata;
                    endcase
                    
`ifdef DEBUG_VERBOSE
                    $display("[L1 @%0t] ALLOC_DATA SNOOPED writeback: word=%d, data=%h (from peer cache)", 
                             $time, bus_addr[3:2], bus_wdata);
`endif
                    
                    // Track words received via snooping
                    if (bus_addr[3:2] == 2'b11) begin
                        // Last word received - allocation complete
`ifdef DEBUG_VERBOSE
                        $display("[L1 @%0t] ALLOC_DATA complete via SNOOP! refill_buffer={%h,%h,%h,%h}", 
                                 $time,
                                 refill_buffer_next[127:96],
                                 refill_buffer_next[95:64],
                                 refill_buffer_next[63:32],
                                 refill_buffer_next[31:0]);
`endif
                        next_state = IDLE;
                        ready_next = 1'b1;
                        write_line_next_cycle_next = 1'b1;
                        // Don't clear latch - let CPU de-assert req
                    end
                end
                // Priority 2: Normal memory allocation
                else if (mem_ready) begin
                    // Store received word in buffer
                    case (word_count)
                        2'b00: refill_buffer_next[31:0]   = mem_rdata;
                        2'b01: refill_buffer_next[63:32]  = mem_rdata;
                        2'b10: refill_buffer_next[95:64]  = mem_rdata;
                        2'b11: refill_buffer_next[127:96] = mem_rdata;
                    endcase
                    
`ifdef DEBUG_VERBOSE
                    $display("[L1 @%0t] ALLOC_DATA captured: word_count=%d, mem_rdata=%h", 
                             $time, word_count, mem_rdata);
`endif
                    
                    if (word_count == 2'b11) begin
                        // All 4 words received
`ifdef DEBUG_VERBOSE
                        $display("[L1 @%0t] ALLOC_DATA complete! refill_buffer={%h,%h,%h,%h}", 
                                 $time, 
                                 refill_buffer_next[127:96], 
                                 refill_buffer_next[95:64],
                                 refill_buffer_next[63:32],
                                 refill_buffer_next[31:0]);
`endif
                        next_state = IDLE;
                        ready_next = 1'b1;  // Assert ready on completion
                        write_line_next_cycle_next = 1'b1;  // Write cache in next cycle
                        // Don't clear latch here - wait for cpu_req to go low
                    end else begin
                        // More words to receive
                        word_count_next = word_count + 2'b01;
                    end
                end
            end

        endcase
    end

endmodule
