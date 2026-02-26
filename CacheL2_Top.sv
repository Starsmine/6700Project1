// ============================================================================
// L2 Cache Top Module
// ============================================================================
// This module instantiates the L2 cache controller and datapath.
// The controller (from CacheL2_Controller.sv) manages FSM and MESI transitions.
// The datapath (inline cache module) contains storage arrays and access logic.
// ============================================================================

module cachel2_top (
    input  logic         clk,
    input  logic         reset,

    // CPU Interface (32-bit)
    input  logic         cpu_req,
    input  logic         cpu_we,
    input  logic         cpu_rwitm,      // CPU Read-With-Intent-To-Modify
    input  logic [31:0]  cpu_addr,
    input  logic [31:0]  cpu_wdata,
    output logic [31:0]  cpu_rdata,
    output logic         cpu_ready,

    // Memory Interface (32-bit blocking bus)
    output logic         mem_valid,      // Memory request valid
    input  logic         mem_ready,      // Memory ready for transaction
    output logic         mem_we,         // Write enable
    output logic         mem_rwitm,      // Read-With-Intent-To-Modify (write miss)
    output logic [31:0]  mem_addr,       // Address or data (for writes)
    input  logic [31:0]  mem_rdata,      // Read data (for reads)
    
    // MESI State Interface (output only - for monitoring)
    output logic [1:0]   mesi_state_out, // Current MESI state for requested address
    
    // Snoop Interface (from other L2's snoop controller)
    input  logic         snoop_req,      // Snoop request
    input  logic [31:0]  snoop_addr,     // Address to snoop
    output logic         snoop_has_line, // This cache has the line
    output logic [31:0]  snoop_data,     // Data for cache-to-cache transfer (single word)
    output logic [1:0]   snoop_mesi_state, // MESI state of snooped line (for cross-CCL coherence)
    
    // Coherence Interface (from snoop controller)
    input  logic         coherence_invalidate,  // Invalidate command
    input  logic         coherence_downgrade,   // Downgrade to Shared command
    input  logic [31:0]  coherence_addr,        // Address for coherence action
    
    // Coherence information (from snoop controller to controller)
    input  logic         shared_copy_exists,    // Another L2 has this line
    input  logic         l1_has_dirty_data,      // L1 has Modified copy (for intervention)
    input  logic [31:0]  l1_snoop_data,          // Fresh data from L1 intervention
    
    // Peer writeback coordination
    output logic         wait_for_peer_wb,       // L2 waiting for peer writeback (L1 should release bus)
    
    // Cross-CCL dirty intervention
    input  logic         coherence_writeback,    // External writeback request (cross-CCL)
    input  logic         snoop_dirty_stall,      // Fill stall: snoop detected dirty peer
    
    // L1 dirty data pending for coherence writeback
    input  logic         l1_dirty_pending_wb      // L1 has dirty data that must be received first
);

    // -------------------------
    // Internal wires
    // -------------------------
    logic hit;
    logic [1:0] mesi_state;       // MESI state from datapath
    logic [19:0] evict_tag;
    logic [7:0]  index;
    logic [127:0] evict_block;
    logic [127:0] refill_data;
    logic [1:0] hit_way;
    logic [1:0] lru_way;

    logic write_enable_word;
    logic write_enable_line;
    logic [1:0] target_way;
    logic use_refill_data;        // Pass-through control signal
    logic use_cache_forward;      // Forward from cache using word_count
    logic [1:0] forward_word_index; // Which word from refill to forward
    logic [31:0] datapath_rdata;   // Data from cache
    logic [31:0] forward_rdata;    // Data from refill buffer
    
    // Write-back address override
    logic        use_wb_addr;      // Use write-back address instead of cpu_addr
    logic [31:0] wb_addr_out;      // Write-back address from controller
    logic [31:0] dp_addr;          // Muxed address for datapath
    
    // Latched address for stable hit detection
    logic [31:0] latched_addr_out; // Latched address from controller
    logic        addr_latched;     // 1 = request latched, use latched address
    
    // MESI control signals from controller to datapath
    logic [1:0]  mesi_state_next;
    logic        mesi_update;
    logic [31:0] mesi_update_addr;
    
    // Expose MESI state for current address/way
    assign mesi_state_out = mesi_state;
    
    // Mux address: use write-back address during RECEIVE_WB,
    // latched address when request is active (stable hit detection),
    // else live cpu_addr for initial lookup
    assign dp_addr = use_wb_addr    ? wb_addr_out :
                     addr_latched   ? latched_addr_out :
                                      cpu_addr;
    
    // Select word from refill buffer based on forward index
    always_comb begin
        case (forward_word_index)
            2'b00: forward_rdata = refill_data[31:0];
            2'b01: forward_rdata = refill_data[63:32];
            2'b10: forward_rdata = refill_data[95:64];
            2'b11: forward_rdata = refill_data[127:96];
        endcase
    end
    
    // Mux between cache data, memory pass-through, buffered forward data, and L1 intervention data
    // Priority:
    //   1. use_cache_forward=1: Use L2 cache data directly (CACHE_FORWARD state)
    //   2. use_refill_data=1: Use refill buffer or live memory data
    //   3. Otherwise: Use L2 cache data
    // Note: l1_has_dirty_data is NOT used to override the data mux — the L2 controller
    //       now waits for the dirty peer to write back before entering CACHE_FORWARD,
    //       ensuring L2 data is always fresh when forwarding from cache.
    assign cpu_rdata = use_cache_forward ? datapath_rdata :
                       use_refill_data ? (mem_ready ? mem_rdata : forward_rdata) :
                       datapath_rdata;

    // -------------------------
    // Controller (from CacheL2_Controller.sv)
    // -------------------------
    cachel2_controller ctrl (
        .clk(clk),
        .reset(reset),

        .cpu_req(cpu_req),
        .cpu_we(cpu_we),
        .cpu_rwitm(cpu_rwitm),
        .cpu_addr(cpu_addr),

        .hit(hit),
        .mesi_state(mesi_state),
        .evict_tag(evict_tag),
        .index(index),
        .evict_block(evict_block),
        .hit_way(hit_way),
        .lru_way(lru_way),

        .mem_ready(mem_ready),
        .mem_valid(mem_valid),
        .mem_we(mem_we),
        .mem_rwitm(mem_rwitm),
        .mem_addr(mem_addr),
        .mem_rdata(mem_rdata),
        .refill_data(refill_data),

        .write_enable_word(write_enable_word),
        .write_enable_line(write_enable_line),
        .target_way(target_way),
        
        // Write-back address override
        .use_wb_addr(use_wb_addr),
        .wb_addr_out(wb_addr_out),
        
        // MESI control outputs
        .mesi_state_next(mesi_state_next),
        .mesi_update(mesi_update),
        .mesi_update_addr(mesi_update_addr),
        
        // Snoop inputs
        .shared_copy_exists(shared_copy_exists),
        .l1_has_dirty_data(l1_has_dirty_data),

        .ready(),                        // Registered ready (not used)
        .ready_comb(cpu_ready),          // Combinational ready for low-latency pass-through
        .wait_for_peer_wb(wait_for_peer_wb),  // Signal L1 to release bus
        .use_refill_data(use_refill_data),
        .use_cache_forward(use_cache_forward),
        .forward_word_index(forward_word_index),
        
        // Cross-CCL dirty intervention
        .coherence_writeback(coherence_writeback),
        .snoop_dirty_stall(snoop_dirty_stall),
        .coherence_addr(coherence_addr),
        .l1_dirty_pending_wb(l1_dirty_pending_wb),
        
        // Latched address for stable datapath addressing
        .latched_addr_out(latched_addr_out),
        .addr_latched(addr_latched)
    );

    // -------------------------
    // Datapath (inline cache module - to be extracted later)
    // -------------------------
    cachel2_datapath dp (
        .clk(clk),
        .reset(reset),

        .addr(dp_addr),              // Use muxed address (wb_addr during RECEIVE_WB, else cpu_addr)
        .wdata(cpu_wdata),
        .cpu_we(cpu_we),

        .write_enable_word(write_enable_word),
        .write_enable_line(write_enable_line),
        .refill_data(refill_data),
        .target_way(target_way),
        .use_cache_forward(use_cache_forward),
        .forward_word_index(forward_word_index),

        .hit(hit),
        .evict_tag(evict_tag),
        .index(index),
        .rdata(datapath_rdata),  // Cache read data
        .hit_way(hit_way),
        .lru_way(lru_way),

        .evict_block(evict_block),
        
        // MESI interface (from controller)
        .mesi_state(mesi_state),
        .mesi_state_in(mesi_state_next),
        .mesi_update(mesi_update),
        .mesi_addr(mesi_update_addr),
        
        // Snoop interface
        .snoop_req(snoop_req),
        .snoop_addr(snoop_addr),
        .snoop_has_line(snoop_has_line),
        .snoop_data(snoop_data),
        .snoop_mesi_state(snoop_mesi_state),
        
        // Coherence interface
        .coherence_invalidate(coherence_invalidate),
        .coherence_downgrade(coherence_downgrade),
        .coherence_addr(coherence_addr)
    );

endmodule
   
// ============================================================================

module cachel2_datapath (
    input  logic         clk,
    input  logic         reset,

    // CPU request
    input  logic [31:0]  addr,
    input  logic [31:0]  wdata,
    input  logic         cpu_we,	
	
    output logic [31:0]  rdata,

    // Controller control signals
    input  logic         write_enable_word,   // write 32-bit word (CPU write hit)
    input  logic         write_enable_line,   // write full 128-bit block (allocate)
    input  logic [1:0]   target_way,          // which way to access
    input  logic         use_cache_forward,   // forward cache data using word index
    input  logic [1:0]   forward_word_index,  // which word to forward (from word_count)
	

    // Outputs to controller
    output logic         hit,
    output logic [19:0]  evict_tag,
    output logic [7:0]   index,
    output logic [1:0]   hit_way,
    output logic [1:0]   lru_way,
	
    //memory
    input  logic [127:0] refill_data,         // from memory
    output logic [127:0] evict_block,
    
    // MESI interface
    output logic [1:0]   mesi_state,          // current MESI state for target_way
    input  logic [1:0]   mesi_state_in,       // new MESI state
    input  logic         mesi_update,         // update MESI state
    input  logic [31:0]  mesi_addr,           // address for MESI update
    
    // Snoop interface
    input  logic         snoop_req,           // Snoop request
    input  logic [31:0]  snoop_addr,          // Address to snoop
    output logic         snoop_has_line,      // This cache has the line
    output logic [31:0]  snoop_data,          // Data for cache-to-cache transfer (single word)
    output logic [1:0]   snoop_mesi_state,    // MESI state of snooped line
    
    // Coherence interface
    input  logic         coherence_invalidate,   // Invalidate command
    input  logic         coherence_downgrade,    // Downgrade to Shared
    input  logic [31:0]  coherence_addr          // Address for coherence action
);

    // Cache geometry: 4-way set associative, 16 KiB total
    localparam WAYS          = 4;
    localparam SETS          = 256;  // 16 KiB / (4 ways * 16 bytes)
    localparam WORDS_PER_BLK = 4;    // 4 × 32-bit words per block

    // Address decode
    logic [19:0] tag;
    logic [1:0]  word_sel;

    assign tag      = addr[31:12];
    assign index    = addr[11:4];
    assign word_sel = use_cache_forward ? forward_word_index : addr[3:2];

    // Storage arrays (4 ways)
    logic [31:0] data_array [0:WAYS-1][0:SETS-1][0:WORDS_PER_BLK-1];
    logic [19:0] tag_array  [0:WAYS-1][0:SETS-1];
    
    // MESI state arrays (2 bits per set per way)
    // Encoding: Invalid=00, Shared=01, Exclusive=10, Modified=11
    typedef enum logic [1:0] {
        MESI_INVALID   = 2'b00,
        MESI_SHARED    = 2'b01,
        MESI_EXCLUSIVE = 2'b10,
        MESI_MODIFIED  = 2'b11
    } mesi_state_t;
    
    logic [1:0] mesi_array [0:WAYS-1][0:SETS-1];
    
    // LRU tracking: 2 bits per set (for pseudo-LRU)
    // Simple LRU counter approach: track most recently used way
    logic [1:0] lru_counter [0:SETS-1];

    // Expose metadata for controller
    assign evict_tag = tag_array[target_way][index];
    
    // Expose MESI state for the target way
    assign mesi_state = mesi_array[target_way][index];

    // Hit detection across all ways (valid = not INVALID)
    logic [3:0] way_hit;
    assign way_hit[0] = (mesi_array[0][index] != MESI_INVALID) && (tag_array[0][index] == tag);
    assign way_hit[1] = (mesi_array[1][index] != MESI_INVALID) && (tag_array[1][index] == tag);
    assign way_hit[2] = (mesi_array[2][index] != MESI_INVALID) && (tag_array[2][index] == tag);
    assign way_hit[3] = (mesi_array[3][index] != MESI_INVALID) && (tag_array[3][index] == tag);
    
    assign hit = |way_hit;  // OR of all way hits
    
    // Determine which way hit
    always_comb begin
        hit_way = 2'b00;
        if (way_hit[0])      hit_way = 2'b00;
        else if (way_hit[1]) hit_way = 2'b01;
        else if (way_hit[2]) hit_way = 2'b10;
        else if (way_hit[3]) hit_way = 2'b11;
    end

    // LRU replacement: find first invalid way, or use LRU counter
    always_comb begin
        lru_way = lru_counter[index];
        
        // Prefer invalid ways
        if (mesi_array[0][index] == MESI_INVALID)      lru_way = 2'b00;
        else if (mesi_array[1][index] == MESI_INVALID) lru_way = 2'b01;
        else if (mesi_array[2][index] == MESI_INVALID) lru_way = 2'b10;
        else if (mesi_array[3][index] == MESI_INVALID) lru_way = 2'b11;
        else                                            lru_way = lru_counter[index];
    end

    // CPU read data (selected word from hit way or target way)
    logic [1:0] read_way;
    assign read_way = use_cache_forward ? target_way : (hit ? hit_way : target_way);
    assign rdata = data_array[read_way][index][word_sel];

    // Full 128-bit block for write-back (from target way)
    assign evict_block = {
        data_array[target_way][index][3],
        data_array[target_way][index][2],
        data_array[target_way][index][1],
        data_array[target_way][index][0]
    };
    
    // Snoop logic: Check all ways for snooped address
    logic [19:0] snoop_tag;
    logic [7:0]  snoop_index;
    logic [1:0]  snoop_word_sel;
    logic [3:0]  snoop_way_hit;
    logic [1:0]  snoop_hit_way;
    
    assign snoop_tag      = snoop_addr[31:12];
    assign snoop_index    = snoop_addr[11:4];
    assign snoop_word_sel = snoop_addr[3:2];
    
    assign snoop_way_hit[0] = (mesi_array[0][snoop_index] != MESI_INVALID) && (tag_array[0][snoop_index] == snoop_tag);
    assign snoop_way_hit[1] = (mesi_array[1][snoop_index] != MESI_INVALID) && (tag_array[1][snoop_index] == snoop_tag);
    assign snoop_way_hit[2] = (mesi_array[2][snoop_index] != MESI_INVALID) && (tag_array[2][snoop_index] == snoop_tag);
    assign snoop_way_hit[3] = (mesi_array[3][snoop_index] != MESI_INVALID) && (tag_array[3][snoop_index] == snoop_tag);
    
    assign snoop_has_line = snoop_req && (|snoop_way_hit);
    
    // Priority encoder for snoop hit way
    always_comb begin
        snoop_hit_way = 2'b00;
        if (snoop_way_hit[0])      snoop_hit_way = 2'b00;
        else if (snoop_way_hit[1]) snoop_hit_way = 2'b01;
        else if (snoop_way_hit[2]) snoop_hit_way = 2'b10;
        else if (snoop_way_hit[3]) snoop_hit_way = 2'b11;
    end
    
    // Provide snoop data (single word selected by address bits [3:2])
    assign snoop_data = data_array[snoop_hit_way][snoop_index][snoop_word_sel];
    
    // Provide MESI state of snooped line (for cross-CCL coherence decisions)
    assign snoop_mesi_state = snoop_has_line ? mesi_array[snoop_hit_way][snoop_index] : MESI_INVALID;

    // Writes
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < WAYS; i++) begin
                for (int j = 0; j < SETS; j++) begin
                    mesi_array[i][j]  <= MESI_INVALID;
                end
            end
            
            for (int j = 0; j < SETS; j++) begin
                lru_counter[j] <= 2'b00;
            end
        end else begin

            // CPU write hit (single word to target way)
            if (write_enable_word) begin
                data_array[target_way][index][word_sel] <= wdata;
            end

            // Allocate (write full 128-bit block to target way)
            if (write_enable_line) begin
                data_array[target_way][index][0] <= refill_data[31:0];
                data_array[target_way][index][1] <= refill_data[63:32];
                data_array[target_way][index][2] <= refill_data[95:64];
                data_array[target_way][index][3] <= refill_data[127:96];
                tag_array[target_way][index]     <= tag;
            end
            
            // Update LRU on any access (hit or allocation)
            if (write_enable_word || write_enable_line) begin
                // Update LRU to point to next way (simple round-robin LRU)
                lru_counter[index] <= (target_way + 2'b01) % WAYS;
            end
            
            // MESI state update from arbiter
            if (mesi_update) begin
                logic [7:0] mesi_index;
                mesi_index = mesi_addr[11:4];
                mesi_array[target_way][mesi_index] <= mesi_state_in;
            end
            
            // Coherence handlers (from snoop controller)
            if (coherence_invalidate) begin
                logic [19:0] coh_tag;
                logic [7:0]  coh_index;
                coh_tag   = coherence_addr[31:12];
                coh_index = coherence_addr[11:4];
                
                // Invalidate matching line in any way
                for (int i = 0; i < WAYS; i++) begin
                    if ((mesi_array[i][coh_index] != MESI_INVALID) && 
                        (tag_array[i][coh_index] == coh_tag)) begin
                        mesi_array[i][coh_index] <= MESI_INVALID;
                    end
                end
            end
            
            if (coherence_downgrade) begin
                logic [19:0] coh_tag;
                logic [7:0]  coh_index;
                coh_tag   = coherence_addr[31:12];
                coh_index = coherence_addr[11:4];
                
                // Downgrade matching line to Shared
                for (int i = 0; i < WAYS; i++) begin
                    if ((mesi_array[i][coh_index] == MESI_MODIFIED || 
                         mesi_array[i][coh_index] == MESI_EXCLUSIVE) && 
                        (tag_array[i][coh_index] == coh_tag)) begin
                        mesi_array[i][coh_index] <= MESI_SHARED;
                    end
                end
            end
        end
    end

endmodule