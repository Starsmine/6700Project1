// ============================================================================
// L1 Cache Top Module
// ============================================================================
// This module instantiates the L1 cache controller and datapath.
// The controller (from CacheL1_Controller.sv) manages FSM and MESI transitions.
// The datapath (inline cache module) contains storage arrays and access logic.
// ============================================================================

module cachel1_top (
    input  logic         clk,
    input  logic         reset,

    // CPU Interface (32-bit)
    input  logic         cpu_req,
    input  logic         cpu_we,
    input  logic [31:0]  cpu_addr,
    input  logic [31:0]  cpu_wdata,
    output logic [31:0]  cpu_rdata,
    output logic         cpu_ready,
    output logic         cpu_miss,       // High when completing a miss operation

    // Memory Interface (32-bit blocking bus)
    output logic         mem_valid,      // Memory request valid
    input  logic         mem_ready,      // Memory ready for transaction
    output logic         mem_we,         // Write enable
    output logic         mem_rwitm,      // Read-With-Intent-To-Modify (write miss)
    output logic [31:0]  mem_addr,       // Address or data (for writes)
    input  logic [31:0]  mem_rdata,      // Read data (for reads)
    
    // MESI State Interface (output only - for monitoring)
    output logic [1:0]   mesi_state_out, // Current MESI state for requested address
    
    // Snoop Interface (from snoop controller)
    input  logic         snoop_req,      // Snoop request
    input  logic [31:0]  snoop_addr,     // Address to check
    output logic         has_line,       // This cache has the line
    output logic [31:0]  snoop_data,     // Data if line is dirty
    input  logic         invalidate,     // Invalidate this line
    input  logic         downgrade,      // Downgrade M/E -> S
    input  logic [31:0]  coherence_addr, // Address for coherence action
    input  logic         writeback_req,  // Write-back request from snoop
    input  logic [31:0]  writeback_addr, // Address to write back
    
    // Bus snooping (to capture write-backs from peer caches)
    input  logic         bus_valid,      // Bus transaction active
    input  logic         bus_we,         // Bus write (write-back)
    input  logic [31:0]  bus_addr,       // Bus address
    input  logic [31:0]  bus_wdata,      // Bus write data
    
    // Coherence information (from snoop controller to controller)
    input  logic         shared_copy_exists,  // Another cache has this line
    input  logic         wait_for_peer_wb,    // L2 waiting for peer WB (release bus)
    
    // Cross-CCL snoop query (separate from intra-CCL snoop controller)
    input  logic [31:0]  xcl_snoop_addr,      // Address to check for cross-CCL snoops
    output logic         xcl_has_line,         // L1 has this line
    output logic [1:0]   xcl_mesi_state        // MESI state of line at snooped address
);

    // -------------------------
    // Internal wires
    // -------------------------
    logic hit;
    logic [1:0] mesi_state;       // MESI state from datapath
    logic [18:0] evict_tag;
    logic [8:0]  index;
    logic [127:0] evict_block;
    logic [127:0] refill_data;
    logic [31:0] datapath_rdata;  // Data from cache read

    logic write_enable_word;
    logic write_enable_line;
    logic use_refill_for_cpu;     // Use refill data instead of cache read
    logic miss;                   // Miss signal from controller
    
    // MESI control signals from controller to datapath
    logic [1:0]  mesi_state_next;
    logic        mesi_update;
    logic [31:0] mesi_update_addr;
    
    // Expose MESI state for current address
    assign mesi_state_out = mesi_state;
    
    // Expose miss signal to CPU interface
    assign cpu_miss = miss;
    
    // Mux CPU read data: use refill buffer for fresh allocations
    // Extract the requested word from refill buffer based on address
    logic [31:0] refill_word;
    always_comb begin
        case (cpu_addr[3:2])  // Word offset within block
            2'b00: refill_word = refill_data[31:0];
            2'b01: refill_word = refill_data[63:32];
            2'b10: refill_word = refill_data[95:64];
            2'b11: refill_word = refill_data[127:96];
        endcase
    end
    assign cpu_rdata = use_refill_for_cpu ? refill_word : datapath_rdata;

    // -------------------------
    // Controller (from CacheL1_Controller.sv)
    // -------------------------
    cachel1_controller ctrl (
        .clk(clk),
        .reset(reset),

        .cpu_req(cpu_req),
        .cpu_we(cpu_we),
        .cpu_addr(cpu_addr),

        .hit(hit),
        .mesi_state(mesi_state),
        .evict_tag(evict_tag),
        .index(index),
        .evict_block(evict_block),

        .mem_ready(mem_ready),
        .mem_valid(mem_valid),
        .mem_we(mem_we),
        .mem_rwitm(mem_rwitm),
        .mem_addr(mem_addr),
        .mem_rdata(mem_rdata),
        .refill_data(refill_data),

        .write_enable_word(write_enable_word),
        .write_enable_line(write_enable_line),
        .use_refill_for_cpu(use_refill_for_cpu),
        
        // MESI control outputs
        .mesi_state_next(mesi_state_next),
        .mesi_update(mesi_update),
        .mesi_update_addr(mesi_update_addr),
        
        // Coherence inputs
        .shared_copy_exists(shared_copy_exists),
        .wait_for_peer_wb(wait_for_peer_wb),
        .writeback_req(writeback_req),
        .writeback_addr(writeback_addr),
        
        // Bus snooping
        .bus_valid(bus_valid),
        .bus_we(bus_we),
        .bus_addr(bus_addr),
        .bus_wdata(bus_wdata),

        .ready(cpu_ready),
        .miss(miss)
    );

    // -------------------------
    // Datapath (inline cache module - to be extracted later)
    // -------------------------
    cachel1_datapath dp (
        .clk(clk),
        .reset(reset),

        .addr(cpu_addr),
        .wdata(cpu_wdata),
        .cpu_we(cpu_we),

        .write_enable_word(write_enable_word),
        .write_enable_line(write_enable_line),
        .refill_data(refill_data),

        .hit(hit),
        .mesi_state(mesi_state),
        .evict_tag(evict_tag),
        .index(index),
        .rdata(datapath_rdata),  // Cache read data

        .evict_block(evict_block),
        
        // MESI update interface (from controller)
        .mesi_state_in(mesi_state_next),
        .mesi_update(mesi_update),
        .mesi_addr(mesi_update_addr),
        
        // Snoop interface
        .snoop_req(snoop_req),
        .snoop_addr(snoop_addr),
        .has_line(has_line),
        .snoop_data(snoop_data),
        .invalidate(invalidate),
        .downgrade(downgrade),
        .coherence_addr(coherence_addr),
        
        // Cross-CCL snoop query
        .xcl_snoop_addr(xcl_snoop_addr),
        .xcl_has_line(xcl_has_line),
        .xcl_mesi_state(xcl_mesi_state)
    );

endmodule
   
//----------------------------------------------------------------------

module cachel1_datapath (
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
	

    // Outputs to controller
    output logic         hit,
    output logic [1:0]   mesi_state,          // Current MESI state
    output logic [18:0]  evict_tag,
    output logic [8:0]   index,
	
    //memory
    input  logic [127:0] refill_data,         // from memory
    output logic [127:0] evict_block,
    
    // MESI State Interface
    input  logic [1:0]   mesi_state_in,       // New MESI state
    input  logic         mesi_update,         // Update MESI state
    input  logic [31:0]  mesi_addr,           // Address for MESI update
    
    // Snoop Interface
    input  logic         snoop_req,           // Snoop request
    input  logic [31:0]  snoop_addr,          // Address to check
    output logic         has_line,            // This cache has the line
    output logic [31:0]  snoop_data,          // Data if line is dirty
    input  logic         invalidate,          // Invalidate command
    input  logic         downgrade,           // Downgrade M/E -> S
    input  logic [31:0]  coherence_addr,      // Address for coherence action
    
    // Cross-CCL snoop query
    input  logic [31:0]  xcl_snoop_addr,      // Cross-CCL snoop address
    output logic         xcl_has_line,         // L1 has this line
    output logic [1:0]   xcl_mesi_state        // MESI state of snooped line
);

    // Cache geometry
    localparam BLOCKS        = 512;  // 8 KiB / 16 bytes
    localparam WORDS_PER_BLK = 4;    // 4 × 32-bit words per block

    // MESI state encoding
    typedef enum logic [1:0] {
        MESI_INVALID   = 2'b00,
        MESI_SHARED    = 2'b01,
        MESI_EXCLUSIVE = 2'b10,
        MESI_MODIFIED  = 2'b11
    } mesi_state_t;

    // Address decode
    logic [18:0] tag;
    logic [1:0]  word_sel;

    assign tag      = addr[31:13];
    assign index    = addr[12:4];
    assign word_sel = addr[3:2];

    // Storage arrays
    logic [31:0] data_array [0:BLOCKS-1][0:WORDS_PER_BLK-1];
    logic [18:0] tag_array  [0:BLOCKS-1];
    logic [1:0]  mesi_array [0:BLOCKS-1];  // MESI state storage

    // Expose metadata for controller
    assign mesi_state = mesi_array[index];
    assign evict_tag = tag_array[index];

    // Hit detection (valid = not INVALID)
    assign hit = (mesi_array[index] != MESI_INVALID) && (tag_array[index] == tag);

    // CPU read data (selected word)
    assign rdata = data_array[index][word_sel];

    // Full 128-bit block for write-back
    assign evict_block = {
        data_array[index][3],
        data_array[index][2],
        data_array[index][1],
        data_array[index][0]
    };
    
    // Snoop logic
    logic [8:0] snoop_index;
    assign snoop_index = snoop_addr[12:4];
    assign has_line = snoop_req && (mesi_array[snoop_index] != MESI_INVALID) && 
                      (tag_array[snoop_index] == snoop_addr[31:13]);
    assign snoop_data = data_array[snoop_index][snoop_addr[3:2]];  // Word selected by snoop addr

    // Cross-CCL snoop query (separate index, no conflict with L1 snoop controller)
    logic [8:0] xcl_snoop_index;
    assign xcl_snoop_index = xcl_snoop_addr[12:4];
    assign xcl_has_line = (mesi_array[xcl_snoop_index] != MESI_INVALID) && 
                          (tag_array[xcl_snoop_index] == xcl_snoop_addr[31:13]);
    assign xcl_mesi_state = mesi_array[xcl_snoop_index];

    // Writes
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            for (int i = 0; i < BLOCKS; i++) begin
                mesi_array[i]  <= MESI_INVALID;
            end
        end else begin

            // Allocate (write full 128-bit block) - must come BEFORE word write
            // so that a simultaneous write_enable_word (write-miss) overrides the
            // refill data for the specific word being written by the CPU.
            if (write_enable_line) begin
                data_array[index][0] <= refill_data[31:0];
                data_array[index][1] <= refill_data[63:32];
                data_array[index][2] <= refill_data[95:64];
                data_array[index][3] <= refill_data[127:96];
                tag_array[index]     <= tag;
                // MESI state set by arbiter via mesi_update
            end

            // CPU write (single word) - comes AFTER line write so it wins
            // for write-miss allocations (both write_enable_line and word fire together)
            if (write_enable_word) begin
                data_array[index][word_sel] <= wdata;
            end
            
            // MESI state update from arbiter
            if (mesi_update && mesi_addr[12:4] == index) begin
                mesi_array[index] <= mesi_state_in;
            end
            
            // Coherence actions from snoop controller
            if (invalidate && coherence_addr[12:4] == index) begin
                mesi_array[index]  <= MESI_INVALID;
            end
            
            if (downgrade && coherence_addr[12:4] == index) begin
                // Downgrade M or E to S
                if (mesi_array[index] == MESI_MODIFIED || mesi_array[index] == MESI_EXCLUSIVE) begin
                    mesi_array[index] <= MESI_SHARED;
                end
            end
        end
    end

endmodule