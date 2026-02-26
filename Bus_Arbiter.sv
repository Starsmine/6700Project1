// ============================================================================
// Multi-Requestor Bus Arbiter
// ============================================================================
// This module provides round-robin bus arbitration for multiple requestors.
// It does NOT handle MESI state transitions - those are handled by the
// cache controllers themselves.
//
// Features:
// - Parameterized number of requestors (default 4)
// - Round-robin arbitration for fairness
// - Single-cycle grant when bus is free
// - Holds grant until transaction completes (ready asserted)
// ============================================================================

module bus_arbiter #(
    parameter NUM_REQUESTORS = 4  // Number of Lower-level requestors
) (
    input  logic         clk,
    input  logic         reset,

    // -------------------------
    // Lower Level Interfaces (Multiple Requestors)
    // -------------------------
    input  logic [NUM_REQUESTORS-1:0]        req_valid,
    output logic [NUM_REQUESTORS-1:0]        req_ready,
    input  logic [NUM_REQUESTORS-1:0]        req_we,
    input  logic [NUM_REQUESTORS-1:0]        req_rwitm,
    input  logic [NUM_REQUESTORS-1:0][31:0]  req_addr,
    input  logic [NUM_REQUESTORS-1:0][31:0]  req_wdata,
    output logic [NUM_REQUESTORS-1:0][31:0]  req_rdata,

    // -------------------------
    // Upper Level Interface (Shared Bus)
    // -------------------------
    output logic         bus_valid,
    input  logic         bus_ready,
    output logic         bus_we,
    output logic         bus_rwitm,
    output logic [31:0]  bus_addr,
    output logic [31:0]  bus_wdata,
    input  logic [31:0]  bus_rdata,
    
    // -------------------------
    // Arbiter Status
    // -------------------------
    output logic [$clog2(NUM_REQUESTORS)-1:0] granted_id_out  // Which requestor has the bus
);

    // =========================================================================
    // Internal Signals
    // =========================================================================
    
    // Current grant (one-hot encoding)
    logic [NUM_REQUESTORS-1:0] grant;
    logic [NUM_REQUESTORS-1:0] grant_next;
    
    // Round-robin pointer
    logic [$clog2(NUM_REQUESTORS)-1:0] rr_pointer;
    logic [$clog2(NUM_REQUESTORS)-1:0] rr_pointer_next;
    
    // Transaction in progress
    logic transaction_active;
    
    // Selected requestor index (binary encoding)
    logic [$clog2(NUM_REQUESTORS)-1:0] granted_id;

    // =========================================================================
    // Grant State Register
    // =========================================================================
    
    logic [NUM_REQUESTORS-1:0][31:0] req_rdata_reg;
    logic [NUM_REQUESTORS-1:0] req_ready_reg;
    
    always_ff @(posedge clk or posedge reset) begin
        if (reset) begin
            grant      <= '0;
            rr_pointer <= '0;
            req_ready_reg <= '0;
            req_rdata_reg <= '{default: 32'b0};
        end else begin
            grant      <= grant_next;
            rr_pointer <= rr_pointer_next;
            
            // Register req_ready and req_rdata to break combinational feedback
            req_ready_reg <= '0;
            req_rdata_reg <= '{default: 32'b0};
            if (|grant) begin
                req_ready_reg[granted_id] <= bus_ready;
                req_rdata_reg[granted_id] <= bus_rdata;
            end
        end
    end

    // =========================================================================
    // Round-Robin Arbitration Logic
    // =========================================================================
    
    // Transaction is active if any grant is asserted
    assign transaction_active = |grant;
    
    always_comb begin
        grant_next      = grant;
        rr_pointer_next = rr_pointer;
        
        if (transaction_active && bus_valid) begin
            // Hold current grant while requestor keeps valid asserted
            grant_next = grant;
            // Hold current grant while requestor keeps valid asserted
            grant_next = grant;
        end else if (transaction_active && !bus_valid) begin
            // Transaction completed - requestor de-asserted valid, release grant
            grant_next      = '0;
            rr_pointer_next = (rr_pointer == NUM_REQUESTORS-1) ? '0 : rr_pointer + 1;
        end else if (|req_valid && !transaction_active) begin
            // New arbitration - find next requestor starting from rr_pointer
            grant_next = '0;
            
            // Round-robin search starting from current pointer
            for (int i = 0; i < NUM_REQUESTORS; i++) begin
                automatic int idx;
                idx = (rr_pointer + i);
                if (idx >= NUM_REQUESTORS)
                    idx = idx - NUM_REQUESTORS;
                if (req_valid[idx] && grant_next == '0) begin
                    grant_next[idx] = 1'b1;
                end
            end
        end else begin
            // No requests, clear grant
            grant_next = '0;
        end
    end

    // =========================================================================
    // One-Hot to Binary Converter (for mux select)
    // =========================================================================
    
    always_comb begin
        granted_id = '0;
        for (int i = 0; i < NUM_REQUESTORS; i++) begin
            if (grant[i]) begin
                granted_id = i[$clog2(NUM_REQUESTORS)-1:0];
            end
        end
    end

    // =========================================================================
    // Output Multiplexing
    // =========================================================================
    
    // Bus outputs - mux based on grant
    always_comb begin
        bus_valid = 1'b0;
        bus_we    = 1'b0;
        bus_rwitm = 1'b0;
        bus_addr  = 32'b0;
        bus_wdata = 32'b0;
        granted_id_out = '0;
        
        if (|grant) begin
            bus_valid = req_valid[granted_id];
            bus_we    = req_we[granted_id];
            bus_rwitm = req_rwitm[granted_id];
            bus_addr  = req_addr[granted_id];
            bus_wdata = req_wdata[granted_id];
            granted_id_out = granted_id;  // Output which requestor is granted
            
            // Debug: Show which requestor is granted
            if (bus_valid) begin
`ifdef DEBUG_VERBOSE
                $display("[ARBITER @%0t] Granted requestor %0d: valid=%b, addr=%h, we=%b, rwitm=%b", 
                         $time, granted_id, bus_valid, bus_addr, bus_we, bus_rwitm);
`endif
            end
        end
    end
    
    // Ready and data distribution - use registered outputs
    assign req_ready = req_ready_reg;
    assign req_rdata = req_rdata_reg;

endmodule
