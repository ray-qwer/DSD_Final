module cache(
    clk,
    proc_reset,
    proc_read,
    proc_write,
    proc_addr,
    proc_rdata,
    proc_wdata,
    proc_stall,
    mem_read,
    mem_write,
    mem_addr,
    mem_rdata,
    mem_wdata,
    mem_ready
);
    parameter IDLE = 2'd0;
    parameter WRITE_BACK = 2'd1;
    parameter ALLOCATE = 2'd2;
    parameter BUFFER = 2'd3;
    
//==== input/output definition ============================
    input          clk;
    // processor interface
    input         proc_reset;
    input         proc_read, proc_write;
    input  [29:0] proc_addr;
    input  [31:0] proc_wdata;
    output reg        proc_stall;
    output reg [31:0] proc_rdata;
    // memory interface
    input  [127:0] mem_rdata;
    input          mem_ready;
    output reg         mem_read, mem_write;
    output reg  [27:0] mem_addr;
    output reg [127:0] mem_wdata;
    
//==== wire/reg definition ================================
    reg [154:0] cache[7:0];
    reg [154:0] cache_nxt[7:0];
    reg [1:0] state, state_nxt;
    wire [24:0] tag;
    wire [2:0]  idx;
    wire [1:0]  sel; 
    wire dirty, hit;

    integer i;

    assign tag = proc_addr[29:5];
    assign idx = proc_addr[4:2];
    assign sel = proc_addr[1:0];
    assign hit = cache[idx][154] & (cache[idx][152:128] == tag);
    assign dirty = cache[idx][153];

//==== Write Buffer        ================================
    reg [31:0] proc_wdata_DWB;
    reg [2:0] idx_DWB;
    reg [1:0] sel_DWB;
    reg [2:0] state_DWB;
    reg proc_write_DWB, hit_DWB;
    wire [127:0] a;
    wire b;
    assign a = cache_nxt[0][127:0];
    assign b = (state_DWB == IDLE) && proc_write_DWB && hit_DWB; 
    
always@(posedge clk) begin
    proc_write_DWB <= proc_write;
    hit_DWB <= hit;
    idx_DWB <= idx;
    sel_DWB <= sel;
    proc_wdata_DWB <= proc_wdata;
    state_DWB <= state;
end

//==== FSM                 ================================
always@(*) begin
    state_nxt = IDLE;
    case(state)
        IDLE:
            begin
                if ((proc_read || proc_write) && hit) begin
                    state_nxt = IDLE;
                end else if ((proc_read || proc_write) && !hit && !dirty) begin
                    state_nxt = ALLOCATE;
                end else if ((proc_read || proc_write) && !hit && dirty) begin
                    state_nxt = WRITE_BACK;
                end
            end
        WRITE_BACK:
            begin
                // write cache block to memory
                // cache address = {cache[idx][152:128], idx} (25 + 3 = 28)
                if (mem_ready) begin
                    state_nxt = ALLOCATE;
                end else begin
                    state_nxt = WRITE_BACK;
                end
            end
        ALLOCATE:
            // replace cache block with corresponding memory
            begin
                if (mem_ready) begin
                    state_nxt = BUFFER;
                end else begin
                    state_nxt = ALLOCATE;
                end
            end
        BUFFER:
            begin
                state_nxt = IDLE;
            end
    endcase
end

//==== combinational circuit ==============================
always@(*) begin
    // unconditional assignment
    mem_write = 0;
    mem_read = 0;
    mem_addr = 0;
    mem_wdata = 0;
    proc_stall = 0;
    proc_rdata = 0;
    for(i=0; i<8; i=i+1)
        cache_nxt[i] = cache[i];
    if ((state_DWB == IDLE) && proc_write_DWB && hit_DWB) begin
        cache_nxt[idx_DWB][153] = 1'b1;
        cache_nxt[idx_DWB][(sel_DWB+1)*32-1 -: 32] = proc_wdata_DWB;
    end

    // case
    case(state)
        IDLE:
            begin
                if (proc_read && hit) begin
                    if ((proc_write_DWB && hit_DWB) && (idx_DWB == idx) && (sel_DWB == sel))
                        proc_rdata = proc_wdata_DWB;
                    else
                        proc_rdata = cache[idx][(sel+1)*32-1 -: 32];
                // end else if (proc_write && hit) begin
                //     cache_nxt[idx][153] = 1'b1;
                //     cache_nxt[idx][(sel+1)*32-1 -: 32] = proc_wdata;
                end else if ((proc_read || proc_write) && !hit) begin  
                    // ALLOCATE, WRITE_BACK
                    if (dirty)
                        mem_write = 1;
                    else
                        mem_read = 1;
                    proc_stall = 1;
                end
            end
        WRITE_BACK:
            // write cache block to memory
            // cache address = {cache[idx][152:128], idx} (25 + 3 = 28)
            begin
                mem_write = 1;
                proc_stall = 1;
                mem_addr = {cache[idx][152:128], idx};
                mem_wdata = cache[idx][127:0];

                if (mem_ready) begin
                    mem_write = 0;
                end
            end
        ALLOCATE:
            // replace cache block with corresponding memory
            begin
                mem_read = 1;
                proc_stall = 1;
                mem_addr = {tag, idx};
            end
        BUFFER:
            begin
                proc_stall = 1;
                mem_read = 0;
                cache_nxt[idx] = {2'b10, tag, mem_rdata};
            end
    endcase
end

//==== sequential circuit =================================
always@( posedge clk ) begin
    if( proc_reset ) begin
        state <= IDLE;
        for (i=0; i<8; i=i+1)
            cache[i] <= 0;
    end
    else begin
        state <= state_nxt;
        for (i=0; i<8; i=i+1)
            cache[i] <= cache_nxt[i];
    end
end

endmodule
