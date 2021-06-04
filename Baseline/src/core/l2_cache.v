module l2_cache(
    // synchronize
    input clk,
    input proc_reset,
    // bewteen L1 i cache
    input            l1i_read,
    input            l1i_write,
    input [27:0]     l1i_addr,
    input [127:0]    l1i_wdata,
    output reg[127:0]l1i_rdata,
    output reg       l1i_ready,
    // bewteen L1 d cache
    input            l1d_read,
    input            l1d_write,
    input [27:0]     l1d_addr,
    input [127:0]    l1d_wdata,
    output reg[127:0]l1d_rdata,
    output reg       l1d_ready,
    // between Slow Memory i
    output reg       memi_read,
    output reg       memi_write,
    output reg[27:0] memi_addr,
    output reg[127:0]memi_wdata,
    input     [127:0]memi_rdata,
    input            memi_ready,
    // between Slow Memory d
    output reg       memd_read,
    output reg       memd_write,
    output reg[27:0] memd_addr,
    output reg[127:0]memd_wdata,
    input     [127:0]memd_rdata,
    input            memd_ready
);
    parameter IDLE =         3'd0;
    parameter WRITE_BACK =   3'd1;
    parameter ALLOCATE =     3'd2;
    parameter BUFFER =       3'd3;
    parameter ACCESS_CACHE = 3'd4;

    parameter BLK1_v = 312;
    parameter BLK1_TAG_H = 310;
    parameter BLK1_TAG_L = 285;
    parameter BLK0_v = 155;
    parameter BLK0_TAG_H = 151;
    parameter BLK0_TAG_L = 128;

// ====  wire definition  ===============
    reg [152:0] cache    [63:0];
    reg [152:0] cache_nxt[63:0];
    reg [2:0]  i_state, i_state_nxt, d_state, d_state_nxt;
    reg [22:0] i_tag, d_tag;    // length 23
    reg [5:0]  i_idx, d_idx;    // length 5
    reg i_hit, i_dirty;
    reg d_hit, d_dirty;

    reg [127:0] l1i_rdata_nxt, l1d_rdata_nxt;

    integer i;

always@(*) begin
    i_tag = l1i_addr[27:5];
    i_idx = {1'b0, l1i_addr[4:0]};
    i_hit = cache[i_idx][152] & (cache[i_idx][150:128] == i_tag);
    i_dirty = cache[i_idx][151];

    d_tag = l1d_addr[27:5];
    d_idx = {1'b1, l1d_addr[4:0]};
    d_hit = cache[d_idx][152] & (cache[d_idx][150:128] == d_tag);
    d_dirty = cache[d_idx][151];
end

// ====       FSM        ===============
always@(*) begin
    i_state_nxt = IDLE;
    case(i_state)
        IDLE:
            begin
                if ((l1i_read || l1i_write) && i_hit) begin
                    i_state_nxt = ACCESS_CACHE;
                end else if ((l1i_read || l1i_write) && !i_hit && !i_dirty) begin
                    i_state_nxt = ALLOCATE;
                end else if ((l1i_read || l1i_write) && !i_hit && i_dirty) begin
                    i_state_nxt = WRITE_BACK;
                end
            end
        ACCESS_CACHE:
            begin
                i_state_nxt = IDLE;
            end
        WRITE_BACK:
            begin
                // write cache block to memory
                if (memi_ready) begin
                    i_state_nxt = ALLOCATE;
                end else begin
                    i_state_nxt = WRITE_BACK;
                end
            end
        ALLOCATE:
            // replace cache block with corresponding memory
            begin
                if (memi_ready) begin
                    i_state_nxt = BUFFER;
                end else begin
                    i_state_nxt = ALLOCATE;
                end
            end
        BUFFER:
            begin
                i_state_nxt = IDLE;
            end
    endcase
end

always@(*) begin
    d_state_nxt = IDLE;
    case(d_state)
        IDLE:
            begin
                if ((l1d_read || l1d_write) && d_hit) begin
                    d_state_nxt = ACCESS_CACHE;
                end else if ((l1d_read || l1d_write) && !d_hit && !d_dirty) begin
                    d_state_nxt = ALLOCATE;
                end else if ((l1d_read || l1d_write) && !d_hit && d_dirty) begin
                    d_state_nxt = WRITE_BACK;
                end
            end
        ACCESS_CACHE:
            begin
                i_state_nxt = IDLE;
            end
        WRITE_BACK:
            begin
                // write cache block to memory
                if (memd_ready) begin
                    d_state_nxt = ALLOCATE;
                end else begin
                    d_state_nxt = WRITE_BACK;
                end
            end
        ALLOCATE:
            // replace cache block with corresponding memory
            begin
                if (memd_ready) begin
                    d_state_nxt = BUFFER;
                end else begin
                    d_state_nxt = ALLOCATE;
                end
            end
        BUFFER:
            begin
                d_state_nxt = IDLE;
            end
    endcase
end

// ==== Combinational Circuit ===========
// Instr.
always@(*) begin
    // unconditional assignment
    memi_write = 0;
    memi_read = 0;
    memi_addr = 0;
    memi_wdata = 0;
    l1i_ready = 0;
    l1i_rdata_nxt = 0;
    
    memd_write = 0;
    memd_read = 0;
    memd_addr = 0;
    memd_wdata = 0;
    l1d_ready = 0;
    l1d_rdata_nxt = 0;
    for(i=0; i<=63; i=i+1)
        cache_nxt[i] = cache[i];

    case(i_state)
        IDLE:
            begin
                if ((l1i_read || l1i_write) && !i_hit) begin  
                    // ALLOCATE, WRITE_BACK
                    if (i_dirty)
                        memi_write = 1;
                    else
                        memi_read = 1;
                end
            end
        ACCESS_CACHE:
            begin
                if (l1i_read) begin
                    l1i_rdata_nxt = cache[i_idx][127:0];
                end else if (l1i_write) begin
                    cache_nxt[i_idx][153] = 1'b1;
                    cache_nxt[i_idx][127:0] = l1i_wdata;
                end
                l1i_ready = 1;
            end
        WRITE_BACK:
            begin
                memi_write = 1;
                memi_addr = {cache[i_idx][150:128], i_idx};
                memi_wdata = cache[i_idx][127:0];

                if (memi_ready) begin
                    memi_write = 0;
                end
            end
        ALLOCATE:
            // replace cache block with corresponding memory
            begin
                memi_read = 1;
                memi_addr = {i_tag, i_idx[4:0]};
            end
        BUFFER:
            begin
                memi_read = 0;
                cache_nxt[i_idx] = {2'b10, i_tag, memi_rdata};
            end
    endcase

    case(d_state)
        IDLE:
            begin
                if ((l1d_read || l1d_write) && !d_hit) begin  
                    // ALLOCATE, WRITE_BACK
                    if (d_dirty)
                        memd_write = 1;
                    else
                        memd_read = 1;
                end
            end
        ACCESS_CACHE:
            begin
                if (l1d_read) begin
                    l1d_rdata_nxt = cache[d_idx][127:0];
                end else if (l1d_write) begin
                    cache_nxt[d_idx][153] = 1'b1;
                    cache_nxt[d_idx][127:0] = l1d_wdata;
                end
                l1d_ready = 1;
            end
        WRITE_BACK:
            begin
                memd_write = 1;
                memd_addr = {cache[d_idx][150:128], d_idx};
                memd_wdata = cache[d_idx][127:0];

                if (memd_ready) begin
                    memd_write = 0;
                end
            end
        ALLOCATE:
            // replace cache block with corresponding memory
            begin
                memd_read = 1;
                memd_addr = {d_tag, d_idx[4:0]};
            end
        BUFFER:
            begin
                memd_read = 0;
                cache_nxt[d_idx] = {2'b10, d_tag, memd_rdata};
            end
    endcase
end

// ==== Sequential Circuit ==============
always@( posedge clk ) begin
    if( proc_reset ) begin
        i_state <= IDLE;
        d_state <= IDLE;
        for (i=0; i<=63; i=i+1)
            cache[i] <= 0;
        l1i_rdata <= 0;
        l1d_rdata <= 0;
    end
    else begin
        i_state <= i_state_nxt;
        d_state <= d_state_nxt;
        for (i=0; i<=64; i=i+1)
            cache[i] <= cache_nxt[i];
        l1i_rdata <= l1i_rdata_nxt;
        l1d_rdata <= l1d_rdata_nxt;
    end
end
endmodule