module i_cache(
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
    
    parameter BLK1_v = 312;
    parameter BLK1_TAG_H = 310;
    parameter BLK1_TAG_L = 285;
    parameter BLK0_v = 155;
    parameter BLK0_TAG_H = 153;
    parameter BLK0_TAG_L = 128;
    

    
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
    reg [313:0] cache    [3:0];
    reg [313:0] cache_nxt[3:0];
    reg [1:0] state, state_nxt;
    reg [25:0] tag;
    reg [1:0]  idx;
    reg [1:0]  sel;
    reg [1:0]  replace;   // select with parts of assoicated block
    reg [1:0]  sel_asso;
    reg hit, hit_1, hit_0, dirty, dirty_1, dirty_0;

    integer i;


//==== FSM                 ================================
always@(*) begin
    tag = proc_addr[29:4];
    idx = proc_addr[3:2];
    sel = proc_addr[1:0];
    hit_1 = cache[idx][BLK1_v] & (cache[idx][BLK1_TAG_H:BLK1_TAG_L] == tag);
    hit_0 = cache[idx][BLK0_v] & (cache[idx][BLK0_TAG_H:BLK0_TAG_L] == tag);
    hit = hit_1 | hit_0;

    dirty_1 = cache[idx][311];
    dirty_0 = cache[idx][154];
    dirty = dirty_1 | dirty_0;

    replace = {cache[idx][313], cache[idx][156]};
    sel_asso = {hit_1, hit_0};
end

always@(*) begin
    state_nxt = IDLE;
    case(state)
        IDLE:
            begin
                if (proc_read) begin
                    if (hit) begin
                        state_nxt = IDLE;
                    end else if (!hit) begin
                        state_nxt = ALLOCATE;
                    end
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

// ------------------
    
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

    // case
    case(state)
        IDLE:
            begin
                if (proc_read) begin
                    case(sel_asso)
                        2'b10:
                        begin
                            if (proc_read) begin
                                proc_rdata = cache[idx][(sel+1)*32-1 + 157 -: 32];
                            end
                            cache_nxt[idx][313] = 0;
                            cache_nxt[idx][156] = 1;
                        end
                        2'b01:
                        begin
                            if (proc_read) begin
                                proc_rdata = cache[idx][(sel+1)*32-1 -: 32];
                            end
                            cache_nxt[idx][313] = 1;
                            cache_nxt[idx][156] = 0;
                        end
                        2'b00:
                        begin
                            proc_stall = 1;
                            mem_read = 1;
                        end
                    endcase
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
                mem_read = 0;
                proc_stall = 1;
                case(replace)
                    2'b10: begin
                        cache_nxt[idx][313:157] = {3'b010, tag, mem_rdata};
                    end
                    2'b01: begin
                        cache_nxt[idx][156:0] = {3'b010, tag, mem_rdata};
                    end
                    default:
                        cache_nxt[idx] = cache[idx];
                endcase
            end
    endcase
end

//==== sequential circuit =================================
always@( posedge clk ) begin
    if( proc_reset ) begin
        state <= IDLE;
        for (i=0; i<8; i=i+1)
            cache[i] <= {1'b1, 313'b0};
    end
    else begin
        state <= state_nxt;
        for (i=0; i<8; i=i+1)
            cache[i] <= cache_nxt[i];
    end
end

endmodule
