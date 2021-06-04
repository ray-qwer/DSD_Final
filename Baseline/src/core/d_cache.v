module d_cache(
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

//==== Write Buffer        ================================
    reg [31:0] proc_wdata_DWB;
    reg [2:0] idx_DWB;
    reg [1:0] sel_DWB;
    reg [2:0] state_DWB;
    reg proc_write_DWB, hit_DWB;
    reg [1:0] sel_asso_DWB;
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
    sel_asso_DWB <= sel_asso;
end

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
                if (proc_read || proc_write) begin
                    if (hit) begin
                        state_nxt = IDLE;
                    end else if (!hit && !dirty) begin
                        state_nxt = ALLOCATE;
                    end else if (!hit && dirty) begin
                        state_nxt = WRITE_BACK;
                    end
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
    if ((state_DWB == IDLE) && proc_write_DWB) begin
        if (sel_asso_DWB == 2'b10) begin
            cache_nxt[idx_DWB][311] = 1'b1;
            cache_nxt[idx_DWB][(sel_DWB+1)*32-1 + 157 -: 32] = proc_wdata_DWB;
        end
        else if (sel_asso_DWB == 2'b01) begin
            cache_nxt[idx_DWB][154] = 1'b1;
            cache_nxt[idx_DWB][(sel_DWB+1)*32-1 -: 32] = proc_wdata_DWB;
        end
    end

    // case
    case(state)
        IDLE:
            begin
                if (proc_read || proc_write) begin
                    case(sel_asso)
                        2'b10:
                        begin
                            if (proc_read) begin
                                if ((proc_write_DWB && hit_DWB) && (idx_DWB == idx) && (sel_DWB == sel) && (sel_asso_DWB == sel_asso))
                                    proc_rdata = proc_wdata_DWB;
                                else
                                    proc_rdata = cache[idx][(sel+1)*32-1 + 157 -: 32];
                            end
                            cache_nxt[idx][313] = 0;
                            cache_nxt[idx][156] = 1;
                        end
                        2'b01:
                        begin
                            if (proc_read) begin
                                if ((proc_write_DWB && hit_DWB) && (idx_DWB == idx) && (sel_DWB == sel) && (sel_asso_DWB == sel_asso))
                                    proc_rdata = proc_wdata_DWB;
                                else
                                    proc_rdata = cache[idx][(sel+1)*32-1 -: 32];
                            end
                            cache_nxt[idx][313] = 1;
                            cache_nxt[idx][156] = 0;
                        end
                        2'b00:
                        begin
                            proc_stall = 1;
                            // if (dirty)
                            //     mem_write = 1;
                            // else
                            //     mem_read = 1;
                        end
                    endcase
                end
            end
        WRITE_BACK:
            // write cache block to memory
            // cache address = {cache[idx][152:128], idx} (25 + 3 = 28)
            begin
                mem_write = 1;
                proc_stall = 1;
                case(replace)
                    2'b10: begin
                        mem_addr = {cache[idx][310:285], idx};
                        mem_wdata = cache[idx][284:157];
                    end
                    2'b01: begin
                        mem_addr = {cache[idx][153:128], idx};
                        mem_wdata = cache[idx][127:0];
                    end
                    default: begin
                        mem_addr = 0;
                        mem_wdata = 0;
                    end
                endcase

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
