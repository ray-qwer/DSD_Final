`include "compress_extention.v"
module compress_IF(
    addr,
    addrNext,
    ins_IF,
    jb,
    clk,
    rst,
    PC,
    ins_icache,
    stall,
    ICACHE_stall
);

// addr: input address, from CPU
// ins_com: compress instruction
// ins_decom: decompress instruction
// ins_icache: words from icache
// ins_IF: to IF/ID
input [31:0] addr,ins_icache;
input jb,clk,rst,ICACHE_stall;
output reg [31:0] addrNext;
output reg [31:0] ins_IF,PC;
output stall;
// reg and wire
// addr_nword: to fetch data from next addr
// store: store the backward of word
// storeIsCompress: data in store is compress instruction
// compressChecking: check if the command is compress
reg [1:0] state;
wire [31:0] addr_nword;
reg [15:0] store, storeNext,ins_com;
wire compressChecking;
wire storeIsCompress;
wire [31:0] ins_decom;

// state parameter
parameter IDLE = 2'b00;
parameter PLUS2 = 2'b01;
parameter JPLUS = 2'b10;

// assign
assign addr_nword = {{addr[31:2] + 1'b1},2'b0};
assign compressChecking = (ins_icache[25:24] != 2'b11); 
assign storeIsCompress = (store[9:8] != 2'b11);

// module
CompressX DC(.ins_cbi(ins_com), .ins_dbi(ins_decom));

// combinatial isCompress
always @(*) begin
    case (state)
        IDLE:   begin
            PC = addr;
            if (compressChecking) begin
                storeNext = ins_icache[15:0];
                ins_com = ins_icache[31:16];
                ins_IF = ins_decom;
            end
            else begin
                storeNext = 16'b0;
                ins_com = 16'b0;
                ins_IF = ins_icache;
            end
        end
        JPLUS:  begin
            PC = addr;
            if (compressChecking) begin
                storeNext = 16'b0;
                ins_com = ins_icache[15:0];
                ins_IF = ins_decom;
            end
            else begin
                // stall and to state PLUS2, which means addr not change, jb = 1'b0
                storeNext = ins_icache[15:0];
                ins_com = 16'b0;
                ins_IF = 32'b0;
            end
        end
        PLUS2:  begin
            if (storeIsCompress) begin
                PC = addr;
                storeNext = 16'b0;
                ins_com = store;
                ins_IF = ins_decom;
            end 
            else begin
                PC = addr_nword;
                storeNext = ins_icache[15:0];
                ins_com = 16'b0;
                ins_IF = {store,ins_icache[31:16]};
            end
        end
        default: begin
            PC = addr;
            storeNext = 16'b0;
            ins_com = 16'b0;
            ins_IF = 32'b0;
        end
    endcase
end
// addrNext:
always @(*) begin
    case(state)
        IDLE:   begin
            if (compressChecking)   addrNext = addr + 32'd2;
            else    addrNext = addr + 32'd4;
        end
        PLUS2:  begin
            if (storeIsCompress)    addrNext = addr + 32'd2;
            else    addrNext = addr + 32'd4;
        end
        JPLUS:  begin
            if (compressChecking) addrNext = addr + 32'd2;
            else addrNext = addr + 32'd4;
        end
    endcase
end
// stall when JPLUS
assign stall = (state == JPLUS);

// sequential
always @(posedge clk) begin
    if (!rst)begin
        store <= 16'b0;
    end 
    else if (ICACHE_stall) begin
        store <= store;
    end  
    else begin
        store <= storeNext;
    end
end



// FSM: cuz the addr wont change in pipeline(will be block), 
// and we need the addr for state change, so write FSM in combinatial
always@(*) begin
    if (addr[1]) begin
        if (jb) state = JPLUS;
        else state = PLUS2;
    end
    else state = IDLE;
end



endmodule
