module CompressX(
    ins_c,
    ins_d
);
input [15:0] ins_c;
output reg [31:0] ins_d;

always @(*) begin
    case(ins_c[1:0])
        2'b00:begin
            // C0
            case(ins_c[15:13])
                3'b010: begin
                    // C.LW
                    ins_d = {5'b0,ins_c[5],ins_c[12:10],ins_c[6],2'b0,2'b01,ins_c[9:7],3'b010,2'b01,ins_c[4:2],7'b0000011};
                end
                3'b110: begin
                    // C.SW
                    ins_d = {5'b0,ins_c[5],ins_c[12],2'b01,ins_c[4:2],2'b01,ins_c[9:7],3'b010,ins_c[11:10],ins_c[6],2'b0,7'b0100011};
                end
                default: begin
                    // not exist!
                    ins_d = {32'b0};
                end
            endcase
        end
        2'b01: begin
            // C1
            case(ins_c[15:13])
                3'b000: begin
                    // NOP, C.ADDI
                    // NOP: addi, x0, x0, 0
                    // C.ADDI: addi, rd, rd, nzimm[5:0]
                    // addi!!
                    ins_d = {{7{ins_c[12]}},ins_c[6:2],ins_c[11:7],3'b000,ins_c[11:7],7'b0010011};
                end
                3'b001: begin
                    // C.JAL => jal x1, imm
                    ins_d = {1'b0,ins_c[8],ins_c[10:9],ins_c[6],ins_c[7],ins_c[2],ins_c[11],ins_c[5:3],ins_c[12],8'b0,5'b00001,7'b1101111};
                end
                3'b110: begin
                    // C.SRAI, C.SRLI, C.ANDI
                    case(ins_c[11:10]) 
                        2'b00: begin
                            // C.SRLI shamt[5] must be zero
                            ins_d = {7'b0,ins_c[6:2],2'b01,ins_c[9:7],3'b101,2'b01,ins_c[9:7],7'b0010011};
                        end
                        2'b01: begin
                            // C.SRAI
                            ins_d = {2'b01,5'b0,ins_c[6:2],2'b01,ins_c[9:7],3'b101,2'b01,ins_c[9:7],7'b0010011};
                        end
                        2'b10: begin
                            // C.ANDI
                            ins_d = {{7{ins_c[12]}},ins_c[6:2],2'b01,ins_c[9:7],3'b101,2'b01,ins_c[9:7],7'b0010011};
                        end
                        default: begin
                            // not exist
                            ins_d = 32'b0;
                        end
                    endcase
                end
                3'b101: begin
                    // C.J -> jal x0, offset[]
                    ins_d = {1'b0,ins_c[8],ins_c[10:9],ins_c[6],ins_c[7],ins_c[2],ins_c[11],ins_c[5:3],ins_c[12],8'b0,5'b00000,7'b1101111};
                end
                3'b110: begin
                    // C.BEQZ -> beq rs1', x0, offset
                    ins_d = {3'b0,ins_c[8],ins_c[6:5],ins_c[2],5'b0,2'b01,ins_c[9:7],3'b000,ins_c[11:10],ins_c[4:3],1'b0,7'b1100011};
                end
                3'b111: begin
                    // C.BNEZ
                    ins_d = {3'b0,ins_c[8],ins_c[6:5],ins_c[2],5'b0,2'b01,ins_c[9:7],3'b001,ins_c[11:10],ins_c[4:3],1'b0,7'b1100011};
                end
                default: begin
                    // not exist!
                    ins_d = 32'b0;
                end
            endcase
        end
        2'b10: begin
            // C2
            case(ins_c[15:13])
                3'b000: begin
                    // C.SLLI  
                    ins_d = {7'b0,ins_c[6:2],ins_c[11:7],3'b001,ins_c[11:7],7'b0010011};
                end
                3'b100: begin
                    // C.JR, C.MV, C.JALR, C.ADD
                    if (ins_c[12]) begin
                        if (ins_c[6:2] == 5'b0) begin
                            // C.JALR -> jalr x1, rs1, 0
                            ins_d = {12'b0,ins_c[11:7],3'b000,5'b00001,7'b1100111};
                        end
                        else begin
                            // C.ADD
                            ins_d = {7'b0,ins_c[6:2],ins_c[11:7],3'b000,ins_c[11:7],7'b0110011};
                        end
                    end
                    else begin
                        if (ins_c[6:2] == 5'b0) begin
                            // C.JR -> jalr x0, rs1, 0
                            ins_d = {12'b0,ins_c[11:7],3'b000,5'b00000,7'b1100111};
                        end
                        else begin
                            // C.MV -> add rd, x0, rs2
                            ins_d = {7'b0,ins_c[6:2],5'b0,3'b000,ins_c[11:7],7'b0110011};
                        end
                    end
                end
                default: ins_d = 32'b0;
            endcase
        end
        2'b11: begin
            // C3 not exist
            ins_d = 32'b0;
        end
        default: begin
            ins_d = 32'b0;
        end
    endcase
end
endmodule
