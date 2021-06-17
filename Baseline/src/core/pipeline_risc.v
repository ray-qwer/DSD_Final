module RISCV_Pipeline(clk,
            rst_n,
            //----------I cache interface-------
            ICACHE_ren,
            ICACHE_wen,
            ICACHE_addr,
            ICACHE_wdata,
            ICACHE_stall,
            ICACHE_rdata,
            //----------D cache interface-------
            DCACHE_ren,
            DCACHE_wen,
            DCACHE_addr,
            DCACHE_wdata,
            DCACHE_stall,
            DCACHE_rdata
    );

    input         clk, rst_n ;
    // for DCACHE
    input         DCACHE_stall; // stall signal from DCACHE
    output        DCACHE_wen  ; // when high, CHIP writes data to DCACHE
    output        DCACHE_ren  ; // CHIP reads data from DCACHE
    output [29:0] DCACHE_addr ; // the specific address to fetch/store data
    output [31:0] DCACHE_wdata; // data writing to DCACHE
    input  [31:0] DCACHE_rdata; // data reading from DCACHE
    // for ICACHE
    input         ICACHE_stall; // stall signal from ICACHE
    output        ICACHE_ren;   // read 
    output [29:0] ICACHE_addr ; // the fetching address of next instruction
    input  [31:0] ICACHE_rdata; // instruction reading from ICACHE
    output        ICACHE_wen;   
    output [31:0] ICACHE_wdata;

    assign ICACHE_ren = 1'b1;
    assign ICACHE_wen = 1'b0;
    assign ICACHE_wdata = 32'b0;
 
    wire ctrlSignal [0:8];
    // 0: Jalr, 1: Jal, 2: Branch, 3: MemRead, 4:MemtoReg, 5: MemWrite
    // 6: ALUSrc 7: RegWrite, 8: Jin
    wire [1:0] ALUOp; 
    wire [31:0] rData1, rData2, Imm, ALUData1, ALUData2, ALUData2_preimm, ALUout, MemAddr, wData, wDataFin;
    wire [31:0] AddrNext, AddrJalPre, AddrJalrPre, AddrFin;
    wire [3:0] alu_ctrl;
    reg [31:0] PC;
    reg [31:0] PC_next;
    reg DinI_state, DinI_state_next; // handle dstall appears while istall

    //----------pipeline registers-------
    reg [96:0] IF_ID, IF_ID_next; 
    reg [155:0] ID_EX, ID_EX_next; 
    reg [96:0] EX_MEM, EX_MEM_next; 
    reg [93:0] MEM_WB, MEM_WB_next; 

    //----------pipeline wires-------
    wire [31:0] IF_ID_PC_4, IF_ID_PC, IF_ID_ICACHE_rdata;
    wire [4:0] IF_ID_rs1, IF_ID_rs2;
    wire IF_ID_Istall;
    assign IF_ID_Istall = IF_ID[96];
    assign IF_ID_PC_4 = IF_ID[95:64];
    assign IF_ID_PC = IF_ID[63:32];
    assign IF_ID_ICACHE_rdata = IF_ID[31:0];

    wire [31:0] ID_EX_PC_4, ID_EX_rData1, ID_EX_rData2, ID_EX_Imm;
    wire [4:0] ID_EX_rs1, ID_EX_rs2, ID_EX_rd;
    wire [2:0] ID_EX_ctrl_EX;
    wire [2:0] ID_EX_ctrl_M;
    wire [1:0] ID_EX_ctrl_WB;
    wire [3:0] ID_EX_func73;
    wire ID_EX_Jin;
    assign ID_EX_Jin = ID_EX[155];
    assign ID_EX_PC_4 = ID_EX[154:123];
    assign ID_EX_func73 = ID_EX[122:119];     // func7 + func3
    assign ID_EX_ctrl_EX = ID_EX[118:116];    // ALUop, ALUSrc
    assign ID_EX_ctrl_WB = ID_EX[115:114];    // RegWrite, MemtoReg
    assign ID_EX_ctrl_M = ID_EX[113:111];     // Branch, MemRead, MemWrite
    assign ID_EX_rData1 = ID_EX[110:79];
    assign ID_EX_rData2 = ID_EX[78:47];
    assign ID_EX_Imm = ID_EX[46:15];
    assign ID_EX_rs1 = ID_EX[14:10];
    assign ID_EX_rs2 = ID_EX[9:5];
    assign ID_EX_rd = ID_EX[4:0];

    wire [31:0] EX_MEM_PC_4, EX_MEM_ALUout, EX_MEM_ALUData2; 
    wire [4:0] EX_MEM_rd;
    wire [2:0] EX_MEM_ctrl_M;
    wire [1:0] EX_MEM_ctrl_WB;
    wire EX_MEM_Jin;
    assign EX_MEM_Jin = EX_MEM[96];
    assign EX_MEM_PC_4 = EX_MEM[95:74];
    assign EX_MEM_ctrl_WB = EX_MEM[73:72];    // RegWrite, MemtoReg
    assign EX_MEM_ctrl_M = EX_MEM[71:69];     // Branch, MemRead, MemWrite
    assign EX_MEM_ALUout = EX_MEM[68:37];
    assign EX_MEM_ALUData2 = EX_MEM[36:5];
    assign EX_MEM_rd = EX_MEM[4:0];

    wire [31:0] MEM_WB_PC_4, MEM_WB_DCACHE_rdata, MEM_WB_ALUout;
    wire [4:0] MEM_WB_rd;
    wire [1:0] MEM_WB_ctrl_WB;
    wire MEM_WB_Jin;
    assign MEM_WB_Jin = MEM_WB[93];
    assign MEM_WB_PC_4 = MEM_WB[92:71];
    assign MEM_WB_ctrl_WB = MEM_WB[70:69];    // RegWrite, MemtoReg
    assign MEM_WB_DCACHE_rdata = MEM_WB[68:37];
    assign MEM_WB_ALUout = MEM_WB[36:5];
    assign MEM_WB_rd = MEM_WB[4:0];

    //----------forward wires-------
    wire [1:0] forward_ctrl_EX_1, forward_ctrl_EX_2;
    wire [1:0] forward_ctrl_ID_1, forward_ctrl_ID_2;
    wire forward_ctrl_RF_1, forward_ctrl_RF_2;

    //----------hazard wires-------
    wire load_EXuse_haz, load_IDuse_haz, ALU_IDuse_haz;
    wire [8:0] ID_EX_ctrl_next;
    wire [1:0] MEM_WB_ctrl_next;
    
    assign load_EXuse_haz = (
                            (ID_EX_ctrl_M[1]) && // previous inst.(load) has MemRead 
                            ~(ID_EX_rd == 5'b0) && 
                            (   
                                (ID_EX_rd == IF_ID_rs1) ||
                                (ID_EX_rd == IF_ID_rs2)
                            )
                            );
    assign ID_EX_ctrl_next =    (
                                load_EXuse_haz  
                                || load_IDuse_haz 
                                || ALU_IDuse_haz 
                                || ((ctrlSignal[2]) && (~ICACHE_stall) && (~DCACHE_stall))
                                || IF_ID_Istall
                                || DinI_state
                                )?                      9'b0:
                                                        {{ctrlSignal[8]}, {ctrlSignal[7]}, {ctrlSignal[6]}, {ctrlSignal[5]}, {ctrlSignal[4]}, {ctrlSignal[3]}, {ctrlSignal[2]}, {ALUOp}};
    assign MEM_WB_ctrl_next = (DCACHE_stall)?   2'b0:
                                                EX_MEM_ctrl_WB;

    assign load_IDuse_haz = (
                            (
                                (
                                    (ID_EX_ctrl_M[1]) &&
                                    ~(ID_EX_rd == 5'b0) && 
                                    (   
                                        (ID_EX_rd == IF_ID_rs1) ||
                                        (ID_EX_rd == IF_ID_rs2)
                                    )
                                ) || // previous is a load
                                (
                                    (EX_MEM_ctrl_M[1]) &&
                                    ~(EX_MEM_rd == 5'b0) && 
                                    (   
                                        (EX_MEM_rd == IF_ID_rs1) ||
                                        (EX_MEM_rd == IF_ID_rs2)
                                    )
                                ) // second previous is a load
                            ) &&
                            (ctrlSignal[2] || ctrlSignal[0]) // now branch
                            );

    assign ALU_IDuse_haz =  (
                            (ID_EX_ctrl_WB[1]) && // previous inst. has RegWrite
                            ~(ID_EX_rd == 5'b0) && 
                            (
                                (ID_EX_rd == IF_ID_rs1) ||
                                (ID_EX_rd == IF_ID_rs2)
                            ) &&
                            (ctrlSignal[2] || ctrlSignal[0]) // now branch
                            );

    // assign
    assign IF_ID_rs1 = {{IF_ID_ICACHE_rdata[11:8]}, {IF_ID_ICACHE_rdata[23]}};
    assign IF_ID_rs2 = {{IF_ID_ICACHE_rdata[0]}, {IF_ID_ICACHE_rdata[15:12]}};
    assign ALUData1 =   ((~forward_ctrl_EX_1[1]) && (~forward_ctrl_EX_1[0]))?   ID_EX_rData1:
                        ((~forward_ctrl_EX_1[1]) && (forward_ctrl_EX_1[0]))?    wData:
                        ((forward_ctrl_EX_1[1]) && (~forward_ctrl_EX_1[0]))?    ((EX_MEM_Jin)? EX_MEM_PC_4 : EX_MEM_ALUout):
                                                                                32'b0;
    assign ALUData2_preimm =    ((~forward_ctrl_EX_2[1]) && (~forward_ctrl_EX_2[0]))?   ID_EX_rData2:
                                ((~forward_ctrl_EX_2[1]) && (forward_ctrl_EX_2[0]))?    wData:
                                ((forward_ctrl_EX_2[1]) && (~forward_ctrl_EX_2[0]))?    ((EX_MEM_Jin)? EX_MEM_PC_4 : EX_MEM_ALUout):
                                                                                        32'b0;
    assign ALUData2 = (ID_EX_ctrl_EX[0])? ID_EX_Imm : ALUData2_preimm;

    assign wData = (MEM_WB_Jin)? {MEM_WB_PC_4} : wDataFin;
    assign DCACHE_wen = EX_MEM_ctrl_M[0];
    assign DCACHE_ren = EX_MEM_ctrl_M[1];
    assign DCACHE_addr = EX_MEM_ALUout[31:2];
    assign DCACHE_wdata = {{EX_MEM_ALUData2[7:0]}, {EX_MEM_ALUData2[15:8]}, {EX_MEM_ALUData2[23:16]}, {EX_MEM_ALUData2[31:24]}};
    assign wDataFin = (MEM_WB_ctrl_WB[0])? {MEM_WB_DCACHE_rdata[7:0], MEM_WB_DCACHE_rdata[15:8], MEM_WB_DCACHE_rdata[23:16], MEM_WB_DCACHE_rdata[31:24]} : MEM_WB_ALUout;
    assign ICACHE_addr = PC[31:2];

    // PC logic
    wire equal, beq, bne;
    wire [31:0] Jal_or_Jalr_data, Addr_jump, forwarded_rdata1, forwarded_rdata2;
    assign forwarded_rdata1 =   ((~forward_ctrl_ID_1[1]) && (~forward_ctrl_ID_1[0]))?   rData1:
                            ((~forward_ctrl_ID_1[1]) && (forward_ctrl_ID_1[0]))?   wData:
                            ((forward_ctrl_ID_1[1]) && (~forward_ctrl_ID_1[0]))?   ((EX_MEM_Jin)? EX_MEM_PC_4 : EX_MEM_ALUout):
                                                            32'b0;
    assign forwarded_rdata2 =   ((~forward_ctrl_ID_2[1]) && (~forward_ctrl_ID_2[0]))?   rData2:
                            ((~forward_ctrl_ID_2[1]) && (forward_ctrl_ID_2[0]))?   wData:
                            ((forward_ctrl_ID_2[1]) && (~forward_ctrl_ID_2[0]))?   ((EX_MEM_Jin)? EX_MEM_PC_4 : EX_MEM_ALUout):
                                                            32'b0;
    assign equal = (forwarded_rdata1 == forwarded_rdata2);
    assign bne = (IF_ID_ICACHE_rdata[20] && (~equal) && ctrlSignal[2]);
    assign beq = ((~IF_ID_ICACHE_rdata[20]) && equal && ctrlSignal[2]);
    assign AddrNext = PC[31:0] + 32'd4;
    // assign AddrJalPre = IF_ID_PC + Imm;
    // assign AddrJalrPre = forwarded_rdata1 + Imm;
    assign Jal_or_Jalr_data =   (ctrlSignal[0])?                forwarded_rdata1:
                                                                IF_ID_PC;
                                                                
    assign Addr_jump = Imm + Jal_or_Jalr_data;
    // assign AddrFin =    (ctrlSignal[0])?                AddrJalrPre:
    //                     (beq || ctrlSignal[1] || bne)?  AddrJalPre:
    //                                                     AddrNext;
    assign AddrFin = (ctrlSignal[8] || beq || bne)?    Addr_jump:
                                                                        AddrNext;

    // combinatial
    always @(*) begin
        if ((~load_EXuse_haz) && (~DCACHE_stall) && (~ICACHE_stall) && (~load_IDuse_haz) && (~ALU_IDuse_haz)) begin
            PC_next = AddrFin;
        end
        else begin
            PC_next = PC;
        end

        if (((~load_EXuse_haz) && (~DCACHE_stall) && (~ICACHE_stall) && (~load_IDuse_haz) && (~ALU_IDuse_haz))) begin
            IF_ID_next[95:64] = AddrNext;
            IF_ID_next[63:32] = PC;
            IF_ID_next[31:0] = ICACHE_rdata;
            if ((beq || bne || ctrlSignal[8])) begin // branch taken / Jar / Jarl flush
                IF_ID_next = 97'b0;
            end
        end
        else begin
            IF_ID_next = IF_ID;
        end

        if (((~load_EXuse_haz) && (~DCACHE_stall) && (~load_IDuse_haz) && (~ALU_IDuse_haz))) begin
            IF_ID_next[96] = ICACHE_stall;
        end
        else IF_ID_next[96] = IF_ID[96];

        if ((~DCACHE_stall)) begin
            ID_EX_next[155] = ID_EX_ctrl_next[8];
            ID_EX_next[154:123] = IF_ID_PC_4;
            ID_EX_next[122:119] = {{IF_ID_ICACHE_rdata[6]}, {IF_ID_ICACHE_rdata[22:20]}};
            ID_EX_next[118:116] = {{ID_EX_ctrl_next[1:0]}, {ID_EX_ctrl_next[6]}};
            ID_EX_next[115:114] = {{ID_EX_ctrl_next[7]}, {ID_EX_ctrl_next[4]}};
            ID_EX_next[113:111] = {{ID_EX_ctrl_next[2]}, {ID_EX_ctrl_next[3]}, {ID_EX_ctrl_next[5]}};
            ID_EX_next[110:79] = (forward_ctrl_RF_1)? wData : rData1;
            ID_EX_next[78:47] = (forward_ctrl_RF_2)? wData : rData2;    
            ID_EX_next[46:15] = Imm;
            ID_EX_next[14:10] = {{IF_ID_ICACHE_rdata[11:8]}, IF_ID_ICACHE_rdata[23]};
            ID_EX_next[9:5] = {IF_ID_ICACHE_rdata[0], {IF_ID_ICACHE_rdata[15:12]}};
            ID_EX_next[4:0] = {{IF_ID_ICACHE_rdata[19:16]}, IF_ID_ICACHE_rdata[31]};

            EX_MEM_next[96] = ID_EX_Jin;
            EX_MEM_next[95:74] = ID_EX_PC_4;
            EX_MEM_next[73:72] = ID_EX_ctrl_WB;
            EX_MEM_next[71:69] = ID_EX_ctrl_M;
            EX_MEM_next[68:37] = ALUout;
            EX_MEM_next[36:5] = ALUData2_preimm;
            EX_MEM_next[4:0] = ID_EX_rd;

            MEM_WB_next[93] = EX_MEM_Jin;
            MEM_WB_next[92:71] = EX_MEM_PC_4;
            MEM_WB_next[70:69] = MEM_WB_ctrl_next;
            MEM_WB_next[68:37] = DCACHE_rdata;
            MEM_WB_next[36:5] = EX_MEM_ALUout;
            MEM_WB_next[4:0] = EX_MEM_rd;
        end
        else begin
            ID_EX_next = ID_EX;

            EX_MEM_next = EX_MEM;

            MEM_WB_next = MEM_WB;
        end
    end

    reg CPU_state, CPU_state_next;

    always @(*) begin
        case(CPU_state)
            1'b0: begin
                if (!rst_n) CPU_state_next = 1'b0;
                else CPU_state_next = 1'b1;
            end
            1'b1: CPU_state_next = 1'b1;
        endcase
    end

    always @(*) begin
        case(DinI_state)
            1'b0: begin
                if (IF_ID_Istall && DCACHE_stall) DinI_state_next = 1'b1;
                else DinI_state_next = 1'b0;
            end
            1'b1: begin
                if (!DCACHE_stall) DinI_state_next = 1'b0;
                else DinI_state_next = 1'b1;
            end
        endcase
    end

    // sequential
    always @(posedge clk) begin
        if (!rst_n) begin
            PC <= 32'b0;
            CPU_state <= 0;
            DinI_state <= 0;
            IF_ID <= 0;
            ID_EX <= 0;
            EX_MEM <= 0;
            MEM_WB <= 0;
        end
        else begin
            if (CPU_state) begin
                PC <= PC_next;
                DinI_state <= DinI_state_next;
                IF_ID <= IF_ID_next;
                ID_EX <= ID_EX_next;
                EX_MEM <= EX_MEM_next;
                MEM_WB <= MEM_WB_next;
            end
            else begin
                PC <= 0;
                DinI_state <= 0;
                IF_ID <= 0;
                ID_EX <= 0;
                EX_MEM <= 0;
                MEM_WB <= 0;
            end
            CPU_state <= CPU_state_next;
        end
    end
    
    // module     
    Reg_File RF(.rs1(IF_ID_rs1),
                .rs2(IF_ID_rs2),
                .wData(wData),
                .rd(MEM_WB_rd),
                .rData1(rData1),
                .rData2(rData2),
                .Reg_write((MEM_WB_ctrl_WB[1])),
                .clk(clk),
                .rst_n(rst_n)
                );

    ALU alu(.rd1(ALUData1),
            .rd2(ALUData2),
            .out1(ALUout),
            .ALU_Ctrl(alu_ctrl)
            );
    
    ALU_Ctrl aluCtrl(   .ALUOp(ID_EX_ctrl_EX[2:1]),
                        .ins(ID_EX_func73),
                        .ctrl(alu_ctrl)
                        );
    
    ImmGen immGen(  .input_reg({{IF_ID_ICACHE_rdata[7:0]}, {IF_ID_ICACHE_rdata[15:8]}, {IF_ID_ICACHE_rdata[23:16]}, {IF_ID_ICACHE_rdata[31:24]}}),
                    .output_reg(Imm)
                    );

    CTRL ctrl(  .ins({IF_ID_ICACHE_rdata[30:24]}),
                .Jalr(ctrlSignal[0]),
                .Jal(ctrlSignal[1]),
                .Branch(ctrlSignal[2]),
                .MemRead(ctrlSignal[3]),
                .MemtoReg(ctrlSignal[4]),
                .ALUOp(ALUOp),
                .MemWrite(ctrlSignal[5]),
                .ALUSrc(ctrlSignal[6]),
                .RegWrite(ctrlSignal[7]),
                .Jin(ctrlSignal[8])
                );
    
    Forward forward(.ID_EX_rs1(ID_EX_rs1),
                    .ID_EX_rs2(ID_EX_rs2),
                    .IF_ID_rs1(IF_ID_rs1),
                    .IF_ID_rs2(IF_ID_rs2),
                    .EX_MEM_rd(EX_MEM_rd),
                    .MEM_WB_rd(MEM_WB_rd),
                    .MEM_WB_RegWrite(MEM_WB_ctrl_WB[1]),
                    .EX_MEM_RegWrite(EX_MEM_ctrl_WB[1]),
                    .forward_ctrl_EX_1(forward_ctrl_EX_1), 
                    .forward_ctrl_EX_2(forward_ctrl_EX_2),
                    .forward_ctrl_ID_1(forward_ctrl_ID_1),
                    .forward_ctrl_ID_2(forward_ctrl_ID_2),
                    .forward_ctrl_RF_1(forward_ctrl_RF_1),
                    .forward_ctrl_RF_2(forward_ctrl_RF_2)
                    );

endmodule

module Reg_File( rs1,
                rs2,
                wData,
                rd,
                rData1,
                rData2,
                Reg_write,
                clk,
                rst_n
    );

    input [4:0] rs1,rs2,rd;
    input [31:0] wData;
    input Reg_write,clk,rst_n;
    output [31:0] rData1,rData2;
    reg [31:0] Rreg [0:31];
    reg [31:0] Wreg [0:31];
    integer i ;
    // assign rData1 = ((rd == rs1) && Reg_write)? wData : Rreg[rs1]; //
    // assign rData2 = ((rd == rs2) && Reg_write)? wData : Rreg[rs2]; // assume write and read same cycle to avoid forward
    assign rData1 = Rreg[rs1];
    assign rData2 = Rreg[rs2];
    always @(*) begin
      for(i=0;i<32;i=i+1)begin
        Wreg[i] = Rreg[i];  
      end
      if (Reg_write) Wreg[rd] = wData;
    end
    always @(posedge clk) begin
        if (!rst_n) begin
            for (i=0;i<32;i=i+1) begin
                Rreg[i] <= 32'b0;
            end
        end
        else begin
            Rreg[0] <= 32'b0;
            for(i=1;i<32;i=i+1)begin
                Rreg[i] <= Wreg[i];
            end
        end
    end

endmodule

module ALU( rd1,
            rd2,
            out1,
            ALU_Ctrl,
    );

    input [31:0] rd1,rd2;
    input [3:0] ALU_Ctrl;
    output reg [31:0] out1;
    wire [31:0] andWire, orWire, addWire, subWire, sltWire,logicShiftLeftWire;
    wire [31:0] logicShiftRightWire, arithShiftRightWire,xorWire;
    assign andWire = rd1 & rd2;
    assign orWire = rd1 | rd2;
    assign addWire = $signed(rd1) + $signed(rd2);
    assign subWire = $signed(rd1) - $signed(rd2);
    assign sltWire = subWire[31];
    assign logicShiftLeftWire = rd1 << rd2[4:0];
    assign logicShiftRightWire = rd1 >> rd2[4:0];
    assign arithShiftRightWire = $signed(rd1) >>> rd2[4:0];
    assign xorWire = rd1 ^ rd2;
    always @(*) begin
        case(ALU_Ctrl)
            4'b0000: out1 = andWire;
            4'b0001: out1 = orWire;
            4'b0010: out1 = addWire;
            4'b0011: out1 = logicShiftLeftWire; // new
            4'b0100: out1 = logicShiftRightWire; // new
            4'b0101: out1 = arithShiftRightWire; // new
            4'b0110: out1 = subWire;
            4'b0111: out1 = sltWire;
            4'b1000: out1 = xorWire; // new
            default: out1 = 32'b0;
        endcase
    end

endmodule

module CTRL(ins,
            Jalr,
            Jal,
            Branch,
            MemRead,
            MemtoReg,
            ALUOp,
            MemWrite,
            ALUSrc,
            RegWrite,
            Jin
    );

    input [6:0] ins;
    output reg Jin,Jal,Jalr,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite;
    output reg [1:0] ALUOp;
    
    always @(*)begin
        {Jal,Jalr,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,Jin} = 9'b0;
        ALUOp = 2'b0;
        case(ins[6:2]) 
            5'b01100: begin                // R type
                        ALUOp = ins[4:3];
                        RegWrite = 1'b1;
                        end
            5'b01000: begin
                        MemWrite = 1'b1;     // StoreWord
                        ALUSrc = 1'b1;
                        end
            5'b00000: begin
                        MemRead = 1'b1;      // LoadWord
                        MemtoReg = 1'b1;
                        ALUSrc = 1'b1;
                        RegWrite = 1'b1;
                        end
            5'b11011: begin               // jal
                        Jal = 1'b1;
                        Jin = 1'b1;
                        RegWrite = 1'b1;
                        end
            5'b11001: begin               // jalr
                        Jalr = 1'b1;
                        Jin = 1'b1;
                        RegWrite = 1'b1;
                        ALUSrc = 1'b1;
                        end
            5'b11000: begin               // beq & bne
                        ALUOp = ins[4:3];
                        Branch = 1'b1;
                        end
            5'b00100: begin // I type
                        ALUOp = 2'b11;
                        RegWrite = 1'b1;
                        ALUSrc = 1'b1;
                        end
        endcase
        if (ins[1:0] == 2'b0) begin
            {Jal,Jalr,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,Jin} = 9'b0;
            ALUOp = 2'b0;
        end
    end 
endmodule

module ImmGen(  input_reg,
                output_reg
    );

    input [31:0] input_reg;
    output reg [31:0] output_reg;
    always @(*) begin
        output_reg = 32'b0;
        case(input_reg[6:2])        // opcode
            5'b00000: begin       // I type
                        output_reg = {{20{input_reg[31]}},{input_reg[31:20]}};
                        end
            5'b11001: begin       // I type
                        output_reg = {{20{input_reg[31]}},{input_reg[31:20]}};
                        end
            5'b01000: begin       // S type
                        output_reg = {{20{input_reg[31]}},{input_reg[31:25]},{input_reg[11:7]}};
                        end
            5'b11000: begin       // B type
                        output_reg = {{20{input_reg[31]}},{input_reg[7]},{input_reg[30:25]},{input_reg[11:8]},1'b0};
                        end
            5'b11011: begin       // J type
                        output_reg = {{20{input_reg[31]}},{input_reg[19:12]},{input_reg[20]},{input_reg[30:21]},1'b0};
                        end
            5'b00100: begin
                        output_reg = {{20{input_reg[31]}},{input_reg[31:20]}};
                        end
        endcase
    end
endmodule

module ALU_Ctrl(ALUOp,
                ins,
                ctrl,
                );
    input [3:0] ins;
    input [1:0] ALUOp;
    output reg [3:0] ctrl;
    always @(*) begin
        case({ins[2:0],ALUOp[1]})
            4'b0001: begin
                        if (~ins[3] || ALUOp[0]) ctrl = 4'b0010; //add
                        else ctrl = 4'b0110;        //sub
            end 
            4'b1111: ctrl = 4'b0000;               // and
            4'b1101: ctrl = 4'b0001;               // or
            4'b0101: ctrl = 4'b0111;               // slt
            4'b1001: ctrl = 4'b1000;               // xor
            4'b1011: begin
                        if (ins[3]) ctrl = 4'b0101; // srai
                        else ctrl = 4'b0100;        // srli
            end
            4'b0011: ctrl = 4'b0011;               // slli
            default: ctrl = 4'b0010;
        endcase
    end
endmodule

module Forward( ID_EX_rs1,
                ID_EX_rs2,
                IF_ID_rs1,
                IF_ID_rs2,
                EX_MEM_rd,
                MEM_WB_rd,
                MEM_WB_RegWrite,
                EX_MEM_RegWrite,
                forward_ctrl_EX_1, 
                forward_ctrl_EX_2,
                forward_ctrl_ID_1, 
                forward_ctrl_ID_2,
                forward_ctrl_RF_1,
                forward_ctrl_RF_2
    );

    input [4:0] ID_EX_rs1;
    input [4:0] ID_EX_rs2;
    input [4:0] IF_ID_rs1;
    input [4:0] IF_ID_rs2;
    input [4:0] EX_MEM_rd;
    input [4:0] MEM_WB_rd;
    input MEM_WB_RegWrite, EX_MEM_RegWrite;
    output [1:0] forward_ctrl_EX_1; 
    output [1:0] forward_ctrl_EX_2;
    output [1:0] forward_ctrl_ID_1; 
    output [1:0] forward_ctrl_ID_2;
    output forward_ctrl_RF_1, forward_ctrl_RF_2;

    // 00: normal, 01: WB, 10: MEM
    assign forward_ctrl_EX_1 =  (EX_MEM_RegWrite && ~(EX_MEM_rd == 5'b0) && (EX_MEM_rd == ID_EX_rs1))? 2'b10:
                                (MEM_WB_RegWrite && ~(MEM_WB_rd == 5'b0) && (MEM_WB_rd == ID_EX_rs1) && ~(EX_MEM_RegWrite && ~(EX_MEM_rd == 5'b0) && (EX_MEM_rd == ID_EX_rs1)))? 2'b01:
                                2'b00;
    
    assign forward_ctrl_EX_2 =  (EX_MEM_RegWrite && ~(EX_MEM_rd == 5'b0) && (EX_MEM_rd == ID_EX_rs2))? 2'b10:
                                (MEM_WB_RegWrite && ~(MEM_WB_rd == 5'b0) && (MEM_WB_rd == ID_EX_rs2) && ~(EX_MEM_RegWrite && ~(EX_MEM_rd == 5'b0) && (EX_MEM_rd == ID_EX_rs2)))? 2'b01:
                                2'b00;

    assign forward_ctrl_ID_1 =  (EX_MEM_RegWrite && ~(EX_MEM_rd == 5'b0) && (EX_MEM_rd == IF_ID_rs1))? 2'b10:
                                (MEM_WB_RegWrite && ~(MEM_WB_rd == 5'b0) && (MEM_WB_rd == IF_ID_rs1) && ~(EX_MEM_RegWrite && ~(EX_MEM_rd == 5'b0) && (EX_MEM_rd == IF_ID_rs1)))? 2'b01:
                                2'b00;

    assign forward_ctrl_ID_2 =  (EX_MEM_RegWrite && ~(EX_MEM_rd == 5'b0) && (EX_MEM_rd == IF_ID_rs2))? 2'b10:
                                (MEM_WB_RegWrite && ~(MEM_WB_rd == 5'b0) && (MEM_WB_rd == IF_ID_rs2) && ~(EX_MEM_RegWrite && ~(EX_MEM_rd == 5'b0) && (EX_MEM_rd == IF_ID_rs2)))? 2'b01:
                                2'b00;

    assign forward_ctrl_RF_1 =  (MEM_WB_RegWrite && ~(MEM_WB_rd == 5'b0) && (MEM_WB_rd == IF_ID_rs1));
    assign forward_ctrl_RF_2 =  (MEM_WB_RegWrite && ~(MEM_WB_rd == 5'b0) && (MEM_WB_rd == IF_ID_rs2));

endmodule
