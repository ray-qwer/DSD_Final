import sys
import math
import argparse
from BitStr import BitStr

"""Define Functions"""
def readfile(fname):
    instr_list = []
    
    f = open(fname, "r")
    for line in f:
        line = line.split('//')
        if len(line) == 1: break # not instr
        instr = line[1].replace('_', '')
        instr = instr.replace(' ', '')
        instr_list.append(instr)
        
    instr_list = [x for x in instr_list if x != '']

    return instr_list


def instr_decode(curr_instr):
    """
    curr_instr: 32 bit string
    """
    Op     = curr_instr[25:32]
    rs1    = 0
    rs2    = 0
    rd     = 0
    funct3 = '0' * 3  # 3 bit string
    funct7 = '0' * 7  # 7 bit string
    imm    = '0' * 32 # 32 bit string
    shamt  = '0' * 5

    # R-type
    if Op == '0110011':
        funct7 = curr_instr[0:7]
        rs2    = int(curr_instr[7:12], 2)
        rs1    = int(curr_instr[12:17], 2)
        funct3 = curr_instr[17:20]
        rd     = int(curr_instr[20:25], 2)
        
    # I-type & load word & jalr
    elif Op == '0010011' or Op == '0000011' or Op == '1100111':
        rs1    = int(curr_instr[12:17], 2)
        funct3 = curr_instr[17:20]
        if (Op == '0010011' and (funct3 == '101' or funct3 == '001')):
            funct7 = curr_instr[0:7]
            shamt = curr_instr[7:12]
        else:
            imm  = f'{curr_instr[0]}' * 21 + curr_instr[1:12]
        rd     = int(curr_instr[20:25], 2)

    # S-type
    elif Op == '0100011':
        imm    = f'{curr_instr[0]}' * 21 + curr_instr[1:7] + curr_instr[20:24] + curr_instr[24]
        rs2    = int(curr_instr[7:12], 2)
        rs1    = int(curr_instr[12:17], 2)
        funct3 = curr_instr[17:20]

    # B-type
    elif Op == '1100011':
        imm    = f'{curr_instr[0]}' * 20 + curr_instr[24] + curr_instr[1:7] + curr_instr[20:24] + '0'
        rs2    = int(curr_instr[7:12], 2)
        rs1    = int(curr_instr[12:17], 2)
        funct3 = curr_instr[17:20]
    
    # J-type
    elif Op == '1101111':
        imm    = f'{curr_instr[0]}' * 12 + curr_instr[12:20] + curr_instr[11] + curr_instr[1:7] + curr_instr[7:11] + '0'
        rd     = int(curr_instr[20:25], 2)
    
    else:
        print("Not defined")

    return Op, rs1, rs2, rd, funct3, funct7, BitStr(bit_str=imm), BitStr(bit_str=shamt, bit_len=5, complement2=False)


def exec_instr(instr_list, curr_idx, registers):
    # get instruction
    curr_instr = instr_list[curr_idx]
    # decode instruction
    Op, rs1, rs2, rd, funct3, funct7, imm, shamt = instr_decode(curr_instr)
    
    print(f'addr:{hex(curr_idx * 4)}, instr:{hex(int(curr_instr, 2))}, PC_word:{hex(curr_idx)}, ')

    # execute instruction
    if int(curr_instr, 2) == 19: print('nop')
    # R-type
    elif Op == '0110011':
        if funct3 == '000':
            if funct7 == '0000000':
                print(f'add x{rd} x{rs1} x{rs2}')
                registers[rd] = registers[rs1] + registers[rs2]
            elif funct7 == '0100000':
                print(f'sub x{rd} x{rs1} x{rs2}')
                registers[rd] = registers[rs1] - registers[rs2]
        
        elif funct3 == '111':
            print(f'and x{rd} x{rs1} x{rs2}')
            registers[rd] = registers[rs1] & registers[rs2]

        elif funct3 == '110':
            print(f'or x{rd} x{rs1} x{rs2}')
            registers[rd] = registers[rs1] | registers[rs2]

        elif funct3 == '100':
            print(f'xor x{rd} x{rs1} x{rs2}')
            registers[rd] = registers[rs1] ^ registers[rs2]
        
        elif funct3 == '010':
            print(f'slt x{rd} x{rs1} x{rs2}')
            if registers[rs1] < registers[rs2]: registers[rd] = BitStr(value=1)
            else: registers[rd] = BitStr(value=0)
           
        else:
            print('unk')
    
    # jalr
    elif Op == '1100111':
        print(f'jalr x{rd} x{rs1} {imm.hex()}')
        registers[rd] = BitStr(value=(curr_idx + 1) * 4)
        curr_idx = (registers[rs1] + imm).dec() // 4 - 1 # -1: idx + 1 in the end

    # J-type
    elif Op == '1101111':
        print(f'jal x{rd} {hex(imm.dec() // 2)}')
        # TODO
        registers[rd] = BitStr((curr_idx + 1) * 4)
        curr_idx = curr_idx + imm.dec() // 4 - 1 # -1: idx + 1 in the end
    
    # B-type
    elif Op == '1100011':
        if funct3 == '000':
            print(f'beq x{rs1} x{rs2} {imm.hex()}')
            if registers[rs1] == registers[rs2]:
                # TODO
                curr_idx = (BitStr(value=curr_idx*4) + imm).dec() // 4 - 1  # -1: idx + 1 in the end

        elif funct3 == '001':
            print(f'bne x{rs1} x{rs2} {imm.hex()}')
            if registers[rs1] != registers[rs2]:
                # TODO
                curr_idx = (BitStr(value=curr_idx*4) + imm).dec() // 4 - 1  # -1: idx + 1 in the end
  
    # I-type
    elif Op == '0010011':
        if funct3 == '000':
            print(f'addi x{rd} x{rs1} {imm.dec()}')
            # TODO
            registers[rd] = registers[rs1] + imm
    
        elif funct3 == '010':
            print(f'slti x{rd} x{rs1} {imm.dec()}')
            # TODO
            if registers[rs1] < imm: registers[rd] = BitStr(value=1)
            else: registers[rd] = BitStr(value=0)
    
        elif funct3 == '111':
            print(f'andi x{rd} x{rs1} {imm.dec()}')
            # TODO
            registers[rd] = registers[rs1] & imm
    
        elif funct3 == '110':
            print(f'ori x{rd} x{rs1} {imm.dec()}')
            # TODO
            registers[rd] = registers[rs1] | imm
    
        elif funct3 == '100':
            print(f'xori x{rd} x{rs1} {imm.dec()}')
            # TODO
            registers[rd] = registers[rs1] ^ imm

        # S type
        elif funct3 == '101':
            # print(f'\n\n{curr_instr}\n\n')
            # print(f'\n\n{funct7}\n\n')
            if funct7 == '0000000':
                print(f'srli x{rd} x{rs1} {shamt.bit_str}')
                registers[rd] = registers[rs1] >> shamt.dec()
            elif funct7 == '0100000':
                print(f'srai x{rd} x{rs1} {shamt.bit_str}')
                registers[rd] = registers[rs1].sra(shamt.dec())
        
        elif funct3 == '001':
            print(f'slli x{rd} x{rs1} {shamt.bit_str}')
            registers[rd] = registers[rs1] << shamt.dec()
    
    # Load
    elif Op == '0000011':
        print(f'lw x{rd} x{rs1} {imm.dec()}')
        # TODO
        mem_idx = (registers[rs1] + imm).dec() // 4
        registers[rd] = D_mem[mem_idx]
        print(f"load D_mem[{mem_idx}]={D_mem[mem_idx].dec()} to x{rd}")
    
    # Store
    elif Op == '0100011':
        print(f'sw x{rs2} x{rs1} {imm.dec()}')
        # TODO
        mem_idx = (registers[rs1] + imm).dec() // 4
        D_mem[mem_idx] = registers[rs2]
        print(f"store x{rs2}={registers[rs2].dec()} to D_mem[{mem_idx}]")
    
    else:
        print('unk')

    # update curr_idx
    return curr_idx + 1


def print_reg(registers, flag):
    registers[0] = BitStr(value=0)
    # for i in range(len(registers)):
    if (flag):
        for i in range(32):
            print(f'[{i}]\t{registers[i].dec()}', end='\t')
            if (i + 1) % 4 == 0:
                print()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--fname", type=str, required=True)
    parser.add_argument("--one_at_a_time", dest="one_at_a_time", action="store_true")
    parser.add_argument("--print_reg", dest="print_reg", action="store_true")
    parser.set_defaults(one_at_a_time=False, print_reg=False)
    args = parser.parse_args()
    one_at_a_time = args.one_at_a_time
    flag = args.print_reg
    fname = args.fname
    
    # define
    registers = [BitStr(value=0)] * 32
    D_mem     = [BitStr(value=0)] * 1024

    # get all instructions
    instr_list = []
    instr_list = readfile(fname)

    # repeat check
    repeat = dict();

    index = 0
    curr_idx = 0
    while (curr_idx < len(instr_list)):
        if one_at_a_time:
            cmd = input()
            if cmd != '': break

        print(f'\n[{index}]', end=' ')
        curr_idx = exec_instr(instr_list, curr_idx, registers)

        # print registers
        print_reg(registers, flag)
        index += 1