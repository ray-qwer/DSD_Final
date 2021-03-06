import math

class BitStr():
# === initialize class ===
    def __init__(self, value=None, bit_str=None, bit_len=32, complement2=True):
        """
        initialize BitStr
        args: parse in wither value or bit_str 
            value: decimal value and convert to bit string
            bit_str: bit string, length < bit_len
        """
        self.bit_len = bit_len
        self.complement2 = complement2
        if value == None:
            self.bit_str = '0' * (self.bit_len - len(bit_str)) + bit_str
        else: # convert with 2's complement
            # avoid overload
            if abs(value) >= math.pow(2, 16):
                value = int(value % math.pow(2, 16))
            self.bit_str = self.dec2complement2(value)
    
# === operator overloading ===
    # ~BitStr
    def __invert__(self):
        return BitStr(bit_str=self.negate(self.bit_str))

    # BitStr + BitStr or BitStr + int
    def __add__(self, other): # can add integer for addi
        if isinstance(other, int):
            value = self.dec() + other
        else: 
            value = self.dec() + other.dec()
        return BitStr(value=value)

    # BitStr - BitStr or BitStr - int
    def __sub__(self, other):
        if isinstance(other, int):
            value = self.dec() - other
        else: 
            value = self.dec() - other.dec()
        return BitStr(value=value)

    # BitStr * BitStr or BitStr * int and return HI and LO
    def __mul__(self, other):
        if isinstance(other, int):
            value = self.dec() * other
        else: 
            value = self.dec() * other.dec()

        mult_bit_str = self.dec2complement2(num=value, bit_len=self.bit_len * 2)
        
        HI = BitStr(bit_str=mult_bit_str[0:self.bit_len])
        LO = BitStr(bit_str=mult_bit_str[self.bit_len:self.bit_len * 2])

        return HI, LO 

    # BitStr / BitStr or BitStr / int and return HI and LO
    def __truediv__(self, other):
        if isinstance(other, int):
            quotient = self.dec() // other
            remainder = self.dec() % other
        else: 
            quotient = self.dec() // other.dec()
            remainder = self.dec() % other.dec()

        HI = BitStr(value=remainder)
        LO = BitStr(value=quotient)
        
        return HI, LO

    # BitStr < BitStr or BitStr < int
    def __lt__(self, other):
        if isinstance(other, int):
            return self.dec() < other
        else: 
            return self.dec() < other.dec()

    # BitStr > BitStr or BitStr > int
    def __gt__(self, other):
        if isinstance(other, int):
            return self.dec() > other
        else: 
            return self.dec() > other.dec()

    # BitStr == BitStr or BitStr == int
    def __eq__(self, other):
        if isinstance(other, int):
            return self.dec() == other
        else: 
            return self.dec() == other.dec()

    # BitStr != BitStr or BitStr != int
    def __ne__(self, other):
        if isinstance(other, int):
            return self.dec() != other
        else: 
            return self.dec() != other.dec()

    # BitStr << int
    def __lshift__(self, sh):
        shifted_bit_str = ''
        for i in range(self.bit_len):
            idx = i + sh
            if idx >= self.bit_len:
                shifted_bit_str += '0'
            else:
                shifted_bit_str += self.bit_str[idx]

        return BitStr(bit_str=shifted_bit_str)

    # BitStr >> int
    def __rshift__(self, sh):
        shifted_bit_str = ''
        for i in range(self.bit_len):
            idx = i - sh
            if idx < 0:
                shifted_bit_str += '0'
            else:
                shifted_bit_str += self.bit_str[idx]

        return BitStr(bit_str=shifted_bit_str)

    # BitStr & BitStr or BitStr & int
    def __and__(self, other):
        if isinstance(other, int):
            other_bit_str = BitStr(value=other).bin()
        else: 
            other_bit_str = other.bin()
        
        and_bit_str = ''
        for i in range(self.bit_len):
            if self.bit_str[i] == '1' and other_bit_str[i] == '1':
                and_bit_str += '1'
            else:
                and_bit_str += '0'

        return BitStr(bit_str=and_bit_str)

    # BitStr | BitStr or BitStr | int
    def __or__(self, other):
        if isinstance(other, int):
            other_bit_str = BitStr(value=other).bin()
        else: 
            other_bit_str = other.bin()
        
        or_bit_str = ''
        for i in range(self.bit_len):
            if self.bit_str[i] == '1' or other_bit_str[i] == '1':
                or_bit_str += '1'
            else:
                or_bit_str += '0'

        return BitStr(bit_str=or_bit_str)

    # BitStr ^ BitStr or BitStr ^ int
    def __xor__(self, other):
        if isinstance(other, int):
            other_bit_str = BitStr(value=other).bin()
        else: 
            other_bit_str = other.bin()
        
        xor_bit_str = ''
        for i in range(self.bit_len):
            if self.bit_str[i] == other_bit_str[i]:
                xor_bit_str += '0'
            else:
                xor_bit_str += '1'

        return BitStr(bit_str=xor_bit_str)


# === utils ===
    # transform number to 2's complement bit string
    def dec2complement2(self, num, bit_len=None):
        if bit_len == None:
            bit_len = self.bit_len

        if num >= 0:
            sub_bit_str = bin(num)[2:]
            return '0' * (bit_len - len(sub_bit_str)) + sub_bit_str
        else:
            usign_value    = -1 * num
            sub_bit_str    = bin(usign_value)[2:]
            usign_bit_str  = '0' * (bit_len - len(sub_bit_str)) + sub_bit_str
            negate_bit_str = self.negate(usign_bit_str)
            negate_dec_str = bin(int(negate_bit_str, 2) + 1)[2:]
            
            return negate_dec_str[len(negate_dec_str)-bit_len:len(negate_dec_str)]

    # input bit string, return negated bit string
    def negate(self, bit_str):
        negate_bit_str = ''
        for i in range(len(bit_str)):
            if bit_str[i] == '0':
                negate_bit_str += '1'
            else:
                negate_bit_str += '0'

        return negate_bit_str

    # shift right arithmetic
    def sra(self, sh):
        shifted_bit_str = ''
        sign_bit = self.bit_str[0]
        for i in range(self.bit_len):
            idx = i - sh
            if idx < 0:
                shifted_bit_str += sign_bit
            else:
                shifted_bit_str += self.bit_str[idx]

        return BitStr(bit_str=shifted_bit_str)


# print functions
    def dec(self):
        if self.complement2:
            sign_bit = self.bit_str[0]
            if sign_bit == '1':
                return -1 * (int(self.__invert__().bin(), 2) + 1)
            else:
                return int(self.bit_str, 2)
        else:
            return int(self.bit_str, 2)

    def bin(self):
        return self.bit_str

    def hex(self):
        return '{:0{}X}'.format(int(self.bit_str, 2), self.bit_len // 4)