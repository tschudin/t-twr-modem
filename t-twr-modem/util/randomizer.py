#!/usr/bin/env python3

# randomizer.py


# ---------------------------------------------------------------------------

class SCRAM:

    def __init__(self):
        pass

    def next6(self): # return an integer in 0..63
        out = 0
        for i in range(6):
            out = (out << 1) | self.next1()
        return out

    def next8(self): # return an integer in 0..255
        out = 0
        for i in range(8):
            out = (out << 1) | self.next1()
        return out

    pass
    
# ---------------------------------------------------------------------------

class PN9(SCRAM): # IEEE P802.15

    def __init__(self):
        self.state = 0x1ff

    def next1(self):
        x = self.state & 0x1ff
        out = ((x >> 8) ^ (x >> 3) ) & 0x1
        self.state = (x << 1) | out
        return out

    pass

# ---------------------------------------------------------------------------

class X7_X4_X_1(SCRAM): # IEEE 802.11a/g/n scrambler

    def __init__(self, seed):
        self.state = seed

    def next1(self):
        x = self.state & 0x7f
        out = (x ^ (x >> 3) ^ (x >> 6) ) & 0x1
        self.state = (x << 1) | out
        return out

    pass

# ---------------------------------------------------------------------------

class X31_X21_X_1(SCRAM): # Unified Space Data Link (USDL) scrambler

    def __init__(self):
        self.state = 0xffffffff

    def next1(self):
        x = self.state & 0xffffffff
        out = (x ^ (x >> 1) ^ (x >> 21) ^ (x >> 31) ) & 0x1
        self.state = (x << 1) | out
        return out

# ---------------------------------------------------------------------------

class X17_X14_1(SCRAM): # CCSDS revised randomizer (2023)

    def __init__(self):
        self.state = int('11000111000111000', 2)
        print(hex(self.state))

    def next1(self):
        x = self.state & 0x01ffff
        out = x & 0x1
        self.state = (x | (( out ^ (x >> 14) ) << 17)) >> 1
        return out

# ---------------------------------------------------------------------------

if __name__ == '__main__':
    
    import sys

    print("IEEE 802.11a/g/n scrambler")

    # a start value of 47 looks good for 6-bit chunks
    # (no lines with all 0s or all 1s)
    seed = 47 if len(sys.argv) == 1 else int(sys.argv[1])

    scram = X7_X4_X_1(seed)
    cnt = 0
    bit_cnt = 0
    while True:
        n = scram.next6()
        bc = bin(n).count('1')
        bit_cnt += bc
        if cnt == 0:
            first = n
        elif n == first:
            print("    patterns repeat from here")
            print()
            r = 100*bit_cnt/6/cnt
            print(f"{bit_cnt} bits out of {6*cnt} are 1s", "({0:.4g}%)".format(r))
            break
        print("{0:-3d} {1:06b}".format(cnt, n), '**' if bc in [0,6] else '')
        cnt += 1


    # --------------------

    print()
    print("USDL scramble")
    scram = X31_X21_X_1()

    print("spec: 6d b6 d8 61 45 1f 11 f1 97 16 72 3c be 7e 00 b1 de 90 b2 62 ..")

    print("this: ", end='')
    for i in range(20):
        print('{0:02x}'.format(scram.next8()), end=' ')
    print()

    # --------------------

    print()
    print("CCSDS randomizer")
    scram = X17_X14_1()

    print("spec: 0001 1100 0111 0001 1011 1001 0001 1011 1010 1001 ..")

    print("this:", end='')
    for i in range(40):
        if i % 4 == 0:
            print(' ', end='')
        print(scram.next1(), end='')
    print()

    # --------------------

    print()
    print("PN9 whitening sequence (IEEE P802.15)")
    scram = PN9()

    print("spec: [0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 1, 0, 1, 1]")
    print("this:", [scram.next1() for i in range(30)])

# eof
