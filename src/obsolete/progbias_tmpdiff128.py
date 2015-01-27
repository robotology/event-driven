#!/usr/bin/env python3

'''
    bias order for tmpdiff128
    biases have to be programmed in descending order, 
    i.e. "cas", #11, has to go first...
'''
biasorder = [
    ("cas", 11),
    ("injGnd", 10),
    ("reqPd", 9),
    ("puX", 8),
    ("diffOff", 7),
    ("req", 6),
    ("refr", 5),
    ("puY", 4),
    ("diffOn", 3),
    ("diff", 2),
    ("foll", 1),
    ("Pr", 0),
]

biasvalues = {
    "cas": 1966,
    "injGnd": 22703,
    "reqPd": 16777215,
    "puX": 4368853,
    "diffOff": 3207,
    "req": 111347,
    "refr": 0,
    "puY": 16777215,
    "diffOn": 483231,
    "diff": 28995,
    "foll": 19,
    "Pr": 8,
}

OneSecond = 1000000 # ???
timestep  = OneSecond // 100
latchexpand = 100

import sys
import array

err = sys.stderr

LATCH_KEEP = 1
LATCH_TRANSPARENT = 0

CLOCK_LO = 0
CLOCK_HI = 1

def create_uint32_array():
    tc_candidates = 'BHIL'

    for tc in tc_candidates:
        ar = array.array(tc)
        if ar.itemsize == 4:
            return ar

    raise Exception('Fatal: Unable to find a array typecode for uint32_t!')

def biasprogtx(time, latch, clock, data):
    addr = 0

    if data: addr += 0x01
    if clock: addr += 0x02
    if latch: addr += 0x04

    addr += 0xFF000000

    #addr = int(sys.argv[1], 16)

    ae = "{} {}".format(time, addr)
    print(ae, file=err)

    u32u32le = create_uint32_array()
    u32u32le.extend( [time, addr] )
    u32u32le.tofile(sys.stdout.buffer)


def progbit(bitvalue):
    # set data
    biasprogtx(timestep, LATCH_KEEP, CLOCK_LO, bitvalue)
    
    # toggle clock
    biasprogtx(timestep, LATCH_KEEP, CLOCK_HI, bitvalue)
    biasprogtx(timestep, LATCH_KEEP, CLOCK_LO, bitvalue)

def latch_commit():
    print("entering latch_commit", file=err)
    biasprogtx(timestep * latchexpand, LATCH_TRANSPARENT, CLOCK_LO, 0)
    biasprogtx(timestep * latchexpand, LATCH_KEEP, CLOCK_LO, 0)
    biasprogtx(timestep * latchexpand, LATCH_KEEP, CLOCK_LO, 0)
    print("exiting latch_commit", file=err)

def monitor(secs):
    print("entering monitor", file=err)
    biasprogtx(secs * OneSecond, LATCH_KEEP, CLOCK_LO, 0)
    print("exiting monitor", file=err)
    

def progbias(name, bits, value):

    for i in range(bits-1, -1, -1):

        if 2**i & value:
            bitvalue = 1
        else:
            bitvalue = 0

        msg = "bias:{} bit:{} value:{}".format(name, i, bitvalue)
        print(msg, file=err)

        progbit(bitvalue)

def main():
    bo = biasorder[:]
    bo.sort(key=lambda x: x[1], reverse=True)

    for (b, o) in bo:
        v = biasvalues[b]
        print( "{} {}".format(b, v), file=err )
        progbias(b, 24, v)

    latch_commit()
    monitor(10)

main()
