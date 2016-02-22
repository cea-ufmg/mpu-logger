#!/usr/bin/env python3


import struct
import sys

import crcmod


USAGE = '''Incorrect script invocation. Usage: 
    {} <logfile>

Parse log file and print contents to STDOUT.
'''


crc8 = crcmod.mkCrcFun(0x107, initCrc=0, rev=False)
payload_fmt = '>IhhhhhhhhhhB'
payload_size = struct.calcsize(payload_fmt)


def parse(logfile):
    while True:
        header = logfile.read(1)
        if not header:
            return
        if header != b'A':
            continue

        payload = logfile.read(payload_size)
        if len(payload) != payload_size:
            return

        calc_crc = crc8(payload[:-1])
        *data, recv_crc = struct.unpack(payload_fmt, payload)
        if calc_crc != recv_crc:
            print('Corrupted entry found!', file=sys.stderr)
        else:
            print(*data, sep='\t')


if __name__ == '__main__':
    if len(sys.argv) == 2:
        logfile = open(sys.argv[1], 'rb', buffering=True)
        parse(logfile)
    else:
        print(USAGE.format(sys.argv[0]), file=sys.stderr)
