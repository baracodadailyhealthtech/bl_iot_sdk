# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import sys

if __name__ == '__main__':
    if len(sys.argv) >= 3:
        input_file = sys.argv[1]
        output_file = sys.argv[2]
        print('input file: %s' % input_file)
        print('output file: %s' % output_file)
    else:
        print('Please specify input and output file!')
        sys.exit(1)

    with open(input_file, 'rb') as f:
        ota_bin = f.read()

    ota_size = len(ota_bin)
    print('ota size: %d 0x%08x' % (ota_size, ota_size))

    head = bytearray(4)
    head[0] = ota_size & 0xFF
    head[1] = (ota_size >> 8) & 0xFF
    head[2] = (ota_size >> 16) & 0xFF
    head[3] = (ota_size >> 24) & 0xFF

    with open(output_file, 'wb') as f:
        f.write(head + ota_bin)

    sys.exit(0)
