# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import sys


def find_pattern(data, pattern):
    for i in range(len(data)):
        j = i + len(pattern)
        if j > len(data):
            return None
        if data[i:j].decode('utf-8', 'ignore') == pattern:
            return i, j

def find_frame(data, start_pattern, end_pattern, frame_size):
    result = find_pattern(data, start_pattern)
    if result == None:
        return None, None
    i1, j1 = result
    
    result = find_pattern(data[j1:], end_pattern)
    if result == None:
        return None, None
    i2, j2 = result
    
    i2 += j1
    j2 += j1
    
    print(i1, j1, i2, j2, i2 - j1 == frame_size)
    return data[j1:i2], data[j2:]


if __name__ == '__main__':
    if len(sys.argv) >= 3:
        input_file = sys.argv[1]
        output_file = sys.argv[2]
        print('input file: %s' % input_file)
        print('output file: %s' % output_file)
    else:
        print('Please specify input file and output file!')
        sys.exit(1)
    
    with open(input_file, 'rb') as fr:
        log_data = fr.read()
        raw_data = bytearray()
        
        start_pattern = 'rawdata start' + '\x0D\x0A'
        end_pattern = '\x0D\x0A' + 'rawdata end' + '\x0D\x0A'
        frame_size = 308*2
        
        while True:
            frame, log_data = find_frame(log_data, start_pattern, end_pattern, frame_size)
            if frame != None:
                raw_data += frame
            else:
                break
        
    with open(output_file, 'wb') as fw:
        fw.write(raw_data)
    
    sys.exit(0)
