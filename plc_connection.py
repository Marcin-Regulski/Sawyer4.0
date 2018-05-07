#! /usr/bin/env python

import snap7.client as c
import struct
from snap7.util import *
from snap7.snap7types import *
import rospy
import sys

def ReadMemory(plc,byte,bit,datatype):
    result = plc.read_area(areas['MK'],0,byte,datatype)
    value = 0
    if datatype == S7WLBit:
        return get_bool(result,0,1)
    elif datatype == S7WLByte or datatype == S7WLWord:
        return get_int(result,0)
    elif datatype == S7WLReal:
        return get_real(result,0)
    elif datatype == S7WLWord:
        return get_dword(result,0)
    else:
        return None

# bit - which bit in the memory byte we are reading
# length - length in bytes of the read
def Read_Input(plc,byte,bit,length):
    result = plc.read_area(0x81,0,byte,length)
    return get_bool(result,0,bit)

def Read_Output(plc,byte,bit,length):
    result = plc.read_area(0x80,0,byte,length)
    return get_bool(result,0,bit)

def WriteMemory(plc,byte,bit,datatype,value):
    result = plc.read_area(areas['MK'],0,byte,datatype)
    if datatype == S7WLBit:
        set_bool(result,0,bit,value)
    elif datatype == S7WLByte or datatype == S7WLWord:
        set_int(result,0,value)
    elif datatype == S7WLReal:
        set_real(result,0,value)
    elif datatype == S7WLDWord:
        set_dword(result,0,value)
    plc.write_area(areas['MK'],0,byte,result)

##Specially for Sawyer purposes

def belt_off():
    plc = c.Client()
    plc.connect('172.21.2.1',0,1)
    result = plc.read_area(0x82,0,1,1)
    return get_bool(result,0,4)

def workpiece_avaiable():
    plc = c.Client()
    plc.connect('172.21.2.1',0,1)
    result = plc.read_area(0x81,0,1,1)
    return get_bool(result,0,4)

def process_ongoing():
    plc = c.Client()
    plc.connect('172.21.2.1',0,1)
    WriteMemory(plc,99,0,S7WLBit,0)

def process_finished():
    plc = c.Client()
    plc.connect('172.21.2.1',0,1)
    WriteMemory(plc,99,0,S7WLBit,1)
    rospy.sleep(4.0)
    WriteMemory(plc,99,0,S7WLBit,0)
    plc.disconnect

def main():
    while True:
        plc = c.Client()
        plc.connect('172.21.2.1',0,1)
        return "Yup"
        result = plc.read_area(0x81,0,1,1)
        return get_bool(result,0,4)
        rospy.sleep(1.0)

if __name__ == '__main__':
    sys.exit(main())