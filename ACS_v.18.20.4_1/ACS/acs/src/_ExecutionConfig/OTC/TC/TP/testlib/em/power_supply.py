#! /usr/bin/python

import serial
import time
import sys
import os

ENV_POWER_SUPPLY = "ENV_EXT_USB_RLY08_2"
NOSERUNNER = "NOSERUNNER"

crc_table = [0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
             0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef]

baudrate = 38400

def genCRC16(ptr, length):
    crc = 0x0000;
    for i in range(length):
        da = (crc/256)/16
        crc = (crc<<4) & 0xffff
        crc^=crc_table[da^(ptr[i]/16)]
        da = (crc/256)/16
        crc = (crc<<4) & 0xffff
        crc^=crc_table[da^(ptr[i]&0x0f)]
    return crc;

class PowerSupply(object):

    def __init__(self, comport):
        self.comport = comport

    def set_power_supply_voltage(self, voltage):
        a = [0xa5, 0x5a, 0xfa, 0xfb, 0x20, 0x80, 0x02, 0, 0, 0, 0]
        V = int(voltage * 100)
        a[7] = (V >> 8) & 0xff
        a[8] = V  & 0xff
        crc = genCRC16(a[2:], 7)
        a[9] = crc >> 8
        a[10] = crc & 0xff
        string = ""
        for i in range(len(a)):
            string += chr(a[i])
        ser = serial.Serial(self.comport, baudrate)
        ser.write(string)
        time.sleep(0.1)
        ser.close()

    def set_power_supply_output(self, status):
        """
        status is the flag to set output, 0 is off, not 0 is on
        """
        if status == 0:
            string = "\xa5\x5a\xfa\xfb\x24\x80\x01\x00\x31\xc3"
        else:
            string = "\xa5\x5a\xfa\xfb\x24\x80\x01\x01\x21\xe2"
        ser = serial.Serial(self.comport, baudrate)
        ser.write(string)
        time.sleep(0.1)
        ser.close()

def get_power_supply_obj():
    if os.environ.get(NOSERUNNER):
        from testlib.common.map import DeviceMap
        mapObj = DeviceMap()
        power_supply_port = mapObj.getValue("power_supply")
        print "[info]---get power supply port by map.conf:", power_supply_port
    else:
        from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager
        io_card = EquipmentManager().get_io_cards()["IO_CARD"]
        power_supply_port = io_card.get_bench_params().get_param_value("DaXin")
        print "[info]---get power supply port by Bench:", power_supply_port
    return PowerSupply(power_supply_port)

if __name__ == "__main__":
    assert len(sys.argv) == 3
    power_supply = PowerSupply(sys.argv[1])
    power_supply.set_power_supply_voltage(float(sys.argv[2]))
    power_supply.set_power_supply_output(1)

