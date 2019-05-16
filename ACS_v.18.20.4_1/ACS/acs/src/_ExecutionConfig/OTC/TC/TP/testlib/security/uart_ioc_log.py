#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
import threading
import serial
import re
from flash_image import FlashImage

NOSERUNNER = "NOSERUNNER"

class GetPort(FlashImage):
    def __init__(self):
        FlashImage.__init__(self)

    def get_device_serial(self):
        dsn = self.dsn
        return dsn

    def get_debug_port(self, port_name):
        #return port str,  IOC: num=-2; serial: -1
        if port_name == "ioc":
            ioc_dir = self.get_port(-2)
            ioc_port = ioc_dir.get(self.dsn)
            return ioc_port
        elif port_name == "serial":
            serial_dir = self.get_port(-1)
            serial_port = serial_dir.get(self.dsn)
            return serial_port
        elif port_name == "relay":
            relay_dir = self.get_port(0)
            relay_port = relay_dir.get(self.dsn)
            return relay_port
        else:
            print "[Info]------Port not IOC or serial"

    def add_access_for_debug_port(self, port):
        # os.system("echo '123456' | sudo -S chmod 777 /dev/ttyUSB4")
        cmd_aft = "sudo chmod 777 {}".format(port)
        cmd_local = "echo '123456' | sudo -S chmod 777 {}".format(port)
        if os.environ.get(NOSERUNNER) is None:
            print cmd_aft
            os.system(cmd_aft)
        else:
            print cmd_local
            os.system(cmd_local)

    def get_serial_port(self, port_name):
        #return dir
        # IOC: num=-2; serial: -1
        if port_name == "ioc":
            ioc_port = self.get_port(-2)
            return ioc_port
        elif port_name == "serial":
            serial_port = self.get_port(-1)
            return serial_port
        elif port_name == "relay":
            relay_port = self.get_port(0)
            return relay_port
        else:
            print "[Info]------Port not IOC or serial"



class UartIOC(threading.Thread):

    def __init__(self, port_name = "ioc"):
        #port_name: {'ioc', 'serial', 'relay'}
        self.get_port = GetPort()
        self.comport = self.get_port.get_debug_port(port_name)
        self.get_port.add_access_for_debug_port(self.comport)
        self.log = []
        self.flag = True
        self.lock = threading.Lock()
        super(UartIOC, self).__init__()

    def run(self):
        ser = serial.Serial(self.comport, baudrate = 115200)
        ser.setTimeout(10)
        time.sleep(5)
        while(self.flag):
            line = ser.readline()
            self.log.append(line)
        ser.close()

    def ioc_reboot(self):
        comport = self.get_port.get_debug_port('ioc')
        self.get_port.add_access_for_debug_port(comport)
        ser_ioc = serial.Serial(comport, baudrate=115200)
        time.sleep(3)
        ser_ioc.write("r")
        time.sleep(2)
        ser_ioc.write("g")
        #O image, [9, 35] boot time too long, process will be block
        time.sleep(28)
        ser_ioc.setTimeout(10)
        ser_ioc.close()

    def ioc_reboot_quickly(self):
        comport = self.get_port.get_debug_port('ioc')
        self.get_port.add_access_for_debug_port(comport)
        ser_q = serial.Serial(comport, baudrate=115200)
        time.sleep(3)
        ser_q.write("r")
        time.sleep(2)
        ser_q.write("g")
        #For M image, wait time[6,7]
        time.sleep(8)
        ser_q.setTimeout(10)
        ser_q.close()

    def stop_reading(self):
        self.flag = False

    def get_log(self):
        return self.log

    def get_temp_fan_status(self):
        board_temp = None
        ambient_temp = None
        fan_percent = None
        pattern = re.compile("\(id 0\) = (\d+) C;.+\(id 1\) = (\d+) C;.+FAN (\d+) %")
        for _ in range(10):
            if not len(self.log):
                time.sleep(3)
                continue
            for i in range(len(self.log)):
                result = pattern.search(self.log[0])
                if result:
                    board_temp = result.group(1)
                    ambient_temp = result.group(2)
                    fan_percent = result.group(3)
                    return int(board_temp), int(ambient_temp), int(fan_percent)
                self.log.pop(0)
        assert False, "Get Fan status failed"

    def check_string_in_log(self, check_str):
        #return Ture / False
        result = True
        for _ in range(10):
            if len(self.log):
                time.sleep(1)
                break
            else:
                time.sleep(2)
                result = False
        for line in self.log:
            if check_str in line:
                print line
                return True
        if result == False:
            print "[Info]------Check string not found from minicom log"
        return result

    def check_string_before_get_key_log(self, check_str):
        # return test_log
        get_key_log = []
        result = True
        for _ in range(10):
            if len(self.log):
                time.sleep(2)
                break
            else:
                time.sleep(2)
                result = False
        if result == False:
            print "[Info]------Check string not found from minicom log"
            return result
        for line in self.log:
            if check_str in line:
                print line
                get_key_log = self.log
                break
        return get_key_log

