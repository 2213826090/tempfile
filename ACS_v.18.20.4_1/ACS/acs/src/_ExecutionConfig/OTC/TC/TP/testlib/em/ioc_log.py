#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import os
import time
import threading
import serial
import re
from tools import get_config_value

class IOC(threading.Thread):

    def __init__(self, comport = None):
        if comport:
            self.comport = comport
        else:
            self.comport = get_config_value("ioc_port", "comport")
        self.log = []
        self.flag = True
        self.lock = threading.Lock()
        #self.comport = get_config_value("fan_ioc_port", "comport")
        super(IOC, self).__init__()

    def run(self):
        ser = serial.Serial(self.comport, baudrate = 115200)
        ser.setTimeout(10)
        while(self.flag):
            line = ser.readline()
            self.log.append(line)
        ser.close()

    def stop_reading(self):
        self.flag = False

    def get_log(self):
        return self.log

    def get_fan_status(self):
        board_temp = None
        ambient_temp = None
        fan_percent = None
        pattern = re.compile("\(id 0\) = (-?\d+) C;.+\(id 1\) = (-?\d+) C;.+FAN (\d+) %")
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
        for line in self.log:
            if check_str in line:
                print line
                return True
        return False

