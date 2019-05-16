#!/usr/bin/env python

##### imports #####
import os
import sys
from testlib.base.base_utils import get_args

##### initialization #####
globals().update(vars(get_args(sys.argv)))

##### test start #####
command = "adb shell cat /proc/cpuinfo"
result = True
r = os.popen(command)
info = r.readlines()
for line in info:
    line = line.strip("\r\n")
    line = line.split()
    for s in line:
        if s == "vmx":
            result = False

if not result:
    raise Exception("The test result did not achieve the desired results")
##### test end #####