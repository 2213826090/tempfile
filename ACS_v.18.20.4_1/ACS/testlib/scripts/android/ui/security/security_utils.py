#!/usr/bin/env python

import os

def get_platform_name():
    return_result = os.popen("adb shell getprop ro.product.device").readlines()
    return return_result[0].strip("\r\n")