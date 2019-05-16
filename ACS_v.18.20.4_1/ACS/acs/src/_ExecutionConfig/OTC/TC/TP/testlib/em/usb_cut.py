# -*- coding: utf-8 -*-
import os
import time

#from constants_def import *
from relay08 import get_relay_obj
from tools import ADBTools


class USBCut(ADBTools):

    def __init__(self, serial = None):
        ADBTools.__init__(self, serial)
        self.relay08 = get_relay_obj()

    def connect_usb_retry(self, retry = 3):
        self.relay08.usb_connect()
        time.sleep(5)
        for i in range(retry):
            if self.check_adb():
                return
            print "[Info]--- adb connect retry"
            self.relay08.usb_disconnect()
            time.sleep(2)
            self.relay08.usb_connect()
            time.sleep(5)
        if not self.check_adb():
            assert False, "adb reconnect failed"

    def cut(self, cut_time):
        self.relay08.usb_disconnect()
        print "[Info]--- Sleep %ss" % cut_time
        time.sleep(cut_time)
        self.connect_usb_retry()

    def usb_disconnect(self):
        self.relay08.usb_disconnect()

    def usb_connect(self):
        self.relay08.usb_connect()

