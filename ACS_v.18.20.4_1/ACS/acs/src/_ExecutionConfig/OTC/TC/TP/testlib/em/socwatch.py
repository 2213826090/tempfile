#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
from tools import ADBTools, download_artifactory_content, shell_cmd

class SoCWatch(ADBTools):

    def __init__(self):
        ADBTools.__init__(self)
        socwatch_zip = download_artifactory_content("socwatch")
        file_dir = os.path.dirname(socwatch_zip)
        cmd = "unzip -oq %s -d %s" % (socwatch_zip, file_dir)
        shell_cmd(cmd)
        self.script_dir = os.path.join(file_dir, "socwatch_android_NDA_v2.4.1")
        self.device_socwatch_dir = "/data/socwatch"

    def socwatch_setup(self):
        cmd = "'cd /vendor/lib/modules; insmod socperf2_0.ko; insmod socwatch2_4.ko'"
        self.testDevice.adb_cmd(cmd)
        cmd = "cd %s; bash %s" % (self.script_dir, "socwatch_android_install.sh")
        shell_cmd(cmd)
        #cmd = "source %s" % os.path.join(self.device_socwatch_dir, "setup_socwatch_env.sh")
        #self.testDevice.adb_cmd(cmd)

    def socwatch_cmd(self, args):
        cmd = "'cd %s; source setup_socwatch_env.sh; ./socwatch %s'" % (self.device_socwatch_dir, args)
        msg = self.testDevice.adb_cmd_capture_msg(cmd, time_out = 100)
        print msg
        assert msg

    def clean_result(self):
        cmd = "rm -rf %s" % os.path.join(self.device_socwatch_dir, "results")
        self.testDevice.adb_cmd(cmd)

