#!/usr/bin/env python

########################################################################
#
# @filename:    pnp_step.py
# @description: PnP basic test step
# @author:      emilianx.c.ioana@intel.com
#
########################################################################

from testlib.base.base_step import step as base_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.power_and_performance import pnp_utils

import os


class step(adb_step):
    
    def __init__(self, **kwargs):
        adb_step.__init__(self,**kwargs)
        self.serial = kwargs['serial']
        self.port= kwargs['port']
        self.test_case = kwargs["test_case"]
        self.benchmark = kwargs["benchmark"]
        
    def do(self):
        adb_steps.connect_device(serial=self.serial, port=self.port)()
        adb_steps.root_connect_device(serial=self.serial, port=self.port)()
        adb_steps.reboot(serial=self.serial, port=self.port)()
        pnp_utils.wait_for_cooldown(self.adb_connection, "t100")
        
