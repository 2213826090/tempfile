#!/usr/bin/env python

#######################################################################
#
# @filename:    WiFi_Scan.py
# @description: Tests if WiFi scans for known APs
# @author:      ion-horia.petrisor@intel.com
#
#######################################################################


import sys
import time
from testlib.scripts.wireless.wifi import wifi_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps

adb_steps.connect_device(serial = sys.argv[1] + ":5555")()

wifi_steps.set_from_wifi_settings()()

time.sleep(15)

ap_list = [
    "dd-wrt",
    "SP_MTBF",
    #"Guest",
    #"TSNOfficeWLAN1",
    #"RSN2OfficeWLAN",
    #"sp_gpt",
    #"DIRECT-CLML-2160",
    #"dlink-dsl",
    #"TP-LinkA2",
    #"SPWB-03",
]

wifi_steps.check_AP_list_is_scanned(ap_list = ap_list)()

ui_steps.press_home()()
