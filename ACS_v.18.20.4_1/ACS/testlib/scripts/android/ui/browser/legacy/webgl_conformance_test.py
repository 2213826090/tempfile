#!/usr/bin/env python
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui.browser import browser_steps

import sys

serial, wlan_interface = get_parms(sys.argv, 'wlan_interface')

if wlan_interface is None:
    wlan_interface = 'wlan0'

adb_steps.connect_device(serial = serial)()
ui_steps.dump(outfile = "inter.xml")()
