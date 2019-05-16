#!/usr/bin/env python

##############################################################################
#
# @filename:    test.601_kmod_stress.py
#
# @description: Removing and inserting kmods in a loop
#
# @author:      alexandrux.n.branciog@intel.com
#
##############################################################################

import sys
import time
from testlib.scripts.hal_autodetect import hal_steps as steps
from testlib.base.base_utils import get_args

# usage:
#       python test.<test_name>.py --serial 192.168.1.1:5555
args = get_args(sys.argv)
globals().update(vars(args))

iterations = 200
kmodules = [{
"ecs_e7":{
    'name':'bmc150_accel',
    'grep_for':'bmc150_accel',
    'lsmod_output':['bmc150_accel 18976 0 - Live 0x0000000000000000']
},
"t100":{
    'name':'brcmfmac',
    'grep_for':'brcmfmac',
    'lsmod_output':['brcmfmac 226122 0 - Live 0x00000000 (O)',
                    'brcmutil 14863 1 brcmfmac, Live 0x00000000 (O)',
                    'cfg80211 443311 1 brcmfmac, Live 0x00000000 (O)',
                    'compat 13340 2 brcmfmac,cfg80211, Live 0x00000000 (O)']
},
"xps12":{
}
},{
"ecs_e7":{
    'name':'i2c_hid',
    'grep_for':'i2c_hid',
    'lsmod_output':['i2c_hid 18717 0 - Live 0x0000000000000000']
},
"t100":{
    'name':'asus_nb_wmi',
    'grep_for':'wmi',
    'lsmod_output':['asus_nb_wmi 16862 0 - Live 0x00000000',
                    'asus_wmi 23517 1 asus_nb_wmi, Live 0x00000000',
                    'wmi 18356 1 asus_wmi, Live 0x00000000']
},
"xps12":{
}
}]

for kmodule in kmodules:
    for i in range(iterations):
        print "Iteration {0}".format(i+1)
        steps.kmod_remove_insert(
            serial = serial,
            kmodule = kmodule,
            media_path = media_path,
        )()
        time.sleep(3)
