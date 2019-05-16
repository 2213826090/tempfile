#!/usr/bin/env python

##############################################################################
#
# @filename:    test.202_bluetooth.py
#
# @description: Testing bluetooth device
#
# @author:      alexandrux.n.branciog@intel.com
#
##############################################################################

import sys
from testlib.scripts.hal_autodetect import hal_steps as steps
from testlib.base.base_utils import get_args

# usage:
#       python test.<test_name>.py --serial 192.168.1.1:5555
args = get_args(sys.argv)
globals().update(vars(args))

module = {
"ecs_e7":{
    'modalias':'"acpi:OBDA8723:"',
    'devpath':'"/devices/platform/80860F0A:00/OBDA8723:00"',
    'refcount':'2',
    'type':'regular',
    'hal_id':'"bluetooth"',
    'library':'"bluetooth.default.so"',
    'modalias_mask':'"acpi*:OBDA8723:*"',
    'script':'"rc.hci-rtl8723_attach"',
    'script_one_shot':'1',
    'priority':'3',
    'hrec_filename':'"bluetooth_rtl8723.hrec"',
    'kmod':'"rfkill_gpio"',
},
"t100":{
    'modalias': '"acpi:BCM2E39:"',
    'devpath': '"/devices/platform/80860F0A:00/BCM2E39:00"',
    'refcount': '2',
    'type': 'regular',
    'hal_id': '"bluetooth"',
    'library': '"bluetooth.default.so"',
    'modalias_mask': '"acpi*:BCM2E39:*"',
    'script': '"rc.hci-bcm43xx_attach"',
    'script_one_shot': '1',
    'priority': '3',
},
"xps12":{
}
}

kmodule = {
"ecs_e7":{
    'name':'rfkill_gpio',
    'grep_for':'rfkill_gpio',
    'lsmod_output':['rfkill_gpio 12782 0 - Live 0x0000000000000000']
},
"t100":None,
"xps12":None
}

steps.platform_device(
    serial = serial,
    media_path = media_path,
    module = module,
    kmodule = kmodule
)()

