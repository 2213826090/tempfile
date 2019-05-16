#!/usr/bin/env python

##############################################################################
#
# @filename:    test.211_storage.py
#
# @description: Testing storage
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
    'modalias':'"platform:usb_phy_gen_xceiv"',
    'devpath':'"/devices/pci0000:00/0000:00:16.0/usb_phy_gen_xceiv.0"',
    'refcount':'2',
    'type':'unsupported',
},
"t100":{
    'modalias':'"usb:v0B05p17E0d0246dc00dsc00dp00ic03isc01ip01in00"',
    'devpath':'"/devices/pci0000:00/0000:00:14.0/usb1/1-3/1-3:1.0"',
    'refcount':'1',
    'type':'unsupported',
},
"xps12":{
}
}

steps.platform_device(
    serial = serial,
    module = module,
    media_path = media_path,
)()
