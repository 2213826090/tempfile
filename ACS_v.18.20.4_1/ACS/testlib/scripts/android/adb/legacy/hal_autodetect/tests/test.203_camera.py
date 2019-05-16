#!/usr/bin/env python

##############################################################################
#
# @filename:    test.203_camera.py
#
# @description: Testing camera device
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

modules = [{
"ecs_e7":{
    'modalias':'"acpi:INT33F0:INT33F0:"',
    'devpath':'"/devices/platform/80860F41:01/i2c-1/i2c-INT33F0:00"',
    'refcount':'2',
    'type':'regular',
    'hal_id':'"camera"',
    'library':'"camera.irda.so"',
    'modalias_mask':'"acpi*:INT33F0:INT33F0:*"',
    'priority':'3',
    'hrec_filename':'"media_int33f0.hrec"',
    'kmod':'"mt9m114"',
},
"t100":{
    'refcount':'1',
    'type':'fallback',
    'hal_id':'"camera"',
    'library':'"camera.uvc.so"',
    'priority':'0',
},
"xps12":{
}
},{
"ecs_e7":{
    'modalias':'"acpi:INT33BE:INT33BE:"',
    'devpath':'"/devices/platform/80860F41:01/i2c-1/i2c-INT33BE:00"',
    'refcount':'2',
    'type':'regular',
    'hal_id':'"camera"',
    'library':'"camera.irda.so"',
    'modalias_mask':'"acpi*:INT33BE:INT33BE:*"',
    'priority':'3',
    'hrec_filename':'"media_int33be.hrec"',
    'kmod':'"ov5693"',
},
"t100":None,
"xps12":None
}]

kmodules = [{
"ecs_e7":{
    'name':'mt9m114',
    'grep_for':'mt9m114',
    'lsmod_output':['mt9m114 27863 1 - Live 0x0000000000000000']
},
"t100":None,
"xps12":None
},{
"ecs_e7":{
    'name':'ov5693',
    'grep_for':'ov5693',
    'lsmod_output':['ov5693 27991 1 - Live 0x0000000000000000']
},
"t100":None,
"xps12":None
}]

for module, kmodule in zip(modules, kmodules):
    steps.platform_device(
        serial = serial,
        media_path = media_path,
        module = module,
        kmodule = kmodule
    )()
