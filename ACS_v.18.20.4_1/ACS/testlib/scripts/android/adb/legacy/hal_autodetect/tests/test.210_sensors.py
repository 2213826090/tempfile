#!/usr/bin/env python

##############################################################################
#
# @filename:    test.210_sensors.py
#
# @description: Testing sensors
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
    'modalias':'"acpi:BSBM0150:INTMAGN:"',
    'devpath':'"/devices/platform/80860F41:02/i2c-2/i2c-BSBM0150:00"',
    'refcount':'2',
    'type':'regular',
    'hal_id':'"sensors"',
    'library':'"iio-sensors-hal.so"',
    'props':'"sensors_bmc150_magn.prop"',
    'modalias_mask':'"acpi*:BSBM0150:*"',
    'priority':'3',
    'hrec_filename':'"sensors_bmc150_magn.hrec"',
    'kmod':'"bmm050"',
},
"t100":{
    'modalias':'"acpi:INVN6500:"',
    'devpath':'"/devices/platform/80860F41:04/i2c-3/i2c-INVN6500:00:0c"',
    'refcount':'3',
    'type':'regular',
    'hal_id':'"sensors"',
    'library':'"iio-sensors-hal.so"',
    'props':'"sensors_inv6500.prop"',
    'modalias_mask':'"acpi*:INVN6500:*"',
    'priority':'3',
},
"xps12":{
}
},{
"ecs_e7":{
    'modalias':'"acpi:BSBA0150:INTACCL:"',
    'devpath':'"/devices/platform/80860F41:02/i2c-2/i2c-BSBA0150:00"',
    'refcount':'2',
    'type':'regular',
    'hal_id':'"sensors"',
    'library':'"iio-sensors-hal.so"',
    'props':'"sensors_bmc150_accel.prop"',
    'modalias_mask':'"acpi*:BSBA0150:*"',
    'priority':'3',
    'hrec_filename':'"sensors_bmc150_accel.hrec"',
    'kmod':'"bmc150_accel"',
},
"t100":None,
"xps12":None,
},{
"ecs_e7":{
    'modalias':'"acpi:CPLM3218:INTLALS:"',
    'devpath':'"/devices/platform/80860F41:02/i2c-2/i2c-CPLM3218:00"',
    'refcount':'2',
    'type':'regular',
    'hal_id':'"sensors"',
    'library':'"iio-sensors-hal.so"',
    'props':'"sensors_cplm3218x_als.prop"',
    'modalias_mask':'"acpi*:CPLM3218:*"',
    'priority':'3',
    'hrec_filename':'"sensors_cplm3218x_als.hrec"',
    'kmod':'"cm32181"',
},
"t100":None,
"xps12":None,
},{
"ecs_e7":{
    'modalias':'"acpi:BMG0160:INTGYRO:"',
    'devpath':'"/devices/platform/80860F41:02/i2c-2/i2c-BMG0160:00"',
    'refcount':'2',
    'type':'regular',
    'hal_id':'"sensors"',
    'library':'"iio-sensors-hal.so"',
    'props':'"sensors_bmg0160_gyro.prop"',
    'modalias_mask':'"acpi*:BMG0160:*"',
    'priority':'3',
    'hrec_filename':'"sensors_bmg0160_gyro.hrec"',
    'kmod':'"bmg160"',
},
"t100":None,
"xps12":None,
}]

kmodules = [{
"ecs_e7":{
    'name':'bmm050',
    'grep_for':'bmm050',
    'lsmod_output':['bmm050 29306 0 - Live 0x0000000000000000']
},
"t100":None,
"xps12":None
},{
"ecs_e7":{
    'name':'bmc150_accel',
    'grep_for':'bmc150_accel',
    'lsmod_output':['bmc150_accel 18976 0 - Live 0x0000000000000000']
},
"t100":None,
"xps12":None
},{
"ecs_e7":{
    'name':'cm32181',
    'grep_for':'cm32181',
    'lsmod_output':['cm32181 13251 0 - Live 0x0000000000000000']
},
"t100":None,
"xps12":None
},{
"ecs_e7":{
    'name':'bmg160',
    'grep_for':'bmg160',
    'lsmod_output':['bmg160 18695 0 - Live 0x0000000000000000']
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
