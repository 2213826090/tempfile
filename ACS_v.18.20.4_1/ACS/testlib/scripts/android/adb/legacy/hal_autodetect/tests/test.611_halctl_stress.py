#!/usr/bin/env python

##############################################################################
#
# @filename:    test.611_halctl_stress.py
#
# @description: Suppressing and adding sensors modalias in a loop
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
module = {
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
}

for i in range(iterations):
    print "Iteration {0}".format(i+1)
    steps.halctl_add_suppress(
        serial = serial,
        module = module,
        media_path = media_path,
    )()
    time.sleep(3)
