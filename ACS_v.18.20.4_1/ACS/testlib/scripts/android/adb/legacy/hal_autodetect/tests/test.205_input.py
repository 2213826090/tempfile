#!/usr/bin/env python

##############################################################################
#
# @filename:    test.205_input.py
#
# @description: Testing input
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
    'modalias':'"input:b0018v0000p0000e0000-e0,3,kra0,1,2,28,mlsfw"',
    'devpath':'"/devices/virtual/input/input7"',
    'refcount':'1',
    'type':'unsupported',
},
"t100":{
    'modalias':'"input:b0018v03EBp8C0Ee0100-e0,1,3,k14A,ra0,1,2F,35,36,39,mlsfw"',
    'devpath':'"/devices/platform/80860F41:05/i2c-4/i2c-ATML1000:00:4a/0018:03EB:8C0E.0001/input/input4"',
    'refcount':'1',
    'type':'unsupported'
},
"xps12":{
}
}

# make sure the binding list is same as after boot
steps.check_hald_recovery(
    serial = serial,
    media_path = media_path,
)()

steps.platform_device(
    serial = serial,
    module = module,
    media_path = media_path,
)()
