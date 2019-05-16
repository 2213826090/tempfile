#!/usr/bin/env python

##############################################################################
#
# @filename:    test.204_graphics.py
#
# @description: Testing graphics device
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
    'modalias':'"pci:v00008086d00000F31sv00008086sd00007270bc03sc00i00"',
    'devpath':'"/devices/pci0000:00/0000:00:02.0"',
    'refcount':'1',
    'type':'regular',
    'hal_id':'"gralloc"',
    'library':'"gralloc.autodetect.so"',
    'mount src':'"ufo_byt"',
    'modalias_mask':'"pci:v00008086d00000F31sv*sd*bc03sc*i*"',
    'priority':'1',
},
"t100":{
    'modalias':'"pci:v00008086d00000F31sv00001043sd000014EDbc03sc00i00"',
    'devpath':'"/devices/pci0000:00/0000:00:02.0"',
    'refcount':'1',
    'type':'regular',
    'hal_id':'"gralloc"',
    'library':'"gralloc.autodetect.so"',
    'mount src':'"ufo_byt"',
    'modalias_mask':'"pci:v00008086d00000F31sv*sd*bc03sc*i*"',
    'priority':'1',
},
"xps12":{
}
}

mount_point = {
"ecs_e7":{
    'name':'gfx',
    'entries':[ '/system/rt/gfx']
},
"t100":{
    'name':'gfx',
    'entries':[ '/system/rt/gfx']
},
"xps12":{
}
}

steps.platform_device(
    serial = serial,
    module = module,
    media_path = media_path,
    mount_point = mount_point
)()

