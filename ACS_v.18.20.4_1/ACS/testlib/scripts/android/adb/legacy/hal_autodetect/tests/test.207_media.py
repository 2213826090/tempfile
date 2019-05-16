#!/usr/bin/env python

##############################################################################
#
# @filename:    test.207_media.py
#
# @description: Testing media device
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
    'hal_id':'"media"',
    'props':'"camera_baytrail.prop"',
    'mount src':'"baytrail"',
    'modalias_mask':'"pci:v00008086d00000F31sv*sd*bc*sc*i*"',
    'priority':'3',
},
"t100":{
    'modalias':'"pci:v00008086d00000F31sv00001043sd000014EDbc03sc00i00"',
    'devpath':'"/devices/pci0000:00/0000:00:02.0"',
    'refcount':'1',
    'type':'regular',
    'hal_id':'"media"',
    'props':'"camera_baytrail.prop"',
    'mount src':'"baytrail"',
    'modalias_mask':'"pci:v00008086d00000F31sv*sd*bc*sc*i*"',
    'priority':'3',
},
"xps12":{
}
}

mount_point = {
"ecs_e7":{
    'name':'media',
    'entries':[ '/system/rt/media',]
},
"t100":{
    'name':'media',
    'entries':[ '/system/rt/media',
                '/system/rt/media',
                '/system/vendor/media/generic']
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
