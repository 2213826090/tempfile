#!/usr/bin/env python

##############################################################################
#
# @filename:    test.201_audio.py
#
# @description: Testing audio device
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
    'refcount':'1',
    'type':'fallback',
    'hal_id':'"audio"',
    'mount src':'"unknown"',
    'priority':'0',
},
"t100":{
    'modalias':'"platform:byt-rt5640"',
    'devpath':'"/devices/platform/80860F28:00/byt-rt5640"',
    'refcount':'1',
    'type':'regular',
    'hal_id':'"audio"',
    'mount src':'"ALC5640"',
    'modalias_mask':'"platform:byt-rt5640"',
    'priority':'3',
},
"xps12":{
}
}

mount_point = {
"ecs_e7":None,
"t100":{
    'name':'audio',
    'entries':[ '/system/rt/audio']
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
