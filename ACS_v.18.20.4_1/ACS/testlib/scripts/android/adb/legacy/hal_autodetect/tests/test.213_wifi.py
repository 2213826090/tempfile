#!/usr/bin/env python

##############################################################################
#
# @filename:    test.213_wifi.py
#
# @description: Testing wifi device
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
    'hal_id':'"wifi"',
    'mount src':'"aosp"',
    'priority':'0',
},
"t100":{
    'modalias':'"sdio:c00v02D0d4324"',
    'devpath':'"/devices/platform/INT33BB:00/mmc_host/mmc1/mmc1:0001/mmc1:0001:1"',
    'refcount':'2',
    'type':'regular',
    'hal_id':'"wifi"',
    'record_kmod':'"brcmfmac"',
    'priority':'2',
    'kmod':'"brcmfmac"',
},
"xps12":{
}
}

kmodule = {
"ecs_e7":None,
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
}

mount_point = {
"ecs_e7":None,
"t100":{
    'name':'wifi',
    'entries':['/system/rt/wifi']
},
"xps12":{
}
}

steps.platform_device(
    serial = serial,
    module = module,
    kmodule = kmodule,
    mount_point = mount_point,
    media_path = media_path,
)()

