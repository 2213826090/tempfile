#!/usr/bin/env python

##############################################################################
#
# @filename:    test.212_tpm.py
#
# @description: Testing tpm device
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
    'hal_id':'"keystore"',
},
"t100":{
    'hal_id':'"keystore"',
},
"xps12":{
}
}

steps.platform_device(
    serial = serial,
    media_path = media_path,
    module = module
)()
