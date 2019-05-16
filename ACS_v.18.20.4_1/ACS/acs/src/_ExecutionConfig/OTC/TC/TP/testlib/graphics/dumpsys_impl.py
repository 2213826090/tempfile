# Intel Corporation All Rights Reserved.
# The source code contained or described herein and
# all documents related to the source code ("Material") are owned by
# Intel Corporation or its suppliers or licensors.
# Title to the Material remains with Intel Corporation or
# its suppliers and licensors.
# The Material contains trade secrets and proprietary and
# confidential information of Intel or its suppliers and licensors.
# The Material is protected by worldwide copyright and
# trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified,
# published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.
# Any license under such intellectual property rights must be express
# and approved by Intel in writing.
"""
@summary: ChromeExtendImpl class
@since: 02/10/2015
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""

import re

from testlib.util.common import g_common_obj


class DumpSysImpl(object):

    """ DumpSysImpl """

    expr_policy = r'policy=(\w+)'
    expr_brightness = r'mScreenBrightness=(\w+)'

    def __init__(self):
        self.device = g_common_obj.get_device()
        self.precheck()

    def precheck(self):
        """check keywords existed in dumpsys"""
        expr_variables = [i for i in dir(self) if not callable(i) and i.startswith('expr_')]
        dump_msg = self.adb_dumpsys('display')
        for each in expr_variables:
            value = self.__getattribute__(each)
            assert re.findall(value, dump_msg),\
                "[FAILURE] There is no matched info: %s" % (value)

    def dump_display_power_state(self):
        """dump display power state"""
        out_dict = {}
        msg = self.adb_dumpsys('display')
        out_dict['policy'] = re.findall(self.expr_policy, msg)[0]
        out_dict['brightness'] = re.findall(self.expr_brightness, msg)[0]
        print "[Debug] %s" % (out_dict)
        return out_dict

    def adb_dumpsys(self, service_name):
        """dumpsys by service name"""
        cmd = 'dumpsys %s' % (service_name)
        msg = g_common_obj.get_test_device()\
            .adb_cmd_capture_msg_ext(repr(cmd), time_out=90)
        if 'Error' in msg or "Can't find service:" in msg:
            assert False, "[FAILURE] %s" % (msg)
        return msg
