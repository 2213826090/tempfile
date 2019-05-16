# Copyright (C) 2015  Zhang,RongX Z <rongx.z.zhang@intel.com>
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

'''
@summary: Class for MIPIImpl operation
@since: 06/26/2015
@author: Zhang,RongX Z
'''

from testlib.util.common import g_common_obj


class MIPIImpl:

    '''
    classdocs
    '''

    def __init__(self):
        self.device = g_common_obj.get_device()
        g_common_obj.root_on_device()
        self.device.screen.on()

    def test_MIPI(self):
        output1 = g_common_obj.adb_cmd_capture_msg("cat /d/dri/0/i915_display_info")
        output2 = g_common_obj.adb_cmd_capture_msg("dmesg | grep -iE 'mipi|dsi'")
        assert output1.find("DSI") != -1 or output2 != "", "[FAILURE]failed to get DSI "
