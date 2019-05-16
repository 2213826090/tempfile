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
@summary: ChromeCast test template class
@since: 01/29/2015
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""
import sys
import time

from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.chromecast.chromecast_impl import ChromeCastNexusPlayerImpl


class ChromeCastTestBase(UIATestBase):

    @classmethod
    def setUpClass(cls):
        super(ChromeCastTestBase, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

        config_file = 'tests.chromecast.device.conf'
        test_conf = cls.config.read(config_file, "TEST_CONF")
        cls.cast_model = test_conf.get('cast_model')
        if cls.cast_model == 'Nexus Player':
            cls.IChromeCastImpl = ChromeCastNexusPlayerImpl
        elif cls.cast_model == 'Chromecast':
            pass

    def tearDown(self):
        msg = sys.exc_info()[1]
        if msg:
            cmd = 'svc wifi disable; sleep 20; svc wifi enable'
            g_common_obj.adb_cmd_capture_msg(repr(cmd))
        super(ChromeCastTestBase, self).tearDown()

    def turn_on_wireless(self):
        cmd = 'svc wifi disable; svc wifi enable'
        g_common_obj.adb_cmd_capture_msg(repr(cmd))

        for _ in range(10):
            state, ssid = self.get_wifi_status()
            print "[Debug] ssid:%s state:%s" % (ssid, state)
            if state == 'COMPLETED':
                return
            time.sleep(2)

        assert False,\
            "[FAILURE] Wait Wifi connection status time over"

    def get_wifi_status(self):
        cmd = "wpa_cli IFNAME=wlan0 status"
        msg = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        lines = msg.splitlines()
        state, ssid = '', ''
        for line in lines:
            if line.startswith('wpa_state'):
                state = line.split('=')[1].strip("\r\n")

            if line.startswith("ssid"):
                ssid = line.split('=')[1].strip("\r\n")

        return (state, ssid)
