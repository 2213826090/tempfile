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
@summary: SpecialActionsImpl class
@since: 05/07/2015
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""

from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
import re


class SpecialActionsImpl(object):

    """SpecialAction"""

    config_file = 'tests.common.special_actions.conf'

    def __init__(self):
        self.device = g_common_obj.get_device()

        self.configer = TestConfig()
        self.config = self.configer.read(self.config_file, "SpecialActionsImpl")
        config_handle = ConfigHandle()
        self.config["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        self.arti = Artifactory(self.config.get('artifactory_location'))
        self.dut_res_dir = self.config.get("dut_res_dir")
        self.actions = []

    def setup(self):
        """resource setup"""
        cmd = "cd %s || mkdir -p %s" % (self.dut_res_dir, self.dut_res_dir)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))

        res_files = self.configer.read(self.config_file, "EventFiles")
        for key_name, value in res_files.iteritems():
            print "[Debug] install %s" % (key_name)
            res_file = self.arti.get(value)
            ret = g_common_obj.push_file(res_file, self.dut_res_dir)
            assert ret, 'Failed push %s' % (res_file)
            self.actions.append(key_name)
        print "[Debug] installed actions\n%s" % (self.actions)

    def zoom_in(self):
        """send event for zoom in"""
        do = 'event_zoom_in'
        assert do in self.actions

        cmd = 'sh %s/%s.sh' % (self.dut_res_dir, do)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))

    def zoom_out(self):
        """send event for zoom out"""
        do = 'event_zoom_out'
        assert do in self.actions

        cmd = 'sh %s/%s.sh' % (self.dut_res_dir, do)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))

    def long_touch_on_1_1(self):
        """send event for long click on left top corner."""

        def _get_eventId():
            '''get the input equipment id'''
            temps=[]
            equipments = g_common_obj.adb_cmd_capture_msg("getevent -p")
            tmp = re.findall(r'(/dev/input/event\d+)|(input props:.*\n.*)', equipments)
            for i in range(len(tmp)):
                if i%2 == 0:
                    continue
                else:
                    n_equip = tmp[i-1][0] + '\n' + tmp[i][1].replace('input props:\n','').strip()
                    temps.append(n_equip)
            for _ in temps:
                t = _.splitlines()
                if t[1] != "<none>":
                    return re.findall(r'\d+', t[0])[0]

        do = 'long_touch_on_1_1'
        eventId = _get_eventId()
        assert do in self.actions
        assert eventId != ''

        cmd = 'sh %s/%s.sh %s' % (self.dut_res_dir, do, eventId)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))

    def clean(self):
        """clean resource"""
        cmd = "rm -rf %s; sync;" % (self.dut_res_dir)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))
        self.actions = []

special_actions = SpecialActionsImpl()
