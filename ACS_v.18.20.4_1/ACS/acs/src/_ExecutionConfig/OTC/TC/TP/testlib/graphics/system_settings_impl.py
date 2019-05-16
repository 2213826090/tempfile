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
#published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.
# Any license under such intellectual property rights must be express
# and approved by Intel in writing.
"""
@summary: ChromeExtendImpl class
@since: 04/10/2015
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""

from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle


class SystemSettingsImpl(object):

    """ SystemSettingsImpl """

    CONFIG_FILE = 'tests.common.system_settings.conf'

    PKG_NAME = "com.android.settings"
    MAIN_ACTIVITY = "com.android.settings.Settings"
    SUB_ACTIVITY = "com.android.settings.SubSettings"

    class FieldStruct(object):

        def __init__(self, name, minv=None, maxv=None, default=None):
            self.name = name
            self.min = minv
            self.max = maxv
            self.default = default
    TABLE_SYSTEM = 'system'

    def __init__(self):
        self.device = g_common_obj.get_device()

        self.configer = TestConfig()
        self.config = self.configer.read(self.CONFIG_FILE, "SystemSettingsImpl")
        self.db_storage = Artifactory(self.config.get('settings_storage'))

    def launch(self):
        g_common_obj.launch_app_am(self.PKG_NAME, self.MAIN_ACTIVITY)

    def resume(self):
        cmdstr = "am start %s" % (self.PKG_NAME)
        g_common_obj.adb_cmd(cmdstr)

    def get_system_setting(self, field):
        return self._get_setting(self.TABLE_SYSTEM, field)

    def put_system_setting(self, field, value):
        return self._put_setting(self.TABLE_SYSTEM, field, value)

    def _get_setting(self, table, field):
        print "[Debug] get_setting table:%s field:%s" % (table, field)

        cmd = 'settings get %s %s' % (table, field)
        value = g_common_obj.adb_cmd_capture_msg(repr(cmd))
        assert value != 'null',\
            "[FAILURE] Not found field, table:%s field:%s value:%s"\
            % (table, field, value)
        print "[Debug] return value:%s" % (value)
        return value

    def _put_setting(self, table, field, value):
        print "[Debug] put_setting table:%s field:%s value:%s"\
            % (table, field, value)
        o_value = self._get_setting(table, field)
        if o_value == value:
            print "[Debug] skip put setting, old_value:%s new_value:%s"\
                % (o_value, value)
            return o_value

        cmd = 'settings put %s %s %s' % (table, field, value)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))
        n_value = self._get_setting(table, field)
        return n_value
