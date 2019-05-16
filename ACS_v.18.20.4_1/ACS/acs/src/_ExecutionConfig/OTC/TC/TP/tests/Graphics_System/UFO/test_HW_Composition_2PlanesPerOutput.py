#Copyright (C) 2014  Zhang,RongX Z <rongx.z.zhang@intel.com>
#Intel Corporation All Rights Reserved.

#The source code contained or described herein and
#all documents related to the source code ("Material") are owned by
#Intel Corporation or its suppliers or licensors.

#Title to the Material remains with Intel Corporation or
#its suppliers and licensors.
#The Material contains trade secrets and proprietary and
#confidential information of Intel or its suppliers and licensors.
#The Material is protected by worldwide copyright and
#trade secret laws and treaty provisions.
#No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
#or disclosed in any way without Intel's prior express written permission.
#No license under any patent, copyright, trade secret or
#other intellectual property right is granted to
#or conferred upon you by disclosure or delivery of the Materials,
#either expressly, by implication, inducement, estoppel or otherwise.

#Any license under such intellectual property rights must be express
#and approved by Intel in writing.

'''
@summary: drag notification and quick settings
@since: 06/17/2015
@author: Zhang,RongX Z(rongx.z.zhang@intel.com)
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.notification_quick_settings_impl import NotificationAndQuickSettingsImpl
from testlib.graphics.common import Logcat


class NotificationAndQuickSettings(UIATestBase):

    def setUp(self):
        super(NotificationAndQuickSettings, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.systemui = SystemUiExtendImpl()
        self.notifi_quick_settings = NotificationAndQuickSettingsImpl()
        self.systemui.unlock_screen()
        self._log = Logcat()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(NotificationAndQuickSettings, self).tearDown()

    def test_HW_Composition_2PlanesPerOutput(self):
        """
        1.Enter into Home screen.
        2. Drag the areas corresponding to the Notification Panel.
        3. Drag the areas corresponding to the  Quick Settings Panel.
        4. Each Panel, Notification and Quick Settings, is properly displayed on top of the Home Screen.
        """
        try:
            self.notifi_quick_settings.drag_notification_panel()
            self.notifi_quick_settings.drag_quick_settings_panel()
        except:
            pass
        layer_num = self._log.get_layer_number()
        assert any(i in layer_num for i in [0, 1]) is False, "Layer number is less than 2"