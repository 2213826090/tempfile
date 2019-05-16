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
@summary: NotificationAndQuickSettingsImpl class
@since: 06/17/2015
@author: Zhang,RongX Z(rongx.z.zhang@intel.com)
"""
import time
from testlib.util.common import g_common_obj

class NotificationAndQuickSettingsImpl(object):

    """ NotificationAndQuickSettingsImpl """

    def __init__(self):
        super(NotificationAndQuickSettingsImpl, self).__init__()
        self.device = g_common_obj.get_device()
        self.device.screen.on()

    def drag_notification_panel(self):
        for _ in range(0,3):
            self.device.press.home()
            self.device.open.notification()
            time.sleep(3)
            if self.device(resourceId="android:id/notification_main_column").exists:
                break
        assert self.device(resourceId="android:id/notification_main_column").exists,"[FAILURE] Notification panel is failed to display on top of the Home Screen."
        for _ in range(0,2):
            self.device.press.back()

    def drag_quick_settings_panel(self):
        for _ in range(0,3):
            self.device.press.home()
            self.device.open.quick_settings()
            time.sleep(3)
            if self.device(resourceId="com.android.systemui:id/quick_settings_panel").exists:
                break
        assert self.device(resourceId="com.android.systemui:id/quick_settings_panel").exists,"[FAILURE] Quick Settings panel is failed to display on top of the Home Screen."
        self.device.press.back()