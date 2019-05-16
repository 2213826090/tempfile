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
@summary: SystemUiExtendImpl class
@since: 02/10/2015
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""

import re
import time
import random
from testlib.util.log import Logger
from testlib.util.common import g_common_obj
from testlib.systemui.systemui_impl import SystemUI
from testlib.graphics.common import launch_settings_am
from testlib.graphics.html5_impl import html5

LOG = Logger.getlogger(__name__)

class SystemUiExtendImpl(SystemUI):

    PKG_NAME = "com.android.systemui"
    MAIN_ACTIVITY = "com.android.systemui.recents.RecentsActivity"

    class RecentUI(object):

        """Recent UI Elements"""

        def __init__(self, device):
            self.device = device

        @property
        def headers(self):
            if self.device(resourceId="com.android.systemui:id/activity_description").exists:
                return self.device(
                    resourceId="com.android.systemui:id/activity_description")
            else:
                return self.device(
                    resourceId="com.android.systemui:id/title")

    def __init__(self):
        super(SystemUiExtendImpl, self).__init__()
        self._recent = SystemUiExtendImpl.RecentUI(self.d)

    def launch_desk_app(self, name):
        self.d.press.back() * 3
        time.sleep(2)
        self.d(descriptionContains="Apps").click.wait()
        if self.d().scroll.horiz.to(text=name):
            time.sleep(1)
            self.d(text=name).click()
            self.d.wait.update()
            print "[Debug] open app %s" % (name)

    def switch_recent_app(self, name):
        device = self.d
        device.press.home()
        time.sleep(1)
        device.press.recent()
        time.sleep(1)
        y = device.info["displayHeight"]
        x = device.info["displayWidth"]
        pre_dump = ''
        for _ in range(100):
            for idx in range(self._recent.headers.count):
                text = self._recent.headers[idx].info['text']
                self.d().scroll.to(textContains=text)
                if text == name:
                    self._recent.headers[idx].click()
                    self.d.wait.update()
                    print "[Debug] switch to app %s" % (name)
                    return
            device.swipe(x / 2, y / 2, x / 2, y / 2 + y / 4, steps=50)
            time.sleep(2)
            if pre_dump == device.dump():
                break
            pre_dump = device.dump()
        assert False, "There is no %s APP existed in recent list" % (name)

    def close_recent_app(self, name):
        device = self.d

        def go_recent_page():
            _, act = self.get_current_focus()
            if act != self.MAIN_ACTIVITY:
                device.press.recent()
                self.d.wait.update()
            time.sleep(3)

        go_recent_page()
        assert device().scroll.vert.to(text=name), \
            "There is no %s recent APP" % (name)
        print "[Debug] close app %s" % (name)
        device(text=name)\
            .right(resourceId='com.android.systemui:id/dismiss_task').click.topleft()
        self.d.wait.update()
        go_recent_page()
        assert not device().scroll.vert.to(text=name), \
            "[FAILURE] Closed app %s always existed in recents" % (name)

    def close_all_recent_apps(self):
        device = self.d
        x = device.info["displayWidth"]
        device.press.home()
        device.press.recent()
        device().fling.toBeginning()
        while self._recent.headers.exists:
            text = self._recent.headers.info['text']
            y = self._recent.headers.info['bounds']['top']
            if self._recent.headers.info.get("visibleBounds").get("bottom") > device.info["displayHeight"]:
                device.swipe(x / 2, y / 4, x / 2, y / 2)
                time.sleep(2)
#             self._recent.headers.drag.to(x, y / 2, steps=5)
            self._recent.headers.swipe.right()
            print "[Debug] close APP %s" % (text)
            device.wait.update()
            time.sleep(1)
        device.press.home()
        device.press.recent()
        device.press.home()

    @staticmethod
    def get_current_focus():
        cmd = "dumpsys window|grep mCurrentFocus"
        pattern = r"(?P<PKG_NAME>[\w.]+)/(?P<ACT_NAME>[\w.]+)}"
        packagename, activityname = '', ''
        for _ in range(3):
            msg = g_common_obj.adb_cmd_capture_msg(cmd)
            match = re.search(pattern, msg)
            if not match:
                time.sleep(1)
                continue
            packagename = match.group('PKG_NAME')
            activityname = match.group('ACT_NAME')
            break
        return packagename, activityname

    def check_photos_app(self):
        if self.d(text="Skip").exists:
            self.d(text="Skip").click()
            time.sleep(2)
        _, act = self.get_current_focus()
        time.sleep(1)
        assert act == "com.google.android.apps.photos.home.HomeActivity", "Fail to launch photos"


class SplitWindow(SystemUiExtendImpl):

    def __init__(self):
        super(SplitWindow, self).__init__()
        # Launch settings & chrome app before testing.
        SystemUiExtendImpl().close_all_recent_apps()
        launch_settings_am()
        self.html = html5
        self.html.launch_chrome_am()
        g_common_obj.back_home() # Back to home screen
        self.width = self.d.info['displayWidth']
        self.height = self.d.info['displayHeight']
        self.recent_btn = self.d(resourceId="com.android.systemui:id/recent_apps")
        self.split_button = self.d(resourceId='com.android.systemui:id/docked_divider_handle')
        self._wait_time = 8 # wait for ui update

    def _is_split(self):
        org_left, org_right = 0, self.width
        now_left, now_right = self.d().bounds['left'], self.d().bounds['right']
        if org_left == now_left and org_right == now_right:
            return False
        else:
            return True

    def _exit_split(self):
        return self.recent_btn.long_click()

    def _get_start_position(self):
        split_button_location = self.split_button.bounds
        s_x = (split_button_location['right'] - split_button_location['left']) / 2 + split_button_location['left']
        s_y = (split_button_location['bottom'] - split_button_location['top']) / 2 + split_button_location['top']
        print split_button_location
        return s_x, s_y

    def split_app_window(self, app_name, orientation='left'):
        '''
        Split app window func
        :param app_name: Input app text name.
        :param orientation: Choose 'left' or 'right'/
        :return: 
        '''
        if not self.d(text="CLEAR ALL").exists:
            try:
                self.recent_btn.click() # Show recent window
            except:
                self.d.press.home()
                time.sleep(2)
                self.d.press.recent() # Handle missing recent button while playing game.
            time.sleep(2)
        self.d().scroll.vert.to(text=app_name)
        app_height = self.d(text=app_name).bounds['top']
        if orientation == 'right':
            x = self.d.info['displayWidth']
        else:
            x = 0
        self.d(text=app_name).drag.to(x, app_height, steps=100)
        time.sleep(self._wait_time)
        assert self._is_split(), "%s app is not splited."

    def change_split_size(self, orientation='left'):
        _step = self.width / 4
        s_x, s_y = self._get_start_position()
        dict_orientation = {'left': -1, 'right': 1}
        _move_to = s_x + _step * dict_orientation[orientation] if orientation in ('left', 'right') \
                                                                else random.choice([10, self.width])
        sx, sy, ex, ey = s_x, s_y, _move_to, s_y
        LOG.info("Split size from %s to %s" % ((sx, sy), (ex, ey)))
        self.d.swipe(sx, sy, ex, ey, steps=200)
        time.sleep(self._wait_time)
        try:
            if self.split_button.exists:
                n_sx, n_sy = self._get_start_position()
                LOG.info("Now split position is in (%s, %s)" % (n_sx, n_sy))
                assert s_x != n_sx, "Fail to change split size."
            else:
                LOG.info("Now it's in full screen.")
        except:
            raise Exception("Split operation failed.")
