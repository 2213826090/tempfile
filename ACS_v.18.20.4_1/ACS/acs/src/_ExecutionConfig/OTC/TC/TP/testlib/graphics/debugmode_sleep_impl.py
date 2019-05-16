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
@summary: Class for DebugMode Sleep operation
@since: 06/4/2015
@author: Zhang,RongX Z
'''

import time
from testlib.util.common import g_common_obj
from testlib.graphics.debugmode_impl import DebugModeImpl

class Locator(object):

    """
    locator
    """

    def __init__(self, device):
        self.device = device

    @property
    def wait_exist(self):
        """ wait until the ui object exist """
        def _wait(uiobj, timeout=5):
            return uiobj.wait("exists", timeout * 500)
        return _wait

    @property
    def performance_tests(self):
        return self.device(text="Performance Tests")

    @property
    def developer_options(self):
        return self.device(text="Developer options")

    @property
    def simulate_secondary_displays(self):
        return self.device(textContains="Simulate secondary displays")

    @property
    def btn_720x480(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("720x480 mdpi", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_720x480_secure(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("720x480 mdpi (secure)", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_1280x720(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("1280x720 tvdpi", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_1280x720_secure(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("1280x720 tvdpi (secure)", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_1920x1080(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("1920x1080 xhdpi", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_1920x1080_secure(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("1920x1080 xhdpi (secure)", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_1280x720_and_1920x1080(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("1280x720 tvdpi and 1920x1080 xhdpi", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_None(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("None", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_480p(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("480p", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_480p_secure(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("480p (secure)", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_720p(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("720p", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_720p_secure(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("720p (secure)", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_1080p(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("1080p", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_1080p_secure(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("1080p (secure)", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_4K(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("4K", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_4K_secure(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("4K (secure)", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_4K_upscaled(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("4K (upscaled)", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_4K_upscaled_secure(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("4K (upscaled, secure)", \
                        className="android.widget.CheckedTextView")

    @property
    def btn_720p_1080p_dualscreen(self):
        return self.device(
            className="android.widget.ListView", \
             resourceId="android:id/select_dialog_listview")\
        .child_by_text("720p, 1080p (dual screen)", \
                        className="android.widget.CheckedTextView")


class DebugModeSleepImpl:

    '''
    classdocs
    '''

    def __init__(self):
        self.device = g_common_obj.get_device()
        self._locator = Locator(self.device)
        self.debugmode = DebugModeImpl()

    def launch_settings_am(self):
        """ Launch Settings via adb am command
        """
        print "Launch Settings by adb am"
        g_common_obj.launch_app_am(
            "com.android.settings", "com.android.settings.Settings")
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_settings_am():
        """ Stop Settings via adb am command
        """
        print "Stop Settings by adb am"
        g_common_obj.stop_app_am("com.android.settings")

    def change_to_vertical(self):
        """ change device to vertical
        """
        g_common_obj.set_vertical_screen()

    def power_off_device(self):
        g_common_obj.adb_cmd("input keyevent 26")

    def power_on_device(self):
        """ Unlock screen by via input keyevent 82
        """
        self.device.wakeup()
        print "Unlock screen via input keyevent 82"
        cmd = 'input keyevent 82; echo $?'
        result_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\r\n')
        suc = False
        if len(result_list):
            ret = result_list[-1].rstrip()
            suc = (int(ret) == 0)

    def check_resolution(self, keyword1, keyword2):
        get_cmd = "settings get global overlay_display_devices"
        msg = g_common_obj.adb_cmd_capture_msg(repr(get_cmd))
        remsg1 = msg.find(keyword1)
        remsg2 = msg.find(keyword2)
        return remsg1, remsg2, msg

    def choose_simulate_secondary_displays(self):
        y = self.device.info["displayHeight"]
        x = self.device.info["displayWidth"]
        if not self.device(text="Developer options").exists:
            self.device(scrollable=True).scroll.to(text="Developer options")
        time.sleep(1)
        self._locator.developer_options.click.wait()
        if not self._locator.simulate_secondary_displays.exists:
            self.device(scrollable=True).scroll.vert.to(textContains="Simulate secondary displays")
        self._locator.simulate_secondary_displays.drag.to(x / 2, y / 2)
        time.sleep(3)

    def change_simulate_None(self):
        self._locator.simulate_secondary_displays.click.wait()
        self._locator.btn_None.click.wait()

    def change_simulate_720x480(self):
        print "[debug] change resolution to 720x480"
        self._locator.simulate_secondary_displays.click.wait()
        self._locator.btn_720x480.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("720x480", "secure")
        assert  remsg1 != -1 and remsg2 == -1 , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()
        y = self.device.info["displayHeight"]
        self.device.drag(0, y / 10, 0, y / 5)

    def change_simulate_720x480_secure(self):
        print "[debug] change resolution to 720x480 secure"
        self._locator.simulate_secondary_displays.click.wait()
        self._locator.btn_720x480_secure.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("720x480", "secure")
        assert not (remsg1 == -1 or remsg2 == -1) , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()
        y = self.device.info["displayHeight"]
        self.device.drag(0, y / 10, 0, y / 5)

    def change_simulate_1280x720(self):
        print "[debug] change resolution to 1280x720"
        self._locator.simulate_secondary_displays.click.wait()
        self._locator.btn_1280x720.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("1280x720", "secure")
        assert  remsg1 != -1 and remsg2 == -1 , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()
        y = self.device.info["displayHeight"]
        self.device.drag(0, y / 10, 0, y / 5)

    def change_simulate_1280x720_secure(self):
        print "[debug] change resolution to 1280x720 secure"
        self._locator.simulate_secondary_displays.click()
        self._locator.btn_1280x720_secure.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("1280x720", "secure")
        assert not (remsg1 == -1 or remsg2 == -1) , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()
        y = self.device.info["displayHeight"]
        self.device.drag(0, y / 10, 0, y / 5)

    def change_simulate_1920x1080(self):
        print "[debug] change resolution to 1920x1080"
        self._locator.simulate_secondary_displays.click()
        self._locator.btn_1920x1080.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("1920x1080", "secure")
        assert  remsg1 != -1 and remsg2 == -1 , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()
        y = self.device.info["displayHeight"]
        self.device.drag(0, y / 10, 0, y / 5)

    def change_simulate_1920x1080_secure(self):
        print "[debug] change resolution to 1920x1080 secure"
        self._locator.simulate_secondary_displays.click.wait()
        self._locator.btn_1920x1080_secure.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("1920x1080", "secure")
        assert not (remsg1 == -1 or remsg2 == -1) , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()
        y = self.device.info["displayHeight"]
        self.device.drag(0, y / 10, 0, y / 5)

    def change_simulate_1280x720_and_1920x1080(self):
        print "[debug] change resolution to 1280x720 and 1920x1080"
        self._locator.simulate_secondary_displays.click.wait()
        self._locator.btn_1280x720_and_1920x1080.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("1280x720", "1920x1080")
        assert not (remsg1 == -1 or remsg2 == -1) , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()
        y = self.device.info["displayHeight"]
        self.device.drag(0, y / 10, 0, y / 5)

    def change_simulate_480p(self):
        print "[debug] change resolution to 480p"
        self._locator.simulate_secondary_displays.click.wait()
        self._locator.btn_480p.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("720x480", "secure")
        assert  remsg1 != -1 and remsg2 == -1 , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()

    def change_simulate_480p_secure(self):
        print "[debug] change resolution to 480p secure"
        self._locator.simulate_secondary_displays.click.wait()
        self._locator.btn_480p_secure.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("720x480", "secure")
        assert not (remsg1 == -1 or remsg2 == -1) , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()

    def change_simulate_720p(self):
        print "[debug] change resolution to 720p"
        self._locator.simulate_secondary_displays.click.wait()
        self._locator.btn_720p.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("1280x720", "secure")
        assert  remsg1 != -1 and remsg2 == -1 , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()

    def change_simulate_720p_secure(self):
        print "[debug] change resolution to 720p secure"
        self._locator.simulate_secondary_displays.click.wait()
        self._locator.btn_720p_secure.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("1280x720", "secure")
        assert not (remsg1 == -1 or remsg2 == -1) , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()

    def change_simulate_1080p(self):
        print "[debug] change resolution to 1080p"
        self._locator.simulate_secondary_displays.click()
        self._locator.btn_1080p.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("1920x1080", "secure")
        assert  remsg1 != -1 and remsg2 == -1 , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()

    def change_simulate_1080p_secure(self):
        print "[debug] change resolution to 1080p secure"
        self._locator.simulate_secondary_displays.click()
        self._locator.btn_1080p_secure.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("1920x1080", "secure")
        assert not (remsg1 == -1 or remsg2 == -1) , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()

    def change_simulate_4K(self):
        print "[debug] change resolution to 4K"
        self._locator.simulate_secondary_displays.click()
        self._locator.btn_4K.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("3840x2160", "secure")
        assert  remsg1 != -1 and remsg2 == -1 , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()

    def change_simulate_4K_secure(self):
        print "[debug] change resolution to 4K secure"
        self._locator.simulate_secondary_displays.click()
        self._locator.btn_4K_secure.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("3840x2160", "secure")
        assert not (remsg1 == -1 or remsg2 == -1) , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()

    def change_simulate_4K_upscaled(self):
        print "[debug] change resolution to 4K upscaled"
        self._locator.simulate_secondary_displays.click()
        self._locator.btn_4K_upscaled.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("1920x1080", "secure")
        assert  remsg1 != -1 and remsg2 == -1 , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()

    def change_simulate_4K_upscaled_secure(self):
        print "[debug] change resolution to 4K upscaled secure"
        self._locator.simulate_secondary_displays.click()
        self._locator.btn_4K_upscaled_secure.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("1920x1080", "secure")
        assert not (remsg1 == -1 or remsg2 == -1) , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()

    def change_simulate_720p_1080p_dualscreen(self):
        print "[debug] change resolution to btn_720p_1080p_dualscreen"
        self._locator.simulate_secondary_displays.click.wait()
        print "--------------GFX_SimulateSecondaryDisplay_Sleep--------------------"
        from testlib.graphics.screenshot_for_liverpt import take_screenshot_for_liverpt
        take_screenshot_for_liverpt()
        self._locator.btn_720p_1080p_dualscreen.click.wait()
        self.power_off_device()
        time.sleep(1)
        self.power_on_device()
        remsg1, remsg2, msg = self.check_resolution("1280x720", "1920x1080")
        assert not (remsg1 == -1 or remsg2 == -1) , "dumpsys msg is %s" % msg
        self.debugmode.set_secondary_displays_none()
