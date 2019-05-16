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

import time

from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.camera.camera_impl import CameraImpl
from testlib.graphics.common import get_current_focus_window
from testlib.graphics.common import Allow_Permission


class Locator(object):

    """
    locator
    """

    def __init__(self, device):
        self.d = device

    @property
    def wait_exist(self):
        """ wait until the ui object exist """
        def _wait(uiobj, timeout=5):
            return uiobj.wait("exists", timeout * 500)
        return _wait

    @property
    def performance_tests(self):
        return self.d(text="Performance Tests")


class CameraExtendImpl(CameraImpl):

    """Camera Extend Implement"""

    config_file = 'tests.common.camera.conf'

    def __init__(self):
        super(CameraExtendImpl, self).__init__()
        self._locator = Locator(self.d)
        configer = TestConfig()
        self.config = configer.read(self.config_file, "Camera")
        self.dut_camera_dir = self.config.get("dut_camera_dir")
        self.a = Allow_Permission()

    def startApp(self):
        self.launch_camera_am()

    def launch_camera_am(self):
        g_common_obj.launch_app_am("com.android.camera2", "com.android.camera.CameraLauncher")
        time.sleep(2)
        _, ACTname = get_current_focus_window()
        # if not ACTname == "com.android.camera.CameraActivity":
        #    g_common_obj.launch_app_am("com.android.camera2", "com.android.camera.CameraLauncher")
        for _ in range(0, 6):
            time.sleep(2)
            self.a.permission_allower()
            if self.d(text="NEXT").exists:
                self.d(text="NEXT").click.wait()
            if self.d(resourceId="com.android.camera2:id/ok_button").exists:
                self.d(resourceId="com.android.camera2:id/ok_button").click.wait()
            if self.d(text="com.android.camera2:id/shutter_button").exists:
                break

    @staticmethod
    def stop_camera_am():
        """ Stop Camera via adb am command
        """
        print "Stop Camera by adb am"
        g_common_obj.stop_app_am("com.google.android.GoogleCamera")
        PKGname, _ = get_current_focus_window()
        if PKGname == "com.android.camera2":
            g_common_obj.stop_app_am("com.android.camera2")

    def picTake(self):
        self.launch_camera_am()
        self.cameraSwitchTo("Camera")
        self.takePics(1)

    def videoRecord(self):
        self.launch_camera_am()
        self.cameraSwitchTo("Video")
        self.videoRecordStart()
        time.sleep(30)
        self.videoRecordEnd()

    def cameraSwitchTo(self, switchText):
        self.d().scroll.horiz.backward()
        time.sleep(0.5)
        if self.d(text=switchText).exists:
            self.d(text=switchText).click()
        else:
            self.d().swipe.right()
            time.sleep(0.5)
            self.d(text=switchText).click()
        time.sleep(1)

    def cameraSwitchBackFront(self, to=None):
        _toDict = {"back": "Back camera", "front": "Front camera"}
        if isinstance(to, basestring):
            to = to.lower()
        if to in _toDict:
            to = _toDict[to]
        time.sleep(2)
        bounds = self.d(description="Options").bounds
        shutter_top = bounds.get("top")
        shutter_bottom = bounds.get("bottom")
        shutter_left = bounds.get("left")
        shutter_right = bounds.get("right")
        x = (shutter_left + shutter_right) / 2
        y = (shutter_top + shutter_bottom) / 2
        self.d(description="Options").click()
        time.sleep(1)
        if not to:
            if self.d(description="Back camera").exists:
                switchTo = "Front camera"
            if self.d(description="Front camera").exists:
                switchTo = "Back camera"
        else:
            switchTo = to
        if self.d(description=switchTo).exists:
            self.d.click(x, y)
            return True
        if self.d(resourceId="com.android.camera2:id/camera_toggle_button").exists:
            self.d(resourceId="com.android.camera2:id/camera_toggle_button").click()
        time.sleep(2)
        ret = self.d(description=switchTo).exists
        for i in range(3):
            time.sleep(2)
            self.d.click(x, y)
            time.sleep(1)
            if not self.d(resourceId="com.android.camera2:id/camera_toggle_button").exists:
                break
        else:
            CameraImpl().getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "Couldn't close the option bar"
        return ret

    def setVideoTo720p(self):
        self.d().scroll.horiz.backward(steps=5)
        self.d.wait.update()
        CameraImpl.setVideoTo720p(self)

    def setVideoTo1080p(self):
        self.d().scroll.horiz.backward(steps=5)
        self.d(resourceId="com.android.camera2:id/settings_button").click()
        time.sleep(0.5)
        self.d(text="Resolution & quality").click()
        self.d(text="Back camera video").click()
        self.d(text="HD 1080p").click()
        self.d.press.back()
        self.d.press.back()

    def switch_rear_front_camera(self):
        """ switch rear front camera
        """
        print "Switch rear front camera"
        self.d(description="Options").click()
        self.d(
            resourceId="com.android.camera2:id/camera_toggle_button").click()
        assert self.d(
            description="Front camera").exists, "Swith camera not success"
        time.sleep(2)
        self.d(
            resourceId="com.android.camera2:id/camera_toggle_button").click()
        assert self.d(
            description="Back camera").exists, "Swith camera not success"
        time.sleep(3)
        self.d(description="Options").click()
        self.d(
            resourceId="com.android.camera2:id/camera_toggle_button").click()

    def set_camera_as_back_camera(self):
        """ set camera as back camera
        """
        print "set camera as back camera"
        self.launch_camera_am()
        self.d(description="Options").click()
        if self.d(description="Front camera").exists:
            self.d(
                resourceId="com.android.camera2:id/camera_toggle_button").click()
        self.stop_camera_am()

    def switch_rear_front_camera_100times(self):
        """ switch rear front camera 100times
        """
        self.d(description="Options").click()
        for i in range(1, 51):
            print "Switch rear front camera %s time" % i
            self.d(
                resourceId="com.android.camera2:id/camera_toggle_button").click.wait()
            time.sleep(2)
            g_common_obj.assert_exp_happens()
            assert self.d(
                description="Front camera").exists, "Swith camera not success"
            time.sleep(1)
            self.d(
                resourceId="com.android.camera2:id/camera_toggle_button").click.wait()
            time.sleep(2)
            g_common_obj.assert_exp_happens()
            assert self.d(
                description="Back camera").exists, "Swith camera not success"
            time.sleep(1)

    def capture_video(self, loop):
        """ capture a video with camera
        """
        print "Capture a video"
        x = self.d.info["displayWidth"]
        self.d.swipe(0, 0, x, 0)
        time.sleep(5)
        self.d(text="Video").click()
        time.sleep(5)
        for _ in range(0, int(loop)):
            self.d(resourceId="com.android.camera2:id/shutter_button").click()
            print "Recoding..."
            time.sleep(10)
            self.d(resourceId="com.android.camera2:id/shutter_button").click()
            print "Stop Recoding"
            time.sleep(2)

    def swith_camera_to_capture_mode(self):
        """ Swith back to capture
        """
        print "Swith back to capture"
        self.launch_camera_am()
        x = self.d.info["displayWidth"]
        self.d.swipe(0, 0, x, 0)
        time.sleep(5)
        self.d(text="Camera").click()
        time.sleep(2)
        self.stop_camera_am()

    def delete_captured_video(self):
        """ delete video captured
        """
        print "delete the video captured"
        cmd = "rm -rf /sdcard/DCIM/Camera"
        g_common_obj.adb_cmd(cmd)
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED \
        -d file:///sdcard/"
        g_common_obj.adb_cmd(cmd)

    def capture_20_times(self):
        """ Capture pictures continues 20 times
        """
        for _ in range(0, 20):
            self.d(resourceId="com.android.camera2:id/shutter_button").click()
            time.sleep(1)
            g_common_obj.assert_exp_happens()

    @staticmethod
    def delete_capture_pictures():
        """ delete the pictures captured
        """
        print "Delete the photos"
        cmd = "rm -rf /sdcard/DCIM/Camera"
        g_common_obj.adb_cmd(cmd)
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED \
        -d file:///sdcard/"
        g_common_obj.adb_cmd(cmd)

    def switch_back_home_serveraltimes(self, switch_times=100):
        """ launch camera switch_times times
        """
        for i in range(1, switch_times + 1):
            print "Lanch Camera %s times" % i
            g_common_obj.adb_cmd_common("shell am start -S -n com.google.android.GoogleCamera/com.android.camera.CameraActivity")
            time.sleep(2)
            if self.d(text="ALLOW").exists: self.d(text="ALLOW").click.wait()
            _, ACTname = get_current_focus_window()
            if not ACTname == "com.android.camera.CameraActivity":
                g_common_obj.launch_app_am("com.android.camera2", "com.android.camera.CameraLauncher")
            self.d.press.home()
            self.d.wait.idle()

    def camera_rotate_capture(self):
        """ Rotate device and capture photos
        """
        self.d.orientation = "l"
        time.sleep(2)
        self.d(resourceId="com.android.camera2:id/shutter_button").click()
        time.sleep(2)
        self.d.orientation = "r"
        time.sleep(2)
        self.d(resourceId="com.android.camera2:id/shutter_button").click()
        time.sleep(2)
        self.d.orientation = "n"

    def get_stored_pics(self):
        cmd = \
            "ls -l %s|grep -i .jpg|awk '{print $7}'"\
            % (self.dut_camera_dir)
        msg = g_common_obj.adb_cmd_capture_msg(cmd)
        pics = msg.splitlines()
        return pics

    def clean(self):
        dut_shell = \
            "rm -rf %s/*; sync; "\
            "am broadcast -a android.intent.action.MEDIA_MOUNTED -d file://%s"\
            % (self.dut_camera_dir, self.dut_camera_dir)
        g_common_obj.adb_cmd_capture_msg(repr(dut_shell))

    def launch_previewrs_am(self):
        """ Launch PreviewRS via adb am command
        """
        print "Launch PreviewRS by adb am"
        g_common_obj.launch_app_am("com.android.rs.livepreview",
                                   "com.android.rs.livepreview.CameraPreviewActivity")
        self._locator.wait_exist(self._locator.performance_tests)

    @staticmethod
    def stop_previewrs_am():
        """ Stop PreviewRS via adb am command
        """
        print "Stop PreviewRS by adb am"
        g_common_obj.stop_app_am("com.android.rs.livepreview")

    def install_apk(self, apk_name):
        """ Install the apk from artifactory
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.basic_render_script.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        arti = Artifactory(cfg_arti.get('location'))
        cfg_apk = config.read(cfg_file, apk_name)
        apk_name = cfg_apk.get("name")
        file_path = arti.get(apk_name)
        g_common_obj.adb_cmd_common('install ' + file_path)

    def uninstall_apk(self):
        g_common_obj.adb_cmd_common('uninstall com.android.rs.livepreview')

    def switch_to_front_camera_rs(self):
        """ switch to front camera
        """
        self.d(resourceId="com.android.rs.livepreview:id/cameras_selection").click()
        self.d(text="Camera 1").click()
        g_common_obj.assert_exp_happens()
        time.sleep(3)

    def switch_to_back_camera_rs(self):
        """ switch to front camera
        """
        self.d(resourceId="com.android.rs.livepreview:id/cameras_selection").click()
        self.d(text="Camera 0").click()
        time.sleep(3)
