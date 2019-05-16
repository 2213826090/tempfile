# coding: UTF-8
import os
import sys
import time
from random import randrange
import ConfigParser
from testlib.common.common import g_common_obj2
from testlib.util.common import g_common_obj
from testlib.common.impl import ImplCommon
from testlib.camera.camera_log import CameraLogger
from testlib.camera.mum_camera_impl import CameraImpl
BASE_PATH = os.path.dirname(__file__)

class Camera(ImplCommon):
    """
    Multi-media functions.
    """

    def launchCamera(self):
        g_common_obj2.launchAppByName("Camera")
        time.sleep(5)
        if self.d(text="NEXT").exists:
            self.d(text="NEXT").click()
        if self.d(resourceId="com.android.camera2:id/ok_button").exists:
            self.d(resourceId="com.android.camera2:id/ok_button").click.wait()

    @staticmethod
    def startApp():
        d = g_common_obj2.get_device()
        g_common_obj2.launch_app_from_home_sc("Camera")
        time.sleep(4)
        if d(text="NEXT").exists:
            d(text="NEXT").click()
        if d(resourceId="com.android.camera2:id/ok_button").exists:
            d(resourceId="com.android.camera2:id/ok_button").click.wait()
        g_common_obj2.back_home()

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
        _toDict = {"back":"Back camera","front":"Front camera"}
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
        x = (shutter_left + shutter_right)/2
        y = (shutter_top + shutter_bottom)/2
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
            self.d.click(x,y)
            return True
        self.d(resourceId="com.android.camera2:id/camera_toggle_button").click()
        time.sleep(2)
        ret = self.d(description=switchTo).exists
        for i in range(3):
            time.sleep(2)
            self.d.click(x,y)
            time.sleep(1)
            if not self.d(resourceId="com.android.camera2:id/camera_toggle_button").exists:
                break
        else:
            CameraImpl().getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "Couldn't close the option bar"
        return ret

    def videoRecord(self):
        self.launchCamera()
        self.cameraSwitchTo("Video")
        self.videoRecordStart()
        time.sleep(30)
        self.videoRecordEnd()

    def picTake(self):
        self.launchCamera()
        self.cameraSwitchTo("Camera")
        self.takePics(1)

    def cameraShutter(self):
        if not self.d(resourceId="com.android.camera2:id/shutter_button", enabled=True).exists:
            CameraImpl().getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert CameraImpl().d(resourceId="com.android.camera2:id/shutter_button", enabled=True).exists
        self.d(resourceId="com.android.camera2:id/shutter_button").click.wait()

    def videoRecordStart(self):
        self.cameraShutter()
        time.sleep(1)

    def videoRecordEnd(self):
        self.cameraShutter()

    def deleteVideo(self):
        self.d().scroll.horiz.forward()
        if not self.d(resourceId="com.android.camera2:id/play_button").exists:
            CameraImpl().getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.d(resourceId="com.android.camera2:id/play_button").exists, "couldn't find preview video"
        self.d(resourceId="com.android.camera2:id/filmstrip_bottom_control_delete").click()
        if self.d(text="ok").exists:
            self.d(text="ok").click()

    def deleteVideoViaAdb(self):
        cmd = "rm /sdcard/DCIM/Camera/VID*.3gp"
        self.commonObj.adb_cmd(cmd)

    def deletePicturesViaAdb(self):
        cmd = "rm /sdcard/DCIM/Camera/IMG*.jpg"
        self.commonObj.adb_cmd(cmd)

    def refreshCameraFolder(self):
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///sdcard/DCIM/Camera"
        self.commonObj.adb_cmd(cmd)

    def takePics(self,num):
        while num>0:
            time.sleep(1)
            self.cameraShutter()
            time.sleep(5)
            num = num-1

    def setVideoTo720p(self):
        self.d(resourceId="com.android.camera2:id/settings_button").click()
        time.sleep(0.5)
        self.d(text="Resolution & quality").click()
        self.d(text="Back camera video").click()
        self.d(text="HD 720p").click()
        self.d.press.back()
        self.d.press.back()

    def checkVideoDisplayWell(self,seconds=5):
        self.d().scroll.horiz.forward()
        if not self.d(resourceId="com.android.camera2:id/play_button").exists:
            CameraImpl().getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.d(resourceId="com.android.camera2:id/play_button").exists, "couldn't find preview video"
        self.d(resourceId="com.android.camera2:id/play_button").click.wait()
        time.sleep(seconds)
        self.d.press.back()

    def checkImageDisplayWell(self):
        self.d().scroll.horiz.forward()
        if not self.d(resourceId="com.android.camera2:id/filmstrip_bottom_control_edit").exists:
            CameraImpl().getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.d(resourceId="com.android.camera2:id/filmstrip_bottom_control_edit").exists, "couldn't find image"
        self.d.press.back()

    def setBackCameraPhoto(self,megapixels):
        self.d(className = "android.widget.FrameLayout").swipe.right()
        self.d(description = "Settings").click.wait()
        self.d(text = "Resolution & quality").click.wait()
        self.d(text = "Back camera photo").click.wait()
        self.d(text = megapixels).click.wait()
        time.sleep(2)
        self.d.press.back()
        time.sleep(2)
        self.d.press.back()

    def setSaveLocation(self):
        self.d(className = "android.widget.FrameLayout").swipe.right()
        self.d(description = "Settings").click.wait()
        if self.d(resourceId = "android:id/switchWidget",text = "OFF").exists:
            self.d(resourceId = "android:id/switchWidget",text = "OFF").click.wait()
        time.sleep(2)
        self.d.press.back()
        time.sleep(5)

    def editImageInCamera(self):
        self.d(className = "android.view.View").swipe.left()
        self.d(className = "android.widget.ImageView").click.wait()
        if not self.d(resourceId = "com.android.camera2:id/filmstrip_bottom_control_edit").exists:
            CameraImpl().getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.d(resourceId = "com.android.camera2:id/filmstrip_bottom_control_edit").exists, "[ERROR]: The button is not found!"
        self.d(resourceId = "com.android.camera2:id/filmstrip_bottom_control_edit").click.wait()
        time.sleep(2)
        CameraLogger.instance().debug("edit")
        self.d(description = "CROP").click.wait()
        time.sleep(2)
        CameraLogger.instance().debug("Done")
        self.d(description = "Apply").click.wait()
        if not self.d(description = "CROP").exists:
            CameraImpl().getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.d(description = "CROP").exists,"[ERROR]: The button is not found!"
        time.sleep(2)
        self.d(text = "Done").click.wait()

    def Capture200ImageAndEdit(self):
        self.setBackCameraPhoto("(4:3) 4.9 megapixels")
        time.sleep(5)
        self.cameraShutter()
        time.sleep(5)
        self.editImageInCamera()

    def takePicWhenRecord(self):
        self.d(resourceId="com.android.camera2:id/labels").click()

    def autoFocus(self):
        time.sleep(3)

    def manualFocus(self):
        self.d(resourceId="com.android.camera2:id/mode_options_overlay").click()
        time.sleep(3)

