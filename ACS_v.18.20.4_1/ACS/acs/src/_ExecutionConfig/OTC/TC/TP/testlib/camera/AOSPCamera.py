'''
Created on Oct 26, 2015

@author: shankang
'''
import os
import time
from testlib.util.common import g_common_obj
from testlib.camera.camera_log import CameraLogger
from testlib.camera.CameraCommon import CameraCommon
from testlib.camera.GMSCamera import GMSCamera
from testlib.camera.cameratestbase import CameraTestBase

class AOSPCamera(GMSCamera):
    PACKAGE_NAME_AOSPCAMERA = "com.android.camera2"
    ACTIVITY_NAME_AOSPCAMERA = 'com.android.camera.CameraLauncher'

    def __init__(self, cfg=None):
        if cfg == None:
            self.cfg = self.config.read(CameraCommon.DEFAULT_CONFIG_FILE, "multimedia_camera")
        else:
            self.cfg = cfg
        self.d = g_common_obj.get_device()
        self.logger = CameraLogger.instance()
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]

    class AOSPCameraWidget:

        def __init__(self):
            self.d = g_common_obj.get_device()

        def camera(self):
            '''
            text is Camera
            '''
            return self.d(text="Camera")

#=============================================================
    def disableOrEnableCamera(self, tag):
        return CameraCommon().disableOrEnableCamera(tag, self.PACKAGE_NAME_AOSPCAMERA)

    def openCameraFromLockScreenIcon(self, camera):
        CameraCommon().openCameraFromLockScreenIcon(self.PACKAGE_NAME_AOSPCAMERA, camera)

#------------------------------------------------------------
# Interfaces' implementations are from here

    def cleanMediaFiles(self):
        """
        Used to clean the media files in some folders; eg. /mnt/sdcard/DCIM/Camera
        """
        self.logger.debug("clean media files start")
        g_common_obj.stop_app_am(self.PACKAGE_NAME_AOSPCAMERA)
        g_common_obj.adb_cmd("pm clear %s" % self.PACKAGE_NAME_AOSPCAMERA)
        self.logger.debug("clean media files successfully")

    def checkCameraApp(self):
        time_left = 0
        start = time.time()
        success = False
        while time_left < CameraCommon().waitStartAppTime + 15:
            if self.isShutterBtnExists() or self.GMSCameraWidget().overlay().exists:
                success = True
                break
            if self.GMSCameraWidget().text("NEXT").exists or self.GMSCameraWidget().text("Allow").exists or \
                    self.GMSCameraWidget().text("ALLOW").exists:
                CameraCommon().checkGuide()
                android_version = CameraCommon().getAndroidVersion()
                if android_version == "N" or android_version == "O":
                    self.stopCameraApp()
                    os.system("adb shell am start -n %s/%s" % (self.PACKAGE_NAME_AOSPCAMERA, self.ACTIVITY_NAME_AOSPCAMERA))
            time_left = time.time() - start
            time.sleep(0.5)
            CameraCommon().checkCameraCrash()
        if (not success) or time_left > CameraCommon().waitStartAppTime + 15:
            scname = g_common_obj.get_user_log_dir() + "/assert.png"
            g_common_obj.take_screenshot(scname)
            assert False, "aosp camera launch fail,launch time is greater than " \
                        + str(CameraCommon().waitStartAppTime + 15) + " seconds"
        self.logger.debug("launch aosp camera successfully")

    def startCameraApp(self):
        """
        Used to start the camera application
        """
        CameraCommon().unlockScreen()
        self.logger.debug("launch aosp camera start")
        os.system("adb shell am start -S -n %s/%s" % (self.PACKAGE_NAME_AOSPCAMERA, self.ACTIVITY_NAME_AOSPCAMERA))
        self.checkCameraApp()

    def stopCameraApp(self):
        """
        Used to stop the camera application
        """
        g_common_obj.stop_app_am(self.PACKAGE_NAME_AOSPCAMERA)
#         g_common_obj.adb_cmd("pm clear %s" % self.PACKAGE_NAME_CAMERA)
        self.logger.debug("stop aosp camera app successfully")

