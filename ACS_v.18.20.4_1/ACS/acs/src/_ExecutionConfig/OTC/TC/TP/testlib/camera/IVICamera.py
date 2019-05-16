'''
Created on Apr 13, 2017

@author: Li Zixi
'''
import os
import time
from testlib.util.common import g_common_obj
from testlib.camera.camera_log import CameraLogger
from testlib.camera.CameraCommon import CameraCommon
from testlib.camera.cameratestbase import CameraTestBase

from testlib.multimedia.relay08_helper import Relay08Helper

class IVICamera(CameraTestBase):

    POWER_REALY = "power_relay"
    CAMERA_RELAY = "camera_relay"

    def __init__(self, cfg=None):
        if cfg == None:
            self.cfg = self.config.read(CameraCommon.DEFAULT_CONFIG_FILE, "multimedia_camera")
        else:
            self.cfg = cfg
        self.d = g_common_obj.get_device()
        self.camera_common = CameraCommon()
        self.logger = CameraLogger.instance()
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]

        self.relay08_helper = Relay08Helper()
        self.waitStartCameraTime = 15

    class IVICameraWidget:

        def __init__(self):
            self.d = g_common_obj.get_device()

        def homeLayout(self):
            '''
            layout capture
            '''
            return self.d(packageName="com.android.launcher")

        def lockLayout(self):
            '''
            layout capture
            '''
            return self.d(packageName="com.android.systemui")

        def cameraLayout(self):
            '''
            layout capture
            '''
            return self.d(packageName="com.intel.rvc")

#=============================================================

    def checkCameraCrash(self):
        self.camera_common.checkCameraCrash()

    def isHomeLayoutExists(self):
        if self.d(text="Got it").exists:
            self.d(text="Got it").click.wait()
        if self.camera_common.isWidgetExists(self.IVICameraWidget().homeLayout()) == True:
            return True
        else:
            return self.camera_common.isWidgetExists(self.d(packageName="com.android.launcher3"))

    def isLockLayoutExists(self):
        return self.camera_common.isWidgetExists(self.IVICameraWidget().lockLayout())

    def isCameraLayoutExists(self):
        return self.camera_common.isWidgetExists(self.IVICameraWidget().cameraLayout())

    def swipeScreen(self, orientation="right"):
        # Don't need to implement
        pass

#------------------------------------------------------------
# Interfaces' implementations are from here

    def clean_media_files(self):
        """
        Used to clean the media files in some folders; eg. /mnt/sdcard/DCIM/Camera
        """
        self.logger.debug("clean media files start")
        g_common_obj.stop_app_am(self.PACKAGE_NAME_ARCSOFTCAMERA)
        g_common_obj.adb_cmd("pm clear %s" % self.PACKAGE_NAME_ARCSOFTCAMERA)
        self.logger.debug("clean media files successfully")

    def reboot_device(self):
        self.pressPowerKey(10)
        time.sleep(5)
        self.pressPowerKey(2)
#         from testlib.common.common import g_common_obj2
#         g_common_obj2.system_reboot(30)
        time.sleep(40)
        g_common_obj.root_on_device()
#         g_common_obj.remount_device()
        os.system("adb shell uiautomator dump /data/local/tmp/uidump.xml")#get layout, 20170822 has bug, can't find layout!
        self.camera_common.unlockScreen()
        self.backHome()
        self.check_home_or_lock_layout()

    def startCameraApp(self, check_success=1):
        """
        Used to start the camera application
        """
        self.logger.debug("launch ivi camera start")
        self.relay08_helper.set_relay_NO(IVICamera.CAMERA_RELAY)
        start = time.time()
        success = False
        if check_success == 1:
            while time.time() - start < self.waitStartCameraTime:
                if self.isCameraLayoutExists():
                    success = True
                    break
                time.sleep(1)
                self.checkCameraCrash()
            if not success:
                scname = g_common_obj.get_user_log_dir() + "/assert.png"
                g_common_obj.take_screenshot(scname)
                assert False, "ivi camera launch fail,launch time is greater than " \
                            + str(self.waitStartCameraTime) + " seconds"
            self.logger.debug("launch ivi camera successfully")
        else:
            self.logger.debug("launch ivi camera, skip check function")

    def stopCameraApp(self):
        """
        Used to stop the camera application
        """
        self.relay08_helper.set_relay_NC(IVICamera.CAMERA_RELAY)
        self.logger.debug("stop ivi camera app successfully")

    def pressPowerKey(self, duration=4):
        self.logger.debug("press power key start")
        self.relay08_helper.set_relay_NO(IVICamera.POWER_REALY)
        time.sleep(duration)
        self.relay08_helper.set_relay_NC(IVICamera.POWER_REALY)

    def backHome(self):
        self.d.press.home()

    def skipAccountLoginLyout(self):
        pass

    def check_home_or_lock_layout(self, check_exist=True):
        assert self.isHomeLayoutExists() or self.isLockLayoutExists(), "Home or Lock layout not exist!"