'''
Created on May 25, 2017

@author: Li Zixi
'''
import os
import time
from testlib.util.common import g_common_obj
from testlib.camera.camera_log import CameraLogger
from testlib.camera.CameraCommon import CameraCommon
from testlib.camera.cameratestbase import CameraTestBase
from testlib.camera.CameraAppInterface import CameraAppInterface
from testlib.multimedia.multimedia_setting import MultiMediaSetting
# from reportlab.graphics.shapes import rotate


class RefCamCamera(CameraAppInterface, CameraTestBase):
    PACKAGE_NAME_REFCAMCAMERA = "com.intel.refcam"
    ACTIVITY_NAME_REFCAMCAMERA = ".CameraActivity"

    PERMISSION_LIST = ["android.permission.ACCESS_FINE_LOCATION"
                       ,"android.permission.CAMERA"
                       ,"android.permission.WRITE_EXTERNAL_STORAGE"
                       ,"android.permission.READ_EXTERNAL_STORAGE"
                       ,"android.permission.RECORD_AUDIO"
                       ]

    def __init__(self, cfg=None):
        if cfg == None:
            self.cfg = self.config.read(CameraCommon.DEFAULT_CONFIG_FILE, "multimedia_camera")
        else:
            self.cfg = cfg
        self.d = g_common_obj.get_device()
        self.logger = CameraLogger.instance()
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]

        self.camera_widget = self.CameraWidget()

        self.camera_common = CameraCommon()
        self.multimedia_setting = MultiMediaSetting(self.camera_common.DEFAULT_CONFIG_FILE)
        self.multimedia_setting.install_apk("ref_camera_1_apk")
        self.camera_common.grantPermission(self.PACKAGE_NAME_REFCAMCAMERA, self.PERMISSION_LIST)

    class CameraWidget:

        def __init__(self):
            self.d = g_common_obj.get_device()

        def frontback(self):
            '''
            front back button
            '''
            return self.d(resourceId="com.intel.refcam:id/switch_frontback")

        def itemChildMenu(self):
            '''
            item clild menu
            '''
            return self.d(resourceId="com.intel.refcam:id/setting_item_child_menu_text")

        def seekBar(self):
            '''
            menu seek bar
            '''
            return self.d(resourceId="com.intel.refcam:id/setting_item_child_menu_seekbar")

        def shutterBtn(self):
            '''
            shutter button
            '''
            return self.d(resourceId="com.intel.refcam:id/shutter")

        def shotlist(self):
            '''
            front back button
            '''
            return self.d(resourceId="com.intel.refcam:id/open_shot_list")

        def settingsBtn(self):
            '''
            settings
            '''
            return self.d(resourceId="com.intel.refcam:id/setting")

        def text(self, _text):
            '''
            page text
            '''
            return self.d(textContains=_text)

        def itemChildMenuText(self, _text):
            '''
            page text
            '''
            return self.d(resourceId="com.intel.refcam:id/setting_item_child_menu_text", textContains=_text)

        def zoomLayer(self):
            '''
            zoom layer
            '''
            return self.d(resourceId="com.intel.refcam:id/zoom_layer")

#=============================================================

    def clickScreen(self):
        d = g_common_obj.get_device()
        x = d.info["displayWidth"]
        self.camera_common.clickBtn(x-100, 300)

    def isShutterBtnExists(self):
        return self.camera_common.isWidgetExists(self.camera_widget.shutterBtn())

    def setCaptureMode(self, value):
        self.camera_common.waitForWidgetToAppear(self.camera_widget.shotlist(), "mode button")
        self.camera_widget.shotlist().click.wait()
        self.camera_widget.text(value).click.wait()
        self.logger.debug("set capture mode to %s successfully" % value)
        self.camera_common.clickScreenCenter()

#------------------------------------------------------------
# Interfaces' implementations are from here

    def cleanMediaFiles(self):
        """
        Used to clean the media files in some folders; eg. /mnt/sdcard/DCIM/Camera
        """
        self.logger.debug("clean media files start")
        g_common_obj.stop_app_am(self.PACKAGE_NAME_REFCAMCAMERA)
        g_common_obj.adb_cmd("pm clear %s" % self.PACKAGE_NAME_REFCAMCAMERA)
        self.logger.debug("clean media files successfully")

    def startCameraApp(self):
        """
        Used to start the camera application
        """
        self.camera_common.unlockScreen()
        self.logger.debug("launch refcam2 camera start")
        os.system("adb shell am start -S %s/%s" % (self.PACKAGE_NAME_REFCAMCAMERA, self.ACTIVITY_NAME_REFCAMCAMERA))
        time_left = 0
        start = time.time()
        success = False
        while time_left < self.camera_common.waitStartAppTime + 15:
            if self.isShutterBtnExists() or self.camera_widget.shotlist().exists:
                success = True
                break
            if self.camera_widget.text("Allow").exists or self.camera_widget.text("ALLOW").exists:
                self.camera_common.checkGuide()
                if self.camera_common.getAndroidVersion() == "N":
                    self.stopCameraApp()
                    os.system("adb shell am start -S %s/%s" % (self.PACKAGE_NAME_REFCAMCAMERA, self.ACTIVITY_NAME_REFCAMCAMERA))
            time_left = time.time() - start
            time.sleep(0.5)
            self.camera_common.checkCameraCrash()
        if (not success) or time_left > self.camera_common.waitStartAppTime + 15:
            scname = g_common_obj.get_user_log_dir() + "/assert.png"
            g_common_obj.take_screenshot(scname)
            assert False, "refcam2 camera launch fail,launch time is greater than " \
                        + str(self.camera_common.waitStartAppTime + 15) + " seconds"
        self.logger.debug("launch refcam2 camera successfully")

    def stopCameraApp(self):
        """
        Used to stop the camera application
        """
        g_common_obj.stop_app_am(self.PACKAGE_NAME_REFCAMCAMERA)
        self.logger.debug("stop arcsoft camera app successfully")

    def selectMode(self, mode="Camera"):
        """
        Used to select a mode such as camera, video, panorama, lens blur, photo sphere and so on...
        """
        self.camera_common.waitForWidgetToAppear(self.camera_widget.shutterBtn(), "shutter button")
        self.camera_widget.shotlist().click.wait()
        self.camera_widget.text(mode).click.wait()
        self.camera_common.clickScreenCenter()
        self.logger.debug("Change to %s mode" % mode)

    def switchRearOrFront(self, lens="Back"):
        """
        Used to switch rear or front camera
        lens = Back / Front
        """
        self.camera_common.waitForWidgetToAppear(self.camera_widget.shutterBtn(), "shutter button")
        if lens == "Front":
            self.camera_widget.frontback().click.wait()
            self.logger.debug("change to " + str(lens) + " camera")
        if lens == "Back" or lens == "Rear":
            self.logger.debug("change to " + str(lens) + " camera")
        time.sleep(2)

    def setExposure(self, value):
        pass

    def setFlash(self, flash="off"):
        """
        Used to control the flash; on, off, auto
        """
        pass

    def setGrid(self, grid="off"):
        """
        Used to control the grid; on, off
        """
        pass

    def setTimer(self, timer="off"):
        """
        Used to control the timer
        value: off/2s/3s/10s
        """
        pass

    def getAllVideoResolutions(self, lens):
        """
        Return all of the video resolutions
        """
        pass

    def setVideoResolution(self, resolution, lens):
        """
        Used to control the video resolution, used with the getAllVideoResolutions
        """
        pass

    def getAllCameraMode(self):
        pass

    def getAllPhotoResolutions(self, lens, type="Capture Size (JPEG)"):
        """
        Return all of the photo resolutions
        """
        self.camera_common.waitForWidgetToAppear(self.camera_widget.settingsBtn(), "setting button")
        self.camera_widget.settingsBtn().click.wait()
        if not self.camera_widget.text(type).exists:
            self.d(scrollable=True).scroll.vert.to(textContains=type)
        self.camera_common.waitForWidgetToAppear(self.camera_widget.text(type), "capture size button")
        self.camera_widget.text(type).click.wait()
        mCount = self.camera_widget.itemChildMenu().count
        mList=[]
        for i in range(mCount):
            mList.append(self.camera_widget.itemChildMenu()[i].info["text"])
        self.clickScreen()
        self.logger.debug("mList=%s" % mList)
        if mList == []:
            self.logger.debug("can't find  photo resolutions with \"%s\" type!" % type)
            return [], -1, -1
        else:
            return mList,mList[0],mList[mCount-1]

    def setSettingsButton(self, main_click_button, sub_click_button):
        self.camera_common.waitForWidgetToAppear(self.camera_widget.settingsBtn(), "setting button")
        self.camera_widget.settingsBtn().click.wait()
        if not self.camera_widget.text(main_click_button).exists:
            self.d(scrollable=True).scroll.vert.to(textContains=main_click_button)
        self.camera_common.waitForWidgetToAppear(self.camera_widget.text(main_click_button), "click main button: %s" % main_click_button )
        self.camera_widget.text(main_click_button).click.wait()
        self.camera_widget.itemChildMenuText(sub_click_button).click.wait()
        self.logger.debug("click sub button: %s" % sub_click_button)
        self.clickScreen()

    def setPhotoResolution(self, resolution, lens, type="Capture Size (JPEG)"):
        """
        Used to control the photo resolution, used with the getAllPhotoResolutions
        """
        self.camera_common.waitForWidgetToAppear(self.camera_widget.settingsBtn(), "setting button")
        self.camera_widget.settingsBtn().click.wait()
        if not self.camera_widget.text(type).exists:
            self.d(scrollable=True).scroll.vert.to(textContains=type)
        self.camera_common.waitForWidgetToAppear(self.camera_widget.text(type), "capture size button")
        self.camera_widget.text(type).click.wait()
        self.camera_widget.itemChildMenuText(resolution).click.wait()
        self.logger.debug("set resolution to " + resolution)
        self.clickScreen()

    def capturePhoto(self, num=1, generated=True):
        """
        Used to capture num photos
        """
        self.camera_common.waitForWidgetToAppear(self.camera_widget.shutterBtn(), "shutter button")
        for i in range(int(num)):
            if self.camera_widget.shutterBtn().exists:
                self.camera_widget.shutterBtn().click.wait()
                self.logger.debug("capture photo %d" % (i + 1))
                if generated:
                    self.camera_common.waitForTheFilesAreGenerated()
                time.sleep(1.5)
            else:
                self.camera_common.checkCameraCrash()

    def reviewPhoto(self, num=1):
        """
        Used to review num photos
        """
        pass

    def reviewPhotoAndVideo(self, num=1, timeout=10):
        """
        Used to review num photos
        """
        pass

    def recordVideo(self, videoNum=1, duration=5):
        """
        Used to capture num duration videos
        """
        pass

    def reviewVideo(self, num=1, duration=1):
        """
        Used to review num duration videos
        """
        pass

    def snapShotDuringVideo(self, videoNum=1, duration=5, snapShotNum=1):
        """
        Used to snapshot num pictures during a duration videos
        """
        pass
