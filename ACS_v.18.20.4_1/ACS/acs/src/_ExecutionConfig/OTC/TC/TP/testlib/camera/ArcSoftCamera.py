'''
Created on Oct 26, 2015

@author: shankang
'''
import os
import time
from testlib.util.common import g_common_obj
from testlib.camera.camera_log import CameraLogger
from testlib.camera.CameraCommon import CameraCommon
from testlib.camera.cameratestbase import CameraTestBase
from testlib.camera.CameraAppInterface import CameraAppInterface
from testlib.camera.QuickPic import QuickPicApp

class ArcSoftCamera(CameraAppInterface, CameraTestBase):
    PACKAGE_NAME_ARCSOFTCAMERA = "com.arcsoft.camera2"
    ACTIVITY_NAME_ARCSOFTCAMERA = 'com.arcsoft.camera.CameraLauncher'

    PACKAGE_NAME_QUICKPIC = "com.alensw.PicFolder"
    ACTIVITY_NAME_QUICKPIC = ".GalleryActivity"

    def __init__(self, cfg=None):
        if cfg == None:
            self.cfg = self.config.read(CameraCommon.DEFAULT_CONFIG_FILE, "multimedia_camera")
        else:
            self.cfg = cfg
        self.d = g_common_obj.get_device()
        self.logger = CameraLogger.instance()
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]

    class ArcSoftCameraWidget:

        def __init__(self):
            self.d = g_common_obj.get_device()

        def bestPhotoReviewApproveBtn(self):
            '''
            best photo review approve
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/btn_bestphoto_review_approve")

        def captureLayout(self):
            '''
            layout capture
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/layout_capture")

        def doneBtn(self):
            '''
            done button
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/done_button")

        def flashBtn(self):
            '''
            flash button
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/btn_flash")

        def modeBtn(self):
            '''
            mode button
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/btn_mode")

        def recordBtn(self):
            '''
            record button
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/record_button")

        def recordingTime(self):
            '''
            record time
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/recording_time")

        def settingScrollView(self):
            '''
            setting scroll view
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/setting_camera_scroll_view")

        def settingCameraBtn(self):
            '''
            setting camera button
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/setting_camera_title")

        def settingBtn(self):
            '''
            setting button
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/btn_setting")

        def settingCommonBtn(self):
            '''
            setting common button
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/setting_common_title")

        def settingExpandText(self):
            '''
            setting expand text
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/setting_expand_text")

        def settingProgressSeekBar(self):
            '''
            setting camera scroll view
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/setting_progress_seek_bar")

        def settingVideoBtn(self):
            '''
            setting video button
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/setting_video_title")

        def shutterBtn(self):
            '''
            shutter button
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/shutter_button")

        def snapshotImageView(self):
            '''
            snapshot image view
            '''
            return self.d(description="switch_mode")

        def switchCameraBtn(self):
            '''
            switch camera button
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/btn_switch_camera")

        def text(self, _text):
            '''
            page text
            '''
            return self.d(textContains=_text)

        def topBar(self):
            '''
            top bar
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/top_bar")

        def videoQualityText(self):
            '''
            video quality text
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/setting_expand_text")

        def chooseFlashOff(self):
            '''
            set flash to off
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/icon_image_view")

        def chooseFlashAuto(self):
            '''
            set flash to auto
            '''
            return self.chooseFlashOff().right(resourceId="com.arcsoft.camera2:id/icon_image_view")

        def chooseFlashOn(self):
            '''
            set flash to on
            '''
            return self.chooseFlashAuto().right(resourceId="com.arcsoft.camera2:id/icon_image_view")

        def chooseMode(self, mode):
            '''
            Auto, Smart, Beauty, Best photo, HDR, Panorama, Smile, Multi angle
            '''
            return self.d(resourceId="com.arcsoft.camera2:id/title_text_view", text=mode)

        def panoramaPromptMsg(self):
            return self.d(text="Tap shutter button and move toward one direction")

        def photoView(self):
            if CameraCommon().currentDisplayIsVertical():
                return self.shutterBtn().left(className="android.widget.ImageView")
            else:
                return self.shutterBtn().down(className="android.widget.ImageView")

#=============================================================

    def checkColorEffect(self, _text="None"):
        if not self.ArcSoftCameraWidget().settingCommonBtn().exists:
            self.ArcSoftCameraWidget().settingBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().settingCommonBtn(), "setting common button")
        self.ArcSoftCameraWidget().settingCommonBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().text("Color effect"), "color effect button")
        if self.ArcSoftCameraWidget().text("Color effect").exists and self.ArcSoftCameraWidget().text(_text).exists:
            if not self.ArcSoftCameraWidget().text("Color effect").right(text=_text).exists:
                self.assertTrue(False, "check color effect to %s failure" %_text)
        self.logger.debug("check color effect to %s successfully" % _text)
        CameraCommon().clickScreenCenter()

    def checkLens(self,lens="Back"):
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().switchCameraBtn(), "switch camera button")
        if lens == "Front":
            if not self.ArcSoftCameraWidget().flashBtn().enabled:
                self.logger.debug("check " + str(lens) + " camera successfully")
            else:
                self.assertTrue(False, "check" + str(lens) + " camera failure")
        if lens == "Back" or lens == "Rear":
            if self.ArcSoftCameraWidget().flashBtn().enabled:
                self.logger.debug("check " + str(lens) + " camera successfully")
            else:
                self.assertTrue(False, "check" + str(lens) + " camera failure")
        time.sleep(2)

    def checkModeExists(self,mode):
        self.ArcSoftCameraWidget().modeBtn().click.wait()
        if mode == "Camera":
            self.ArcSoftCameraWidget().chooseMode("Auto").click.wait()
        elif mode == "Video":
            pass
        else:
            if self.ArcSoftCameraWidget().text(mode).exists:
                CameraCommon().clickScreenCenter()
                return True
            else:
                CameraCommon().clickScreenCenter()
                return False

    def checkVideoRecordingButtonStatus(self):
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().shutterBtn(), "shutter button")
        status = False
        if self.ArcSoftCameraWidget().shutterBtn().enabled:
            status = True
        if not status:
            assert False, "check video recording button status fail, actual=False,expected=True"
        else:
            self.logger.debug("check video recording button status successfully")

    def clickShutterBtnArea(self, mode="Camera"):
        """
        @summary: click the area of shutter button
        """
        if mode == "Video":
            mLeft, mTop, mRight, mBottom = CameraCommon().getBounds(self.ArcSoftCameraWidget().recordBtn())
        else:
            mLeft, mTop, mRight, mBottom = CameraCommon().getBounds(self.ArcSoftCameraWidget().shutterBtn())
        mx = mLeft + mRight / 2
        my = mTop + mBottom / 2
        self.logger.debug("width = " + str(mx) + ", height = " + str(my))
        self.d.click(mx, my)
        return mx, my

    def clickShutterBtnWithoutRelease(self, mode="Camera", duration=60):
        if mode == "Video":
            mLeft, mTop, mRight, mBottom = CameraCommon().getBounds(self.ArcSoftCameraWidget().recordBtn())
        else:
            mLeft, mTop, mRight, mBottom = CameraCommon().getBounds(self.ArcSoftCameraWidget().shutterBtn())
        self.d.swipe(mLeft, mTop, mRight + mLeft, mBottom + mTop, steps=duration * 18)

    def clickAllSetting(self):
        lens = "Front"
        mPhotoDict, mPhotoMax, mPhotoMin = self.getAllPhotoResolutions(lens)
        mVideoDict, mVideoMax, MVideoMin = self.getAllVideoResolutions(lens)
        for k in mPhotoDict:
            self.setPhotoResolution(mPhotoDict[k], lens)
        for v in mVideoDict:
            self.setVideoResolution(mVideoDict[v], lens)
        lens = "Back"
        mPhotoDict2, mPhotoMax, mPhotoMin = self.getAllPhotoResolutions(lens)
        mVideoDict2, mVideoMax, MVideoMin = self.getAllVideoResolutions(lens)
        for k in mPhotoDict:
            self.setPhotoResolution(mPhotoDict2[k], lens)
        for v in mVideoDict:
            self.setVideoResolution(mVideoDict2[v], lens)

    def clickDoneBtn(self):
        if self.ArcSoftCameraWidget().doneBtn().exists:
            self.ArcSoftCameraWidget().doneBtn().click.wait()

    def clickbestPhotoApproveBtn(self):
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().bestPhotoReviewApproveBtn(), "best photo review approve")
        self.ArcSoftCameraWidget().bestPhotoReviewApproveBtn().click.wait()
        self.logger.debug("click best photo approve button successfully")

    def clickRecordBtn(self):
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().shutterBtn(), "record button")
        if self.isRecordTimeExists():
            self.ArcSoftCameraWidget().shutterBtn().click()
            self.logger.debug("recording video start")
        else:
            self.ArcSoftCameraWidget().recordBtn().click()
            self.logger.debug("recording  video end")
        time.sleep(0.5)

    def disableOrEnableCamera(self, tag):
        return CameraCommon().disableOrEnableCamera(tag, self.PACKAGE_NAME_ARCSOFTCAMERA)

    def enterPreviewPhotos(self):
        return QuickPicApp().enterPreviewPhotos()

    def enterPreviewVideos(self):
        return QuickPicApp().enterPreviewVideos()

    def faceDetectionOnAndOff(self):
        self.logger.debug("enter setting")
        if not self.ArcSoftCameraWidget().settingCommonBtn().exists:
            self.ArcSoftCameraWidget().settingBtn().click.wait()
        self.ArcSoftCameraWidget().settingCameraBtn().click.wait()
        self.ArcSoftCameraWidget().text("Face detection").click.wait()
        CameraCommon().clickScreenCenter()
        self.logger.debug("switch face detection successfully")
        self.logger.debug("exit setting")

    def getFOVRegion(self):
        width = self.d.info["displayWidth"]
        height = self.d.info["displayHeight"]
        rotate = False
        if width < height:
            rotate = True 
        [x1,y1,w1,h1] = CameraCommon().getBounds(self.ArcSoftCameraWidget().captureLayout())
        print [x1,y1,w1,h1]
        if rotate==False:
            w1 += 20
            x = w1
            y = (height-h1)/2
            w = width - 2*w1
            h = h1
        else:
            h1 += 20
            x = (width-w1)/2
            y = h1
            w = w1
            h = height - 2*h1
        if rotate==True:
            [x,y,w,h] = CameraCommon().rotateRegion([x,y,w,h], width, height, 90)
        return rotate, [x,y,w,h]

    def isRecordTimeExists(self):
        return CameraCommon().isWidgetExists(self.ArcSoftCameraWidget().recordingTime())

    def isShutterBtnExists(self):
        return CameraCommon().isWidgetExists(self.ArcSoftCameraWidget().shutterBtn())

    def isShutterBtnEnabled(self):
        return CameraCommon().isWidgetEnabled(self.ArcSoftCameraWidget().shutterBtn())

    def launchQuickPic(self):
        time.sleep(1)
        os.system("adb shell am start -W -n %s/%s" % (self.PACKAGE_NAME_QUICKPIC, self.ACTIVITY_NAME_QUICKPIC))
        CameraCommon().waitForWidgetToAppear(self.QuickPicWidget().searchBtn(), "search button")
        time.sleep(1)

    def openGEO(self):
        self.logger.debug("enter setting")
        if not self.ArcSoftCameraWidget().settingCommonBtn().exists:
            self.ArcSoftCameraWidget().settingBtn().click.wait()
        self.ArcSoftCameraWidget().settingCameraBtn().click.wait()
        self.ArcSoftCameraWidget().text("GEO").click.wait()
        CameraCommon().clickScreenCenter()
        self.logger.debug("open GEO")
        self.logger.debug("exit setting")

    def openLocation(self):
        self.logger.debug("enter system settings")
        os.system("adb shell am start -W -n %s/%s" % (CameraCommon().PACKAGE_NAME_SETTING, CameraCommon().ACTIVITY_NAME_SETTING))
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().text("Settings"),"Settings page")
        self.d(scrollable=True).scroll.vert.to(textContains="Apps")
        self.ArcSoftCameraWidget().text("Apps").click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().text("Camera"),"Settings apps page")
        mCameraCount = self.ArcSoftCameraWidget().text("Camera").count
        for i in range(mCameraCount):
            self.ArcSoftCameraWidget().text("Camera")[i].click.wait()
            CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().text("Permissions"),"Permissions")
            self.ArcSoftCameraWidget().text("Permissions").click.wait()
            CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().text("App permissions"),"App permissions")
            for j in range(5):
                if self.ArcSoftCameraWidget().text("OFF").exists:
                    self.ArcSoftCameraWidget().text("OFF").click.wait()
            CameraCommon().pressBack(2)
        CameraCommon().pressBack(2)
        g_common_obj.stop_app_am(CameraCommon().PACKAGE_NAME_SETTING)
        self.logger.debug("open arcsoft camera location successfully")
        self.logger.debug("exit system settings")

    def openManualExposure(self):
        # Don't need to implement
        pass

    def openCameraFromLockScreenIcon(self, camera):
        CameraCommon().openCameraFromLockScreenIcon(self.PACKAGE_NAME_ARCSOFTCAMERA, camera)

    def resetCameraSetting(self):
        if not self.ArcSoftCameraWidget().settingCommonBtn().exists:
            self.ArcSoftCameraWidget().settingBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().settingCommonBtn(), "setting common button")
        self.ArcSoftCameraWidget().settingCommonBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().text("Reset"), "Reset button")
        self.ArcSoftCameraWidget().text("Reset").click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().text("OK"), "OK button")
        self.ArcSoftCameraWidget().text("OK").click.wait()
        self.logger.debug("reset arcsoft camera setting successfully")

    def reviewAvailableModes(self):
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().recordBtn(), "record button")
        if self.ArcSoftCameraWidget().recordBtn().exists:
            self.ArcSoftCameraWidget().recordBtn().click.wait()
            self.logger.debug("recording video start")
            time.sleep(3)
            self.ArcSoftCameraWidget().shutterBtn().click.wait()
            self.logger.debug("recording video end")
        else:
            CameraCommon().checkCameraCrash()

    def setCaptureMode(self, value):
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().modeBtn(), "mode button")
        self.ArcSoftCameraWidget().modeBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().text(value), value)
        self.ArcSoftCameraWidget().text(value).click.wait()
        self.logger.debug("set capture mode to %s successfully" % value)
        CameraCommon().clickScreenCenter()

    def setCaptureModeValue(self, category, value="middle"):
        if not self.ArcSoftCameraWidget().settingVideoBtn().exists:
            self.ArcSoftCameraWidget().settingBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().settingCameraBtn(), "setting capture button")
        self.ArcSoftCameraWidget().settingCameraBtn().click.wait()
        if self.ArcSoftCameraWidget().settingScrollView().scrollable:
            self.d(scrollable=True).scroll.vert.to(textContains=category)
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().text(category), "setting %s button" % category)
        self.ArcSoftCameraWidget().text(category).click.wait()
        if self.ArcSoftCameraWidget().text(value).exists:
            self.ArcSoftCameraWidget().text(value).click.wait()
        else:
            CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().settingProgressSeekBar(), "setting %s seek bar" % category)
            bounds = self.ArcSoftCameraWidget().settingProgressSeekBar().bounds
    #         print  bounds["left"], bounds["right"], bounds["top"], bounds["bottom"]
            if value == "middle":
                self.d.click(((bounds["right"] - bounds["left"]) / 2 + bounds["left"]) + 5, \
                             (bounds["bottom"] - bounds["top"]) / 2 + bounds["top"])
            elif value == "low":
                self.d.click(bounds["left"] + 45, \
                             (bounds["bottom"] - bounds["top"]) / 2 + bounds["top"])
            elif value == "high":
                self.d.click(bounds["right"] - 45, \
                             (bounds["bottom"] - bounds["top"]) / 2 + bounds["top"])
            else:
                self.logger.debug("set %s value failure" % category)
        self.logger.debug("set %s to %s successfully" % (category, value))
        CameraCommon().clickScreenCenter()

    def setColorEffect(self, _text="None"):
        if not self.ArcSoftCameraWidget().settingCommonBtn().exists:
            self.ArcSoftCameraWidget().settingBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().settingCommonBtn(), "setting common button")
        self.ArcSoftCameraWidget().settingCommonBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().text("Color effect"), "color effect button")
        self.ArcSoftCameraWidget().text("Color effect").click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().text(_text), _text)
        self.ArcSoftCameraWidget().text(_text).click.wait()
        self.logger.debug("set color effect to %s successfully" % _text)
        CameraCommon().clickScreenCenter()

    def swipeScreen(self, orientation="right"):
        # Don't need to implement
        pass

#------------------------------------------------------------
# Interfaces' implementations are from here

    def cleanMediaFiles(self):
        """
        Used to clean the media files in some folders; eg. /mnt/sdcard/DCIM/Camera
        """
        self.logger.debug("clean media files start")
        g_common_obj.stop_app_am(self.PACKAGE_NAME_ARCSOFTCAMERA)
        g_common_obj.adb_cmd("pm clear %s" % self.PACKAGE_NAME_ARCSOFTCAMERA)
        self.logger.debug("clean media files successfully")

    def startCameraApp(self):
        """
        Used to start the camera application
        """
        CameraCommon().unlockScreen()
        self.logger.debug("launch arcsoft camera start")
        os.system("adb shell am start -S %s/%s" % (self.PACKAGE_NAME_ARCSOFTCAMERA, self.ACTIVITY_NAME_ARCSOFTCAMERA))
        time_left = 0
        start = time.time()
        success = False
        while time_left < CameraCommon().waitStartAppTime + 5:
            if self.isShutterBtnExists():
                success = True
                break
            CameraCommon().checkGuide()
            time_left = time.time() - start
            time.sleep(0.5)
            CameraCommon().checkCameraCrash()
        if (not success) or time_left > CameraCommon().waitStartAppTime:
            scname = g_common_obj.get_user_log_dir() + "/assert.png"
            g_common_obj.take_screenshot(scname)
            assert False, "arcsoft camera launch fail,launch time is greater than " \
                        + str(CameraCommon().waitStartAppTime + 5) + " seconds"
        self.logger.debug("launch arcsoft camera successfully")

    def stopCameraApp(self):
        """
        Used to stop the camera application
        """
        g_common_obj.stop_app_am(self.PACKAGE_NAME_ARCSOFTCAMERA)
        self.logger.debug("stop arcsoft camera app successfully")

    def selectMode(self, mode="Camera"):
        """
        Used to select a mode such as camera, video, panorama, lens blur, photo sphere and so on...
        """
        self.logger.debug("Change to %s mode" % mode)
        self.ArcSoftCameraWidget().modeBtn().click.wait()
        if mode == "Camera":
            self.ArcSoftCameraWidget().chooseMode("Auto").click.wait()
        elif mode == "Video":
            pass
        else:
            self.ArcSoftCameraWidget().chooseMode(mode).click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().shutterBtn(), "shutter button")

    def switchRearOrFront(self, lens="Back"):
        """
        Used to switch rear or front camera
        lens = Back / Front
        """
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().switchCameraBtn(), "switch camera button")
        if lens == "Front":
            if self.ArcSoftCameraWidget().flashBtn().enabled:
                self.ArcSoftCameraWidget().switchCameraBtn().click.wait()
                self.logger.debug("change to " + str(lens) + " camera")
            else:
                self.logger.debug("the current is " + str(lens) + " camera")
        if lens == "Back" or lens == "Rear":
            if not self.ArcSoftCameraWidget().flashBtn().enabled:
                self.ArcSoftCameraWidget().switchCameraBtn().click.wait()
                self.logger.debug("change to " + str(lens) + " camera")
            else:
                self.logger.debug("the current is " + str(lens) + " camera")
        time.sleep(2)

    def setExposure(self, value):
        self.logger.debug("enter setting")
        if not self.ArcSoftCameraWidget().settingCommonBtn().exists:
            self.ArcSoftCameraWidget().settingBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().settingCameraBtn(), "setting common button")
        self.ArcSoftCameraWidget().settingCameraBtn().click.wait()
        if self.ArcSoftCameraWidget().settingScrollView().scrollable:
            self.d(scrollable=True).scroll.vert.to(textContains="Exposure")
            self.ArcSoftCameraWidget().text("Exposure").click.wait()
        mx, my, sx, sy = CameraCommon().getBounds(self.ArcSoftCameraWidget().settingProgressSeekBar())
        ax = mx + sx / 2
        ay = my + sy / 2
        if value == "0":
            self.d.click(ax, ay)
        elif value == "-1":
            self.d.click(ax - 30, ay)
        elif value == "-2":
            self.d.click(ax - 60, ay)
        elif value == "+1":
            self.d.click(ax + 30, ay)
        elif value == "+2":
            self.d.click(ax + 60, ay)
        else:
            assert False, "The Exposure is not found!"
        self.logger.debug("set exposure to " + str(value))
        CameraCommon().clickScreenCenter()
        self.logger.debug("exit setting")

    def setFlash(self, flash="off"):
        """
        Used to control the flash; on, off, auto
        """
        if not self.ArcSoftCameraWidget().flashBtn().enabled:
            self.logger.debug("flash is not available")
            return 
        self.ArcSoftCameraWidget().flashBtn().click.wait()
        if flash == "off":
            self.ArcSoftCameraWidget().chooseFlashOff().click.wait()
        elif flash == "auto":
            self.ArcSoftCameraWidget().chooseFlashAuto().click.wait()
        elif flash == "on":
            self.ArcSoftCameraWidget().chooseFlashOn().click.wait()
        self.logger.debug("set flash to " + flash)

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
        self.logger.debug("enter setting")
        if not self.ArcSoftCameraWidget().settingCommonBtn().exists:
            self.ArcSoftCameraWidget().settingBtn().click.wait()
        self.ArcSoftCameraWidget().settingCameraBtn().click.wait()
        self.ArcSoftCameraWidget().text("Timer").click.wait()
        if timer == "off":
            self.ArcSoftCameraWidget().text("Off").click.wait()
        elif timer == "2s" or timer == "3s":
            self.ArcSoftCameraWidget().text("2 seconds").click.wait()
        elif timer == "10s":
            self.ArcSoftCameraWidget().text("10 seconds").click.wait()
        CameraCommon().clickScreenCenter()
        self.logger.debug("set timer to " + str(timer))
        self.logger.debug("exit setting")

    def getAllVideoResolutions(self, lens):
        """
        Return all of the video resolutions
        """
        self.switchRearOrFront(lens)
        if not self.ArcSoftCameraWidget().settingVideoBtn().exists:
            self.ArcSoftCameraWidget().settingBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().settingVideoBtn(), "setting video button")
        self.ArcSoftCameraWidget().settingVideoBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().text("Video quality"), "video quality text")
        self.ArcSoftCameraWidget().text("Video quality").click.wait()
        mCount = self.ArcSoftCameraWidget().videoQualityText().count
        mDict = {}
        for i in range(int(mCount)):
            mDict.setdefault(i + 1, self.ArcSoftCameraWidget().videoQualityText()[i].text)
        self.logger.debug("get all video resolution successfully")
        print mDict
        return mDict, mDict[1], mDict[mCount]

    def setVideoResolution(self, resolution, lens):
        """
        Used to control the video resolution, used with the getAllVideoResolutions
        """
        if not self.ArcSoftCameraWidget().settingVideoBtn().exists:
            self.ArcSoftCameraWidget().settingBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().settingVideoBtn(), "setting video button")
        self.ArcSoftCameraWidget().settingVideoBtn().click.wait()
        if not self.ArcSoftCameraWidget().videoQualityText().exists:
            self.ArcSoftCameraWidget().text("Video quality").click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().videoQualityText(), "video quality text")
        mCount = self.ArcSoftCameraWidget().videoQualityText().count
        if mCount > 0:
            if self.ArcSoftCameraWidget().text(resolution).exists:
                self.ArcSoftCameraWidget().text(resolution).click.wait()
                self.logger.debug("set video resolution to " + resolution)
            else:
                self.assertTrue(False, "camera not found " + resolution + " resolution")
        else:
            self.logger.debug("camera not found " + resolution + " resolution")

    def getAllCameraMode(self):
        pass

    def getAllPhotoResolutions(self, lens, type = ""):
        """
        Return all of the photo resolutions
        """
        self.switchRearOrFront(lens)
        if not self.ArcSoftCameraWidget().settingCommonBtn().exists:
            self.ArcSoftCameraWidget().settingBtn().click.wait()
        self.ArcSoftCameraWidget().settingCameraBtn().click.wait()
        self.ArcSoftCameraWidget().text("Resolution").click.wait()
        mList = []
        mx, my, sx, sy = CameraCommon().getBounds(self.ArcSoftCameraWidget().settingScrollView())
        for j in range(2):
            try:
                for i in range(5):
                    if self.ArcSoftCameraWidget().settingExpandText()[i].exists:
                        mList.append(self.ArcSoftCameraWidget().settingExpandText()[i].info["text"])
                self.d.swipe(sx + mx - 150, sy + my - 30, sx + mx - 150, my - 40, steps=50)
            except:
                break
#         print "===list len = " + str(len(mList))
        mDict = {}
        for i in range(len(mList)):
            mDict.setdefault(i + 1, mList[i])
        CameraCommon().clickScreenCenter()
        self.logger.debug("get all photo resolution successfully")
        print mDict
        return mDict, mDict[len(mList)], mDict[1]

    def setPhotoResolution(self, resolution, lens, type = ""):
        """
        Used to control the photo resolution, used with the getAllPhotoResolutions
        """
        if not self.ArcSoftCameraWidget().settingCommonBtn().exists:
            self.ArcSoftCameraWidget().settingBtn().click.wait()
        self.ArcSoftCameraWidget().settingCameraBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().text("Resolution"), "resolution button")
        self.ArcSoftCameraWidget().text("Resolution").click.wait()
        if self.ArcSoftCameraWidget().settingScrollView().scrollable:
            self.d(scrollable=True).scroll.vert.to(textContains=resolution)
            self.ArcSoftCameraWidget().text(resolution).click.wait()
            self.logger.debug("set resolution to " + resolution)
        else:
            CameraCommon().getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "This resolution is not found!"
        CameraCommon().clickScreenCenter()

    def capturePhoto(self, num=1, generated=True):
        """
        Used to capture num photos
        """
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().shutterBtn(), "shutter button")
        for i in range(int(num)):
            if self.ArcSoftCameraWidget().shutterBtn().exists:
                self.ArcSoftCameraWidget().shutterBtn().click.wait()
                self.logger.debug("capture photo %d" % (i + 1))
                if generated:
                    CameraCommon().waitForTheFilesAreGenerated()
                time.sleep(1.5)
            else:
                CameraCommon().checkCameraCrash()

    def capturePanorama(self):
        "Used to capture a panorama photo"
        # import subprocess
        # cmd = "ls -l /dev/ttyACM*"
        # output=subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE).communicate()
        # s=output[0][:-1]
        # s=" ".join(s.split())
        # deviceName = s.split(" ")[-1]
        # self.logger.debug("====deviceName="+deviceName)
        # "moving dut..."
        # import testaid.robot
        # self.logger.debug("====init====")
        # robot = testaid.robot.Robot(deviceName)
        # self.logger.debug("====reset====")
        # robot.reset()
        # self.logger.debug("====rotate -90 to prepare capturing====")
        # robot.rotate(90)
        self.logger.debug("====wait for panorama prompt message====")
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().panoramaPromptMsg(), "panorama prompt message")
        self.logger.debug("====click shutter button=====")
        self.ArcSoftCameraWidget().shutterBtn().click.wait()
        # self.logger.debug("====rotate====")
        # robot.rotate(-190, 3) #maybe in this app, do not need so many degrees

        # Replace rotate temporarily
        time.sleep(15);
        self.logger.debug("====wait for panorama prompt message to finish=====")
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().panoramaPromptMsg(), "panorama prompt message")
        # time.sleep(2)
        # "change to the initial angle"
        # robot.reset()
        self.ArcSoftCameraWidget().photoView().click.wait()
        time.sleep(2)

    def reviewPhoto(self, num=1):
        """
        Used to review num photos
        """
        pass

    def reviewPhotoAndVideo(self, num=1, timeout=10):
        """
        Used to review num photos
        """
        return QuickPicApp().reviewPhotoAndVideo(num, timeout)

    def recordVideo(self, videoNum=1, duration=5):
        """
        Used to capture num duration videos
        """
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().recordBtn(), "record button")
        for i in range(int(videoNum)):
            if self.ArcSoftCameraWidget().recordBtn().exists:
                if not self.isShutterBtnEnabled():
                    time.sleep(1)
                    if not self.isShutterBtnEnabled():
                        self.assertTrue(False, "record button is not enabled")
                self.ArcSoftCameraWidget().recordBtn().click.wait()
                self.logger.debug("recording video start")
                time.sleep(int(duration))
                self.ArcSoftCameraWidget().shutterBtn().click.wait()
                self.logger.debug("video duration is %ds" % int(duration))
                self.logger.debug("recording video end")
                CameraCommon().waitForTheFilesAreGenerated()
                time.sleep(2)
            else:
                CameraCommon().checkCameraCrash()

    def reviewVideo(self, num=1, duration=1):
        """
        Used to review num duration videos
        """
        pass

    def snapShotDuringVideo(self, videoNum=1, duration=5, snapShotNum=1):
        """
        Used to snapshot num pictures during a duration videos
        """
        CameraCommon().waitForWidgetToAppear(self.ArcSoftCameraWidget().recordBtn(), "record button")
        for i in range(int(videoNum)):
            if self.ArcSoftCameraWidget().recordBtn().exists:
                if not self.isShutterBtnEnabled():
                    time.sleep(1)
                    if not self.isShutterBtnEnabled():
                        self.assertTrue(False, "record button is not enabled")
                self.ArcSoftCameraWidget().recordBtn().click.wait()
                self.logger.debug("recording video start")
                time.sleep(2)
                for j in range(int(snapShotNum)):
                    self.ArcSoftCameraWidget().snapshotImageView().click.wait()
                    self.logger.debug("snapshot while video recording %d" % (j + 1))
                    if self.ArcSoftCameraWidget().snapshotImageView().clickable:
                        time.sleep(0.5)
                    else:
                        time.sleep(1)
                if int(duration) > (int(snapShotNum) * 2 + 2):
                    time.sleep(int(duration) - 2 - int(snapShotNum) * 2)
                else:
                    duration = int(snapShotNum) * 2 + 2
                self.ArcSoftCameraWidget().shutterBtn().click.wait()
                self.logger.debug("video duration is %ds" % int(duration))
                self.logger.debug("recording video end")
                time.sleep(2)
            else:
                CameraCommon().checkCameraCrash()
