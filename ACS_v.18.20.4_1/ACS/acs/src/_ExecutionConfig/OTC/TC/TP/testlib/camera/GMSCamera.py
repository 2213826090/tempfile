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


class GMSCamera(CameraAppInterface, CameraTestBase):
    PACKAGE_NAME_GMSCAMERA = "com.google.android.GoogleCamera"
    ACTIVITY_NAME_GMSCAMERA = 'com.android.camera.CameraLauncher'

    def __init__(self, cfg=None):
        if cfg == None:
            self.cfg = self.config.read(CameraCommon.DEFAULT_CONFIG_FILE, "multimedia_camera")
        else:
            self.cfg = cfg
        self.d = g_common_obj.get_device()
        self.logger = CameraLogger.instance()
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]

    class GMSCameraWidget:

        def __init__(self):
            self.d = g_common_obj.get_device()

        def camera(self):
            '''
            text is Camera
            '''
            return self.d(text="Camera")

        def countdownToggleBtn(self):
            '''
            count down toggle button,turn on/off countdown
            '''
            return self.d(resourceId="com.android.camera2:id/countdown_toggle_button")

        def exposureBtn(self):
            '''
            exposure_button, turn on/off exposure function
            '''
            return self.d(resourceId="com.android.camera2:id/exposure_button")

        def exposureOption(self):
            '''
            exposure list: -2 -1 0 +1 +2
            need turn on the exposure function first
            '''
            return self.d(resourceId="com.android.camera2:id/mode_options_exposure")

        def gridLinesToggleBtn(self):
            '''
            grid lines toggle button
            '''
            return self.d(resourceId="com.android.camera2:id/grid_lines_toggle_button")

        def lensBlur(self):
            '''
            text is Lens Blur
            '''
            return self.d(text="Lens Blur")

        def lockScreenIcon(self):
            '''
            camera lock screen icon
            '''
            return self.d(resourceId="com.android.systemui:id/lock_icon")

        def photoPreviewPageDeleteBtn(self):
            '''
            preview page delete button
            '''
            return self.d(resourceId="com.android.camera2:id/filmstrip_bottom_control_delete")

        def PhotoSphere(self):
            '''
            text is  Photo Sphere
            '''
            return self.d(text="Photo Sphere")

        def panorama(self):
            '''
            text is Panorama
            '''
            return self.d(text="Panorama")

        def photosApp(self):
            '''
            app photos
            '''
            return self.d(packageName="com.google.android.apps.photos")

        def picFolder(self):
            '''
             pic folder
            '''
            return self.d(packageName="com.alensw.PicFolder")

        def resolution(self,):
            '''
            resolution text
            '''
            return self.d(resourceId="android:id/text1")

        def resolutionText(self, _index):
            '''
            resolution text
            '''
            return self.d(resourceId="android:id/text1", index=_index)

        def recordingTime(self):
            '''
            record time
            '''
            return self.d(resourceId="com.android.camera2:id/recording_time")

        def selectorText(self):
            '''
            the selector text
            '''
            return self.d(resourceId="com.android.camera2:id/selector_text")

        def shutterBtn(self):
            '''
            the shutter button
            '''
            return self.d(resourceId="com.android.camera2:id/shutter_button")

        def settingsBtn(self):
            '''
            the setting button, to enter the camera setting page
            '''
            return self.d(resourceId="com.android.camera2:id/settings_button")

        def settingAdvancedSwitchWidget(self):
            '''
            the Manual exposure switch button in the Advanced page in the camera setting
            '''
            return self.d(resourceId="android:id/switchWidget")

        def settingAdvancedSwitchWidgetForN(self):
            '''
            the Manual exposure switch button in the Advanced page in the camera setting
            '''
            return self.d(resourceId="android:id/switch_widget")

        def text(self, _text):
            '''
            page text
            '''
            return self.d(textContains=_text)

        def threeDots(self):
            '''
            the three dots
            '''
            return self.d(resourceId="com.android.camera2:id/three_dots")

        def title(self):
            '''
            the title
            '''
            return self.d(resourceId="android:id/title", className="android.widget.TextView")

        def toggleBtn(self):
            '''
            camera toggle button,change front/back camera
            '''
            return self.d(resourceId="com.android.camera2:id/camera_toggle_button")

        def video(self):
            '''
            text is Video
            '''
            return self.d(text="Video")

        def videoView(self):
            '''
            video player
            '''
            return self.d(resourceId="com.google.android.apps.plus:id/videoplayer")

        def videoViewForM(self):
            '''
            camera video play view
            '''
            return self.d(resourceId="com.google.android.apps.photos:id/photo_hashtag_fragment_container")

        def videoPlayerBtn(self):
            '''
            video player button
            '''
            return self.d(resourceId="com.android.camera2:id/play_button")

        def doneWidget(self):
            '''
            done button
            '''
            return self.d(resourceId="com.android.camera2:id/done_button")

        def photoProcessingBar(self):
            '''
            panorama photo processing bar
            sphere photo processing bar
            '''
            return self.d(resourceId="com.android.camera2:id/bottom_session_progress_text")

        def overlay(self):
            '''
            over lay
            '''
            return self.d(resourceId="com.android.camera2:id/progress_overlay")

        def flashToggleBtn(self):
            '''
            flash button
            '''
            return self.d(resourceId="com.android.camera2:id/flash_toggle_button")

#==============================================================

    def clickShutterBtnArea(self, mode="Camera"):
        """
        @summary: click the area  of shutter button
        """
        mLeft, mTop, mRight, mBottom = CameraCommon().getBounds(self.GMSCameraWidget().shutterBtn())
        mx = mLeft + mRight / 2
        my = mTop + mBottom / 2
        self.logger.debug("width = " + str(mx) + ", height = " + str(my))
        self.d.click(mx, my)
        return mx, my

    def clickAllSetting(self):
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        self.swipeScreen("right")
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().settingsBtn(), "settings button")
        self.GMSCameraWidget().settingsBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().text("Resolution & quality"), "resolution text")
        self.GMSCameraWidget().text("Resolution & quality").click.wait()
        mTitleCount = self.GMSCameraWidget().title().count
        for i in range(mTitleCount):
            self.GMSCameraWidget().title()[i].click.wait()
            mCount = self.GMSCameraWidget().resolution().count
            for j in range(mCount):
                mText = self.GMSCameraWidget().resolution()[j].text
                self.GMSCameraWidget().resolution()[j].click.wait()
                CameraCommon().checkCameraCrash()
                self.logger.debug("set camera resolution to " + mText)
                if not (j == (mCount - 1)):
                    self.GMSCameraWidget().title()[i].click.wait()
        if not self.GMSCameraWidget().text("Resolution & quality").exists:
            CameraCommon().pressBack()
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().text("Resolution & quality"), "Resolution & quality")
        time.sleep(1)
        for i in range(2):
            self.GMSCameraWidget().text("Save location").right(resourceId="android:id/switchWidget").click.wait()
        self.logger.debug("set camera save location")
        self.GMSCameraWidget().text("Advanced").click.wait()
        if self.GMSCameraWidget().settingAdvancedSwitchWidget().exists:
            self.GMSCameraWidget().settingAdvancedSwitchWidget().click()
            self.logger.debug("set camera advance manual exposure")
        elif self.GMSCameraWidget().settingAdvancedSwitchWidgetForN().exists:
            self.GMSCameraWidget().settingAdvancedSwitchWidgetForN().click()
            self.logger.debug("set camera advance manual exposure")
        for i in range(4):
            if not self.GMSCameraWidget().shutterBtn().exists:
                CameraCommon().pressBack()

    def clickDoneBtn(self):
        if self.GMSCameraWidget().doneWidget().exists:
            self.GMSCameraWidget().doneWidget().click.wait()

    def clickRecordBtn(self):
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "record button")
        self.GMSCameraWidget().shutterBtn().click()
        time.sleep(0.5)

    def clickPanoramaDoneBtn(self):
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().doneWidget(), "panorama done button")
        self.GMSCameraWidget().doneWidget().click()
        time.sleep(0.5)

    def checkLens(self,lens="Back"):
        if lens == "Rear" or lens == "rear":
            lens = "Back"
        if self.GMSCameraWidget().threeDots().exists:
            self.GMSCameraWidget().threeDots().click.wait()
        elif self.d(description="Options").exists:
            self.d(description="Options").click.wait()
        time.sleep(1)
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().toggleBtn(), "switch camera lens button")
        description = self.GMSCameraWidget().toggleBtn().contentDescription
        time.sleep(1)
        if lens in description:
            self.logger.debug("check " + str(lens) + " camera successfully")
        else:
            self.assertTrue(False, "check" + str(lens) + " camera failure")
        CameraCommon().clickScreenCenter()
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        time.sleep(1)

    def checkModeExists(self,mode):
        self.swipeScreen()
        if self.GMSCameraWidget().text(mode).exists:
            CameraCommon().clickScreenCenter()
            return True
        else:
            CameraCommon().clickScreenCenter()
            return False

    def checkShutterButtonAttribute(self, attr="enabled"):
        mAttr = self.GMSCameraWidget().shutterBtn().info[attr]
        if mAttr:
            self.logger.debug("shutter button " + str(mAttr) + "is True")
            return True
        else:
            self.logger.debug("shutter button " + str(mAttr) + "is False")
            return False

    def disableOrEnableCamera(self, tag):
        return CameraCommon().disableOrEnableCamera(tag, self.PACKAGE_NAME_GMSCAMERA)

    def enterPreviewPhotos(self):
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        self.swipeScreen("left")

    def enterPreviewVideos(self):
        self.enterPreviewPhotos()
        CameraCommon().clickScreenCenter()

    def getAllPanoramaResolution(self):
        """
        Return all of the panorama resolutions
        """
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        self.swipeScreen("right")
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().settingsBtn(), "settings button")
        self.GMSCameraWidget().settingsBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().text("Resolution & quality"), "resolution text")
        self.GMSCameraWidget().text("Resolution & quality").click.wait()
        self.GMSCameraWidget().text("Panorama resolution").click.wait()
        mDict = {}
        mCount = self.GMSCameraWidget().resolution().count
        for i in range(mCount):
            if self.GMSCameraWidget().resolutionText(i).exists:
                mText = self.GMSCameraWidget().resolutionText(i).info["text"]
                mDict.setdefault(i + 1, mText)
            else:
                break
        for i in range(3):
            if not self.isShutterBtnExists():
                CameraCommon().pressBack()
            time.sleep(1)
        self.logger.debug("get all panorama resolution successfully")
        print mDict
        return mDict, mDict[1], mDict[mCount]

    def getFOVRegion(self):
        width = self.d.info["displayWidth"]
        height = self.d.info["displayHeight"]
        rotate = False
        if width < height:
            rotate = True 
        [x,y,w,h] = CameraCommon().getBounds(self.GMSCameraWidget().overlay())
        if rotate==True:
            [x,y,w,h] = CameraCommon().rotateRegion([x,y,w,h], width, height, 90)
        return rotate, [x,y,w,h]

    def isRecordTimeExists(self):
        return CameraCommon().isWidgetExists(self.GMSCameraWidget().recordingTime())

    def isShutterBtnExists(self):
        return CameraCommon().isWidgetExists(self.GMSCameraWidget().shutterBtn())

    def isShutterBtnEnabled(self):
        return CameraCommon().isWidgetEnabled(self.GMSCameraWidget().shutterBtn())

    def openGEO(self):
        pass

    def openLocation(self):
        pass

    def openManualExposure(self):
        self.logger.debug("enter camera setting")
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        self.swipeScreen("right")
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().settingsBtn(), "settings button")
        self.GMSCameraWidget().settingsBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().text("Advanced"), "Advanced text")
        time.sleep(1)
        self.GMSCameraWidget().text("Advanced").click.wait()
        if self.GMSCameraWidget().settingAdvancedSwitchWidget().exists:
            self.GMSCameraWidget().settingAdvancedSwitchWidget().click()
            self.logger.debug("set manual exposure successfully")
        elif self.GMSCameraWidget().settingAdvancedSwitchWidgetForN().exists:
            self.GMSCameraWidget().settingAdvancedSwitchWidgetForN().click()
            self.logger.debug("set manual exposure successfully")
        for i in range(3):
            if not self.isShutterBtnExists():
                CameraCommon().pressBack()
            else:
                break
        self.logger.debug("exit camera setting")
        for i in range(10):
            CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
            mTop = CameraCommon().getBounds(self.GMSCameraWidget().shutterBtn())[1]
            print mTop, self.y/2 - self.y/4, self.y/2 + self.y/4
            if mTop < (self.y/2 - self.y/4) or mTop > (self.y/2 + self.y/4):
                break
            else:
                time.sleep(0.5)

    def openCameraFromLockScreenIcon(self, camera):
        CameraCommon().openCameraFromLockScreenIcon(self.PACKAGE_NAME_GMSCAMERA, camera)

    def reviewAvailableModes(self):
        mList = self.getAllCameraMode()
        for i in range(len(mList)):
            self.swipeScreen()
            self.GMSCameraWidget().text(mList[i]).click.wait()
            self.logger.debug("click " + mList[i] + "mode ")
            CameraCommon().checkGuide()

    def swipeScreen(self, orientation="right"):
        """
        @summary: swipe screen
        """
        if not self.GMSCameraWidget().settingsBtn().exists:
            CameraCommon().swipeScreen(orientation)
        else:
            from testlib.camera.mum_camera_impl import CameraImpl
            self.logger.debug("===Layout is not found!===")
            CameraImpl().getScreenshotAndPullToHost("debug.png", g_common_obj.get_user_log_dir())
            #self.assertTrue(False, "Layout is not found!")
        time.sleep(2)
        if self.GMSCameraWidget().text("Photos").exists:
            self.GMSCameraWidget().text("Photos").click.wait()
        if self.GMSCameraWidget().text("Always").exists:
            self.GMSCameraWidget().text("Always").click.wait()
        time.sleep(1)

    def skipPopupButton(self, t_text, not_check_exist=0):
        self.logger.debug("skipPopupButton: start")
        if not_check_exist or self.d(textContains=t_text).exists:
            self.logger.debug("skipPopupButton: click popup")
            self.d(textContains=t_text).click.wait()

    def setPanoramaResolution(self, resolution):
        """
        Used to control the panorama resolution, used with the getAllPanoramaResolutions
        """
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        self.swipeScreen("right")
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().settingsBtn(), "settings button")
        self.GMSCameraWidget().settingsBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().text("Resolution & quality"), "resolution text")
        self.GMSCameraWidget().text("Resolution & quality").click.wait()
        self.GMSCameraWidget().text("Panorama resolution").click.wait()
        if self.GMSCameraWidget().text(resolution).exists:
            self.GMSCameraWidget().text(resolution).click.wait()
            self.logger.debug("set resolution to " + resolution)
        else:
            CameraCommon().getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "This resolution is not found!"
        for i in range(3):
            if not self.isShutterBtnExists():
                CameraCommon().pressBack()
            time.sleep(1)

#------------------------------------------------------------
# Interfaces' implementations are from here

    def cleanMediaFiles(self):
        """
        Used to clean the media files in some folders; eg. /mnt/sdcard/DCIM/Camera
        """
        g_common_obj.stop_app_am(self.PACKAGE_NAME_GMSCAMERA)
        g_common_obj.adb_cmd("pm clear %s" % self.PACKAGE_NAME_GMSCAMERA)
        self.logger.debug("clean media files successfully")

    def startCameraApp(self):
        """
        Used to start the camera application
        """
        CameraCommon().unlockScreen()
        self.logger.debug("launch gms camera start")
        os.system("adb shell am start -n %s/%s" % (self.PACKAGE_NAME_GMSCAMERA, self.ACTIVITY_NAME_GMSCAMERA))
        time_left = 0
        start = time.time()
        success = False
        while time_left < CameraCommon().waitStartAppTime:
            if self.isShutterBtnExists() or self.GMSCameraWidget().photoPreviewPageDeleteBtn().exists:
                success = True
                break
            CameraCommon().checkCameraAccessDevicesLocation()
            CameraCommon().checkGuide()
            time_left = time.time() - start
            time.sleep(0.5)
            CameraCommon().checkCameraCrash()
        if (not success) or time_left > CameraCommon().waitStartAppTime:
            scname = g_common_obj.get_user_log_dir() + "/assert.png"
            g_common_obj.take_screenshot(scname)
            assert False, "gms camera launch fail,launch time is greater than " \
                        + str(CameraCommon().waitStartAppTime) + " seconds"
        time.sleep(2)
        if self.GMSCameraWidget().text("NEXT").exists:
                CameraCommon().checkGuide()
        self.logger.debug("launch gms camera successfully")

    def stopCameraApp(self):
        """
        Used to stop the camera application
        """
        g_common_obj.stop_app_am(self.PACKAGE_NAME_GMSCAMERA)
#         g_common_obj.adb_cmd("pm clear %s" % self.PACKAGE_NAME_CAMERA)
        self.logger.debug("stop gms camera app successfully")

    def selectMode(self, mode="Camera"):
        """
        Used to select a mode such as camera, video, panorama, lens blur, photo sphere and so on...
        """
        self.swipeScreen()
        isPhotoSphersMode = False
        if mode == "Camera":
            self.GMSCameraWidget().camera().click.wait()
            self.logger.debug("change to camera mode")
        elif mode == "Video":
            self.GMSCameraWidget().video().click.wait()
            self.logger.debug("change to video mode")
        elif mode == "Lens Blur":
            if not self.GMSCameraWidget().lensBlur().exists:
                if not self.checkShutterButtonAttribute("enabled"):
                    self.logger.debug("maybe storage is full")
                    return
                else:
                    self.assertTrue(False, "camera not found Lens Blur mode")
            self.GMSCameraWidget().lensBlur().click.wait()
            self.logger.debug("change to lens blur mode")
            CameraCommon().checkGuide()
        elif mode == "Panorama":
            if not self.GMSCameraWidget().panorama().exists:
                if not self.checkShutterButtonAttribute("enabled"):
                    self.logger.debug("maybe storage is full")
                    return
                else:
                    self.assertTrue(False, "camera not found Panorama mode")
            self.GMSCameraWidget().panorama().click.wait()
            self.logger.debug("change to panorama mode")
            CameraCommon().checkGuide()
        elif mode == "Photo Sphere":
            if not self.GMSCameraWidget().PhotoSphere().exists:
                if not self.checkShutterButtonAttribute("enabled"):
                    self.logger.debug("maybe storage is full")
                    return
                else:
                    self.assertTrue(False, "camera not found Photo Sphere mode")
            isPhotoSphersMode = True
            self.GMSCameraWidget().PhotoSphere().click.wait()
            self.logger.debug("change to photo sphere mode")
            CameraCommon().checkGuide()
        else:
            self.logger.debug("camera has not change model")
        if not isPhotoSphersMode:
            CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
            time.sleep(1)

    def switchRearOrFront(self, lens="Back"):
        """
        Used to switch rear or front camera
        lens = Back / Front
        """
        if lens == "Rear" or lens == "rear":
            lens = "Back"
        if self.GMSCameraWidget().threeDots().exists:
            self.GMSCameraWidget().threeDots().click.wait()
        elif self.d(description="Options").exists:
            self.d(description="Options").click.wait()
        time.sleep(1)
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().toggleBtn(), "switch camera lens button")
        description = self.GMSCameraWidget().toggleBtn().contentDescription
        time.sleep(1)
        if (("Back" in description) and lens == "Front") or (("Front" in description) and lens == "Back"):
            self.GMSCameraWidget().toggleBtn().click.wait()
        self.logger.debug("change to " + str(lens) + " camera")
        CameraCommon().clickScreenCenter()
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        time.sleep(1)
        return description

    def setExposure(self, value):
        """
        type: String
        value: +2/+1/0/-1/-2
        """
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        if self.GMSCameraWidget().threeDots().exists:
            self.GMSCameraWidget().threeDots().click.wait()
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().exposureBtn(), "exposure button")
        self.GMSCameraWidget().exposureBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().exposureOption(), "exposure button")
        self.GMSCameraWidget().exposureOption().child_by_description("Exposure Compensation " + str(value), \
                            className="android.widget.ImageButton").click.wait()
        self.logger.debug("set exposure to " + str(value))
        CameraCommon().clickScreenCenter()

    def setFlash(self, flash="off"):
        """
        Used to control the flash; on, off, auto
        """
        if self.GMSCameraWidget().threeDots().exists:
            self.GMSCameraWidget().threeDots().click.wait()
            time.sleep(1)
        flash_str = "Flash " + flash
        if not self.GMSCameraWidget().flashToggleBtn().enabled:
            self.logger.debug("flash is not available")
            return
        for i in range(3):
            if not self.d(descriptionContains=flash_str):
                if self.GMSCameraWidget().flashToggleBtn().exists:
                    self.GMSCameraWidget().flashToggleBtn().click.wait()
            else:
                break
        CameraCommon().clickScreenCenter()
        self.logger.debug("set flash to " + flash_str)

    def setGrid(self, grid="off"):
        """
        Used to control the flash; on, off, auto
        """
        if self.GMSCameraWidget().threeDots().exists:
            self.GMSCameraWidget().threeDots().click.wait()
        elif self.d(description="Options").exists:
            self.d(description="Options").click.wait()
        time.sleep(1)
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().gridLinesToggleBtn(), " grid button")
        description = self.GMSCameraWidget().gridLinesToggleBtn().contentDescription
        time.sleep(1)
        if grid not in description:
            self.GMSCameraWidget().gridLinesToggleBtn().click.wait()

        self.logger.debug("grid lines is " + str(grid))
        CameraCommon().clickScreenCenter()
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        time.sleep(1)
        return description

    def setLocation(self, location="ON"):
        self.logger.debug("enter camera setting")
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        self.swipeScreen("right")
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().settingsBtn(), "settings button")
        self.GMSCameraWidget().settingsBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().text("Save location"), "Save location")
        time.sleep(1)
        if self.GMSCameraWidget().settingAdvancedSwitchWidget().exists:
            if self.GMSCameraWidget().settingAdvancedSwitchWidget().text != location:
                self.GMSCameraWidget().settingAdvancedSwitchWidget().click.wait()
        elif self.GMSCameraWidget().settingAdvancedSwitchWidgetForN().exists:
            if self.GMSCameraWidget().settingAdvancedSwitchWidgetForN().text != location:
                self.GMSCameraWidget().settingAdvancedSwitchWidgetForN().click.wait()
        for i in range(3):
            if not self.isShutterBtnExists():
                CameraCommon().pressBack()
            else:
                break
        time.sleep(1)
        self.logger.debug("exit camera setting")

    def setTimer(self, timer="off"):
        """
        Used to control the timer
        value: off/2s/3s/10s
        """
        if self.GMSCameraWidget().threeDots().exists:
            self.GMSCameraWidget().threeDots().click.wait()
            time.sleep(1)
        if self.d(descriptionContains="timer is off") and \
            self.GMSCameraWidget().countdownToggleBtn().exists:
            if timer == "3s":
                self.GMSCameraWidget().countdownToggleBtn().click.wait()
            elif timer == "10s":
                self.GMSCameraWidget().countdownToggleBtn().click.wait()
                self.GMSCameraWidget().countdownToggleBtn().click.wait()
        elif self.d(descriptionContains="3 seconds") and \
                self.GMSCameraWidget().countdownToggleBtn().exists:
            if timer == "10s":
                self.GMSCameraWidget().countdownToggleBtn().click.wait()
            elif timer == "off":
                self.GMSCameraWidget().countdownToggleBtn().click.wait()
                self.GMSCameraWidget().countdownToggleBtn().click.wait()
        elif self.d(descriptionContains="10 seconds") and \
                self.GMSCameraWidget().countdownToggleBtn().exists:
            if timer == "3s":
                self.GMSCameraWidget().countdownToggleBtn().click.wait()
                self.GMSCameraWidget().countdownToggleBtn().click.wait()
            elif timer == "off":
                self.GMSCameraWidget().countdownToggleBtn().click.wait()
        CameraCommon().clickScreenCenter()
        self.logger.debug("set timer to " + str(timer))

    def getAllVideoResolutions(self, lens):
        """
        Return all of the video resolutions
        """
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        self.swipeScreen("right")
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().settingsBtn(), "settings button")
        self.GMSCameraWidget().settingsBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().text("Resolution & quality"), "resolution text")
        self.GMSCameraWidget().text("Resolution & quality").click.wait()
        self.GMSCameraWidget().text(str(lens) + " camera video").click.wait()
        mDict = {}
        mCount = self.GMSCameraWidget().resolution().count
        for i in range(mCount):
            if self.GMSCameraWidget().resolutionText(i).exists:
                mText = self.GMSCameraWidget().resolutionText(i).info["text"]
                mDict.setdefault(i + 1, mText)
            else:
                break
        for i in range(3):
            if not self.isShutterBtnExists():
                CameraCommon().pressBack()
            time.sleep(1)
        self.logger.debug("get all video resolution successfully")
        print mDict
        return mDict, mDict[1], mDict[mCount]

    def setVideoResolution(self, resolution, lens):
        """
        Used to control the video resolution, used with the getAllVideoResolutions
        """
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        self.swipeScreen("right")
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().settingsBtn(), "settings button")
        self.GMSCameraWidget().settingsBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().text("Resolution & quality"), "resolution text")
        self.GMSCameraWidget().text("Resolution & quality").click.wait()
        self.GMSCameraWidget().text(str(lens) + " camera video").click.wait()
        if self.GMSCameraWidget().text(resolution).exists:
            self.GMSCameraWidget().text(resolution).click.wait()
            self.logger.debug("set resolution to " + resolution)
        else:
            CameraCommon().getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "This resolution is not found!"
        for i in range(3):
            if not self.isShutterBtnExists():
                CameraCommon().pressBack()
            time.sleep(1)

    def getAllCameraMode(self):
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        self.swipeScreen()
        mList = []
        mCount = self.GMSCameraWidget().selectorText().count
        for i in range(mCount):
            mList.append(self.GMSCameraWidget().selectorText()[i].text)
#             print self.GMSCameraWidget().selectorText()[i].text
        CameraCommon().clickScreenCenter()
        return mList

    def getAllPhotoResolutions(self, lens, type = ""):
        """
        Return all of the photo resolutions
        """
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        self.swipeScreen("right")
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().settingsBtn(), "settings button")
        self.GMSCameraWidget().settingsBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().text("Resolution & quality"), "resolution text")
        self.GMSCameraWidget().text("Resolution & quality").click.wait()
        self.GMSCameraWidget().text(str(lens) + " camera photo").click.wait()
        mDict = {}
        mCount = self.GMSCameraWidget().resolution().count
        for i in range(int(mCount)):
            if self.GMSCameraWidget().resolutionText(i).exists:
                mText = self.GMSCameraWidget().resolutionText(i).info["text"]
                if mText:
                    mTmp1 = mText.split(')')
                    mTmp2 = mTmp1[1].split('m')
                    mDict.setdefault(float(mTmp2[0].strip()), mText)
            else:
                break
        first = True
        for key in mDict:
            if first:
                mMax = key
                mMin = key
                first = False
            if mMax < key:
                mMax = key
            if mMin > key:
                mMin = key
        for i in range(3):
            if not self.isShutterBtnExists():
                CameraCommon().pressBack()
            time.sleep(1)
            first = True
        self.logger.debug("get all photo resolution successfully")
        print mDict
        return mDict, mDict[mMax], mDict[mMin]

    def setPhotoResolution(self, resolution, lens, type = ""):
        """
        Used to control the photo resolution, used with the getAllPhotoResolutions
        """
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        self.swipeScreen("right")
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().settingsBtn(), "settings button")
        self.GMSCameraWidget().settingsBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().text("Resolution & quality"), "resolution text")
        self.GMSCameraWidget().text("Resolution & quality").click.wait()
        self.GMSCameraWidget().text(str(lens) + " camera photo").click.wait()
        if self.GMSCameraWidget().text(resolution).exists:
            self.GMSCameraWidget().text(resolution).click.wait()
            self.logger.debug("set resolution to " + resolution)
        else:
            CameraCommon().getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "This resolution is not found!"
        for i in range(3):
            if not self.isShutterBtnExists():
                CameraCommon().pressBack()
            time.sleep(1)

    def capturePhoto(self, num=1, generated=True, wait_time=10, flag=0):
        """
        Used to capture num photos
        """
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button", wait_time, flag)
        for i in range(int(num)):
            if self.GMSCameraWidget().shutterBtn().exists:
                if not self.GMSCameraWidget().shutterBtn().enabled:
                    time.sleep(1)
                self.GMSCameraWidget().shutterBtn().click()
                self.logger.debug("capture photo %d" % (i + 1))
                if generated:
                    ret = CameraCommon().waitForTheFilesAreGenerated()
                    self.logger.debug("==waitForTheFilesAreGenerated ret=" + str(ret))
                time.sleep(1)
            else:
                self.checkCameraCrash()

    def capturePanorama(self):
        "Used to capture a panorama photo"
        #import subprocess
        #cmd = "ls -l /dev/ttyACM*"
        #output=subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE).communicate()
        #s=output[0][:-1]
        #s=" ".join(s.split())
        #deviceName = s.split(" ")[-1]
        #self.logger.debug("====deviceName="+deviceName)
        #"moving dut..."
        #import testaid.robot
        #self.logger.debug("====init====")
        #robot = testaid.robot.Robot(deviceName)
        #self.logger.debug("====reset====")
        #robot.reset()
        #self.logger.debug("====rotate -90 to prepare capturing====")
        #robot.rotate(90)
        self.logger.debug("====click shutter button=====")
        self.GMSCameraWidget().shutterBtn().click.wait()
        #self.logger.debug("====wait done button=====")
        #CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget.doneWidget(), "done button")
        #self.logger.debug("====rotate====")
        #robot.rotate(-190, 3)
        
        #Replace rotate temporarily
        time.sleep(15);
        
        if self.GMSCameraWidget().doneWidget().exists:
            self.logger.debug("====click done button=====")
            self.GMSCameraWidget().doneWidget().click.wait()
            CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        #time.sleep(2)
        #"change to the initial angle"
        #robot.reset()
        CameraCommon().swipeScreen("left")
        self.logger.debug("====wait photo process bar to disappear=====")
        CameraCommon().waitForWidgetToDisappear(self.GMSCameraWidget().photoProcessingBar(), "photo process bar")

    def reviewPhoto(self, num=1):
        """
        Used to review num photos
        """
        pass

    def reviewPhotoAndVideo(self, num=1, timeout=10):
        """
        Used to review num photos
        """
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        self.logger.debug("preview photos start")
        mNum = 0
        while mNum < num:
            self.swipeScreen("left")
            if self.GMSCameraWidget().videoPlayerBtn().exists:
                CameraCommon().clickScreenCenter()
                time_left = 0
                start = time.time()
                while time_left < 3:
                    if self.GMSCameraWidget().text("Photos").exists:
                        self.GMSCameraWidget().text("Photos").click.wait()
                        if self.GMSCameraWidget().text("Always").exists:
                            self.GMSCameraWidget().text("Always").click.wait()
                        elif self.GMSCameraWidget().text("ALWAYS").exists:
                            self.GMSCameraWidget().text("ALWAYS").click.wait()
                        break
                    time_left = time.time() - start
                    time.sleep(0.5)
                if self.GMSCameraWidget().photosApp().exists:
                    self.d.swipe(self.x / 2, 10, self.x / 2, self.y / 2 + 100, steps=50)
                if self.GMSCameraWidget().text("Always").exists:
                    self.GMSCameraWidget().text("Always").click.wait()
                elif self.GMSCameraWidget().text("ALWAYS").exists:
                    self.GMSCameraWidget().text("ALWAYS").click.wait()
                time.sleep(2)
                if self.GMSCameraWidget().videoView().exists or self.GMSCameraWidget().videoViewForM().exists or \
                    self.GMSCameraWidget().picFolder().exists:
                    CameraCommon().pressBack()
            mNum += 1
        for i in range(2):
            if not self.GMSCameraWidget().shutterBtn().exists:
                CameraCommon().pressBack()
        self.logger.debug("preview photos end")

    def recordVideo(self, videoNum=1, duration=5):
        """
        Used to capture num duration videos
        """
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "record button")
        for i in range(int(videoNum)):
            if self.GMSCameraWidget().shutterBtn().exists:
                if not self.isShutterBtnEnabled():
                    time.sleep(1)
                    if not self.isShutterBtnEnabled():
                        self.assertTrue(False, "record button is not enabled")
                self.GMSCameraWidget().shutterBtn().click()
                self.logger.debug("recording video start")
                time.sleep(int(duration))
                self.GMSCameraWidget().shutterBtn().click()
                self.logger.debug("video duration is %ds" % int(duration))
                self.logger.debug("recording video end")
                CameraCommon().waitForTheFilesAreGenerated()
                time.sleep(1.5)
            else:
                self.checkCameraCrash()

    def reviewVideo(self, num=1, duration=1):
        """
        Used to review num duration videos
        """
        pass

    def snapShotDuringVideo(self, videoNum=1, duration=5, snapShotNum=1):
        """
        Used to snapshot num pictures during a duration videos
        """
        CameraCommon().waitForWidgetToAppear(self.GMSCameraWidget().shutterBtn(), "shutter button")
        for i in range(int(videoNum)):
            if self.GMSCameraWidget().shutterBtn().exists:
                self.GMSCameraWidget().shutterBtn().click.wait()
                self.logger.debug("video recording start")
                time.sleep(2)
                for j in range(int(snapShotNum)):
                    CameraCommon().clickScreenCenter()
                    self.logger.debug("snapshot while video recording %d" % (j + 1))
                    if self.GMSCameraWidget().shutterBtn().clickable:
                        time.sleep(0.5)
                    else:
                        time.sleep(1)
                if int(duration) > (int(snapShotNum) * 2 + 2):
                    time.sleep(int(duration) - 2 - int(snapShotNum) * 2)
                else:
                    duration = int(snapShotNum) * 2 + 2
                self.GMSCameraWidget().shutterBtn().click.wait()
                self.logger.debug("video duration is %ds" % int(duration))
                self.logger.debug("video recording end")
                CameraCommon().waitForTheFilesAreGenerated()
                time.sleep(1.5)
            else:
                self.checkCameraCrash()
