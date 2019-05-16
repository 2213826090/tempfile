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
# from reportlab.graphics.shapes import rotate


class RefCam2Camera(CameraAppInterface, CameraTestBase):
    PACKAGE_NAME_REFCAM2CAMERA = "com.intel.refcam2"
    ACTIVITY_NAME_REFCAM2CAMERA = ".CameraActivity"

#     PACKAGE_NAME_QUICKPIC = "com.alensw.PicFolder"
#     ACTIVITY_NAME_QUICKPIC = ".GalleryActivity"

    def __init__(self, cfg=None):
        if cfg == None:
            self.cfg = self.config.read(CameraCommon.DEFAULT_CONFIG_FILE, "multimedia_camera")
        else:
            self.cfg = cfg
        self.d = g_common_obj.get_device()
        self.logger = CameraLogger.instance()
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]

    class RefCam2Widget:

        def __init__(self):
            self.d = g_common_obj.get_device()

        def frontback(self):
            '''
            front back button
            '''
            return self.d(resourceId="com.intel.refcam2:id/switch_frontback")

        def itemChildMenu(self):
            '''
            item clild menu
            '''
            return self.d(resourceId="com.intel.refcam2:id/setting_item_child_menu_text")

        def seekBar(self):
            '''
            menu seek bar
            '''
            return self.d(resourceId="com.intel.refcam2:id/setting_item_child_menu_seekbar")

        def shutterBtn(self):
            '''
            shutter button
            '''
            return self.d(resourceId="com.intel.refcam2:id/shutter")

        def shotlist(self):
            '''
            front back button
            '''
            return self.d(resourceId="com.intel.refcam2:id/open_shot_list")

        def settingsBtn(self):
            '''
            settings
            '''
            return self.d(resourceId="com.intel.refcam2:id/setting")

        def text(self, _text):
            '''
            page text
            '''
            return self.d(textContains=_text)

        def itemChildMenuText(self, _text):
            '''
            page text
            '''
            return self.d(resourceId="com.intel.refcam2:id/setting_item_child_menu_text", textContains=_text)

        def zoomLayer(self):
            '''
            zoom layer
            '''
            return self.d(resourceId="com.intel.refcam2:id/zoom_layer")

#=============================================================

    def checkColorEffect(self, _text="OFF"):
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().settingsBtn(), "setting button")
        self.RefCam2Widget().settingsBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().text("Effect Mode"), "color effect button")
        if _text == "Negative":
            _text = "NEGATIVE"
        elif _text == "None":
            _text = "OFF"
        elif _text == "Mono":
            _text = "MONO"
        elif _text == "Sepia":
            _text = "SEPIA"
        if self.RefCam2Widget().text("Effect Mode").exists and self.RefCam2Widget().text(_text).exists:
            if not self.RefCam2Widget().text("Effect Mode").right(text = _text).exists:
                self.assertTrue(False, "check color effect to %s failure" %_text)
        self.logger.debug("check color effect to %s successfully" % _text)
        CameraCommon().clickBtn(self.x - 50, 350)

    def clickShutterBtnArea(self, mode="Camera"):
        """
        @summary: click the area  of shutter button
        """
        mLeft, mTop, mRight, mBottom = CameraCommon().getBounds(self.RefCam2Widget().shutterBtn())
        mx = mLeft + mRight / 2
        my = mTop + mBottom / 2
        self.logger.debug("width = " + str(mx) + ", height = " + str(my))
        self.d.click(mx, my)
        return mx, my

    def clickShutterBtnWithoutRelease(self, mode="Camera", duration=50):
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().shutterBtn(), "shutter button")
        self.RefCam2Widget().shotlist().click.wait()
        self.RefCam2Widget().text("Continuous").click.wait()
        CameraCommon().clickBtn(self.x - 50, 350)
        mLeft, mTop, mRight, mBottom = CameraCommon().getBounds(self.RefCam2Widget().shutterBtn())
        self.d.swipe(mLeft, mTop, mRight + mLeft, mBottom + mTop, steps=duration * 18)

    def faceDetectionOnAndOff(self):
        self.logger.debug("enter setting")
        self.RefCam2Widget().settingsBtn().click.wait()
        self.d(scrollable=True).scroll.vert.to(textContains="Face Detect Mode")
        self.RefCam2Widget().text("Face Detect Mode").click.wait()
        self.RefCam2Widget().text("SIMPLE").click.wait()
        self.RefCam2Widget().settingsBtn().click.wait()
        self.logger.debug("switch face detection successfully")
        self.logger.debug("exit setting")

    def getFOVRegion(self):
        width = self.d.info["displayWidth"]
        height = self.d.info["displayHeight"]
        rotate = False
        if width < height:
            rotate = True 
        [x,y,w,h] = CameraCommon().getBounds(self.RefCam2Widget().zoomLayer())
        if rotate==True:
            [x,y,w,h] = CameraCommon().rotateRegion([x,y,w,h], width, height, 90)
        return rotate, [x,y,w,h]
        #return zoom_l, zoom_t, zoom_r, zoom_b - zoom_l

    def isShutterBtnExists(self):
        return CameraCommon().isWidgetExists(self.RefCam2Widget().shutterBtn())

    def setCaptureMode(self, value):
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().shotlist(), "mode button")
        self.RefCam2Widget().shotlist().click.wait()
        self.RefCam2Widget().text(value).click.wait()
        self.logger.debug("set capture mode to %s successfully" % value)
        CameraCommon().clickScreenCenter()

    def setColorEffect(self, _text="OFF"):
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().settingsBtn(), "setting button")
        self.RefCam2Widget().settingsBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().text("Effect Mode"), "color effect button")
        self.RefCam2Widget().text("Effect Mode").click.wait()
        if _text == "Negative":
            _text = "NEGATIVE"
        elif _text == "None":
            _text = "OFF"
        elif _text == "Mono":
            _text = "MONO"
        elif _text == "Sepia":
            _text = "SEPIA"
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().text(_text), _text)
        self.RefCam2Widget().text(_text).click.wait()
        self.logger.debug("set color effect to %s successfully" % _text)
        CameraCommon().clickBtn(self.x - 50, 350)

    def setExposureBracket(self):
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().settingsBtn(), "setting button")
        self.RefCam2Widget().settingsBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().text("Bracketing"), "Bracketing button")
        self.RefCam2Widget().text("Bracketing").click.wait()
        self.d(text="EXPOSURE").click.wait()
        count = self.RefCam2Widget().seekBar().count
        x1,y1,w1,h1 = CameraCommon().getBounds(self.RefCam2Widget().seekBar()[0])
        y2 = CameraCommon().getBounds(self.RefCam2Widget().seekBar()[1])[1]
        y3 = CameraCommon().getBounds(self.RefCam2Widget().seekBar()[2])[1]
        CameraCommon().clickBtn(x1, y1 + h1/2)
        CameraCommon().clickBtn(x1 + w1/2, y2 + h1/2)
        CameraCommon().clickBtn(x1 + w1 - 10, y3 + h1/2)
        if not self.RefCam2Widget().text("-5").exists:
            self.assertTrue(False, "bracketing exposure -5 not found!")
        if not self.RefCam2Widget().text("0").exists:
            self.assertTrue(False, "bracketing exposure 0 not found!")
        if not self.RefCam2Widget().text("5").exists:
            self.assertTrue(False, "bracketing exposure 5 not found!")
        self.RefCam2Widget().text("OK").click.wait()
        CameraCommon().clickBtn(self.x-50, 350)
        CameraCommon().clickScreenCenter()

#------------------------------------------------------------
# Interfaces' implementations are from here

    def cleanMediaFiles(self):
        """
        Used to clean the media files in some folders; eg. /mnt/sdcard/DCIM/Camera
        """
        self.logger.debug("clean media files start")
        g_common_obj.stop_app_am(self.PACKAGE_NAME_REFCAM2CAMERA)
        g_common_obj.adb_cmd("pm clear %s" % self.PACKAGE_NAME_REFCAM2CAMERA)
        self.logger.debug("clean media files successfully")

    def startCameraApp(self):
        """
        Used to start the camera application
        """
        CameraCommon().unlockScreen()
        self.logger.debug("launch refcam2 camera start")
        os.system("adb shell am start -S %s/%s" % (self.PACKAGE_NAME_REFCAM2CAMERA, self.ACTIVITY_NAME_REFCAM2CAMERA))
        time_left = 0
        start = time.time()
        success = False
        while time_left < CameraCommon().waitStartAppTime + 15:
            if self.isShutterBtnExists() or self.RefCam2Widget().shotlist().exists:
                success = True
                break
            if self.RefCam2Widget().text("Allow").exists or self.RefCam2Widget().text("ALLOW").exists:
                CameraCommon().checkGuide()
                if CameraCommon().getAndroidVersion() == "N":
                    self.stopCameraApp()
                    os.system("adb shell am start -S %s/%s" % (self.PACKAGE_NAME_REFCAM2CAMERA, self.ACTIVITY_NAME_REFCAM2CAMERA))
            time_left = time.time() - start
            time.sleep(0.5)
            CameraCommon().checkCameraCrash()
        if (not success) or time_left > CameraCommon().waitStartAppTime + 15:
            scname = g_common_obj.get_user_log_dir() + "/assert.png"
            g_common_obj.take_screenshot(scname)
            assert False, "refcam2 camera launch fail,launch time is greater than " \
                        + str(CameraCommon().waitStartAppTime + 15) + " seconds"
        self.logger.debug("launch refcam2 camera successfully")

    def stopCameraApp(self):
        """
        Used to stop the camera application
        """
        g_common_obj.stop_app_am(self.PACKAGE_NAME_REFCAM2CAMERA)
        self.logger.debug("stop arcsoft camera app successfully")

    def selectMode(self, mode="Camera"):
        """
        Used to select a mode such as camera, video, panorama, lens blur, photo sphere and so on...
        """
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().shutterBtn(), "shutter button")
        self.RefCam2Widget().shotlist().click.wait()
        self.RefCam2Widget().text(mode).click.wait()
        CameraCommon().clickScreenCenter()
        self.logger.debug("Change to %s mode" % mode)

    def switchRearOrFront(self, lens="Back"):
        """
        Used to switch rear or front camera
        lens = Back / Front
        """
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().shutterBtn(), "shutter button")
        if lens == "Front":
            self.RefCam2Widget().frontback().click.wait()
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
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().settingsBtn(), "setting button")
        self.RefCam2Widget().settingsBtn().click.wait()
        if not self.RefCam2Widget().text(type).exists:
            self.d(scrollable=True).scroll.vert.to(textContains=type)
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().text(type), "capture size button")
        self.RefCam2Widget().text(type).click.wait()
        mCount = self.RefCam2Widget().itemChildMenu().count
        mList=[]
        for i in range(mCount):
            mList.append(self.RefCam2Widget().itemChildMenu()[i].info["text"])
        CameraCommon().clickBtn(self.x-100, 300)
        self.logger.debug("mList=%s" % mList)
        if mList == []:
            self.logger.debug("can't find  photo resolutions with \"%s\" type!" % type)
            return [], -1, -1
        else:
            return mList,mList[0],mList[mCount-1]

    def setSettingsButton(self, main_click_button, sub_click_button):
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().settingsBtn(), "setting button")
        self.RefCam2Widget().settingsBtn().click.wait()
        if not self.RefCam2Widget().text(main_click_button).exists:
            self.d(scrollable=True).scroll.vert.to(textContains=main_click_button)
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().text(main_click_button), "click main button: %s" % main_click_button )
        self.RefCam2Widget().text(main_click_button).click.wait()
        self.RefCam2Widget().itemChildMenuText(sub_click_button).click.wait()
        self.logger.debug("click sub button: %s" % sub_click_button)
        CameraCommon().clickBtn(self.x-100, 300)

    def setPhotoResolution(self, resolution, lens, type="Capture Size (JPEG)"):
        """
        Used to control the photo resolution, used with the getAllPhotoResolutions
        """
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().settingsBtn(), "setting button")
        self.RefCam2Widget().settingsBtn().click.wait()
        if not self.RefCam2Widget().text(type).exists:
            self.d(scrollable=True).scroll.vert.to(textContains=type)
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().text(type), "capture size button")
        self.RefCam2Widget().text(type).click.wait()
        self.RefCam2Widget().itemChildMenuText(resolution).click.wait()
        self.logger.debug("set resolution to " + resolution)
        CameraCommon().clickBtn(self.x-100, 300)

    def capturePhoto(self, num=1, generated=True):
        """
        Used to capture num photos
        """
        CameraCommon().waitForWidgetToAppear(self.RefCam2Widget().shutterBtn(), "shutter button")
        for i in range(int(num)):
            if self.RefCam2Widget().shutterBtn().exists:
                self.RefCam2Widget().shutterBtn().click.wait()
                self.logger.debug("capture photo %d" % (i + 1))
                if generated:
                    CameraCommon().waitForTheFilesAreGenerated()
                time.sleep(1.5)
            else:
                CameraCommon().checkCameraCrash()

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
