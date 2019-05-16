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


class QuickPicApp(CameraTestBase):
    PACKAGE_NAME_QUICKPIC = "com.alensw.PicFolder"
    ACTIVITY_NAME_QUICKPIC = ".GalleryActivity"

    def __init__(self, cfg=None):
        self.cfg = cfg
        self.d = g_common_obj.get_device()
        self.logger = CameraLogger.instance()
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]

    class QuickPicWidget:
        def __init__(self):
            self.d = g_common_obj.get_device()

        def actionBar(self):
            '''
            action bar
            '''
            return self.d(resourceId="android:id/action_bar")

        def actionBarTitle(self):
            '''
            action bar title
            '''
            return self.d(resourceId="android:id/action_bar_title")

        def drawerList(self):
            '''
            drawer list
            '''
            return self.d(resourceId="com.alensw.PicFolder:id/drawer_list")

        def editText(self):
            '''
            edit text
            '''
            return self.d(className="android.widget.EditText", index=0)

        def imageView(self):
            '''
            image view
            '''
            return self.d(resourceId="com.alensw.PicFolder:id/image")

        def moreOption(self):
            '''
            more option
            '''
            return self.d(description="More options")

        def searchBtn(self):
            '''
            search button
            '''
            return self.d(resourceId="com.alensw.PicFolder:id/search")

        def slideShow(self):
            '''
            slide show
            '''
            return self.d(resourceId="com.alensw.PicFolder:id/slideshow")

        def selectBtn(self):
            '''
            select button
            '''
            return self.d(resourceId="com.alensw.PicFolder:id/select")

        def text(self, _text):
            '''
            page text
            '''
            return self.d(text=_text)

        def upBtn(self):
            '''
            up button
            '''
            return self.d(resourceId="android:id/up")

        def videoPlayBtn(self):
            '''
            video play button
            '''
            return self.d(resourceId="com.alensw.PicFolder:id/play")

        def videoView(self):
            '''
            video view
            '''
            return self.d(className="android.widget.VideoView", index=0)
#=============================================================

    def enterPreviewPhotos(self):
        self.startQuickPicApp()
        CameraCommon().waitForWidgetToAppear(self.QuickPicWidget().searchBtn(), "search button")
        if self.QuickPicWidget().drawerList().exists:
            self.QuickPicWidget().upBtn().click.wait()
            CameraCommon().waitForWidgetToAppear(self.QuickPicWidget().text("Folders"), "folder bar")
        if self.QuickPicWidget().upBtn().exists:
            self.QuickPicWidget().upBtn().click.wait()
            CameraCommon().waitForWidgetToAppear(self.QuickPicWidget().text("Folders"), "folder menu")
            self.QuickPicWidget().text("Folders").click.wait()
        self.QuickPicWidget().searchBtn().click.wait()
        CameraCommon().waitForWidgetToAppear(self.QuickPicWidget().editText(), "edit text")
        self.QuickPicWidget().editText().set_text("Camera")
        if self.QuickPicWidget().text("OK").exists:
            self.QuickPicWidget().text("OK").click.wait()
        CameraCommon().waitForWidgetToAppear(self.QuickPicWidget().slideShow(), "slide show")
        bounds = self.QuickPicWidget().actionBar().bounds
        self.d.click(160, int(bounds["bottom"]) + 100)
        CameraCommon().waitForWidgetToAppear(self.QuickPicWidget().selectBtn(), "select button")
        mText = self.QuickPicWidget().actionBarTitle().text
        mCount = int(mText[8:-1])
        print mCount
        self.d.click(160, int(bounds["bottom"]) + 100)
        time.sleep(2)
        if self.QuickPicWidget().text("QuickPic").exists:
            self.QuickPicWidget().text("QuickPic").click.wait()
            self.QuickPicWidget().text("Always").click.wait()
        return mCount

    def enterPreviewVideos(self):
        self.enterPreviewPhotos()
        self.d.click(self.x/2, self.y/2)

    def startQuickPicApp(self):
        """
        Used to start the camera application
        """
        CameraCommon().unlockScreen()
        self.logger.debug("launch quickpic app start")
        os.system("adb shell am start -n %s/%s" % (self.PACKAGE_NAME_QUICKPIC, self.ACTIVITY_NAME_QUICKPIC))
        CameraCommon().waitForWidgetToAppear(self.QuickPicWidget().searchBtn(), "search button")
        self.logger.debug("launch quickpic app successfully")
        time.sleep(1)

    def stopQuickPicApp(self):
        """
        Used to stop the camera application
        """
        g_common_obj.stop_app_am(self.PACKAGE_NAME_QUICKPIC)
        self.logger.debug("stop quickpic app successfully")

    def reviewPhotoAndVideo(self, num=1, timeout=10):
        """
        Used to review num photos
        """
        self.logger.debug("preview photos start")
        mCount = self.enterPreviewPhotos()
        for i in range(mCount):
            if self.QuickPicWidget().videoView().exists:
                time.sleep(3)
                CameraCommon().pressBack()
                break
            if self.QuickPicWidget().imageView().exists:
                time.sleep(2)
            if self.QuickPicWidget().videoPlayBtn().exists:
                self.QuickPicWidget().videoPlayBtn().click.wait()
                time.sleep(3)
                if not self.QuickPicWidget().videoPlayBtn().exists:
                    CameraCommon().pressBack()
            self.d.swipe(self.x - 100, self.y / 2, 100, self.y / 2, 50)
            time.sleep(1)
        self.logger.debug("preview photos end")
        CameraCommon().pressBack(2)
        self.stopQuickPicApp()
