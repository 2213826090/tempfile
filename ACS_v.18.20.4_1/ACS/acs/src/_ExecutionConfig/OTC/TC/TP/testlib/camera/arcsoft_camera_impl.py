# coding: utf-8
import time
import os
from testlib.util.common import g_common_obj
from testlib.camera.camera_log import CameraLogger
from testlib.camera.mum_camera_impl import CameraImpl


class ArcSoftCameraImpl(CameraImpl):
    PACKAGE_NAME_ARCSOFTCAMERA = "com.arcsoft.camera2"
    ACTIVITY_NAME_ARCSOFTCAMERA = 'com.arcsoft.camera.CameraLauncher'

    PACKAGE_NAME_SETTING = "com.android.settings"
    ACTIVITY_NAME_SETTING = ".Settings"

    PACKAGE_NAME_QUICKPIC = "com.alensw.PicFolder"
    ACTIVITY_NAME_QUICKPIC = ".GalleryActivity"

    def __init__(self, cfg=None):
        self.d = g_common_obj.get_device()
        self.cfg = cfg
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]
        self.logger = CameraLogger.instance()

    def clean_up_camera_data(self):
        """
        @summary: clean Up Camera Data
        """
        g_common_obj.stop_app_am(self.PACKAGE_NAME_ARCSOFTCAMERA)
        g_common_obj.adb_cmd("pm clear %s" % self.PACKAGE_NAME_ARCSOFTCAMERA)

    #enter camera from home
    def enter_camera_from_home(self):
        """
        @summary: enter camera from home using am command
        """
        if self.ArcSoftCamera().lock_screen_icon().exists:
            g_common_obj.adb_cmd_capture_msg("input keyevent 82")
            self.logger.debug("unlock screen")
        time.sleep(3)
        self.d.press.home()
        os.system("adb shell am start -S %s/%s" %(self.PACKAGE_NAME_ARCSOFTCAMERA, self.ACTIVITY_NAME_ARCSOFTCAMERA))
        time_left = 0
        start = time.time()
        while time_left < 20:
            if self.isShutterBtnExists():
                break
            if self.GoogleDefaultCamera().camera_page_text("NEXT").exists():
                self.check_notification_after_switch_mode()
            self.checkCameraAccessDevicesLocation()
            time_left = time.time() - start
            time.sleep(0.5)
            self.judge_if_camera_crash()
        if not self.isWidgetExists(self.ArcSoftCamera().shutter_button()):
            assert False, "camera launch fail,launch time is greater than 20 seconds"

    def enter_camera_from_app_icon(self):
        """
        @summary: enter camera from Apps gallery
        """
        self.d.press.home()
        g_common_obj.launch_app_from_home_sc("Camera")
        time.sleep(2)
        self.judge_if_camera_crash()

#     def arcSoftCameraDisableOrEnable(self,type):
#         if type == "Enable":
#             os.system("adb shell pm enable %s" %self.PACKAGE_NAME_ARCSOFTCAMERA)
#         elif type == "Disable":
#             os.system("adb shell pm disable %s" %self.PACKAGE_NAME_ARCSOFTCAMERA)
#         time.sleep(1)

    def launchQuickPic(self):
        time.sleep(1)
        os.system("adb shell am start -W -n %s/%s" %(self.PACKAGE_NAME_QUICKPIC, self.ACTIVITY_NAME_QUICKPIC))

    def getDeviceModel(self):
        check_cmd = "getprop ro.product.model"
        msgs = g_common_obj.adb_cmd_capture_msg(check_cmd)
        print msgs
        assert len(msgs) != 0
        return msgs

    def waitForWidgetToAppear(self, widget, desc = "", timeout = 10):
        time_left = 0
        start = time.time()
        while time_left < timeout:
            if widget==None or widget.exists==False or widget.exists==None:
                time.sleep(0.5)
            else:
                return True
            time_left = time.time()-start
            self.logger.debug("time_left=" + str(time_left))
            self.judge_if_camera_crash()
        if widget==None or widget.exists==False or widget.exists==None:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "Wait "+str(desc) + " to appear timeout in "+str(timeout)+" s"

    def isWidgetExists(self,widget):
        if widget.exists:
            return True
        else:
            return False

    def change_front_back_camera(self, type = "Back"):
        self.waitForWidgetToAppear(self.ArcSoftCamera().switch_camera_button(), "switch camera button")
        if type == "Front":
            if self.ArcSoftCamera().flash_button().enabled:
                self.ArcSoftCamera().switch_camera_button().click.wait()
        if type == "Back" or type == "Rear":
            if not self.ArcSoftCamera().flash_button().enabled:
                self.ArcSoftCamera().switch_camera_button().click.wait()
        self.logger.debug("change to " + str(type) + " camera")
        time.sleep(2)

    def setVideoQuality(self, size = "max"):
        if not self.ArcSoftCamera().setting_video_button().exists:
            self.ArcSoftCamera().setting_button().click.wait()
        self.waitForWidgetToAppear(self.ArcSoftCamera().setting_video_button(), "setting video button")
        self.ArcSoftCamera().setting_video_button().click.wait()
        if not self.ArcSoftCamera().video_quality_text().exists:
            self.ArcSoftCamera().setting_text("Video quality").click.wait()
        self.waitForWidgetToAppear(self.ArcSoftCamera().video_quality_text(), "video quality text")
        mCount = self.ArcSoftCamera().video_quality_text().count
        if mCount > 0:
            if size == "max" or size == "Max":
                self.ArcSoftCamera().video_quality_text()[mCount - 1].click.wait()
            elif size == "min" or size == "Min":
                self.ArcSoftCamera().video_quality_text()[0].click.wait()
            else:
                self.ArcSoftCamera().setting_text(size).click.wait()
        else:
            self.logger.debug("camera not found radio button")

    def getAllVideoQuality(self):
        if not self.ArcSoftCamera().setting_video_button().exists:
            self.ArcSoftCamera().setting_button().click.wait()
        self.waitForWidgetToAppear(self.ArcSoftCamera().setting_video_button(), "setting video button")
        self.ArcSoftCamera().setting_video_button().click.wait()
        self.waitForWidgetToAppear(self.ArcSoftCamera().setting_text("Video quality"), "video quality text")
        self.ArcSoftCamera().setting_text("Video quality").click.wait()
        mCount = self.ArcSoftCamera().video_quality_text().count
        size_list = []
        for i in range(int(mCount)):
            self.logger.debug(self.ArcSoftCamera().video_quality_text()[i].text)
            size_list.append(self.ArcSoftCamera().video_quality_text()[i].text)
        return size_list

    def setColorEffect(self,_text = "None"):
        if not self.ArcSoftCamera().setting_common_button().exists:
            self.ArcSoftCamera().setting_button().click.wait()
        self.waitForWidgetToAppear(self.ArcSoftCamera().setting_common_button(), "setting common button")
        self.ArcSoftCamera().setting_common_button().click.wait()
        self.waitForWidgetToAppear(self.ArcSoftCamera().setting_text("Color effect"), "color effect button")
        self.ArcSoftCamera().setting_text("Color effect").click.wait()
        self.waitForWidgetToAppear(self.ArcSoftCamera().setting_text(_text), _text)
        self.ArcSoftCamera().setting_text(_text).click.wait()

    def setCaptureMode(self,type):
        self.waitForWidgetToAppear(self.ArcSoftCamera().mode_button(),"mode button")
        self.ArcSoftCamera().mode_button().click.wait()
        self.waitForWidgetToAppear(self.ArcSoftCamera().setting_text(type), type)
        self.ArcSoftCamera().setting_text(type).click.wait()

    def resetCameraSetting(self):
        if not self.ArcSoftCamera().setting_common_button().exists:
            self.ArcSoftCamera().setting_button().click.wait()
        self.waitForWidgetToAppear(self.ArcSoftCamera().setting_common_button(), "setting common button")
        self.ArcSoftCamera().setting_common_button().click.wait()
        self.waitForWidgetToAppear(self.ArcSoftCamera().setting_text("Reset"), "Reset button")
        self.ArcSoftCamera().setting_text("Reset").click.wait()
        self.waitForWidgetToAppear(self.ArcSoftCamera().setting_text("OK"), "OK button")
        self.ArcSoftCamera().setting_text("OK").click.wait()

    def pressbestPhotoApproveButton(self):
        self.waitForWidgetToAppear(self.ArcSoftCamera().best_photo_review_approve(), "best photo review approve")
        self.ArcSoftCamera().best_photo_review_approve().click.wait()

    def video_recording(self):
        self.waitForWidgetToAppear(self.ArcSoftCamera().record_button(), "record button")
        self.ArcSoftCamera().record_button().click.wait()

    def video_recording_capture(self):
        self.waitForWidgetToAppear(self.ArcSoftCamera().video_recording_capture_button(), "recording capture button")
        self.ArcSoftCamera().video_recording_capture_button().click.wait()

    def shutter_button(self):
        self.waitForWidgetToAppear(self.ArcSoftCamera().shutter_button(), "shutter button")
        self.ArcSoftCamera().shutter_button().click.wait()

    def continuousClickShutterButton(self,duration):
        self.waitForWidgetToAppear(self.ArcSoftCamera().shutter_button(), "shutter button")
        bounds = self.ArcSoftCamera().shutter_button().bounds
        self.logger.debug(time.time())
        self.d.swipe(bounds["left"], bounds["top"], bounds["right"], bounds["bottom"], steps=int(duration)*18)
        self.logger.debug(time.time())

    def previewPhotos(self):
        self.logger.debug("preview photos start")
#         self.refesh_camera_dir()
        self.launchQuickPic()
        self.waitForWidgetToAppear(self.QuickPic().search_button(), "search button")
        if self.QuickPic().drawer_list().exists:
            self.QuickPic().up_button().click.wait()
            self.waitForWidgetToAppear(self.QuickPic().text("Folders"), "folder bar")
        if self.QuickPic().up_button().exists:
            self.QuickPic().up_button().click.wait()
            self.waitForWidgetToAppear(self.QuickPic().text("Folders"), "folder menu")
            self.QuickPic().text("Folders").click.wait()
        self.QuickPic().search_button().click.wait()
        self.waitForWidgetToAppear(self.QuickPic().edit_text(), "edit text")
#         self.QuickPic().edit_text().clear_text()
        self.QuickPic().edit_text().set_text("Camera")
        if self.QuickPic().text("OK").exists:
            self.QuickPic().text("OK").click.wait()
        self.waitForWidgetToAppear(self.QuickPic().slide_show(), "slide show")
        bounds = self.QuickPic().action_bar().bounds
        self.d.click(160, int(bounds["bottom"]) + 100)
        self.waitForWidgetToAppear(self.QuickPic().select_button(),"select button")
        mText = self.QuickPic().action_bar_title().text
        mCount = int(mText[8:-1])
        print mCount
        self.d.click(160, int(bounds["bottom"]) + 100)
        if self.QuickPic().text("QuickPic").exists:
            self.QuickPic().text("QuickPic").click.wait()
            self.QuickPic().text("Always").click.wait()
        for i in range(mCount - 1):
            if self.QuickPic().image_view().exists:
                time.sleep(2)
            if self.QuickPic().video_play_button().exists:
                self.QuickPic().video_play_button().click.wait()
                time.sleep(3)
                if not self.QuickPic().video_play_button().exists:
                    self.press_back()
            self.d.swipe(self.x - 100, self.y/2, 100, self.y/2, 50)
            time.sleep(1)
        self.logger.debug("preview photos end")
        self.press_back_loop(2)

    def setSettingCaptureModeValue(self, category, value="middle"):
        if not self.ArcSoftCamera().setting_video_button().exists:
            self.ArcSoftCamera().setting_button().click.wait()
        self.waitForWidgetToAppear(self.ArcSoftCamera().setting_camera_button(),"setting capture button")
        self.ArcSoftCamera().setting_camera_button().click.wait()
        if self.ArcSoftCamera().setting_camera_scroll_view().scrollable:
            self.d(scrollable=True).scroll.vert.to(textContains=category)
        self.waitForWidgetToAppear(self.ArcSoftCamera().setting_text(category),"setting %s button" % category)
        self.ArcSoftCamera().setting_text(category).click.wait()
        self.waitForWidgetToAppear(self.ArcSoftCamera().setting_progress_seek_bar(),"setting %s seek bar" % category)
        bounds = self.ArcSoftCamera().setting_progress_seek_bar().bounds
#         print  bounds["left"], bounds["right"], bounds["top"], bounds["bottom"]
        if value == "middle":
            self.d.click((bounds["right"] - bounds["left"])/2 + bounds["left"], \
                         (bounds["bottom"] - bounds["top"])/2 + bounds["top"])
        elif value == "low":
            self.d.click(bounds["left"] + 45, \
                         (bounds["bottom"] - bounds["top"])/2 + bounds["top"])
        elif value == "high":
            self.d.click(bounds["right"] - 45, \
                         (bounds["bottom"] - bounds["top"])/2 + bounds["top"])
        else:
            self.logger.debug("set %s value failure" % category)

    def setFlashMode(self,value="off"):
        self.ArcSoftCamera().flash_button().click.wait()
        self.waitForWidgetToAppear(self.ArcSoftCamera().flash_icon_image_view(),"flash switch button")
#         mCount = self.ArcSoftCamera().flash_icon_image_view().count
        if value == "off" or value == "Off" or value == "OFF":
            self.ArcSoftCamera().flash_icon_image_view()[0].click.wait()
        elif value == "auto" or value == "Auto" or value == "AUTO":
            self.ArcSoftCamera().flash_icon_image_view()[1].click.wait()
        elif value == "on" or value == "On" or value == "ON":
            self.ArcSoftCamera().flash_icon_image_view()[2].click.wait()
        else:
            self.logger.debug("set flash to %s value failure" % value)

    def checkVideoRecordingButtonStatus(self):
        self.waitForWidgetToAppear(self.ArcSoftCamera().shutter_button(),"shutter button")
        status = False
        if self.ArcSoftCamera().shutter_button().enabled:
            status = True
        if not status:
            assert False,"check video recording button status fail, actual=False,expected=True"

    class ArcSoftCamera:

        def __init__(self):
            self.d = g_common_obj.get_device()

        def shutter_button(self):
            '''
            shutter button
            '''
            return self.d(resourceId = "com.arcsoft.camera2:id/shutter_button")

        def record_button(self):
            '''
            record button
            '''
            return self.d(resourceId = "com.arcsoft.camera2:id/record_button")

        def image_view(self):
            '''
            image view
            '''
            return self.d(index = 0, className = "android.widget.ImageView")

        def setting_button(self):
            '''
            setting button
            '''
            return self.d(resourceId = "com.arcsoft.camera2:id/btn_setting")

        def flash_button(self):
            '''
            flash button
            '''
            return self.d(resourceId = "com.arcsoft.camera2:id/btn_flash")

        def switch_camera_button(self):
            '''
            switch camera button
            '''
            return self.d(resourceId = "com.arcsoft.camera2:id/btn_switch_camera")

        def mode_button(self):
            '''
            mode button
            '''
            return self.d(resourceId = "com.arcsoft.camera2:id/btn_mode")

        def setting_text(self, _text):
            '''
            page text
            '''
            return self.d(text = _text)

        def setting_common_button(self):
            '''
            setting common button
            '''
            return self.d(resourceId = "com.arcsoft.camera2:id/setting_common_title")

        def setting_camera_button(self):
            '''
            setting camera button
            '''
            return self.d(resourceId = "com.arcsoft.camera2:id/setting_camera_title")

        def setting_video_button(self):
            '''
            setting video button
            '''
            return self.d(resourceId = "com.arcsoft.camera2:id/setting_video_title")

        def radio_button(self):
            '''
            radio button
            '''
            return self.d(resourceId = "com.arcsoft.camera2:id/setting_expand_icon")

        def video_quality_text(self):
            '''
            video quality text
            '''
            return self.d(resourceId = "com.arcsoft.camera2:id/setting_expand_text")

        def video_recording_capture_button(self):
            '''
            video quality text
            '''
            return self.d(description = "switch_mode", index=2)

        def best_photo_review_approve(self):
            '''
            best photo review approve
            '''
            return self.d(resourceId = "com.arcsoft.camera2:id/btn_bestphoto_review_approve")

        def best_photo_icon(self):
            '''
            best photo icon
            '''
            return self.d(className="android.widget.ImageView", index = 2)

        def setting_camera_scroll_view(self):
            '''
            setting camera scroll view
            '''
            return self.d(resourceId = "com.arcsoft.camera2:id/setting_camera_scroll_view")

        def setting_progress_seek_bar(self):
            '''
            setting camera scroll view
            '''
            return self.d(resourceId = "com.arcsoft.camera2:id/setting_progress_seek_bar")

        def lock_screen_icon(self):
            '''
            camera lock screen icon
            '''
            return self.d(resourceId = "com.android.systemui:id/lock_icon")

        def flash_icon_image_view(self):
            '''
            flash icon image view
            '''
            return self.d(resourceId = "com.arcsoft.camera2:id/icon_image_view")

    class QuickPic:

        def __init__(self):
            self.d = g_common_obj.get_device()

        def more_option(self):
            '''
            more option
            '''
            return self.d(description="More options")

        def up_button(self):
            '''
            up button
            '''
            return self.d(resourceId = "android:id/up")

        def text(self, _text):
            '''
            page text
            '''
            return self.d(text = _text)

        def drawer_list(self):
            '''
            drawer list
            '''
            return self.d(resourceId = "com.alensw.PicFolder:id/drawer_list")

        def search_button(self):
            '''
            search button
            '''
            return self.d(resourceId = "com.alensw.PicFolder:id/search")

        def edit_text(self):
            '''
            edit text
            '''
            return self.d(className="android.widget.EditText", index = 0)

        def slide_show(self):
            '''
            slide show
            '''
            return self.d(resourceId = "com.alensw.PicFolder:id/slideshow")

        def action_bar_title(self):
            '''
            action bar title
            '''
            return self.d(resourceId = "android:id/action_bar_title")

        def select_button(self):
            '''
            select button
            '''
            return self.d(resourceId = "com.alensw.PicFolder:id/select")

        def image_view(self):
            '''
            image view
            '''
            return self.d(resourceId = "com.alensw.PicFolder:id/image")

        def video_play_button(self):
            '''
            video play button
            '''
            return self.d(resourceId = "com.alensw.PicFolder:id/play")

        def action_bar(self):
            '''
            action bar
            '''
            return self.d(resourceId = "android:id/action_bar")


#camera_impl = ArcSoftCameraImpl()
