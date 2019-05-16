# coding: utf-8
import time
import platform
import os
import random
import subprocess
from testlib.util.otc_node import OTCNode
from testlib.util.common import g_common_obj
from testlib.util import constant
from testlib.util.otc_image import otcImage
from testlib.browser.chrome_impl import ChromeImpl
from testlib.common.common import g_common_obj2
from subprocess import Popen,PIPE
from testlib.util.repo import Artifactory
from testlib.camera.camera_log import CameraLogger
# from testlib.util.pytesser.pytesser import *


class CameraImpl:
    """
        @summary: class for camera application Home UI
    """

    PACKAGE_NAME_CAMERA = "com.google.android.GoogleCamera"
    ACTIVITY_NAME_CAMERA = 'com.android.camera.CameraLauncher'

    PACKAGE_NAME_SETTING = "com.android.settings"
    ACTIVITY_NAME_SETTING = ".Settings"

    PACKAGE_NAME_FLASH = "com.devuni.flashlight"
    ACTIVITY_NAME_FLASH= ".MainActivity"

    PACKAGE_NAME_CAMERA2 = "com.android.camera2"
    ACTIVITY_NAME_CAMERA2="com.android.camera.CameraLauncher"

    # --------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_next(self):
            """ UI button next """
            return self.d(text="NEXT")

        @property
        def btn_okgotit(self):
            """ UI button next """
            return self.d(text="OK, GOT IT")


    def __init__(self, cfg=None):
        self.d = g_common_obj.get_device()
        self._locator = CameraImpl.Locator(self.d)
        self.cfg = cfg
        self.dut = self.d
        self.sutter_btn_top = 0
        self.sutter_btn_bottom = 0
        self.sutter_btn_left = 0
        self.sutter_btn_right = 0
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]
        self.logger = CameraLogger.instance()

    def set_orientation_n(self):
        """
        @summary: set orientation as n
        """
        #self.d.orientation = "n"
        g_common_obj.set_vertical_screen()

    def clean_up_camera_data(self):
        """
        @summary: clean Up Camera Data
        """
        g_common_obj.stop_app_am(self.PACKAGE_NAME_CAMERA)
        g_common_obj.adb_cmd("pm clear %s" % self.PACKAGE_NAME_CAMERA)

    def check_notification_after_switch_mode(self):
        """
        @summary: check the notifications after switch mode
        """
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        for i in range(5):
            # check "YES" button
            if self.dut(text='Yes').exists:
                self.dut(text='Yes').click()
            # check "NEXT" button
            if self.dut(text='NEXT').exists:
                self.dut(text='NEXT').click()
            # check "OK, GOT IT" button
            if self.dut(text='OK, GOT IT').exists:
                self.dut(text='OK, GOT IT').click()
            # check "OK, GOT it" button
            if self.dut(text='OK, Got it').exists:
                self.dut(text='OK, Got it').click()

    def checkCameraAccessDevicesLocation(self):
        if self.d(text="Allow").exists:
            self.d(text="Allow").click.wait()

    def unlockScreen(self):
        self.d.wakeup()
        if self.GoogleDefaultCamera().lock_screen_icon().exists():
            g_common_obj.adb_cmd_capture_msg("input keyevent 82")
            self.logger.debug("unlock screen")
        time.sleep(2)

    #enter camera from home
    def enter_camera_from_home(self):
        """
        @summary: enter camera from home using am command
        """
#         g_common_obj2.unlock()
        self.unlockScreen()
        time.sleep(3)
        self.d.press.home()
#         g_common_obj.launch_app_am(self.PACKAGE_NAME_CAMERA, self.ACTIVITY_NAME_CAMERA)
        self.logger.debug("launch camera start")
        os.system("adb shell am start -n %s/%s" %(self.PACKAGE_NAME_CAMERA, self.ACTIVITY_NAME_CAMERA))
        time_left = 0
        start = time.time()
        success = False
        while time_left < 20:
            if self.isShutterBtnExists() or self.GoogleDefaultCamera().camera_photo_show_page_delete_button().exists():
                success = True
                break
            self.checkCameraAccessDevicesLocation()
            if self.GoogleDefaultCamera().camera_page_text("NEXT").exists():
                self.check_notification_after_switch_mode()
            time_left = time.time() - start
            time.sleep(0.5)
            self.judge_if_camera_crash()
        if (not success) or time_left > 20:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "camera launch fail,launch time is greater than 20 seconds"
        self.logger.debug("launch camera success")

    def enter_camera_from_app_icon(self):
        """
        @summary: enter camera from Apps gallery
        """
        self.d.press.home()
        g_common_obj.launch_app_from_home_sc("Camera")
        time.sleep(2)
        self.judge_if_camera_crash()
        self.check_notification_after_switch_mode()
        time.sleep(1)

    def enter_camera_from_recent_task(self):
        """
        @summary: enter camera from recent tasks page
        """
        self.d.press.home()
        self.click_recent_app_and_enter_app()

    def click_recent_app_and_enter_app(self,app_name="Camera"):
        """
        @summary: click recent applications button and enter camera
        """
        self.recent_app()
        self.click_recent_app(app_name)
        time.sleep(2)
        self.judge_if_camera_crash()
        self.check_notification_after_switch_mode()
        time.sleep(1)

    def remove_camera_from_recent_task(self):
        """
        @summary: remove camera from recent tasks page
        """
        self.recent_app()
        self.remove_recent_app("Camera")
        time.sleep(1)

    def remove_all_app_from_recent_task(self):
        self.unlockScreen()
        for i in range(3):
            self.d.press.recent()
            time.sleep(1)
            while True:
                if self.d(resourceId="com.android.systemui:id/task_view_thumbnail").exists:
                    bounds = self.d(resourceId="com.android.systemui:id/task_view_thumbnail").bounds
                    self.d.swipe(bounds.get("left")+50, bounds.get("top")+30, bounds.get("right"), bounds.get("top")+30,steps=100)
                else:
                    break
                time.sleep(0.5)
            self.d.press.home()
        assert (not self.d(resourceId="com.android.systemui:id/task_view_thumbnail").exists), "remove all app from recent task failed"

    def recent_app(self):
        '''
        former name : recentApp
        '''
        self.d.press.recent()
        time.sleep(2)
        reID = "com.android.systemui:id/task_view_content"
        if self.d(resourceId=reID).exists==False:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.d(resourceId=reID).exists, "enter recents container failed"

    def remove_recent_app(self, appName):
        '''
        former name : removeRecentApp
        '''
        x = self.d.info["displayWidth"]
        if not self.d(text=appName).exists:
            return False, "not found " + str(appName)
        bounds = self.d(text=appName).bounds
        self.logger.debug(str(bounds))
        self.d.swipe(bounds.get("left"), bounds.get("top"), x-50, bounds.get("top"))
        time.sleep(2)
        if self.d(text=appName).exists and self.d(resourceId="com.android.systemui:id/activity_description").exists:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert not self.d(text=appName).exists, "remove recent app " + str(appName) + " failed"

    def click_recent_app(self, appName):
        '''
        former name : clickRecentApp
        '''
        if self.d(text=appName).exists==False:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.d(text=appName).exists, "not found " + str(appName)
        bounds = self.d(text=appName).bounds
        self.logger.debug(str(bounds))
        self.d.click(bounds.get("left"), bounds.get("top"))

    def get_camera_folder_info(self):
        """
        @summary: get the detail info of /sdcard/DCIM/Camera/
        @return: photoNum, folderSize
            photoNum: the number of all photoes in /sdcard/DCIM/Camera/
            folderSize: the size of /sdcard/DCIM/Camera/
        """
        photoNum = int(g_common_obj.adb_cmd_capture_msg("ls /sdcard/DCIM/Camera/*.jpg | wc -l"))
        if photoNum < 3:
            textList = g_common_obj.adb_cmd_capture_msg("ls /sdcard/DCIM/Camera/*.jpg")
            if textList.find("No such file or directory") != -1:
                photoNum = 0
        self.logger.debug("now there are " + str(photoNum) + " photos")

        folderSize = long(g_common_obj.adb_cmd_capture_msg("du /sdcard/DCIM/Camera | awk '{print $1}'"))
        self.logger.debug("folder Size is " + str(folderSize))
        return photoNum, folderSize

    def get_camera_folder_video_num(self):
        """
        @summary: get the detail info of /sdcard/DCIM/Camera/
        @return: videoNum, folderSize
            videoNum: the number of all videos in /sdcard/DCIM/Camera/
        """
        videoNum = int(g_common_obj.adb_cmd_capture_msg("ls /sdcard/DCIM/Camera/*.3gp | wc -l"))
        textList = g_common_obj.adb_cmd_capture_msg("ls /sdcard/DCIM/Camera/*.3gp")
        if textList.find("No such file or directory") != -1:
            videoNum = 0
        self.logger.debug("now there are " + str(videoNum) + " videos")

        return videoNum

    def switch_module_in_camera(self, module="Video"):
        """
        @summary: switch module
        @param: module
            module: the module name need to be set, defaul module is "Video"
        module list: Photo Sphere, Panorama, Lens Blur, Camera, Video
        """
        if not self.d(text=module).exists:
            x = self.d.info["displayWidth"]
            y = self.d.info["displayHeight"]
            self.d.swipe(0, y/2, x/2, y/2, 10)
        if not self.d(text=module).exists:
            return
        self.d(text=module).click()
        self.check_notification_after_switch_mode()
        time.sleep(1)

    def capture_photos(self, num):
        """
        @summary: capture mutiple photos
        @param: num
            num: the number of photos need to be captured
        If not set checkNum, will check each picture.
        """
        for i in range(num):
            photoNum, folderSize = self.get_camera_folder_info()
            self.logger.debug("photoNum = " + str(photoNum))
            self.logger.debug("folderSize = " + str(folderSize))
            self.capture_photo()
            time.sleep(3)
            photoNumNow, folderSizeNow = self.get_camera_folder_info()
            self.logger.debug("photoNumNow = " + str(photoNumNow))
            self.logger.debug("folderSizeNow = " + str(folderSizeNow))
            self.logger.debug("[compute]take " + str(photoNumNow - photoNum) + " photo")
            self.logger.debug("[compute]increase " + str(folderSizeNow - folderSize) + " space")
            capture_button_status = self.d(resourceId="com.android.camera2:id/shutter_button").enabled
            self.logger.debug("capture_button_status: " + str(capture_button_status))
            self.logger.debug(str(capture_button_status == True))
            if (folderSizeNow > folderSize) and (photoNumNow - photoNum == 1) and capture_button_status == True:
                pass
            else:
                time.sleep(3)
                self.check_capture_hang()
                self.logger.debug("Camera not hang") 
                photoNumNow, folderSizeNow = self.get_camera_folder_info()
                if photoNumNow == photoNum or capture_button_status == False:
                    self.judge_if_camera_crash()
                    if self.check_memory_full():
                        return "Storage run out"
                    self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
                    assert False, "Save picture failed."
        return "Pass"

    def capture_photo(self):
        """
        @summary: click the shutter button to capture one photo
        """
        self.waitForWidgetToAppear(self.GoogleDefaultCamera().camera_page_shutter(False),"shutter button")
        if self.GoogleDefaultCamera().camera_page_shutter(False).exists:
            self.GoogleDefaultCamera().camera_page_shutter().click()
        else:
            if not self.GoogleDefaultCamera().camera_page_three_dots(False).exists():
                # Check camera crash
                self.judge_if_camera_crash()

    def check_capture_hang(self):
        if self.GoogleDefaultCamera().camera_page_three_dots(False).exists==False:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.GoogleDefaultCamera().camera_page_three_dots(False).exists, "Can't find the three dots on the camera screen"
#         if self.GoogleDefaultCamera().camera_page_three_dots(False).exists:
#             self.GoogleDefaultCamera().camera_page_three_dots(False).click()
#             assert self.GoogleDefaultCamera().camera_page_camera_toggle_button().exists, "Camera hang"

    def check_memory_full(self):
        total, used, free = self.get_sdcard_memory()
        self.logger.debug("storage has " + str(free) + "M space else")
        self.d.press.home()
        self.d.open.notification()
        time.sleep(4)
        if self.d(text="Storage space running out").exists:
            self.logger.debug("storage space runs out")
            return True
        else:
            self.logger.debug("[error]there is no notification of no space") 
            return False

    def check_video_hang_when_recording(self):
        fileNum, folderSize = self.get_camera_folder_info()
        time.sleep(2)
        fileNumNow, folderSizeNow = self.get_camera_folder_info()
        if (folderSizeNow <= folderSize):
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "Video hang."

    def change_front_back_camera(self, type):
        """
        @summary: switch to back or front camera
        @param: type
            type: Front/Back
        @return: description
        """
        if self.GoogleDefaultCamera().camera_page_three_dots(False).exists():
            self.GoogleDefaultCamera().camera_page_three_dots().click()
            time.sleep(1)
        elif self.d(description = "Options").exists:
            self.d(description = "Options").click()
            time.sleep(1)
        self.waitForWidgetToAppear(self.GoogleDefaultCamera().camera_page_camera_toggle_button(), "switch camera lens button")
        description = self.GoogleDefaultCamera().camera_page_camera_toggle_button().getContentDescription()
        self.logger.debug("description is " + str(description) + " want to change to " + str(type))
        time.sleep(1)
        if str(type).upper().replace(" ", "") not in str(description).replace(" ", "").upper():
            self.GoogleDefaultCamera().camera_page_camera_toggle_button().click()
        description = self.GoogleDefaultCamera().camera_page_camera_toggle_button().getContentDescription()
        return description
#=========================================================End xiaofei finished=============================================================================
    def on_off_grid(self,type):
        """
        @summary: switch to grid
        @param: type
            type: on/off
        @return: description
        """
        time.sleep(3)
        if self.GoogleDefaultCamera().camera_page_three_dots(False).exists():
            self.GoogleDefaultCamera().camera_page_three_dots().click()
            time.sleep(1)
        description = self.GoogleDefaultCamera().camera_page_grid_lines_toggle_button().getContentDescription()
        self.logger.debug("description is " + str(description) + " want to change to " + str(type))
        time.sleep(3)
        type = "Grid lines " + type
        if str(type).upper().replace(" ", "") not in str(description).replace(" ", "").upper():
            self.GoogleDefaultCamera().camera_page_grid_lines_toggle_button().click()
        description = self.GoogleDefaultCamera().camera_page_grid_lines_toggle_button().getContentDescription()
        return description

    def set_orientation(self,orientation="n"):
        """
        @summary: set orientation as n,l,r
        """
        self.d.orientation = orientation

    def shutter_btn_long_click(self):
        """
        @summary: long click the shutter button to capture one photo
        """
        self.GoogleDefaultCamera().camera_page_shutter().long_click()

    def click_screen_center(self):
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        self.d.click(x/2, y/2)
        time.sleep(2)
        if self.GoogleDefaultCamera().camera_page_text("Photos").exists():
            self.GoogleDefaultCamera().camera_page_text("Photos").click()
            self.GoogleDefaultCamera().camera_page_text("Always").click()

    def click_screen_any_area(self):
        """
        @summary: click any area  of the screen
        """
        width = self.d.info["displayWidth"]
        height = self.d.info["displayHeight"]
        x = random.randint(1, width)
        y = random.randint(1, height)
        self.d.click(x, y)

    def click_shutter_button_area(self):
        """
        @summary: click the area  of shutter button
        """
        bounds = self.d(resourceId="com.android.camera2:id/shutter_button").bounds
        shutter_top = bounds.get("top")
        shutter_bottom = bounds.get("bottom")
        shutter_left = bounds.get("left")
        shutter_right = bounds.get("right")
        mx = (shutter_left + shutter_right)/2
        my = (shutter_top + shutter_bottom)/2
        self.logger.debug("width = " + str(mx) + ", height = " + str(my))
        self.d.click(mx, my)
        return mx,my

    def click_panorama_done_button_area(self):
        """
        @summary: click the area of panorama done button
        """
        if self.d(resourceId="com.android.camera2:id/done_button").exists:
            bounds = self.d(resourceId="com.android.camera2:id/done_button").bounds
            shutter_top = bounds.get("top")
            shutter_bottom = bounds.get("bottom")
            shutter_left = bounds.get("left")
            shutter_right = bounds.get("right")
            mx = (shutter_left + shutter_right)/2
            my = (shutter_top + shutter_bottom)/2
            self.logger.debug("width = " + str(mx) + ", height = " + str(my))
            self.d.click(mx, my)

    def capture_photo_manual_focus_and_long_click(self):
        """
        @summary: manual focus and long click capture photo
        """
        self.click_screen_center()
        self.shutter_btn_long_click()

    def long_click_shutter_then_flip_up(self):
        """
        @summary: long click shutter then flip up
        """
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        bounds = self.d(resourceId="com.android.camera2:id/shutter_button").bounds
        shutter_top = bounds.get("top")
        shutter_bottom = bounds.get("bottom")
        self.logger.debug("shutter button position top=%s,bottom=%s" %(shutter_top,shutter_bottom)) 
        if (shutter_top < y-100 and shutter_bottom > y-100):
            self.d.swipe(x/2, y-75,x/2,y-400, steps=300)
        else:
            self.d.swipe(x/2, 75, x/2, 400, steps=300)
        time.sleep(2)

    def swipe_screen(self, orientation = "right"):
        """
        @summary: swipe to capture mode
        """
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        if not self.GoogleDefaultCamera().camera_page_settings_button(False).exists():
            if orientation == "left":
                self.d.swipe(x-10,y/2,x/2,y/2,steps=50)
                self.logger.info("Swipe to left")
            elif orientation == "right":
                self.d.swipe(0,y/2,x/2,y/2,steps=50)
                self.logger.info("Swipe to right")
            elif orientation == "up":
                self.d.swipe(x/2,y/2,x/2,0,steps=50)
                self.logger.info("Swipe to up")
            else:
                self.logger.info("No change")
        else:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False,"[ERROR]: Layout not found!"
        time.sleep(3)
        if self.GoogleDefaultCamera().camera_page_text("Photos").exists():
            self.GoogleDefaultCamera().camera_page_text("Photos").click()
            self.GoogleDefaultCamera().camera_page_text("Always").click()

    def switch_to_camera_options(self, text = "Camera"):
        """
        @summary: switch to camera options
        """
        self.swipe_screen()
        if text == "Camera":
            self.GoogleDefaultCamera().camera_page_camera().click()
            self.logger.info("Change to camera mode")
        elif text == "Video":
            self.GoogleDefaultCamera().camera_page_photo_video().click()
            self.logger.info("Change to video mode")
        elif text == "Lens Blur":
            if not self.GoogleDefaultCamera().camera_page_lens_blur().exists():
                assert False, "camera not found Lens Blur mode"
            self.GoogleDefaultCamera().camera_page_lens_blur().click()
            self.logger.info("Change to lens blur")
        elif text == "Panorama":
            if not self.GoogleDefaultCamera().camera_page_panorama().exists():
                assert False, "camera not found Panorama mode"
            self.GoogleDefaultCamera().camera_page_panorama().click()
            self.logger.info("Change to panorama")
            if self.d(resourceId="com.android.camera2:id/next_button").exists:
                self.d(resourceId="com.android.camera2:id/next_button").click()
            if self.d(resourceId="com.android.camera2:id/next_button").exists:
                self.d(resourceId="com.android.camera2:id/next_button").click()
        elif text == "Photo Sphere":
            if not self.GoogleDefaultCamera().camera_page_photo_sphere().exists():
                assert False, "camera not found Photo Sphere mode"
            self.GoogleDefaultCamera().camera_page_photo_sphere().click()
            self.logger.info("Change to photo sphere")
            if self.d(resourceId="com.android.camera2:id/next_button").exists:
                self.d(resourceId="com.android.camera2:id/next_button").click()
            if self.d(resourceId="com.android.camera2:id/next_button").exists:
                self.d(resourceId="com.android.camera2:id/next_button").click()
        else:
            self.logger.info("No change")
        time.sleep(2)

    def capture_images_continuously(self, number = 1):
        for i in range(number):
            self.capture_photo()
            time.sleep(2)

    def press_back_loop(self,times=1):
        for i in range(times):
            self.d.press.back()

    def video_recording(self):
        self.waitForWidgetToAppear(self.GoogleDefaultCamera().camera_page_shutter(False),"shutter button")
        self.GoogleDefaultCamera().camera_page_shutter().click()
        self.logger.info("video recording")

    def setCameraMaxOrMinResolution(self, max_min, cameraType=constant.CAMERA_TYPE_BACK, founcType="video"):
        self.GoogleDefaultCamera().camera_Reso_quality_setting_page_set(True, cameraType, founcType).click()
        mList=[]
        for i in range(10):
            if self.d(resourceId = "android:id/text1",index=i).exists:
                mList.append(self.d(resourceId = "android:id/text1",index=i).info["text"])
            else:
                break
        minSize = len(mList)
        self.logger.debug("mlist lenth= " + str(minSize))
        if (max_min == "max" or max_min =="Max"):
            self.logger.debug("change to " + str(mList[0]))
            self.d(resourceId = "android:id/text1",index=0).click()
            return mList[0]
        elif (max_min == "middle" or max_min =="Middle"): 
            self.logger.debug("change to " + str(mList[1]))
            self.d(resourceId = "android:id/text1",index=1).click()
            return mList[1]
        elif (max_min == "min" or max_min =="Min"):
            self.logger.debug("change to " + str(mList[minSize-1]))
            self.d(resourceId = "android:id/text1",index=(minSize-1)).click()
            return mList[minSize-1]
        else:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False,"This resolution not find!"
            return None

    def getRadioButtonsOnCurrentView(self):
        radio_button_list=[]
        for i in range(10):
            if self.d(resourceId = "android:id/text1",index=i).exists:
                radio_button_list.append(self.d(resourceId = "android:id/text1",index=i))
            else:
                break
        return radio_button_list

    def install_verify_apps(self, apk_name, package_name):
        for i in range(10):
            time.sleep(2)
            result = g_common_obj.adb_cmd_common("shell pm list package | grep %s" % (package_name))
            if  "package" in result:
                self.logger.debug("app has been installed")
                return
            else:
                self.logger.debug("install app")
                g_common_obj.adb_cmd_common('install -r %s' % (apk_name))
                for i in range(10):
                    time.sleep(2)
                    if self.d(text="Accept").exists:
                        self.d(text="Accept").click.wait()
                        break

    def download_content(self, url, file_name):
    # download content from url
        self.arti_obj = Artifactory(url)
        for i in range(10):
            self.ret_file = self.arti_obj.get(file_name)
            if os.path.exists(self.ret_file):
                self.logger.debug("download file exists")
                return self.ret_file
        if os.path.exists(self.ret_file)==False:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert os.path.exists(self.ret_file), "download file filed!"

    def downloadAndInstallApp(self,url,apk_name,pkg_name):
        filepath = self.download_content(url, apk_name)
        self.install_verify_apps(filepath,pkg_name)

    def launchCamera2App(self,pkg,activity):
        self.unlockScreen()
        time.sleep(2)
        self.d.press.home()
        g_common_obj.launch_app_am(pkg, activity)
        time.sleep(2)
        if self.d(text="NEXT").exists:
            self.d(text="NEXT").click()

    def launchAlarmApp(self,package_name,activity_name):
        self.unlockScreen()
        g_common_obj.launch_app_am(package_name,activity_name)
        time.sleep(3)
        if self.Alarm().alarm_page_title().exists==False:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.Alarm().alarm_page_title().exists, "launch alarm app failed!"

    def launchFlashApp(self):
        self.unlockScreen()
        time.sleep(2)
        self.d.press.home()
        g_common_obj.launch_app_am(self.PACKAGE_NAME_FLASH, self.ACTIVITY_NAME_FLASH)
        time.sleep(3)

    def setAlarmTime(self, seconds):
        self.Alarm().alarm_page_time().set_text(seconds)
        time.sleep(1)
        self.d(text="OK").click()

    def waitAlarmTriiggered(self, wait_time=30):
        t_time = 2
        for i in range(1, wait_time, t_time):
            time.sleep(t_time)
            if self.Alarm().alarm_prompt_message().exists:
                break
        if self.Alarm().alarm_prompt_message().exists==False:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.Alarm().alarm_prompt_message().exists, "Alarm wait timeout!"

    def alarmDismissOrSnooze(self,trigger="Dismiss"):
        if self.Alarm().alarm_prompt_dismiss_btn().exists or \
            self.Alarm().alarm_prompt_snooze_btn().exists:
            if trigger == "Dismiss":
                self.Alarm().alarm_prompt_dismiss_btn().click()
                return True
            else:
                self.Alarm().alarm_prompt_snooze_btn().click()
                return True
        return False

    def setWallpaper(self,tag="default"):
        self.d.press.home()
        self.logger.debug("set wallpaper start")
        bounds=self.d(description = "Apps").bounds
        icon_top = bounds.get("top")
        icon_left = bounds.get("left")
        self.d.long_click(icon_left, icon_top-100)
        self.Settings().set_wallpaper_text("Wallpapers").click.wait()
        if tag == "picture":
            self.Settings().set_wallpaper_item_label().click.wait()
            self.waitForWidgetToAppear(self.Settings().set_wallpaper_icon_mime(),"wallpaper thumbnail")
            self.Settings().set_wallpaper_icon_mime().click.wait()
            time.sleep(3)
            self.Settings().set_wallpaper_text("Set wallpaper").click.wait()
        elif tag == "default":
            print self.Settings().set_wallpaper_image().count
            self.Settings().set_wallpaper_image()[4].click.wait()
            self.Settings().set_wallpaper_text("Set wallpaper").click.wait()
        time.sleep(2)
        self.logger.debug("set wallpaper end")

    def changeLanguageInSetting(self,languageAndInput,languageItem,language):
        self.unlockScreen()
        g_common_obj.launch_app_am(self.PACKAGE_NAME_SETTING, self.ACTIVITY_NAME_SETTING)
        time.sleep(2)
        if not self.d(text = languageAndInput).exists:
            self.d(scrollable=True).scroll.vert.to(text = languageAndInput)
        self.d(text = languageAndInput).click()
        if self.d(text = languageItem).exists==False:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.d(text = languageItem).exists,"language not exists"
        self.d(text = languageItem).click()
        if not self.d(text = language).exists:
            self.d(scrollable=True).scroll.vert.to(text = language)
        self.d(text = language).click()
        time.sleep(2)
        if self.d(text = languageItem).exists:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert not self.d(text = languageItem).exists,"language not change"
        self.logger.debug("change language to %s success" %(language))
        self.press_back_loop(2)

    def capturePicture(self):
        time.sleep(1)
        self.d(resourceId="com.android.camera2:id/shutter_button").click()
        time.sleep(2)

    def getFileName(self,device_path):
        cmd = "adb shell ls " + str(device_path) + "*.*"
        list = []
        output=subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE).communicate()
        result = output[0][:-1]
        if result.find("No such file or directory")!= -1:
            return []
        else:
            for r in result.split("\n"):
                fullpath = r[:-1]
                l = fullpath.split('/')
                name = l[len(l)-1]
                list.append(name) 
            self.logger.debug("list length = %d" %(len(list)))
            return list

    def getFileSize(self,cmd):
        list=[]
        output=subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE).communicate()
        return output[0].rstrip()

    def adb_pull_file(self, device_path, host_path, is_check_file=False, file_size=None):
        """ adb pull file from device to host """ 
        self.logger.debug("Adb pull file from device: %s to host: %s" % (device_path, host_path))
        cmd = 'adb pull %s %s' % (device_path, host_path)
        self.logger.debug("Pull file: %s" % cmd) 
        os.popen(cmd)
        if is_check_file:
            exist = self.file_exists(host_path, location="HOST")
            if file_size:
                self.verify_file_size(host_path, file_size)
            if exist==False:
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert exist, "Expect file exist, Actual file not exist"
        self.logger.debug("Pull file successfully") 
        time.sleep(3)

    def adb_push_file(self, host_path, device_path, is_check_file=False):
        """ adb push file from host to device """ 
        self.logger.debug("Adb push file from host: %s to device: %s" % \
        (host_path, device_path))
        cmd = 'adb push %s %s' % (host_path, device_path)
        self.logger.debug("Push file: %s" % cmd)
        os.popen(cmd)
        if is_check_file:
            exist = self.file_exists(device_path, location="DEVICE")
            if exist==False:
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert exist, \
                "Expect file exist, Actual file not exist"
        self.logger.debug("Push file successfully")

    @staticmethod
    def file_exists(file_path, location):
        """ check file exist, support check on device or host """
        CameraLogger.instance().debug("Check file: %s exist on %s or not" % \
        (file_path, location.upper()))
        exist = False
        if location.upper() == "DEVICE":
            cmd = 'ls %s; echo $?' % (file_path)
            result_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\n')
            CameraLogger.instance().debug(str(result_list))
        elif location.upper() == "HOST":
            cmd = 'ls %s 2>&1; echo $?' % (file_path)
            result_list = os.popen(cmd).readlines()
        else: 
            CameraLogger.instance().debug("Can not support this location:[%s] information" % location)
            return False

        if len(result_list):
            ret = result_list[-1].rstrip()
            ret2= result_list[0].rstrip()
            if ret2.count("No such file or directory")!=0:
                exist = False
                return exist
            if int(ret) == 0:
                exist = True
        CameraLogger.instance().debug("File " + str(exist and "Exist" or "Not Exist"))
        return exist

    @staticmethod
    def get_file_size(file_path):
        """ get size of file on host, return size of bytes """ 
        CameraLogger.instance().debug("Get size of file: %s" % file_path)
        cmd = ''.join(['stat -c %s ', file_path, ' 2>&1; echo $?'])
        result_list = os.popen(cmd).readlines()
        if len(result_list):
            if int(result_list[-1].rstrip()) == 0:
                return int(result_list[0].rstrip())
        return -1

    def verify_file_size(self, file_path, exp_size):
        """ verify file size on host """
        self.logger.debug("Verify file size of %s, expect size: %s" % (file_path, exp_size))
        bs, count = self.parse_bs_count(exp_size)
        if bs <= 0:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert bs>0, "Parse bs, count for dd command fail"
        exp_size = bs*count
        act_size = self.get_file_size(file_path)
        self.logger.debug("Expect size: %d bytes, Actual size: %d bytes" %(exp_size, act_size))
        if act_size!=exp_size:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert act_size==exp_size, "File %s expect size:[%d], but actual size:[%d]" % \
            (file_path, exp_size, act_size)

    def checkFile(self,path):
        self.logger.debug(path)
        isExists = False
        try:
            if os.path.exists(path):
                isExists = True
            else:
                self.logger.debug("file not exists")
        except Exception as e:
            self.logger.debug(str(e))
        return isExists

    def openNotificationAndCheckMusicStatus(self,status):
        self.d.swipe(self.x/2, 3, self.x/2, self.y/3, steps=100)
        self.d.open.notification()
        start = time.time()
        success = False
        while time.time() - start < 10:
            if self.d(resourceId="android:id/action0").exists:
                success = True
                break
            time.sleep(1)
        if (not self.d(resourceId="android:id/action0").exists) or (not success):
            assert False,"open notification bar fail"
        if not self.d(resourceId="android:id/action0", description=status):
            assert False, "check music status fail, expect=play actual=%" %status
        if status == "Play":
            self.d(resourceId="android:id/action0", description=status).click.wait()
        self.press_back()

    def getExifInfo(self, filename, host_path):
        cmd = "exiftool %s%s" %(host_path,filename)
        self.logger.debug(cmd)
        output = subprocess.Popen(cmd,shell=True,stdout=subprocess.PIPE).communicate()
        exifinfo=output[0]
        lineList = exifinfo.split("\n")
        info={}
        for line in lineList:
            mLine= line.split(":")
            if len(mLine) <=1:
                continue
            else:
                info.setdefault(mLine[0].strip(),mLine[1].strip())
        return info

    def removeFile(self,path):
        try: 
            self.logger.debug("rm -rf " + path)
            os.system("rm -rf " + path)
            if os.path.exists(path):
                os.system("rm -rf " + path)
            else:
                self.logger.debug("remove file success")
        except Exception as e:
            self.logger.debug(str(e))

    def getTmpDir(self):
        path = "/tmp"
        if not os.access(path, os.R_OK|os.W_OK):
            path = "~/tmp"
            if not os.path.exists(path):
                os.mkdir(path)
        path = os.path.join(path, "logs")
        if not os.path.exists(path):
            os.mkdir(path)
        return path

    def get_file_from_temp_dir(self):
        '''
        get a file list under tmp dir
        '''
        tmp_dir = self.getTmpDir()
        file_list = os.listdir(tmp_dir) 
        self.logger.debug('The files under %s are %s' % (tmp_dir, file_list))
        return file_list

    def checkVideoDisplay(self,sleep_time,timeout=10):
        time.sleep(2)
        self.swipe_screen("left")
        self.click_screen_center()
        if self.GoogleDefaultCamera().camera_page_video_player_button(False).exists():
            self.click_screen_center()
        time_left = 0
        start = time.time()
        while time_left < timeout:
            if self.d(packageName="com.google.android.apps.photos").exists:
                self.closeFullScreenView()
            if self.GoogleDefaultCamera().camera_page_text("Always").exists():
                self.GoogleDefaultCamera().camera_page_text("Always").click()
            if (not self.GoogleDefaultCamera().camera_page_video_player().exists()) and \
                (not self.GoogleDefaultCamera().camera_video_view_for_m().exists()):
                time.sleep(0.5)
            else:
                break
            time_left = time.time() - start
        if time_left > timeout:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "wait media player widget timeout"
        if self.GoogleDefaultCamera().camera_page_video_player().exists() or \
                self.GoogleDefaultCamera().camera_video_view_for_m().exists():
            time.sleep(int(sleep_time))
        #If the video duration is very short such like 1s, the video player will disappear when the following statement wants to check it 
#         if self.waitForWidgetToAppear(self.GoogleDefaultCamera().camera_page_video_player(False),"video play view"):
#             time.sleep(int(sleep_time))
#         else:
#             self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
#             assert False,"can't play video"
        #if self.GoogleDefaultCamera().camera_page_video_player(False).exists():
        #    time.sleep(int(sleep_time))
        #else :
        #    assert False,"can't play video"

    def getCameraResolution(self, cameraType=constant.CAMERA_TYPE_BACK, founcType="video"):
        self.GoogleDefaultCamera().camera_Reso_quality_setting_page_set(True, cameraType, founcType).click()
        mList=[]
        for i in range(10):
            if self.d(resourceId = "android:id/text1",index=i).exists:
                mList.append(self.d(resourceId = "android:id/text1",index=i).info["text"])
            else:
                break
        return mList

    def getScreenshotAndPullToHost(self,file_name,host_path,base_path="/sdcard/Pictures/"):
        cmd1 = "adb shell screencap %s%s " %(base_path, file_name)
        cmd2 = "adb pull %s%s %s" %(base_path, file_name, host_path)
        time.sleep(1)
        os.system(cmd1)
        time.sleep(2)
        os.system(cmd2)
        time.sleep(2)

    def switch_timer(self,timer="timer is off"):
        if self.GoogleDefaultCamera().camera_page_three_dots(False).exists():
            self.GoogleDefaultCamera().camera_page_three_dots().click()
            time.sleep(1)
        if self.d(descriptionContains = "timer is off") and \
            self.GoogleDefaultCamera().camera_page_camera_countdown_toggle_button(False).exists:
            if timer == "3s":
                self.GoogleDefaultCamera().camera_page_camera_countdown_toggle_button().click()
            elif timer == "10s":
                self.GoogleDefaultCamera().camera_page_camera_countdown_toggle_button().click()
                self.GoogleDefaultCamera().camera_page_camera_countdown_toggle_button().click()
        elif self.d(descriptionContains = "3 seconds") and \
                self.GoogleDefaultCamera().camera_page_camera_countdown_toggle_button(False).exists:
            if timer == "10s":
                self.GoogleDefaultCamera().camera_page_camera_countdown_toggle_button().click()
            elif timer == "timer is off":
                self.GoogleDefaultCamera().camera_page_camera_countdown_toggle_button().click()
                self.GoogleDefaultCamera().camera_page_camera_countdown_toggle_button().click()
        elif self.d(descriptionContains = "10 seconds") and \
                self.GoogleDefaultCamera().camera_page_camera_countdown_toggle_button(False).exists:
            if timer == "3s":
                self.GoogleDefaultCamera().camera_page_camera_countdown_toggle_button().click()
                self.GoogleDefaultCamera().camera_page_camera_countdown_toggle_button().click()
            elif timer == "timer is off":
                self.GoogleDefaultCamera().camera_page_camera_countdown_toggle_button().click()

    def disableOrEnableCamera(self,type,pkgName=PACKAGE_NAME_CAMERA):
        if type == "Enable":
            os.system("adb shell pm enable %s" %pkgName)
            cmd = 'pm list packages -e; echo $?'
        elif type == "Disable":
            os.system("adb shell pm disable %s" %pkgName)
            cmd = 'pm list packages -d; echo $?'
        time.sleep(2)
        apps = g_common_obj.adb_cmd_capture_msg(cmd).count(pkgName)
        if apps > 0:
            return True
        else:
            return False

#     def findCameraApp(self,type):
#         isExists = False
#         self.dut.press.home()
#         self.d(description="Apps").click()
#         for i in range(2):
#             self.swipe_screen("right")
#             if self.d(textContains = "Camera").exists:
#                 isExists = True
#                 break
#         return isExists

    def isShutterBtnExists(self):
        if self.GoogleDefaultCamera().camera_page_shutter(False).exists():
            return True
        else:
            return False

    def isGoogleMusicBtnExists(self):
        if self.GoogleMusic().google_music_page_pause_button().exists:
            return True
        else:
            return False

    def isCameraPageTextExists(self,text):
        if self.GoogleDefaultCamera().camera_page_text(text).exists():
            return True
        else:
            return False

    def isVideoRecordingExists(self):
        if self.GoogleDefaultCamera().camera_page_video_recording(False).exists():
            return True
        else:
            return False

    def isVideoViewExists(self):
        if self.GoogleDefaultCamera().camera_video_view(False).exists():
            return True
        else:
            return False

    def isPhotoEditBtnExists(self):
        if self.GoogleDefaultCamera().camera_photo_show_page_edite_button(False).exists():
            return True
        else:
            return False

    def isCameraOptionBtnExists(self):
        if self.GoogleDefaultCamera().camera_page_three_dots(False).exists():
            return True
        else:
            return False

    def press_power(self):
        self.d.press.power()

    def press_home(self):
        self.d.press.home()

    def openCameraFromLockScreenIcon(self):
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        if self.GoogleDefaultCamera().camera_icon_in_lock_screen().exists():
            bounds=self.d(resourceId="com.android.systemui:id/camera_button").bounds
            icon_top = bounds.get("top")
            icon_bottom = bounds.get("bottom")
            icon_left = bounds.get("left")
            icon_right = bounds.get("right")
            #self.logger.debug(icon_top,icon_bottom,icon_left,icon_right)
            sx = icon_left+int((icon_right-icon_left)/2)
            sy = icon_top + int((icon_bottom-icon_top)/2)
            self.d.swipe(sx,sy, x/2, sy, steps =100)
            self.chooseCamera()
        wait_time = 0
        while wait_time < 20:
            self.chooseCamera()
            self.checkCameraAccessDevicesLocation()
            self.checkNextExists()
            if self.isShutterBtnExists():
                break
            else:
                time.sleep(1)
            wait_time += 1
        if not self.isShutterBtnExists():
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False,"open camera from lock screen icon fail"

    def chooseCamera(self):
        if self.d(text=self.PACKAGE_NAME_CAMERA).exists:
            self.d(text=self.PACKAGE_NAME_CAMERA).click()
        if self.d(text = "Always").exists:
            self.d(text = "Always").click()

    def checkNextExists(self):
        if self.d(text="NEXT").exists:
            self.d(text="NEXT").click()

    def checkScreenLock(self):
        if not self.GoogleDefaultCamera().lock_screen_icon().exists():
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "screen does not lock"

    def checkShutterButtonEnabled(self):
        enabled = self.d(resourceId="com.android.camera2:id/shutter_button").info["enabled"]
        self.logger.debug(str(enabled))
        if enabled :
            return True
        else:
            return False

    def rotationScreen(self,orientation ="portrait"):
        width = self.d.info["displayWidth"]
        height = self.d.info["displayHeight"]
        displayOrientation = self.d.info["displayRotation"]
        if orientation == "landscape":
            if width < height and displayOrientation > 0 :
                self.d.orientation = "n"
            elif width < height and displayOrientation == 0:
                self.d.orientation = "l"
        else :
            if width > height and displayOrientation > 0 :
                self.d.orientation = "n"
            elif width > height and displayOrientation == 0 :
                self.d.orientation = "r"
        self.d.freeze_rotation()

    def waitForWidgetToAppear(self, widget, desc = "", timeout = 10):
        time_left = 0
        start = time.clock()
        while time_left < timeout:
            try:
                if widget==None or widget.exists()==False or widget.exists()==None:
                    time.sleep(0.5)
                else:
                    return True
            except TypeError:
                if widget==None or widget.exists==False or widget.exists==None:
                    time.sleep(0.5)
                else:
                    return True
            time_left = time.clock()-start
#             self.logger.debug("time.clock=" + str(time.clock()))
#             self.logger.debug("start=" + str(start))
            self.logger.debug("time_left=" + str(time_left))
            self.judge_if_camera_crash()
        try:
            if widget==None or widget.exists()==False or widget.exists()==None:
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
                assert False, "Wait "+str(desc) + " to appear timeout in "+str(timeout)+" s"
        except TypeError:
            if widget==None or widget.exists==False or widget.exists==None:
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
                assert False, "Wait "+str(desc) + " to appear timeout in "+str(timeout)+" s"

    def waitForWidgetToDisappear(self, widget, desc = "", timeout = 10):
        time_left = 0
        start = time.clock()
        while time_left < timeout:
            try:
                if widget!=None and widget.exists()==True:
                    time.sleep(0.5)
                else:
                    return True
            except TypeError:
                if widget!=None and widget.exists==True:
                    time.sleep(0.5)
                else:
                    return True
            time_left = time.clock()-start
        try:
            if widget!=None and widget.exists()==True:
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
                assert False, "Wait widget "+str(desc) + " to disappear timeout in "+str(timeout)+" s"
        except TypeError:
            if widget!=None and widget.exists==True:
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
                assert False, "Wait widget "+str(desc) + " to disappear timeout in "+str(timeout)+" s"

#     def deleteLegacyFilesAfterEnterCamera(self):
#         while True:
#             self.swipe_screen("left")
#             has_meadia = False
#             while self.GoogleDefaultCamera().camera_photo_show_page_delete_button().exists():
#                 if self.d(resourceId="com.android.camera2:id/filmstrip_bottom_control_delete").enabled:
#                     has_meadia = True
#                     self.GoogleDefaultCamera().camera_photo_show_page_delete_button().click()
#                 else:
#                     time.sleep(0.5)
#             if has_meadia==False:
#                 break

#     def checkImageLandscapeToPortrait(self,orientation1,orientation2):
#         self.swipe_screen("left")
#         self.click_screen_center()
#         self.rotationScreen(orientation1)
#         for i in range(2):
#             x = self.d.info["displayWidth"]
# #             y = self.d.info["displayHeight"]
#             if self.d(className="android.widget.ImageView").exists:
#                 bounds = self.d(className="android.widget.ImageView").bounds
#                 img_left = bounds.get("left")
#                 img_right = bounds.get("right") 
#                 #self.logger.debug(img_left+img_right+ x)
#                 orientation = self.d.info["displayRotation"]
#                 time.sleep(3)
#                 range_x = x/2 -(img_right-img_left)/2 + 50
#                 if img_left <0 or img_left > range_x:
#                     self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
#                 assert img_left >=0 and img_left <= range_x, "check image display error"
#                 if img_right < 0 or img_right > x:
#                     self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
#                 assert img_right >= 0 and img_right <= x, "check image display error"
#                 self.rotationScreen(orientation2)
#                 self.logger.debug("rotation screen to " + str(orientation2)) 
#                 time.sleep(2)
#             else:
#                 self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
#                 assert False, " image not find"

    def close_lock_screen(self,tag=True):
    # delete clock screen
        self.unlockScreen()
        g_common_obj.launch_app_am(self.PACKAGE_NAME_SETTING, self.ACTIVITY_NAME_SETTING)
        time.sleep(3)
        self.checkSettingCrash()
        if not self.d(text="Security").exists:
            self.d(scrollable=True).scroll.vert.to(textContains="Security")
        self.d(text="Security").click()
        self.waitForWidgetToAppear(self.GoogleDefaultCamera().camera_page_text("Screen lock"),"screen lock")
        if self.d(text="Screen lock").exists:
            self.d(text="Screen lock").click()
        if tag:
            if self.d(text="None").exists:
                self.d(text="None").click()
            else:
                assert False, " unlock screen fail"
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        else:
            if self.d(text="Swipe").exists:
                self.d(text="Swipe").click()
            else:
                assert False, "lock screen fail"
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        self.press_home()

    def switch_Manual_exposure_on_off(self,tag="ON"):
        self.GoogleDefaultCamera().camera_setting_page_advanced_button().click()
        time.sleep(2)
        if not self.GoogleDefaultCamera().camera_setting_page_switch(tag).exists():
            self.GoogleDefaultCamera().camera_setting_advanced_page_switch_Widget().click()
        if self.GoogleDefaultCamera().camera_setting_advanced_page_switch_Widget().exists()==False:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.GoogleDefaultCamera().camera_setting_advanced_page_switch_Widget().exists(), 'enter camera_setting_advanced_page failed'

    def removeDeivceFile(self):
        g_common_obj.adb_cmd_capture_msg("rm -rf /mnt/sdcard/DCIM/Camera/*")
        if self.file_exists("/mnt/sdcard/DCIM/big_file", "DEVICE"):
            self.logger.debug("big file exists")
            g_common_obj.adb_cmd('rm -rf /mnt/sdcard/DCIM/big_file')
        if self.file_exists("/mnt/sdcard/bigfile", "DEVICE"):
            self.logger.debug("bigfile exists") 
            g_common_obj.adb_cmd('rm -rf /mnt/sdcard/bigfile')
        self.refesh_camera_dir()

    def removeDeivceCameraFolder(self):
        g_common_obj.adb_cmd_capture_msg("rm -rf /mnt/sdcard/DCIM/Camera")
        self.refesh_camera_dir()

    def check_exposure_value(self, text):
        time.sleep(3)
        if self.GoogleDefaultCamera().camera_page_three_dots(False).exists():
            self.GoogleDefaultCamera().camera_page_three_dots().click()
        time.sleep(3)
        self.GoogleDefaultCamera().camera_page_camera_exposure_button().click()
        if not self.GoogleDefaultCamera().camera_page_camera_exposure_button_value(text).exists():
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "check exposure value faile"

    def getGPSLocation(self):
        if self.AndroidGPS().gps_page_network_location().exists:
            self.AndroidGPS().gps_page_network_location().click()
        if self.AndroidGPS().gps_page_setting_text("Settings").exists:
            self.AndroidGPS().gps_page_setting_text("Settings").click.wait()
            if self.AndroidGPS().gps_setting_switch().info["text"] !="ON":
                self.AndroidGPS().gps_setting_switch().click.wait()
            if self.AndroidGPS().gps_page_setting_text("Mode").exists:
                self.AndroidGPS().gps_page_setting_text("Mode").click.wait()
                if self.AndroidGPS().gps_page_setting_text("High accuracy").exists:
                    self.AndroidGPS().gps_page_setting_text("High accuracy").click()
                try:
                    self.AndroidGPS().gps_page_setting_text("Agree").click.wait()
                except:
                    self.logger.debug("without prompt")
                if not self.AndroidGPS().gps_page_setting_text("Mode").exists:
                    self.press_back()
                if self.AndroidGPS().gps_setting_switch().info["text"] =="ON":
                    self.AndroidGPS().gps_setting_switch().click.wait()
                    self.AndroidGPS().gps_setting_switch().click.wait()
                    try:
                        self.AndroidGPS().gps_page_setting_text("Agree").click.wait()
                    except:
                            self.logger.debug("without prompt")
                assert self.AndroidGPS().gps_setting_switch().info["text"] =="ON", \
                        "gps switch is off"
            for i in range(5):
                if not self.AndroidGPS().gps_page_network_location().exists:
                    self.press_back()
            if self.AndroidGPS().gps_page_network_location().exists:
                self.AndroidGPS().gps_page_network_location().click()
            time.sleep(4)
            for j in range(5):
                if self.AndroidGPS().gps_page_setting_text("Settings").exists:
                    self.AndroidGPS().gps_page_setting_text("Settings").click.wait()
                    if self.AndroidGPS().gps_setting_switch().info["text"] =="ON":
                        self.AndroidGPS().gps_setting_switch().click.wait()
                        self.AndroidGPS().gps_setting_switch().click.wait()
                    try:
                        self.AndroidGPS().gps_page_setting_text("Agree").click.wait()
                    except:
                            self.logger.debug("without prompt")
                    assert self.AndroidGPS().gps_setting_switch().info["text"] =="ON", \
                        "gps switch is off"
                    for m in range(5):
                        if not self.AndroidGPS().gps_page_network_location().exists:
                            self.press_back()
                    if self.AndroidGPS().gps_page_network_location().exists:
                        self.AndroidGPS().gps_page_network_location().click.wait()
            if self.AndroidGPS().gps_page_setting_text("Settings").exists:
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
                assert False,"Fail to get location,maybe wifi not available!"
        longitute = self.AndroidGPS().gps_page_longitute_value().info["text"]
        latitude = self.AndroidGPS().gps_page_latitude_value().info["text"]
        if longitute != "unknown" and latitude != "unknown":
            longitute = longitute.split('.')
            latitude = latitude.split('.')
        return longitute[0],latitude[0]

    def press_volume(self,tag="up"):
        '''
        change volume
        '''
        if tag == "up":
            g_common_obj.adb_cmd_capture_msg("input keyevent 24")
            self.logger.debug("volume up")
        elif tag == "down":
            g_common_obj.adb_cmd_capture_msg("input keyevent 25")
            self.logger.debug("volume down")

    def quicklyClickShutterButton(self,x,y):
        self.d.click(x, y)

    def closeFullScreenView(self):
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        self.d.swipe(x/2, 10, x/2, y/2 + 100, steps=50)

    def isFlashButtonExists(self):
        if self.GoogleDefaultCamera().camera_page_three_dots(False).exists():
            self.GoogleDefaultCamera().camera_page_three_dots().click()
        time.sleep(3)
        if self.GoogleDefaultCamera().camera_page_flash_toggle_button().exists():
            return True
        else:
            return False

    def changeFlashButtonStatus(self,status):
        if self.GoogleDefaultCamera().camera_page_three_dots(False).exists():
            self.GoogleDefaultCamera().camera_page_three_dots().click()
        self.waitForWidgetToAppear(self.GoogleDefaultCamera().camera_page_flash_toggle_button(),"flash button")
        while True:
            desc = self.d(resourceId="com.android.camera2:id/flash_toggle_button").contentDescription
            if status =="on" and desc == "Flash on":
                break
            if status == "off" and desc == "Flash off":
                break
            if status == "auto" and desc == "Flash auto":
                break
            self.GoogleDefaultCamera().camera_page_flash_toggle_button().click()

    def switch_manual_exposure(self):
        self.GoogleDefaultCamera().camera_setting_page_advanced_button().click()
        time.sleep(2)
        if self.GoogleDefaultCamera().camera_setting_advanced_page_switch_Widget(False).exists():
            if self.d(resourceId = "android:id/switchWidget").info["checked"] == False:
                self.GoogleDefaultCamera().camera_setting_advanced_page_switch_Widget().click()
        if self.GoogleDefaultCamera().camera_setting_advanced_page_switch_Widget().exists()==False:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.GoogleDefaultCamera().camera_setting_advanced_page_switch_Widget().exists(), 'enter camera_setting_advanced_page failed'

    def change_exposure(self, text):
        time.sleep(3)
        if self.d(resourceId="com.android.camera2:id/three_dots").exists:
            self.GoogleDefaultCamera().camera_page_three_dots().click()
        #time.sleep(3)
        self.waitForWidgetToAppear(self.GoogleDefaultCamera().camera_page_camera_exposure_button(False), "exposure button")
        self.GoogleDefaultCamera().camera_page_camera_exposure_button().click()
        self.waitForWidgetToAppear(self.GoogleDefaultCamera().camera_exposure_page_exposure(False), "exposure button")
        self.GoogleDefaultCamera().camera_exposure_page_exposure().child_by_description("Exposure Compensation " + str(text), className="android.widget.ImageButton").click()

    def counter_time(self):
        time.sleep(1)
        if self.d(resourceId="com.android.camera2:id/three_dots").exists:
            self.GoogleDefaultCamera().camera_page_three_dots().click()
        time.sleep(1)
        self.GoogleDefaultCamera().camera_page_camera_countdown_toggle_button().click()
        self.GoogleDefaultCamera().camera_page_camera_countdown_toggle_button().click()

    def stop_counter_time(self):
        time.sleep(1)
        t = time.time()
        bool_status = False
        while (time.time() - t < 10):
            if self.d(resourceId="com.android.camera2:id/remaining_seconds").exists:
                self.d(resourceId="com.android.camera2:id/shutter_cancel_button").click()
                bool_status = True
                return
        if bool_status==False:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert bool_status, "remaining_seconds not exists"

    def unitConversion(self, str):
        if "G" in str:
            return float(str.replace("G", ""))*1024
        if "M" in str:
            return float(str.replace("M", ""))

    def get_sdcard_memory(self, path="/mnt/shell/emulated"):
        results = g_common_obj.adb_cmd_capture_msg(" df " + path)
        results = results.split("\n")
        reList = None
        for result in results:
            if path in result:
                reList = result.replace("\n", "").replace("\r", "").replace("/mnt/shell/emulated", "").split(" ")
        result = []
        for re in reList:
            if len(re) > 0:
                reF = self.unitConversion(re)
                if reF:
                    result.append(reF)
        if len(result) == 3:
            total = result[0]
            used = result[1]
            free = result[2] 
            self.logger.debug("SDCard total "+str(total)+ "used "+ str(used)+ "free "+ str(free))
            return total, used, free
        else:
            return None

    def click_menu_camera_prime_page(self):
        self.d(resourceId="com.android.camera2:id/menu").click()

    def press_back(self):
        self.d.press.back()

    #enter setting menu
    def click_camera_menu_setting_vert(self):
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        if not self.GoogleDefaultCamera().camera_page_settings_button(False).exists():
            self.d.swipe(0,y/2,x/2,y/2,steps=50)
            time.sleep(3)
        self.GoogleDefaultCamera().camera_page_settings_button().click()
        time.sleep(2)
        if self.GoogleDefaultCamera().camera_setting_page_resolution_quality_button().exists()==False:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.GoogleDefaultCamera().camera_setting_page_resolution_quality_button().exists(), \
            "Enter Setting page failed"

    #click Resolution & quality
    def enter_camera_setting_video_quality(self, ssim=0.75):
        self.logger.debug("enter_camera_setting_video_quality")
        self.waitForWidgetToAppear(self.GoogleDefaultCamera().camera_setting_page_resolution_quality_button(False), "Resolution & quality")
        self.GoogleDefaultCamera().camera_setting_page_resolution_quality_button().click()
        time.sleep(3)

    def set_camera_setting_save_location(self,on_off= "ON"): 
        self.logger.debug("enter_camera_setting_save location")
        if on_off == "OFF":
            if self.GoogleDefaultCamera().camera_setting_page_save_location_switch_button2().exists():
                self.GoogleDefaultCamera().camera_setting_page_save_location_switch_button().click()
        else:
            if not self.GoogleDefaultCamera().camera_setting_page_save_location_switch_button2().exists():
                self.GoogleDefaultCamera().camera_setting_page_save_location_switch_button().click()

    # set resolution & quality
    def set_camera_setting_video_quality(self, resolution, cameraType=constant.CAMERA_TYPE_BACK, founcType="video", flag=1):
        self.logger.debug("set_camera_setting_video_quality")
        self.GoogleDefaultCamera().camera_Reso_quality_setting_page_set(True, cameraType, founcType).click()
        if self.GoogleDefaultCamera().camera_setting_alert_page_cancel().exists():
            if not self.d(textContains=resolution).exists:
                self.logger.debug("there is no resolution as " + str(resolution))
                if flag != 1:
                    self.GoogleDefaultCamera().camera_setting_alert_page_cancel().click()
                    return
                else:
                    self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
                    assert False, "there is no resolution as " + resolution
            self.GoogleDefaultCamera().camera_setting_alert_page_select_dialog(resolution).click()
            time.sleep(3)
            if self.GoogleDefaultCamera().camera_setting_alert_page_cancel(False).exists():
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert not self.GoogleDefaultCamera().camera_setting_alert_page_cancel(False).exists(), \
                "set Resolution " + resolution + " failed"
        else:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "Enter Resolution Setting page failed"

    def set_camera_setting_panorama_resolution(self, resolution): 
        self.logger.debug("set_camera_setting_panorama_resolution")
        if not self.d(text="Panorama resolution").exists:
            return
        self.GoogleDefaultCamera().camera_Reso_quality_setting_page_panorama_resolution().click()
        if self.GoogleDefaultCamera().camera_setting_alert_page_cancel().exists():
            if not self.GoogleDefaultCamera().camera_setting_alert_page_select_dialog(resolution).exists():
                return False, "there is no resolution as " + resolution
            self.GoogleDefaultCamera().camera_setting_alert_page_select_dialog(resolution).click()
            time.sleep(3)
            if self.GoogleDefaultCamera().camera_setting_alert_page_cancel(False).exists():
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert not self.GoogleDefaultCamera().camera_setting_alert_page_cancel(False).exists(), \
                "set Resolution " + resolution + " failed"
        else:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "Enter Resolution Setting page failed"

    def set_camera_setting_image_quality(self, resolution):
        self.logger.debug("set_camera_setting_image_quality")
        if self.GoogleDefaultCamera().camera_Reso_quality_setting_page_front_image_quality(False).exists():
            self.GoogleDefaultCamera().camera_Reso_quality_setting_page_front_image_quality().click()
            if self.GoogleDefaultCamera().camera_setting_alert_page_cancel().exists():
                if not self.GoogleDefaultCamera().camera_setting_alert_page_select_dialog(resolution).exists():
                    return False, "there is no resolution as " + resolution
                self.GoogleDefaultCamera().camera_setting_alert_page_select_dialog(resolution).click()
                time.sleep(3)
                if self.GoogleDefaultCamera().camera_setting_alert_page_cancel(False).exists():
                    self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
                assert not self.GoogleDefaultCamera().camera_setting_alert_page_cancel(False).exists(), \
                    "set Resolution " + resolution + " failed"
            else:
                assert False, "Enter Resolution Setting page failed"

    # Capture Video from initial page
    def capture_video_camera_initial_page(self, recordTime, times=1):
        if not self.GoogleDefaultCamera().camera_page_three_dots(False).exists():
            self.enter_camera_from_home()
        recordTime = int(recordTime)
        sleep_time = 1
        if recordTime > 10:
            sleep_time = recordTime / 10
        if sleep_time > 30:
            sleep_time = 30
        for t in range(times):
            self.logger.debug("times is:" + str(t))
            time.sleep(3)
            self.logger.debug("Recording video times is :[" + str(t) + "]")
            self.d(resourceId="com.android.camera2:id/shutter_button").click()
            time.sleep(3)
            try:
                self.d(resourceId="com.android.camera2:id/recording_time").text
            except:
                time.sleep(3)
                self.logger.debug("not find time ,video record not started")
            try:
                self.d(resourceId="com.android.camera2:id/recording_time").text
            except:
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
                assert False, "video record not started"
            startTime = time.time()
            while True:
                time.sleep(sleep_time)
                text = None
                notFoundTime = 0
                for i in range(10):
                    try:
                        text = self.d(resourceId="com.android.camera2:id/recording_time").text
                    except:
                        self.logger.debug(str(i) + "times not find recording_time,try again")
                        time.sleep(10)
                    if text:
                        self.logger.debug("recording_time text is "+str(text))
                        break
                if not text:
                    total, used, free = self.get_sdcard_memory()
                    if free >= (total/30):
                        self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
                    assert free < (total/30), "not find recording_time"
                    return
                text = text.split(":")
                if len(text) == 2:
                    text = int(text[0]) * 60 + int(text[1])
                elif len(text) == 3:
                    text = int(text[0]) * 3600 + int(text[1]) * 60 + int(text[2])
                self.logger.debug("times is :"+str(t)+" recordTime text is :"+ str(text)+ " recordTime is "+str(recordTime))
                if (text >= recordTime) or (time.time() - startTime > recordTime):
                    self.d(resourceId="com.android.camera2:id/shutter_button").click()
                    break

    def playback_after_captured_video(self, playTime):
        if not self.GoogleDefaultCamera().camera_page_three_dots(False).exists():
            self.enter_camera_from_home()
        # swipe scrren
        self.d.swipe(550, 200, 200, 200)
        time.sleep(2)
        # enter
        if self.d(resourceId="com.android.camera2:id/play_button").exists:
            self.d(resourceId="com.android.camera2:id/play_button").click()
            time.sleep(3)
            if self.d(packageName="com.google.android.apps.photos").exists:
                self.d.swipe(self.x/2, 10, self.x/2, self.y/2 + 100, steps=50)
#             if self.d(resourceId="com.android.camera2:id/play_button").exists:
#                 self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert not self.d(resourceId="com.android.camera2:id/play_button").exists, \
                        "play back failed,it has not startd"
            timeNow = time.time()
            while time.time() - timeNow <= int(playTime):
                print "play time :", time.time() - timeNow
                assert not self.d(textContains="error").exists, "playback error!"
                time.sleep(5)
        else:
#             self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "play back failed,swipe scrren,not find play botton"

    def playback_after_captured_video_AdjustVolume(self, playTime):
        # swipe scrren
        self.d.swipe(700, 200, 200, 200)
        time.sleep(2)
        # enter
        if self.d(resourceId="com.android.camera2:id/play_button").exists:
            self.d(resourceId="com.android.camera2:id/play_button").click()
            time.sleep(1)
            if self.d(resourceId="com.android.camera2:id/play_button").exists:
                return False, "play back failed,it has not startd"
            timeNow = time.time()
            while time.time() - timeNow <= playTime:
                for i in range(20):
                    self.d.press("volume_up")
                for i in range(20):
                    self.d.press("volume_down")
                if self.d(resourceId="com.android.camera2:id/play_button").exists:
                    break
        else:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "play back failed,swipe scrren,not find play botton"

    def camera_video_capture_pause(self, click_times):
        if not self.GoogleDefaultCamera().camera_page_three_dots(False).exists():
            self.enter_camera_from_home()
        click_times = int(click_times)
        for index in range(0, click_times):
            self.GoogleDefaultCamera().camera_page_shutter().click()
        self.GoogleDefaultCamera().camera_page_three_dots().click()
        if self.GoogleDefaultCamera().camera_page_camera_toggle_button(False).exists()==False:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.GoogleDefaultCamera().camera_page_camera_toggle_button(False).exists(), "Start/stop capturing video for multiple times quickly, click 3 dots, there is no response"

    def play_back_browsing(self):
        open_web_page = ChromeImpl(self.d)
        open_web_page.launch_by_am()
        open_web_page.open_new_tab()
        open_web_page.open_website("http://ie.microsoft.com/testdrive/graphics/videoformatsupport/default.html")

    def enter_photo_plus_from_camera(self):
        self.d.swipe(400, 200, 100, 200)
        time.sleep(3)
        if self.d(resourceId="com.android.camera2:id/filmstrip_view").exists:
            pass
        else:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "enter_photo_plus_from_camera failed"

    def export_photos_captured(self):
        if "Windows" in platform.platform():
            path = constant.LogPath+"\\"+constant.caseName
        else:
            path = constant.LogPath+"/"+constant.caseName

        os.system("mkdir " + path)
        os.system("adb pull /sdcard/DCIM " + path)

    def judge_if_camera_crash(self):
        if self.d(text='Unfortunately, Camera has stopped.').exists:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert not self.d(text='Unfortunately, Camera has stopped.').exists, "Unfortunately, Camera has stopped."
        if self.d(textContains="Camera isn't responding.").exists:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert not self.d(textContains="Camera isn't responding.").exists, "Camera isn't responding."
        if self.d(textContains="Can't connect to the camera.").exists:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert not self.d(textContains="Can't connect to the camera.").exists, "Can't connect to the camera."

    def judge_if_setting_crash(self):
        if self.d(text='Unfortunately, Settings has stopped.').exists:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert not self.d(text='Unfortunately, Settings has stopped.').exists, "Unfortunately, Settings has stopped."

    def checkSettingCrash(self):
        if self.d(text='Unfortunately, Settings has stopped.').exists:
            g_common_obj.launch_app_am(self.PACKAGE_NAME_SETTING, self.ACTIVITY_NAME_SETTING)
            time_left = 0
            start = time.time()
            while time_left < 10:
                if self.GoogleDefaultCamera().camera_page_text("Settings").exists():
                    break
                time_left = time.time() - start
                time.sleep(0.5)
            if not self.GoogleDefaultCamera().camera_page_text("Settings").exists():
                g_common_obj.reboot_device()
                self.wait_boot_completed()
                self.unlockScreen()
                g_common_obj.launch_app_am(self.PACKAGE_NAME_SETTING, self.ACTIVITY_NAME_SETTING)
                time_left2 = 0
                start2 = time.time()
                while time_left2 < 10:
                    if self.GoogleDefaultCamera().camera_page_text("Settings").exists():
                        break
                    time_left2 = time.time() - start2
                    time.sleep(0.5)
                if not self.GoogleDefaultCamera().camera_page_text("Settings").exists():
                    self.judge_if_setting_crash()

    def isCameraPreviewPageVideoViewExists(self):
        if self.d(resourceId="com.android.camera2:id/video_view").exists:
            return True
        else:
            return False

    def wait_boot_completed(self, timeout=1000):
        ''' 
        wait Android boot_completed
        args: timeout -- optional timeout in second, default 180s
        '''
        count = 0
        sleep_time = 5
        while count < timeout:
            prop_val = g_common_obj.adb_cmd_capture_msg('getprop sys.boot_completed')
            if '1' in prop_val:
                print 'boot_completed'
                return
            count += sleep_time
            time.sleep(sleep_time)
        raise Exception('%ds timeout waiting for boot_completed' % timeout)

    def get_playing_image_screenshot_when_review(self):
        '''
        when review the picture, take screenshot and crop the image
        '''
        image = otcImage.getWidgetImage(self.d, self.d(className="android.widget.ImageView"))
        return image

    def get_playing_video_screenshot_when_review(self):
        '''
        when playing the video, take screenshot and crop the image
        '''
        image = otcImage.getWidgetImage(self.d, self.d(className="android.widget.VideoView"))
        return image

    def get_playing_video_screenshot_when_review_for_L(self):
        '''
        when playing the video, take screenshot and crop the image
        '''
        image = otcImage.getWidgetImage(self.d, self.d(className="iax"))
        return image

    def get_playing_video_screenshot_when_review_for_M(self):
        '''
        when playing the video, take screenshot and crop the image
        '''
        image = otcImage.getWidgetImage(self.d, \
                self.d(resourceId="com.google.android.apps.photos:id/photos_videoplayer_video_surface_view"))
        return image

    def refesh_camera_dir(self):
        """
        Send media_mount broadcase to notify the direcorty change.
        """
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED  --ez read-only false -d file://sdcard"
        g_common_obj.adb_cmd(cmd)

    def go_into_photoes_app(self):
        """
         go in to photos app
         @precondition: camera is launched, and in preview status
        """
        self.swipe_screen("left")
        self.GoogleDefaultCamera().camera_pic_preview_photos_icon().click()

    def check_shutterbutton_clickable(self):
        '''
        check if the shutter button clickable
        '''
        return self.GoogleDefaultCamera().camera_page_shutter().getWidgetEnabled()

    def fill_storage(self, headspace=0):
        """
        fill the sdcard to nearly full
        """
        #delete the former created file if exists.
        self.remove_big_file()
        free_space = self.get_sdcard_memory()[2]
        self.logger.debug("Now will create %s MB to fill storage space" % str(free_space))
        cmd = 'dd if=/dev/zero of=/sdcard/dcim/big_file bs=1048576 count=%s' % (int(free_space)-int(headspace))
        g_common_obj.adb_cmd(cmd, 1800)
        self.logger.debug( "free space = " + str(self.get_sdcard_memory()[2]))
        self.refesh_camera_dir()

    def remove_big_file(self):
        """
        remove the big_file created by function: fill_storage
        """
        #delete the former created file if exists.
        if self.file_exists("/sdcard/dcim/big_file", "DEVICE"):
            g_common_obj.adb_cmd('rm -r /sdcard/dcim/big_file')
        self.refesh_camera_dir()

    def launch_and_initialize_barcode_apk(self):
        """
        start and initiaize the barcode apk
        """
        self.unlockScreen()
        time.sleep(2)
        self.d.press.home()
        g_common_obj.launch_app_am('com.google.zxing.android',\
                                    'com.google.zxing.client.android.CaptureActivity')
        time.sleep(2)
        done_button = self.d(resourceId='com.google.zxing.android:id/done_button')
        if done_button:
            done_button.click()

    def verify_barcod_work_correct(self):
        '''
        @precondition: the barcode app is launched successfull
        '''
        self.logger.debug('check the ui widget in barcode screen')
        if self.d(text="Place a barcode inside the viewfinder rectangle to scan it.").exists==False:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.d(text="Place a barcode inside the viewfinder rectangle to scan it.").exists,\
            'failed to launch barcode app'
        log_output = g_common_obj.adb_cmd_capture_msg("logcat -t 20", 15)
        self.logger.debug('check logcat, to see the output of barcode app')
        assert 'decode....' in log_output, 'not normal log print in logcat'
        assert 'width:' in log_output, 'not normal log print in logcat'
        assert 'width:' in log_output, 'not normal log print in logcat'
        
    def get_camera_setting_supported_parameter(self):
        '''
        @precondition: already in the camera's Resolution & quality page
        @return: a map return all the supported setting with category (Camera/Video/Photo/LensBlur)
        this function would get all the settings, the camera app support
        '''
        self.logger.debug("should pop up the value list to select")
        map_resolution = {}
        for view in self.d(className="android.widget.RelativeLayout"):
            #setting items
            key = ""
            titel_view = view.child(resourceId="android:id/title")
            if titel_view:
                key = titel_view.info["text"]
            else:
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
                assert False, "No supported resolution listed"
            self.logger.debug(str(key))
            value = []
            self.logger.debug('click to see supported values')
            view.click()
            time.sleep(1)
            for item in self.d(className="android.widget.CheckedTextView"):
                value.append(item.info["text"])
            map_resolution[key] = value
            self.press_back()
        self.logger.debug(str(map_resolution))
        return map_resolution
    
    def get_camera_setting_supported_parameter_with_action(self, _map):
        '''
        very find each setting
        '''
        for view in self.d(className="android.widget.RelativeLayout"):
            title_view = view.child(resourceId="android:id/title")
            key = ""
            if title_view:
                key = title_view.info["text"]
            else:
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
                assert False, "no supported resolution listed"
            self.logger.debug('now check, '+str(key))
            value = _map[key]
            time.sleep(1)
            for txt in value:
                view.click()
                self.logger.debug(str(txt))
                self.d(text=txt).click()
            #   selected = txt
                self.logger.debug('exit camera by press home')
                self.press_home()
                self.enter_camera_from_home()
                self.click_camera_menu_setting_vert()
                self.enter_camera_setting_video_quality()
                if self.d(text=txt).exists == False:
                    self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
                assert self.d(text=txt).exists == True, "The camera parameter is not consistent after camera reopen"
            #   self.d(text=key).click()
            #if self.d(text='Cancel').exists:
            #    self.d(text='Cancel').click()

    class GoogleDefaultCamera:

        def __init__(self):
            self.d = g_common_obj.get_device()
            self.node = OTCNode(self.d)

        def __setNode__(self, expectation=True, **kwargs):
            self.node.findNode(expectation, **kwargs)
            return self.node

        def camera_page_shutter(self, expectation=False):
            '''
            the shutter button
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/shutter_button")

        def camera_page_three_dots(self, expectation=False):
            '''
            the three dots
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/three_dots")

        def camera_page_photo_sphere(self, expectation=True):
            '''
            text is  Photo Sphere
            '''
            return self.__setNode__(expectation, text="Photo Sphere")

        def camera_page_panorama(self, expectation=True):
            '''
            text is Panorama
            '''
            return self.__setNode__(expectation, text="Panorama")

        def camera_page_lens_blur(self, expectation=True):
            '''
            text is Lens Blur
            '''
            return self.__setNode__(expectation, text="Lens Blur")

        def camera_page_camera(self, expectation=True):
            '''
            text is Camera
            '''
            return self.__setNode__(expectation, text="Camera")

        def camera_page_photo_video(self, expectation=True):
            '''
            text is Video
            '''
            return self.__setNode__(expectation, text="Video")

        def camera_page_grid_lines_toggle_button(self, expectation=True):
            '''
            grid lines toggle button,turn on/off grid lines
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/grid_lines_toggle_button")

        def camera_page_camera_toggle_button(self, expectation=False):
            '''
            camera toggle button,change front/back camera
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/camera_toggle_button")

        def camera_page_camera_countdown_toggle_button(self, expectation=True):
            '''
            count down toggle button,turn on/off countdown
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/countdown_toggle_button")

        def camera_page_camera_exposure_button(self, expectation=True):
            '''
            exposure_button, turn on/off exposure function
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/exposure_button")

        def camera_exposure_page_exposure(self, expectation=True):
            '''
            exposure list: -2 -1 0 +1 +2
            need turn on the exposure function first
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/mode_options_exposure")

        def camera_page_settings_button(self, expectation=True):
            '''
            the setting button, to enter the camera setting page
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/settings_button")

        # panorama capure page
        def camera_page_panorama_done_button(self, expectation=False):
            '''
            panorama capture page, done button
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/done_button")

        def camera_page_panorama_retake_button(self, expectation=True):
            '''
            panorama capture page, retake button
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/retake_button")

        def camera_page_panorama_cancel_button(self, expectation=True):
            '''
            panorama capture page, cancel button
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/cancel_button")

        def camera_setting_page_resolution_quality_button(self, expectation=True):
            '''
            text is Resolution & quality in the camera setting page
            '''
            return self.__setNode__(expectation, text="Resolution & quality")

        def camera_setting_page_save_location_button(self, expectation=True):
            '''
            text is Save location in the camera setting page
            '''
            return self.__setNode__(expectation, text="Save location")

        def camera_setting_page_save_location_switch_button(self, expectation=True):
            '''
            the Save location switch button in the camera setting page
            '''
            return self.__setNode__(expectation, resourceId="android:id/switchWidget")

        def camera_setting_page_advanced_button(self, expectation=True):
            '''
            text is Advanced in the camera setting page
            '''
            return self.__setNode__(expectation, text="Advanced")

        def camera_setting_advanced_page_switch_Widget(self, expectation=True):
            '''
            the Manual exposure switch button in the Advanced page in the camera setting
            '''
            return self.__setNode__(expectation, resourceId="android:id/switchWidget")

        def camera_setting_page_help_button(self, expectation=True):
            '''
            text is Help & feedback in the camera setting page
            '''
            return self.__setNode__(expectation, text="Help & feedback")

        def camera_Reso_quality_setting_page_set(self, expectation=True, cameraType="Back camera", founcType="video"):
            '''
            set front/back camera/video resolution in Resolution & quality page in camera setting
            include:
            Back camera photo
            Front camera photo
            Back camera video
            Front camera video
            '''
            return self.__setNode__(expectation, text=cameraType + " " + founcType)

        def camera_Reso_quality_setting_page_panorama_resolution(self, expectation=True):
            '''
            set panorama resolution in Resolution & quality page in camera setting
            '''
            return self.__setNode__(expectation, text="Panorama resolution")

        def camera_Reso_quality_setting_page_front_image_quality(self, expectation=True):
            '''
            set lens blur image quality in Resolution & quality page in camera setting
            '''
            return self.__setNode__(expectation, text="Image quality")

        def camera_setting_alert_page_title(self, expectation=True):
            '''
            '''
            return self.__setNode__(expectation, resourceId="android:id/alertTitle")

        def camera_setting_alert_page_select_dialog_listview(self, expectation=True):
            '''
            '''
            return self.__setNode__(expectation, resourceId="android:id/select_dialog_listview")

        def camera_setting_alert_page_select_dialog(self, text, expectation=True):
            '''
            '''
            return self.__setNode__(expectation, textContains=text)

        def camera_setting_alert_page_cancel(self, expectation=True):
            '''
            '''
            return self.__setNode__(expectation, text="Cancel")

        def camera_wizard_page_confirm_button(self, expectation=False):
            '''
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/confirm_button")

        def camera_photo_sphere_wizard_page_next_button(self, expectation=False):
            '''
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/next_button")

        def camera_photo_show_page_share_button(self, expectation=False):
            '''
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/filmstrip_bottom_control_share")

        def camera_photo_show_page_edite_button(self, expectation=False):
            '''
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/filmstrip_bottom_control_edit")

        def camera_photo_show_page_delete_button(self, expectation=False):
            '''
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/filmstrip_bottom_control_delete")

        def camera_photo_show_page_delete_filmstrip_view(self, expectation=False):
            '''
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/filmstrip_view")

        def camera_video_capture_page_recording_time(self, expectation=False):
            '''
            the time when video recording
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/recording_time")

        def camera_mode_options_overlay(self , expectation=False):
            '''
            the time when video recording
            '''
            return self.__setNode__(expectation , resourceId="com.android.camera2:id/mode_options_overlay")

        def camera_icon_in_lock_screen(self, expectation = False):
            '''
            camera lock screen icon
            '''
            return self.__setNode__(expectation ,resourceId="com.android.systemui:id/camera_button")

        def lock_screen_icon(self, expectation = False):
            '''
            camera lock screen icon
            '''
            return self.__setNode__(expectation ,resourceId = "com.android.systemui:id/lock_icon")

        def camera_setting_page_save_location_switch_button2(self, expectation=False):
            '''
            the Save location switch button in the camera setting page
            '''
            return self.__setNode__(expectation, text="ON")

        def camera_page_camera_exposure_button_value(self, text, expectation=False):
            '''
            exposure value
            '''
            desc = "Exposure Compensation " + text
            return self.__setNode__(expectation, description=desc)

        def camera_setting_page_switch(self, text, expectation=False):
            '''
            setting switch
            '''
            return self.__setNode__(expectation, text = text)

        def camera_page_video_player(self,expectation=False):
            '''
            video player
            '''
            return self.__setNode__(expectation, resourceId = "com.google.android.apps.plus:id/videoplayer")

        def camera_page_video_player_button(self,expectation=False):
            '''
            video player button
            '''
            return self.__setNode__(expectation, resourceId = "com.android.camera2:id/play_button")

        def camera_pic_preview_photos_icon(self, expectation=False):
            '''
            the photos icon, click it would launch the photoes app
            '''
            return self.__setNode__(expectation, description="Photos")

        def camera_page_video_recording(self, expectation=False):
            '''
            the photos icon, click it would launch the photoes app
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/labels")


        def camera_page_text(self, text, expectation=False):
            '''
            can't play video
            '''
            return self.__setNode__(expectation, text = text)

        def camera_page_flash_toggle_button(self,expectation=False):
            '''
            flash toggle button
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/flash_toggle_button")

        def camera_video_view(self,expectation=False):
            '''
            camera video view
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/video_view")

        def setting_more_options(self,expectation=False):
            '''
            setting apps more option button
            '''
            return self.__setNode__(expectation, description="More options")

        def camera_video_view_for_m(self,expectation=False):
            '''
            camera video play view
            '''
            return self.__setNode__(expectation, resourceId="com.google.android.apps.photos:id/photo_hashtag_fragment_container")
        
        def panoramaModeWidget(self, expectation=True, mode="Horizontal panorama"):
            '''
            support modes are: Horizontal panorama/Vertical panorama/Wide angle/Fisheye
            
            '''
            return self.__setNode__(expectation, className="android.widget.ImageButton").child_by_description(mode)
            #return self.d(className="android.widget.ImageButton").child_by_description(mode)
        
        def photoDoneWidget(self, expectation=True):
            '''
            panorama done button
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/done_button")
        
        def photoRetakeWidget(self, expectation=True):
            '''
            panorama retake button
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/retake_button")
        
        def photoCancelWidget(self, expectation=True):
            '''
            panorama cancel button
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/cancel_button")
        
        def photoProcessingBar(self, expectation=True):
            '''
            panorama photo processing bar
            sphere photo processing bar
            '''
            return self.__setNode__(expectation, resourceId="com.android.camera2:id/bottom_session_progress_text")
        
    class AndroidGPS:

        def __init__(self):
            self.d = g_common_obj.get_device()

        def gps_page_network_location(self):
            '''
            network location button
            '''
            return self.d(resourceId = "com.javapapers.android.androidgps:id/btnNWShowLocation")

        def gps_page_longitute_value(self):
            '''
            gps longitute value
            '''
            return self.d(resourceId = "com.javapapers.android.androidgps:id/TextView04")

        def gps_page_latitude_value(self):
            '''
            gps latitude value
            '''
            return self.d(resourceId = "com.javapapers.android.androidgps:id/TextView02")

        def gps_page_setting_text(self,text):
            '''
            gps setting value
            '''
            return self.d(text=text)

        def gps_setting_switch(self):
            '''
            gps setting switch widget
            '''
            return self.d(resourceId="com.android.settings:id/switch_widget")

    class Alarm:
        def __init__(self):
            self.d = g_common_obj.get_device()

        def alarm_page_time(self):
            '''
            alarm edit text
            '''
            return self.d(className="android.widget.EditText")

        def alarm_page_title(self):
            '''
            alarm title
            '''
            return self.d(text="OtcAlarm")

        def alarm_prompt_message(self):
            '''
            alarm prompt message
            '''
            return self.d(text="OTC Alarm is triggered!")

        def alarm_prompt_dismiss_btn(self):
            '''
            alarm prompt dismiss button
            '''
            return self.d(text="Dismiss")

        def alarm_prompt_snooze_btn(self):
            '''
            alarm prompt snooze button
            '''
            return self.d(text="Snooze")

    class GoogleMusic:
        def __init__(self):
            self.d = g_common_obj.get_device()

        def google_music_page_pause_button(self):
            '''
            google music pause button
            '''
            return self.d(resourceId = "com.google.android.music:id/pause")

    class Settings:
        def __init__(self):
            self.d = g_common_obj.get_device()

        def set_wallpaper_item_label(self):
            '''
            wallpaper item label
            '''
            return self.d(resourceId="com.google.android.googlequicksearchbox:id/wallpaper_item_label")

        def set_wallpaper_image(self):
            '''
            wallpaper image
            '''
            return self.d(resourceId="com.google.android.googlequicksearchbox:id/wallpaper_image")

        def set_wallpaper_icon_mime(self):
            '''
            wallpaper icon mime
            '''
            return self.d(resourceId="com.android.documentsui:id/icon_mime")

        def set_wallpaper_date(self):
            '''
            wallpaper date
            '''
            return self.d(resourceId="com.android.documentsui:id/date")


        def set_wallpaper_text(self, _text):
            '''
            wallpaper text
            '''
            return self.d(text=_text)
#camera_impl = CameraImpl()
