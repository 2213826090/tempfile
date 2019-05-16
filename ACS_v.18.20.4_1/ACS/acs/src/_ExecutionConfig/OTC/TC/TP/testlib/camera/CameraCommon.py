'''
Created on Oct 26, 2015

@author: shankang
'''
import os
import time
import subprocess
from testlib.camera.camera_log import CameraLogger
from testlib.util.common import g_common_obj
from testlib.camera.checkImage import CheckImage
from testlib.camera.checkVideo import CheckVideo
from testlib.camera.cameratestbase import CameraTestBase


class CameraCommon(CameraTestBase):

    PACKAGE_NAME_SETTING = "com.android.settings"
    ACTIVITY_NAME_SETTING = ".Settings"

    PACKAGE_NAME_ALARM = "videoplayer.app.instrument.otc.intel.com.otcalarm"
    ACTIVITY_NAME_ALARM = "otc.intel.com.otcalarm.MainActivity"

    PACKAGE_NAME_GPS = "com.javapapers.android.androidgps"
    ACTIVITY_NAME_GPS = ".AndroidLocationActivity"

    PACKAGE_NAME_NOTIFICATION = "com.example.android.support.wearable.notifications"
    ACTIVITY_NAME_NOTIFICATION = "com.test.android.notifications.MainActivity"

    PACKAGE_NAME_THIRDPARTAPP = "com.example.mycamera"
    ACTIVITY_NAME_THIRDPARTAPP = ".MainActivity"

    DEFAULT_CONFIG_FILE = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), "..", "Multimedia_Camera", 'tests.testplan.camera.conf')

    def __init__(self, cfg=None):
        self.d = g_common_obj.get_device()
        self.logger = CameraLogger.instance()
        self.checkImage = CheckImage()
        self.video = CheckVideo()
        self.x = self.d.info["displayWidth"]
        self.y = self.d.info["displayHeight"]
        self.camera_dir = "/sdcard/DCIM/Camera/"
        self.host_path = self.getTmpDir()
        self.makefileTime = 4
        self.waitStartAppTime = 20

    class CommonWidget:

        def __init__(self):
            self.d = g_common_obj.get_device()

        def activityDescription(self):
            '''
            camera activity description
            '''
            return self.d(resourceId="com.android.systemui:id/activity_description")

        def cameraIconInLockScreen(self):
            '''
            camera lock screen icon
            '''
            return self.d(resourceId="com.android.systemui:id/camera_button")

        def clockIcon(self):
            '''
            lock screen clock icon
            '''
            return self.d(resourceId="com.android.systemui:id/clock_view")

        def lockScreenIcon(self):
            '''
            camera lock screen icon
            '''
            return self.d(resourceId="com.android.systemui:id/lock_icon")

        def musicPauseBtn(self):
            '''
            google music pause button
            '''
            return self.d(resourceId="com.google.android.music:id/pause")

        def notificationAction(self):
            '''
            camera notification action
            '''
            return self.d(resourceId="android:id/action0")

        def lockScreenPasswordEntry(self):
            '''
            password entry
            '''
            return self.d(resourceId="com.android.systemui:id/passwordEntry")

        def passwordEntry(self):
            '''
            password entry
            '''
            return self.d(resourceId="com.android.settings:id/password_entry")

        def pinEntry(self):
            '''
            pin entry
            '''
            return self.d(resourceId="com.android.systemui:id/pinEntry")

        def setWallpaperItemLabel(self):
            '''
            wallpaper item label
            '''
            return self.d(resourceId="com.google.android.googlequicksearchbox:id/wallpaper_item_label")

        def setWallpaperImage(self):
            '''
            wallpaper image
            '''
            return self.d(resourceId="com.google.android.googlequicksearchbox:id/wallpaper_image")

        def setWallpaperIconThumb(self):
            '''
            wallpaper icon thumb
            '''
            return self.d(resourceId="com.android.documentsui:id/icon_thumb")

        def setWallpaperDate(self):
            '''
            wallpaper date
            '''
            return self.d(resourceId="com.android.documentsui:id/date")

        def taskViewThumbnail(self):
            '''
            task view thumbnail
            '''
            return self.d(resourceId="com.android.systemui:id/task_view_thumbnail")

        def text(self, _text):
            '''
            page text
            '''
            return self.d(textContains=_text)

        def yesRemoveBtn(self):
            '''
            yes remove button
            '''
            return self.d(resourceId="android:id/button1")

    class Alarm:
        def __init__(self):
            self.d = g_common_obj.get_device()

        def alarmPageTime(self):
            '''
            alarm edit text
            '''
            return self.d(className="android.widget.EditText")

        def alarmPageTitle(self):
            '''
            alarm title
            '''
            return self.d(text="OtcAlarm")

        def alarmPromptMessage(self):
            '''
            alarm prompt message
            '''
            return self.d(resourceId="android:id/message")

        def alarmPromptDismissBtn(self):
            '''
            alarm prompt dismiss button
            '''
            return self.d(resourceId="android:id/button2")

        def alarmPromptSnoozeBtn(self):
            '''
            alarm prompt snooze button
            '''
            return self.d(resourceId="android:id/button1")

    class AndroidGPS:

        def __init__(self):
            self.d = g_common_obj.get_device()

        def gpsNetworkLocation(self):
            '''
            network location button
            '''
            return self.d(resourceId="com.javapapers.android.androidgps:id/btnNWShowLocation")

        def gpsLongituteValue(self):
            '''
            gps longitute value
            '''
            return self.d(resourceId="com.javapapers.android.androidgps:id/TextView04")

        def gpsLatitudeValue(self):
            '''
            gps latitude value
            '''
            return self.d(resourceId="com.javapapers.android.androidgps:id/TextView02")

        def gpsSettingText(self, _text):
            '''
            gps setting value
            '''
            return self.d(text=_text)

        def gpsSettingTextContains(self, _text):
            '''
            gps setting value
            '''
            return self.d(textContains=_text)

        def gpsSettingSwitch(self):
            '''
            gps setting switch widget
            '''
            return self.d(resourceId="com.android.settings:id/switch_widget")

#======================================================

    def adbPullFile(self, device_path, host_path, is_check_file=False, file_size=None):
        """ adb pull file from device to host """
        self.logger.debug("Adb pull file from device: %s to host: %s" % (device_path, host_path))
        cmd = 'adb pull %s %s' % (device_path, host_path)
        self.logger.debug("Pull file: %s" % cmd)
        os.popen(cmd)
        if is_check_file:
            exist = self.fileExists(host_path, location="HOST")
            if file_size:
                self.verifyFileSize(host_path, file_size)
            if exist == False:
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert exist, "Expect file exist, Actual file not exist"
        self.logger.debug("Pull file successfully")
        time.sleep(3)

    def adbPushFile(self, host_path, device_path, is_check_file=False):
        """ adb push file from host to device """
        self.logger.debug("Adb push file from host: %s to device: %s" % \
        (host_path, device_path))
        cmd = 'adb push %s %s' % (host_path, device_path)
        self.logger.debug("Push file: %s" % cmd)
        os.popen(cmd)
        if is_check_file:
            exist = self.fileExists(device_path, location="DEVICE")
            if exist == False:
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert exist, "Expect file exist, Actual file not exist"
        self.logger.debug("Push file successfully")

    def alarmDismissOrSnooze(self, trigger="Dismiss"):
        if self.Alarm().alarmPromptDismissBtn().exists or \
            self.Alarm().alarmPromptSnoozeBtn().exists:
            if trigger == "Dismiss":
                self.Alarm().alarmPromptDismissBtn().click()
                return True
            else:
                self.Alarm().alarmPromptSnoozeBtn().click()
                return True
        return False

    def changeLanguageInSetting(self, languageAndInput, languageItem, language):
        self.unlockScreen()
        g_common_obj.launch_app_am(self.PACKAGE_NAME_SETTING, self.ACTIVITY_NAME_SETTING)
        time.sleep(2)
        if not self.d(text=languageAndInput).exists:
            self.d(scrollable=True).scroll.vert.to(text=languageAndInput)
        self.d(text=languageAndInput).click.wait()
        if self.d(text=languageItem).exists == False:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.d(text=languageItem).exists, "language not exists"
        self.d(text=languageItem).click.wait()
        if not self.d(text=language).exists:
            self.d(scrollable=True).scroll.vert.to(text=language)
        self.d(text=language).click.wait()
        time.sleep(2)
        if self.d(text=languageItem).exists:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert not self.d(text=languageItem).exists, "language not change"
        self.logger.debug("change language to %s successfully" % (language))
        self.pressBack(3)

    def changeLanguageInSettingForN(self, languageAndInput, languageItem, language, country=""):
        self.unlockScreen()
        g_common_obj.launch_app_am(self.PACKAGE_NAME_SETTING, self.ACTIVITY_NAME_SETTING)
        time.sleep(2)
        if not self.d(text=languageAndInput).exists:
            self.d(scrollable=True).scroll.vert.to(text=languageAndInput)
        self.d(text=languageAndInput).click.wait()
        if self.d(text=languageItem).exists == False:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.d(text=languageItem).exists, "language not exists"
        self.d(text=languageItem).click.wait()
        if not self.d(textStartsWith=language).exists:
            if self.d(text="Add a language").exists:
                self.d(text="Add a language").click.wait()
            if not self.d(text=language).exists:
                self.d(scrollable=True).scroll.vert.to(text=language)
                self.d(text=language).click.wait()
            if self.d(text=country).exists:
                self.d(text=country).click.wait()
        if self.d(resourceId="com.android.settings:id/dragHandle").exists:
            bounds = self.d(textStartsWith=language).right(resourceId="com.android.settings:id/dragHandle").bounds
            top = bounds.get("top")
            left = bounds.get("left")
            self.d.swipe(left + 50, top + 50, left + 50, top - 100, steps=200)
        time.sleep(2)
        if self.d(text=languageItem).exists:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert not self.d(text=languageItem).exists, "language not change"
        self.logger.debug("change language to %s successfully" % (language))
        self.pressBack(3)

    def checkCameraCrash(self):
        if self.d(textContains='stop').exists:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert not self.d(textContains='stop').exists, "Unfortunately, app has stopped."
        if self.d(textContains='error').exists:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert not self.d(textContains='error').exists, "Unfortunately, app has error."
        if self.d(textContains="n't").exists:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert not self.d(textContains="n't").exists, "Unfortunately, app has problem."
        if self.d(textContains="not").exists:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert not self.d(textContains="not").exists, "Unfortunately, app has problem."
#         if self.d(text='Unfortunately, Camera has stopped.').exists:
#             self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
#         assert not self.d(text='Unfortunately, Camera has stopped.').exists, "Unfortunately, Camera has stopped."
#         if self.d(textContains="Camera isn't responding").exists:
#             self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
#         assert not self.d(textContains="Camera isn't responding").exists, "Camera isn't responding."
#         if self.d(textContains="Can't connect to the camera.").exists:
#             self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
#         assert not self.d(textContains="Can't connect to the camera.").exists, "Can't connect to the camera."
#         if self.d(textContains="Google App isn't responding.").exists:
#             self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
#         assert not self.d(textContains="Google App isn't responding.").exists, "Google App isn't responding."
#         if self.d(text='Camera error').exists:
#             self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
#         assert not self.d(text='Camera error').exists, "Camera error"
#         self.logger.debug("camera running smoothly...")
#         if self.d(text='Unfortunately, Photos has stopped.').exists:
#             self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
#         assert not self.d(text='Unfortunately, Photos has stopped.').exists, "Unfortunately, Photos has stopped."
#         if self.d(textContains="ReferenceCamera2 isn't responding").exists:
#             self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
#         assert not self.d(textContains="ReferenceCamera2 isn't responding").exists, "ReferenceCamera2 isn't responding"

    def checkCameraAccessDevicesLocation(self):
        if self.d(text="Allow").exists:
            self.d(text="Allow").click()
        if self.d(text="ALLOW").exists:
            self.d(text="ALLOW").click()

    def checkDeviceFile(self):
        if g_common_obj.adb_cmd_capture_msg("ls %s" % self.camera_dir):
            return True
        else:
            return False

    def checkFile(self, path):
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

    def checkFileCorrupt(self, mediaFileCount=1):
        file_name_list = self.pullFileAndCheckFileNumber(self.camera_dir, self.host_path, mediaFileCount)
        self.info = ""
        for i in range(len(file_name_list)):
            self.path = self.host_path + "/" + file_name_list[i]
            for j in range(10):
                if not self.checkFile(self.path):
                    self.adbPullFile(self.camera_dir + file_name_list[i], self.host_path)
                    time.sleep(2)
                else:
                    self.logger.debug(str(file_name_list[i]) + " exists")
                    break
                time.sleep(2)
            str_name = file_name_list[i].split('.')
            suffix = str_name[1]
            if not suffix:
                assert False, "file name without the suffix"
            if suffix == "jpg":
                errMsg = self.checkImage.check_image_corrupt(self.path)
                if errMsg != "":
                    assert False, errMsg
                else:
                    self.info = self.getExifInfo(file_name_list[i], self.host_path + "/")
                    self.logger.debug("picture validation successful")
                continue
            elif suffix == "mp4" or suffix == "3gp":
                if self.video.check_video_corrupt(self.path):
                    self.info = self.getExifInfo(file_name_list[i], self.host_path + "/")
                    self.logger.debug("video validation successful")
                else:
                    self.logger.debug("video validation fails")
#             self.removeFile(self.host_path + "/*")
#         self.removeDeivceFile()
        return self.info, file_name_list[0]

    def checkEXIFInfo(self, image, name):
        error_msg = ""
        if image == None:
            assert False, "media file is null, maybe file is not generated"

        file_type = image.get("File Type")
        self.logger.debug("file type: %s" % file_type)
        if file_type == None or (file_type != "JPEG" and file_type != "MP4"):
                error_msg += "check file type fail,actual=%s,expected=JPEG or MP4; " % file_type

#         camera_model_name = image.get("Camera Model Name")
#         model_name = self.getDeviceModel()
#         self.logger.debug("Camera Model Name: %s" % camera_model_name)
#         self.logger.debug("prop Model Name: %s" % model_name)
#         if camera_model_name == None or camera_model_name != model_name:
#             error_msg += "check file mode name fail,actual=%s,expected=%s; " % (camera_model_name, model_name)

        orientation = image.get("Orientation")
        self.logger.debug("Orientation: %s" % orientation)
        px = self.x
        py = self.y
        if px < py:
            if orientation != None:
                if ("90" in orientation) or ("270" in orientation):
                    self.logger.debug("check orientation success")
                else:
                    error_msg += "check orientation fail,actual=%s,expected=(Rotate 90) or (Rotate 270); " % orientation
            else:
                error_msg += "check orientation fail,actual=%s,expected= Rotate 180; " % orientation
        else:
            if orientation != None:
                if ("180" in orientation) or ("0" in orientation):
                    self.logger.debug("check orientation success")
                else:
                    error_msg += "check orientation fail,actual=%s,expected= Rotate 180; " % orientation
            else:
                error_msg += "check orientation fail,actual=%s,expected= Rotate 180; " % orientation

#         exposure_mode = image.get("Exposure Mode")
#         self.logger.debug("Exposure Mode: %s" % exposure_mode)
#         if exposure_mode == None or exposure_mode != "Auto":
#             error_msg += "check exposure mode fail, actual=%s,expected=auto; " % exposure_mode

#         digital_zoom_Ratio = image.get("Digital Zoom Ratio")
#         self.logger.debug("Digital Zoom Ratio: %s" % digital_zoom_Ratio)
#         if digital_zoom_Ratio == None or digital_zoom_Ratio != "1":
#             error_msg += "check digital zoom ratio fail, actual=%s,expected=1; " % digital_zoom_Ratio

#         exposure_program = image.get("Exposure Program")
#         self.logger.debug("Exposure Program: %s" % exposure_program)
#         if exposure_program == None or exposure_program != "Program AE":
#             error_msg += "check exposure program fail, actual=%s,expected=Program AE; " % exposure_program

#         white_balance = image.get("White Balance")
#         self.logger.debug("White Balance: %s" % white_balance)
#         if white_balance == None or white_balance != "Auto":
#             error_msg += "check white balance fail,actual=%s,expected=Auto; " % white_balance

#         exposure_time = image.get("Exposure Time")
#         self.logger.debug("Exposure Time: %s" % exposure_time)
#         if exposure_time:
#             value = exposure_time.split('/')
#             mValue = float(value[0]) / float(value[1])
#             if mValue < (1 / 2500) or mValue > 1:
#                 error_msg += "check exposure time fail,actual=%s,expected=1/2500<=value<=1; " % mValue
#         else:
#             error_msg += "check exposure time check fail,actual=%s,expected=1/2500<=value<=1; " % exposure_time

#         flash = image.get("Flash")
#         self.logger.debug("flash: %s" % flash)
#         if flash != None:
#             if ("Auto" in flash) or ("No Flash" in flash) or ("Off" in flash) or ("On" in flash):
#                 self.logger.debug("check flash success")
#             else:
#                 error_msg += "check flash fail,actual=%s,expected=Auto or No Flash or Off or On; " % flash
#         else:
#             error_msg += "check flash fail,actual=%s,expected=Auto or No Flash; " % flash

#         iso = image.get("ISO")
#         self.logger.debug("ISO: %s" % iso)
#         if iso == None or int(iso) < 0 or int(iso) > 1600:
#             error_msg += "check iso fail, actual=%s,expected=0<value<1600; "

#         thumbnail_length = image.get("Thumbnail Length")
#         self.logger.debug("Thumbnail Length: %s" % thumbnail_length)
#         if thumbnail_length != None:
#             error_msg += "check Thumbnail Length fail,actual=%s,expected=None; " % thumbnail_length

#         thumbnail_offset = image.get("Thumbnail Offset")
#         self.logger.debug("Thumbnail Offset: %s" % thumbnail_offset)
#         if thumbnail_offset != None:
#             error_msg += "check thumbnail offset fail,actual=%s,expected=None; " % thumbnail_offset

        image_width = image.get("Image Width")
        image_height = image.get("Image Height")
        size = image.get("Image Size").split('x')
        self.logger.debug("Image Width: %s" % image_width)
        self.logger.debug("Image Height: %s" % image_height)
        self.logger.debug("Image Size: %s" % size)
        if image_width == None or image_width != size[0]:
            error_msg += "check width fail,actual=%s,expected=%s; " % (image_width, size[0])
        if image_height == None or image_height != size[1]:
            error_msg += "check height fail,actual=%s,expected=%s; " % (image_height, size[1])

#         thumbnail_image = image.get("Thumbnail Image")
#         self.logger.debug("Thumbnail Image: %s" % thumbnail_image)
#         if thumbnail_image != None:
#             error_msg += "check Thumbnail Image fail,actual=%s,expected=None; " % thumbnail_image

        date_time_original = image.get("File Modification Date/Time")
        self.logger.debug("File Modification Date/Time: %s" % date_time_original)
        year = time.strftime("%Y", time.localtime())
        if date_time_original == None or (year not in date_time_original):
            error_msg += "check Date/Time Original fail,actual=%s,expected=%s; " % (date_time_original, year)

        if error_msg != "":
            os.system("cp " + self.host_path + "/" + name + " " + g_common_obj.get_user_log_dir())
            self.assertTrue(False, error_msg)

    def checkGuide(self):
        """
        @summary: check the notifications after switch mode
        """
        for i in range(4):
            # check "YES" button
            if self.d(text='Yes').exists:
                self.d(text='Yes').click()
            # check "NEXT" button
            if self.d(text='NEXT').exists:
                self.d(text='NEXT').click()
            # check "OK, GOT IT" button
            if self.d(text='OK, GOT IT').exists:
                self.d(text='OK, GOT IT').click()
            # check "OK, GOT it" button
            if self.d(text='OK, Got it').exists:
                self.d(text='OK, Got it').click()
            if self.d(text="Allow").exists:
                self.d(text="Allow").click()
            if self.d(text="ALLOW").exists:
                self.d(text="ALLOW").click()

    def checkMemoryFull(self):
        total, used, free = self.getSdcardMemory()
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

    def checkScreenLock(self, timeout=10):
        time_left = 0
        start = time.time()
        while time_left < timeout:
            if self.CommonWidget().lockScreenIcon().exists or self.CommonWidget().clockIcon().exists:
                break
            else:
                time.sleep(1)
            time_left = time.time() - start
        if not (self.CommonWidget().lockScreenIcon().exists or self.CommonWidget().clockIcon().exists):
            self.assertTrue(False, "screen does not lock")

    def chooseCamera(self, package_name):
        if self.d(text=package_name).exists:
            self.d(text=package_name).click.wait()
#         if self.d(text = "Always").exists:
#             self.d(text = "Always").click.wait()
        if self.d(text="Just once").exists:
            self.d(text="Just once").click.wait()
        time.sleep(1)

    def clickScreenCenter(self):
        self.d.click(self.x / 2, self.y / 2)
        time.sleep(2)

    def clickBtn(self, x, y):
        self.d.click(x, y)

    def captureImageOrRecordingVia3rdPartApp(self, action="image"):
        self.CommonWidget().text(action).click.wait()
        if self.CommonWidget().text("Camera").exists:
            self.CommonWidget().text("Camera").click.wait()
        if self.d(text="Just once").exists:
            self.d(text="Just once").click.wait()

    def disableOrEnableCamera(self, tag, package):
        if tag == "Enable":
            os.system("adb shell pm enable %s" % package)
            cmd = 'pm list packages -e; echo $?'
        elif tag == "Disable":
            os.system("adb shell pm disable %s" % package)
            cmd = 'pm list packages -d; echo $?'
        time.sleep(2)
        apps = g_common_obj.adb_cmd_capture_msg(cmd).count(package)
        if apps > 0:
            return True
        else:
            return False

    def enterAppFromRecentTask(self, appName="Camera"):
        self.d.press.recent()
        time.sleep(1)
        self.logger.debug("enter recent task")
        if self.CommonWidget().text("Camera").exists:
            self.CommonWidget().text("Camera").click.wait()
            self.logger.debug("enter " + appName + " app from recent task successfully")
        else:
            self.logger.debug("the recent task list is not found " + appName)

    def fileExists(self, file_path, location):
        """ check file exist, support check on device or host """
        self.logger.debug("Check file: %s exist on %s or not" % \
        (file_path, location.upper()))
        exist = False
        if location.upper() == "DEVICE":
            cmd = 'ls %s; echo $?' % (file_path)
            result_list = g_common_obj.adb_cmd_capture_msg(cmd).split('\n')
            self.logger.debug(str(result_list))
        elif location.upper() == "HOST":
            cmd = 'ls %s 2>&1; echo $?' % (file_path)
            result_list = os.popen(cmd).readlines()
        else: 
            self.logger.debug("Can not support this location:[%s] information" % location)
            return False

        if len(result_list):
            ret = result_list[-1].rstrip()
            ret2 = result_list[0].rstrip()
            if ret2.count("No such file or directory") != 0:
                exist = False
                return exist
            if int(ret) == 0:
                exist = True
        self.logger.debug("File " + str(exist and "Exist" or "Not Exist"))
        return exist

    def fillStorage(self, headspace=0):
        """
        fill the sdcard to nearly full
        """
        # delete the former created file if exists.
        self.removeBigFile()
        free_space = self.getSdcardMemory()[2]
        self.logger.debug("Now will create %s MB to fill storage space" % str(free_space))
        cmd = 'dd if=/dev/zero of=/sdcard/DCIM/big_file bs=1048576 count=%s' % (int(free_space) - int(headspace))
        g_common_obj.adb_cmd(cmd, 1800)
        self.logger.debug("free space = " + str(self.getSdcardMemory()[2]))
        self.refreshCameraDir()

    def flipDownTheStatusBar(self):
        self.d.swipe(self.x / 2, 10, self.x / 2, self.y * 0.75, steps=100)
        time.sleep(0.5)

    def flashlightOperation(self):
        self.flipDownTheStatusBar()
        self.flipDownTheStatusBar()
        self.CommonWidget().text("Flashlight").click.wait()

    def rotateRegion(self, region, width, height, degree=90):
        """
        The region is a list [x1,y1,w1,h1]
        degree is only support 90 or 270
        Return value is a list [x,y,w,h]
        """
        x = x1 = region[0]
        y = y1 = region[1]
        w = w1 = region[2]
        h = h1 = region[3]
        if degree == 90:
            x = height - y1 - h1
            y = x1
            w = h1
            h = w1
        elif degree == 270:
            x = y1
            y = width - w1 - x1
            w = h1
            h = w1
        return [x, y, w, h]

    def getAndroidVersion(self):
        androidVersion = ""
        try:
            cmd = "adb shell getprop ro.build.version.release"
            r = os.popen(cmd)
            androidVersion = r.read().strip()
            if "7" in androidVersion:
                androidVersion = "N"
            r.close()
        except Exception as e:
            print e
        return androidVersion

    def getBounds(self, widget):
        bounds = widget.bounds
        # return [x,y,w,h]
        return [bounds.get("left"), bounds.get("top"), bounds.get("right") - bounds.get("left"), bounds.get("bottom") - bounds.get("top")]
        # return bounds.get("left"), bounds.get("top"), bounds.get("right"), bounds.get("bottom")

    def getExifInfo(self, filename, host_path):
        cmd = "exiftool %s%s" % (host_path, filename)
        self.logger.debug(cmd)
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
        exifinfo = output[0]
        lineList = exifinfo.split("\n")
        info = {}
        for line in lineList:
            mLine = line.split(":", 1)
            if len(mLine) <= 1:
                continue
            else:
                info.setdefault(mLine[0].strip(), mLine[1].strip())
        return info

    def getFileFromTempDir(self):
        '''
        get a file list under tmp dir
        '''
        tmp_dir = self.getTmpDir()
        file_list = os.listdir(tmp_dir)
        self.logger.debug('The files under %s are %s' % (tmp_dir, file_list))
        return file_list

    def getFileName(self, device_path):
        cmd = "adb shell ls " + str(device_path) + "*.*"
        list = []
        self.logger.debug("getFileName adb cmd:%s" % (cmd))
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
        self.logger.debug("getFileName adb cmd output:%s" % (str(output)))
        result = output[0][:-1]
        if result.find("No such file or directory") != -1:
            return []
        else:
            for r in result.split("\n"):
                fullpath = r.replace("\r", "")
                l = fullpath.split('/')
                name = l[len(l) - 1].strip()
                list.append(name)
            self.logger.debug("filename list length = %d" % (len(list)))
            return list

    def getFileSize(self, cmd):
        list = []
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
        return output[0].rstrip()

    def getDeviceModel(self):
        check_cmd = "getprop ro.product.model"
        msgs = g_common_obj.adb_cmd_capture_msg(check_cmd)
        print msgs
        assert len(msgs) != 0
        return msgs

    def getGPSLocation(self):
        self.logger.debug("getGPSLocation start")
        if self.AndroidGPS().gpsNetworkLocation().exists:
            self.AndroidGPS().gpsNetworkLocation().click.wait()
        time.sleep(1)
        if self.AndroidGPS().gpsSettingText("Settings").exists:
            self.logger.debug("getGPSLocation Settings string exists")
            self.AndroidGPS().gpsSettingText("Settings").click.wait()
            if self.CommonWidget().text("agree").exists:
                self.CommonWidget().text("agree").click.wait()
            if self.AndroidGPS().gpsSettingSwitch().info["text"] != "ON":
                self.AndroidGPS().gpsSettingSwitch().click.wait()
            if self.AndroidGPS().gpsSettingText("Mode").exists:
                self.logger.debug("getGPSLocation Mode string exists")
                self.AndroidGPS().gpsSettingText("Mode").click.wait()
                if self.AndroidGPS().gpsSettingTextContains("agree").exists:
                    self.logger.debug("1. Click 'agree' button")
                    self.AndroidGPS().gpsSettingTextContains("agree").click.wait()
                assert self.AndroidGPS().gpsSettingTextContains("accuracy").click(), "Can't find 'High accuracy' button"
                self.logger.debug("Click 'accuracy' button end")
                time.sleep(3)
                if self.AndroidGPS().gpsSettingText("Agree").exists:
                    self.logger.debug("2. Click 'Agree' button")
                    self.AndroidGPS().gpsSettingText("Agree").click.wait()
                if self.AndroidGPS().gpsSettingText("AGREE").exists:
                    self.logger.debug("2. Click 'AGREE' button")
                    self.AndroidGPS().gpsSettingText("AGREE").click.wait()
                    time.sleep(2)
                    if self.AndroidGPS().gpsSettingTextContains("yes").exists:
                        self.AndroidGPS().gpsSettingTextContains("yes").click.wait()
                for i in range(3):
                    if self.AndroidGPS().gpsSettingText("Mode").exists:
                        break
                    self.pressBack()
                    self.logger.debug("%s. Click 'back' button" % str(i))
                self.logger.debug("Exit click 'back' button loop")
                if self.CommonWidget().text("agree").exists:
                    self.CommonWidget().text("agree").click.wait()
                    self.logger.debug("3. Click 'agree' button")
                if self.CommonWidget().text("AGREE").exists:
                    self.CommonWidget().text("AGREE").click.wait()
                    self.logger.debug("3. Click 'AGREE' button")
                if self.AndroidGPS().gpsSettingSwitch().info["text"] == "ON":
                    self.AndroidGPS().gpsSettingSwitch().click.wait()
                    self.AndroidGPS().gpsSettingSwitch().click.wait()
                time.sleep(2)
                if self.CommonWidget().text("agree").exists:
                    self.CommonWidget().text("agree").click.wait()
                if self.CommonWidget().text("AGREE").exists:
                    self.CommonWidget().text("AGREE").click.wait()
                assert self.AndroidGPS().gpsSettingSwitch().info["text"] == "ON", \
                        "gps switch is off"
            for i in range(5):
                if not self.AndroidGPS().gpsNetworkLocation().exists:
                    self.pressBack()
            if self.AndroidGPS().gpsNetworkLocation().exists:
                self.AndroidGPS().gpsNetworkLocation().click()
            time.sleep(4)
            self.logger.debug("getGPSLocation loop start")
            for j in range(5):
                if self.AndroidGPS().gpsSettingText("Settings").exists:
                    self.logger.debug("getGPSLocation Settings string exists")
                    self.AndroidGPS().gpsSettingText("Settings").click.wait()
                    if self.CommonWidget().text("agree").exists:
                        self.CommonWidget().text("agree").click.wait()
                    if self.AndroidGPS().gpsSettingText("Agree").exists:
                        self.logger.debug("3. Click 'Agree' button")
                        self.AndroidGPS().gpsSettingText("Agree").click.wait()
                    if self.AndroidGPS().gpsSettingText("AGREE").exists:
                        self.logger.debug("3. Click 'AGREE' button")
                        self.AndroidGPS().gpsSettingText("AGREE").click.wait()
                    self.logger.debug("check gpsSettingSwitch on")
                    if self.AndroidGPS().gpsSettingSwitch().info["text"] == "ON":
                        self.AndroidGPS().gpsSettingSwitch().click.wait()
                        self.AndroidGPS().gpsSettingSwitch().click.wait()
                        time.sleep(2)
                    if self.CommonWidget().text("agree").exists:
                        self.CommonWidget().text("agree").click.wait()
                    if self.AndroidGPS().gpsSettingText("Agree").exists:
                        self.logger.debug("4. Click 'Agree' button")
                        self.AndroidGPS().gpsSettingText("Agree").click.wait()
                    if self.AndroidGPS().gpsSettingText("AGREE").exists:
                        self.logger.debug("4. Click 'AGREE' button")
                        self.AndroidGPS().gpsSettingText("AGREE").click.wait()
                    assert self.AndroidGPS().gpsSettingSwitch().info["text"] == "ON", \
                        "gps switch is off"
                    for m in range(5):
                        if not self.AndroidGPS().gpsNetworkLocation().exists:
                            self.pressBack()
                    if self.AndroidGPS().gpsNetworkLocation().exists:
                        self.AndroidGPS().gpsNetworkLocation().click.wait()
            if self.AndroidGPS().gpsSettingText("Settings").exists:
                self.assertTrue(False, "Fail to get location,maybe wifi not available!")
        longitute = self.AndroidGPS().gpsLongituteValue().info["text"]
        latitude = self.AndroidGPS().gpsLatitudeValue().info["text"]
        if longitute != "unknown" and latitude != "unknown":
            longitute = longitute.split('.')
            latitude = latitude.split('.')
        self.pressBack()
        self.logger.debug("get gps location successfully")
        return longitute[0], latitude[0]

    def getPlatform(self):
        cmd = "adb shell getprop ro.product.device"
        platform = os.popen(cmd).read()
        platform = platform.replace("\r\n", "", 1)
        self.logger.debug("getprop ro.produc.device=%s" % platform)
        if (platform != None) or (platform != ""):
            return platform.strip().lower()
        else:
            return platform.strip()

    def getPlatformName(self):
        import subprocess
        cmd = "ps aux|grep ACS.py"
        output = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE).communicate()
        boardName = ""
        for s in output[0][:-1].split("\n"):
            if s.find("python") != -1:
                s = s[s.find("OTC/CAMPAIGN"):]
                s = s[:s.find(".xml")]
                print s
                l = s.split(".")
                if s.find("reliability") != -1:
                    boardName = l[2].lower()
                else:
                    boardName = l[1].lower()
                break
        # print boardName
        return boardName

    def getScreenshotAndPullToHost(self, file_name, mhost_path, base_path="/sdcard/Pictures/"):
        cmd1 = "adb shell screencap %s%s " % (base_path, file_name)
        cmd2 = "adb pull %s%s %s" % (base_path, file_name, mhost_path)
        time.sleep(1)
        self.logger.debug("get screenshot start")
        os.system(cmd1)
        time.sleep(2)
        self.logger.debug("get screenshot end")
        self.logger.debug("pull screenshot to host start")
        os.system(cmd2)
        time.sleep(2)
        self.logger.debug("pull screenshot to host end")

    def getSdcardMemory(self, path="/mnt/shell/emulated"):
        sdcard_memory = []
        for i in range(2):
            results = g_common_obj.adb_cmd_capture_msg(" df " + path)
            results = results.split("\n")
            reList = None
            for result in results:
                if path in result:
                    reList = result.replace("\n", "").replace("\r", "").replace(path, "").replace("/dev/fuse", "").replace("%", "").split(" ")
                    print reList
            for re in reList:
                if len(re) > 0:
                    reF = self.unitConversion(re)
                    print reF
                    if reF:
                        sdcard_memory.append(reF)
                    else:
                        break
            path = "/storage/emulated"
        if len(sdcard_memory) >= 3:
            total = sdcard_memory[0]
            used = sdcard_memory[1]
            free = sdcard_memory[2]
            self.logger.debug("SDCard total " + str(total) + ", used " + str(used) + ", free " + str(free))
            return total, used, free
        else:
            return None

    def getTmpDir(self):
        home_path = os.path.expanduser("~")
        path = os.path.join(home_path, "tmp")
        if not os.access(path, os.R_OK | os.W_OK):
            if not os.path.exists(path):
                os.makedirs(path)
        path = os.path.join(path, "logs")
        if not os.path.exists(path):
            os.makedirs(path)
        return path

    def isWidgetExists(self, widget):
        if widget.exists:
            return True
        else:
            return False

    def isWidgetEnabled(self, widget):
        if widget.enabled:
            return True
        else:
            return False

    def launchAlarmApp(self):
        self.unlockScreen()
        g_common_obj.launch_app_am(self.PACKAGE_NAME_ALARM, self.ACTIVITY_NAME_ALARM)
        time.sleep(3)
        if not self.Alarm().alarmPageTitle().exists:
            self.assertTrue(False, "launch alarm app failed!")

    def launchGpsApp(self):
        self.unlockScreen()
        g_common_obj.launch_app_am(self.PACKAGE_NAME_GPS, self.ACTIVITY_NAME_GPS)
        time.sleep(2)

    def launchThirdPartApp(self):
        self.unlockScreen()
        g_common_obj.launch_app_am(self.PACKAGE_NAME_THIRDPARTAPP, self.ACTIVITY_NAME_THIRDPARTAPP)
        time.sleep(2)

    def launchWearableNotification(self):
        self.unlockScreen()
        g_common_obj.launch_app_am(self.PACKAGE_NAME_NOTIFICATION, self.ACTIVITY_NAME_NOTIFICATION)
        time.sleep(2)

    def launchUnlockAppToUnlockScreen(self):
        os.system("adb shell am start -n io.appium.unlock/.Unlock;adb shell am force-stop io.appium.unlock")
        time.sleep(3)
        self.logger.debug("unlock screen")

    def lockScreen(self, tag=True):
        self.openSettingsScreenLockPage()
        if tag:
            if self.CommonWidget().text("Swipe").exists:
                self.CommonWidget().text("Swipe").click.wait()
                self.logger.debug("enter setting and lock screen successfully")
            else:
                self.assertTrue(False, "lock screen fail")
        else:
            if self.CommonWidget().text("None").exists:
                self.CommonWidget().text("None").click.wait()
                self.logger.debug("enter setting and unlock screen successfully")
            else:
                self.assertTrue(False, "unlock screen fail")
        self.pressBack(3)
        time.sleep(1)

    def lockPIN(self):
        self.openSettingsScreenLockPage()
        if self.CommonWidget().text("PIN").exists:
            self.CommonWidget().text("PIN").click.wait()
        if self.CommonWidget().text("No thanks").exists:
            self.CommonWidget().text("No thanks").click.wait()
        if not self.CommonWidget().text("Choose").exists:
            self.CommonWidget().text("Continue").click.wait()
        self.CommonWidget().passwordEntry().set_text("1234")
        g_common_obj.adb_cmd("input keyevent 66")
        time.sleep(3)
        if self.CommonWidget().passwordEntry().exists:
            self.CommonWidget().passwordEntry().set_text("1234")
            g_common_obj.adb_cmd("input keyevent 66")
            time.sleep(1)
        if self.CommonWidget().text("Done").exists:
            self.CommonWidget().text("Done").click.wait()
        self.pressBack(3)
        self.logger.debug("PIN lock screen successfully")
        g_common_obj.stop_app_am(self.PACKAGE_NAME_SETTING)

    def unlockPIN(self):
        self.pressHome()
        time_left = 0
        start = time.time()
        while time_left < CameraCommon().waitStartAppTime:
            if self.CommonWidget().clockIcon().exists or self.CommonWidget().cameraIconInLockScreen().exists:
                self.d.swipe(self.x / 2, self.y - 10, self.x / 2, self.y / 2, steps=100)
                break
            time_left = time.time() - start
            time.sleep(0.5)
        time.sleep(3)
        if self.CommonWidget().pinEntry().exists:
            if self.CommonWidget().passwordEntry().exists:
                self.CommonWidget().passwordEntry().set_text("1234")
            else:
                self.d(resourceId="com.android.systemui:id/key1").click()
                self.d(resourceId="com.android.systemui:id/key2").click()
                self.d(resourceId="com.android.systemui:id/key3").click()
                self.d(resourceId="com.android.systemui:id/key4").click()
            g_common_obj.adb_cmd("input keyevent 66")
        self.openSettingsScreenLockPage()
        for i in range(5):
            if self.CommonWidget().text("Device security").exists:
                if self.CommonWidget().text("PIN").exists:
                    self.CommonWidget().text("PIN").click.wait()
            if self.CommonWidget().text("Confirm your PIN").exists:
                self.CommonWidget().passwordEntry().set_text("1234")
                g_common_obj.adb_cmd("input keyevent 66")
            time.sleep(1)
            if self.CommonWidget().text("Choose screen lock").exists:
                if self.CommonWidget().text("None").exists:
                    self.CommonWidget().text("None").click.wait()
                    if self.CommonWidget().yesRemoveBtn().exists:
                        self.CommonWidget().yesRemoveBtn().click.wait()
                    break
        self.pressBack(3)
        self.logger.debug("PIN unlock screen successfully")
        g_common_obj.stop_app_am(self.PACKAGE_NAME_SETTING)

    def setPasswd(self, password):
        time.sleep(1)
        if self.CommonWidget().passwordEntry().exists:
            self.CommonWidget().passwordEntry().set_text(password)
        else:
            g_common_obj.adb_cmd("input text %s" % password)

    def setLockScreenWithPasswd(self, tag=True):
        if self.d(resourceId="com.android.systemui:id/user_avatar").exists:
            # For O Car, need click user mode.
            self.d(resourceId="com.android.systemui:id/user_avatar").click()
        t_password = "qwer"
        self.openSettingsScreenLockPage()
        if tag:# lock
            if self.CommonWidget().text("Confirm").exists:
                self.setLockScreenWithPasswd(False)
                self.openSettingsScreenLockPage()
            self.CommonWidget().text("Password").click.wait()
            if self.CommonWidget().text("NO").exists:
                try:
                    self.d(text="NO").click()
                except:
                    self.CommonWidget().text("NO").click.wait()
            if self.CommonWidget().text("No thanks").exists:
                self.CommonWidget().text("No thanks").click.wait()
            if not self.CommonWidget().text("Choose").exists:
                if self.CommonWidget().text("Continue").exists:
                    self.CommonWidget().text("Continue").click.wait()
            self.setPasswd(t_password)
            g_common_obj.adb_cmd("input keyevent 66")
            time.sleep(3)
            self.setPasswd(t_password)
            g_common_obj.adb_cmd("input keyevent 66")
            time.sleep(1)
            if self.CommonWidget().text("Done").exists:
                self.CommonWidget().text("Done").click.wait()
            self.logger.debug("Password lock screen successfully")
        else:# unlock
            self.setPasswd(t_password)
            g_common_obj.adb_cmd("input keyevent 66")
            time.sleep(1)
            self.CommonWidget().text("None").click.wait()
            if self.CommonWidget().text("Yes, remove").exists:
                self.CommonWidget().text("Yes, remove").click.wait()
            if self.CommonWidget().text("YES, REMOVE").exists:
                self.CommonWidget().text("YES, REMOVE").click.wait()
            self.logger.debug("Password unlock screen successfully")
        self.pressBack(3)
        g_common_obj.stop_app_am(self.PACKAGE_NAME_SETTING)

    def openSettingsScreenLockPage(self):
        g_common_obj.stop_app_am(self.PACKAGE_NAME_SETTING)
        self.unlockScreen()
        g_common_obj.launch_app_am(self.PACKAGE_NAME_SETTING, self.ACTIVITY_NAME_SETTING)
        time.sleep(3)
        if not self.CommonWidget().text("Security").exists:
            self.d(scrollable=True).scroll.vert.to(textContains="Security")
        self.waitForWidgetToAppear(self.CommonWidget().text("Security"), "security text")
        self.CommonWidget().text("Security").click.wait()
        self.waitForWidgetToAppear(self.CommonWidget().text("Screen lock"), "screen lock")
        if self.CommonWidget().text("Screen lock").exists:
            self.CommonWidget().text("Screen lock").click.wait()

    def openCameraFromLockScreenIcon(self, package_name, camera):
        if self.CommonWidget().cameraIconInLockScreen().exists:
            iLeft, iTop, iRight, iBottom = self.getBounds(self.CommonWidget().cameraIconInLockScreen())
            sx = iLeft + iRight / 2
            sy = iTop + iBottom / 2
            self.d.swipe(sx, sy, self.x / 4, sy, steps=50)
        wait_time = 0
        while wait_time < self.waitStartAppTime:
            if wait_time < 2:
                self.chooseCamera(package_name)
                self.checkCameraAccessDevicesLocation()
                self.checkGuide()
            if camera.isShutterBtnExists():
                break
            else:
                time.sleep(1)
            wait_time += 1
            if wait_time == 4:
                self.d.press.recent()
                time.sleep(5)
                self.d.press.back()
        if not camera.isShutterBtnExists():
            self.assertTrue(False, "open camera from lock screen icon fail")

    def openNotificationAndCheckMusicStatus(self, status):
        self.d.swipe(self.x / 2, 3, self.x / 2, self.y / 5, steps=50)
        self.d.swipe(self.x / 2, 3, self.x / 2, self.y / 2, steps=50)
        self.logger.debug("open notification bar")
        start = time.time()
        success = False
        while time.time() - start < 10:
            if self.CommonWidget().notificationAction().exists:
                success = True
                break
            time.sleep(1)
        if (not self.CommonWidget().notificationAction().exists) or (not success):
            assert False, "open notification bar fail"
        if not self.d(resourceId="android:id/action0", description=status):
            assert False, "check music status fail, expect=play actual=%" % status
        if status == "Play":
            self.d(resourceId="android:id/action0", description=status).click.wait()
            self.logger.debug("click play button")
        self.pressBack()

    def openNotificationAndCheckReceiveEmail(self):
        self.flipDownTheStatusBar()
        self.flipDownTheStatusBar()
        if self.CommonWidget().text("Test email line").exists:
            self.logger.debug("email has received")
        else:
            self.logger.debug("did not receive an email")
            self.assertTrue(False, "did not receive an email")
        self.pressBack()

    def swipeCorner(self):
        if self.CommonWidget().cameraIconInLockScreen().exists:
            iLeft, iTop, iRight, iBottom = self.getBounds(self.CommonWidget().cameraIconInLockScreen())
            sx = iLeft + iRight / 2
            sy = iTop + iBottom / 2
            self.d.swipe(sx, sy, self.x / 4, sy, steps=50)

    def switchCamera(self, platform):
        if self.CommonWidget().text("Complete action using").exists and "3gr" in platform:
            time.sleep(2)
            if self.CommonWidget().text("com.arcsoft.camera2").exists:
                self.CommonWidget().text("com.arcsoft.camera2").click.wait()
            self.CommonWidget().text("just once").click.wait()

    def setSettingSleepTime(self, mTime):
        g_common_obj.stop_app_am(self.PACKAGE_NAME_SETTING)
        self.unlockScreen()
        g_common_obj.launch_app_am(self.PACKAGE_NAME_SETTING, self.ACTIVITY_NAME_SETTING)
        time.sleep(3)
        self.CommonWidget().text("Display").click.wait()
        self.CommonWidget().text("Sleep").click.wait()
        self.CommonWidget().text(mTime).click.wait()
        if not self.CommonWidget().text(mTime).exists:
            self.assertTrue(False, "set sleep time to " + mTime + " fail")
        self.pressBack(2)

    def setKeepAwakeOn(self, tag=True):
        self.unlockScreen()
        g_common_obj.launch_app_am(self.PACKAGE_NAME_SETTING, self.ACTIVITY_NAME_SETTING)
        time.sleep(3)
        if not self.CommonWidget().text("Developer options").exists:
            self.d(scrollable=True).scroll.vert.to(textContains="Developer options")
        self.CommonWidget().text("Developer options").click.wait()
        if self.CommonWidget().text("Developer options are not available for this user").exists:
            self.assertTrue(False, "Developer options are not available for this user")
        if self.CommonWidget().text("Stay awake").right(resourceId="android:id/checkbox") != None:
            if tag:
                if not self.CommonWidget().text("Stay awake").right(resourceId="android:id/checkbox").checked:
                    self.CommonWidget().text("Stay awake").right(resourceId="android:id/checkbox").click()
                    self.logger.debug("Stay awake on")
                else:
                    self.logger.debug("Stay awake off")
            else:
                if self.CommonWidget().text("Stay awake").right(resourceId="android:id/checkbox").checked:
                    self.CommonWidget().text("Stay awake").right(resourceId="android:id/checkbox").click()
                    self.logger.debug("Stay awake off")
                else:
                    self.logger.debug("Stay awake on")
        elif self.CommonWidget().text("Stay awake").right(resourceId="android:id/switchWidget") != None:
            if tag:
                if not self.CommonWidget().text("Stay awake").right(resourceId="android:id/switchWidget").checked:
                    self.CommonWidget().text("Stay awake").right(resourceId="android:id/switchWidget").click()
                    self.logger.debug("Stay awake on")
                else:
                    self.logger.debug("Stay awake off")
            else:
                if self.CommonWidget().text("Stay awake").right(resourceId="android:id/switchWidget").checked:
                    self.CommonWidget().text("Stay awake").right(resourceId="android:id/switchWidget").click()
                    self.logger.debug("Stay awake off")
                else:
                    self.logger.debug("Stay awake on")
        elif self.CommonWidget().text("Stay awake").right(resourceId="android:id/switch_widget") != None:
            if tag:
                if not self.CommonWidget().text("Stay awake").right(resourceId="android:id/switch_widget").checked:
                    self.CommonWidget().text("Stay awake").right(resourceId="android:id/switch_widget").click()
                    self.logger.debug("Stay awake on")
                else:
                    self.logger.debug("Stay awake off")
            else:
                if self.CommonWidget().text("Stay awake").right(resourceId="android:id/switch_widget").checked:
                    self.CommonWidget().text("Stay awake").right(resourceId="android:id/switch_widget").click()
                    self.logger.debug("Stay awake off")
                else:
                    self.logger.debug("Stay awake on")
        
        self.pressBack()
        if not self.CommonWidget().text("Display").exists:
            self.d(scrollable=True).scroll.vert.to(text="Display")
        self.CommonWidget().text("Display").click.wait()
        if not self.CommonWidget().text("After 30 minutes of inactivity").exists:
            self.CommonWidget().text("Sleep").click.wait()
            self.CommonWidget().text("30 minutes").click.wait()
        assert self.CommonWidget().text("After 30 minutes of inactivity").exists

    def pullFileAndCheckFileNumber(self, camera_dir, host_path, mediaFileCount=1):
        self.logger.debug("pull file to host start")
        loop_time = 0
        while loop_time < 10:
            file_name_list = self.getFileName(camera_dir)
            if (not file_name_list) or (len(file_name_list) < mediaFileCount):
                self.logger.debug("actual quantity=%d, expected quantity=%d" % (len(file_name_list), mediaFileCount))
                self.logger.debug("filename list is null or the total files number isn't meet to target!")
                time.sleep(1)
                loop_time += 1
                continue
            if len(file_name_list) >= mediaFileCount:
                self.logger.debug("actual quantity=%d, expected quantity=%d" % (len(file_name_list), mediaFileCount))
                break
#         print "camera_dir=" + camera_dir
#         self.logger.debug("==== camera folder file list =====")
#         self.logger.debug(g_common_obj.adb_cmd_capture_msg("ls %s" % camera_dir))
#         self.logger.debug("==========================")
        if not file_name_list:
            assert False, "the current file can't be taken successful!"
        if len(file_name_list) < mediaFileCount:
            lost_pic = mediaFileCount - len(file_name_list)
            assert False, "actual quantity=%d, expected quantity=%d, the total files number isn't meet to target and lost %d files!" \
                            % (len(file_name_list), mediaFileCount, lost_pic)
        mTime = 0
        mCount = 0
        while mTime < 5:
            for v in range(len(file_name_list)):
                mStr = file_name_list[v].split('.')
                try:
                    if mStr[2]:
                        self.logger.debug(str(mStr[2]))
                        time.sleep(1)
                        mCount = 0
                        break
                except:
                    mCount += 1
                    continue
            if mCount >= len(file_name_list):
                break
            file_name_list = self.getFileName(camera_dir)
            mTime += 1
        self.adbPullFile(camera_dir + ".", host_path)
        self.logger.debug("pull file to host end")
        return file_name_list

    def pressBack(self, num=1):
        for i in range(int(num)):
            self.d.press.back()
            self.logger.debug("press back key")
            time.sleep(1)

    def pressHome(self):
        self.d.press.home()
        self.logger.debug("press home key")
        time.sleep(1)

    def pressPower(self):
        self.d.press.power()
        self.logger.debug("press power key")
        time.sleep(1)

    def pressVolume(self, tag="up"):
        '''
        change volume
        '''
        if tag == "up":
            g_common_obj.adb_cmd_capture_msg("input keyevent 24")
            self.logger.debug("press volume up")
        elif tag == "down":
            g_common_obj.adb_cmd_capture_msg("input keyevent 25")
            self.logger.debug("press volume down")

    def setAlarmTime(self, seconds):
        self.Alarm().alarmPageTime().set_text(seconds)
        time.sleep(1)
        self.d(text="OK").click()
        self.logger.debug("set alarm time " + str(seconds) + "s")

    def setOrientationToVertical(self):
        """
        @summary: set orientation as n
        """
        # self.d.orientation = "n"
        g_common_obj.set_vertical_screen()

    def switchPlatform(self, cfg_file=None, aosp_switch=True, \
                       arcsoft_switch=True, gms_switch=True, refcam2_switch=False, mode_option=""):
        from ArcSoftCamera import ArcSoftCamera
        from AOSPCamera import AOSPCamera
        from GMSCamera import GMSCamera
        from RefCam2Camera import RefCam2Camera
        from testlib.multimedia.multimedia_setting import MultiMediaSetting
        from testlib.util.config import TestConfig

        self.config = TestConfig()
        if None == cfg_file:
            self.cameraConfig = self.config.read(self.DEFAULT_CONFIG_FILE, "multimedia_camera")
        else:
            self.cameraConfig = self.config.read(cfg_file, "multimedia_camera")
        platform = self.getPlatform()
        aosp = self.cameraConfig.get("aosp").split(";")
        arcsoft = self.cameraConfig.get("arcsoft").split(";")
        gms = self.cameraConfig.get("gms").split(";")
        refcam2 = self.cameraConfig.get("refcam2").split(";")
        print aosp, arcsoft, refcam2
        self.multimedia_setting = MultiMediaSetting(self.DEFAULT_CONFIG_FILE)
        self.logger.debug("===platform is %s" % platform)
        self.camera = None
        isFind = False
        if aosp_switch:
            for i in aosp:
                if self.cameraConfig.get(i.lower()) in platform:
                    self.multimedia_setting.install_apk("aosp_camera_apk")
                    self.camera = AOSPCamera()
                    self.logger.debug("new aosp camera successfully")
                    isFind = True
                    break
        if arcsoft_switch:
            for j in arcsoft:
                if self.cameraConfig.get(j.lower()) in platform:
                    self.camera = ArcSoftCamera()
                    self.logger.debug("new arcsoft camera successfully")
                    isFind = True
                    self.multimedia_setting.install_apk("quickpic_apk")
                    break
        if gms_switch:
            for k in gms:
                if self.cameraConfig.get(k.lower()) in platform and 'bxtp_abl' not in platform:
                    self.camera = GMSCamera()
                    self.logger.debug("new GMS camera successfully")
                    isFind = True
                    break
        if refcam2_switch:
            for l in refcam2:
                if self.cameraConfig.get(l.lower()) in platform:
                    self.camera = RefCam2Camera()
                    self.logger.debug("new refcam2 camera successfully")
                    isFind = True
                    self.multimedia_setting.install_apk("quickpic_apk")
                    break
        if not isFind and "HDR" in mode_option:
            if self.cameraConfig.get("CHT_T3".lower()) in platform:
                self.camera = RefCam2Camera()
                self.logger.debug("new refcam2 camera successfully")
                isFind = True
                self.multimedia_setting.install_apk("quickpic_apk")
        if not isFind:
            xmlname = self.getPlatformName()
            print "xmlname=" + xmlname
            if xmlname.upper() in aosp:
                self.camera = AOSPCamera()
                self.logger.debug("fronm xml file new aosp camera successfully")
            if xmlname.upper() in arcsoft:
                self.camera = ArcSoftCamera()
                self.logger.debug("from xml file new arcsoft camera successfully")
                self.multimedia_setting.install_apk("quickpic_apk")
        if None == self.camera:
            self.camera = GMSCamera()
            self.logger.debug("new gms camera successfully")
        self.multimedia_setting.install_apk("gps_apk")
        return self.camera

    def unlockScreen(self):
        self.logger.debug("wakeup start")
        self.d.wakeup()
        self.logger.debug("wakeup end")
        time.sleep(1)
        self.logger.debug("start unlock Screen")
        if self.CommonWidget().clockIcon().exists or self.CommonWidget().cameraIconInLockScreen().exists:
            self.d.swipe(self.x / 2, self.y - 10, self.x / 2, self.y / 2, steps=100)
            self.logger.debug("slides up screen")
        if self.CommonWidget().lockScreenIcon().exists:
            g_common_obj.adb_cmd_capture_msg("input keyevent 82")
            self.logger.debug("unlock screen")
        if self.CommonWidget().pinEntry().exists:
            self.CommonWidget().pinEntry().set_text("1234")
            g_common_obj.adb_cmd("input keyevent 66")
        if self.CommonWidget().lockScreenPasswordEntry().exists:
            self.CommonWidget().lockScreenPasswordEntry().set_text("qwer")
            g_common_obj.adb_cmd("input keyevent 66")
        if self.CommonWidget().text("Owner").exists:
            self.CommonWidget().text("Owner").click()
            if self.CommonWidget().lockScreenPasswordEntry().exists:
                self.CommonWidget().lockScreenPasswordEntry().set_text("qwer")
                g_common_obj.adb_cmd("input keyevent 66")
        time.sleep(2)

    def refreshCameraDir(self):
        """
        Send media_mount broadcase to notify the direcorty change.
        """
        cmd = "am broadcast -a android.intent.action.MEDIA_MOUNTED  --ez read-only false -d file://sdcard"
        g_common_obj.adb_cmd(cmd)

    def removeAllAppFromRecentTask(self):
        while True:
            self.d.press.recent()
            time.sleep(1)
            while True:
                if self.CommonWidget().activityDescription().exists:
                    ml, mt, mr, mb = self.getBounds(self.CommonWidget().activityDescription())
                    print ml, mt, mr, mb
                    self.d.swipe(ml + 40, mt + 40, mr, mt + 40, steps=50)
                else:
                    break
                time.sleep(1)
            if not self.CommonWidget().activityDescription().exists:
                break
        self.logger.debug("remove all app from recent task successfully")

    def removeBigFile(self):
        """
        remove the big_file created by function: fillStorage
        """
        # delete the former created file if exists.
        if self.fileExists("/sdcard/DCIM/big_file", "DEVICE"):
            g_common_obj.adb_cmd('rm -r /sdcard/DCIM/big_file')
        self.refreshCameraDir()

    def removeDeivceFile(self, file_path="/mnt/sdcard/DCIM/Camera/*"):
        g_common_obj.adb_cmd_capture_msg("rm -rf %s" % file_path)
        self.removeBigFile()
        self.logger.debug("remove device files successfully")

    def moveFilesFromCameraFolder(self):
        self.logger.debug("move files from camera folder to the camera temp folder")
        g_common_obj.adb_cmd_capture_msg("mkdir -p /mnt/sdcard/DCIM/camera_tmp")
        g_common_obj.adb_cmd_capture_msg("mv /mnt/sdcard/DCIM/Camera/* /mnt/sdcard/DCIM/camera_tmp/")

    def recoverFilesFromCameraTemp(self):
        self.logger.debug("move files from camera temp folder to the camera folder")
        g_common_obj.adb_cmd_capture_msg("mv /mnt/sdcard/DCIM/camera_tmp/* /mnt/sdcard/DCIM/Camera/")

    def removeFile(self, path):
        try:
            self.logger.debug("rm -rf " + path)
            os.system("rm -rf " + path)
            if os.path.exists(path):
                os.system("rm -rf " + path)
            else:
                self.logger.debug("remove file success")
        except Exception as e:
            self.logger.debug(str(e))

    def removeRecentApp(self, appName="Camera"):
        '''
        former name : removeRecentApp
        '''
        self.d.press.recent()
        time.sleep(1)
        self.logger.debug("click recent app button successfully")
        for i in range(5):
            if not (self.CommonWidget().text(appName).exists and self.CommonWidget().taskViewThumbnail().exists):
                break
            bounds = self.d(text=appName).bounds
            self.logger.debug(str(bounds))
            if self.d(text=appName).right(resourceId="com.android.systemui:id/dismiss_task") != None:
                self.d(text=appName).right(resourceId="com.android.systemui:id/dismiss_task").click.wait()
                self.logger.debug("remove " + str(appName) + " from recent app successfully")
            else:
                self.d.swipe(bounds.get("left") + 30, bounds.get("top") + 30, self.x - 50, bounds.get("top") + 30)
                self.logger.debug("remove " + str(appName) + " from recent app successfully")
            time.sleep(2)
        if self.CommonWidget().text(appName).exists and self.CommonWidget().taskViewThumbnail().exists:
            self.assertTrue(False, "remove " + str(appName) + " from recent app failed")

    def setWallpaper(self, tag="default"):
        self.pressHome()
        self.logger.debug("set wallpaper start")
        bounds = self.d().bounds
        screen_x = int(bounds.get("right") / 2)
        screen_y = int(bounds.get("bottom") / 2)
        offset_y = screen_y / (5 + 3)
        for _ in range (5):
            self.d.long_click(screen_x, screen_y)
            if self.CommonWidget().text("Wallpapers").exists:
                self.CommonWidget().text("Wallpapers").click.wait()
                break
            if self.CommonWidget().text("WALLPAPERS").exists:
                self.CommonWidget().text("WALLPAPERS").click.wait()
                break
            screen_y += offset_y
            self.d.long_click(screen_x, screen_y)
        self.logger.debug("click wallpapers button successfully")
        time.sleep(1)
        if "5" in self.getAndroidVersion() or self.getAndroidVersion() == "L" or "cht_mrd" in self.getPlatform():
#             self.d.long_click(icon_left, icon_top - 350)
#             if tag == "picture":
#                 self.CommonWidget().text("Photos").click.wait()
#                 self.logger.debug("click wallpapers button successfully")
#                 self.CommonWidget().text("Camera").click.wait()
#                 title = self.d(resourceId="com.google.android.apps.photos:id/toolbar").bounds
#                 title_bottom = title.get("bottom")
#                 self.d.click(20, title_bottom + 50)
#                 time.sleep(2)
#                 if not self.CommonWidget().text("SET WALLPAPER").exists:
#                     self.d.click(20, title_bottom + 150)
#                     time.sleep(2)
#                 self.CommonWidget().text("SET WALLPAPER").click.wait()
#             elif tag == "default":
#                 self.CommonWidget().text("Wallpapers").click.wait()
#                 self.CommonWidget().text("Set wallpaper").click.wait()
            if tag == "picture":
                if self.CommonWidget().setWallpaperItemLabel().exists:
                    self.CommonWidget().setWallpaperItemLabel().click.wait()
                elif self.CommonWidget().text("My photos").exists:
                    self.CommonWidget().text("My photos").click.wait()
                else:
                    self.logger.debug("Can't choose the photo album")
                self.waitForWidgetToAppear(self.CommonWidget().setWallpaperIconThumb(), "wallpaper thumbnail")
                time.sleep(2)
                self.CommonWidget().setWallpaperIconThumb()[0].long_click()
                time.sleep(3)
                if self.CommonWidget().text("open").exists or self.CommonWidget().text("OPEN").exists:
                    self.CommonWidget().text("open").click()
                time.sleep(3)
                self.CommonWidget().text("Set wallpaper").click.wait()
                if self.CommonWidget().text("Home screen").exists:
                    self.CommonWidget().text("Home screen").click.wait()
            elif tag == "default":
                self.logger.debug("set wallpaper default tag start")
                t_count = self.d(className="android.widget.FrameLayout", longClickable="false").count
                self.logger.debug("count:" + str(t_count))
                self.d(className="android.widget.FrameLayout", longClickable="false")[4].click()
                time.sleep(3)
                self.logger.debug("set wallpaper default tag : click wallpaper button")
                self.CommonWidget().text("wallpaper").click.wait()
                if self.CommonWidget().text("Home screen").exists:
                    self.logger.debug("set wallpaper default tag : click Home screen button")
                    self.CommonWidget().text("Home screen").click.wait()
                self.deleteAllWallpaper()
        else:
            if tag == "picture":
                if self.CommonWidget().setWallpaperItemLabel().exists:
                    self.CommonWidget().setWallpaperItemLabel().click.wait()
                elif self.CommonWidget().text("My photos").exists:
                    self.CommonWidget().text("My photos").click.wait()
                else:
                    self.logger.debug("Can't choose the photo album")
                time.sleep(5)
                self.waitForWidgetToAppear(self.CommonWidget().setWallpaperIconThumb(), "wallpaper thumbnail")
                time.sleep(2)
                self.CommonWidget().setWallpaperIconThumb()[1].long_click()
                self.CommonWidget().text("open").click()
                time.sleep(3)
                self.CommonWidget().text("Set wallpaper").click.wait()
                if self.CommonWidget().text("Home screen").exists:
                    self.CommonWidget().text("Home screen").click.wait()
            elif tag == "default":
                self.logger.debug("count:" + str(self.CommonWidget().setWallpaperImage().count))
                self.CommonWidget().setWallpaperImage()[4].click.wait()
                self.CommonWidget().text("Set wallpaper").click.wait()
                if self.CommonWidget().text("Home screen").exists:
                    self.CommonWidget().text("Home screen").click.wait()
        time.sleep(2)
        self.logger.debug("set wallpaper end")

    def deleteAllWallpaper(self):
        self.pressHome()
        self.logger.debug("deleteAllWallpaper start")
        bounds = self.d().bounds
        screen_x = int(bounds.get("right") / 2)
        screen_y = int(bounds.get("bottom") / 2)
        offset_y = screen_y / (5 + 3)
        for _ in range (5):
            self.d.long_click(screen_x, screen_y)
            if self.CommonWidget().text("Wallpapers").exists:
                self.CommonWidget().text("Wallpapers").click.wait()
                break
            if self.CommonWidget().text("WALLPAPERS").exists:
                self.CommonWidget().text("WALLPAPERS").click.wait()
                break
            screen_y += offset_y
            self.d.long_click(screen_x, screen_y)
        time.sleep(3)
        t_count = self.d(longClickable="true").count
        self.logger.debug("count:" + str(t_count))
        for i in range(t_count):
            self.d(longClickable="true")[i].long_click()
        if self.d(resourceId="com.android.wallpaperpicker:id/menu_delete").exists:
            self.d(resourceId="com.android.wallpaperpicker:id/menu_delete").click()
        elif self.d(resourceId="com.google.android.googlequicksearchbox:id/menu_delete").exists:
            self.d(resourceId="com.google.android.googlequicksearchbox:id/menu_delete").click()
        else:
            self.d(description="Delete").click()
        self.pressBack(2)
        self.logger.debug("deleteAllWallpaper end")

    def swipeScreen(self, orientation="right"):
        """
        @summary: swipe screen
        """
        if orientation == "left":
            self.d.swipe(self.x - 50, self.y / 2, 150, self.y / 2, steps=50)
            self.logger.debug("swipe the screen to the left")
        elif orientation == "right":
            self.d.swipe(50, self.y / 2, self.x - 150, self.y / 2, steps=50)
            self.logger.debug("swipe the screen to the right")
        elif orientation == "up":
            self.d.swipe(self.x / 2, self.y / 2, self.x / 2, 50, steps=50)
            self.logger.debug("Swipe the screen to the up")
        else:
            self.logger.debug("The orientation has not changed")

    def unitConversion(self, str):
        if "G" in str:
            return float(str.replace("G", "")) * 1024
        elif "M" in str:
            return float(str.replace("M", ""))
        else:
            try:
                re = float(str)
                return re / 1024
            except:
                return None

    def verifyFileSize(self, file_path, exp_size):
        """ verify file size on host """
        self.logger.debug("Verify file size of %s, expect size: %s" % (file_path, exp_size))
        bs, count = self.parse_bs_count(exp_size)
        if bs <= 0:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert bs > 0, "Parse bs, count for dd command fail"
        exp_size = bs * count
        act_size = self.get_file_size(file_path)
        self.logger.debug("Expect size: %d bytes, Actual size: %d bytes" % (exp_size, act_size))
        if act_size != exp_size:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert act_size == exp_size, "File %s expect size:[%d], but actual size:[%d]" % \
            (file_path, exp_size, act_size)

    def waitAlarmTriiggered(self, wait_time=30):
        t_time = 2
        for i in range(1, wait_time, t_time):
            time.sleep(t_time)
            if self.Alarm().alarmPromptMessage().exists:
                break
        if self.Alarm().alarmPromptMessage().exists == False:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
        assert self.Alarm().alarmPromptMessage().exists, "Alarm wait timeout!"

    def waitForWidgetToAppear(self, widget, desc="", timeout=10, flag=0):
        time_left = 0
        start = time.time()
        while time_left < timeout:
            if widget == None or widget.exists == False or widget.exists == None:
                if flag == 1:
                    self.d.press.recent()
                    time.sleep(3)
                    self.d.press.back()
                    time.sleep(2)
                time.sleep(0.5)
            else:
                return True
            time_left = time.time() - start
            self.logger.debug("time_left=" + str(time_left))
            self.checkCameraCrash()
        if widget == None or widget.exists == False or widget.exists == None:
            self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
            assert False, "Wait " + str(desc) + " to appear timeout in " + str(timeout) + " s"
        time.sleep(0.5)

    def waitForWidgetToDisappear(self, widget, desc="", timeout=10):
        time_left = 0
        start = time.clock()
        while time_left < timeout:
            try:
                if widget != None and widget.exists == True:
                    time.sleep(0.5)
                else:
                    return True
            except TypeError:
                if widget != None and widget.exists == True:
                    time.sleep(0.5)
                else:
                    return True
            time_left = time.clock() - start
        try:
            if widget != None and widget.exists == True:
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
                assert False, "Wait widget " + str(desc) + " to disappear timeout in " + str(timeout) + " s"
        except TypeError:
            if widget != None and widget.exists == True:
                self.getScreenshotAndPullToHost("assert.png", g_common_obj.get_user_log_dir())
                assert False, "Wait widget " + str(desc) + " to disappear timeout in " + str(timeout) + " s"

    def waitForTheFilesAreGenerated(self, expectCount=1):
        if not self.makefileTime:
            return False
        for i in range(int(self.makefileTime)):
            self.list = self.getFileName(self.camera_dir)
            self.logger.debug("generated files count = %d" % (len(self.list)))
            if len(self.list) >= int(expectCount):
                return True
            else:
                time.sleep(1)
        self.logger.debug("generated file time for greater than %d seconds" % self.makefileTime)
        return False

    def zoomInOrOut(self, camera_Obj, cmd):
        from testlib.camera.checkIQ import CheckIQ
        from testlib.camera.checkIQ import CheckZoom
        self.camera = camera_Obj
        zoom_name = ""
        mTime = 1
        for i in range(10):
            os.system(cmd)
            time.sleep(2)
            self.camera.capturePhoto()
            time.sleep(2)
            zoom_name = self.checkFileCorrupt()[1]
            zoom_name = self.host_path + "/" + zoom_name
            # iqcheckDir=self.host_path +os.sep + "IQCheck"
            mRatio = CheckZoom().getZoomRatio(zoom_name)
            self.logger.debug("Digital Zoom Ratio=" + str(mRatio))
            if mRatio == "1":
                mTime += 1
                self.removeDeivceFile()
                self.removeFile(zoom_name)
            else:
                print zoom_name
                break
        print "mTime total=" + str(mTime)
        return zoom_name, mRatio

    def currentDisplayIsVertical(self):
        width = self.d.info["displayWidth"]
        height = self.d.info["displayHeight"]
        if width > height:
            self.logger.debug("Current display is horizontal")
            return False
        else:
            self.logger.debug("Current display is vertical")
            return True

    def initDevice(self):
        from DeviceControl import DeviceControl
        from testaid.robot import Robot
        deviceName = DeviceControl().getRobotDeviceName()
        return Robot(deviceName)

    def moveDevice(self, device, steps, speed=10):
        try:
            device.move(steps, speed)
        except:
            pass

    def resetDevice(self, device):
        device.reset()

    def rotateDevice(self, device, angle, speed=10, sleep_time=3):
        device.rotate(angle, speed)
        time.sleep(sleep_time)

    def controlLightBox(self, index):
        from DeviceControl import DeviceControl
        DeviceControl().pressLightBoxButton(index)
        time.sleep(0.5)

    def getMediaFolderOnHost(self):
        return g_common_obj.get_user_log_dir()

    def getTestEntryName(self):
        import traceback
        for file, line, method, function in traceback.extract_stack():
            if "test_Camera" in method or "test_camera" in method:
                return method

    def getMediaFilesFromDevice(self, file_index, mediaFileCount=1):
        file_name_list = self.pullFileAndCheckFileNumber(self.camera_dir, self.host_path, mediaFileCount)
        hostPath = self.getMediaFolderOnHost()
        namePrefix = self.getTestEntryName()
        fileNameList = []
        for f in file_name_list:
            if f.find(".jpg") != -1:
                fileName = hostPath + "/" + namePrefix + "_" + str(file_index) + ".jpg"
            elif f.find(".mp4") != -1:
                fileName = hostPath + "/" + namePrefix + "_" + str(file_index) + ".mp4"
            for j in range(10):
                if not self.checkFile(fileName) or j == 0:
                    self.adbPullFile(self.camera_dir + f, fileName)
                    time.sleep(2)
                else:
                    self.logger.debug(fileName + " exists")
                    break
            fileNameList.append(fileName)
            file_index += 1
        # Need to copy the files into the TCR folder
        # for f in fileNameList:
        #    os.system("cp "+f+" ")
        return fileNameList

    def grantPermission(self, package_name, permission_list):
        cmd = "pm grant %s %s"
        for permission in permission_list:
            g_common_obj.adb_cmd_capture_msg(cmd % (package_name, permission))

    def clean_app_data(self, package_name):
        self.logger.debug("clean %s app data start" % package_name)
        g_common_obj.stop_app_am(package_name)
        g_common_obj.adb_cmd("pm clear %s" % package_name)
        self.logger.debug("clean %s app data successfully" % package_name)
