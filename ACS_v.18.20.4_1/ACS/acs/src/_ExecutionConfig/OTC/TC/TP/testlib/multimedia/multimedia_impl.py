# coding: UTF-8
import os
import sys
import time
from time import sleep
from random import randrange
from igascomparator import *
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2
from ctcrunner import *


def verifyImage(img):
    try:
        from PIL import Image
    except:
        Image = None
    if Image is not None:
        v_image = Image.open(img)
        v_image.verify()


class MultiMediaImpl:
    """
    Multi-media functions.
    """

    setting_pkg_name = "com.android.settings"
    setting_activity_name = ".Settings"

    musicplayer_pkg_name="com.google.android.music"
    music_activity_name=".ui.HomeActivity"

    music_pkg_name = "com.android.music"
    musicactivityname = ".MusicBrowserActivity"

    chrome_pkg_name = "com.android.chrome"
    chrome_activity_name = "com.google.android.apps.chrome.Main"

    photos_package = "com.google.android.apps.plus"
    photos_activity = "com.google.android.apps.photos.phone.PhotosLauncherActivity"

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()
        g_common_obj.adb_cmd_capture_msg("pm clear com.android.music")
        self.cfg = cfg

    @staticmethod
    def startAppMusic():
        d = g_common_obj.get_device()
        g_common_obj.launch_app_from_home_sc("Play Music")
        time.sleep(10)
        for _ in range(30):
            time.sleep(2)
            if not d(resourceId="com.google.android.music:id/tutorial_logo").exists:
                break
        assert not d(resourceId="com.google.android.music:id/tutorial_logo").exists
        if d(text="Skip").exists:
            d(text="Skip").click.wait()
        if d(text="Use Standard").exists:
            d(text="Use Standard").click.wait()
        if d(text="Done").exists:
            d(text="Done").click.wait()
        if d(resourceId="com.google.android.music:id/play_drawer_list").exists:
            d.press.back()
        if d(text="Got it").exists:
            d(text="Got it").click.wait()
        g_common_obj.launch_app_am(
            "com.google.android.music", "com.android.music.activitymanagement.TopLevelActivity")
        time.sleep(5)
        g_common_obj.back_home()

    def change_wallpaper_random(self):
        self.d.press.home()
        self.d(resourceId="com.google.android.googlequicksearchbox:id/page_indicator").long_click()
        self.d(text = "Wallpapers").click.wait()
        i = randrange(1, 10)
        self.d(className = "android.widget.HorizontalScrollView").scroll.horiz.to(descriptionStartsWith = "Wallpaper %d of" % i)
        assert self.d(descriptionStartsWith = "Wallpaper %d of" % i).exists
        self.d(descriptionStartsWith = "Wallpaper %d of" % i).click.wait()
        self.d(text = "Set wallpaper").click.wait()
        assert not self.d(text = "Set wallpaper").exists

    def launch_setting_by_am(self):
        g_common_obj.launch_app_am(MultiMediaImpl.setting_pkg_name, MultiMediaImpl.setting_activity_name)
        time.sleep(2)

    def stop_setting_by_am(self):
        g_common_obj.stop_app_am(MultiMediaImpl.setting_pkg_name)

    def launch_photo_by_am(self):
        g_common_obj.launch_app_am(MultiMediaImpl.photos_package, MultiMediaImpl.photos_activity)
        time.sleep(2)

    def stop_photo_by_am(self):
        g_common_obj.stop_app_am(MultiMediaImpl.photos_package)

    def force_stop_app(self,appname):
        g_common_obj.launch_app_am(MultiMediaImpl.setting_pkg_name, MultiMediaImpl.setting_activity_name)
        self.d(text = "Apps").click()
        i = 0
        while not self.d(text = "All",clickable = False).exists and i<=1:
            self.d(className = "android.widget.FrameLayout").swipe.left()
            i = i + 1
        i = 0
        while (not self.d(text = "All",clickable = False).exists) and i<=1:
            self.d(className = "android.widget.FrameLayout").swipe.right()
            i = i + 1
        self.d(resourceId = "android:id/list").scroll.vert.to(text=appname)
        self.d(text = appname).click()
        self.d(text = "Force stop").click()
        if self.d(text = "OK").exists:
            self.d(text = "OK").click()

    def launch_by_am(self):
        g_common_obj.launch_app_am(MultiMediaImpl.musicplayer_pkg_name,MultiMediaImpl.music_activity_name)
        time.sleep(3)
        if (self.d(text = 'Not now').exists):
            self.d(text = 'Not now').click()
        if (self.d(text = 'Skip').exists):
            self.d(text = 'Skip').click()


    def stop_by_am(self):
        g_common_obj.stop_app_am(MultiMediaImpl.musicplayer_pkg_name)

    def launch_music_by_am(self):
        g_common_obj.launch_app_am(MultiMediaImpl.music_pkg_name,MultiMediaImpl.musicactivityname)
        time.sleep(2)
        
    def stop_music_by_am(self):
        g_common_obj.stop_app_am(MultiMediaImpl.music_pkg_name)

    def enter_songs_page(self):
        if self.d(description="Show navigation drawer").exists:
            self.d(description="Show navigation drawer").click()
        time.sleep(1)
        self.d(text="My Library").click()
        time.sleep(1)
        if self.d(resourceId="com.google.android.music:id/play_drawer_list").exists:
            self.d(text="SONGS").click()
        self.d(text="SONGS").click()


    def get_ready_toplay(self):
        self.enter_songs_page()
        assert self.d(text="AAC_HE_48kHz_76kbps_Stereo_CBR").exists

    def play_music(self):
        self.d(text="AAC_HE_48kHz_76kbps_Stereo_CBR",resourceId = "com.google.android.music:id/li_title").click.wait()
        assert self.d(description="Pause").exists

    def isMusicPlaying(self):
        return self.d(description="Pause").exists

    def clickPause(self):
        self.d(resourceId = "com.google.android.music:id/pause").click()

    def clickPrevious(self):
        self.d(resourceId="com.google.android.music:id/prev").click()

    def clickNext(self):
        self.d(resourceId="com.google.android.music:id/next").click()

    def enterPlayView(self):
        if self.d(resourceId="com.google.android.music:id/album_small").exists:
            self.d(resourceId="com.google.android.music:id/album_small").click()

    def musicRepeat(self,times):
        """
            @summary: music Repeat
        """
        for i in range(times):
#             Constant.Log.info("click repeat " + str(i) + " times")
            self.d(resourceId = "com.google.android.music:id/repeat").click()
            time.sleep(1)

    def open_music_by_index(self, index=0):
        self.d(resourceId="android:id/list").child(className="android.widget.RelativeLayout",index=index).click()
        time.sleep(1)

    def next_music(self):
        self.clickNext()

    def stop_playing_music(self):
        self.d(resourceId = "com.google.android.music:id/pause").click.wait()
        assert self.d(description="Play").exists

    def play_music_aosp(self):
        if self.d(text = "Songs").exists:
            self.d(text = "Songs").click.wait()
            self.d(text="AAC_HE_48kHz_128kbps_Stereo_CBR").click()

    def launch_voice_recorder(self):
        audio_record_pkg = "com.intel.psi_recorder"
        audio_record_activity = ".PSI_Recorder"
        g_common_obj.launch_app_am (audio_record_pkg,audio_record_activity)
        time.sleep(5)

    def record_sound(self,lasttime):
        time.sleep(3)
        if self.d(resourceId = "com.intel.psi_recorder:id/bRecord" , text = "Rec.").exists:
            self.d(resourceId = "com.intel.psi_recorder:id/bRecord" , text = "Rec.").click()
        else:
            print "can't start PSI recorder!"
            assert("cant' start PSI recorder")
        time.sleep(lasttime)
        self.d(resourceId = "com.intel.psi_recorder:id/bStop", text = "Stop").click()

    def quit_record(self):
        self.d(className = "android.widget.ImageButton").click()
        time.sleep(2)
        self.d(text = "Exit").click()

    def audio_record_playback(self,playtime):
        self.d(resourceId = "com.intel.psi_recorder:id/bPlayback", text = "Play").click()
        time.sleep(playtime)
        if self.d(resourceId = "com.intel.psi_recorder:id/bStop", text = "Stop").exists:
            self.d(resourceId = "com.intel.psi_recorder:id/bStop", text = "Stop").click()

    def audio_record_delete(self):
        self.d(resourceId = "com.intel.psi_recorder:id/bDelete", text = "Delete").click()
        time.sleep(3)

    def find_icon(self,filename,timeout = 3,error = "can not get the picture"):
        from testlib.common.base import getTmpDir
        if_getpic_succeed = False
        time = 0
        base_path = os.path.split(os.path.realpath(__file__))[0].split(os.sep)
        pics_path = (os.sep).join(base_path + ['pics', ''])
        if timeout > 8:
            timeout = 3
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        mycomp = igascomparator()
        region = matchedregion_s()
        base_filename = filename
        next = 2
        toEnd = False
        swipetime = 1
        sleep(2)
        screenshotPath = getTmpDir()
        if not os.path.exists(screenshotPath):
            os.mkdir(screenshotPath)
        screenshotPath = os.path.join(screenshotPath, "screenshot.png")
        while region.center_x == 0 and region.center_y == 0 and time<timeout:
            verifyImage(pics_path+filename)
            for eachTry in range(3):
                try:
                    g_common_obj.take_screenshot(screenshotPath)
                    verifyImage(screenshotPath)
                    sleep(2)
                    break
                except BaseException,e:
                    if eachTry == 2:
                        raise
                    print e
                    print "screen shot failed, try again!"
            mycomp.findpattern(pics_path+filename, screenshotPath, region)
            sleep(1)
            time = time + 2
            if region.center_x != 0 or region.center_y !=0:
                if_getpic_succeed = True
            if time >= timeout and region.center_x == 0 and region.center_y == 0:
                tempname = base_filename.split(".")
                next_file = tempname[0] + str(next) + "."+ tempname[1]
                if(os.path.exists(pics_path + next_file)):
                    filename = next_file
                    print filename
                    next += 1
                    time = 1
                else:
                    if toEnd:
                        break
                    swipetime +=1
                    #self.d.swipe(x-50,y/2,50,y/2)
                    time = 1
                    filename = base_filename
                    print filename
                    next = 2
                    if swipetime > 5 :
                        toEnd = True
                    continue
        delete_cmd = "rm -rf " + screenshotPath
        os.system(delete_cmd)
        return (region.center_x,region.center_y)

    def close_background_app(self):
        print "close background app"
        time.sleep(1)
        self.d.press.home()
        time.sleep(1)
        self.d.press(187)
        time.sleep(1)
        while True:
            if self.d(resourceId = "com.android.systemui:id/task_view_thumbnail").exists:
                self.d(resourceId = "com.android.systemui:id/task_view_thumbnail").swipe.left()
            else:
                break
        time.sleep(1)
    def launch_chrome_by_am(self):
        g_common_obj.launch_app_am(MultiMediaImpl.chrome_pkg_name, \
            MultiMediaImpl.chrome_activity_name)
        time.sleep(5)
        while self.d(text="Accept & continue").exists:
            self.d(text="Accept & continue").click.wait()
        if self.d(text="Done").exists:
            self.d(text="Done").click.wait()
        if self.d(text="No thanks").exists:
            self.d(text="No thanks").click.wait()
        if self.d(text="No Thanks").exists:
            self.d(text="No Thanks").click.wait()

    def stop_chrome_by_am(self):
        g_common_obj.stop_app_am(MultiMediaImpl.chrome_pkg_name)

    def open_website(self, url):
        self.d(resourceId = \
            "com.android.chrome:id/url_bar").set_text(url)
        self.d.press("enter")

    def play_youtube_video(self):
        (x,y) = self.find_icon(self.getUrl("video_start"))
        if x!=0 and y!=0:
            self.d.click(x,y)
            if self.d(text = "Always").exists:
                self.d(text = "Always").click()
        if x==0 and y==0:
            (x,y) = self.find_icon(self.getUrl("video_play"))
            if x!=0 and y!=0:
                self.d.click(x,y)
                if self.d(text = "Always").exists:
                    self.d(text = "Always").click()
            else:
                assert x > 0,("can't find the play icon!")

    def close_pop_up_menu(self):
        if self.d(text = "Chrome").exists:
            self.d(text = "Chrome").click.wait()
            if self.d(text = "Always").exists:
                self.d(text = "Always").click.wait()
            time.sleep(20)

    def getUrl(self,key):
        return self.cfg.get(key)

    def clear_cache(self):
        self.d(text = "Clear cache").click.wait()

    def connectMiracast(self):
        """connect Miracast adapter"""
        self.launch_setting_by_am()
        self.d(text = "Display").click.wait()
        self.d(text = "Cast screen").click.wait()
        time.sleep(3)
        adapterName=self.cfg.get("miracast_adapter_name")
        print "adapter Name=",adapterName
        if self.d(text=adapterName).exists:
            if self.d(text = adapterName).down().info["text"] == "Connected":
                return
            else :
                self.d(description="More options", \
                       className="android.widget.ImageButton").click.wait()
                checked = self.d(resourceId="android:id/checkbox", \
                className="android.widget.CheckBox").info["checked"]
                if not checked :
                    self.d(resourceId="android:id/checkbox", \
                           className="android.widget.CheckBox").click.wait()
                else :
                    self.d(resourceId="android:id/checkbox", \
                           className="android.widget.CheckBox").click.wait()
                    self.d(description="More options", \
                           className="android.widget.ImageButton").click.wait()
                    self.d(resourceId="android:id/checkbox", \
                           className="android.widget.CheckBox").click.wait()
        else :
            self.d(description="More options", \
                   className="android.widget.ImageButton").click.wait()
            checked = self.d(resourceId="android:id/checkbox", \
                    className="android.widget.CheckBox").info["checked"]
            if not checked :
                self.d(resourceId="android:id/checkbox", \
                       className="android.widget.CheckBox").click.wait()
            else :
                self.d(resourceId="android:id/checkbox", \
                       className="android.widget.CheckBox").click.wait()
                self.d(description="More options", \
                       className="android.widget.ImageButton").click.wait()
                self.d(resourceId="android:id/checkbox", \
                       className="android.widget.CheckBox").click.wait()
        time.sleep(10)
        repeat_time = 0
        while True:
            if not (self.d(text = adapterName).down().info["text"] == "Connected"):
                if self.d(text="Connecting").exists:
                    time.sleep(15)
                else:
                    self.d(text = adapterName).click.wait()
                    time.sleep(15)
            else:
                break
            if repeat_time >10:
                break
            repeat_time += 1
        assert (self.d(text = adapterName).down().info["text"] == "Connected"),\
                "[ERROR]: Miracast adapter not connected"
        print "connect miracast adapter success"

    def playMoviesFromPhotosApp(self,folderName):
        """play movies from the Photos application and select a video file to play"""
        self.openFileFromPhotoApp(folderName)
        self.d.click(150,250)
        assert self.d(className = "android.widget.FrameLayout",index = 0).exists, \
                "[ERROR]: View not found!"
        self.d(className = "android.widget.FrameLayout",index = 0).click()
        time.sleep(5)

    def openFileFromPhotoApp(self,folderName):
        self.launch_photo_by_am()
        print folderName
        self.d(description="Open navigation drawer").click.wait()
        self.d(text = "On device").click.wait()
        if self.d(text = folderName).exists:
            self.d(text = folderName).click.wait()
        else :
            self.d(scrollable = True).scroll.to(text = folderName)
            self.d(text = folderName).click.wait()
        time.sleep(2)

    def cleanNotification(self):
        y = self.d.info["displayHeight"]
        self.d.swipe(0,0,0,y/2,steps=10)
        time.sleep(1)
        btn_clear = self.d(resourceId="com.android.systemui:id/clear_all_button")
        btn_clear2 = self.d(resourceId="com.android.systemui:id/dismiss_text")
        if btn_clear.exists:
            btn_clear.click.wait()
        elif btn_clear2.exists:
            btn_clear2.click.wait()
        else:
            self.d.press.back()

    def share_images_to_picasa(self):
        """share images to picasa"""
        self.d(className="android.view.View", index = 0).click.wait()
        self.d(resourceId = "com.google.android.apps.plus:id/share").click.wait()
        print "share images to picasa..."
        if self.d(text = "Picasa Tool").exists:
            self.d(text = "Picasa Tool").click.wait()
        else:
            self.d(resourceId="android:id/list").swipe.up()
            self.d(text = "Picasa Tool").click.wait()
        time.sleep(2)
        print "start share files..."
        if self.d(text = "Upload").exists:
            self.d(text = "Upload").click.wait()
        elif self.d(textStartsWith = "Upload (1)").exists:
            self.d(textStartsWith = "Upload (1)").click.wait()
        time.sleep(3)
        self.d.press.back()
        if self.d(text="OK").exists:
            self.d(text="OK").click.wait()
        time.sleep(10)


class WidevineImpl:
    """
    @summary: The basic function to test security
    """
#--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def package_widevine(self):
            """ UI button widevine package"""
            return self.d(packageName="com.widevine.demo")

        @property
        def btn_stream(self):
            """ UI button stream """
            return self.d(text="Streaming", resourceId="android:id/title")

        @property
        def btn_no_content(self):
            """ UI button no content """
            return self.d(text="No Content Found")

        @property
        def btn_first_view(self):
            """ UI button no content """
            return self.d(instance="0", \
                className="android.widget.ImageView")

        @property
        def btn_acquire_rights(self):
            """ UI button acquire rights """
            return self.d(text="Acquire Rights", \
                className="android.widget.Button")

        @property
        def btn_cannot_play(self):
            """ UI button cannot play """
            return self.d(textContains="Can't play")

        @property
        def btn_stop(self):
            """ UI button acquire rights """
            return self.d(text="Stop", \
                className="android.widget.Button")

        @property
        def btn_chrome_setup(self):
            """ UI button setup title in Welcome pop up """
            return self.d(textContains = "Set up Chrome", \
                resourceId = "com.android.chrome:id/title")

        @property
        def btn_wide_info(self):
            """ UI info """
            return self.d(className="android.widget.TextView")

        @property
        def btn_first_video(self):
            """ UI button first video """
            return self.d(index="1", \
                className="android.widget.ImageView")

    def __init__ (self):
        """
        init
        """
        self._device = g_common_obj.get_device()
        self._locator = WidevineImpl.Locator(self._device)


    def widevine_video_play(self, timeout=10):
        """
        @summary: play widevine drm protected video
        @return: None
        """
        self.launch_app_from_home_sc("Widevine Demo")
        self._locator.package_widevine.wait.exists(timeout=5000)
        self._locator.btn_stream.click()
        assert not self._locator.btn_no_content.exists, \
        "ERROR: The wifi is not available! Please check!"
        self._locator.btn_first_view.click.wait()
        self._locator.btn_acquire_rights.click()
        time.sleep(30)
        wide_info = self._locator.btn_wide_info.text
        wide_info = wide_info.encode('gbk')
        assert wide_info.find("Rights installed") != -1, \
        "ERROR: Acquire rights failed in 30s"
        self._locator.btn_first_video.click()
        time.sleep(60)
        wide_info = self._locator.btn_wide_info.text
        wide_info = wide_info.encode('gbk')
        assert wide_info.find("Playback start") != -1, \
        "ERROR: The video doesn't play in 60s"
        assert wide_info.find("Playback start.acquireRights = -2000") == -1, \
        "Acquire rights failed"
        time.sleep(timeout)
        assert not self._locator.btn_cannot_play.exists, \
        "Can not play this video!"
        self._locator.btn_stop.click()
        g_common_obj.adb_cmd_capture_msg("am force-stop com.widevine.demo")

    def launch_app_from_home_sc(self, appname):
        """
            restrute for there is no switch widget/apps in app screen
        """
        iffind = False
        self._device.press.home()
        self._device(description="Apps").click()
        for i in range(10):
            time.sleep(2)
            if self._device(text=appname).exists:
                self._device(text=appname).click()
                iffind = True
                break
            if i <= 5:
                self._device(scrollable=True).scroll.horiz()
            else:
                self._device(scrollable=True).scroll.horiz.backward()
        assert iffind == True


class VideoImpl:
    """
    @summary: The basic function to video
    """

    def __init__ (self, cfg):
        """
        init
        """
        self._device = g_common_obj.get_device()
        self._action = "android.intent.action.VIEW"
        self._activity = "com.google.android.apps.plus/.phone.VideoViewActivity"
        self._package = "com.google.android.apps.plus"
        self.cfg = cfg

    @staticmethod
    def file_download(url, path):
        """
        @summary: Download the file host and restore the file to host folder
        @parameter:
            url : the website to download the file
            path : the folder to restore the file on host
        @return : None
        """
        file_name = url.split("/")[-1]
        file_path = path + "/" + file_name
        if os.path.exists(file_path):
            print("INFO:The file [%s] is existed. Skip download" % file_path)
            return True
        cmd = "cd %s;wget -c " % path + url + ";echo $?"
        res_download = os.popen(cmd).read().strip()
        assert int(res_download) == 0, "Download file error [url]: %s" % url
        return True

    def video_sideload_play(self, file_name, file_path, target_path, time_last=0):
        """
        @summary: play the sideload video
        @parameter:
            file_name : file name
            file_path : file path in the host
            target_path : the target path to push the file on DUT
        @return: None
        """

        self._device.press.home()
        cmd = "adb push " + file_path + "/" \
        + file_name + " " + target_path + ";echo $?"
        print("INFO:Push %s to %s" % (file_name, target_path))
        re_push = os.popen(cmd).read().strip()
        print cmd, re_push
        assert int(re_push[-1]) == 0, "Push file error! mes: %s" % re_push

        os.system("adb logcat -c")
        cmd_play = "am start -a " + self._action + \
        " -n " + self._activity + " -d " + target_path + \
        "/" + file_name + " --el Timecode 0 --activity-clear-top;echo $?"
        print cmd_play
        re_play = g_common_obj.adb_cmd_capture_msg(cmd_play)
        assert int(re_play[-1]) == 0, "Play the video error! mes: %s" % re_play
        time.sleep(2)
        self.__play_video_log_check()
        time.sleep(time_last)

    def __play_video_log_check(self):
        """
        check the key log in logcat
        """
        key_log = "Displayed\ " + self._activity
        time.sleep(10)
        cmd = "adb logcat -d|grep -i %s" % key_log
        pipe = os.popen(cmd).read().strip()
        print "Search is %s" % pipe
        assert len(pipe) != 0, "Search is None"
        print("INFO:Get the key log in logcat!")

    def video_force_stop(self):
        """
        @summary: stop video process
        @return: None
        """
        cmd = "am force-stop " + self._package
        g_common_obj.adb_cmd(cmd)

    def error_popup_check(self):
        """
        @summary: check if error dialog popup
        @return: None
        """
        error_check = self._device(textContains="error").exists|\
        self._device(textContains="Can't").exists|\
        self._device(textContains="stop").exists
        assert not error_check, "error pop up!"
