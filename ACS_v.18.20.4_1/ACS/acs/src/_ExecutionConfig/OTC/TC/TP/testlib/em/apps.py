# -*- coding: utf-8 -*-
import os
import time
from constants_def import *
from tools import *


class AppBase(UIBase):

    def __init__(self, serial = None):
        UIBase.__init__(self, serial)
        self.package = None # str
        self.activity = None # str
        self.permissions = None # list
        self.conf_name = None # config name in artifactory

    def install(self):
        self.install_artifactory_app(self.conf_name, self.package)

    def launch(self):
        self.testDevice.launch_app_am(self.package, self.activity)

    def stop(self):
        assert self.package != None
        self.testDevice.stop_app_am(self.package)

    def clear_data(self):
        self.clear_app_data(self.package)

    def grant_permissions(self):
        if self.permissions:
            for permission in self.permissions:
                self.grant_permission(self.package, permission)

    def get_user_id(self):
        cmd = "dumpsys package %s | grep userId" % self.package
        msg = self.testDevice.adb_cmd_capture_msg(cmd)
        user_id = msg.split("=")[1]
        return user_id

    def reject_network_access(self):
        user_id = self.get_user_id()
        # create chain
        cmd = "iptables -N %s" % user_id
        self.testDevice.adb_cmd(cmd)
        # Clean chain
        cmd = "iptables -F %s" % user_id
        self.testDevice.adb_cmd(cmd)
        # associate chain
        cmd = "iptables -A OUTPUT -m owner --uid-owner %s -j %s" % (user_id, user_id)
        self.testDevice.adb_cmd(cmd)
        # reject network access
        cmd = "iptables -A %s -j REJECT" % user_id
        self.testDevice.adb_cmd(cmd)

    def check_app_running(self):
        if self.d(packageName=self.package).exists:
            return True
        top_activity = self.get_focused_app()
        if not top_activity:
            return False
        return self.package in(top_activity)

class CleanMusic(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.myskyspark.music"
        self.activity = ".OpenActivity"
        self.conf_name = "clean_music"
        self.permissions = [PERMISSION_READ_EXTERNAL_STORAGE, PERMISSION_WRITE_EXTERNAL_STORAGE]
        self.play_button_x = None
        self.play_button_y = None

    def get_play_button_position(self):
        self.launch()
        bounds = self.d(resourceId="com.myskyspark.music:id/playPause").bounds
        self.play_button_x = (bounds["left"] + bounds["right"]) / 2
        self.play_button_y = (bounds["top"] + bounds["bottom"]) / 2

    def play_audio(self, audio_uri):
        cmd = "am start -S -n %s/%s -d %s" % (self.package, self.activity, audio_uri)
        self.testDevice.adb_cmd(cmd)

    def press_play_pause_button(self):
        cmd = "input tap %s %s" % (self.play_button_x, self.play_button_y)
        self.testDevice.adb_cmd(cmd)

    def get_play_position(self):
        text = self.d(resourceId="com.myskyspark.music:id/position").text
        text_arr = text.split(":")
        m = text_arr[0]
        s = text_arr[1]
        return int(m) * 60 + int(s)


class VideoPlayer(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "videoplayer.app.instrument.otc.intel.com.otcvideoplayer"
        self.activity = "otc.intel.com.otcvideoplayer.InitActivity"
        self.conf_name = "video_player"
        #self.permissions = [PERMISSION_READ_EXTERNAL_STORAGE, PERMISSION_WRITE_EXTERNAL_STORAGE]

    def play_local_video(self, video_path):
        self.launch()
        time.sleep(3)
        self.d.press("menu")
        self.d(text="Open a local video").click()
        self.d(className="android.widget.EditText").set_text(video_path)
        self.d(text="OK").click()

    def play_http_video(self, video_url):
        self.launch()
        time.sleep(3)
        self.d.press("menu")
        self.d(text="Open a HTTP video").click()
        self.d(className="android.widget.EditText").set_text(video_url)
        self.d(text="OK").click()

    def get_play_position(self):
        text = self.d(resourceId="android:id/time_current").text
        text_arr = text.split(":")
        m = text_arr[0]
        s = text_arr[1]
        return int(m) * 60 + int(s)

    def play_pause(self, status=True):
        for _ in range(3):
            if status != self.d(resourceId="android:id/time_current").exists:
                break
            self.input_keyevent("KEYCODE_SPACE")
            #time.sleep(1)
        assert status != self.d(resourceId="android:id/time_current").exists, "Play Pause failed"


class SimpleGallery(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.simplemobiletools.gallery"
        self.activity = ".activities.SplashActivity"
        self.conf_name = "simple_gallery"
        self.play_button_x = None
        self.play_button_y = None
        self.permissions = [PERMISSION_READ_EXTERNAL_STORAGE, PERMISSION_WRITE_EXTERNAL_STORAGE]

    def select_gallery(self):
        if self.d(text="Open with").exists:
            self.d(text="Gallery").click()
            self.d(resourceId="android:id/button_once").click()
        elif self.d(text="Open with Gallery").exists:
            self.d(resourceId="android:id/button_once").click()

    def prepare_to_play_video(self):
        self.launch()
        time.sleep(5)
        self.d(text="Movies").click()
        self.d(resourceId="com.simplemobiletools.gallery:id/play_outline").click()
        self.select_gallery()
        bounds = self.d(resourceId="com.simplemobiletools.gallery:id/video_play_outline").bounds
        self.play_button_x = (bounds["left"] + bounds["right"]) / 2
        self.play_button_y = (bounds["top"] + bounds["bottom"]) / 2

    def press_play_pause_button(self):
        cmd = "input tap %s %s" % (self.play_button_x, self.play_button_y)
        self.testDevice.adb_cmd(cmd)

    def get_play_position(self):
        text = self.d(resourceId="com.simplemobiletools.gallery:id/video_curr_time").text
        text_arr = text.split(":")
        m = text_arr[0]
        s = text_arr[1]
        return int(m) * 60 + int(s)

    def am_play_video(self, video_uri):
        cmd = "am start -a android.intent.action.VIEW -t video/* -d %s" % (video_uri)
        self.testDevice.adb_cmd(cmd)


class Firefox(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "org.mozilla.firefox"
        self.activity = ".App"
        self.permissions = [PERMISSION_READ_EXTERNAL_STORAGE, PERMISSION_WRITE_EXTERNAL_STORAGE]
        self.conf_name = "firefox"

    def open_url(self, url):
        print "[Info] ---Open webpage %s" % url
        for i in range(5):
            self.launch()
            time.sleep(5)
            self.d(resourceId="org.mozilla.firefox:id/url_bar_title").click()
            #self.d(resourceId="org.mozilla.firefox:id/url_bar_title").set_text(url)
            self.input_text(url)
            time.sleep(2)
            if not self.d(text=url).exists:
                continue
            self.input_keyevent("KEYCODE_ENTER")
            time.sleep(5)
            if not self.d(packageName="org.mozilla.firefox").exists:
                continue
            break
        else:
            assert False , "Input URL failed"

    def play_video(self):
        if self.d(description="media control").exists:
            self.d(description="media control").click()
        else:
            self.input_keyevent("KEYCODE_SPACE")

    def play_audio(self):
        if self.d(description="play").exists:
            self.d(description="play").click()
        else:
            self.input_keyevent("KEYCODE_SPACE")


class AngryBirds(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.rovio.angrybirds.badk"
        self.activity = "com.rovio.fusion.App"
        self.conf_name = "angry_birds"


class KittenCat(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.koenigz.KittenCatSimulator"
        self.activity = "com.unity3d.player.UnityPlayerNativeActivity"
        self.conf_name = "kitten_cat"

    def launch(self):
        self.testDevice.launch_app_am(self.package, self.activity)
        time.sleep(10)
        bounds = self.d(packageName=self.package).bounds
        x = bounds["right"] * 0.76
        y = bounds["bottom"] * 0.63
        self.d.click(x, y)


class Riptidegp(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.vectorunit.blue"
        self.activity = ".Blue"
        self.conf_name = "riptidegp"
        self.permissions = [PERMISSION_READ_EXTERNAL_STORAGE, PERMISSION_WRITE_EXTERNAL_STORAGE]


class Messenger(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "sk.forbis.messenger"
        self.activity = "sk.forbis.recommendedapps.activities.MessengerActivity"
        self.conf_name = "messenger"


class MyTuner(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.appgeneration.itunerfree"
        self.activity = "com.appgeneration.ituner.application.activities.SplashActivity"
        self.conf_name = "mytuner"
        self.permissions = [PERMISSION_ACCESS_FINE_LOCATION]


class Telegram(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "org.telegram.messenger"
        self.activity = "org.telegram.ui.LaunchActivity"
        self.conf_name = "telegram"


class VLC(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "org.videolan.vlc"
        self.activity = "org.videolan.vlc.StartActivity"
        self.conf_name = "vlc"
        self.permissions = [PERMISSION_READ_EXTERNAL_STORAGE, PERMISSION_WRITE_EXTERNAL_STORAGE]


class RadioFM(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.radio.fmradio"
        self.activity = ".activities.PlayerActivityDrawer"
        self.conf_name = "radiofm"


class MusicPlayer(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.simplemobiletools.musicplayer"
        self.activity = "com.simplemobiletools.musicplayer.activities.SplashActivity"
        self.conf_name = "muisc_player"
        self.permissions = [PERMISSION_READ_EXTERNAL_STORAGE, PERMISSION_WRITE_EXTERNAL_STORAGE]


class Antutu4(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.antutu.ABenchMark"
        self.activity = ".ABenchMarkStart"
        self.conf_name = "antutu4"

    def launch(self):
        print "[info]--- Launch Antutu"
        AppBase.launch(self)
        time.sleep(15)
        for _ in range(5):
            if self.d(resourceId="com.antutu.ABenchMark:id/test_btn").exists:
                return True
            time.sleep(3)
        return False

    def run_test(self):
        print "[info]--- Run Antutu Test"
        self.clear_data()
        for i in range(5):
            if self.launch():
                self.d(resourceId="com.antutu.ABenchMark:id/test_btn").click()
                self.d(resourceId="com.antutu.ABenchMark:id/test_all_btn").click()
                break
        else:
            assert False, "Run Antutu test failed"

    def check_running(self):
        if self.d(packageName=self.package).exists:
            return True
        elif self.d(textStartsWith="Unfortunately").exists:
            self.d(text="OK").click()
        elif self.d(text="AnTuTu Benchmark has stopped").exists:
            if self.d(text="Close app").exists:
                self.d(text="Close app").click()
            else:
                self.d(text="Open app again").click()
        return False

    def get_score(self):
        text = None
        if self.d(text="Ranking", className="android.widget.TextView").exists:
            self.d(text="Test", className="android.widget.TextView").click()
            text = self.d(resourceId="com.antutu.ABenchMark:id/score_text").text
        if text:
            print "[info]--- Get Antutu score:", text
        return text


class GLBenchmark(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.glbenchmark.glbenchmark27"
        self.activity = "com.glbenchmark.activities.GLBenchmarkDownloaderActivity"
        self.conf_name = "glbenchmark"

    def launch(self):
        print "[info]--- Run GLbenchmark27"
        AppBase.launch(self)
        time.sleep(3)
        self.d(text="Performance Tests").click()
        time.sleep(2)

    def select_all_tests(self):
        if self.d(text="All").exists:
            rect = self.d(text="All").info["bounds"]
            x = (rect["left"] + rect["right"])/2
            y = (rect["top"] + rect["bottom"])/2
            self.testDevice.adb_cmd("input tap %d %d" % (x, y)) # uiautomator click does not work
            if self.d(text="Low memory").exists:
                self.d(text="No").click()

    def select_offscreen_etc1_test(self):
        self.d(resourceId="com.glbenchmark.glbenchmark27:id/listView1").scroll.to(text="C24Z16 Offscreen ETC1")
        self.d(text="C24Z16 Offscreen ETC1").right(className="android.widget.CheckBox").click()
        if self.d(text="Low memory").exists:
            self.d(text="No").click()

    def run_tests(self):
        self.d(text="Start").click()
        time.sleep(2)
        if self.d(text="Start").exists:
            return False
        return True

    def check_running(self):
        if self.d(packageName=self.package).exists:
            if not self.d(text="Results").exists:
                return True
            return False
        return False

    def get_score(self):
        if self.d(resourceId="com.glbenchmark.glbenchmark27:id/textViewScore").exists:
            text = self.d(resourceId="com.glbenchmark.glbenchmark27:id/textViewScore").text
            score = text.split()[0]
            print "[info]--- Get benchmark score:", score
            return score
        return None


class Maps(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.google.android.apps.maps"
        self.activity = "com.google.android.maps.MapsActivity"
        self.permissions = [PERMISSION_ACCESS_FINE_LOCATION]

    def launch(self):
        print "[info]--- Launch Maps"
        AppBase.launch(self)
        time.sleep(10)
        if self.d(text="SKIP").exists:
            self.d(text="SKIP").click()
        elif self.d(text="Skip").exists:
            self.d(text="Skip").click()
        time.sleep(2)
        if self.d(resourceId="com.google.android.apps.maps:id/tutorial_side_menu_got_it").exists:
            self.d(resourceId="com.google.android.apps.maps:id/tutorial_side_menu_got_it").click()


class GPSTools(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "appinventor.ai_Samuel_Seyller.GPSTools"
        self.activity = ".Screen1"
        self.conf_name = "gps_tools"

    def get_location_info(self):
        self.launch()
        for i in range(5):
            time.sleep(10)
            if self.d(className="android.widget.TextView", textStartsWith="Latitude").exists:
                text = self.d(className="android.widget.TextView", textStartsWith="Latitude").text
                text_list = text.splitlines()
                latitude = text_list[0].split()[1]
                longitude = text_list[1].split()[1]
                return float(latitude), float(longitude)
        else:
            return 0, 0


class GeoLocation(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.ascentfuturetech.location"
        self.activity = ".MainActivity"
        self.conf_name = "geo_location"
        self.permissions = [PERMISSION_ACCESS_FINE_LOCATION]

    def update_location(self):
        self.launch()
        time.sleep(3)
        self.d(resourceId="com.ascentfuturetech.location:id/btnLocationUpdates").click()
        time.sleep(10)

    def get_latitude_longitude(self):
        self.update_location()
        self.launch()
        time.sleep(3)
        for i in range(5):
            self.d(resourceId="com.ascentfuturetech.location:id/btnShowLocation").click()
            text = self.d(resourceId="com.ascentfuturetech.location:id/lblLocation").text
            if text:
                text_list = text.split(",")
                latitude = text_list[0].strip()
                longitude = text_list[1].strip()
                print "Lati: %s, Long: %s" % (latitude, longitude)
                return float(latitude), float(longitude)
            time.sleep(5)
        else:
            return None


class CPU_Stats(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "jp.takke.cpustats"
        self.activity = ".PreviewActivity"
        self.conf_name = "cpu_stats"

    def launch(self):
        print "[info]--- Launch CPU Stats"
        AppBase.launch(self)
        time.sleep(5)

    def get_cpu_cores(self):
        counts = self.d(resourceId="jp.takke.cpustats:id/coreText").count
        print "[Info]--- CPU cores in CPU Stats:", counts
        return counts

    def get_cpu_freq(self):
        text = self.d(resourceId="jp.takke.cpustats:id/freqText1").text
        freq = text.split(":")[1].strip()
        print "[Info]--- CPU freq:", freq
        return freq

    def get_cpu_usage_percentage(self):
        text = self.d(resourceId="jp.takke.cpustats:id/core1").child(resourceId="jp.takke.cpustats:id/coreText").text
        percentage = text.split()[1]
        print "[Info]--- CPU usage:", percentage
        return percentage


class CPU_Monitor(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.glgjing.stark"
        self.activity = ".HomeActivity"
        self.conf_name = "cpu_monitor"

    def launch(self):
        print "[info]--- Launch CPU Monitor"
        AppBase.launch(self)
        time.sleep(10)
        if self.d(text="Never").exists:
            self.d(text="Never").click.wait()
        assert self.d(resourceId="com.glgjing.stark:id/pie_percent").exists

    def get_cpu_cores(self):
        counts = self.d(resourceIdMatches="com.glgjing.stark:id/cpu_percent_.*").count
        print "[Info]--- CPU cores in CPU Monitor:", counts
        return counts

    def get_cpu_freq_percent(self):
        percent = self.d(resourceId="com.glgjing.stark:id/pie_percent").text
        print "[Info]--- CPU freq percent:", percent
        return int(percent)


class Androtics(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.xsoftstudio.androtics"
        self.activity = ".MainActivity"
        self.conf_name = "androtics"

    def launch(self, filed = ""):
        print "[info]--- Launch Androtics"
        super(Androtics, self).launch()
        time.sleep(5)
        if filed:
            self.d(text=filed).click()
            time.sleep(5)

    def get_cpu_cores(self):
        text = self.d(text="CPU Cores").right(className="android.widget.TextView").text
        cores = text.split()[0]
        print "[Info]--- CPU cores in Androtics:", cores
        return int(cores)

    def get_cpu_freq(self):
        text = self.d(text="Current Frequency").right(className="android.widget.TextView").text
        freq_list = [line.split()[2] for line in text.splitlines()]
        return [int(freq) for freq in freq_list]


class Chrome(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.android.chrome"
        self.activity = "com.google.android.apps.chrome.Main"
        self.permissions = [PERMISSION_READ_EXTERNAL_STORAGE, PERMISSION_WRITE_EXTERNAL_STORAGE]
        self.conf_name = "chrome"

    def close_tabs(self):
        self.testDevice.adb_cmd("rm -rf /data/data/com.android.chrome/app_tabs/")

    def launch(self):
        super(Chrome, self).launch()
        time.sleep(5)
        for i in range(10):
            if self.d(resourceId="com.android.chrome:id/url_bar").exists:
                return
            if self.d(resourceId="com.android.chrome:id/terms_accept").exists:
                self.d(resourceId="com.android.chrome:id/terms_accept").click.wait()
            if self.d(resourceId="com.android.chrome:id/negative_button").exists:
                self.d(resourceId="com.android.chrome:id/negative_button").click.wait()
            if self.d(text="Done").exists:
                self.d(text="Done").click.wait()
            if self.d(text="Next").exists:
                self.d(text="Next").click.wait()
            time.sleep(2)
        assert False , "Launch Chrome failed"

    def open_url(self, url):
        print "[Info] ---Open webpage %s." % url
        #cmd = "am start -S -n %s/%s -d %s" % (self.package, self.activity, url)
        #self.testDevice.adb_cmd(cmd)
        bounds = self.d(resourceId="com.android.chrome:id/url_bar").bounds
        x = (bounds["left"] + bounds["right"]) / 2
        y = (bounds["top"] + bounds["bottom"]) / 2
        self.d.click(x, y)
        self.input_text(url)
        if self.d(resourceId="com.android.chrome:id/url_bar", text=url).exists:
            self.input_keyevent("KEYCODE_ENTER")
            return
        assert False , "Input URL failed"

    def play_video(self):
        if self.d(description="media control").exists:
            self.d(description="media control").click()
        else:
            self.input_keyevent("KEYCODE_SPACE")

    def play_audio(self):
        if self.d(description="play").exists:
            self.d(description="play").click()
        else:
            self.input_keyevent("KEYCODE_SPACE")


class Camera(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.android.camera2"
        self.activity = "com.android.camera.CameraLauncher"
        self.permissions = [PERMISSION_CAMERA, PERMISSION_ACCESS_FINE_LOCATION,
                            PERMISSION_RECORD_AUDIO, PERMISSION_WRITE_EXTERNAL_STORAGE]

    def launch(self, mode = "Camera"):
        component = "%s/%s" % (self.package, self.activity)
        if mode == "Camera":
            action = "android.media.action.IMAGE_CAPTURE"
        else:
            action = "android.media.action.VIDEO_CAPTURE"
        cmd = "am start -S -n %s -a %s" % (component, action)
        self.testDevice.adb_cmd(cmd)
        for i in range(10):
            if self.d(resourceId = "com.android.camera2:id/mode_options_overlay").exists:
                return
            if self.d(text = "Unfortunately, Camera has stopped.").exists:
                self.d(text = "OK").click()
                assert False, "Camera force close!"
            if self.d(text = "NEXT").exists:
                self.d(text = "NEXT").click()
            if self.d(resourceId="com.android.camera2:id/ok_button").exists:
                self.d(resourceId="com.android.camera2:id/ok_button").click.wait()
            time.sleep(2)
        assert False, "open camera failed"

    def click_shutter_button(self, mode = "Camera"):
        """
        mode: 'Camera' or 'Video'
        """
        assert self.d(resourceId="com.android.camera2:id/shutter_button", enabled=True).exists
        self.d(resourceId="com.android.camera2:id/shutter_button").click.wait()
        if self.d(resourceId="com.android.camera2:id/done_button").exists:
            self.d(resourceId="com.android.camera2:id/done_button").click.wait()


class Email(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.google.android.gm"
        self.activity = "com.google.android.gm.ConversationListActivityGmail"

    def open_email(self):
        self.launch()
        time.sleep(5)
        for i in range(3):
            self.set_screen_status("on")
            if self.d(packageName = self.package).exists:
                return
            time.sleep(3)
        assert False


class EMTools(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.intel.yang.emtools"
        self.conf_name = "em_tool"

class EMToolsCharger(EMTools):

    def __init__(self, serial = None):
        EMTools.__init__(self, serial)
        self.activity = ".ChargerActivity"
        from energy import Energy
        self.source = Energy().get_charger_type_path()
        self.target = "/sdcard/type"
        self.copy_interval = 0.2

    def start_monitor(self):
        self.launch()
        time.sleep(2)
        from nohup_process import NohupProcess
        script_path = os.path.join(os.path.dirname(__file__), "device_scripts", "copy_charger_type.sh")
        string_args = "%s %s %s" % (self.source, self.target, self.copy_interval)
        self.np = NohupProcess(self.testDevice, script_path, "/data/local/tmp/", string_args)
        self.np.start()
        self.d(resourceId="com.intel.yang.emtools:id/input_charger_type_path").set_text(self.target)
        self.d(resourceId="com.intel.yang.emtools:id/start_monitor").click()

    def stop_monitor(self):
        self.np.stop()

    def get_history(self):
        return self.d(resourceId="com.intel.yang.emtools:id/history").text

class EMToolsScreen(EMTools):

    def __init__(self, serial = None):
        EMTools.__init__(self, serial)
        self.activity = ".ScreenOnOffActivity"

    def start_monitor(self):
        self.launch()
        time.sleep(2)
        self.set_screen_status("off")
        time.sleep(2)

    def stop_monitor(self):
        self.set_screen_status("on")
        self.unlock_screen()

    def get_history(self):
        return self.d(resourceId="com.intel.yang.emtools:id/screen_on_off_history").text

class EMToolsDownload(EMTools):

    def __init__(self, serial = None):
        EMTools.__init__(self, serial)
        self.activity = ".DownloadActivity"
        self.download_folder = "/mnt/sdcard/emDownload"

    def download_file(self, url):
        print "[info]--- Start to Download %s" % url
        self.launch()
        time.sleep(2)
        self.d(resourceId="com.intel.yang.emtools:id/input_url").set_text(url)
        self.d(resourceId="com.intel.yang.emtools:id/start_downloading").click()
        time.sleep(2)

    def clean_download_folder(self):
        cmd = "rm -rf %s/*" % self.download_folder
        self.testDevice.adb_cmd(cmd)

    def get_download_file_size(self):
        cmd = "du -sk %s" % self.download_folder
        size = self.testDevice.adb_cmd_capture_msg(cmd).split()[0]
        if size.isdigit():
            print "[info]--- Download file size: %sKB" % size
            return int(size)
        else:
            print "[info]--- %s dir does not exist" % self.download_folder
            return 0

    def get_download_file_status(self, time_str = None):
        if time_str:
            cmd = "logcat -t '%s' -s EMDownloadTool | grep -c Failed" % time_str
        else:
            cmd = "logcat -d -s EMDownloadTool | grep -c Failed"
        count = self.testDevice.adb_cmd_common(cmd)
        print "[info]--- Failed number: %s" % count
        if int(count) > 0:
            return False
        return True


class GarageModeTest(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.intel.garagetest"
        self.activity = ".MainActivity"
        self.conf_name = "garage_mode_test"
        self.permissions = [PERMISSION_READ_EXTERNAL_STORAGE, PERMISSION_WRITE_EXTERNAL_STORAGE]
        self.download_folder = "/mnt/sdcard/GarageTest"

    def clean_download_folder(self):
        cmd = "rm -rf %s/*" % self.download_folder
        self.testDevice.adb_cmd(cmd)

    def launch(self, latency, url):
        component = "%s/%s" % (self.package, self.activity)
        cmd = "am start -S -n %s --ei latency %s --es url %s" % (component, latency, url)
        self.testDevice.adb_cmd(cmd)

    def schedule_job(self, latency, url):
        self.launch(latency, url)
        time.sleep(3)
        self.d(resourceId="com.intel.garagetest:id/verify_download_url").click()
        time.sleep(3)
        text = self.d(resourceId="com.intel.garagetest:id/textview_download_urls").text
        assert text != ""
        for line in text.splitlines():
            line_arr = line.split()
            assert len(line_arr) == 2 and line_arr[1] == "OK"
        assert self.d(resourceId="com.intel.garagetest:id/schedule_download_manager_job").enabled
        self.d(resourceId="com.intel.garagetest:id/schedule_download_manager_job").click()

    def get_job_event_time(self, event):
        event_dict = {"start" : "onStartJob", "finished": "job finished"}
        #cmd = "logcat -s -d %s" % "GarageTest"
        cmd = "cat %s/Log.txt" % self.download_folder
        log = self.testDevice.adb_cmd_capture_msg(cmd)
        check_str = event_dict[event]
        for line in log.splitlines():
            if check_str in line:
                log_time = line.split()[1]
                h, m, s = log_time.split(":")
                return int(h), int(m), int(s)
        return None


class CPUPrimeBenchmark(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.obkircherlukas.cpuprimebenchmark"
        self.activity = ".MainActivity"
        self.conf_name = "cpu_prime"

    def cpu_load(self):
        self.launch()
        time.sleep(5)
        self.d(text = "Stress test").click()
        time.sleep(10)


class Clock(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.google.android.deskclock"
        self.activity = "com.android.deskclock.DeskClock"

    def get_device_seconds_since_Epoch(self):
        device_time_str = self.testDevice.adb_cmd_capture_msg("date +%Y:%m:%d:%H:%M:%S")
        ts = [int(item) for item in device_time_str.split(":")]
        ts.extend([0, 0, 0])
        seconds = time.mktime(ts)
        print "[info]--- device time %s, seconds %d" % (device_time_str, seconds)
        return seconds

    def get_delay_time(self, delay):
        seconds = self.get_device_seconds_since_Epoch()
        st = time.localtime(seconds + delay)
        print "[info]--- %ds later is %02d:%02d:%02d" % (delay, st.tm_hour, st.tm_min, st.tm_sec)
        return st.tm_hour, st.tm_min

    def launch_alarm(self):
        self.launch()
        time.sleep(2)
        for i in range(5):
            if self.d(description="Alarm").exists:
                self.d(description="Alarm").click()
                break
            time.sleep(1)

    def get_circle_data(self):
        rect = self.d(resourceId="android:id/radial_picker").info["bounds"]
        left = int(rect["left"])
        right = int(rect["right"])
        top = int(rect["top"])
        bottom = int(rect["bottom"])
        center_x = (left + right) / 2
        center_y = (top + bottom) / 2
        length = (right - left) / 2
        hight = (bottom - top) / 2
        if length > hight:
            r = hight
        else:
            r = length
        return center_x, center_y, r

    def add_alarm(self, hour, minite):
        print "[info]--- Set alarm time %02d:%02d" % (hour, minite)
        self.d(description="Add alarm").click()
        time.sleep(1)
        self.d(resourceId = "android:id/radial_picker").child(description=hour).click()
        # set minute
        import math
        center_x, center_y, r = self.get_circle_data()
        time.sleep(1)
        adjust_list = [0, 0.5, -0.5, -1 , 1]
        for i in range(len(adjust_list)):
            adjust_min = (minite + adjust_list[i]) % 60
            angle = adjust_min * 3.14159 / 30
            r_minite = r * 2.0 / 3
            x = int(center_x + r_minite * math.sin(angle))
            y = int(center_y - r_minite * math.cos(angle))
            self.d.click(x, y)
            if self.d(resourceId="android:id/minutes", text="%02d" % minite).exists:
                break
        else:
            assert False
        self.d(text="OK").click()

    def add_alarm_by_delay_time(self, delay):
        hour, minite = self.get_delay_time(delay)
        self.add_alarm(hour, minite)


class BatteryWidget(AppBase):

    def __init__(self, serial = None):
        AppBase.__init__(self, serial)
        self.package = "com.elvison.batterywidget"
        self.activity = ".BatteryInfoDialog"
        self.conf_name = "battery_widget"

    def launch(self):
        AppBase.launch(self)
        time.sleep(5)
        for i in range(5):
            if self.d(text="Battery Information").exists:
                return
            if self.d(text="Important usage notes").exists:
                self.d(text="OK").click()
            time.sleep(2)

    def get_battery_level(self):
        self.launch()
        if not self.d(resourceId="com.elvison.batterywidget:id/info_data_level").exists:
            time.sleep(2)
            if self.d(resourceId="com.elvison.batterywidget:id/showHideTogglesButton").exists:
                self.d(resourceId="com.elvison.batterywidget:id/showHideTogglesButton").click()
            time.sleep(2)
        text = self.d(resourceId="com.elvison.batterywidget:id/info_data_level").text
        level = text.split("%")[0]
        print "[info]--- Get battery level from battery widget: %s" % level
        return int(level)

