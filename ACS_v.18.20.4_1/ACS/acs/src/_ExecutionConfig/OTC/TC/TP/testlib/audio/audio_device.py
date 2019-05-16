'''
Created on Dec 18, 2015

@author: bob
'''
from testlib.util.device import TestDevice
from testlib.audio.audio_impl import AudioImpl
from testlib.audio import resource
from testlib.audio.audio_log import AudioLogger
from testlib.audio.audiostub import AudioStubApp, AudioStubRPC
import datetime
import tempfile
import time
import os
import re
from testlib.util.common import g_common_obj


class AudioDevice(TestDevice):
    '''
    Audio device obj for Audio test
    '''
    device_instances = {}# The instance of audio devices
    MEDIA_PATH = "/sdcard/otctest/"
    SETTINGS_PKG_NAME = "com.android.settings"
    SETTOMGS_ACTIVITY_NAME = ".Settings"
    DEFAULT_CONTENTS = ["mp3_sample1.mp3", "mp3_sample2.mp3", "mp3_sample3.mp3"]
    logger = AudioLogger.getLogger()

    def __init__(self, *args, **kws):
        super(AudioDevice, self).__init__(*args, **kws)
        self.uia_device = self.get_device()
        self.TAG = "Device_%s"%self.serial if self.serial else "Device_Default"

        #self.systemui = None
        #self.system = None
        #self.common = None
        self.init_audio_stub()
        # FIXME: google music is only applied to default music
        self.gmusic = AudioImpl(device = self)

        self.tmpfiles = []
        self.dut_tmpfiles = []

    def __new__(cls, serial= None, adb_server_host=None, adb_server_port=None):
        if serial not in cls.device_instances:
            orig = super(AudioDevice, cls)
            cls.device_instances.update({serial: orig.__new__(cls, serial, adb_server_host, adb_server_port)})
        return cls.device_instances[serial]

    def __str__(self, *args, **kwargs):
        return self.TAG

    def clean_up(self):
        '''
        do some clean if device not used any more
        '''
        self.adb_cmd("rm -rf %s"%self.MEDIA_PATH)
        for f in self.tmpfiles:
            os.system('rm -rf ' + f)
        for f in self.dut_tmpfiles:
            self.adb_cmd('rm -rf ' + f)

        if hasattr(self, 'rpc'):
            self.rpc.stop()
        self.stop_app_am(self.gmusic.PACKAGE_NAME_PLAY_MUSIC)

    def log_debug(self, msg):
        self.logger.debug('%s: %s' % (self.TAG, msg))

    def log_error(self, msg):
        self.logger.error('%s: %s' % (self.TAG, msg))

    """
    def __setattr__(self, name, value):
        if hasattr(self, name):
            raise Exception("Device obj already have %s attribute"%name)
        setattr(self, name, value)
    """

    def init_audio_stub(self):
        """
        Init audio stub
        """
        self.log_debug("Init audio stub")
        self.app = AudioStubApp(self)
        self.app.install()
        self.rpc = AudioStubRPC(self)
        self.rpc.start()
        self.rpc.initEnv()

    def _refresh_storage(self):
        """
        Refresh storage by am broadcase
        """
        self.log_debug("Refresh storage")
        cmd = " ".join(["am broadcast -a android.intent.action.MEDIA_MOUNTED",
                        "--ez read-only false",
                        "-d file://sdcard"])
        self.adb_cmd(cmd)
        time.sleep(1)

    def launch_play_music_n_play(self, song, effect=False):
        '''
        Launch Google Play Music and play *song*

        params:
            song: path or name of the song, can have file extension
            effect: whether to enable music effect
        '''
        self._refresh_storage()
        gmusic = self.gmusic
        self.stop_app_am(gmusic.PACKAGE_NAME_PLAY_MUSIC)
        self.adb_cmd("pm clear %s" % gmusic.PACKAGE_NAME_PLAY_MUSIC)
        self.unlock_screen()  # Ensure DUT unlocked

        if self.get_android_version() == "O":
            gmusic.launch_music_am()
        else:
            gmusic.enterPlayPageFromHome()

        time.sleep(3)
        #if effect:
         #   gmusic.setEffect()
        gmusic.enterSongsPage()
        song_name = os.path.splitext(os.path.basename(song))[0]
        gmusic.playMusic(song_name)
        time.sleep(3)
        gmusic.setRepeat(1)

    def deploy_play_music_content(self, files=[]):
        if not files:
            files = self.DEFAULT_CONTENTS
        for f in files:
            local_path = resource.get_media_content(f)
            push_path = os.path.join(self.MEDIA_PATH, f)
            self.push_file(local_path, push_path)
        self._refresh_storage()

    def clean_play_music_content(self):
        """
        Clean up audio media content foler
        """
        self.log_debug("rm audio media contents")
        cmd = "rm -rf %s" % self.MEDIA_PATH
        self.adb_cmd(cmd)
        self.refresh_storage()

    def launch_sound_settings(self):
        """
        Enter settings --> sound & notification
        """
        self.adb_cmd("am start -W com.android.settings")
        if not self.scroll_n_click("Sound & notification"):
            self.scroll_n_click("Sound")

    def enable_screen_lock(self):
        """
        Enable screen lock in settings
        """
        self.enable_lock_screen()
        self.back_home()
        time.sleep(1)
        self.enable_screen_lock_sound()
        self.back_home()
        time.sleep(1)

        # when screenlock None --> Swipe
        # the first lock_screen request can't trigger lock sound
        # do a round of lock/unlock to skip this.
        self.lock_screen()
        time.sleep(1)
        self.unlock_screen()

    def check_screen_lock_sound(self):
        """
        Check screen lock sound
        """
        self.log_debug("Check screen lock sound")
        self.lock_screen()
        time.sleep(4)
        assert self.rpc.isStreamActive("system", 8000), "Can't hear screen lock sound"
        time.sleep(3)
        self.unlock_screen()
        time.sleep(2)

    def check_volume_in_settings(self, mode = 1):
        """
        Check volume up/down by change vol in settings --> sound & notification
        Mode:
        1 --> Media volume
        2 --> Alarm volume
        3 --> Ring volume
        """
        if mode == 1:
            mode_str = "Media volume"
            stream = 'music'
        elif mode == 2:
            mode_str = "Alarm volume"
            stream = 'alarm'
        elif mode == 3:
            mode_str = "Ring volume"
            stream = 'ring'
        else:
            raise Exception("Unknow mode of volume. Only support 1,2,3")

        self.log_debug("Check sound in by change vol in settings, mode: %s"%mode_str)
        self.launch_sound_settings()
        self.uia_device(resourceId="android:id/seekbar").wait.exists()

        seek_bar = self.uia_device(text = mode_str).down(resourceId="android:id/seekbar").info["bounds"]
        y = (seek_bar["top"] + seek_bar["bottom"])/2
        start = seek_bar["left"] + 10
        end = seek_bar["right"] -10
        self.uia_device.click((end - start)/2, y)# volume middle
        time.sleep(1)
        start_vol = self.rpc.getStreamVolume(stream)
        self.log_debug("Set vol middle, vol is %d"%start_vol)

        self.uia_device.click((end - start)/4, y)# Volume down
        time.sleep(1)
        current_vol = self.rpc.getStreamVolume(stream)
        self.log_debug("Vol down, vol is %d"%current_vol)

        assert current_vol < start_vol,\
            "Volume down failed by settings. Before: %d, after: %d"%(current_vol, start_vol)

        self.uia_device.click((end - start)/2, y)#Volume middle
        time.sleep(1)
        last_vol = self.rpc.getStreamVolume(stream)
        self.log_debug("Vol up, vol is %d"%last_vol)
        assert last_vol > current_vol,\
            "Volume up failed by settings. Before: %d, after: %d"%(current_vol, last_vol)

    def set_option(self, opt_name, enable=True):
        """
        Set option on off in settings
        """
        self.log_debug("Set Option: %s, enable: %s" % (opt_name, enable))

        if self.uia_device(scrollable=True).exists:
            self.uia_device(scrollable=True).scroll.to(text=opt_name)
        opt_text = self.uia_device(className="android.widget.TextView",
                          text=opt_name)

        widget = opt_text.right(className="android.widget.CheckBox")
        if not widget:
            widget = opt_text.right(className="android.widget.Switch")

        enabled = widget.checked
        if enable != enabled:
            opt_text.click.wait()

    def scroll_n_click(self, target):
        '''
        Scroll to UI element and click
        '''
        if self.uia_device(scrollable=True).exists:
            self.uia_device(scrollable=True).scroll.to(text=target)
        wgt = self.uia_device(text = target)
        if not wgt.exists:
            self.logger.debug("Wgt %s not found!"%target)
            return False
        else:
            self.uia_device(text=target).click.wait()
            return True

    def get_android_version(self):
        '''
        return Android verison of DUT
        '''
        from distutils.version import LooseVersion

        def cmp_version(v1, v2):
            return cmp(LooseVersion(v1), LooseVersion(v2))

        attr = "__android_version__"
        version = getattr(self, attr, None)
        if not version:
            prop = "ro.build.version.release"
            buf = self.adb_cmd_capture_msg("getprop " + prop)
            v_str = buf.strip()
            if len(v_str) == 1 and v_str.isalpha():  # version in alpha
                version = v_str
            else:  # version in number
                if v_str.startswith("8."):
                    version = "O"
                elif v_str.startswith("7."):
                    version = "N"
                elif v_str.startswith("6."):
                    version = "M"
                else:
                    setattr(self, attr, version)
            self.log_debug("android verson: %s" % version)
        return version

    def screenshot(self, fname = None):
        ''' failure handler when exception happens '''
        try:
            # take screen shoot
            now = datetime.datetime.now()
            fname = fname if fname else "screenshot%s.png" % now.strftime("%H%M%S")
            fpath = os.path.join(g_common_obj.globalcontext.user_log_dir, fname)
            self.log_debug("Saving screenshot: " + fpath)
            self.uia_device.screenshot(fpath)
        except Exception, e:
            self.log_error(str(e))

    def set_alarm(self, minutes):
        """
        @Sammary: Set an alarm after minutes
        @param  minute: int,the delay time of minute
        """
        time_delay = datetime.timedelta(0, minutes*60, 0)
        cmd = "date +%Y:%m:%d:%H:%M:%S"
        self.root_on_device()  # set alarm Intent require root permission
        time.sleep(3)
        dut_time_str = self.adb_cmd_capture_msg(cmd).strip().split(":")
        assert len(dut_time_str) == 6, \
            "Get DUT time failed. Actually get %s" % " ".join(dut_time_str)
        dut_time = datetime.datetime(*map(int, dut_time_str))
        alarm_time = dut_time + time_delay
        self.log_debug("DUT time: %s, alarm time: %s"
                       % (str(dut_time), str(alarm_time)))
        set_alarm_cmd = ["am start -a android.intent.action.SET_ALARM",
                 "--ei android.intent.extra.alarm.HOUR %s" % alarm_time.hour,
                 "--ei android.intent.extra.alarm.MINUTES %s" % alarm_time.minute,
                 "--ez android.intent.extra.alarm.SKIP_UI true"]
        self.adb_cmd(" ".join(set_alarm_cmd))
        time.sleep(1)

    def wait_alarm(self, timeout=180):
        """
        @summary: Wait for alarm comes. timeout = seconds
        @param timeout: int, timeout second
        """
        self.log_debug("Waiting alarm for %ds"%timeout)
        start = time.time()
        time_wait = 1
        while time.time() - start < timeout:
            if self.rpc.isStreamActive("alarm", 0):
                return True
            time.sleep(time_wait)
        return False

    def delete_alarms(self):
        """
        @summary: Delete all enabled alarms
        """
        alarm_pkg = "com.google.android.deskclock"
        self.stop_app_am(alarm_pkg)
        self.adb_cmd("pm clear %s" % alarm_pkg)

    def refresh_storage(self):
        return self._refresh_storage()

    def enable_lock_screen(self):
        """Set security --> screen lock to swipe"""
        self.log_debug("Set screen lock to swipe")
        self.launch_app_am(self.SETTINGS_PKG_NAME, self.SETTOMGS_ACTIVITY_NAME)
        #Be compatible with Sofia
        #Optimize for BXTP
        if self.uia_device(scrollable = True).exists:
            self.uia_device(scrollable = True).scroll.vert.to(text = "Security")
        self.uia_device(textContains = "Security").click.wait()
        self.uia_device(text = "Screen lock").click.wait()
        self.uia_device(text = "Swipe").click.wait()

    def enable_screen_lock_sound(self):
        """Enable screen lock sound in settings"""
        self.log_debug("Enalbe screen lock sound")
        self.launch_sound_settings()
        if self.uia_device(textContains='Other sounds'):
            self.uia_device(textContains='Other sounds').click()
        else:
            self.scroll_n_click("Advanced")

        self.set_option("Screen locking sounds", True)

    def lock_screen(self):
        """
        Lock screen
        """
        self.log_debug("Device sleep")
        self.uia_device.sleep()

    def unlock_screen(self):
        """
        Unlock screen by via input keyevent 82
        """
        self.log_debug("Wake up device by keyevent 82")
        cmd = 'input keyevent 82'
        lockicon = 'com.android.systemui:id/lock_icon'
        self.adb_cmd(cmd)
        # unlock again if swipe lock screen is detected.
        if self.uia_device(resourceId = lockicon).exists:
            self.adb_cmd(cmd)

    def mktemp_wav(self, prefix=''):
        wav = tempfile.mktemp(prefix=prefix, suffix='.wav')
        self.tmpfiles.append(wav)
        return wav

    def record_call(self, duration=0):
        '''
        record call using audiostub

        @duration: if duration is 0, means async mode

        return: if duration != 0, return record wav, else None
        '''
        wav = self.mktemp_wav("record_call")
        dut_path = os.path.join('/sdcard/', os.path.basename(wav))
        self.dut_tmpfiles.append(dut_path)
        self.rpc.startCallRecording(dut_path)
        self.log_debug("start record call to " + dut_path)
        if duration:
            time.sleep(duration)
            self.log_debug("stop record")
            self.rpc.stopCallRecording()
            self.pull_file(wav, dut_path)
            return wav
        else:
            self._record_path = dut_path

    def stop_record_call(self):
        '''
        stop recording if previous using async mode to 'record_call'

        return: record wav file
        '''
        self.log_debug("stop record")
        self.rpc.stopCallRecording()
        wav = self.mktemp_wav("record_call")
        self.pull_file(wav, self._record_path)
        return wav

    def start_record(self, duration=0):
        '''
        record call using audiostub

        @duration: if duration is 0, means async mode

        return: if duration != 0, return record wav, else None
        '''
        wav = self.mktemp_wav("record_call")
        dut_path = os.path.join('/sdcard/', os.path.basename(wav))
        self.dut_tmpfiles.append(dut_path)
        self.rpc.startRecording(dut_path)
        self.log_debug("start record call to " + dut_path)
        if duration:
            time.sleep(duration)
            self.log_debug("stop record")
            self.rpc.stopRecording()
            self.pull_file(wav, dut_path)
            return wav
        else:
            self._record_path = dut_path

    def stop_record(self):
        '''
        stop recording if previous using async mode to 'record_call'

        return: record wav file
        '''
        self.log_debug("stop record")
        self.rpc.stopRecording()
        wav = self.mktemp_wav("record_call")
        self.pull_file(wav, self._record_path)
        return wav

    def record_mic(self, duration=0):
        '''
        record sound from MIC

        @duration: if duration is 0, means async mode

        return: if duration != 0, return record wav, else None
        '''
        wav = self.mktemp_wav("record_mic")
        dut_path = os.path.join('/sdcard/', os.path.basename(wav))
        self.dut_tmpfiles.append(dut_path)
        self.rpc.startWavRecording(dut_path, 8000, 1)
        self.log_debug("start record MIC to " + dut_path)
        if duration:
            time.sleep(duration)
            self.log_debug("stop record")
            self.rpc.stopWavRecording()
            self.pull_file(wav, dut_path)
            return wav
        else:
            self._record_mic_path = dut_path

    def stop_record_mic(self):
        '''
        stop recording if previous using async mode to 'record_mic'

        return: record wav file
        '''
        self.log_debug("stop record MIC")
        self.rpc.stopWavRecording()
        wav = self.mktemp_wav("record_mic")
        self.pull_file(wav, self._record_mic_path)
        return wav

    def play_file(self, media, stream="music", sync=True):
        '''
        play media file in DUT

        params:
            media: media file path in Host PC, will be pushed to DUT
            stream: play file on which stream, default 'music'
            sync: if True, wait for media playing done
        '''
        dut_path = os.path.join('/sdcard/', os.path.basename(media))
        self.push_file(media, dut_path)
        self.rpc.playFileOnStream(dut_path, stream, False)  # no loop
        time.sleep(1)
        if sync:
            while self.rpc.isPlaying():
                time.sleep(1)

    def get_stream_info(self, stream):
        '''
        Get stream info through dumpsys audio
        current support below streams:
        STREAM_VOICE_CALL
        STREAM_SYSTEM
        STREAM_RING
        STREAM_MUSIC
        STREAM_ALARM
        STREAM_NOTIFICATION
        STREAM_BLUETOOTH_SCO
        STREAM_SYSTEM_ENFORCED
        STREAM_DTMF
        STREAM_TTS
        '''
        data = self.adb_cmd_capture_msg('dumpsys audio')
        p = re.compile(
            '\-\s+%s:\r\n\s+Muted:\s+(?P<Muted>.+)\r\n\s+.+\r\n\s+.+\r\n\s+.+\r\n\s+Devices:\s+(?P<Devices>.+)\r\n'
            %stream)
        m = re.search(p, data)
        if m:
            return m.groupdict()
        else:
            return {}

    def skip_use_hints(self):
        '''
        skip use hints with swipe method.
        param: None
        '''
        s_x = self.uia_device.info["displayWidth"] / 2
        e_y = self.uia_device.info["displayHeight"] / 2
        time.sleep(1)
        self.uia_device.click(s_x / 2, e_y)
        time.sleep(.5)
        self.uia_device.swipe(s_x, 0, s_x, e_y)
        if self.uia_device(className="android.widget.RelativeLayout",
                           resourceId="com.android.systemui:id/header").exists:
            self.uia_device.press.back()
