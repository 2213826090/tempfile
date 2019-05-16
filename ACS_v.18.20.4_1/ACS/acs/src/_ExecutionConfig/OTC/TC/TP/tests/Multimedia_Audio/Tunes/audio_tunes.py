import time
import os
import datetime
from testlib.audio.audio_test_base import AudioStubTestBase
from testlib.clock.clock import Clock


class AudioTunesTest(AudioStubTestBase):
    def setUp(self):
        super(AudioTunesTest, self).setUp()
        self.clock = Clock()

    def set_interrupt_mode(self, mode):  # NOQA
        '''
        set Interruption mode in Sound & Notification
        0 -- Always interrupt
        1 -- Allow only priority interruption
        2 -- Don't interrupt
        '''
        img_v = self.get_android_version()
        if img_v < 'M':
            options = ["Always interrupt",
                       "Allow only priority interruptions",
                       "Don't interrupt"]
            #self.system.launch_sound()
            self.test_device.launch_sound_settings()
            self.d(text="Interruptions").click.wait()  # click to "Interruption"
            self.d(text="When calls and notifications arrive").click.wait()
            self.d(text=options[mode]).click.wait()
            time.sleep(5)
            if (mode != 0):  # confirm the pop up dialog
                if self.d.exists(text="OK"):
                    self.d(text="OK").click.wait()
            if (mode == 1):  # only enable Events for priority mode
                self.set_option("Events and reminders", True)
                self.set_option("Calls", False)
                self.set_option("Messages", False)
            self.d.press.back()
            self.d.press.home()
        else:
            # switch locates in quick settings
            def click_off():
                ''' click off any priority mode '''
                hints = ["Total silence",
                         "Alarms only",
                         "Priority only"]
                off_wgt = self.d(text="Do not disturb")
                for txt in hints:
                    wgt = self.d(text=txt)
                    if wgt.exists:
                        wgt.click.wait()
                        time.sleep(1)
                        break
                return off_wgt.exists

            self.d.open.quick_settings()
            time.sleep(3)#New uiauto need to sleep 3s for action done
            if not click_off():
                raise Exception("Can't set disturb off")
            time.sleep(1)
            if mode == 0:
                return
            elif (mode == 1):
                text = "Priority"
                more_settings = True
            elif (mode == 2):
                text = "Total"
                more_settings = False
            else:
                raise Exception("Unknown mode, only support 0/1/2")
            #if (mode != 0):
            self.d(text="Do not disturb").click.wait()
            time.sleep(.5)
            self.d(textContains=text).click.wait()
            time.sleep(.5)
            self.d(text="Until you turn this off").click.wait()
            if not more_settings:
                if self.d(text = "Done").exists:
                    self.d(text="Done").click.wait()
                else:
                    self.d(text = "DONE").click.wait()

            else:  # Priority mode: configure priority sound
                #self.system.launch_sound()
                #self.d(text="Do not disturb").click.wait()
                if self.d(text = "MORE SETTINGS").exists:
                    self.d(text = "MORE SETTINGS").click.wait()
                else:
                    self.d(text = "More settings").click.wait()
                time.sleep(1)  # Add sleep time to make sure the next ui can be detected.
                self.d(text="Priority only allows").click.wait()
                time.sleep(.5)
                self.set_option("Reminders", False)
                self.set_option("Events", True)
                self.d(text="Messages").click.wait()
                self.d(text="None").click.wait()
                #self.d.press.home()
            self.d.press.home()

    def testNotificationsArrive_DoNotInterrupt(self):
        self.set_interrupt_mode(2)
        try:
            self.rpc.triggerNotificationIM()
            time.sleep(3)
            self.assertFalse(self.rpc.isStreamActive("notification", 3000))
            time.sleep(0.5)
        except:
            self.screenshot()
            raise
        finally:
            self.set_interrupt_mode(0)

    def testNotificationsArrive_AllowOnlyPriorityInterruptions(self):
        self.set_interrupt_mode(1)
        try:
            # trigger IM notification, should not be heard
            self.rpc.triggerNotificationIM()
            time.sleep(5)
            self.assertFalse(self.rpc.isStreamActive("notification", 5000))
            time.sleep(5)

            # trigger Event notification, should be heard
            self.rpc.triggerNotificationEvent()
            time.sleep(5)
            self.assertTrue(self.rpc.isStreamActive("notification", 5000))
        except:
            self.screenshot()
            raise
        finally:
            self.set_interrupt_mode(0)

    def testAdjustNotificationSound_InSetting_InterruptEventAppear(self):
        self.set_interrupt_mode(1)
        try:
            #self.system.launch_sound()
            self.test_device.launch_sound_settings()
            time.sleep(1)
            tab_name = ''
            for name in ["Notification volume", "Ring volume"]:
                if self.d.exists(text=name):
                    tab_name = name
                    break
            seekbar = self.d(text=tab_name).down(resourceId="android:id/seekbar")
            loop = 3
            while not seekbar.exists:
                if loop <= 0:
                    raise Exception("Can't find seekbar")
                loop-=1
                time.sleep(.5)
            rect = seekbar.info["bounds"]
            y = (rect["top"] + rect["bottom"])/2
            x_min = rect["left"] + 10
            x_max = rect["right"] - 10

            # change notification volume
            x_range = x_max - x_min
            self.d.click(x_min + x_range, y)
            time.sleep(3)
            self.d.click(x_min + x_range/2, y)
            time.sleep(3)
            # trigger interruption event
            self.rpc.triggerNotificationEvent()
            time.sleep(5)
            self.assertTrue(self.rpc.isStreamActive("notification", 5000))
        except:
            self.screenshot()
            raise
        finally:
            self.set_interrupt_mode(0)

    def testScreenLockSound_IHF(self):
        self.enable_screen_lock()
        self.check_screen_lock_sound()

    def enable_volume_button_dismiss(self):
        '''
        enable volume button to dismiss alarm in Alarm Settings
        '''
        # activity = "com.android.deskclock/com.android.deskclock.DeskClock"
        # self.adb.adb_cmd("am start -n " + activity)
        self.clock.launch()
        # click menu button
        menu_id = "com.android.deskclock:id/menu_button"
        if self.d.exists(resourceId=menu_id):
            self.d(resourceId=menu_id).click.wait()
        else:
            self.d.press.menu()
        self.d(text="Settings").click.wait()  # enter Settings menu
        text = "Volume buttons"
        if self.d(scrollable=True).exists:
            self.d(scrollable=True).scroll.to(text=text)
        self.d(text=text).click.wait()
        # select "Dismiss" options
        self.d(text="Dismiss").click.wait()
        self.d.press.back()
        self.d.press.back()

    def wait_alarm(self, timeout=180):
        remain = timeout
        interval = 5
        while remain > 0:
            if self.rpc.isStreamActive("alarm", 0):
                break
            time.sleep(interval)
            remain -= interval
        else:
            raise Exception("%ds timeout wait for alarm" % timeout)

    def _set_alarm(self, h, m):
        parts = ["am start -a android.intent.action.SET_ALARM",
                 "--ei android.intent.extra.alarm.HOUR %s" % h,
                 "--ei android.intent.extra.alarm.MINUTES %s" % m,
                 "--ez android.intent.extra.alarm.SKIP_UI true"]
        cmd = " ".join(parts)
        self.adb.root_on_device()  # set alarm Intent require root permission
        time.sleep(3)
        self.adb.adb_cmd(cmd)
        time.sleep(2)
        self.d.press.home()

    def add_alarm(self):
        '''
        add a alarm, and return remains seconds for the alarm
        '''
        # get the time on device
        cmd = "date +%H:%M:%S"
        ret = self.adb.adb_cmd_capture_msg(cmd).strip()
        h, m, s = map(int, ret.split(':'))
        t = datetime.datetime(1970, 1, 1, h, m, s)
        remain = 60 - s
        if remain < 20:
            remain += 60
        tt = t + datetime.timedelta(0, remain, 0)
        self._set_alarm(tt.hour, tt.minute)
        return remain

    def dismiss_alarm(self):
        self.test_device.delete_alarms()
        time.sleep(.5)

    def testAdjustVolume_InSetting_AlarmClockAlert(self):
        remain = self.add_alarm()
        try:
            self.wait_alarm(remain * 2)
            self.check_volume_in_settings()
        except:
            self.screenshot()
            raise
        finally:
            self.dismiss_alarm()

    def testAlarmClockAlert_HWControl(self):
        self.enable_volume_button_dismiss()
        remain = self.add_alarm()
        try:
            self.test_device.lock_screen()  # first lock screen
            self.wait_alarm(remain * 2)  # then wait for alarm
            time.sleep(3)
            self.test_device.skip_use_hints()
            self.d.press.volume_up()
            time.sleep(1)
            self.assertFalse(self.rpc.isStreamActive("alarm", 0))
        except:
            self.screenshot()
            raise
        finally:
            self.test_device.unlock_screen()
            self.dismiss_alarm()

    def testScreenLockSound_While_AlarmClockAlert(self):
        self.enable_screen_lock()
        remain = self.add_alarm()
        try:
            self.wait_alarm(remain * 2)
            self.check_screen_lock_sound()
        except:
            self.screenshot()
            raise
        finally:
            self.dismiss_alarm()

    def testDeletePlayingFile_ThenPlaybackPrevNextSong(self):
        self.deploy_play_music_content()
        try:
            self.launch_play_music_n_play("mp3_sample2")
            time.sleep(1)
            # delete the file
            fpath = os.path.join(self.media_path, "mp3_sample2.mp3")
            self.adb.adb_cmd("rm %s" % fpath)
            self._refresh_storage()  # avoid error msg
            time.sleep(1)
            self.audio.clickPrevious()  # first prev seek to 0
            self.audio.clickPrevious()  # jump to previous
            time.sleep(5)
            self.audio.clickNext()
            time.sleep(5)
            self.audio.clickNext()
            time.sleep(5)
        except:
            self.screenshot()
            raise
        finally:
            self.adb.stop_app_am(self.audio.PACKAGE_NAME_PLAY_MUSIC)
            self.clean_play_music_content()

    def testChange_Ringtone_IHF_Normal_Ring(self):
        item_cls_name = "android.widget.CheckedTextView"

        def launch_picker():
            menu = "Default notification ringtone"
            self.scroll_n_click(menu)
            time.sleep(2)

        def get_selected_ringtone():
            launch_picker()
            self.d(scrollable=True).scroll.to(
                className=item_cls_name,
                checked=True)
            return self.d(className=item_cls_name, checked=True).text

        def select_ringtone(name):
            launch_picker()
            self.d(scrollable=True).scroll.to(
                className=item_cls_name,
                text=name)
            self.d(className=item_cls_name, text=name).click.wait()
            time.sleep(1)
            self.d(text="OK").click.wait()

        self.launch_sound_settings()
        time.sleep(2)
        orig_tone = get_selected_ringtone()
        self.logger.info("original Ringtone: %s" % orig_tone)
        try:
            target = self.d(className=item_cls_name, checked=False).text
            self.logger.info("set Ringtone to " + target)
            select_ringtone(target)
            time.sleep(3)
            cur_tone = get_selected_ringtone()
            assert cur_tone == target, \
                "Can't change Ringtone, %s VS %s (target)" % (cur_tone, target)
        finally:
            select_ringtone(orig_tone)