import time
import os
from testlib.audio.audiostub import MusicWidget
from testlib.audio.audio_test_base import AudioStubTestBase


class AudioDecodeTest(AudioStubTestBase):
    def setUp(self):
        super(AudioDecodeTest, self).setUp()
        self.media_file = self.get_media_file()
        self.push_path = ""

    def tearDown(self):
        if self.push_path:
            self.adb.adb_cmd("rm %s" % self.push_path)
        super(AudioDecodeTest, self).tearDown()

    def testAudioDecode_Lock_Device(self):
        self.rpc.playLong()
        self.test_device.lock_screen()
        time.sleep(5)
        self.assertTrue(self.rpc.isStreamActive("music", 0))
        self.d.wakeup()
        self.test_device.unlock_screen()
        time.sleep(5)
        self.assertTrue(self.rpc.isStreamActive("music", 0))

    def testAudioDecode_Iterative_Receive_LargerFile_from_PC(self):
        self.push_path = "/storage/sdcard0/media_file.mp3"
        self.rpc.playLong()
        self.adb.push_file(self.media_file, self.push_path)
        self.assertTrue(self.rpc.isPlaying())

    def get_mount_point(self):  #NOQA
        def get_mount_point_m():
            buf = self.adb.adb_cmd_capture_msg("sm list-volumes public")
            name = ''
            for l in buf.splitlines():
                if l.startswith("public"):
                    name = l.split()[-1]
                    break
            else:
                return None
            fuse_path = "/storage/" + name
            # check if SD card mounted as fuse
            buf = self.adb.adb_cmd_capture_msg("mount")
            for l in buf.splitlines():
                if fuse_path in l and l.startswith("/dev/fuse"):
                    return fuse_path
            else:
                return None

        if self.get_android_version() >= 'M':  # mount point changed in M
            self.adb.root_on_device()
            time.sleep(3)
            # get the mount point of sd card
            mount_point = get_mount_point_m()

            if not mount_point:
                # enable adoptable storage
                # this must be set first, otherwise, "Set up" menu not appear
                self.adb.adb_cmd_capture_msg("sm set-force-adoptable true")
                time.sleep(3)
                # click "Set up" menu in notification
                self.d.open.notification()
                time.sleep(2)
                if self.d.exists(text="Set up"):
                    self.d(text="Set up").click.wait()
                    # enter "Set up your SD card" page
                    self.d(text="Use as portable storage").click()
                    self.d(text="Next").click.wait()
                    self.d(text="Done").click.wait()
                else:
                    self.d.press.home()

                mount_point = get_mount_point_m()
        else:
            buf = self.adb.adb_cmd_capture_msg("mount")
            mount_point = "/storage/sdcard1"
            if mount_point not in buf:
                mount_point = None
        if not mount_point:
            raise Exception("No mount point found, is SD card present?")
        return mount_point

    def push_media_to_SD(self):
        # before pushing files, make sure SD Card is mounted
        mount_point = self.get_mount_point()
        if not mount_point:
            self.mount_SD(True)
            time.sleep(10)
            mount_point = self.get_mount_point()

        self.push_path = os.path.join(mount_point, "media_file.mp3")
        success = self.adb.push_file(self.media_file, self.push_path)
        if not success:
            msg = "Pushing file to SD card fails, is SD Card present?"
            raise Exception(msg)

    def testAudioDecode_SDcard_Copy_SDCardSongs_to_PC_SameTime(self):
        # first, deploy media file to SD card first
        self.push_media_to_SD()
        # now start test
        self.rpc.playFile(self.push_path)
        time.sleep(1)
        self.assertTrue(self.rpc.isPlaying())
        # start copy file to PC
        self.adb.pull_file("/tmp/", self.push_path)
        self.assertTrue(self.rpc.isPlaying())

    def set_enable_MTP(self, enable):
        if self.get_android_version() >= 'M':
            return
        self.logger.debug("enable MTP %s" % enable)
        prop = "persist.sys.usb.config"
        cmd = "getprop " + prop
        cur_conf = self.adb.adb_cmd_capture_msg(cmd)
        attr = '_sys_usb_config'
        if enable:
            prev_conf = getattr(self, attr, '')
            conf = prev_conf if 'mtp' in prev_conf else 'mtp,adb'
        else:
            setattr(self, '_sys_usb_config', cur_conf)
            functons = cur_conf.split(',')
            if 'mtp' in functons:
                functons.pop(functons.index('mtp'))
            conf = ','.join(functons)
        cmd = "setprop %s %s" % (prop, conf)
        self.adb.root_on_device()  # root first
        time.sleep(3)
        self.adb.adb_cmd(cmd)
        time.sleep(3)  # wait for adb reconnect

    def mount_SD(self, mount):
        cmd = "am start -W com.android.settings"
        self.adb.adb_cmd(cmd)  # launch Settings
        if self.get_android_version() < 'M':
            self.scroll_n_click("Storage")
            if mount:
                self.scroll_n_click("Mount SD card")
            else:
                self.scroll_n_click("Unmount SD card")
                if self.d.exists(text="OK"):  # confirm the dialog
                    self.d(text="OK").click()
        else:
            self.scroll_n_click("Storage & USB")
            if mount:
                text = "Ejected"
                if self.d.exists(text=text):
                    self.d(text=text).click.wait()
                    self.d(text="Mount").click()
            else:
                resourceId = "com.android.settings:id/unmount"
                if self.d.exists(resourceId=resourceId):
                    self.d(resourceId=resourceId).click()
        self.d.press.back()
        self.d.press.home()

    def testAudioDecode_SDcard_Unmount_SDcard_SameTime(self):
        self.push_media_to_SD()
        # in order to unmount SD card, we must disable MTP
        self.set_enable_MTP(False)
        try:
            # start test
            self.rpc.playFile(self.push_path)
            time.sleep(1)
            # start umount
            self.mount_SD(False)
            time.sleep(5)
            self.rpc.stop()  # stop music to make sure SD can be unmount

            time.sleep(60)  # longer enough to unmount SD card
            self.test_device.unlock_screen()  # in case, screen is locked
            self.d.press.home()
            self.mount_SD(True)
        except:
            self.screenshot()
            raise
        finally:
            self.set_enable_MTP(True)

    def testAudioDecode_Musicwidget_PauseResumeNextBackSong(self):
        '''
        Verify that can add music widget, can pause / resume song,
        jump to next/back song on home screen or lock screen.
        '''
        self.enable_screen_lock()
        self.deploy_play_music_content()
        self.launch_play_music_n_play("mp3_sample2")

        try:
            # test on widgets
            self.logger.debug("Add widget")
            mw = MusicWidget()
            mw.add()
            time.sleep(1)
            # test pause
            self.logger.debug("Pause")
            mw.pause()
            time.sleep(1)
            self.assertFalse(self.rpc.isStreamActive("music", 0))
            # test play
            self.logger.debug("Play")
            mw.play()
            time.sleep(1)
            self.assertTrue(self.rpc.isStreamActive("music", 0))
            # test next
            self.logger.debug("Next")
            track = mw.get_track_name()
            mw.next()
            time.sleep(.5)
            self.assertNotEquals(mw.get_track_name(), track)
            # test prev
            self.logger.debug("Previous")
            track = mw.get_track_name()
            mw.prev(2)
            time.sleep(.5)
            self.assertNotEquals(mw.get_track_name(), track)

            # test on lock screen
            self.logger.debug("Lock screen")
            self.test_device.lock_screen()
            time.sleep(2)
            self.d.wakeup()  # show lockscreen
            # now music is playing, pause it
            self.logger.debug("Pause")
            self.d(description="Pause").click.wait()
            time.sleep(1)
            self.assertFalse(self.rpc.isStreamActive("music", 0))
            # resume playing
            self.logger.debug("Play")
            self.d(description="Play").click.wait()
            time.sleep(1)
            self.assertTrue(self.rpc.isStreamActive("music", 0))
            # Next
            self.logger.debug("Next")
            self.d(description="Next").click.wait()
            time.sleep(1)
            self.assertTrue(self.rpc.isStreamActive("music", 0))
            self.logger.debug("Previous")
            self.d(description="Previous").click()
            self.d(description="Previous").click()
            time.sleep(2)
            self.assertTrue(self.rpc.isStreamActive("music", 0))
        except:
            self.screenshot()
            raise
        finally:
            self.test_device.unlock_screen()
            self.d.press.home()
            mw.remove()
            self.adb.stop_app_am(self.audio.PACKAGE_NAME_PLAY_MUSIC)

    def testAudio_IntelAccelerated_Decode(self):
        # clean logcat first
        self.adb.adb_cmd("logcat -c")
        self.rpc.playLong()  # play mp3
        time.sleep(30)
        buf = self.adb.adb_cmd_capture_msg("logcat -d")
        keyword = "OMX.Intel.mp3.decoder"
        assert keyword in buf, "Can't find %s in logcat" % keyword

    def testMP3OffloadPlayback(self):
        '''
        Verify that DUT support MP3 offload playback
        '''
        self.adb.adb_cmd('logcat -c')
        self.rpc.playLong()
        time.sleep(30)
        buf = self.adb.adb_cmd_capture_msg("logcat -d")
        keyword = "Offload"
        assert keyword in buf, "Can't find %s in logcat" % keyword

    def testAudioDecode_gaplessAaclc(self):
        '''
        Verify that can play gapless music of AACLC format, lock screen, unlock screen
        and music should still play.
        '''
        gapless_aaclc = ['AACLC_ChirpZero-01.m4a', 'AACLC_ChirpZero-02.m4a',
                         'AACLC_ChirpZero-03.m4a', 'AACLC_ChirpZero-04.m4a',
                         'AACLC_ChirpZero-05.m4a', 'AACLC_ChirpZero-06.m4a']
        self.enable_screen_lock()
        self.deploy_play_music_content(gapless_aaclc)
        self.launch_play_music_n_play("AACLC_ChirpZero-01.m4a")

        self.test_device.lock_screen()
        time.sleep(2)
        self.d.wakeup()
        time.sleep(2)
        self.test_device.unlock_screen()
        time.sleep(2)
        self.assertTrue(self.rpc.isStreamActive("music", 0))

    def testAudioDecode_gaplessHeaac(self):
        '''
        Verify that can play gapless music of HEAAC format, lock screen, unlock screen
        and music should still play.
        '''
        gapless_heaac = ['HEAAC_ChirpZero-01.m4a', 'HEAAC_ChirpZero-02.m4a',
                         'HEAAC_ChirpZero-03.m4a', 'HEAAC_ChirpZero-04.m4a',
                         'HEAAC_ChirpZero-05.m4a', 'HEAAC_ChirpZero-06.m4a']
        self.enable_screen_lock()
        self.deploy_play_music_content(gapless_heaac)
        self.launch_play_music_n_play("HEAAC_ChirpZero-01.m4a")

        self.test_device.lock_screen()
        time.sleep(2)
        self.d.wakeup()
        time.sleep(2)
        self.test_device.unlock_screen()
        time.sleep(2)
        self.assertTrue(self.rpc.isStreamActive("music", 0))

    def testAudioDecode_gaplessHeaacPlus(self):
        '''
        Verify that can play gapless music of HEAAC Plus format, lock screen, unlock screen
        and music should still play.
        '''
        gapless_heaacplus = ['HEAAC_Plus_ChirpZero-01.m4a', 'HEAAC_Plus_ChirpZero-02.m4a',
                         'HEAAC_Plus_ChirpZero-03.m4a', 'HEAAC_Plus_ChirpZero-04.m4a',
                         'HEAAC_Plus_ChirpZero-05.m4a', 'HEAAC_Plus_ChirpZero-06.m4a']
        self.enable_screen_lock()
        self.deploy_play_music_content(gapless_heaacplus)
        self.launch_play_music_n_play("HEAAC_Plus_ChirpZero-01.m4a")

        self.test_device.lock_screen()
        time.sleep(2)
        self.d.wakeup()
        time.sleep(2)
        self.test_device.unlock_screen()
        time.sleep(2)
        self.assertTrue(self.rpc.isStreamActive("music", 0))

    def testAudioDecode_gaplessMp3(self):
        '''
        Verify that can play gapless music of HEAAC format, lock screen, unlock screen
        and music should still play.
        '''
        gapless_mp3 = ['ChirpZero-01.mp3', 'ChirpZero-02.mp3',
                         'ChirpZero-03.mp3', 'ChirpZero-04.mp3',
                         'ChirpZero-05.mp3', 'ChirpZero-06.mp3']
        self.enable_screen_lock()
        self.deploy_play_music_content(gapless_mp3)
        self.launch_play_music_n_play("ChirpZero-01.mp3")

        self.test_device.lock_screen()
        time.sleep(2)
        self.d.wakeup()
        time.sleep(2)
        self.test_device.unlock_screen()
        time.sleep(2)
        self.assertTrue(self.rpc.isStreamActive("music", 0))

    def testAudioDecodeAllOpusogg(self):
        '''
        Verify that can play Opus music of ogg format,
        '''
        opusogg = ['32bits_per_sample.ogg']
        self.enable_screen_lock()
        self.deploy_play_music_content(opusogg)
        self.launch_play_music_n_play("32bits_per_sample.ogg")

        self.test_device.lock_screen()
        time.sleep(2)
        self.d.wakeup()
        time.sleep(2)
        self.test_device.unlock_screen()
        time.sleep(2)
        self.assertTrue(self.rpc.isStreamActive("music", 0))

    def test4ChannelsAnalogAudioOutput(self):
            '''
            Verify that DUT support 4 channel audio analog output playback.
            '''
            channel_output = ['Surround_Sound_Test_5_1.wav']
            self.enable_screen_lock()
            self.deploy_play_music_content(channel_output)
            self.launch_play_music_n_play("Surround_Sound_Test_5_1.wav")

            self.test_device.lock_screen()
            time.sleep(4)
            self.d.wakeup()
            time.sleep(4)
            self.test_device.unlock_screen()
            time.sleep(4)
            self.assertTrue(self.rpc.isStreamActive("music", 0))

    def test4ChannelsAnalogAudioOutput_RightOnly(self):
            '''
            Verify that DUT support 4 channel audio analog output playback in right speaker only.
            '''
            channel_right = ['sine_right_997_44k_0dB-wav.wav']
            self.enable_screen_lock()
            self.deploy_play_music_content(channel_right)
            self.launch_play_music_n_play("sine_right_997_44k_0dB-wav.wav")

            self.test_device.lock_screen()
            time.sleep(2)
            self.d.wakeup()
            time.sleep(2)
            self.test_device.unlock_screen()
            time.sleep(2)
            self.assertTrue(self.rpc.isStreamActive("music", 0))

    def test4ChannelsAnalogAudioOutput_LeftOnly(self):
            '''
            Verify that DUT support 4 channel audio analog output playback in left speaker only.
            '''
            channel_left = ['sine_left_997_44k_0dB-wav.wav']
            self.enable_screen_lock()
            self.deploy_play_music_content(channel_left)
            self.launch_play_music_n_play("sine_left_997_44k_0dB-wav.wav")

            self.test_device.lock_screen()
            time.sleep(2)
            self.d.wakeup()
            time.sleep(2)
            self.test_device.unlock_screen()
            time.sleep(2)
            self.assertTrue(self.rpc.isStreamActive("music", 0))