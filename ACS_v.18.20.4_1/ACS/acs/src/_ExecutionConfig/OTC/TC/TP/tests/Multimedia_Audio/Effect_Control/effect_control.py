import time
from testlib.audio.audio_test_base import AudioStubTestBase


class EffectControlTest(AudioStubTestBase):
    def testEffectPlayback_LockUnlockScreenSound(self):
        self.enable_screen_lock()
        self.rpc.play()
        self.rpc.enableEffect()
        self.check_screen_lock_sound()

    def testEffectPlayback_VolumeControl_InSettings(self):
        self.rpc.playLong()
        self.rpc.enableEffect()
        self.check_volume_in_settings()

    def testPlayback_Speaker_VolumeControl_InSettings(self):
        self.rpc.playLong()
        self.check_volume_in_settings()

    def testRotateScreen_VolumeControl_InSettings(self):
        self.rpc.playLong()
        old_orient = self.d.orientation
        self.d.orientation = 'left'
        time.sleep(1)
        try:
            self.assertTrue(self.rpc.isPlaying())

            self.check_volume_in_settings()
            self.assertTrue(self.rpc.isPlaying())
        except:
            self.screenshot()
            raise
        finally:
            self.d.orientation = old_orient

    def wait_boot_completed(self, timeout=500):
        ''' wait Android boot_completed

        args: timeout -- optional timeout in second, default 180s
        '''
        count = 0
        sleep_time = 5
        while count < timeout:
            prop_val = self.adb.adb_cmd_capture_msg(
                'getprop sys.boot_completed')
            if '1' in prop_val:
                self.logger.debug('boot_completed')
                return
            count += sleep_time
            time.sleep(sleep_time)
        raise Exception('%ds timeout waiting for boot_completed' % timeout)

    def testPlayback_Reboot_20Times(self):
        for i in range(20):
            self.logger.debug("loop: %d"%i)
            self.adb.reboot_device()
            self.wait_boot_completed()
            self.test_device.unlock_screen()
            retry = 5
            while retry > 0:
                # for low performance device, we got multi retry
                try:
                    self.test_device.app.instr_run(
                        "multimedia_audio.MediaPlayerTest#testBasicPlayback")
                    break
                except:
                    retry -= 1
                    if retry <= 0:
                        raise
