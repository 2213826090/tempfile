import time
from testlib.audio.audio_test_base import AudioStubTestBase


class AudioStreamTest(AudioStubTestBase):
    def testStreaming_Audio_MusicTransferring(self):
        media_file = self.get_media_file()
        push_path = "/storage/sdcard0/audio_file.mp3"
        self.rpc.playStream()
        time.sleep(1)
        self.assertTrue(self.rpc.isPlaying())
        # push media
        self.adb.push_file(media_file, push_path)
        try:
            self.assertTrue(self.rpc.isPlaying())
        finally:
            self.adb.adb_cmd("rm %s" % push_path)

    def testStreaming_Http_Live_Audio_IHF(self):
        self.rpc.playStream()
        time.sleep(1)
        self.assertTrue(self.rpc.isPlaying())
        time.sleep(3)

    def testStreaming_Http_Live_Audio_LockScreen_Resume(self):
        self.enable_screen_lock()
        self.rpc.playStream()
        self.check_screen_lock_sound()
        # check isPlaying after resume
        self.assertTrue(self.rpc.isPlaying())

    def testStreaming_LockScreen_ReceiveIM(self):
        self.rpc.playStream()
        self.test_device.lock_screen()
        try:
            time.sleep(1)
            self.rpc.triggerNotificationIM()
            time.sleep(5)
            assert self.rpc.isStreamActive("notification", 5000),"Can't get notification sound"
        finally:
            self.d.wakeup()
            time.sleep(1)
            self.test_device.unlock_screen()
