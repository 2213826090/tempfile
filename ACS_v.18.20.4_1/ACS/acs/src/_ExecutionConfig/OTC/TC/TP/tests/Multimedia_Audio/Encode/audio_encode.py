import time
import os
from testlib.audio.audio_test_base import AudioStubTestBase
from testlib.audio.recorder_impl import RecorderImpl


class AudioEncodeTest(AudioStubTestBase):
    record_file = "/sdcard/audio_record_file.amr"
    media_name = "audio_record_file"

    def setUp(self):
        AudioStubTestBase.setUp(self)

    def tearDown(self):
        self.adb.stop_app_am(self.audio.PACKAGE_NAME_PLAY_MUSIC)
        self.adb.adb_cmd("rm %s" % self.record_file)
        super(AudioEncodeTest, self).tearDown()

    def testAudioEncode_SoundRecorder_GMSPlayback(self):
        '''
        Device is recording audio, while GMS is playing music
        '''
        self.deploy_play_music_content()
        try:
            self.rpc.startRecording(self.record_file)

            (music_name, ext) = os.path.splitext(self.media_files[0])
            self.launch_play_music_n_play(music_name)
            time.sleep(5)
            cur_track = self.audio.getTrackName()
            self.assertTrue(self.rpc.isStreamActive("music", 0))
            # wait just to next song, check bug OAM-24617:
            # when automatically just to next song, timestamp goto end
            timeout = time.time() + 120  # 120 timeout
            self.logger.info("wait play to next song")
            while time.time() < timeout:
                if self.audio.getTrackName() != cur_track:
                    # already play to next song
                    curtime = self.audio.getCurrentTimestamp()
                    assert curtime < 8, \
                        "Timestamp goto %d when start playing" % curtime
                    break
                time.sleep(1)
            self.rpc.stopRecording()
        finally:
            self.clean_play_music_content()

    def testAudioEncode_Longlasting_1hr(self):
        record_time = 3600
        self.logger.info("Start to record and saved to file %s"%self.record_file)
        recorder = RecorderImpl()
        recorder.install()
        recorder.launch()
        start = time.time()
        try:
            recorder.start_record()
            while time.time() - start < record_time:
                self.d.wakeup()
                if not self.d(text = "Stop", className = "android.widget.Button").enabled:
                    raise Exception("Record stopped after time %ds"%time.time())
                time.sleep(1)

            self.logger.debug("Stop recording")
            recorder.stop()
            time.sleep(1)

            self.logger.debug("Play recorded audio")
            recorder.playback()
            time.sleep(3)
            assert self.rpc.isStreamActive("music", 5000), "Failed to play recorded file!"
        finally:
            recorder.force_stop()

    def testAudioRecording_1hr_Capture20Photo(self):
        record_file = "/sdcard/audio_record_file.amr"
        from testlib.multimedia.multimedia_camera_helper import MultiMediaCameraHelper
        from testlib.camera.CameraCommon import CameraCommon

        camera_helper = MultiMediaCameraHelper()
        camera_helper.camera.cleanMediaFiles()
        CameraCommon().setOrientationToVertical()
        try:
            self.rpc.startRecording(record_file)
            camera_helper.camera.startCameraApp()
            camera_helper.camera.selectMode(mode = "Camera")
            camera_helper.camera.switchRearOrFront(lens="Back")
            camera_helper.camera.capturePhoto(20)
            self.rpc.playFile(record_file)
            time.sleep(2)
            assert self.rpc.isPlaying(), "Failed to play recored audio"
            self.rpc.stop()
        finally:
            CameraCommon().checkCameraCrash()
            camera_helper.camera.cleanMediaFiles()
            self.adb.adb_cmd("rm %s" % record_file)

    def testAudio_IntelAccelerated_Encode(self):
        recorder = RecorderImpl()
        recorder.install()
        recorder.launch()
        try:
            time.sleep(3)
            # config recorder
            recorder.configure({
                "Codecs": "AMR Narrow Band",
                "Container": "3gp",
            })
            # clean logcat
            self.adb.adb_cmd("logcat -c")

            recorder.record(20)
            buf = self.adb.adb_cmd_capture_msg("logcat -d")
            keyword = "OMX.Intel.amrnb.encoder"
            assert keyword in buf, "Can't find %s in logcat" % keyword
        finally:
            recorder.force_stop()

