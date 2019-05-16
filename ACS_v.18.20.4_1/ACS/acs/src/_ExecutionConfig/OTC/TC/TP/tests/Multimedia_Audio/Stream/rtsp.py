import time
import os
from testlib.audio import resource
from testlib.audio.audio_test_base import AudioStubTestBase


class RtspStreamTest(AudioStubTestBase):
    rtsp_bin = "live555MediaServer"
    rtsp_host = "rtsp://127.0.0.1/"
    files = {
        "amrnb": "AMR-NB_mono_8KHz_12.2Kbps.amr",
        "amrwb": "AMR-WB_mono_16KHz_23.85Kbps.amr",
        "aac": "AAC_VBR_48khz_2ch_256kbps_60s.aac",
        "short": "AAC_VBR_48khz_2ch_256kbps_11s.aac",
    }
    target_dir = "/data/live555/"

    def deploy(self):
        self.adb.root_on_device()
        time.sleep(3)

        self.adb.adb_cmd("mkdir -p %s" % self.target_dir)

        # push media files
        for f in self.files.values():
            local_path = resource.get_media_content(f)
            basename = os.path.basename(local_path)
            name, ext = os.path.splitext(basename)
            fname = name + ext.lower()
            target_path = os.path.join(self.target_dir, fname)
            assert self.adb.push_file(local_path, target_path), \
                "push fails: %s to %s" % (f, target_path)

        if not self._get_rtsp_pid():  # if rtsp_bin has been running, just skip
            self.adb.push_file(resource.get_app(self.rtsp_bin),
                               self.target_dir)
            bin_path = os.path.join(self.target_dir, self.rtsp_bin)
            self.adb.adb_cmd("chmod 777 %s" % bin_path)

            self.adb.adb_cmd("%s &" % bin_path)
            time.sleep(1)  # wait some time for process run

    def cleanup(self):
        # find RtspServer pid and kill
        pid = self._get_rtsp_pid()
        if pid:
            self.adb.adb_cmd("kill %s" % pid)
        self.adb.adb_cmd("rm -rf %s " % self.target_dir)

    def _get_rtsp_pid(self):
        '''
        find the pid of rtsp_bin in Android device.

        return: pid if found, otherwise, 'None'
        '''
        pid = None
        buf = self.adb.adb_cmd_capture_msg("ps")
        for line in buf.split("\n"):
            if self.rtsp_bin in line:
                pid = line.split()[1]  # pid is in second place
                break
        return pid

    def get_rtsp_url(self, t):
        fname = self.files.get(t)
        path = os.path.join(self.target_dir, fname)
        url = self.rtsp_host + path
        return url

    def setUp(self):
        super(RtspStreamTest, self).setUp()
        self.deploy()

    def tearDown(self):
        # self.cleanup()
        super(RtspStreamTest, self).tearDown()

    def _do_playing_rtsp(self, t):
        url = self.get_rtsp_url(t)
        self.logger.debug("playing %s" % url)
        try:
            self.rpc.playFile(url)
        except Exception as e:
            if "Prepare failed" in str(e):
                self.fail("fail to play %s" % url)
            else:
                raise e
        time.sleep(2)
        self.assertTrue(self.rpc.isPlaying())
        time.sleep(4)

    def testStreaming_AACLC_RTP_RTSP_BroadcastMode(self):
        self._do_playing_rtsp("aac")

    def testStreaming_RTSP_AMRNB_AMRContainer(self):
        self._do_playing_rtsp("amrnb")

    def testStreaming_RTSP_AMRWB_AMRContainer(self):
        self._do_playing_rtsp("amrwb")

    def testStreaming_RTP_RTSP_VOD_Audio_ShortDuration(self):
        self._do_playing_rtsp("short")

    def testStreaming_RTP_RTSP_RotateScreen(self):
        self._do_playing_rtsp("aac")
        old_orient = self.d.orientation
        try:
            self.d.orientation = 'l'
            time.sleep(1)
            self.assertTrue(self.rpc.isPlaying())

            self.d.orientation = 'u'
            time.sleep(1)
            self.assertTrue(self.rpc.isPlaying())
        finally:
            self.d.orientation = old_orient

    def testStreaming_RTP_RTSP_Change_Volume(self):
        self._do_playing_rtsp("aac")
        self.check_volume_in_settings()

    def testPlayback_Local_RTP_RTSP(self):
        # play local
        self.rpc.play()
        time.sleep(1.5)
        self.assertTrue(self.rpc.isPlaying())
        time.sleep(2)

        # play rtsp
        self.rpc.resetPlayer()
        self._do_playing_rtsp("aac")

    def testStreaming_RTP_RTSP_Audio_wsHS(self):
        self.rpc.forceRouteHeadset(True)
        try:
            self._do_playing_rtsp("aac")
        finally:
            self.rpc.forceRouteHeadset(False)
