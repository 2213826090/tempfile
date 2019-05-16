# coding: UTF-8
from testlib.audio import resource
from testlib.common.common import g_common_obj2
from testlib.settings.settings import Settings
from testlib.audio.audio_test_base import AudioStubTestBase


class ChangeNotificationSound(AudioStubTestBase):
    impl = Settings()
    CONFIG_FILE = "tests.tablet.mum_auto_audio.conf"

    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(ChangeNotificationSound, self).setUp()
        self._test_name = __name__
        self.logger.info( "[Setup]: %s" % self._test_name)

    def tearDown(self):
        super(ChangeNotificationSound, self).tearDown()
        if hasattr(self, "appList"):
            for each in self.appList:
                g_common_obj2.stopAppByName(each)

    def appPrepare(self, case_name=None):
        self.audio.cfg = self.config.read(self.CONFIG_FILE, case_name)
        g_common_obj2.adb_cmd_capture_msg(self.audio.cfg.get("remove_audio"))
        self.file_name = self.audio.cfg.get("push_audio").split("/")[-1].replace("\"","")
        self.push_path = self.audio.cfg.get("push_audio").split("\" \"")[1].replace("\"","")
        ret_file = resource.get_media_content(self.file_name)
        g_common_obj2.adb_cmd_common("push \"" + ret_file + "\" " + "\"" + self.push_path + "\"")
        g_common_obj2.adb_cmd_capture_msg(self.audio.cfg.get("refresh_sd"))
        self.audio.set_orientation_n()
        self.audio.cleanUpData()

    def testAudioPlayback_Iteration_ChangeNotification_100cycles(self, loops=10):
        self.impl.launchSettings()
        self.impl.openSoundNotification()
        self.impl.saveNotificationRing()
        for i in range(int(loops)):
            self.impl.openDefaultNotificationRingtone()
            self.logger.debug("loops:%d"%(i+1))
            self.impl.setDefaultNotificationRing(index=(i%2+1))
            self.impl.clickOk()
        self.impl.restoreNotificationRing()
