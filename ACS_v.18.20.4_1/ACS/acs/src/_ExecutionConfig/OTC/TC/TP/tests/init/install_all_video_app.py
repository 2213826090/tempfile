import os
from testlib.util.uiatestbase import UIATestBase
from testlib.multimedia.multimedia_setting import MultiMediaSetting

class InstallAllVideoApp(UIATestBase):

    def setUp(self):
        super(InstallAllVideoApp, self).setUp()
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.dut_init.conf'
        self.retry_num = self.config.read(cfg_file,'init_list').get("delete_camera_folder_file")
        if self.retry_num is None:
            self.retry_num = 3
        self.retry_num = int(self.retry_num)
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.mum_auto_video.conf')
        self.multimedia_setting = MultiMediaSetting(cfg_file)

    def tearDown(self):
        super(InstallAllVideoApp, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testInstallAllVideoApp(self):
        """
        delete all camera folder files
        """
        print "[RunTest]: %s" % self.__str__()

        succeed = False
        for _ in range(self.retry_num):
            try:
                self.multimedia_setting.install_apk("video_apk")
                self.multimedia_setting.install_apk("alarm_apk")
                self.multimedia_setting.install_apk("photo_apk")
                succeed = True
                break
            except Exception as e:
                print e
        assert succeed

