import os
from testlib.util.uiatestbase import UIATestBase
from testlib.multimedia.multimedia_setting import CheckPhotoApp
from testlib.util.common import g_common_obj
from testlib.common.common import g_common_obj2

class UpgradePhotoApp(UIATestBase):

    def setUp(self):
        super(UpgradePhotoApp, self).setUp()
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.dut_init.conf'
        self.retry_num = self.config.read(cfg_file,'init_list').get("upgrade_photo_app")
        self.retry_num = int(self.retry_num)
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.mum_auto_video.conf')
        self.check_photo_app = CheckPhotoApp(cfg_file)

    def tearDown(self):
        super(UpgradePhotoApp, self).tearDown()
        print "[Teardown]: %s" % self._test_name

    def testUpgradePhotoApp(self):
        """
        delete all camera folder files
        """
        print "[RunTest]: %s" % self.__str__()
        
        message = ""
        succeed = False
        for i in range(self.retry_num):
            try:
                android_version_release = g_common_obj.adb_cmd_capture_msg("getprop |grep 'version.release'").split(':')[-1].strip()
                if (android_version_release == '[8.0.0]') or (android_version_release == '[8.1.0]'):
                    self.check_photo_app.check_photo_app()
                succeed = True
                break
            except Exception as e:
                print e
                message += "%d:%s\n" % (i, e)
        assert succeed, message

