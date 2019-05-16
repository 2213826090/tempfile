'''
@summary: drag notification and check transparency
@since: 11/30/2015
@author: Zhang,RongX Z(rongx.z.zhang@intel.com)
'''
import os
import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.QRcode_impl import QRcode
from testlib.graphics.photos_impl import get_photo_implement


class NotificationBar(UIATestBase):

    def setUp(self):
        super(NotificationBar, self).setUp()
        self._test_name = __name__
        self.d = g_common_obj.get_device()
        self._qr = QRcode()
        self.photos = get_photo_implement()
        print "[Setup]: %s" % self._test_name
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_picture')
        arti = Artifactory(cfg_arti.get('location'))
        pic_name = cfg.get("qr_image")
        file_path = arti.get(pic_name)
        g_common_obj.adb_cmd_common('push ' + file_path + ' /sdcard/Pictures')
        self.pic_path = '/sdcard/Pictures/' + os.path.basename(file_path)
#         self.systemui = SystemUiExtendImpl()
#         self.notifi_quick_settings = NotificationAndQuickSettingsImpl()
#         self.systemui.unlock_screen()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
#         g_common_obj.set_vertical_screen()
#         super(NotificationBar, self).tearDown()
        self.d.press.home()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        try:
            os.remove(".tmp.png")
        except OSError:
            pass

    def test_Shadow_NotificationBar(self):
        """
        1. Open an image file with QR code in it, pull down the notification bar and then take a screenshot.
        2. Use qrtools to read the QR code from the screenshot.
        """
        self.d.orientation = "n"
        _result = []
        for _ in range(0, 3):
            self.photos.open_image_command(self.pic_path)
            time.sleep(2)
            if self.d.open.notification():
                time.sleep(2)
                _result.append(self._qr.decode_image_qrcode(self.d.screenshot(".tmp.png"))[1] == "QRCODE_TEST_STRING")
            if _result[0]:
                break
        print _result
        assert True in _result, "The notification bar is not transparent!"


#         common.rotate_screen()
#         self.notifi_quick_settings.drag_notification_panel()
#         g_common_obj.assert_exp_happens()
#         self.notifi_quick_settings.drag_quick_settings_panel()
#         g_common_obj.assert_exp_happens()
