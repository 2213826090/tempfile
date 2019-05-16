'''
Created on Jan 27, 2015
@author: yusux
'''
from testlib.graphics.switch_photo_folders import SwitchFolderImpl
from testlib.graphics.launcher import LaunchFromDrawer
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.photosMgrImpl import PhotosPPImpl


class SwitchMultiPhotosFolder(UIATestBase):

    @classmethod
    def setUpClass(self):
        super(SwitchMultiPhotosFolder, self).setUpClass()

    @classmethod
    def tearDownClass(self):

        super(SwitchMultiPhotosFolder, self).tearDownClass()

    def setUp(self):
        super(SwitchMultiPhotosFolder, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SwitchMultiPhotosFolder, self).tearDown()
        g_common_obj.adb_cmd_common("shell rm -rf /sdcard/DCIM/png")
        g_common_obj.adb_cmd_common("shell rm -rf /sdcard/Pictures/webp")
        PhotosPPImpl().refresh_sdcard()

    def test_switch_multifolders(self, switchtimes=5):
        instanc = SwitchFolderImpl()
        instanc.get_pictures()
        instanc.push_content()
        PhotosPPImpl().refresh_sdcard()
        PhotosPPImpl().unlock_screen()
        if not LaunchFromDrawer().launch_specific_and_check("com.google.android.apps.plus",
                                                            "com.google.android.apps.photos.phone.PhotosHomeActivity"):
            g_common_obj.launch_app_am(
                "com.google.android.apps.plus", "com.google.android.apps.photos.phone.PhotosHomeActivity")
        PhotosPPImpl().init_photos_signin()
        for x in xrange(switchtimes):
            PhotosPPImpl().navigate_to_ondevice()
            instanc.openpngfolders()
            instanc.navigator_up()
            instanc.openwebpfolders()
            instanc.navigator_up()
        instanc.rm_installed_content()
        PhotosPPImpl().refresh_sdcard()

    def test_thumbernail_switch_multifolders(self, switchtimes=5):
        instanc = SwitchFolderImpl()
        instanc.get_pictures()
        instanc.push_content()
        instanc.pull_and_push_thumbernail()
        g_common_obj.launch_app_am(
            "com.google.android.apps.plus", "com.google.android.apps.photos.phone.PhotosHomeActivity")
        PhotosPPImpl().init_photos_signin()
        instanc.browse_thumbernail(switchtimes)
        g_common_obj.adb_cmd_common("shell rm -rf /sdcard/DCIM/png")
        g_common_obj.adb_cmd_common("shell rm -rf /sdcard/Pictures/webp")
        PhotosPPImpl().refresh_sdcard()