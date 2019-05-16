"""
Created on Jan 23, 2015

@author: yusux
"""
from testlib.graphics.photosMgrImpl import PhotosPPImpl
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.aosp_apps_init import AospAppsInit

class PhotosFolderTest(UIATestBase):
    @classmethod
    def setUpClass(self):
        super(PhotosFolderTest, self).setUpClass()

    @classmethod
    def tearDownClass(self):
        super(PhotosFolderTest, self).tearDownClass()

    def setUp(self):
        super(PhotosFolderTest, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(PhotosFolderTest, self).tearDown()

    def testRemoveFolderWithInOneImg(self):
        instanc = PhotosPPImpl()
        instanc.accquire_photos_from_internet()
        instanc.refresh_sdcard()
        instanc.unlock_screen()
        if AospAppsInit().check_if_app_not_first_init("Google+") is False:
            PhotosPPImpl().init_photos_signin()
        g_common_obj.launch_app_from_home_sc("Photos")
        instanc.navigate_to_ondevice()
        instanc.remove_folder_within_oneimg()
