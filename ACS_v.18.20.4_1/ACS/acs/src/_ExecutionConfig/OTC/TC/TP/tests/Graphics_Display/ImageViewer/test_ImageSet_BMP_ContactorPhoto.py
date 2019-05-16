# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 11/16/2015
@author: Xiangyi Zhao
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.set_contactor_photo_impl import SetAccount, SetPhoto
from testlib.graphics.photos_impl import get_photo_implement
from testlib.util.common import g_common_obj


class SetContactorPhoto(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(SetContactorPhoto, self).setUpClass()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.photos.deploy_photo_content("qrcode_marked_image", "wallpaperset_bmp_image")
        g_common_obj.adb_cmd_capture_msg("pm disable com.android.gallery3d")

    def setUp(self):
        super(SetContactorPhoto, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._setPhoto = SetPhoto()
        self._setAccount = SetAccount()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SetContactorPhoto, self).tearDown()
        try:
            self._setAccount.delete_account("TEST_BMP")
        except:
            print "account was not removed in tearDown."
        self.photos.rm_delete_photos()

    def test_setBMP_ContactorPhoto(self):
        self._setAccount.launch_app_am()
        self._setAccount.create_account("TEST_BMP")
        self._setAccount.stop_app_am()
        self.photos.launch_photos_am()
        self.photos.open_a_picture()
        self.photos.set_picture_as_contact_photo("TEST_BMP")
        self.photos.stop_photos_am()
        self._setAccount.launch_app_am()
        self._setAccount.check_setphoto("TEST_BMP", "F0PGNM595G")
        self._setAccount.stop_app_am()
