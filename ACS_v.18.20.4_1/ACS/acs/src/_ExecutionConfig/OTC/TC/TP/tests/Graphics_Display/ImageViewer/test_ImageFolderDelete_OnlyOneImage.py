# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/30/2015
@author: Xiangyi Zhao
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.graphics.photos_impl import get_photo_implement


class DeleteImageFolder(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(DeleteImageFolder, self).setUpClass()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()

    def setUp(self):
        super(DeleteImageFolder, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(DeleteImageFolder, self).tearDown()
        self.photos.rm_delete_photos()
        self.photos.stop_photos_am()

    def test_delete_one_image_folder(self):
        g_common_obj.shell_cmd("adb shell mkdir /sdcard/Pictures/test")
        self.photos.deploy_photo_content("Pictures", "email", "/sdcard/Pictures/test")
        self.photos.launch_photos_am()
        self.photos.delete_photos_in_a_folder("test", 1)
        self.photos.launch_photos_am()
        self.photos.check_if_folder_deleted("test")
