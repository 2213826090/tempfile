# -*- coding: utf-8 -*-
'''
@summary: HDMI out tests.
@since: 09/19/2017
@author: Rui
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.common import FileSystem, g_common_obj


class DisplayHDMIout(UIATestBase):

    def setUp(self):
        super(DisplayHDMIout, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.fs = FileSystem()
        self.photosImpl = get_photo_implement()
        self.photosImpl.rm_delete_photos()
        self.photosImpl.refresh_sdcard()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(DisplayHDMIout, self).tearDown()
        self.photosImpl.rm_delete_photos()
        self.photosImpl.refresh_sdcard()

    def test_HDMI_ImageDisplay_JPEG(self):
        self.fs.is_HDMI_connected()
        self.photosImpl.deploy_photo_content("Pictures", "picture_003", "/sdcard/Pictures")
        self.photosImpl.refresh_sdcard()
        self.photosImpl.launch_photos_am()
        self.photosImpl.open_a_picture()
        g_common_obj.assert_exp_happens()
