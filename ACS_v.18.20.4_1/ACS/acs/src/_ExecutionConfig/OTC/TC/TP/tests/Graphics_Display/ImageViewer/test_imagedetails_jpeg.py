# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 01/26/2015
@author: Yingjun Jin
'''
from testlib.graphics.test_template.phototestbase import PhotoTestBase
from testlib.graphics.imagedetails_impl import ImageDetails
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.graphics.photos_impl import get_photo_implement
import os
import time


class CheckImageDetails(PhotoTestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(CheckImageDetails, self).setUpClass()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_picture')
        arti = Artifactory(cfg_arti.get('location'))
        pic_name = cfg.get("name")
        file_path = arti.get(pic_name)
        g_common_obj.adb_cmd_common('push ' + file_path + ' /sdcard/Pictures')
        self.pic_path = '/sdcard/Pictures/' + os.path.basename(file_path)
        ImageDetails.set_workaround()

    def setUp(self):
        super(CheckImageDetails, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._checkDetails = ImageDetails()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckImageDetails, self).tearDown()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()

    def test_check_image_details_jpeg(self):
        #         self.photos.launch_photos_am()
        #         self.photos.open_a_picture()
        self.photos.open_image_command(self.pic_path)
        time.sleep(3)
        self.photos.view_picture_details()
        self._checkDetails.check_detail()
        self.photos.stop_photos_am()
