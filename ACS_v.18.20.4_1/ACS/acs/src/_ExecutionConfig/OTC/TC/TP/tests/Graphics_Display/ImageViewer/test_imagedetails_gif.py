# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 01/26/2015
@author: Yingjun Jin
'''
from testlib.graphics.test_template.phototestbase import PhotoTestBase
from testlib.graphics.imagedetails_impl import ImageDetails
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.common import file_sys
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
        self.photos.deploy_photo_content('Pictures', 'picture_002')
        self.pic_path = file_sys.get_file_list("/sdcard/Pictures")[0]

    def setUp(self):
        super(CheckImageDetails, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._checkDetails = ImageDetails()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckImageDetails, self).tearDown()
        self.photos.rm_delete_photos()

    def test_check_image_details_gif(self):
        #         self.photos.launch_photos_am()
        #         self.photos.open_a_picture()
        self.photos.open_image_command(self.pic_path)
        time.sleep(3)
        self.photos.view_picture_details()
        self._checkDetails.check_detail()
        self.photos.stop_photos_am()
