# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 05/15/2015
@author: Yingjun Jin
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.imagedetails_impl import ImageDetails
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.common import multi_display
from testlib.graphics.compare_pic_impl import compare_pic
from time import sleep


class ImageEdit(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        set environment
        """
        super(ImageEdit, self).setUpClass()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.photos.deploy_photo_content("content_picture", "bmp")

    def setUp(self):
        super(ImageEdit, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._imageviewer = ImageDetails()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ImageEdit, self).tearDown()
        self.photos.rm_delete_photos()
        self._imageviewer.clean_tmp()

    def test_ImageEdit_ContinouslyRotate_SaveImage(self):
        ''' refer TC test_ImageEdit_ContinouslyRotate_SaveImage
        '''
        print "[RunTest]: %s" % self.__str__()
        self.photos.launch_photos_am()
        self.photos.open_a_picture()
        self.photos.click_crop_tools()
        im1 = multi_display.get_screenshot_forMultiDisplay(0)
        # --------Rotate left or right--------
        for _ in range(4):
            self.photos.rotate_90_degrees()
        sleep(3)
        im2 = multi_display.get_screenshot_forMultiDisplay(0)
        assert compare_pic.compare_pic(im1, im2) < 5, "Got diff between two pics."
        self.photos.stop_photos_am()
