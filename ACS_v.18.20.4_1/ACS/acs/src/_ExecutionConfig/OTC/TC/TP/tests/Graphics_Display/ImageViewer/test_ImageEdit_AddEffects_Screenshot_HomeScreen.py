# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 05/27/2015
@author: Xiangyi Zhao
'''
from testlib.graphics.test_template.phototestbase import PhotoTestBase
from testlib.graphics.imagedetails_impl import ImageDetails
from testlib.graphics.photos_impl import get_photo_implement
from testlib.common.base import clearTmpDir


class ImageEdit(PhotoTestBase):

    @classmethod
    def setUpClass(self):
        """
        set environment
        """
        super(ImageEdit, self).setUpClass()
        self.photos = get_photo_implement()
        self._imageviewer = ImageDetails()
        self.photos.rm_delete_photos()
        self.photos.deploy_photo_content("content_picture", "bmp")

    def setUp(self):
        super(ImageEdit, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ImageEdit, self).tearDown()
        self.photos.rm_delete_photos()
        self.photos.stop_photos_am()
        clearTmpDir()

    def test_imageedit_addeffects_screenshot_homescreen(self):
        ''' refer TC test_imageedit_addeffects_screenshot_homescreen
        '''
        print "[RunTest]: %s" % self.__str__()
        self.photos.launch_photos_am()
        self._imageviewer.AddEffects_Screenshot_HomeScreen()
        self.photos.stop_photos_am()
