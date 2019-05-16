# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 09/02/2015
@author:Zhang,RongX Z
'''
from testlib.graphics.test_template.phototestbase import PhotoTestBase
from testlib.graphics.imagedetails_impl import ImageDetails
from testlib.graphics.photos_impl import get_photo_implement


class ImageView(PhotoTestBase):

    @classmethod
    def setUpClass(self):
        """
        set environment
        """
        super(ImageView, self).setUpClass()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        name = ["gif3", "gif7", "gif8", "gif9", "gif10", "gif11"]
        for i in name:
            self.photos.deploy_photo_content("content_picture_gif", i)

    def setUp(self):
        super(ImageView, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._imageviewer = ImageDetails()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ImageView, self).tearDown()
        self.photos.rm_delete_photos()

    def test_ImageView_gif_ImageSize2M(self):
        ''' refer TC test_ImageView_gif_ImageSize2M
        '''
        print "[RunTest]: %s" % self.__str__()
        self.photos.launch_photos_am()
        self._imageviewer.check_image_fullscreen_onebyone(6)
        self.photos.stop_photos_am()
