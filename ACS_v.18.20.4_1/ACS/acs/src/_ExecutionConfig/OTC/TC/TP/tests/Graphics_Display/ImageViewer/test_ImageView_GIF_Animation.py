# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 09/07/2015
@author:Zhang,RongX Z
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.imagedetails_impl import ImageDetails
from testlib.graphics.photos_impl import get_photo_implement


class ImageView(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        set environment
        """
        super(ImageView, self).setUpClass()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        push_pics = ['gif' + str(i) for i in range(1, 6)]
        for i in push_pics:
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

    def test_ImageView_GIF_Animation(self):
        ''' refer TC test_ImageView_GIF_Animation
        '''
        print "[RunTest]: %s" % self.__str__()
        self.photos.launch_photos_am()
        self._imageviewer.check_image_fullscreen_onebyone(5)
        self.photos.stop_photos_am()
