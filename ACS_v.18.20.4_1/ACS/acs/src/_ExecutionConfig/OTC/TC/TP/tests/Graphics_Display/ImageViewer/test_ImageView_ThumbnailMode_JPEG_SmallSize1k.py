# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 09/06/2015
@author: Zhang,RongX Z
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.imagedetails_impl import ImageDetails
from testlib.util.common import g_common_obj
from testlib.graphics.photos_impl import get_photo_implement


class ImageView(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        set environment
        """
        super(ImageView, self).setUpClass()
        self._imageviewer = ImageDetails()
        self.device = g_common_obj.get_device()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.photos.deploy_photo_content("content_picture_jpeg", "1k_jpeg")

    def setUp(self):
        super(ImageView, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ImageView, self).tearDown()
        self.photos.rm_delete_photos()

    def test_ImageView_ThumbnailMode_JPEG_SmallSize1k(self):
        ''' refer TC test_ImageView_ThumbnailMode_JPEG_SmallSize1k
        '''
        print "[RunTest]: %s" % self.__str__()
        self.photos.launch_photos_am()
        g_common_obj.assert_exp_happens()
        self.photos.open_a_picture()
        g_common_obj.assert_exp_happens()
        assert not self.device(text="Couldn't load photo").exists, "view 1k jpeg failed, and popup 'Couldn't load photo'"
        self.photos.stop_photos_am()
