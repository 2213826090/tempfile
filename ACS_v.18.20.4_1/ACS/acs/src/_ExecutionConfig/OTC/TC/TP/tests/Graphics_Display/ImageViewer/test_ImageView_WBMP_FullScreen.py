# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 07/07/2015
@author: Xiangyi Zhao
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
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.photos.deploy_photo_content("content_picture", "wbmp")

    def setUp(self):
        super(ImageView, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._imageviewer = ImageDetails()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ImageView, self).tearDown()
        self.photos.rm_delete_photos()

    def test_imageview_wbmp_fullscreen(self):
        ''' refer TC test_ImageView_WBMP_FullScreen
        '''
        print "[RunTest]: %s" % self.__str__()
        self.photos.launch_photos_am()
        self.photos.open_a_picture()
        self.photos.view_a_picture_fullscreen()
        g_common_obj.assert_exp_happens()
        self.photos.stop_photos_am()
