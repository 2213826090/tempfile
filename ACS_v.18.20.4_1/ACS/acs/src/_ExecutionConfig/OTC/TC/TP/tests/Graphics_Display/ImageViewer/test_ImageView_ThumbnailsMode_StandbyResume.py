# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 12/07/2015
@author: Zhang,RongX Z
'''
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.imagedetails_impl import ImageDetails
from testlib.util.common import g_common_obj
from testlib.graphics.photos_impl import get_photo_implement


class ImageViewThumbnailsMode(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        set environment
        """
        super(ImageViewThumbnailsMode, self).setUpClass()
        self.device = g_common_obj.get_device()
        self._imageviewer = ImageDetails()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        push_pics = ['jpeg' + str(i) for i in range(1, 5)]
        for i in push_pics:
            self.photos.deploy_photo_content("content_picture_jpeg", i)

    def setUp(self):
        super(ImageViewThumbnailsMode, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ImageViewThumbnailsMode, self).tearDown()
        self.photos.rm_delete_photos()

    def test_ImageView_ThumbnailsMode_StandbyResume(self):
        ''' refer TC test_ImageView_ThumbnailsMode_StandbyResume
        '''
        print "[RunTest]: %s" % self.__str__()
        self.photos.launch_photos_am()
        g_common_obj.assert_exp_happens()
        self.photos.view_pictures_in_thumbnailsMode()
        time.sleep(1)
        self.device.screen.off()
        time.sleep(3)
        self.device.screen.on()
        g_common_obj.assert_exp_happens()
        assert not self.device(text="Couldn't load photo").exists, "view 1k jpeg failed, and popup 'Couldn't load photo'"
        self.photos.stop_photos_am()
