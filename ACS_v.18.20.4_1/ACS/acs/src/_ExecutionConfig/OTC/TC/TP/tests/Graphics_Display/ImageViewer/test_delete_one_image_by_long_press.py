# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 09/01/2015
@author: Xiangyi Zhao
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.photos_impl import get_photo_implement


class DeleteOneImage(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        set up and push picture
        """
        super(DeleteOneImage, self).setUpClass()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.photos.deploy_photo_content('Pictures', 'email')

    def setUp(self):
        super(DeleteOneImage, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(DeleteOneImage, self).tearDown()
        self.photos.rm_delete_photos()

    def test_delete_one_image_long_press(self):
        self.photos.launch_photos_am()
        before_num = self.photos.check_pic_number()
        self.photos.delete_photos_in_a_folder("Pictures", 1)
        after_num = self.photos.check_pic_number()
        assert before_num - after_num == 1, "delete photos failed,before num is %s,after num is %s" % (before_num, after_num)
        self.photos.stop_photos_am()
