# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 06/02/2015
@author: Xiangyi Zhao
'''
from testlib.graphics.test_template.phototestbase import PhotoTestBase
from testlib.graphics.imagedelete_impl import ImageDelete
from testlib.graphics.photos_impl import get_photo_implement


class DeleteOneImage(PhotoTestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(DeleteOneImage, self).setUpClass()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.photos.deploy_photo_content('Pictures', 'email')

    def setUp(self):
        super(DeleteOneImage, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._deleteImage = ImageDelete()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(DeleteOneImage, self).tearDown()
        self.photos.rm_delete_photos()

    def test_thumbnailmode_press_menukey_select_photo_delete(self):
        self.photos.launch_photos_am()
        self._deleteImage.check_picture_thumbnailmode()
        before_num = self.photos.check_pic_number()
        self.photos.delete_photos_in_a_folder("Pictures", 1)
        after_num = self.photos.check_pic_number()
        assert before_num - after_num == 1, "delete photos failed,before num is %s,after num is %s" % (before_num, after_num)
        self.photos.stop_photos_am()
