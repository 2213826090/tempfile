# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 08/06/2015
@author: Xiangyi Zhao
'''
from testlib.graphics.test_template.phototestbase import PhotoTestBase
from testlib.graphics.photos_impl import get_photo_implement


class ImageEdit(PhotoTestBase):

    @classmethod
    def setUpClass(self):
        """
        set environment
        """
        super(ImageEdit, self).setUpClass()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()

    def setUp(self):
        super(ImageEdit, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ImageEdit, self).tearDown()
        self.photos.rm_delete_photos()

    def test_ImageEdit_SelectRandomMediaeffect(self):
        ''' refer TC test_ImageEdit_SelectRandomMediaeffect
        '''
        print "[RunTest]: %s" % self.__str__()
        self.photos.deploy_photo_content("Pictures", "email", "/sdcard/Pictures")
        self.photos.launch_photos_am()
        self.photos.open_a_picture()
        self.photos.change_display_effect(effType='Light', effVol=100)
        self.photos.save_changes()
        self.photos.stop_photos_am()
        assert self.photos.check_saved_pic_result("/sdcard/Pictures") >= 5, \
            "Failed! No effects on the saving pic."
