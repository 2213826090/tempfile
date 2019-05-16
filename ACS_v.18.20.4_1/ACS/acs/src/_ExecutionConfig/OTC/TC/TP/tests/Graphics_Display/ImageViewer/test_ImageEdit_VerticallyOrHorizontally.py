# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 08/07/2015
@author: Xiangyi Zhao
'''
from testlib.graphics.test_template.phototestbase import PhotoTestBase
# from testlib.graphics.ImageApp_MeituPic_impl import MeituPicEdit
from testlib.graphics.ImageApp_PhotoGrid_impl import PhotoGridEdit
from testlib.util.common import g_common_obj
from testlib.graphics.photos_impl import get_photo_implement
from testlib.common.base import clearTmpDir
from testlib.graphics.common import get_resource_from_atifactory, pkgmgr


class ImageEdit(PhotoTestBase):

    @classmethod
    def setUpClass(self):
        """
        set environment
        """
        super(ImageEdit, self).setUpClass()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        apk_path = get_resource_from_atifactory('tests.tablet.artifactory.conf',
                                                'content_photogrid',
                                                'name')
        pkgmgr.apk_install(apk_path)
        self.photos.deploy_photo_content('Pictures','email')

    def setUp(self):
        super(ImageEdit, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ImageEdit, self).tearDown()
        clearTmpDir()
        self.photos.rm_delete_photos()
        PhotoGridEdit().uninstall_app()

    def test_ImageEdit_VerticallyOrHorizontally(self):
        ''' refer TC test_ImageEdit_VerticallyOrHorizontally
        '''
        print "[RunTest]: %s" % self.__str__()
        PhotoGridEdit().launch_app_am()
        PhotoGridEdit().choose_pic()
        PhotoGridEdit().function_add_image()
        PhotoGridEdit().save_pic()
        g_common_obj.assert_exp_happens()
        PhotoGridEdit().stop_app_am()
