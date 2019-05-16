# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 06/3/2015
@author: Zhang RongX,Z
'''
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.ImageApp_PhotoGrid_impl import PhotoGridEdit
from testlib.util.common import g_common_obj
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.common import get_resource_from_atifactory, pkgmgr


class PhotoGrid(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(PhotoGrid, self).setUpClass()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        apk_path = get_resource_from_atifactory('tests.tablet.artifactory.conf',
                                                'content_photogrid',
                                                'name')
        pkgmgr.apk_install(apk_path)
        self.photos.deploy_photo_content('Pictures','email')

    def setUp(self):
        print "setup"
        super(PhotoGrid, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._photogrid_edit = PhotoGridEdit()
        self._photogrid_edit.fresh_sdcard()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(PhotoGrid, self).tearDown()
        self.photos.rm_delete_photos()
        PhotoGridEdit().uninstall_app()

    def test_photogrid(self):
        print "edit a picture using  apk photogrid"
        self._photogrid_edit.launch_app_am()
        self._photogrid_edit.choose_pic()
        self._photogrid_edit.edit_pic()
        self._photogrid_edit.save_pic()
        g_common_obj.assert_exp_happens()
        self._photogrid_edit.stop_app_am()