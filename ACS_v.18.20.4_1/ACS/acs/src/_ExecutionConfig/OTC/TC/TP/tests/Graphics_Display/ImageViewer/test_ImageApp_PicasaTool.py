# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 06/10/2015
@author: Xiangyi Zhao
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.ImageApp_PicasaTool_impl import PicasaTool
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.common import get_resource_from_atifactory, pkgmgr


class Picasa(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(Picasa, self).setUpClass()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        apk_path = get_resource_from_atifactory('tests.tablet.artifactory.conf',
                                                'content_picasatool',
                                                'name')
        pkgmgr.apk_install(apk_path)
        self.photos.deploy_photo_content('Pictures', 'email')

    def setUp(self):
        super(Picasa, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._picasa = PicasaTool()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Picasa, self).tearDown()
        self.photos.rm_delete_photos()
        self._picasa.uninstall_app()

    def test_picasa_tool(self):
        self._picasa.launch_app_am()
        self._picasa.choose_pic()
        self._picasa.edit_pic()
        self._picasa.check_pic()
        self._picasa.stop_app_am()