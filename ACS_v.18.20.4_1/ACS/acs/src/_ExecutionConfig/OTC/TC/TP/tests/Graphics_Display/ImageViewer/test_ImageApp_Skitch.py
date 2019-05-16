# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 05/29/2015
@author: Xiangyi Zhao
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.ImageApp_Skitch_impl import Skitch
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.common import get_resource_from_atifactory, pkgmgr


class SkitchSnap(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(SkitchSnap, self).setUpClass()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        apk_path = get_resource_from_atifactory('tests.tablet.artifactory.conf',
                                                'content_skitch',
                                                'name')
        pkgmgr.apk_install(apk_path)
        self.photos.deploy_photo_content('Pictures', 'email')

    def setUp(self):
        super(SkitchSnap, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._skitch = Skitch()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SkitchSnap, self).tearDown()
        self.photos.rm_delete_photos()
        self._skitch.uninstall_app()

    def test_skitch(self):
        self._skitch.launch_app_am()
        self._skitch.choose_pic()
        self._skitch.edit_pic()
        self._skitch.save_pic()
        self._skitch.check_pic()