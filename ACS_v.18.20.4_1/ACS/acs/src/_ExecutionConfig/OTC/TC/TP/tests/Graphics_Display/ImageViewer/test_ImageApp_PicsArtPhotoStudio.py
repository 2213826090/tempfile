# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 05/13/2015
@author: Xiangyi Zhao
'''

from testlib.graphics.test_template.phototestbase import PhotoTestBase
from testlib.graphics.ImageApp_PicsArtPhotoStudio_impl import PicsArtStudio
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.common import get_resource_from_atifactory, pkgmgr


class PicsArtPhotoStudio(PhotoTestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(PicsArtPhotoStudio, self).setUpClass()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        apk_path = get_resource_from_atifactory('tests.tablet.artifactory.conf',
                                                'content_picsart',
                                                'name')
        pkgmgr.apk_install(apk_path)
        self.photos.deploy_photo_content('Pictures', 'email')

    def setUp(self):
        super(PicsArtPhotoStudio, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._picsArt = PicsArtStudio()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(PicsArtPhotoStudio, self).tearDown()
        self.photos.rm_delete_photos()
        self._picsArt.uninstall_app()

    def test_picsartstudio(self):
        self._picsArt.launch_app_am()
        self._picsArt.choose_pic()
        self._picsArt.edit_pic()
        self._picsArt.save_pic()
        self._picsArt.check_pic()
        self._picsArt.stop_app_am()