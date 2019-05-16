# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/24/2015
@author: Yingjun Jin
'''
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.imagedetails_impl import ImageDetails
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.graphics.photos_impl import get_photo_implement


class ImageEdit(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(ImageEdit, self).setUpClass()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_picture')
        arti = Artifactory(cfg_arti.get('location'))
        pic_name = cfg.get("bmp")
        file_path = arti.get(pic_name)
        g_common_obj.adb_cmd_common('push ' + file_path + ' /sdcard/Pictures')
        ImageDetails.set_workaround()

    def setUp(self):
        super(ImageEdit, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._imageviewer = ImageDetails()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ImageEdit, self).tearDown()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()

    def test_imageedit_rotateleftmorethan6times(self):
        ''' refer TC test_ImageEdit_RotateLeftMoreThan6times
        '''
        print "[RunTest]: %s" % self.__str__()
        self.photos.launch_photos_am()
        self._imageviewer.open_photo_and_rotate_left_7times()
        self.photos.stop_photos_am()
