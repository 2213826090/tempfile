# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 12/02/2015
@author: Zhang,RongX Z
'''
import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.imagedetails_impl import ImageDetails
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.QRcode_impl import QRcode
from testlib.graphics.extend_chrome_impl import ChromeExtendImpl
from testlib.graphics.common import file_sys


class ImageViewByWebBrowser(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        set environment
        """
        super(ImageViewByWebBrowser, self).setUpClass()
        self.photos = get_photo_implement()
        self._qrcode = QRcode()
        self._chrome = ChromeExtendImpl()
        self.photos.rm_delete_photos()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'qrcode_marked_image')
        arti = Artifactory(cfg_arti.get('location'))
        pic_name = cfg.get("bmp01")
        self.pic_name1 = pic_name.split("/")[-1]
        self.qrcode1 = cfg.get("bmp01_qrcode")
        file_path = arti.get(pic_name)
        self.photos.deploy_photo_content(local_file=file_path)

        pic_name = cfg.get("bmp02")
        self.pic_name2 = pic_name.split("/")[-1]
        self.qrcode2 = cfg.get("bmp02_qrcode")
        file_path = arti.get(pic_name)
        self.photos.deploy_photo_content(local_file=file_path)

        self._chrome.launch()
        self._chrome.chrome_setup()

    def setUp(self):
        super(ImageViewByWebBrowser, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._imageviewer = ImageDetails()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ImageViewByWebBrowser, self).tearDown()
        self.photos.rm_delete_photos()

    def test_ImageView_ByWebBrowser_BMP(self):
        ''' refer TC test_ImageView_ByWebBrowser_BMP
        '''
        print "[RunTest]: %s" % self.__str__()
        pics = {self.pic_name1: self.qrcode1, self.pic_name2: self.qrcode2}
        for p in pics.keys():
            self._chrome.launch()
            self._chrome.open_website("file:///mnt/sdcard/Pictures/%s" % p)
            time.sleep(5)
            self.photos.refresh_sdcard()
            self.photos.open_image_command("/sdcard/Download/%s" % p)
            self._qrcode.verify_qrcode_marked_image(pics[p], False)
            g_common_obj.assert_exp_happens()
