# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 01/26/2015
@author: Yingjun Jin
'''

import os
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.util.config import TestConfig
from testlib.graphics.gits_impl import GitsImpl


class ImageViewer(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(ImageViewer, self).setUpClass()
        self.config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        self.case_cfg = 'tests.tablet.gits.conf'
        GitsImpl.setup_enviroment(self.config.read(cfg_file, "content_gits"))

    def setUp(self):
        super(ImageViewer, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.gits = GitsImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ImageViewer, self).tearDown()
        self.gits.remove_file()

    def test_ImageEdit_AddSharpnessEffect(self):
        """
        refer TC test_ImageEdit_AddSharpnessEffect_ImageSize5MB
                   test_ImageEdit_AddSharpnessEffect_SaveImage
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'sharpness_effect')

    def test_ImageEdit_GIF_CancelEdit(self):
        """
        refer TC test_ImageEdit_EditGIFImage_CancelEdit
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'GIF_Cancel_Edit')

    def test_ImageEdit_GIF_Crop(self):
        """
        refer TC test_ImageEdit_GIF_Crop
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'GIF_Crop')

    def test_ImageEdit_GIF_SaveEdit(self):
        """
        refer TC test_ImageEdit_EditGIFImage_SaveImage
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'GIF_Save_Edit')

    def test_ImageEdit_JPG_PersonFaceEdit(self):
        """
        refer TC test_ImageEdit_JPEGImageOfPersonFace_MoreEffects
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'JPG_Face')

    def test_ImageEdit_GIF_Rotate(self):
        """
        refer TC test_ImageView_GIF_Rotate
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'GIF_Rotate')

    def test_ImageEdit_PNG_1K_Check(self):
        """
        refer TC test_ImageView_PNG_ImageSize1k
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'PNG_1K')

    def test_ImageEdit_PNG_2M_Check(self):
        """
        refer TC test_ImageView_PNG_ImageSize2M
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'PNG_2M')

    def test_ImageEdit_PNG_SemiTransParent_Check(self):
        """
        refer TC test_ImageView_PNG_Semitransparent
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'PNG_STP')

    def test_ImageEdit_Crop_ThumbnailMode(self):
        """
        refer TC test_ImageEdit_Crop_ThumbnailMode
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'Crop_ThumbnailMode')

    def test_ImageEdit_AddSharpnessEffect_ImageSize5MB(self):
        """
        refer TC test_ImageEdit_AddSharpnessEffect_ImageSize5MB
        """
        print "[RunTest]: %s" % self.__str__()
        self.gits.handle_test(self.case_cfg, 'SharpnessEffect_5MB')
