# -*- coding: utf-8 -*-
'''
Created on 11/9/2015
@author: Zhao, XiangyiX
'''

import os

from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.camera.camera_impl import CameraImpl
from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.extend_systemui_impl import SystemUiExtendImpl
from testlib.graphics.HDMI_switch_card_impl import HDMISwitchCardImpl


class HDMITest(UIATestBase):

    @classmethod
    def setUpClass(self):
        print "[setUpClass]: %s" % self.__name__
        super(HDMITest, self).setUpClass()
        self.photosImpl = get_photo_implement()

        self.photosImpl.rm_delete_photos()
        self.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(HDMITest, self).setUp()

        self.systemui = SystemUiExtendImpl()
        self.camera = CameraImpl()
        self.HDMI_switch_card = HDMISwitchCardImpl()
        self.HDMI_switch_card.setup()
        self.HDMI_switch_card.switch_off()

        self.photosImpl.refresh_sdcard()
        self.systemui.unlock_screen()
        self.d.screen.on()
        self.d.orientation = 'natural'

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(HDMITest, self).tearDown()
        self.photosImpl.rm_delete_photos()
        self.photosImpl.refresh_sdcard()

    def test_HDMI_TakePicture(self):
        """
        test_HDMI_TakePicture
        """
        print "[RunTest]: %s" % self.__str__()
        self.HDMI_switch_card.switch_on()
        self.camera.startApp()
        self.camera.picTake()
        self.photosImpl.launch_photos_am()
        self.photosImpl.delete_photos_in_a_folder("Camera", 1)
        self.camera.videoRecord()
        g_common_obj.assert_exp_happens()
        self.HDMI_switch_card.switch_off()
