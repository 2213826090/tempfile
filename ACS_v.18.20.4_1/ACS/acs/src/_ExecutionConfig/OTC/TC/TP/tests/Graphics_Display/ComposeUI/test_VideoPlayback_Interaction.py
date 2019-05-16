# -*- coding: utf-8 -*-
'''
@summary: Test case test_VideoPlayback_Interaction.
@since: 03/04/2016
@author: Xiangyi Zhao
'''

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.composeui_impl import ComposeUiImpl
from testlib.graphics.photos_impl import get_photo_implement
from testlib.util.common import g_common_obj


class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._composeui = ComposeUiImpl()
        self.photos = get_photo_implement()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        self._composeui.init_local_video()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()
        self._composeui.delete_local_video()
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()
        g_common_obj.set_vertical_screen()

    def test_videoplayback_interaction(self):
        ''' refer TC test_VideoPlayback_Interaction
        '''
        print "[RunTest]: %s" % self.__str__()
        self.photos.launch_photos_am()
        self._composeui.rotate_during_video_playback()
        time.sleep(2)
        g_common_obj.set_vertical_screen()
        g_common_obj.assert_exp_happens()
        self._composeui.swipedown_notification_and_rotate_during_video_playback()
        time.sleep(2)
        g_common_obj.set_vertical_screen()
        g_common_obj.assert_exp_happens()
        self.photos.stop_photos_am()
