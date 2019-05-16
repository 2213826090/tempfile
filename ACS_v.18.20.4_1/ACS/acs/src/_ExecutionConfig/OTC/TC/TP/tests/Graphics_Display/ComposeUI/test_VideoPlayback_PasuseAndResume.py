# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/30/2015
@author: Yingjun Jin
'''
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.composeui_impl import ComposeUiImpl
from testlib.graphics.photos_impl import get_photo_implement


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
        self.photos.rm_delete_photos()
        self.photos.refresh_sdcard()

    def test_videoplayback_pasuseandresume(self):
        ''' refer TC test_VideoPlayback_PasuseAndResume
        '''
        print "[RunTest]: %s" % self.__str__()
        self.photos.launch_photos_am()
        self._composeui.play_video_and_pause_resume()
        self.photos.stop_photos_am()
