# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 09/07/2015
@author: Xiangyi Zhao
'''

from testlib.util.uiatestbase import UIATestBase
# from testlib.graphics.ImageApp_FLVPlayer_impl import FLVVideoPlayer
# from testlib.util.config import TestConfig
# from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
# from testlib.util.common import g_common_obj
from testlib.graphics.composeui_impl import ComposeUiImpl
# from testlib.graphics.photos_impl import get_photo_implement
from testlib.graphics.fit_impl import FitImpl


class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        #### Replace with MX
        # self._flvvideoplayer = FLVVideoPlayer()
        # self._composeui = ComposeUiImpl()
        # self.photos = get_photo_implement()
        # config = TestConfig()
        # cfg_file = 'tests.tablet.composeui.conf'
        # cfg_arti = config.read(cfg_file, 'artifactory')
        # config_handle = ConfigHandle()
        # cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        # arti = Artifactory(cfg_arti.get('location'))
        # cfg_apk = config.read(cfg_file, 'FLVplayer')
        # binary_name = cfg_apk.get("name")
        # file_path = arti.get(binary_name)
        # result = config_handle.check_apps("air.br.com.bitlabs.FLVPlayer")
        # if result == 0:
        #     g_common_obj.adb_cmd_common('install ' + file_path)
        # self.photos.rm_delete_photos()
        # self.photos.refresh_sdcard()
        # self._composeui.init_local_video()
        self._composeui = ComposeUiImpl()
        self._fit = FitImpl()
        config_handle = ConfigHandle()
        result = config_handle.check_apps("com.mxtech.videoplayer.ad")
        if result == 0:
            self._composeui.install_apk('Mxplayer')
        self._composeui.init_local_video()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()
        # self._composeui.delete_local_video()
        # self._flvvideoplayer.uninstall_app()
        self._composeui.uninstall_mxtech()
        self._fit.open_wifi()
        self._composeui.delete_local_video()

    def test_VideoPlayer_FLV(self):
        ''' refer TC test_VideoPlayer_FLV
        '''
        print "[RunTest]: %s" % self.__str__()
        # self._flvvideoplayer.launch_app_am()
        # self._flvvideoplayer.play_video()
        # self._flvvideoplayer.stop_app_am()
        self._composeui.launch_mxtech_am()
        self._composeui.check_notification_during_playing()
        self._composeui.stop_mxtech_am()