# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 11/24/2015
@author: Xiangyi Zhao
'''

from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.graphics.grafika_impl import grafika
import os

class Render(RenderAppTestBase):
    """
    Graphic Test
    """
    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(Render, self).setUpClass()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_grafika')
        arti = Artifactory(cfg_arti.get('location'))
        apk_name = cfg.get("name")
        file_path = arti.get(apk_name)
        result = config_handle.check_apps("grafika-debug.apk")
        if result == 0:
            g_common_obj.adb_cmd_common('install ' + file_path)
        video_name = cfg.get("video")
        self.video_file = video_name.split('/')[-1]
        self.video_file_path = arti.get(video_name)

    def setUp(self):
        super(Render, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._grafika = grafika()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Render, self).tearDown()
#         g_common_obj.adb_cmd('rm -rf /data/data/com.android.grafika/files/')
        self._grafika.uninstall_app()

    def test_SurfaceTexture_VideoPlackback(self):
        """test_SurfaceTexture_VideoPlackback"""
        self._grafika.launch_app_am()
        #g_common_obj.adb_cmd_common('push ' + self.video_file_path + ' /data/data/com.android.grafika/files/' + self.video_file)
        os.system(' adb push ' + self.video_file_path + ' /data/data/com.android.grafika/files/' + self.video_file)
        self._grafika.play_movie()
        self._grafika.stop_app_am()
