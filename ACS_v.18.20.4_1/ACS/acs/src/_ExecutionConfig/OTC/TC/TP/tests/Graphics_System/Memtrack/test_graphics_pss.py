# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 02/05/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.memtrack_impl import MemTrackImpl
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
import os


class MemTrack(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(MemTrack, self).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_graphic')
        arti = Artifactory(cfg_arti.get('location'))
        apk_name = cfg.get("name")
        file_path = arti.get(apk_name)
        os.system('adb install -r -g %s' % file_path)

    def setUp(self):
        super(MemTrack, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._memtrack = MemTrackImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(MemTrack, self).tearDown()
        self._memtrack.uninstall_app()
        self._memtrack.set_default_screen()

    def test_graphics_pss(self):
        ''' refer TC test_Graphics_PSS
        '''
        before_size = self._memtrack.check_graphics_mem()
        self._memtrack.launch_app_am()
        self._memtrack.run_performance_test()
        after_size = self._memtrack.check_graphics_mem()
        differ = abs(before_size - after_size)
        assert differ > 0, "The Graphics mem is not changed"
        self._memtrack.stop_app_am()
