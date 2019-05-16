# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 09/21/2015
@author:Zhang RongX Z
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.hwui_impl import HwuiImpl
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle

class HwuiUnitest(UIATestBase):

    def setUp(self):
        super(HwuiUnitest, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._hwui = HwuiImpl()
        if g_common_obj.adb_cmd_capture_msg("ps | grep adbd")[0:4] != "root":
            g_common_obj.root_on_device()
#         g_common_obj.remount_device()
        self.config = TestConfig()
        self.cfg_file = 'tests.hwui.binary.conf'
        cfg_arti = self.config.read(self.cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        cfg = self.config.read(self.cfg_file, 'hwui_binary')
        arti = Artifactory(cfg_arti.get('location'))
        binary_name = cfg.get("name")
        file_path = arti.get(binary_name)
        self.file_name = file_path.split("/")[-1]
        print "%s" % file_path
        g_common_obj.adb_cmd_common('push ' + file_path + ' /data/app/')
        g_common_obj.adb_cmd('chmod 777 /data/app/' + self.file_name)

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(HwuiUnitest, self).tearDown()

    def test_hwui_unitest_RectangleList_basics(self):
        ''' refer TC test_hwui_unitest_RectangleList_basics
        '''
        print "[RunTest]: %s" % self.__str__()
        self._hwui.run_hwui_unitest_case("RectangleList.basics")

    def test_hwui_unitest_TransformedRectangle_basics(self):
        ''' refer TC test_hwui_unitest_TransformedRectangle_basics
        '''
        print "[RunTest]: %s" % self.__str__()
        self._hwui.run_hwui_unitest_case("TransformedRectangle.basics")

    def test_hwui_unitest_ClipArea_basics(self):
        ''' refer TC test_hwui_unitest_ClipArea_basics
        '''
        print "[RunTest]: %s" % self.__str__()
        self._hwui.run_hwui_unitest_case("ClipArea.basics")

    def test_hwui_unitest_ClipArea_paths(self):
        ''' refer TC test_hwui_unitest_ClipArea_paths
        '''
        print "[RunTest]: %s" % self.__str__()
        self._hwui.run_hwui_unitest_case("ClipArea.paths")

    def test_hwui_unitest_ClipArea_replaceNegative(self):
        ''' refer TC test_hwui_unitest_ClipArea_replaceNegative
        '''
        print "[RunTest]: %s" % self.__str__()
        self._hwui.run_hwui_unitest_case("ClipArea.replaceNegative")

    def test_hwui_unitest_DamageAccumulator_identity(self):
        ''' refer TC test_hwui_unitest_DamageAccumulator_identity
        '''
        print "[RunTest]: %s" % self.__str__()
        self._hwui.run_hwui_unitest_case("DamageAccumulator.identity")

    def test_hwui_unitest_DamageAccumulator_translate(self):
        ''' refer TC test_hwui_unitest_DamageAccumulator_translate
        '''
        print "[RunTest]: %s" % self.__str__()
        self._hwui.run_hwui_unitest_case("DamageAccumulator.translate")

    def test_hwui_unitest_LinearAllocator_rewind(self):
        ''' refer TC test_hwui_unitest_LinearAllocator_rewind
        '''
        print "[RunTest]: %s" % self.__str__()
        self._hwui.run_hwui_unitest_case("LinearAllocator.rewind")

    def test_hwui_unitest_LinearAllocator_dtor(self):
        ''' refer TC test_hwui_unitest_LinearAllocator_dtor
        '''
        print "[RunTest]: %s" % self.__str__()
        self._hwui.run_hwui_unitest_case("LinearAllocator.dtor")

    def test_hwui_unitest_DamageAccumulator_union(self):
        ''' refer TC test_hwui_unitest_DamageAccumulator_union
        '''
        print "[RunTest]: %s" % self.__str__()
        self._hwui.run_hwui_unitest_case("DamageAccumulator.union")

    def test_hwui_unitest_LinearAllocator_alloc(self):
        ''' refer TC test_hwui_unitest_LinearAllocator_alloc
        '''
        print "[RunTest]: %s" % self.__str__()
        self._hwui.run_hwui_unitest_case("LinearAllocator.alloc")
