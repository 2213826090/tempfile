# Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
# Intel Corporation All Rights Reserved.

# The source code contained or described herein and
# all documents related to the source code ("Material") are owned by
# Intel Corporation or its suppliers or licensors.

# Title to the Material remains with Intel Corporation or
# its suppliers and licensors.
# The Material contains trade secrets and proprietary and
# confidential information of Intel or its suppliers and licensors.
# The Material is protected by worldwide copyright and
# trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified,
# published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

'''
@summary: run GLBenchmark
@since: 10/08/2014
@author: Grace Yi(gracex.yi@intel.com)
'''
import os
from testlib.graphics.glbenckmark_impl import GLBenchmarkImpl
from testlib.graphics.extend_glbenckmark_impl import GLBenchmarkExtendImpl
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.common import close_all_tasks

class RunGLBenchmark(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install apk
        """
        super(RunGLBenchmark, self).setUpClass()
        config = TestConfig()
        cfg_file = 'tests.tablet.artifactory.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        cfg = config.read(cfg_file, 'content_graphic')
        arti = Artifactory(cfg_arti.get('location'))
        apk_name = cfg.get("name")
        file_path = arti.get(apk_name)
        result = config_handle.check_apps("com.glbenchmark.glbenchmark27")
        if result == 0:
            g_common_obj.adb_cmd_common('install -r ' + file_path, 300)

    @classmethod
    def tearDownClass(self):
        """
        uninstall apk
        """
        super(RunGLBenchmark, self).tearDownClass()

    def setUp(self):
        super(RunGLBenchmark, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._glBenchmark = GLBenchmarkImpl()
        self.benchmark = GLBenchmarkExtendImpl()
        close_all_tasks()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RunGLBenchmark, self).tearDown()
        self._glBenchmark.stop_app_am()
        self._glBenchmark = None
        self.benchmark.clean()

    def testEgyptHDETC1ScreenOn(self):
        """
        This test case is to Run GLBenchmark 2.7 Egypt HD ETC1 OnScreen test

        Test Case Precondition:
        GLBenchmark 2.7 apk installed on DUT

        Test Case Step:
        1. Launch GLBenchmark
        2. Open Performance Tests, select GLBenchmark 2.7 EgyptHD ETC1
        3. Start run with Onscreen

        Expect Result:
        1. GLBenchmark can run successfully and fluently

        The real implementation will be in GLBenchmarkImpl class.
        """

        print "[RunTest]: %s" % self.__str__()
        self.benchmark.launch()
        self.benchmark.run_performance_test("test23")
        self.benchmark.check_all_results()
        g_common_obj.assert_exp_happens()
