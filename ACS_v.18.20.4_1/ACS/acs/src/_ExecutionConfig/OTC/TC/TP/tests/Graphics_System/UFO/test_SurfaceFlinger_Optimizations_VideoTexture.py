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
@since: 11/24/2014
@author: Xiangyi Zhao
'''
import os
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.util.config import TestConfig
from testlib.graphics.oglconform_impl import OglconformImpl

class UfoTest(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install conformance test tool
        """
        super(UfoTest, self).setUpClass()
        g_common_obj.root_on_device()

    @classmethod
    def tearDownClass(self):
        """
        uninstall conformance test tool
        """
        g_common_obj.adb_cmd_common('rm -rf /data/app/conform')
        g_common_obj.adb_cmd_common('rm -rf /data/app/covegl')
        g_common_obj.adb_cmd_common('rm -rf /data/app/covgl')
        g_common_obj.adb_cmd_common('rm -rf /data/app/primtest')
        g_common_obj.adb_cmd_common('rm -rf /data/app/android_standalone_x86/')
        g_common_obj.adb_cmd_common('rm -rf /data/app/oglconform')
        g_common_obj.adb_cmd_common('rm -rf /data/app/egl-config')
        g_common_obj.adb_cmd_common('rm -rf /data/app/get-program-binary_es3')
        super(UfoTest, self).tearDownClass()

    def setUp(self):
        super(UfoTest, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._oglconform = OglconformImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(UfoTest, self).tearDown()
        self._oglcnform = None

    def testSurfaceFlingerOptimizations_VideoTexture(self):
        """
        @summary: UFO-> testSurfaceFlingerOptimizations_VideoTexture
        """
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_EGL_image_external')
        self._oglconform.check_extension('es2', 'GL_OES_EGL_image_external')
