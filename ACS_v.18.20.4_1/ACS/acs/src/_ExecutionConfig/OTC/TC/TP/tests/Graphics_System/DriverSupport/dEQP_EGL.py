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
@summary: run Oglconform
@since: 06/13/2017
@author: rui
'''
from testlib.graphics.dEQP_impl import dEQPImpl
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.common.base import clearTmpDir
from testlib.graphics.common import Logcat


class RundEQP(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install conformance test tool
        """
        super(RundEQP, self).setUpClass()
        if g_common_obj.adb_cmd_capture_msg("ps | grep adbd")[0:4] != "root":
            g_common_obj.root_on_device()

    @classmethod
    def tearDownClass(self):
        """
        uninstall conformance test tool
        """
        super(RundEQP, self).tearDownClass()
        clearTmpDir()

    def setUp(self):
        super(RundEQP, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._deqp = dEQPImpl()
        self.log = Logcat()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RundEQP, self).tearDown()
        self._oglcnform = None

    def test_EGL_extension_EGL_ANDROID_image_native_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.info.extensions*", True, "EGL_ANDROID_image_native_buffer")

    def test_EGL_extension_EGL_ANDROID_framebuffer_target(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.info.extensions*", True, "EGL_ANDROID_framebuffer_target")

    def test_Dumpsys_SurfaceFlinger_EGL_ANDROID_framebuffer_target(self):
        print "[RunTest]: %s" % self.__str__()
        assert self.log.check_dumpsys_SurfaceFlinger_info(keyword="EGL_ANDROID_framebuffer_target", \
                                                          assertword="EGL_ANDROID_framebuffer_target"), \
            "EGL_ANDROID_framebuffer_target extension not found."

    def test_GLES31_extension_GL_EXT_texture_format_BGRA8888_color_display(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.info.extensions*", True, "GL_EXT_texture_format_BGRA8888")

    def test_GLES31_extension_GL_OES_depth24_color_display(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.info.extensions*", True, "GL_OES_depth24")

