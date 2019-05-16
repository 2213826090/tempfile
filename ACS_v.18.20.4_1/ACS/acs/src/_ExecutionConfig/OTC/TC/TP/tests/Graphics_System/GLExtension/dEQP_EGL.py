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
@since: 12/01/2015
@author: Zhao Xiangyi
'''
import os
from testlib.graphics.dEQP_impl import dEQPImpl
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.common.base import clearTmpDir

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
        self._deqp.setup()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RundEQP, self).tearDown()
        self._oglcnform = None

    def test_EGL_extension_EGL_ANDROID_blob_cache(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension.egl_android_blob_cache")

    def test_EGL_extension_EGL_ANDROID_image_native_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.info.extensions*", True, "EGL_ANDROID_image_native_buffer")

    def test_EGL_extension_EGL_ANDROID_native_fence_sync(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension.egl_android_native_fence_sync")

    def test_EGL_extension_EGL_ANDROID_recordable(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.info.extensions*", True, "EGL_ANDROID_recordable")

    def test_EGL_extension_EGL_KHR_image(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension.egl_khr_image")

    def test_EGL_extension_EGL_KHR_image_base(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension.egl_khr_image_base")

    def test_EGL_extension_GL_OES_blend_equation_separate(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension.gl_oes_blend_equation_separate")

    def test_EGL_extension_GL_OES_blend_func_separate(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension.gl_oes_blend_func_separate")

    def test_EGL_extension_GL_OES_blend_subtract(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension.gl_oes_blend_subtract")

    def test_EGL_extension_GL_OES_EGL_image(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension.gl_oes_egl_image")

    def test_EGL_extension_GL_OES_fixed_point(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension.gl_oes_fixed_point")

    def test_EGL_extension_EGL_KHR_partial_update(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.info.extensions*", True, "EGL_KHR_partial_update")

    def test_EGL_extension_EGL_EXT_swap_buffers_with_damage(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.info.extensions*", True, "EGL_EXT_swap_buffers_with_damage")

    def test_EGL_extension_GL_OES_framebuffer_object(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension.gl_oes_framebuffer_object")

    def test_EGL_extension_GL_OES_get_program_binary(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension.gl_oes_get_program_binary")

    def test_EGL_extension_GL_OES_mapbuffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension.gl_oes_mapbuffer")

    def test_EGL_extension_GL_OES_point_size_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension.gl_oes_point_size_array")

    def test_EGL_extension_GL_OES_single_precision(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension.gl_oes_single_precision")

    def test_EGL_extension_GL_OES_texture_cube_map(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension.gl_oes_texture_cube_map")

    def test_EGL_extension_EGL_ANDROID_get_native_client_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.info.extensions*", True, "EGL_ANDROID_get_native_client_buffer")

    def test_EGL_extension_EGL_ANDROID_presentation_time(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.info.extensions*", True, "EGL_ANDROID_presentation_time")

    def test_EGL_extension_EGL_KHR_get_all_proc_addresses(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.info.extensions*", True, "EGL_KHR_get_all_proc_addresses")
