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
@since: 03/07/2016
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

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RundEQP, self).tearDown()
        self._oglcnform = None

    def test_GLES2_extension_GL_EXT_discard_framebuffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.other.GL_EXT_discard_framebuffer")

    def test_GLES2_extension_GL_EXT_multi_draw_arrays(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.other.GL_EXT_multi_draw_arrays")

    def test_GLES2_extension_GL_EXT_texture_format_BGRA8888(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.uncompressed_texture_formats.GL_EXT_texture_format_BGRA8888")

    def test_GLES2_extension_GL_OES_compressed_paletted_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.compressed_texture_formats.GL_OES_compressed_paletted_texture")

    def test_GLES2_extension_GL_OES_depth_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.texture.GL_OES_depth_texture")

    def test_GLES2_extension_GL_OES_depth24(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.fbo.GL_OES_depth24")

    def test_GLES2_extension_GL_OES_draw_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.other.GL_OES_draw_texture")

    def test_GLES2_extension_GL_OES_element_index_uint(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.vertex_data_formats.GL_OES_element_index_uint")

    def test_GLES2_extension_GL_OES_mapbuffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.other.GL_OES_mapbuffer")

    def test_GLES2_extension_GL_OES_packed_depth_stencil(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.fbo.GL_OES_packed_depth_stencil")

    def test_GLES2_extension_GL_OES_rgb8_rgba8(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.fbo.GL_OES_rgb8_rgba8")

    def test_GLES2_extension_GL_OES_standard_derivatives(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.shaders.GL_OES_standard_derivatives")

    def test_GLES2_extension_GL_OES_texture_npot(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.uncompressed_texture_formats.GL_OES_texture_npot")

    def test_GLES2_extension_GL_OES_vertex_array_object(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.other.GL_OES_vertex_array_object")

    def test_GLES2_extension_GL_OES_vertex_half_float(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.vertex_data_formats.GL_OES_vertex_half_float")

    def test_GLES31_AEP_macros_GL_ANDROID_extension_pack_es31a(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.shaders.extension_macros.android_extension_pack_es31a")

    def test_GLES31_AEP_limits(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.limits.*")

    def test_GLES31_AEP_shaders_extension_directive(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.shaders.extension_directive.*")

    def test_GLES31_AEP_shaders_implementation_limits(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.shaders.implementation_limits.*")

    def test_GLES31_AEP_EXT_copy_image(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.ext_copy_image")

    def test_GLES31_AEP_EXT_draw_buffers_indexed(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.ext_draw_buffers_indexed")

    def test_GLES31_AEP_EXT_geometry_shader(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.ext_geometry_shader")

    def test_GLES31_AEP_EXT_gpu_shader5(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.ext_gpu_shader5")

    def test_GLES31_AEP_EXT_primitive_bounding_box(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.ext_primitive_bounding_box")

    def test_GLES31_AEP_EXT_shader_io_blocks(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.ext_shader_io_blocks")

    def test_GLES31_AEP_EXT_tessellation_shader(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.ext_tessellation_shader")

    def test_GLES31_AEP_EXT_texture_border_clamp(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.ext_texture_border_clamp")

    def test_GLES31_AEP_EXT_texture_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.ext_texture_buffer")

    def test_GLES31_AEP_EXT_texture_cube_map_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.ext_texture_cube_map_array")

    def test_GLES31_AEP_EXT_texture_sRGB_decode(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.ext_draw_buffers_indexed")

    def test_GLES31_AEP_KHR_blend_equation_advanced(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.khr_blend_equation_advanced")

    def test_GLES31_AEP_KHR_debug(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.khr_debug")

    def test_GLES31_AEP_KHR_texture_compression_astc_ldr(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.khr_texture_compression_astc_ldr")

    def test_GLES31_AEP_OES_sample_shading(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.oes_sample_shading")

    def test_GLES31_AEP_OES_sample_variables(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.oes_sample_variables")

    def test_GLES31_AEP_OES_shader_image_atomic(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.oes_shader_image_atomic")

    def test_GLES31_AEP_OES_shader_multisample_interpolation(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.oes_shader_multisample_interpolation")

    def test_GLES31_AEP_OES_texture_stencil8(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.oes_texture_stencil8")

    def test_GLES31_AEP_OES_texture_storage_multisample_2d_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.android_extension_pack.extensions.oes_texture_storage_multisample_2d_array")

