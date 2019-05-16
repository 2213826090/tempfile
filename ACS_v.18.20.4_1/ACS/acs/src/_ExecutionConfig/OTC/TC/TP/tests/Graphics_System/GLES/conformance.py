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
@since: 01/06/2015
@author: Shuang He(shuang.he@intel.com)
'''
import os
from testlib.graphics.oglconform_impl import OglconformImpl
from testlib.graphics.GLCTS_impl import GLCTSImpl
from testlib.graphics.conform_impl import ConformImpl
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.util.config import TestConfig

class RunConformance(UIATestBase):

    @classmethod
    def setUpClass(self):
        """
        install conformance test tool
        """
        super(RunConformance, self).setUpClass()
        if g_common_obj.adb_cmd_capture_msg("ps | grep adbd")[0:4] != "root":
            g_common_obj.root_on_device()
#         config = TestConfig()
#         cfg_file = 'tests.tablet.artifactory.conf'
#         cfg_arti = config.read(cfg_file, 'artifactory')
#         config_handle = ConfigHandle()
#         cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
#         cfg = config.read(cfg_file, 'content_oglconform')
#         arti = Artifactory(cfg_arti.get('location'))
#         binary_name = cfg.get("name")
#         file_path = arti.get(binary_name)
#         g_common_obj.push_file(file_path, "/data/app/oglconform")
#         g_common_obj.adb_cmd_common('shell chmod 777 /data/app/oglconform')
#         egl_config = cfg.get("egl_config")
#         egl_config_file_path = arti.get(egl_config)
#         g_common_obj.push_file(egl_config_file_path, "/data/app/egl-config")
#         g_common_obj.adb_cmd_common('shell chmod 777 /data/app/egl-config')
#         egl_config = cfg.get("binary_es3")
#         egl_config_file_path = arti.get(egl_config)
#         g_common_obj.push_file(egl_config_file_path, "/data/app/get-program-binary_es3")
#         g_common_obj.adb_cmd_common('shell chmod 777 /data/app/get-program-binary_es3')
# 
#         cfg = config.read(cfg_file, 'content_gits')
#         arti = Artifactory(cfg_arti.get('location'))
#         binary_name = cfg.get("busybox")
#         file_path = arti.get(binary_name)
#         g_common_obj.push_file(file_path, "/data/busybox")
#         g_common_obj.adb_cmd_common('shell chmod 777 /data/busybox')
# 
#         cfg = config.read(cfg_file, 'content_GLCTS')
#         arti = Artifactory(cfg_arti.get('location'))
#         binary_name = cfg.get("name")
#         file_path = arti.get(binary_name)
# #       g_common_obj.shell_cmd("tar zxf %s" % (file_path))
#         
# 	# TODO: Fix stuck issue
#         print "push android_standalone_x86/ to DUT"
# 	g_common_obj.push_file(file_path, "/data/app/")
# # 	g_common_obj.push_file("android_standalone_x86", "/data/app/")
# #       g_common_obj.adb_cmd_common("push android_standalone_x86.tgz /data/app/", 600)
#         g_common_obj.adb_cmd_common("shell /data/busybox tar xzvf /data/app/android_standalone_x86.tgz -C /data/app", 10)
# #       g_common_obj.push_file("android_standalone_x86/", "/data/app/android_standalone_x86")
# #       os.system("adb push android_standalone_x86/ /data/app/android_standalone_x86/ > /dev/null")
#         g_common_obj.shell_cmd("rm -rf android_standalone_x86*")
#         g_common_obj.adb_cmd_common('shell rm /data/app/android_standalone_x86.tgz')
#         g_common_obj.adb_cmd_common('shell chmod 777 /data/app/android_standalone_x86/glcts')
#         g_common_obj.adb_cmd_common('shell chmod 777 /data/app/android_standalone_x86/test.sh')
# 
#         cfg = config.read(cfg_file, 'content_conform')
#         arti = Artifactory(cfg_arti.get('location'))
#         binary_name = cfg.get("name")
# 	file_path = arti.get(binary_name)
# 	g_common_obj.push_file(file_path, "/data/app/conform")
#         g_common_obj.adb_cmd_common('shell chmod 777 /data/app/conform')
#         binary_name = cfg.get("covegl")
# 	file_path = arti.get(binary_name)
# 	g_common_obj.push_file(file_path, "/data/app/covegl")
#         g_common_obj.adb_cmd_common('shell chmod 777 /data/app/covegl')
#         binary_name = cfg.get("covgl")
# 	file_path = arti.get(binary_name)
# 	g_common_obj.push_file(file_path, "/data/app/covgl")
#         g_common_obj.adb_cmd_common('shell chmod 777 /data/app/covgl')
#         binary_name = cfg.get("primtest")
# 	file_path = arti.get(binary_name)
# 	g_common_obj.push_file(file_path, "/data/app/primtest")
#         g_common_obj.adb_cmd_common('shell chmod 777 /data/app/primtest')


    @classmethod
    def tearDownClass(self):
        """
        uninstall conformance test tool
        """
        super(RunConformance, self).tearDownClass()

    def setUp(self):
        super(RunConformance, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._oglconform = OglconformImpl()
        self._glcts = GLCTSImpl()
        self._conform = ConformImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RunConformance, self).tearDown()
        self._oglcnform = None

    def test_EGL_ANDROID_blob_cache(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_libGL('es1', 'EGL_ANDROID_blob_cache')
        self._oglconform.check_extension_libGL('es2', 'EGL_ANDROID_blob_cache')

    def test_EGL_ANDROID_native_fence_sync(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_libGL('es2', 'EGL_ANDROID_native_fence_sync')

    def test_EGL_ANDROID_native_fence_sync_functionality(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.run_case_libGL("es2", "egl-android-sync")

    def test_EGL_KHR_image_base(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension("es1", "EGL_KHR_image_base")
        self._oglconform.check_extension("es2", "EGL_KHR_image_base")

    def test_EGL_KHR_gl_texture_2D_image(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension("es1", "EGL_KHR_gl_texture_2D_image")
        self._oglconform.check_extension("es2", "EGL_KHR_gl_texture_2D_image")
        self._oglconform.run_case("es2", "egl-image", "1.1.1")
        self._oglconform.run_case("es2", "egl-image", "1.1.2")
        self._oglconform.run_case("es2", "egl-image", "1.1.3")
        self._oglconform.run_case("es2", "egl-image", "2.1.1")
        self._oglconform.run_case("es2", "egl-image", "2.1.2")

    def test_EGL_KHR_gl_texture_cubemap_image(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension("es1", "EGL_KHR_gl_texture_cubemap_image")
        self._oglconform.check_extension("es2", "EGL_KHR_gl_texture_cubemap_image")
        self._oglconform.run_case("es2", "egl-image", "1.4.1")
        self._oglconform.run_case("es2", "egl-image", "1.4.2")

    def test_EGL_KHR_gl_renderbuffer_image(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension("es1", "EGL_KHR_gl_renderbuffer_image")
        self._oglconform.check_extension("es2", "EGL_KHR_gl_renderbuffer_image")
        self._oglconform.run_case("es2", "egl-image", "1.5.1")
        self._oglconform.run_case("es2", "egl-image", "1.5.2")
        self._oglconform.run_case("es2", "egl-image", "1.5.3")
        self._oglconform.run_case("es2", "egl-image", "1.5.4")

    def test_EGL_ANDROID_framebuffer_target(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_libGL("es1", "EGL_ANDROID_framebuffer_target")
        self._oglconform.check_extension_libGL("es2", "EGL_ANDROID_framebuffer_target")
        self._oglconform.run_case_libGL("es2", "egl-android-fb")

    def test_GL_OES_required_internalformat(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension("es1", "GL_OES_required_internalformat")
        self._oglconform.check_extension("es2", "GL_OES_required_internalformat")
        self._oglconform.run_case("es2", "req_internalformat_es")

    def test_GL_OES_get_program_binary(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_libGL("es2", "GL_OES_get_program_binary")
        self._oglconform.check_extension_libGL("es3", "GL_OES_get_program_binary")
        self._oglconform.run_case_libGL("es2", "get-program-binary_es2")
        self._oglconform.run_case_program_binary_es3()

    def test_texture_compression_etc1(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.compressed_etc1_rgb8_texture.compressed_etc1_rgb8_texture")

    def test_texture_compression_etc2(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.etc2_compression_rgb8.gl_compressed_rgb8_etc2")
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.etc2_compression_rgb8_pt_alpha1.gl_compressed_rgb8_pt_alpha1_etc2")
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.etc2_compression_rgba8.gl_compressed_rgba8_etc2")

    def test_texture_compression_dxtc(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.run_case("es2", "tex-compress-dxt135_es")

    def test_GL_OES_mapbuffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension("es1", "GL_OES_mapbuffer")
        self._oglconform.check_extension("es2", "GL_OES_mapbuffer")
        self._oglconform.run_case("es1", "mapbuffer_es1")
        self._oglconform.run_case("es2", "mapbuffer_es")

    def test_GL_OES_rgb8_rgba8(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension("es1", "GL_OES_rgb8_rgba8")
        self._oglconform.check_extension("es2", "GL_OES_rgb8_rgba8")
        self._oglconform.run_case("es1", "fbo_es1", "7.7")
        self._oglconform.run_case("es1", "fbo_es1", "7.8")
        self._oglconform.run_case("es1", "fbo_es1", "7.9")
        self._oglconform.run_case("es1", "fbo_es1", "7.10")
        self._oglconform.run_case("es2", "fbo_es", "7.7")
        self._oglconform.run_case("es2", "fbo_es", "7.8")
        self._oglconform.run_case("es2", "fbo_es", "7.9")
        self._oglconform.run_case("es2", "fbo_es", "7.10")

    def test_EGL_ANDROID_recordable(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'EGL_ANDROID_recordable')
        self._oglconform.check_extension('es2', 'EGL_ANDROID_recordable')
        self._oglconform.run_case("es1", "blend-subtract_es1")

    def test_GL_OES_blend_subtract(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_blend_subtract')
        self._oglconform.run_case("es1", "blend-subtract_es1")

    def test_GL_OES_depth_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es2', 'GL_OES_depth_texture')
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.depth_texture.depth_texture")

    def test_GL_OES_texture_float(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es2', 'GL_OES_texture_float')
        self._oglconform.run_case("es2", "tex-extrg_es")
        self._oglconform.run_case("es2", "tex-float_es")

    def test_GL_OES_vertex_half_float(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es2', 'GL_OES_vertex_half_float')
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.vertex_half_float.vertex_half_float")

    def test_GL_OES_element_index_uint(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es2', 'GL_OES_element_index_uint')
        # TODO: This test case need further confirm
        # self._oglconform.run_case("es2", "draw-unitindex_es")
        self._oglconform.run_case("es2", "mapbuffer_es")

    def test_GL_EXT_discard_framebuffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es2', 'GL_EXT_discard_framebuffer')
        self._oglconform.run_case("es2", "discard-fbo_es")

    def test_GL_OES_vertex_array_object(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es2', 'GL_OES_vertex_array_object')
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.vertex_array_object.vertex_array_object")

    def test_GL_OES_packed_depth_stencil(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es2', 'GL_OES_packed_depth_stencil')
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.packed_depth_stencil.packed_depth_stencil_init")
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.packed_depth_stencil.packed_depth_stencil_error")
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.packed_depth_stencil.packed_depth_stencil_readpixels")
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.packed_depth_stencil.packed_depth_stencil_getteximage")
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.packed_depth_stencil.packed_depth_stencil_copyteximage")
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.packed_depth_stencil.packed_depth_stencil_partialattachments")
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.packed_depth_stencil.packed_depth_stencil_mixedattachments")
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.packed_depth_stencil.packed_depth_stencil_parameters")
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.packed_depth_stencil.packed_depth_stencil_renderbuffers")
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.packed_depth_stencil.packed_depth_stencil_stencilsize")
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.packed_depth_stencil.packed_depth_stencil_clearbuffer")
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.packed_depth_stencil.packed_depth_stencil_blit")

    def test_GL_OES_standard_derivatives(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es2', 'GL_OES_standard_derivatives')
        self._oglconform.run_case("es2", "glsl-versions_es", "4.28")

    def test_EGL_KHR_gl_renderbuffer_image_2(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'EGL_KHR_gl_renderbuffer_image')
        self._oglconform.check_extension('es2', 'EGL_KHR_gl_renderbuffer_image')
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image.egl_image")

    def test_GL_OES_EGL_image_external(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es2', 'GL_OES_EGL_image_external')
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image_external.TestGetBinding")
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image_external.TestImageUnits")
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image_external.TestImage2D")
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image_external.TestFilterMode")
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image_external.TestMipmap")
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image_external.TestShaderExtension")
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image_external.TestGetActiveUniform")
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image_external.TestTargetTextureValid")
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image_external.TestTargetTextureInvalid")
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image_external.TestTargetTextureSupported")
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image_external.TestSimpleUnassociated")
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image_external.TestSimple")
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image_external.TestVertexShaderBad")
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image_external.TestVertexShader")

    def test_GL_OES_fixed_point(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_fixed_point')
        self._oglconform.run_case("es1", "draw_texture_es1")

    def test_GL_OES_depth24(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_depth24')
        self._oglconform.check_extension('es2', 'GL_OES_depth24')
        self._oglconform.run_case("es1", "fbo_es1", "7.1")
        self._oglconform.run_case("es1", "fbo_es1", "7.5")
        self._oglconform.run_case("es2", "fbo_es", "7.1")
        self._oglconform.run_case("es2", "fbo_es", "7.5")
        self._oglconform.run_case("es2", "buffer_values_es", "1.1.3")
        self._oglconform.run_case("es2", "req_internalformat_es", "1.1.7")
        self._oglconform.run_case("es2", "req_internalformat_es", "1.3.5")
        self._oglconform.run_case("es2", "req_internalformat_es", "3.1.2")
        self._oglconform.run_case("es2", "req_internalformat_es", "3.1.8")

    def test_GL_OES_EGL_image(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_EGL_image')
        self._oglconform.check_extension('es2', 'GL_OES_EGL_image')
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image.egl_image")

    def test_GL_EXT_texture_format_BGRA8888(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_EXT_texture_format_BGRA8888')
        self._oglconform.check_extension('es2', 'GL_EXT_texture_format_BGRA8888')
        self._oglconform.run_case("es2", "tex-bgra_es")

    def test_GL_OES_texture_npot(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_texture_npot')
        self._oglconform.check_extension('es2', 'GL_OES_texture_npot')
        self._oglconform.run_case("es2", "tex-npot_es")

    def test_GL_OES_compressed_ETC1_RGB8_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_compressed_ETC1_RGB8_texture')
        self._oglconform.check_extension('es2', 'GL_OES_compressed_ETC1_RGB8_texture')
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.compressed_etc1_rgb8_texture.compressed_etc1_rgb8_texture")

    def test_GL_OES_byte_coordinates(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_byte_coordinates')
        self._oglconform.run_case("es1", "byte_coords_es1")

    def test_GL_OES_matrix_get(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_matrix_get')
        self._conform.run_case("mget.c")

    def test_GL_OES_compressed_paletted_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_compressed_paletted_texture')
        self._conform.run_case("texpalet.c")

    def test_GL_OES_point_sprite(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_point_sprite')
        self._conform.run_case("pntsprt.c")

    def test_GL_OES_point_size_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_point_size_array')
        self._conform.run_case("pntszary.c")

    def test_GL_OES_draw_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_draw_texture')
        self._conform.run_case("mustpass.c")
        self._conform.run_case("texdecal.c")
        self._conform.run_case("texedge.c")
        self._conform.run_case("multitex.c")
        self._conform.run_case("texpalet.c")

    def test_GL_OES_texture_env_crossbar(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_texture_env_crossbar')
        self._oglconform.run_case("es1", "texenv-crossbar_es1")

    def test_GL_OES_texture_mirrored_repeat(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_texture_mirrored_repeat')
        self._oglconform.run_case("es1", "tex-mirrored_es")

    def test_GL_OES_texture_cube_map(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_texture_cube_map')
        self._oglconform.run_case("es1", "tex-cubemap_es1")

    def test_GL_OES_blend_func_separate(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_blend_func_separate')
        self._oglconform.run_case("es1", "blend-funcseparate_es1")

    def test_GL_OES_blend_equation_separate(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_blend_equation_separate')
        self._oglconform.run_case("es1", "blend-eqseparate_es1")

    def test_GL_OES_stencil_wrap(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_stencil_wrap')
        self._oglconform.run_case("es1", "spop-wrap_es")

    def test_GL_OES_framebuffer_object(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_framebuffer_object')
        self._oglconform.run_case("es1", "fbo_es1")

    def test_GL_OES_stencil8(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_stencil8')
        # TODO: this test case need further confirm
        # self._oglconform.run_case("es1", "req_internalformat_es")

    def test_GL_EXT_multi_draw_arrays(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_EXT_multi_draw_arrays')
        self._oglconform.check_extension('es2', 'GL_EXT_multi_draw_arrays')
        self._oglconform.run_case("es1", "multi_draw_arrays_es1")
        self._oglconform.run_case("es2", "multi_draw_arrays_es")

    def test_GL_OES_single_precision(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_single_precision')
        self._oglconform.run_case("es1", "single_precision_es1")

    def test_GL_OES_read_format(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'GL_OES_read_format')
        self._conform.run_case("readfmt.c")

    def test_texture_compression_astc_hdr(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es3', 'GL_KHR_texture_compression_astc_hdr')

    def test_texture_compression_astc_ldr(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es3', 'GL_KHR_texture_compression_astc_ldr')

    def test_texture_compression_astc(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es3', 'GL_OES_texture_compression_astc')

    def test_EGL_ANDROID_image_native_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension('es1', 'EGL_ANDROID_image_native_buffer')
        self._oglconform.check_extension('es2', 'EGL_ANDROID_image_native_buffer')
#         self._oglconform.run_case("es2", "egl-android", "2.1")
#         self._oglconform.run_case("es2", "egl-android", "2.2")
#         self._oglconform.run_case("es2", "egl-android", "2.3.1")
#         self._oglconform.run_case("es2", "egl-android", "2.3.2")
#         self._oglconform.run_case("es2", "egl-image", "1.2.1")
#         self._oglconform.run_case("es2", "egl-image", "1.2.2")
#         self._oglconform.run_case("es2", "egl-image", "1.2.3")
#         self._oglconform.run_case("es2", "egl-image", "2.3.1")
#         self._oglconform.run_case("es2", "egl-image", "2.3.2")
#         self._oglconform.run_case("es2", "egl-image", "2.3.3")
#         self._oglconform.run_case("es2", "egl-image", "2.3.4")

    def test_EGL_1_4_egl_android(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.run_case_libGL("es2", "egl-android")

    def test_EGL_1_4_egl_android_fb(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.run_case_libGL("es2", "egl-android-fb")

    def test_EGL_1_4_egl_basic(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.run_case_libGL("es2", "egl-basic")

#     def test_EGL_1_4_egl_config(self):
#         print "[RunTest]: %s" % self.__str__()
#         self._oglconform.run_case_libGL("es2", "egl-config")

    def test_EGL_1_4_egl_config(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.run_case_libGL("es2", "egl-config")

    def test_EGL_1_4_egl_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.run_case_libGL("es2", "egl-context")

    def test_EGL_1_4_egl_gpa_es1(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.run_case("es1", "egl-gpa_es1")

    def test_EGL_1_4_egl_gpa_es2(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.run_case("es2", "egl-gpa_es2")

    def test_EGL_1_4_egl_image(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.run_case_libGL("es2", "egl-image")

    def test_EGL_1_4_egl_initialization(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.run_case_libGL("es2", "egl-initialization")

    def test_EGL_1_4_egl_reusable_sync(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.run_case_libGL("es2", "egl-reusable-sync")

    def test_EGL_1_4_egl_surface(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.run_case_libGL("es2", "egl-surface")

    def test_EGL_1_4_egl_sync(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.run_case_libGL("es2", "egl-sync")

    def test_EGL_1_4_egl_android_sync(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.run_case_libGL("es2", "egl-android-sync")

    def test_GLES11_mustpass_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("mustpass.c")

    def test_GLES11_divzero_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("divzero.c")

    def test_GLES11_vpclamp_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("vpclamp.c")

    def test_GLES11_mquery_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("mquery.c")

    def test_GLES11_xformmix_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("xformmix.c")

    def test_GLES11_vorder_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("vorder.c")

    def test_GLES11_xform_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("xform.c")

    def test_GLES11_xformvp_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("xformvp.c")

    def test_GLES11_xformw_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("xformw.c")

    def test_GLES11_bclear_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("bclear.c")

    def test_GLES11_bcorner_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("bcorner.c")

    def test_GLES11_bcolor_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("bcolor.c")

    def test_GLES11_clip_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("clip.c")

    def test_GLES11_colramp_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("colramp.c")

    def test_GLES11_mask_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("mask.c")

    def test_GLES11_scissor_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("scissor.c")

    def test_GLES11_apfunc_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("apfunc.c")

    def test_GLES11_spclear_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("spclear.c")

    def test_GLES11_spcorner_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("spcorner.c")

    def test_GLES11_spop_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("spop.c")

    def test_GLES11_spfunc_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("spfunc.c")

    def test_GLES11_zbclear_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("zbclear.c")

    def test_GLES11_zbfunc_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("zbfunc.c")

    def test_GLES11_dither_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("dither.c")

    def test_GLES11_logicop_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("logicop.c")

    def test_GLES11_pntaa_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("pntaa.c")

    def test_GLES11_pntrast_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("pntrast.c")

    def test_GLES11_linehv_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("linehv.c")

    def test_GLES11_linerast_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("linerast.c")

    def test_GLES11_trirast_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("trirast.c")

    def test_GLES11_tritile_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("tritile.c")

    def test_GLES11_polycull_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("polycull.c")

    def test_GLES11_l_al_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_al.c")

    def test_GLES11_l_am_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_am.c")

    def test_GLES11_l_as_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_as.c")

    def test_GLES11_l_ac_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_ac.c")

    def test_GLES11_l_ap_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_ap.c")

    def test_GLES11_l_dl_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_dl.c")

    def test_GLES11_l_dm_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_dm.c")

    def test_GLES11_l_dmn_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_dmn.c")

    def test_GLES11_l_dmp_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_dmp.c")

    def test_GLES11_l_em_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_em.c")

    def test_GLES11_l_se_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_se.c")

    def test_GLES11_l_sen_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_sen.c")

    def test_GLES11_l_sl_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_sl.c")

    def test_GLES11_l_sm_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_sm.c")

    def test_GLES11_l_sn_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_sn.c")

    def test_GLES11_l_sp_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_sp.c")

    def test_GLES11_blend_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("blend.c")

    def test_GLES11_foglin_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("foglin.c")

    def test_GLES11_texdecal_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("texdecal.c")

    def test_GLES11_texedge_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("texedge.c")

    def test_GLES11_texpalet_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("texpalet.c")

    def test_GLES11_mipsel_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("mipsel.c")

    def test_GLES11_miplin_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("miplin.c")

    def test_GLES11_packpix_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("packpix.c")

    def test_GLES11_multitex_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("multitex.c")

    def test_GLES11_copytex_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("copytex.c")

    def test_GLES11_readfmt_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("readfmt.c")

    def test_GLES11_mstack_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("mstack.c")

    def test_GLES11_xformn_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("xformn.c")

    def test_GLES11_fogexp_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("fogexp.c")

    def test_GLES11_rescalen_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("rescalen.c")

    def test_GLES11_l_sed_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_sed.c")

    def test_GLES11_l_sep_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("l_sep.c")

    def test_GLES11_gets_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("gets.c")

    def test_GLES11_texcombine_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("texcombine.c")

    def test_GLES11_userclip_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("userclip.c")

    def test_GLES11_bufobj_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("bufobj.c")

    def test_GLES11_mget_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("mget.c")

    def test_GLES11_pntszary_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("pntszary.c")

    def test_GLES11_pntsprt_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("pntsprt.c")

    def test_GLES11_mipgen_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("mipgen.c")

    def test_GLES11_mpalette_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("mpalette.c")

    def test_GLES11_drawtex_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("drawtex.c")

    def test_GLES11_miplevels_c(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_case("miplevels.c")

    def test_GLES11_covegl(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_covegl()

    def test_GLES11_covgl(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_covgl()

    def test_GLES11_primtest(self):
        print "[RunTest]: %s" % self.__str__()
        self._conform.run_primtest()

    def test_ES2_CTS_info(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.info*")

    def test_ES2_CTS_gtf_GL_abs(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.abs*")

    def test_ES2_CTS_gtf_GL_acos(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.acos*")

    def test_ES2_CTS_gtf_GL_all(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.all*")

    def test_ES2_CTS_gtf_GL_any(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.any*")

    def test_ES2_CTS_gtf_GL_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.array*")

    def test_ES2_CTS_gtf_GL_asin(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.asin*")

    def test_ES2_CTS_gtf_GL_atan(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.atan*")

    def test_ES2_CTS_gtf_GL_biConstants(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.biConstants*")

    def test_ES2_CTS_gtf_GL_biuDepthRange(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.biuDepthRange*")

    def test_ES2_CTS_gtf_GL_build(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.build*")

    def test_ES2_CTS_gtf_GL_built_in_varying_array_out_of_bounds(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.built_in_varying_array_out_of_bounds*")

    def test_ES2_CTS_gtf_GL_ceil(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.ceil*")

    def test_ES2_CTS_gtf_GL_clamp(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.clamp*")

    def test_ES2_CTS_gtf_GL_control_flow(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.control_flow*")

    def test_ES2_CTS_gtf_GL_cos(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.cos*")

    def test_ES2_CTS_gtf_GL_cross(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.cross*")

    def test_ES2_CTS_gtf_GL_default(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.default*")

    def test_ES2_CTS_gtf_GL_degrees(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.degrees*")

    def test_ES2_CTS_gtf_GL_discard(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.discard*")

    def test_ES2_CTS_gtf_GL_distance(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.distance*")

    def test_ES2_CTS_gtf_GL_dot(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.dot*")

    def test_ES2_CTS_gtf_GL_equal(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.equal*")

    def test_ES2_CTS_gtf_GL_exp(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.exp*")

    def test_ES2_CTS_gtf_GL_exp2(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.exp2*")

    def test_ES2_CTS_gtf_GL_faceforward(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.faceforward*")

    def test_ES2_CTS_gtf_GL_floor(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.floor*")

    def test_ES2_CTS_gtf_GL_fract(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.fract*")

    def test_ES2_CTS_gtf_GL_functions(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.functions*")

    def test_ES2_CTS_gtf_GL_glGetShaderSource(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.glGetShaderSource*")

    def test_ES2_CTS_gtf_GL_gl_FragCoord(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.gl_FragCoord*")

    def test_ES2_CTS_gtf_GL_gl_FrontFacing(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.gl_FrontFacing*")

    def test_ES2_CTS_gtf_GL_greaterThan(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.greaterThan*")

    def test_ES2_CTS_gtf_GL_greaterThanEqual(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.greaterThanEqual*")

    def test_ES2_CTS_gtf_GL_inversesqrt(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.inversesqrt*")

    def test_ES2_CTS_gtf_GL_length(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.length*")

    def test_ES2_CTS_gtf_GL_lessThan(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.lessThan*")

    def test_ES2_CTS_gtf_GL_lessThanEqual(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.lessThanEqual*")

    def test_ES2_CTS_gtf_GL_log(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.log*")

    def test_ES2_CTS_gtf_GL_log2(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.log2*")

    def test_ES2_CTS_gtf_GL_mat(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.mat*")

    def test_ES2_CTS_gtf_GL_mat3(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.mat3*")

    def test_ES2_CTS_gtf_GL_matrixCompMult(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.matrixCompMult*")

    def test_ES2_CTS_gtf_GL_max(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.max*")

    def test_ES2_CTS_gtf_GL_min(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.min*")

    def test_ES2_CTS_gtf_GL_mix(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.mix*")

    def test_ES2_CTS_gtf_GL_mod(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.mod*")

    def test_ES2_CTS_gtf_GL_normalize(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.normalize*")

    def test_ES2_CTS_gtf_GL_not(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.not*")

    def test_ES2_CTS_gtf_GL_notEqual(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.notEqual*")

    def test_ES2_CTS_gtf_GL_operators(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.operators*")

    def test_ES2_CTS_gtf_GL_pow(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.pow*")

    def test_ES2_CTS_gtf_GL_radians(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.radians*")

    def test_ES2_CTS_gtf_GL_read_format(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.read_format*")

    def test_ES2_CTS_gtf_GL_reflect(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.reflect*")

    def test_ES2_CTS_gtf_GL_refract(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.refract*")

    def test_ES2_CTS_gtf_GL_sign(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.sign*")

    def test_ES2_CTS_gtf_GL_sin(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.sin*")

    def test_ES2_CTS_gtf_GL_smoothstep(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.smoothstep*")

    def test_ES2_CTS_gtf_GL_sqrt(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.sqrt*")

    def test_ES2_CTS_gtf_GL_stencil8(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.stencil8*")

    def test_ES2_CTS_gtf_GL_step(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.step*")

    def test_ES2_CTS_gtf_GL_struct(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.struct*")

    def test_ES2_CTS_gtf_GL_swizzlers(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.swizzlers*")

    def test_ES2_CTS_gtf_GL_tan(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.tan*")

    def test_ES2_CTS_gtf_GL_vec(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.vec*")

    def test_ES2_CTS_gtf_GL_vec3(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL.vec3*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_compressed_astc_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.compressed_astc_texture*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_compressed_astc_texture_full(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.compressed_astc_texture_full*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_compressed_astc_texture_hdr(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.compressed_astc_texture_hdr*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_compressed_etc1_rgb8_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.compressed_etc1_rgb8_texture*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_compressed_paletted_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.compressed_paletted_texture*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_data_type_10_10_10_2(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.data_type_10_10_10_2*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_debug(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.debug*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_depth_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.depth_texture*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_depth_texture_cube_map(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.depth_texture_cube_map*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_dFdx(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.dFdx*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_dFdy(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.dFdy*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_egl_create_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_create_context*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_egl_image(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_egl_image_external(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.egl_image_external*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_fwidth(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.fwidth*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_packed_depth_stencil(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.packed_depth_stencil*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_stencil1(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.stencil1*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_stencil4(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.stencil4*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_texture_float(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.texture_float*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_vertex_array_object(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.vertex_array_object*")

    def test_ES2_CTS_gtf_GL2ExtensionTests_vertex_half_float(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2ExtensionTests.vertex_half_float*")

    def test_ES2_CTS_gtf_GL2FixedTests_blend(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.blend*")

    def test_ES2_CTS_gtf_GL2FixedTests_buffer_clear(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.buffer_clear*")

    def test_ES2_CTS_gtf_GL2FixedTests_buffer_color(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.buffer_color*")

    def test_ES2_CTS_gtf_GL2FixedTests_buffer_corners(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.buffer_corners*")

    def test_ES2_CTS_gtf_GL2FixedTests_buffer_objects(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.buffer_objects*")

    def test_ES2_CTS_gtf_GL2FixedTests_clip(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.clip*")

    def test_ES2_CTS_gtf_GL2FixedTests_color_ramp(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.color_ramp*")

    def test_ES2_CTS_gtf_GL2FixedTests_copy_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.copy_texture*")

    def test_ES2_CTS_gtf_GL2FixedTests_depth_buffer_clear(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.depth_buffer_clear*")

    def test_ES2_CTS_gtf_GL2FixedTests_depth_buffer_functions(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.depth_buffer_functions*")

    def test_ES2_CTS_gtf_GL2FixedTests_dither(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.dither*")

    def test_ES2_CTS_gtf_GL2FixedTests_divide_by_zero(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.divide_by_zero*")

    def test_ES2_CTS_gtf_GL2FixedTests_gets(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.gets*")

    def test_ES2_CTS_gtf_GL2FixedTests_lighting_diffuse(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.lighting_diffuse*")

    def test_ES2_CTS_gtf_GL2FixedTests_mipmaps_interpolation(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.mipmaps_interpolation*")

    def test_ES2_CTS_gtf_GL2FixedTests_mipmaps_selection(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.mipmaps_selection*")

    def test_ES2_CTS_gtf_GL2FixedTests_point_rasterization(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.point_rasterization*")

    def test_ES2_CTS_gtf_GL2FixedTests_point_sprites(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.point_sprites*")

    def test_ES2_CTS_gtf_GL2FixedTests_polygon_cull(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.polygon_cull*")

    def test_ES2_CTS_gtf_GL2FixedTests_scissor(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.scissor*")

    def test_ES2_CTS_gtf_GL2FixedTests_stencil_plane_clear(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.stencil_plane_clear*")

    def test_ES2_CTS_gtf_GL2FixedTests_stencil_plane_corners(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.stencil_plane_corners*")

    def test_ES2_CTS_gtf_GL2FixedTests_stencil_plane_function(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.stencil_plane_function*")

    def test_ES2_CTS_gtf_GL2FixedTests_stencil_plane_operation(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.stencil_plane_operation*")

    def test_ES2_CTS_gtf_GL2FixedTests_texture_edge_clamp(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.texture_edge_clamp*")

    def test_ES2_CTS_gtf_GL2FixedTests_transform_viewport(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.transform_viewport*")

    def test_ES2_CTS_gtf_GL2FixedTests_triangle_rasterization(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.triangle_rasterization*")

    def test_ES2_CTS_gtf_GL2FixedTests_triangle_tiling(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.triangle_tiling*")

    def test_ES2_CTS_gtf_GL2FixedTests_user_clip_planes(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.user_clip_planes*")

    def test_ES2_CTS_gtf_GL2FixedTests_vertex_order(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.vertex_order*")

    def test_ES2_CTS_gtf_GL2FixedTests_viewport_clamp(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2FixedTests.viewport_clamp*")

    def test_ES2_CTS_gtf_GL2Tests_attach_shader(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.attach_shader*")

    def test_ES2_CTS_gtf_GL2Tests_bind_attribute_location(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.bind_attribute_location*")

    def test_ES2_CTS_gtf_GL2Tests_compile_shader(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.compile_shader*")

    def test_ES2_CTS_gtf_GL2Tests_create_objects_shaders_programs(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.create_objects_shaders_programs*")

    def test_ES2_CTS_gtf_GL2Tests_delete_object(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.delete_object*")

    def test_ES2_CTS_gtf_GL2Tests_detach_shader(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.detach_shader*")

    def test_ES2_CTS_gtf_GL2Tests_fixed_data_type(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.fixed_data_type*")

    def test_ES2_CTS_gtf_GL2Tests_framebuffer_objects(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.framebuffer_objects*")

    def test_ES2_CTS_gtf_GL2Tests_GetBIFD(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.GetBIFD*")

    def test_ES2_CTS_gtf_GL2Tests_get_active_attribute(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.get_active_attribute*")

    def test_ES2_CTS_gtf_GL2Tests_get_active_uniform(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.get_active_uniform*")

    def test_ES2_CTS_gtf_GL2Tests_get_attached_objects(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.get_attached_objects*")

    def test_ES2_CTS_gtf_GL2Tests_get_attribute_location(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.get_attribute_location*")

    def test_ES2_CTS_gtf_GL2Tests_get_extensions(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.get_extensions*")

    def test_ES2_CTS_gtf_GL2Tests_get_handle(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.get_handle*")

    def test_ES2_CTS_gtf_GL2Tests_get_uniform_location(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.get_uniform_location*")

    def test_ES2_CTS_gtf_GL2Tests_glGetProgramInfoLog_2_0(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.glGetProgramInfoLog_2_0*")

    def test_ES2_CTS_gtf_GL2Tests_glGetProgramiv_2_0(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.glGetProgramiv_2_0*")

    def test_ES2_CTS_gtf_GL2Tests_glGetShaderInfoLog_2_0(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.glGetShaderInfoLog_2_0*")

    def test_ES2_CTS_gtf_GL2Tests_glGetUniform(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.glGetUniform*")

    def test_ES2_CTS_gtf_GL2Tests_glGetVertexAttrib(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.glGetVertexAttrib*")

    def test_ES2_CTS_gtf_GL2Tests_glUniform(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.glUniform*")

    def test_ES2_CTS_gtf_GL2Tests_link_program(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.link_program*")

    def test_ES2_CTS_gtf_GL2Tests_precision_specifiers(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.precision_specifiers*")

    def test_ES2_CTS_gtf_GL2Tests_relink_program(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.relink_program*")

    def test_ES2_CTS_gtf_GL2Tests_shader_source(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.shader_source*")

    def test_ES2_CTS_gtf_GL2Tests_three_uniforms(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.three_uniforms*")

    def test_ES2_CTS_gtf_GL2Tests_use_program(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.use_program*")

    def test_ES2_CTS_gtf_GL2Tests_validate_program(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.validate_program*")

    def test_ES2_CTS_gtf_GL2Tests_vertex_program_point_size(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GL2Tests.vertex_program_point_size*")

    def test_ES2_CTS_gtf_GLCoverage(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES2-CTS.gtf.GLCoverage*")

    def test_ES3_CTS_info(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.info*")

    def test_ES3_CTS_shaders_arrays_constructor(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.arrays.constructor*")

    def test_ES3_CTS_shaders_arrays_return(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.arrays.return*")

    def test_ES3_CTS_shaders_arrays_unnamed_parameter(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.arrays.unnamed_parameter*")

    def test_ES3_CTS_shaders_arrays_declaration(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.arrays.declaration*")

    def test_ES3_CTS_shaders_arrays_length(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.arrays.length*")

    def test_ES3_CTS_shaders_arrays_invalid(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.arrays.invalid*")

    def test_ES3_CTS_shaders_fragdepth_write(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.fragdepth.write*")

    def test_ES3_CTS_shaders_fragdepth_compare(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.fragdepth.compare*")

    def test_ES3_CTS_shaders_indexing_varying_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.indexing.varying_array*")

    def test_ES3_CTS_shaders_indexing_uniform_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.indexing.uniform_array*")

    def test_ES3_CTS_shaders_indexing_tmp_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.indexing.tmp_array*")

    def test_ES3_CTS_shaders_indexing_vector_subscript(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.indexing.vector_subscript*")

    def test_ES3_CTS_shaders_indexing_matrix_subscript(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.indexing.matrix_subscript*")

    def test_ES3_CTS_shaders_loops_for_constant_iterations(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.loops.for_constant_iterations*")

    def test_ES3_CTS_shaders_loops_for_uniform_iterations(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.loops.for_uniform_iterations*")

    def test_ES3_CTS_shaders_loops_for_dynamic_iterations(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.loops.for_dynamic_iterations*")

    def test_ES3_CTS_shaders_loops_while_constant_iterations(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.loops.while_constant_iterations*")

    def test_ES3_CTS_shaders_loops_while_uniform_iterations(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.loops.while_uniform_iterations*")

    def test_ES3_CTS_shaders_loops_while_dynamic_iterations(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.loops.while_dynamic_iterations*")

    def test_ES3_CTS_shaders_loops_do_while_constant_iterations(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.loops.do_while_constant_iterations*")

    def test_ES3_CTS_shaders_loops_do_while_uniform_iterations(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.loops.do_while_uniform_iterations*")

    def test_ES3_CTS_shaders_loops_do_while_dynamic_iterations(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.loops.do_while_dynamic_iterations*")

    def test_ES3_CTS_shaders_preprocessor_basic(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.basic*")

    def test_ES3_CTS_shaders_preprocessor_definitions(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.definitions*")

    def test_ES3_CTS_shaders_preprocessor_invalid_definitions(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.invalid_definitions*")

    def test_ES3_CTS_shaders_preprocessor_object_redefinitions(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.object_redefinitions*")

    def test_ES3_CTS_shaders_preprocessor_invalid_redefinitions(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.invalid_redefinitions*")

    def test_ES3_CTS_shaders_preprocessor_comments(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.comments*")

    def test_ES3_CTS_shaders_preprocessor_line_continuation(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.line_continuation*")

    def test_ES3_CTS_shaders_preprocessor_function_definitions(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.function_definitions*")

    def test_ES3_CTS_shaders_preprocessor_recursion(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.recursion*")

    def test_ES3_CTS_shaders_preprocessor_function_redefinitions(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.function_redefinitions*")

    def test_ES3_CTS_shaders_preprocessor_invalid_function_definitions(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.invalid_function_definitions*")

    def test_ES3_CTS_shaders_preprocessor_semantic(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.semantic*")

    def test_ES3_CTS_shaders_preprocessor_predefined_macros(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.predefined_macros*")

    def test_ES3_CTS_shaders_preprocessor_conditional_inclusion(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.conditional_inclusion*")

    def test_ES3_CTS_shaders_preprocessor_invalid_ops(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.invalid_ops*")

    def test_ES3_CTS_shaders_preprocessor_undefined_identifiers(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.undefined_identifiers*")

    def test_ES3_CTS_shaders_preprocessor_invalid_conditionals(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.invalid_conditionals*")

    def test_ES3_CTS_shaders_preprocessor_conditionals(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.conditionals*")

    def test_ES3_CTS_shaders_preprocessor_directive(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.directive*")

    def test_ES3_CTS_shaders_preprocessor_builtin(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.builtin*")

    def test_ES3_CTS_shaders_preprocessor_pragmas(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.pragmas*")

    def test_ES3_CTS_shaders_preprocessor_extensions(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.extensions*")

    def test_ES3_CTS_shaders_preprocessor_expressions(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.expressions*")

    def test_ES3_CTS_shaders_preprocessor_invalid_expressions(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.invalid_expressions*")

    def test_ES3_CTS_shaders_preprocessor_operator_precedence(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.preprocessor.operator_precedence*")

    def test_ES3_CTS_shaders_struct_local(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.struct.local*")

    def test_ES3_CTS_shaders_struct_uniform(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.struct.uniform*")

    def test_ES3_CTS_shaders_switch(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.switch*")

    def test_ES3_CTS_shaders_uniform_block_single_basic_type(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.single_basic_type*")

    def test_ES3_CTS_shaders_uniform_block_single_basic_type_shared(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.single_basic_type.shared*")

    def test_ES3_CTS_shaders_uniform_block_single_basic_type_packed(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.single_basic_type.packed*")

    def test_ES3_CTS_shaders_uniform_block_single_basic_type_std140(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.single_basic_type.std140*")

    def test_ES3_CTS_shaders_uniform_block_single_basic_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.single_basic_array*")

    def test_ES3_CTS_shaders_uniform_block_single_basic_array_shared(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.single_basic_array.shared*")

    def test_ES3_CTS_shaders_uniform_block_single_basic_array_packed(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.single_basic_array.packed*")

    def test_ES3_CTS_shaders_uniform_block_single_basic_array_std140(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.single_basic_array.std140*")

    def test_ES3_CTS_shaders_uniform_block_single_struct(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.single_struct*")

    def test_ES3_CTS_shaders_uniform_block_single_struct_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.single_struct_array*")

    def test_ES3_CTS_shaders_uniform_block_single_nested_struct(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.single_nested_struct*")

    def test_ES3_CTS_shaders_uniform_block_single_nested_struct_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.single_nested_struct_array*")

    def test_ES3_CTS_shaders_uniform_block_instance_array_basic_type_shared(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.instance_array_basic_type.shared*")

    def test_ES3_CTS_shaders_uniform_block_instance_array_basic_type_packed(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.instance_array_basic_type.packed*")

    def test_ES3_CTS_shaders_uniform_block_instance_array_basic_type_std140(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.instance_array_basic_type.std140*")

    def test_ES3_CTS_shaders_uniform_block_multi_basic_types_per_block_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.multi_basic_types.per_block_buffer*")

    def test_ES3_CTS_shaders_uniform_block_multi_basic_types_single_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.multi_basic_types.single_buffer*")

    def test_ES3_CTS_shaders_uniform_block_multi_nested_struct_per_block_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.multi_nested_struct.per_block_buffer*")

    def test_ES3_CTS_shaders_uniform_block_multi_nested_struct_single_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.multi_nested_struct.single_buffer*")

    def test_ES3_CTS_shaders_uniform_block_random_scalar_types(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.random.scalar_types*")

    def test_ES3_CTS_shaders_uniform_block_random_vector_types(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.random.vector_types*")

    def test_ES3_CTS_shaders_uniform_block_random_basic_types(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.random.basic_types*")

    def test_ES3_CTS_shaders_uniform_block_random_basic_arrays(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.random.basic_arrays*")

    def test_ES3_CTS_shaders_uniform_block_random_basic_instance_arrays(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.random.basic_instance_arrays*")

    def test_ES3_CTS_shaders_uniform_block_random_nested_structs(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.random.nested_structs*")

    def test_ES3_CTS_shaders_uniform_block_random_nested_structs_arrays(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.random.nested_structs_arrays*")

    def test_ES3_CTS_shaders_uniform_block_random_nested_structs_instance_arrays(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.random.nested_structs_instance_arrays*")

    def test_ES3_CTS_shaders_uniform_block_random_nested_structs_arrays_instance_arrays(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.random.nested_structs_arrays_instance_arrays*")

    def test_ES3_CTS_shaders_uniform_block_random_all_per_block_buffers(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.random.all_per_block_buffers*")

    def test_ES3_CTS_shaders_uniform_block_random_all_shared_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.uniform_block.random.all_shared_buffer*")

    def test_ES3_CTS_shaders_shader_integer_mix(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.shaders.shader_integer_mix*")

    def test_ES3_CTS_gtf_GL_build(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL.build*")

    def test_ES3_CTS_gtf_GL3Tests_blend_minmax(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.blend_minmax*")

    def test_ES3_CTS_gtf_GL3Tests_color_buffer_float(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.color_buffer_float*")

    def test_ES3_CTS_gtf_GL3Tests_copy_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.copy_buffer*")

    def test_ES3_CTS_gtf_GL3Tests_depth24(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.depth24*")

    def test_ES3_CTS_gtf_GL3Tests_depth_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.depth_texture*")

    def test_ES3_CTS_gtf_GL3Tests_draw_buffers(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.draw_buffers*")

    def test_ES3_CTS_gtf_GL3Tests_draw_instanced(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.draw_instanced*")

    def test_ES3_CTS_gtf_GL3Tests_eac_compression_r11(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.eac_compression_r11*")

    def test_ES3_CTS_gtf_GL3Tests_eac_compression_rg11(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.eac_compression_rg11*")

    def test_ES3_CTS_gtf_GL3Tests_eac_compression_signed_r11(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.eac_compression_signed_r11*")

    def test_ES3_CTS_gtf_GL3Tests_eac_compression_signed_rg11(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.eac_compression_signed_rg11*")

    def test_ES3_CTS_gtf_GL3Tests_element_index_uint(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.element_index_uint*")

    def test_ES3_CTS_gtf_GL3Tests_etc2_compression_rgb8(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.etc2_compression_rgb8*")

    def test_ES3_CTS_gtf_GL3Tests_etc2_compression_rgb8_pt_alpha1(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.etc2_compression_rgb8_pt_alpha1*")

    def test_ES3_CTS_gtf_GL3Tests_etc2_compression_rgba8(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.etc2_compression_rgba8*")

    def test_ES3_CTS_gtf_GL3Tests_explicit_attrib_location(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.explicit_attrib_location*")

    def test_ES3_CTS_gtf_GL3Tests_framebuffer_srgb(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.framebuffer_srgb*")

    def test_ES3_CTS_gtf_GL3Tests_half_float(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.half_float*")

    def test_ES3_CTS_gtf_GL3Tests_instanced_arrays(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.instanced_arrays*")

    def test_ES3_CTS_gtf_GL3Tests_map_buffer_range(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.map_buffer_range*")

    def test_ES3_CTS_gtf_GL3Tests_packed_depth_stencil(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.packed_depth_stencil*")

    def test_ES3_CTS_gtf_GL3Tests_packed_pixels(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.packed_pixels*")

    def test_ES3_CTS_gtf_GL3Tests_primitive_restart(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.primitive_restart*")

    def test_ES3_CTS_gtf_GL3Tests_rgb8_rgba8(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.rgb8_rgba8*")

    def test_ES3_CTS_gtf_GL3Tests_sgis_texture_lod(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.sgis_texture_lod*")

    def test_ES3_CTS_gtf_GL3Tests_shadow(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.shadow*")

    def test_ES3_CTS_gtf_GL3Tests_sync(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.sync*")

    def test_ES3_CTS_gtf_GL3Tests_texture_float32(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.texture_float32*")

    def test_ES3_CTS_gtf_GL3Tests_texture_lod_bias(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.texture_lod_bias*")

    def test_ES3_CTS_gtf_GL3Tests_texture_storage(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.texture_storage*")

    def test_ES3_CTS_gtf_GL3Tests_transform_feedback(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.transform_feedback*")

    def test_ES3_CTS_gtf_GL3Tests_transform_feedback2(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.transform_feedback2*")

    def test_ES3_CTS_gtf_GL3Tests_vertex_type_2_10_10_10_rev(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.vertex_type_2_10_10_10_rev*")

    def test_ES3_CTS_gtf_GL3Tests_pixel_buffer_object(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.pixel_buffer_object*")

    def test_ES3_CTS_gtf_GL3Tests_framebuffer_blit(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.framebuffer_blit*")

    def test_ES3_CTS_gtf_GL3Tests_uniform_buffer_object(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.uniform_buffer_object*")

    def test_ES3_CTS_gtf_GL3Tests_occlusion_query(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.occlusion_query*")

    def test_ES3_CTS_gtf_GL3Tests_npot_textures(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.npot_textures*")

    def test_ES3_CTS_gtf_GL3Tests_copy_tex_image_conversions(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GL3Tests.copy_tex_image_conversions*")

    def test_ES3_CTS_gtf_GLCoverage(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES3-CTS.gtf.GLCoverage*")

    def test_ES31_CTS_info(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.info*")

    def test_ES31_CTS_texture_storage_multisample_APIGLGetActiveUniform(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.texture_storage_multisample.APIGLGetActiveUniform*")

    def test_ES31_CTS_texture_storage_multisample_APIGLTexStorage2DMultisample(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.texture_storage_multisample.APIGLTexStorage2DMultisample*")

    def test_ES31_CTS_texture_storage_multisample_APIGLTexStorage3DMultisample(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.texture_storage_multisample.APIGLTexStorage3DMultisample*")

    def test_ES31_CTS_texture_storage_multisample_APIGLGetMultisamplefv(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.texture_storage_multisample.APIGLGetMultisamplefv*")

    def test_ES31_CTS_texture_storage_multisample_APIGLGetTexLevelParameterifv(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.texture_storage_multisample.APIGLGetTexLevelParameterifv*")

    def test_ES31_CTS_texture_storage_multisample_APIGLSampleMaski(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.texture_storage_multisample.APIGLSampleMaski*")

    def test_ES31_CTS_texture_storage_multisample_APIDependencies(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.texture_storage_multisample.APIDependencies*")

    def test_ES31_CTS_texture_storage_multisample_GLCoverage(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.texture_storage_multisample.GLCoverage*")

    def test_ES31_CTS_texture_storage_multisample_FunctionalTests(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.texture_storage_multisample.FunctionalTests*")

    def test_ES31_CTS_shader_atomic_counters(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_atomic_counters*")

    def test_ES31_CTS_texture_gather(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.texture_gather*")

    def test_ES31_CTS_sample_shading_render_rgba8(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.sample_shading.render.rgba8*")

    def test_ES31_CTS_sample_shading_render_rgba8i(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.sample_shading.render.rgba8i*")

    def test_ES31_CTS_sample_shading_render_rgba8ui(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.sample_shading.render.rgba8ui*")

    def test_ES31_CTS_sample_shading_render_rgba32f(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.sample_shading.render.rgba32f*")

    def test_ES31_CTS_sample_variables_mask_rgba8(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.sample_variables.mask.rgba8*")

    def test_ES31_CTS_sample_variables_mask_rgba8i(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.sample_variables.mask.rgba8i*")

    def test_ES31_CTS_sample_variables_mask_rgba8ui(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.sample_variables.mask.rgba8ui*")

    def test_ES31_CTS_sample_variables_mask_rgba32f(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.sample_variables.mask.rgba32f*")

    def test_ES31_CTS_sample_variables_position_non_fixed(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.sample_variables.position.non-fixed*")

    def test_ES31_CTS_sample_variables_position_fixed(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.sample_variables.position.fixed*")

    def test_ES31_CTS_sepshaderobjs(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.sepshaderobjs*")

    def test_ES31_CTS_shader_bitfield_operation_frexp(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_bitfield_operation.frexp*")

    def test_ES31_CTS_shader_bitfield_operation_ldexp(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_bitfield_operation.ldexp*")

    def test_ES31_CTS_shader_bitfield_operation_packUnorm4x8(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_bitfield_operation.packUnorm4x8*")

    def test_ES31_CTS_shader_bitfield_operation_packSnorm4x8(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_bitfield_operation.packSnorm4x8*")

    def test_ES31_CTS_shader_bitfield_operation_unpackUnorm4x8(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_bitfield_operation.unpackUnorm4x8*")

    def test_ES31_CTS_shader_bitfield_operation_unpackSnorm4x8(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_bitfield_operation.unpackSnorm4x8*")

    def test_ES31_CTS_shader_bitfield_operation_bitfieldExtract(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_bitfield_operation.bitfieldExtract*")

    def test_ES31_CTS_shader_bitfield_operation_bitfieldInsert(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_bitfield_operation.bitfieldInsert*")

    def test_ES31_CTS_shader_bitfield_operation_bitfieldReverse(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_bitfield_operation.bitfieldReverse*")

    def test_ES31_CTS_shader_bitfield_operation_bitCount(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_bitfield_operation.bitCount*")

    def test_ES31_CTS_shader_bitfield_operation_findLSB(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_bitfield_operation.findLSB*")

    def test_ES31_CTS_shader_bitfield_operation_findMSB(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_bitfield_operation.findMSB*")

    def test_ES31_CTS_shader_bitfield_operation_uaddCarry(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_bitfield_operation.uaddCarry*")

    def test_ES31_CTS_shader_bitfield_operation_usubBorrow(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_bitfield_operation.usubBorrow*")

    def test_ES31_CTS_shader_bitfield_operation_umulExtended(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_bitfield_operation.umulExtended*")

    def test_ES31_CTS_shader_bitfield_operation_imulExtended(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_bitfield_operation.imulExtended*")

    def test_ES31_CTS_shader_multisample_interpolation_render_base(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_multisample_interpolation.render.base*")

    def test_ES31_CTS_shader_multisample_interpolation_render_sample(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_multisample_interpolation.render.sample*")

    def test_ES31_CTS_shader_multisample_interpolation_render_centroid(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_multisample_interpolation.render.centroid*")

    def test_ES31_CTS_shader_multisample_interpolation_render_interpolate_at_sample(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_multisample_interpolation.render.interpolate_at_sample*")

    def test_ES31_CTS_shader_multisample_interpolation_render_interpolate_at_sample_check(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_multisample_interpolation.render.interpolate_at_sample_check*")

    def test_ES31_CTS_shader_multisample_interpolation_render_interpolate_at_centroid(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_multisample_interpolation.render.interpolate_at_centroid*")

    def test_ES31_CTS_shader_multisample_interpolation_render_interpolate_at_offset(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_multisample_interpolation.render.interpolate_at_offset*")

    def test_ES31_CTS_shader_multisample_interpolation_render_interpolate_at_offset_check(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_multisample_interpolation.render.interpolate_at_offset_check*")

    def test_ES31_CTS_layout_binding(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.layout_binding*")

    def test_ES31_CTS_shader_integer_mix(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_integer_mix*")

    def test_ES31_CTS_blend_equation_advanced_BlendEquationSeparate(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.blend_equation_advanced.BlendEquationSeparate*")

    def test_ES31_CTS_blend_equation_advanced_mismatching_qualifier(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.blend_equation_advanced.mismatching_qualifier*")

    def test_ES31_CTS_blend_equation_advanced_missing_qualifier(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.blend_equation_advanced.missing_qualifier*")

    def test_ES31_CTS_blend_equation_advanced_blend_all(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.blend_equation_advanced.blend_all*")

    def test_ES31_CTS_blend_equation_advanced_blend_specific(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.blend_equation_advanced.blend_specific*")

    def test_ES31_CTS_blend_equation_advanced_test_coherency(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.blend_equation_advanced.test_coherency*")

    def test_ES31_CTS_vertex_attrib_binding(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.vertex_attrib_binding*")

    def test_ES31_CTS_shader_storage_buffer_object(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_storage_buffer_object*")

    def test_ES31_CTS_compute_shader(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.compute_shader*")

    def test_ES31_CTS_shader_image_load_store(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_image_load_store*")

    def test_ES31_CTS_shader_image_size(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.shader_image_size*")

    def test_ES31_CTS_draw_indirect(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.draw_indirect*")

    def test_ES31_CTS_explicit_uniform_location(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.explicit_uniform_location*")

    def test_ES31_CTS_program_interface_query(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.program_interface_query*")

    def test_ES31_CTS_framebuffer_no_attachments(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.framebuffer_no_attachments*")

    def test_ES31_CTS_arrays_of_arrays(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.arrays_of_arrays*")

    def test_ES31_CTS_gtf_GL31Tests_texture_stencil8(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.gtf.GL31Tests.texture_stencil8*")

    def test_ES31_CTS_gtf_GL31Tests_shader_helper(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.gtf.GL31Tests.shader_helper*")

    def test_ES31_CTS_gtf_GL3Tests_packed_depth_stencil(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.gtf.GL3Tests.packed_depth_stencil*")

    def test_ES31_CTS_geometry_shader_adjacency(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.geometry_shader.adjacency*")

    def test_ES31_CTS_geometry_shader_rendering_rendering(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.geometry_shader.rendering.rendering*")

    def test_ES31_CTS_geometry_shader_program_resource(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.geometry_shader.program_resource*")

    def test_ES31_CTS_geometry_shader_nonarray_input(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.geometry_shader.nonarray_input*")

    def test_ES31_CTS_geometry_shader_input(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.geometry_shader.input*")

    def test_ES31_CTS_geometry_shader_primitive_counter(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.geometry_shader.primitive_counter*")

    def test_ES31_CTS_geometry_shader_layered_rendering(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.geometry_shader.layered_rendering*")

    def test_ES31_CTS_geometry_shader_clipping(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.geometry_shader.clipping*")

    def test_ES31_CTS_geometry_shader_blitting(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.geometry_shader.blitting*")

    def test_ES31_CTS_geometry_shader_layered_rendering_boundary_condition(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.geometry_shader.layered_rendering_boundary_condition*")

    def test_ES31_CTS_geometry_shader_layered_framebuffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.geometry_shader.layered_framebuffer*")

    def test_ES31_CTS_geometry_shader_output(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.geometry_shader.output*")

    def test_ES31_CTS_geometry_shader_primitive_queries(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.geometry_shader.primitive_queries*")

    def test_ES31_CTS_geometry_shader_layered_rendering_fbo_no_attachment(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.geometry_shader.layered_rendering_fbo_no_attachment*")

    def test_ES31_CTS_geometry_shader_constant_variables(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.geometry_shader.constant_variables*")

    def test_ES31_CTS_geometry_shader_limits(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.geometry_shader.limits*")

    def test_ES31_CTS_gpu_shader5(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.gpu_shader5*")

    def test_ES31_CTS_tessellation_shader_tessellation_shader_tc_barriers(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.tessellation_shader.tessellation_shader_tc_barriers*")

    def test_ES31_CTS_tessellation_shader_compilation_and_linking_errors(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.tessellation_shader.compilation_and_linking_errors*")

    def test_ES31_CTS_tessellation_shader_tessellation_invariance(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.tessellation_shader.tessellation_invariance*")

    def test_ES31_CTS_tessellation_shader_tessellation_shader_point_mode(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.tessellation_shader.tessellation_shader_point_mode*")

    def test_ES31_CTS_tessellation_shader_tessellation_shader_quads_tessellation(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.tessellation_shader.tessellation_shader_quads_tessellation*")

    def test_ES31_CTS_tessellation_shader_tessellation_control_to_tessellation_evaluation(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.tessellation_shader.tessellation_control_to_tessellation_evaluation*")

    def test_ES31_CTS_tessellation_shader_tessellation_shader_tessellation_gl_TessCoord(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.tessellation_shader.tessellation_shader_tessellation.gl_TessCoord*")

    def test_ES31_CTS_tessellation_shader_tessellation_shader_triangles_tessellation(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.tessellation_shader.tessellation_shader_triangles_tessellation*")

    def test_ES31_CTS_texture_cube_map_array_image_texture_size(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.texture_cube_map_array.image_texture_size*")

    def test_ES31_CTS_texture_border_clamp_sampling_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.texture_border_clamp.sampling_texture*")

    def test_ES31_CTS_texture_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES31-CTS.texture_buffer*")

    def test_ESEXT_CTS_texture_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ESEXT-CTS.texture_buffer*")

    def test_ES32_CTS_blend_equation_advanced_blend_all(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.blend_equation_advanced.blend_all*")

    def test_ES32_CTS_blend_equation_advanced_BlendEquationSeparate(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.blend_equation_advanced.BlendEquationSeparate*")

    def test_ES32_CTS_texture_cube_map_array_tex3D_validation(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.tex3D_validation*")

    def test_ES32_CTS_blend_equation_advanced_blend_specific(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.blend_equation_advanced.blend_specific*")

    def test_ES32_CTS_blend_equation_advanced_coherentEnableDisable(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.blend_equation_advanced.coherentEnableDisable*")

    def test_ES32_CTS_blend_equation_advanced_extension_directive_enable(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.blend_equation_advanced.extension_directive_enable*")

    def test_ES32_CTS_texture_cube_map_array_subimage3D(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.subimage3D*")

    def test_ES32_CTS_blend_equation_advanced_extension_directive_warn(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.blend_equation_advanced.extension_directive_warn*")

    def test_ES32_CTS_texture_cube_map_array_stencil_attachments_mutable_nonlayered(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.stencil_attachments_mutable_nonlayered*")

    def test_ES32_CTS_blend_equation_advanced_mismatching_qualifier(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.blend_equation_advanced.mismatching_qualifier*")

    def test_ES32_CTS_blend_equation_advanced_missing_qualifier(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.blend_equation_advanced.missing_qualifier*")

    def test_ES32_CTS_blend_equation_advanced_MRT_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.blend_equation_advanced.MRT_array*")

    def test_ES32_CTS_texture_cube_map_array_stencil_attachments_mutable_layered(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.stencil_attachments_mutable_layered*")

    def test_ES32_CTS_blend_equation_advanced_MRT_separate(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.blend_equation_advanced.MRT_separate*")

    def test_ES32_CTS_blend_equation_advanced_test_coherency(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.blend_equation_advanced.test_coherency*")

    def test_ES32_CTS_draw_buffers_indexed_blending(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.draw_buffers_indexed.blending*")

    def test_ES32_CTS_texture_cube_map_array_stencil_attachments_immutable_nonlayered(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.stencil_attachments_immutable_nonlayered*")

    def test_ES32_CTS_texture_cube_map_array_sampling(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.sampling*")

    def test_ES32_CTS_texture_cube_map_array_stencil_attachments_immutable_layered(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.stencil_attachments_immutable_layered*")

    def test_ES32_CTS_texture_cube_map_array_image_texture_size(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.image_texture_size*")

    def test_ES32_CTS_draw_buffers_indexed_color_masks(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.draw_buffers_indexed.color_masks")

    def test_ES32_CTS_texture_cube_map_array_image_op_vertex_sh(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.image_op_vertex_sh*")

    def test_ES32_CTS_draw_buffers_indexed_coverage(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.draw_buffers_indexed.coverage*")

    def test_ES32_CTS_draw_buffers_indexed_default_state(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.draw_buffers_indexed.default_state*")

    def test_ES32_CTS_draw_buffers_indexed_negative(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.draw_buffers_indexed.negative*")

    def test_ES32_CTS_draw_buffers_indexed_set_get(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.draw_buffers_indexed.set_get*")

    def test_ES32_CTS_texture_cube_map_array_image_op_tessellation_evaluation_sh(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.image_op_tessellation_evaluation_sh*")

    def test_ES32_CTS_geometry_shader_adjacency(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.adjacency*")

    def test_ES32_CTS_texture_cube_map_array_image_op_tessellation_control_sh(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.image_op_tessellation_control_sh*")

    def test_ES32_CTS_geometry_shader_api(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.api*")

    def test_ES32_CTS_geometry_shader_blitting(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.blitting*")

    def test_ES32_CTS_texture_cube_map_array_image_op_geometry_sh(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.image_op_geometry_sh*")

    def test_ES32_CTS_geometry_shader_clipping(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.clipping*")

    def test_ES32_CTS_texture_cube_map_array_image_op_fragment_sh(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.image_op_fragment_sh*")

    def test_ES32_CTS_geometry_shader_constant_variables(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.constant_variables*")

    def test_ES32_CTS_geometry_shader_input(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.input*")

    def test_ES32_CTS_texture_cube_map_array_image_op_compute_sh(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.image_op_compute_sh*")

    def test_ES32_CTS_geometry_shader_layered_framebuffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.layered_framebuffer*")

    def test_ES32_CTS_texture_cube_map_array_getter_calls(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.getter_calls*")

    def test_ES32_CTS_geometry_shader_layered_rendering(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.layered_rendering*")

    def test_ES32_CTS_texture_cube_map_array_generate_mip_map_non_filterable_mutable_storage(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.generate_mip_map_non_filterable_mutable_storage*")

    def test_ES32_CTS_texture_cube_map_array_generate_mip_map_non_filterable_immutable_storage(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.generate_mip_map_non_filterable_immutable_storage*")

    def test_ES32_CTS_texture_cube_map_array_generate_mip_map_filterable_internalformat_mutable(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.generate_mip_map_filterable_internalformat_mutable*")

    def test_ES32_CTS_texture_cube_map_array_generate_mip_map_filterable_internalformat_immutable(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.generate_mip_map_filterable_internalformat_immutable*")

    def test_ES32_CTS_geometry_shader_layered_rendering_boundary_condition(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.layered_rendering_boundary_condition*")

    def test_ES32_CTS_texture_cube_map_array_fbo_incompleteness(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.fbo_incompleteness*")

    def test_ES32_CTS_geometry_shader_layered_rendering_fbo_no_attachment(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.layered_rendering_fbo_no_attachment*")

    def test_ES32_CTS_texture_cube_map_array_color_depth_attachments(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_cube_map_array.color_depth_attachments*")

    def test_ES32_CTS_texture_buffer_texture_buffer_texture_buffer_range(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_buffer.texture_buffer_texture_buffer_range*")

    def test_ES32_CTS_sample_shading_render(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.sample_shading.render*")

    def test_ES32_CTS_geometry_shader_limits(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.limits*")

    def test_ES32_CTS_geometry_shader_linking(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.linking*")

    def test_ES32_CTS_geometry_shader_nonarray_input(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.nonarray_input*")

    def test_ES32_CTS_texture_buffer_texture_buffer_precision(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_buffer.texture_buffer_precision*")

    def test_ES32_CTS_geometry_shader_output(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.output*")

    def test_ES32_CTS_texture_buffer_texture_buffer_parameters(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_buffer.texture_buffer_parameters*")

    def test_ES32_CTS_sample_variables_mask(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.sample_variables.mask*")

    def test_ES32_CTS_texture_buffer_texture_buffer_operations_transform_feedback(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_buffer.texture_buffer_operations_transform_feedback*")

    def test_ES32_CTS_geometry_shader_primitive_counter(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.primitive_counter*")

    def test_ES32_CTS_geometry_shader_primitive_queries(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.primitive_queries*")

    def test_ES32_CTS_texture_buffer_texture_buffer_operations_ssbo_writes(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_buffer.texture_buffer_operations_ssbo_writes*")

    def test_ES32_CTS_geometry_shader_program_resource(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.program_resource*")

    def test_ES32_CTS_texture_buffer_texture_buffer_operations_framebuffer_readback(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_buffer.texture_buffer_operations_framebuffer_readback*")

    def test_ES32_CTS_geometry_shader_qualifiers(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.qualifiers*")

    def test_ES32_CTS_geometry_shader_rendering(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.geometry_shader.rendering*")

    def test_ES32_CTS_gpu_shader5_atomic_counters_array_indexing(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.atomic_counters_array_indexing*")

    def test_ES32_CTS_gpu_shader5_fma_accuracy(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.fma_accuracy*")

    def test_ES32_CTS_texture_buffer_texture_buffer_operations_image_store(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_buffer.texture_buffer_operations_image_store*")

    def test_ES32_CTS_gpu_shader5_fma_precision_float(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.fma_precision_float*")

    def test_ES32_CTS_gpu_shader5_fma_precision_vec2(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.fma_precision_vec2*")

    def test_ES32_CTS_texture_buffer_texture_buffer_operations_cpu_writes(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_buffer.texture_buffer_operations_cpu_writes*")

    def test_ES32_CTS_sample_variables_position(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.sample_variables.position*")

    def test_ES32_CTS_gpu_shader5_fma_precision_vec3(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.fma_precision_vec3*")

    def test_ES32_CTS_gpu_shader5_fma_precision_vec4(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.fma_precision_vec4*")

    def test_ES32_CTS_texture_buffer_texture_buffer_operations_buffer_load(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_buffer.texture_buffer_operations_buffer_load*")

    def test_ES32_CTS_gpu_shader5_images_array_indexing(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.images_array_indexing*")

    def test_ES32_CTS_texture_buffer_texture_buffer_max_size(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_buffer.texture_buffer_max_size*")

    def test_ES32_CTS_gpu_shader5_precise_qualifier(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.precise_qualifier*")

    def test_ES32_CTS_texture_buffer_texture_buffer_errors(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_buffer.texture_buffer_errors")

    def test_ES32_CTS_gpu_shader5_sampler_array_indexing(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.sampler_array_indexing*")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_color_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_color_array*")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_color_clamp_to_border(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_color_clamp_to_border*")

    def test_ES32_CTS_texture_buffer_texture_buffer_conv_int_to_float(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_buffer.texture_buffer_conv_int_to_float*")

    def test_ES32_CTS_shader_multisample_interpolation_api(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.shader_multisample_interpolation.api*")

    def test_ES32_CTS_texture_buffer_texture_buffer_buffer_parameters(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_buffer.texture_buffer_buffer_parameters*")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_color_clamp_to_edge(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_color_clamp_to_edge*")

    def test_ES32_CTS_texture_buffer_texture_buffer_atomic_functions(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_buffer.texture_buffer_atomic_functions*")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_color_repeat(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_color_repeat*")

    def test_ES32_CTS_texture_buffer_texture_buffer_active_uniform_validation_fragment_shader(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_buffer.texture_buffer_active_uniform_validation_fragment_shader*")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_depth_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_depth_array*")

    def test_ES32_CTS_texture_buffer_texture_buffer_active_uniform_validation_compute_shader(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_buffer.texture_buffer_active_uniform_validation_compute_shader*")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_depth_clamp_border(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_depth_clamp_border*")

    def test_ES32_CTS_texture_border_clamp_texparameteri_errors(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_border_clamp.texparameteri_errors*")

    def test_ES32_CTS_texture_border_clamp_sampling_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_border_clamp.sampling_texture*")

    def test_ES32_CTS_shader_multisample_interpolation_render(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.shader_multisample_interpolation.render*")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_depth_clamp_edge(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_depth_clamp_edge*")

    def test_ES32_CTS_texture_border_clamp_samplerparameteri_with_wrong_pname(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_border_clamp.samplerparameteri_with_wrong_pname*")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_depth_repeat(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_depth_repeat*")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_depth_repeat_y(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_depth_repeat_y*")

    def test_ES32_CTS_tessellation_shader_compilation_and_linking_errors(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.compilation_and_linking_errors*")

    def test_ES32_CTS_texture_border_clamp_samplerparameteri_non_gen_sampler_error(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_border_clamp.samplerparameteri_non_gen_sampler_error*")

    def test_ES32_CTS_gpu_shader5_texture_gather_offsets_color(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.texture_gather_offsets_color*")

    def test_ES32_CTS_gpu_shader5_texture_gather_offsets_depth(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.texture_gather_offsets_depth*")

    def test_ES32_CTS_texture_border_clamp_samplerparameteri_border_color(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_border_clamp.samplerparameteri_border_color*")

    def test_ES32_CTS_tessellation_shader_default_values_of_context_wide_properties(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.default_values_of_context_wide_properties*")

    def test_ES32_CTS_gpu_shader5_uniform_blocks_array_indexing(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.gpu_shader5.uniform_blocks_array_indexing*")

    def test_ES32_CTS_texture_border_clamp_gettexparameteri_errors(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_border_clamp.gettexparameteri_errors*")

    def test_ES32_CTS_tessellation_shader_ext_program_interface_query_dependency(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.ext_program_interface_query_dependency*")

    def test_ES32_CTS_tessellation_shader_isolines_tessellation(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.isolines_tessellation*")

    def test_ES32_CTS_info(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.info*")

    def test_ES32_CTS_tessellation_shader_max_patch_vertices(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.max_patch_vertices*")

    def test_ES32_CTS_texture_border_clamp_border_color_errors(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.texture_border_clamp.border_color_errors*")

    def test_ES32_CTS_tessellation_shader_primitive_coverage(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.primitive_coverage*")

    def test_ES32_CTS_sample_shading_api(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.sample_shading.api*")

    def test_ES32_CTS_tessellation_shader_xfb_captures_data_from_correct_stage(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.xfb_captures_data_from_correct_stage*")

    def test_ES32_CTS_tessellation_shader_vertex_spacing(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.vertex_spacing*")

    def test_ES32_CTS_tessellation_shader_program_object_properties(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.program_object_properties*")

    def test_ES32_CTS_tessellation_shader_vertex_ordering(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.vertex_ordering*")

    def test_ES32_CTS_tessellation_shader_tessellation_control_to_tessellation_evaluation(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.tessellation_control_to_tessellation_evaluation*")

    def test_ES32_CTS_tessellation_shader_tessellation_invariance(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.tessellation_invariance*")

    def test_ES32_CTS_tessellation_shader_tessellation_shader_triangles_tessellation(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.tessellation_shader_triangles_tessellation*")

    def test_ES32_CTS_tessellation_shader_tessellation_shader_tessellation(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.tessellation_shader_tessellation*")

    def test_ES32_CTS_tessellation_shader_tessellation_shader_point_mode(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.tessellation_shader_point_mode*")

    def test_ES32_CTS_tessellation_shader_tessellation_shader_tc_barriers(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.tessellation_shader_tc_barriers*")

    def test_ES32_CTS_tessellation_shader_tessellation_shader_quads_tessellation(self):
        print "[RunTest]: %s" % self.__str__()
        self._glcts.run_case("ES32-CTS.tessellation_shader.tessellation_shader_quads_tessellation*")