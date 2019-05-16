# -*- coding: utf-8 -*-
'''
@summary: run Oglconform
@since: 05/27/2017
@author: rui
'''
import os
from testlib.graphics.oglconform_impl import OglconformImpl
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.util.config import TestConfig


class RunConformance(UIATestBase):

    merge_result = []
    config_file = "tests.tablet.androidextensionpack.conf"

    @classmethod
    def setUpClass(self):
        super(RunConformance, self).setUpClass()
        if g_common_obj.adb_cmd_capture_msg("ps | grep adbd")[0:4] != "root":
            g_common_obj.root_on_device()

    @classmethod
    def tearDownClass(self):
        super(RunConformance, self).tearDownClass()

    def setUp(self):
        super(RunConformance, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._oglconform = OglconformImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RunConformance, self).tearDown()
        self._oglcnform = None
        self.merge_result = []

    def test_GL_NV_draw_buffers(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es2', 'GL_NV_draw_buffers')

    def test_GL_NV_fbo_color_attachments(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es2', 'GL_NV_fbo_color_attachments')

    def test_GL_NV_read_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es2', 'GL_NV_read_buffer')

    def test_GL_OES_surfaceless_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_surfaceless_context')

    def test_GL_APPLE_texture_max_level(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_APPLE_texture_max_level')

    def test_GL_EXT_read_format_bgra(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_EXT_read_format_bgra')

    def test_GL_EXT_texture_format_BGRA8888(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_EXT_texture_format_BGRA8888')

    def test_GL_EXT_blend_minmax(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_EXT_blend_minmax')

    def test_GL_EXT_robustness(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_EXT_robustness')

    def test_GL_EXT_texture_lod_bias(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_EXT_texture_lod_bias')

    def test_GL_EXT_discard_framebuffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_EXT_discard_framebuffer')

    def test_GL_EXT_texture_compression_dxt1(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_EXT_texture_compression_dxt1')

    def test_GL_EXT_texture_storage(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_EXT_texture_storage')

    def test_GL_EXT_multi_draw_arrays(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_EXT_multi_draw_arrays')

    def test_GL_EXT_texture_filter_anisotropic(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_EXT_texture_filter_anisotropic')

    def test_GL_KHR_debug(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_KHR_debug')

    def test_GL_EXT_debug_marker(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_EXT_debug_marker')

    def test_GL_EXT_sRGB(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_EXT_sRGB')

    def test_GL_EXT_texture_sRGB_decode(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_EXT_texture_sRGB_decode')

    def test_GL_EXT_map_buffer_range(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_EXT_map_buffer_range')

    def test_GL_EXT_texture_compression_s3tc(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_EXT_texture_compression_s3tc')

    def test_GL_INTEL_performance_query(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_INTEL_performance_query')

    def test_GL_OES_blend_equation_separate(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_blend_equation_separate')

    def test_GL_OES_blend_func_separate(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_blend_func_separate')

    def test_GL_OES_blend_subtract(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_blend_subtract')

    def test_GL_OES_byte_coordinates(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_byte_coordinates')

    def test_GL_OES_compressed_ETC1_RGB8_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_compressed_ETC1_RGB8_texture')

    def test_GL_OES_compressed_paletted_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_compressed_paletted_texture')

    def test_GL_OES_EGL_sync(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_EGL_sync')

    def test_GL_OES_depth24(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_depth24')

    def test_GL_OES_draw_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_draw_texture')

    def test_GL_OES_EGL_image(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_EGL_image')

    def test_GL_OES_EGL_image_external(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_EGL_image_external')

    def test_GL_OES_element_index_uint(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_element_index_uint')

    def test_GL_OES_fixed_point(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_fixed_point')

    def test_GL_OES_framebuffer_object(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_framebuffer_object')

    def test_GL_OES_fbo_render_mipmap(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_fbo_render_mipmap')

    def test_GL_OES_mapbuffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_mapbuffer')

    def test_GL_OES_matrix_get(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_matrix_get')

    def test_GL_OES_packed_depth_stencil(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_packed_depth_stencil')

    def test_GL_OES_point_size_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_point_size_array')

    def test_GL_OES_point_sprite(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_point_sprite')

    def test_GL_OES_required_internalformat(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_required_internalformat')

    def test_GL_OES_rgb8_rgba8(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_rgb8_rgba8')

    def test_GL_OES_single_precision(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_single_precision')

    def test_GL_OES_query_matrix(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_query_matrix')

    def test_GL_OES_stencil_wrap(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_stencil_wrap')

    def test_GL_OES_stencil8(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_stencil8')

    def test_GL_OES_texture_cube_map(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_texture_cube_map')

    def test_GL_OES_texture_env_crossbar(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_texture_env_crossbar')

    def test_GL_OES_read_format(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_read_format')

    def test_GL_OES_texture_mirrored_repeat(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_texture_mirrored_repeat')

    def test_GL_OES_texture_npot(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_texture_npot')

    def test_GL_OES_vertex_array_object(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es1', 'GL_OES_vertex_array_object')

    def test_31_driver_GL_NV_polygon_mode(self):
        print "[RunTest]: %s" % self.__str__()
        self._oglconform.check_extension_with_verbosity_level('4', 'es2', 'GL_NV_polygon_mode')
        self._oglconform.check_extension_with_verbosity_level('4', 'es3', 'GL_NV_polygon_mode')

    def test_31_driver_GL_KHRs(self):
        print "[RunTest]: %s" % self.__str__()
        case_list = TestConfig().read(self.config_file, 'opengles31_extensions').get('gl_khr').split(',')
        for i in case_list:
            self.merge_result.append(\
                self._oglconform.check_extension_with_verbosity_level_get_result('4', 'es2', i))
        rst1 = [self.merge_result[i] for i, x in enumerate(self.merge_result) if x[1] == 'FAIL']
        if rst1 == []:
            print "All of the cases in <es2> list are passed!"
        else:
            for _ in rst1:
                print ">" * 10, "%s is %s in es2" % (_[0], _[1]), "<" * 10
        self.merge_result = []
        for i in case_list:
            self.merge_result.append(\
                self._oglconform.check_extension_with_verbosity_level_get_result('4', 'es3', i))
        rst2 = [self.merge_result[i] for i, x in enumerate(self.merge_result) if x[1] == 'FAIL']
        if rst2 == []:
            print "All of the cases in <es3> list are passed!"
        else:
            for _ in rst2:
                print ">" * 10, "%s is %s in es3" % (_[0], _[1]), "<" * 10
        assert rst1 == [], "Some tests are failed in es2 or es3!"
        assert rst2 == [], "Some tests are failed in es2 or es3!"

    def test_31_driver_GL_INTELs(self):
        print "[RunTest]: %s" % self.__str__()
        case_list = TestConfig().read(self.config_file, 'opengles31_extensions').get('gl_intel').split(',')
        for i in case_list:
            self.merge_result.append(\
                self._oglconform.check_extension_with_verbosity_level_get_result('4', 'es2', i))
        rst1 = [self.merge_result[i] for i, x in enumerate(self.merge_result) if x[1] == 'FAIL']
        if rst1 == []:
            print "All of the cases in <es2> list are passed!"
        else:
            for _ in rst1:
                print ">" * 10, "%s is %s in es2" % (_[0], _[1]), "<" * 10
        self.merge_result = []
        for i in case_list:
            self.merge_result.append(\
                self._oglconform.check_extension_with_verbosity_level_get_result('4', 'es3', i))
        rst2 = [self.merge_result[i] for i, x in enumerate(self.merge_result) if x[1] == 'FAIL']
        if rst2 == []:
            print "All of the cases in <es3> list are passed!"
        else:
            for _ in rst2:
                print ">" * 10, "%s is %s in es3" % (_[0], _[1]), "<" * 10
        assert rst1 == [], "Some tests are failed in es2 or es3!"
        assert rst2 == [], "Some tests are failed in es2 or es3!"

    def test_31_driver_GL_OESs(self):
        print "[RunTest]: %s" % self.__str__()
        case_list = TestConfig().read(self.config_file, 'opengles31_extensions').get('gl_oes').split(',')
        for i in case_list:
            self.merge_result.append(\
                self._oglconform.check_extension_with_verbosity_level_get_result('4', 'es2', i))
        rst1 = [self.merge_result[i] for i, x in enumerate(self.merge_result) if x[1] == 'FAIL']
        if rst1 == []:
            print "All of the cases in <es2> list are passed!"
        else:
            for _ in rst1:
                print ">" * 10, "%s is %s in es2" % (_[0], _[1]), "<" * 10
        self.merge_result = []
        for i in case_list:
            self.merge_result.append(\
                self._oglconform.check_extension_with_verbosity_level_get_result('4', 'es3', i))
        rst2 = [self.merge_result[i] for i, x in enumerate(self.merge_result) if x[1] == 'FAIL']
        if rst2 == []:
            print "All of the cases in <es3> list are passed!"
        else:
            for _ in rst2:
                print ">" * 10, "%s is %s in es3" % (_[0], _[1]), "<" * 10
        assert rst1 == [], "Some tests are failed in es2 or es3!"
        assert rst2 == [], "Some tests are failed in es2 or es3!"

    def test_31_driver_GL_EXTs(self):
        print "[RunTest]: %s" % self.__str__()
        case_list = TestConfig().read(self.config_file, 'opengles31_extensions').get('gl_ext').split(',')
        for i in case_list:
            self.merge_result.append(\
                self._oglconform.check_extension_with_verbosity_level_get_result('4', 'es2', i))
        rst1 = [self.merge_result[i] for i, x in enumerate(self.merge_result) if x[1] == 'FAIL']
        if rst1 == []:
            print "All of the cases in <es2> list are passed!"
        else:
            for _ in rst1:
                print ">" * 10, "%s is %s in es2" % (_[0], _[1]), "<" * 10
        self.merge_result = []
        for i in case_list:
            self.merge_result.append(\
                self._oglconform.check_extension_with_verbosity_level_get_result('4', 'es3', i))
        rst2 = [self.merge_result[i] for i, x in enumerate(self.merge_result) if x[1] == 'FAIL']
        if rst2 == []:
            print "All of the cases in <es3> list are passed!"
        else:
            for _ in rst2:
                print ">" * 10, "%s is %s in es3" % (_[0], _[1]), "<" * 10
        assert rst1 == [], "Some tests are failed in es2 or es3!"
        assert rst2 == [], "Some tests are failed in es2 or es3!"
