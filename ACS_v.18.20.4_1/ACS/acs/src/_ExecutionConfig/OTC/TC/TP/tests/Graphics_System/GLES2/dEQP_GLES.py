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

    def test_GLES2_accuracy_interpolation(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.accuracy.interpolation.*")

    def test_GLES2_accuracy_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.accuracy.texture.*")

    def test_GLES2_attribute_location(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.attribute_location.*")

    def test_GLES2_buffer_write(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.buffer.write.*")

    def test_GLES2_clipping_line(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.clipping.line.*")

    def test_GLES2_clipping_point(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.clipping.point.*")

    def test_GLES2_clipping_polygon(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.clipping.polygon.*")

    def test_GLES2_clipping_polygon_edge(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.clipping.polygon_edge.*")

    def test_GLES2_clipping_triangle_vertex(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.clipping.triangle_vertex.*")

    def test_GLES2_color_clear(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.color_clear.*")

    def test_GLES2_debug_marker(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.debug_marker.*")

    def test_GLES2_default_vertex_attrib(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.default_vertex_attrib.*")

    def test_GLES2_depth_range_compare(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.depth_range.compare.*")

    def test_GLES2_depth_range_write(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.depth_range.write.*")

    def test_GLES2_depth_stencil_clear(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.depth_stencil_clear.*")

    def test_GLES2_dither_disabled(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.dither.disabled.*")

    def test_GLES2_dither_enabled(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.dither.enabled.*")

    def test_GLES2_draw_draw_arrays(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.draw.draw_arrays.*")

    def test_GLES2_draw_draw_elements(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.draw.draw_elements.*")

    def test_GLES2_draw_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.draw.random.*")

    def test_GLES2_fbo_api(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.fbo.api.*")

    def test_GLES2_fbo_completeness(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.fbo.completeness.*")

    def test_GLES2_fbo_render(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.fbo.render.*")

    def test_GLES2_flush_finish_wait(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.flush_finish.wait*")

    def test_GLES2_fragment_ops_blend(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.fragment_ops.blend.*")

    def test_GLES2_fragment_ops_depth(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.fragment_ops.depth.*")

    def test_GLES2_fragment_ops_depth_stencil(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.fragment_ops.depth_stencil.*")

    def test_GLES2_fragment_ops_interaction(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.fragment_ops.interaction.*")

    def test_GLES2_fragment_ops_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.fragment_ops.random.*")

    def test_GLES2_fragment_ops_scissor(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.fragment_ops.scissor.*")

    def test_GLES2_fragment_ops_stencil(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.fragment_ops.stencil.*")

    def test_GLES2_implementation_limits(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.implementation_limits.*")

    def test_GLES2_info(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.info.*")

    def test_GLES2_lifetime(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.lifetime.*")

    def test_GLES2_light_amount(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.light_amount.*")

    def test_GLES2_multisample(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.multisample.*")

    def test_GLES2_negative_api_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.negative_api.buffer.*")

    def test_GLES2_negative_api_fragment(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.negative_api.fragment.*")

    def test_GLES2_negative_api_shader(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.negative_api.shader.*")

    def test_GLES2_negative_api_state(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.negative_api.state.*")

    def test_GLES2_negative_api_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.negative_api.texture.*")

    def test_GLES2_negative_api_vertex_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.negative_api.vertex_array.*")

    def test_GLES2_polygon_offset(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.polygon_offset.*")

    def test_GLES2_prerequisite(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.prerequisite.*")

    def test_GLES2_rasterization_culling(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.rasterization.culling.*")

    def test_GLES2_rasterization_fill_rules(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.rasterization.fill_rules.*")

    def test_GLES2_rasterization_interpolation(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.rasterization.interpolation.*")

    def test_GLES2_rasterization_primitives(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.rasterization.primitives.*")

    def test_GLES2_read_pixels(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.read_pixels.*")

    def test_GLES2_shader_api(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shader_api.*")

    def test_GLES2_shaders_algorithm(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.algorithm.*")

    def test_GLES2_shaders_builtin_variable(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.builtin_variable.*")

    def test_GLES2_shaders_conditionals(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.conditionals.*")

    def test_GLES2_shaders_constant_expressions(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.constant_expressions.*")

    def test_GLES2_shaders_constants(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.constants.*")

    def test_GLES2_shaders_conversions(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.conversions.*")

    def test_GLES2_shaders_declarations(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.declarations.*")

    def test_GLES2_shaders_discard(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.discard.*")

    def test_GLES2_shaders_fragdata(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.fragdata.*")

    def test_GLES2_shaders_functions(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.functions.*")

    def test_GLES2_shaders_indexing(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.indexing.*")

    def test_GLES2_shaders_invariance(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.invariance.*")

    def test_GLES2_shaders_keywords(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.keywords.*")

    def test_GLES2_shaders_linkage(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.linkage.*")

    def test_GLES2_shaders_loops(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.loops.*")

    def test_GLES2_shaders_matrix(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.matrix.*")

    def test_GLES2_shaders_operator(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.operator.*")

    def test_GLES2_shaders_preprocessor(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.preprocessor.*")

    def test_GLES2_shaders_qualification_order(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.qualification_order.*")

    def test_GLES2_shaders_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.random.*")

    def test_GLES2_shaders_reserved_operators(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.reserved_operators.*")

    def test_GLES2_shaders_return(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.return.*")

    def test_GLES2_shaders_scoping(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.scoping.*")

    def test_GLES2_shaders_struct(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.struct.*")

    def test_GLES2_shaders_swizzles(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.swizzles.*")

    def test_GLES2_shaders_texture_functions(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.shaders.texture_functions.*")

    def test_GLES2_state_query_boolean(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.state_query.boolean.*")

    def test_GLES2_state_query_buffer_object(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.state_query.buffer_object.*")

    def test_GLES2_state_query_fbo(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.state_query.fbo.*")

    def test_GLES2_state_query_floats(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.state_query.floats.*")

    def test_GLES2_state_query_integers(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.state_query.integers.*")

    def test_GLES2_state_query_rbo(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.state_query.rbo.*")

    def test_GLES2_state_query_shader(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.state_query.shader.*")

    def test_GLES2_state_query_string(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.state_query.string.*")

    def test_GLES2_state_query_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.state_query.texture.*")

    def test_GLES2_texture_completeness(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.texture.completeness.*")

    def test_GLES2_Texture_Compression_DXTC(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.compressed_texture_formats.GL_EXT_texture_compression_dxt1")

    def test_GLES2_Texture_Compression_ETC1(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.capability.extensions.compressed_texture_formats.GL_OES_compressed_ETC1_RGB8_texture")

    def test_GLES2_texture_filtering(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.texture.filtering.*")

    def test_GLES2_texture_format(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.texture.format.*")

    def test_GLES2_texture_mipmap(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.texture.mipmap.*")

    def test_GLES2_texture_size(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.texture.size.*")

    def test_GLES2_texture_specification(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.texture.specification.*")

    def test_GLES2_texture_units(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.texture.units.*")

    def test_GLES2_texture_vertex(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.texture.vertex.*")

    def test_GLES2_texture_wrap(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.texture.wrap.*")

    def test_GLES2_uniform_api_info_query(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.uniform_api.info_query.*")

    def test_GLES2_uniform_api_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.uniform_api.random.*")

    def test_GLES2_uniform_api_value(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.uniform_api.value.*")

    def test_GLES2_vertex_arrays_multiple_attributes(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.vertex_arrays.multiple_attributes.*")

    def test_GLES2_vertex_arrays_single_attribute(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES2.functional.vertex_arrays.single_attribute.*")
