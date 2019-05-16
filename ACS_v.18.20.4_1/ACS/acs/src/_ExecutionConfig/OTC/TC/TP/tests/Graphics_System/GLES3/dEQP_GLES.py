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

    def test_GLES3_attribute_location(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.attribute_location.*")

    def test_GLES3_buffer_copy(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.buffer.copy.*")

    def test_GLES3_buffer_map(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.buffer.map.*")

    def test_GLES3_buffer_write(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.buffer.write.*")

    def test_GLES3_clipping_line(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.clipping.line.*")

    def test_GLES3_clipping_point(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.clipping.point.*")

    def test_GLES3_clipping_polygon(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.clipping.polygon.*")

    def test_GLES3_clipping_polygon_edge(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.clipping.polygon_edge.*")

    def test_GLES3_clipping_triangle_vertex(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.clipping.triangle_vertex.*")

    def test_GLES3_color_clear(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.color_clear.*")

    def test_GLES3_default_vertex_array_object_vertex_attrib_divisor(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.default_vertex_array_object.vertex_attrib_divisor*")

    def test_GLES3_default_vertex_attrib(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.default_vertex_attrib.*")

    def test_GLES3_depth_stencil_clear(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.depth_stencil_clear.*")

    def test_GLES3_dither_disabled(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.dither.disabled.*")

    def test_GLES3_dither_enabled(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.dither.enabled.*")

    def test_GLES3_draw_draw_arrays(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.draw.draw_arrays.*")

    def test_GLES3_draw_draw_arrays_instanced(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.draw.draw_arrays_instanced.*")

    def test_GLES3_draw_draw_elements(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.draw.draw_elements.*")

    def test_GLES3_draw_draw_elements_instanced(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.draw.draw_elements_instanced.*")

    def test_GLES3_draw_draw_range_elements(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.draw.draw_range_elements.*")

    def test_GLES3_draw_instancing(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.draw.instancing.*")

    def test_GLES3_draw_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.draw.random.*")

    def test_GLES3_fbo_api(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fbo.api.*")

    def test_GLES3_fbo_blit(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fbo.blit.*")

    def test_GLES3_fbo_color(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fbo.color.*")

    def test_GLES3_fbo_completeness(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fbo.completeness.*")

    def test_GLES3_fbo_depth(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fbo.depth.*")

    def test_GLES3_fbo_invalidate(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fbo.invalidate.*")

    def test_GLES3_fbo_msaa(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fbo.msaa.*")

    def test_GLES3_fbo_render(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fbo.render.*")

    def test_GLES3_fbo_stencil(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fbo.stencil.*")

    def test_GLES3_fence_sync(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fence_sync.*")

    def test_GLES3_flush_finish_finish(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.flush_finish.finish*")

    def test_GLES3_flush_finish_finish_wait(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.flush_finish.finish_wait*")

    def test_GLES3_flush_finish_flush(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.flush_finish.flush*")

    def test_GLES3_flush_finish_wait(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.flush_finish.wait*")

    def test_GLES3_fragment_ops_blend(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fragment_ops.blend.*")

    def test_GLES3_fragment_ops_depth(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fragment_ops.depth.*")

    def test_GLES3_fragment_ops_depth_stencil(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fragment_ops.depth_stencil.*")

    def test_GLES3_fragment_ops_interaction(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fragment_ops.interaction.*")

    def test_GLES3_fragment_ops_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fragment_ops.random.*")

    def test_GLES3_fragment_ops_scissor(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fragment_ops.scissor.*")

    def test_GLES3_fragment_ops_stencil(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fragment_ops.stencil.*")

    def test_GLES3_fragment_out_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fragment_out.array.*")

    def test_GLES3_fragment_out_basic(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fragment_out.basic.*")

    def test_GLES3_fragment_out_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.fragment_out.random.*")

    def test_GLES3_implementation_limits(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.implementation_limits.*")

    def test_GLES3_instanced(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.instanced.*")

    def test_GLES3_lifetime(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.lifetime.*")

    def test_GLES3_multisample(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.multisample.*")

    def test_GLES3_negative_api_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.negative_api.buffer.*")

    def test_GLES3_negative_api_fragment(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.negative_api.fragment.*")

    def test_GLES3_negative_api_shader(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.negative_api.shader.*")

    def test_GLES3_negative_api_state(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.negative_api.state.*")

    def test_GLES3_negative_api_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.negative_api.texture.*")

    def test_GLES3_negative_api_vertex_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.negative_api.vertex_array.*")

    def test_GLES3_occlusion_query(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.occlusion_query.*")

    def test_GLES3_pbo_native(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.pbo.native.*")

    def test_GLES3_pbo_renderbuffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.pbo.renderbuffer.*")

    def test_GLES3_polygon_offset(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.polygon_offset.*")

    def test_GLES3_prerequisite_clear_color(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.prerequisite.clear_color*")

    def test_GLES3_prerequisite_read_pixels(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.prerequisite.read_pixels*")

    def test_GLES3_prerequisite_state_reset(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.prerequisite.state_reset*")

    def test_GLES3_primitive_restart_basic(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.primitive_restart.basic.*")

    def test_GLES3_primitive_restart_begin_restart(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.primitive_restart.begin_restart.*")

    def test_GLES3_primitive_restart_begin_restart_duplicate_restarts(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.primitive_restart.begin_restart_duplicate_restarts.*")

    def test_GLES3_primitive_restart_begin_restart_end_restart(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.primitive_restart.begin_restart_end_restart.*")

    def test_GLES3_primitive_restart_begin_restart_end_restart_duplicate_restarts(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.primitive_restart.begin_restart_end_restart_duplicate_restarts.*")

    def test_GLES3_primitive_restart_duplicate_restarts(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.primitive_restart.duplicate_restarts.*")

    def test_GLES3_primitive_restart_end_restart(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.primitive_restart.end_restart.*")

    def test_GLES3_primitive_restart_end_restart_duplicate_restarts(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.primitive_restart.end_restart_duplicate_restarts.*")

    def test_GLES3_rasterization(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.rasterization.*")

    def test_GLES3_rasterizer_discard_basic(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.rasterizer_discard.basic.*")

    def test_GLES3_rasterizer_discard_fbo(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.rasterizer_discard.fbo.*")

    def test_GLES3_rasterizer_discard_scissor(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.rasterizer_discard.scissor.*")

    def test_GLES3_read_pixels_alignment(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.read_pixels.alignment.*")

    def test_GLES3_read_pixels_rowlength(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.read_pixels.rowlength.*")

    def test_GLES3_read_pixels_skip(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.read_pixels.skip.*")

    def test_GLES3_samplers_multi_cubemap(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.samplers.multi_cubemap.*")

    def test_GLES3_samplers_multi_tex_2d(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.samplers.multi_tex_2d.*")

    def test_GLES3_samplers_multi_tex_3d(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.samplers.multi_tex_3d.*")

    def test_GLES3_samplers_single_cubemap(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.samplers.single_cubemap.*")

    def test_GLES3_samplers_single_tex_2d(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.samplers.single_tex_2d.*")

    def test_GLES3_samplers_single_tex_3d(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.samplers.single_tex_3d.*")

    def test_GLES3_shader_api_compile_link(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shader_api.compile_link.*")

    def test_GLES3_shader_api_create_delete(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shader_api.create_delete.*")

    def test_GLES3_shader_api_program_binary(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shader_api.program_binary.*")

    def test_GLES3_shader_api_program_state(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shader_api.program_state.*")

    def test_GLES3_shader_api_shader_source(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shader_api.shader_source.*")

    def test_GLES3_shaders_arrays(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.arrays.*")

    def test_GLES3_shaders_builtin_functions(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.builtin_functions.*")

    def test_GLES3_shaders_builtin_variable(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.builtin_variable.*")

    def test_GLES3_shaders_conditionals(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.conditionals.*")

    def test_GLES3_shaders_constant_expressions(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.constant_expressions.*")

    def test_GLES3_shaders_constants(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.constants.*")

    def test_GLES3_shaders_conversions(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.conversions.*")

    def test_GLES3_shaders_declarations(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.declarations.*")

    def test_GLES3_shaders_derivate(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.derivate.*")

    def test_GLES3_shaders_discard(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.discard.*")

    def test_GLES3_shaders_fragdata(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.fragdata.*")

    def test_GLES3_shaders_fragdepth(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.fragdepth.*")

    def test_GLES3_shaders_functions(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.functions.*")

    def test_GLES3_shaders_indexing(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.indexing.*")

    def test_GLES3_shaders_invariance(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.invariance.*")

    def test_GLES3_shaders_keywords(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.keywords.*")

    def test_GLES3_shaders_linkage(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.linkage.*")

    def test_GLES3_shaders_loops(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.loops.*")

    def test_GLES3_shaders_matrix(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.matrix.*")

    def test_GLES3_shaders_negative(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.negative.*")

    def test_GLES3_shaders_operator(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.operator.*")

    def test_GLES3_shaders_precision(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.precision.*")

    def test_GLES3_shaders_preprocessor(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.preprocessor.*")

    def test_GLES3_shaders_qualification_order(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.qualification_order.*")

    def test_GLES3_shaders_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.random.*")

    def test_GLES3_shaders_return(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.return.*")

    def test_GLES3_shaders_scoping(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.scoping.*")

    def test_GLES3_shaders_struct(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.struct.*")

    def test_GLES3_shaders_switch(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.switch.*")

    def test_GLES3_shaders_swizzles(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.swizzles.*")

    def test_GLES3_shaders_texture_functions(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.texture_functions.*")

    def test_GLES3_shaders_uniform_block(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.shaders.uniform_block.*")

    def test_GLES3_state_query_boolean(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.state_query.boolean.*")

    def test_GLES3_state_query_buffer_object(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.state_query.buffer_object.*")

    def test_GLES3_state_query_fbo(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.state_query.fbo.*")

    def test_GLES3_state_query_floats(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.state_query.floats.*")

    def test_GLES3_state_query_indexed(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.state_query.indexed.*")

    def test_GLES3_state_query_integers(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.state_query.integers.*")

    def test_GLES3_state_query_integers64(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.state_query.integers64.*")

    def test_GLES3_state_query_internal_format(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.state_query.internal_format.*")

    def test_GLES3_state_query_rbo(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.state_query.rbo.*")

    def test_GLES3_state_query_sampler(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.state_query.sampler.*")

    def test_GLES3_state_query_shader(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.state_query.shader.*")

    def test_GLES3_state_query_string(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.state_query.string.*")

    def test_GLES3_state_query_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.state_query.texture.*")

    def test_GLES3_texture_compressed(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.texture.compressed.*")

    def test_GLES3_texture_filtering(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.texture.filtering.*")

    def test_GLES3_texture_format(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.texture.format.*")

    def test_GLES3_texture_mipmap(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.texture.mipmap.*")

    def test_GLES3_texture_shadow(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.texture.shadow.*")

    def test_GLES3_texture_size(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.texture.size.*")

    def test_GLES3_texture_specification(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.texture.specification.*")

    def test_GLES3_texture_swizzle(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.texture.swizzle.*")

    def test_GLES3_texture_units(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.texture.units.*")

    def test_GLES3_texture_vertex(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.texture.vertex.*")

    def test_GLES3_texture_wrap(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.texture.wrap.*")

    def test_GLES3_transform_feedback_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.transform_feedback.array.*")

    def test_GLES3_transform_feedback_array_element(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.transform_feedback.array_element.*")

    def test_GLES3_transform_feedback_basic_types(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.transform_feedback.basic_types.*")

    def test_GLES3_transform_feedback_interpolation(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.transform_feedback.interpolation.*")

    def test_GLES3_transform_feedback_point_size(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.transform_feedback.point_size.*")

    def test_GLES3_transform_feedback_position(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.transform_feedback.position.*")

    def test_GLES3_transform_feedback_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.transform_feedback.random.*")

    def test_GLES3_ubo_instance_array_basic_type(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.ubo.instance_array_basic_type.*")

    def test_GLES3_ubo_multi_basic_types(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.ubo.multi_basic_types.*")

    def test_GLES3_ubo_multi_nested_struct(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.ubo.multi_nested_struct.*")

    def test_GLES3_ubo_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.ubo.random.*")

    def test_GLES3_ubo_single_basic_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.ubo.single_basic_array.*")

    def test_GLES3_ubo_single_basic_type(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.ubo.single_basic_type.*")

    def test_GLES3_ubo_single_nested_struct(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.ubo.single_nested_struct.*")

    def test_GLES3_ubo_single_nested_struct_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.ubo.single_nested_struct_array.*")

    def test_GLES3_ubo_single_struct(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.ubo.single_struct.*")

    def test_GLES3_ubo_single_struct_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.ubo.single_struct_array.*")

    def test_GLES3_uniform_api_info_query(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.uniform_api.info_query.*")

    def test_GLES3_uniform_api_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.uniform_api.random.*")

    def test_GLES3_uniform_api_value(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.uniform_api.value.*")

    def test_GLES3_vertex_array_objects(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.vertex_array_objects.*")

    def test_GLES3_vertex_arrays_multiple_attributes(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.vertex_arrays.multiple_attributes.*")

    def test_GLES3_vertex_arrays_single_attribute(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.vertex_arrays.single_attribute.*")

    def test_GLES3_info(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.info.*")

    def test_GLES3_Texture_Compression_ETC2(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.texture.format.compressed.etc2*")

    def test_GLES3_Texture_Compression_ASTC(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES3.functional.texture.compressed.astc*")
