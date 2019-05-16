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

    def test_GLES31_atomic_counter(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.atomic_counter.*")

    def test_GLES31_blend_equation_advanced_barrier(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.blend_equation_advanced.barrier.*")

    def test_GLES31_blend_equation_advanced_basic(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.blend_equation_advanced.basic.*")

    def test_GLES31_blend_equation_advanced_coherent(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.blend_equation_advanced.coherent.*")

    def test_GLES31_blend_equation_advanced_coherent_msaa(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.blend_equation_advanced.coherent_msaa.*")

    def test_GLES31_blend_equation_advanced_msaa(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.blend_equation_advanced.msaa.*")

    def test_GLES31_blend_equation_advanced_srgb(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.blend_equation_advanced.srgb.*")

    def test_GLES31_blend_equation_advanced_state_query(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.blend_equation_advanced.state_query.*")

    def test_GLES31_compute_basic(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.compute.basic.*")

    def test_GLES31_compute_indirect_dispatch(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.compute.indirect_dispatch.*")

    def test_GLES31_compute_shared_var(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.compute.shared_var.*")

    def test_GLES31_copy_image_compressed(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.copy_image.compressed.*")

    def test_GLES31_copy_image_mixed(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.copy_image.mixed.*")

    def test_GLES31_copy_image_non_compressed(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.copy_image.non_compressed.*")

    def test_GLES31_debug_async(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.debug.async.*")

    def test_GLES31_debug_error_filters(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.debug.error_filters.*")

    def test_GLES31_debug_error_groups(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.debug.error_groups.*")

    def test_GLES31_debug_externally_generated(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.debug.externally_generated.*")

    def test_GLES31_debug_negative_coverage(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.debug.negative_coverage.*")

    def test_GLES31_debug_object_labels(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.debug.object_labels.*")

    def test_GLES31_debug_state_query(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.debug.state_query.*")

    def test_GLES31_default_vertex_array_object_vertex_attrib_divisor(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.default_vertex_array_object.vertex_attrib_divisor*")

    def test_GLES31_draw_buffers_indexed_overwrite_common(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.draw_buffers_indexed.overwrite_common.*")

    def test_GLES31_draw_buffers_indexed_overwrite_indexed(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.draw_buffers_indexed.overwrite_indexed.*")

    def test_GLES31_draw_buffers_indexed_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.draw_buffers_indexed.random.*")

    def test_GLES31_draw_indirect_compute_interop(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.draw_indirect.compute_interop.*")

    def test_GLES31_draw_indirect_draw_arrays_indirect(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.draw_indirect.draw_arrays_indirect.*")

    def test_GLES31_draw_indirect_draw_elements_indirect(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.draw_indirect.draw_elements_indirect.*")

    def test_GLES31_draw_indirect_instancing(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.draw_indirect.instancing.*")

    def test_GLES31_draw_indirect_negative(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.draw_indirect.negative.*")

    def test_GLES31_draw_indirect_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.draw_indirect.random.*")

    def test_GLES31_fbo_color(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.fbo.color.*")

    def test_GLES31_fbo_completeness(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.fbo.completeness.*")

    def test_GLES31_fbo_no_attachments(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.fbo.no_attachments.*")

    def test_GLES31_geometry_shading_basic(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.geometry_shading.basic.*")

    def test_GLES31_geometry_shading_conversion(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.geometry_shading.conversion.*")

    def test_GLES31_geometry_shading_emit(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.geometry_shading.emit.*")

    def test_GLES31_geometry_shading_input(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.geometry_shading.input.*")

    def test_GLES31_geometry_shading_instanced(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.geometry_shading.instanced.*")

    def test_GLES31_geometry_shading_layered(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.geometry_shading.layered.*")

    def test_GLES31_geometry_shading_negative(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.geometry_shading.negative.*")

    def test_GLES31_geometry_shading_query(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.geometry_shading.query.*")

    def test_GLES31_geometry_shading_varying(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.geometry_shading.varying.*")

    def test_GLES31_geometry_shading_vertex_transform_feedback(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.geometry_shading.vertex_transform_feedback.*")

    def test_GLES31_image_load_store_2d(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.image_load_store.2d.*")

    def test_GLES31_image_load_store_2d_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.image_load_store.2d_array.*")

    def test_GLES31_image_load_store_3d(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.image_load_store.3d.*")

    def test_GLES31_image_load_store_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.image_load_store.buffer.*")

    def test_GLES31_image_load_store_cube(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.image_load_store.cube.*")

    def test_GLES31_image_load_store_early_fragment_tests(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.image_load_store.early_fragment_tests.*")

    def test_GLES31_layout_binding_image(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.layout_binding.image.*")

    def test_GLES31_layout_binding_negative(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.layout_binding.negative.*")

    def test_GLES31_layout_binding_sampler(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.layout_binding.sampler.*")

    def test_GLES31_layout_binding_ssbo(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.layout_binding.ssbo.*")

    def test_GLES31_layout_binding_ubo(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.layout_binding.ubo.*")

    def test_GLES31_multisample_default_framebuffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.multisample.default_framebuffer.*")

    def test_GLES31_primitive_bounding_box_blit_fbo(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.primitive_bounding_box.blit_fbo.*")

    def test_GLES31_primitive_bounding_box_call_order(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.primitive_bounding_box.call_order.*")

    def test_GLES31_primitive_bounding_box_clear(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.primitive_bounding_box.clear.*")

    def test_GLES31_primitive_bounding_box_depth(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.primitive_bounding_box.depth.*")

    def test_GLES31_primitive_bounding_box_lines(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.primitive_bounding_box.lines.*")

    def test_GLES31_primitive_bounding_box_points(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.primitive_bounding_box.points.*")

    def test_GLES31_primitive_bounding_box_state_query(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.primitive_bounding_box.state_query.*")

    def test_GLES31_primitive_bounding_box_triangles(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.primitive_bounding_box.triangles.*")

    def test_GLES31_primitive_bounding_box_wide_lines(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.primitive_bounding_box.wide_lines.*")

    def test_GLES31_primitive_bounding_box_wide_points(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.primitive_bounding_box.wide_points.*")

    def test_GLES31_program_interface_query_atomic_counter_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.program_interface_query.atomic_counter_buffer.*")

    def test_GLES31_program_interface_query_buffer_limited_query(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.program_interface_query.buffer_limited_query.*")

    def test_GLES31_program_interface_query_buffer_variable(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.program_interface_query.buffer_variable.*")

    def test_GLES31_program_interface_query_program_input(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.program_interface_query.program_input.*")

    def test_GLES31_program_interface_query_program_output(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.program_interface_query.program_output.*")

    def test_GLES31_program_interface_query_shader_storage_block(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.program_interface_query.shader_storage_block.*")

    def test_GLES31_program_interface_query_transform_feedback_varying(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.program_interface_query.transform_feedback_varying.*")

    def test_GLES31_program_interface_query_uniform(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.program_interface_query.uniform.*")

    def test_GLES31_program_interface_query_uniform_block(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.program_interface_query.uniform_block.*")

    def test_GLES31_program_uniform_basic_array_assign_full(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.program_uniform.basic_array_assign_full.*")

    def test_GLES31_program_uniform_basic_array_assign_partial(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.program_uniform.basic_array_assign_partial.*")

    def test_GLES31_program_uniform_by_pointer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.program_uniform.by_pointer.*")

    def test_GLES31_program_uniform_by_value(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.program_uniform.by_value.*")

    def test_GLES31_program_uniform_unused_uniforms(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.program_uniform.unused_uniforms.*")

    def test_GLES31_sample_shading_min_sample_shading(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.sample_shading.min_sample_shading.*")

    def test_GLES31_sample_shading_state_query(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.sample_shading.state_query.*")

    def test_GLES31_separate_shader_api(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.separate_shader.api.*")

    def test_GLES31_separate_shader_create_shader_program(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.separate_shader.create_shader_program.*")

    def test_GLES31_separate_shader_interface(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.separate_shader.interface.*")

    def test_GLES31_separate_shader_pipeline(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.separate_shader.pipeline.*")

    def test_GLES31_separate_shader_program_uniform(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.separate_shader.program_uniform.*")

    def test_GLES31_separate_shader_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.separate_shader.random.*")

    def test_GLES31_separate_shader_validation(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.separate_shader.validation.*")

    def test_GLES31_shaders_arrays(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.shaders.arrays.*")

    def test_GLES31_shaders_arrays_of_arrays(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.shaders.arrays_of_arrays.*")

    def test_GLES31_shaders_builtin_constants(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.shaders.builtin_constants.*")

    def test_GLES31_shaders_builtin_functions(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.shaders.builtin_functions.*")

    def test_GLES31_shaders_builtin_var(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.shaders.builtin_var.*")

    def test_GLES31_shaders_functions(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.shaders.functions.*")

    def test_GLES31_shaders_helper_invocation(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.shaders.helper_invocation.*")

    def test_GLES31_shaders_implicit_conversions(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.shaders.implicit_conversions.*")

    def test_GLES31_shaders_linkage(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.shaders.linkage.*")

    def test_GLES31_shaders_multisample_interpolation(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.shaders.multisample_interpolation.*")

    def test_GLES31_shaders_opaque_type_indexing(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.shaders.opaque_type_indexing.*")

    def test_GLES31_shaders_sample_variables(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.shaders.sample_variables.*")

    def test_GLES31_shaders_uniform_block(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.shaders.uniform_block.*")

    def test_GLES31_ssbo_array_length(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.ssbo.array_length.*")

    def test_GLES31_ssbo_atomic(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.ssbo.atomic.*")

    def test_GLES31_ssbo_layout(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.ssbo.layout.*")

    def test_GLES31_state_query_boolean(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.state_query.boolean.*")

    def test_GLES31_state_query_framebuffer_default(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.state_query.framebuffer_default.*")

    def test_GLES31_state_query_indexed(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.state_query.indexed.*")

    def test_GLES31_state_query_integer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.state_query.integer.*")

    def test_GLES31_state_query_internal_format(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.state_query.internal_format.*")

    def test_GLES31_state_query_multisample_interpolation(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.state_query.multisample_interpolation.*")

    def test_GLES31_state_query_program(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.state_query.program.*")

    def test_GLES31_state_query_program_pipeline(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.state_query.program_pipeline.*")

    def test_GLES31_state_query_sampler(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.state_query.sampler.*")

    def test_GLES31_state_query_shader(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.state_query.shader.*")

    def test_GLES31_state_query_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.state_query.texture.*")

    def test_GLES31_state_query_texture_level(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.state_query.texture_level.*")

    def test_GLES31_state_query_vertex_attribute_binding(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.state_query.vertex_attribute_binding.*")

    def test_GLES31_stencil_texturing_format(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.stencil_texturing.format.*")

    def test_GLES31_stencil_texturing_misc(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.stencil_texturing.misc.*")

    def test_GLES31_stencil_texturing_render(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.stencil_texturing.render.*")

    def test_GLES31_synchronization_in_invocation(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.synchronization.in_invocation.*")

    def test_GLES31_synchronization_inter_call(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.synchronization.inter_call.*")

    def test_GLES31_synchronization_inter_invocation(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.synchronization.inter_invocation.*")

    def test_GLES31_tessellation_common_edge(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.tessellation.common_edge.*")

    def test_GLES31_tessellation_fractional_spacing(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.tessellation.fractional_spacing.*")

    def test_GLES31_tessellation_geometry_interaction_feedback(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.tessellation_geometry_interaction.feedback.*")

    def test_GLES31_tessellation_geometry_interaction_point_size(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.tessellation_geometry_interaction.point_size.*")

    def test_GLES31_tessellation_geometry_interaction_render(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.tessellation_geometry_interaction.render.*")

    def test_GLES31_tessellation_invariance(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.tessellation.invariance.*")

    def test_GLES31_tessellation_misc_draw(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.tessellation.misc_draw.*")

    def test_GLES31_tessellation_primitive_discard(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.tessellation.primitive_discard.*")

    def test_GLES31_tessellation_shader_input_output(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.tessellation.shader_input_output.*")

    def test_GLES31_tessellation_state_query(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.tessellation.state_query.*")

    def test_GLES31_tessellation_tesscoord(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.tessellation.tesscoord.*")

    def test_GLES31_tessellation_user_defined_io(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.tessellation.user_defined_io.*")

    def test_GLES31_tessellation_winding(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.tessellation.winding.*")

    def test_GLES31_texture_border_clamp(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.texture.border_clamp.*")

    def test_GLES31_texture_filtering(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.texture.filtering.*")

    def test_GLES31_texture_format(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.texture.format.*")

    def test_GLES31_texture_gather(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.texture.gather.*")

    def test_GLES31_texture_multisample(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.texture.multisample.*")

    def test_GLES31_texture_specification(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.texture.specification.*")

    def test_GLES31_texture_texture_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.texture.texture_buffer.*")

    def test_GLES31_ubo_2_level_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.ubo.2_level_array.*")

    def test_GLES31_ubo_2_level_struct_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.ubo.2_level_struct_array.*")

    def test_GLES31_ubo_3_level_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.ubo.3_level_array.*")

    def test_GLES31_ubo_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.ubo.random.*")

    def test_GLES31_uniform_location_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.uniform_location.array.*")

    def test_GLES31_uniform_location_basic(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.uniform_location.basic.*")

    def test_GLES31_uniform_location_link(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.uniform_location.link.*")

    def test_GLES31_uniform_location_min_max(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.uniform_location.min_max.*")

    def test_GLES31_uniform_location_negative(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.uniform_location.negative.*")

    def test_GLES31_uniform_location_nested_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.uniform_location.nested_array.*")

    def test_GLES31_uniform_location_nested_struct(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.uniform_location.nested_struct.*")

    def test_GLES31_uniform_location_struct(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.uniform_location.struct.*")

    def test_GLES31_vertex_attribute_binding_negative(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.vertex_attribute_binding.negative.*")

    def test_GLES31_vertex_attribute_binding_usage(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.vertex_attribute_binding.usage.*")

    def test_GLES31_atomic_counter_dec(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.functional.atomic_counter.*")

    def test_GLES31_info(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-GLES31.info.*")
