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
@summary: dEQP GLES32
@since: 05/18/2017
@author: Ding, JunnanX
'''
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.GLCTS_impl import GLCTSImpl
from testlib.graphics.common import Logcat
from testlib.graphics.dEQP_impl import deqp_impl


class RundEQP(UIATestBase):

    @classmethod
    def setUpClass(self):
        super(RundEQP, self).setUpClass()
        g_common_obj.root_on_device()

    @classmethod
    def tearDownClass(self):
        super(RundEQP, self).tearDownClass()

    def setUp(self):
        super(RundEQP, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.executor = GLCTSImpl()
        self.log = Logcat()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(RundEQP, self).tearDown()

    def test_ES32_CTS_blend_equation_advanced_blend_all(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.blend_equation_advanced.blend_all")

    def test_ES32_CTS_blend_equation_advanced_BlendEquationSeparate(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.blend_equation_advanced.BlendEquationSeparate")

    def test_ES32_CTS_texture_cube_map_array_tex3D_validation(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.tex3D_validation")

    def test_ES32_CTS_blend_equation_advanced_blend_specific(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.blend_equation_advanced.blend_specific")

    def test_ES32_CTS_blend_equation_advanced_coherentEnableDisable(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.blend_equation_advanced.coherentEnableDisable")

    def test_ES32_CTS_blend_equation_advanced_extension_directive_enable(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.blend_equation_advanced.extension_directive_enable")

    def test_ES32_CTS_texture_cube_map_array_subimage3D(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.subimage3D")

    def test_ES32_CTS_blend_equation_advanced_extension_directive_warn(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.blend_equation_advanced.extension_directive_warn")

    def test_ES32_CTS_texture_cube_map_array_stencil_attachments_mutable_nonlayered(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.stencil_attachments_mutable_nonlayered")

    def test_ES32_CTS_blend_equation_advanced_mismatching_qualifier(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.blend_equation_advanced.mismatching_qualifier")

    def test_ES32_CTS_blend_equation_advanced_missing_qualifier(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.blend_equation_advanced.missing_qualifier")

    def test_ES32_CTS_blend_equation_advanced_MRT_array(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.blend_equation_advanced.MRT_array")

    def test_ES32_CTS_texture_cube_map_array_stencil_attachments_mutable_layered(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.stencil_attachments_mutable_layered")

    def test_ES32_CTS_blend_equation_advanced_MRT_separate(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.blend_equation_advanced.MRT_separate")

    def test_ES32_CTS_blend_equation_advanced_test_coherency(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.blend_equation_advanced.test_coherency")

    def test_ES32_CTS_draw_buffers_indexed_blending(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.draw_buffers_indexed.blending")

    def test_ES32_CTS_texture_cube_map_array_stencil_attachments_immutable_nonlayered(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.stencil_attachments_immutable_nonlayered")

    def test_ES32_CTS_texture_cube_map_array_sampling(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.sampling")

    def test_ES32_CTS_texture_cube_map_array_stencil_attachments_immutable_layered(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.stencil_attachments_immutable_layered")

    def test_ES32_CTS_texture_cube_map_array_image_texture_size(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.image_texture_size")

    def test_ES32_CTS_draw_buffers_indexed_color_masks(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.draw_buffers_indexed.color_masks")

    def test_ES32_CTS_texture_cube_map_array_image_op_vertex_sh(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.image_op_vertex_sh")

    def test_ES32_CTS_draw_buffers_indexed_coverage(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.draw_buffers_indexed.coverage")

    def test_ES32_CTS_draw_buffers_indexed_default_state(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.draw_buffers_indexed.default_state")

    def test_ES32_CTS_draw_buffers_indexed_negative(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.draw_buffers_indexed.negative")

    def test_ES32_CTS_draw_buffers_indexed_set_get(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.draw_buffers_indexed.set_get")

    def test_ES32_CTS_texture_cube_map_array_image_op_tessellation_evaluation_sh(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.image_op_tessellation_evaluation_sh")

    def test_ES32_CTS_geometry_shader_adjacency(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.adjacency")

    def test_ES32_CTS_texture_cube_map_array_image_op_tessellation_control_sh(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.image_op_tessellation_control_sh")

    def test_ES32_CTS_geometry_shader_api(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.api")

    def test_ES32_CTS_geometry_shader_blitting(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.blitting")

    def test_ES32_CTS_texture_cube_map_array_image_op_geometry_sh(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.image_op_geometry_sh")

    def test_ES32_CTS_geometry_shader_clipping(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.clipping")

    def test_ES32_CTS_texture_cube_map_array_image_op_fragment_sh(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.image_op_fragment_sh")

    def test_ES32_CTS_geometry_shader_constant_variables(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.constant_variables")

    def test_ES32_CTS_geometry_shader_input(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.input")

    def test_ES32_CTS_texture_cube_map_array_image_op_compute_sh(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.image_op_compute_sh")

    def test_ES32_CTS_geometry_shader_layered_framebuffer(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.layered_framebuffer")

    def test_ES32_CTS_texture_cube_map_array_getter_calls(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.getter_calls")

    def test_ES32_CTS_geometry_shader_layered_rendering(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.layered_rendering")

    def test_ES32_CTS_texture_cube_map_array_generate_mip_map_non_filterable_mutable_storage(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.generate_mip_map_non_filterable_mutable_storage")

    def test_ES32_CTS_texture_cube_map_array_generate_mip_map_non_filterable_immutable_storage(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.generate_mip_map_non_filterable_immutable_storage")

    def test_ES32_CTS_texture_cube_map_array_generate_mip_map_filterable_internalformat_mutable(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.generate_mip_map_filterable_internalformat_mutable")

    def test_ES32_CTS_texture_cube_map_array_generate_mip_map_filterable_internalformat_immutable(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.generate_mip_map_filterable_internalformat_immutable")

    def test_ES32_CTS_geometry_shader_layered_rendering_boundary_condition(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.layered_rendering_boundary_condition")

    def test_ES32_CTS_texture_cube_map_array_fbo_incompleteness(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.fbo_incompleteness")

    def test_ES32_CTS_geometry_shader_layered_rendering_fbo_no_attachment(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.layered_rendering_fbo_no_attachment")

    def test_ES32_CTS_texture_cube_map_array_color_depth_attachments(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_cube_map_array.color_depth_attachments")

    def test_ES32_CTS_texture_buffer_texture_buffer_texture_buffer_range(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_buffer.texture_buffer_texture_buffer_range")

    def test_ES32_CTS_sample_shading_render(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.sample_shading.render")

    def test_ES32_CTS_geometry_shader_limits(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.limits")

    def test_ES32_CTS_geometry_shader_linking(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.linking")

    def test_ES32_CTS_geometry_shader_nonarray_input(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.nonarray_input")

    def test_ES32_CTS_texture_buffer_texture_buffer_precision(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_buffer.texture_buffer_precision")

    def test_ES32_CTS_geometry_shader_output(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.output")

    def test_ES32_CTS_texture_buffer_texture_buffer_parameters(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_buffer.texture_buffer_parameters")

    def test_ES32_CTS_sample_variables_mask(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.sample_variables.mask")

    def test_ES32_CTS_texture_buffer_texture_buffer_operations_transform_feedback(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_buffer.texture_buffer_operations_transform_feedback")

    def test_ES32_CTS_geometry_shader_primitive_counter(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.primitive_counter")

    def test_ES32_CTS_geometry_shader_primitive_queries(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.primitive_queries")

    def test_ES32_CTS_texture_buffer_texture_buffer_operations_ssbo_writes(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_buffer.texture_buffer_operations_ssbo_writes")

    def test_ES32_CTS_geometry_shader_program_resource(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.program_resource")

    def test_ES32_CTS_texture_buffer_texture_buffer_operations_framebuffer_readback(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_buffer.texture_buffer_operations_framebuffer_readback")

    def test_ES32_CTS_geometry_shader_qualifiers(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.qualifiers")

    def test_ES32_CTS_geometry_shader_rendering(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.geometry_shader.rendering")

    def test_ES32_CTS_gpu_shader5_atomic_counters_array_indexing(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.atomic_counters_array_indexing")

    def test_ES32_CTS_gpu_shader5_fma_accuracy(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.fma_accuracy")

    def test_ES32_CTS_texture_buffer_texture_buffer_operations_image_store(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_buffer.texture_buffer_operations_image_store")

    def test_ES32_CTS_gpu_shader5_fma_precision_float(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.fma_precision_float")

    def test_ES32_CTS_gpu_shader5_fma_precision_vec2(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.fma_precision_vec2")

    def test_ES32_CTS_texture_buffer_texture_buffer_operations_cpu_writes(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_buffer.texture_buffer_operations_cpu_writes")

    def test_ES32_CTS_sample_variables_position(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.sample_variables.position")

    def test_ES32_CTS_gpu_shader5_fma_precision_vec3(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.fma_precision_vec3")

    def test_ES32_CTS_gpu_shader5_fma_precision_vec4(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.fma_precision_vec4")

    def test_ES32_CTS_texture_buffer_texture_buffer_operations_buffer_load(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_buffer.texture_buffer_operations_buffer_load")

    def test_ES32_CTS_gpu_shader5_images_array_indexing(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.images_array_indexing")

    def test_ES32_CTS_texture_buffer_texture_buffer_max_size(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_buffer.texture_buffer_max_size")

    def test_ES32_CTS_gpu_shader5_precise_qualifier(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.precise_qualifier")

    def test_ES32_CTS_texture_buffer_texture_buffer_errors(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_buffer.texture_buffer_errors")

    def test_ES32_CTS_gpu_shader5_sampler_array_indexing(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.sampler_array_indexing")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_color_array(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_color_array")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_color_clamp_to_border(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_color_clamp_to_border")

    def test_ES32_CTS_texture_buffer_texture_buffer_conv_int_to_float(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_buffer.texture_buffer_conv_int_to_float")

    def test_ES32_CTS_shader_multisample_interpolation_api(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.shader_multisample_interpolation.api")

    def test_ES32_CTS_texture_buffer_texture_buffer_buffer_parameters(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_buffer.texture_buffer_buffer_parameters")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_color_clamp_to_edge(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_color_clamp_to_edge")

    def test_ES32_CTS_texture_buffer_texture_buffer_atomic_functions(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_buffer.texture_buffer_atomic_functions")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_color_repeat(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_color_repeat")

    def test_ES32_CTS_texture_buffer_texture_buffer_active_uniform_validation_fragment_shader(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_buffer.texture_buffer_active_uniform_validation_fragment_shader")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_depth_array(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_depth_array")

    def test_ES32_CTS_texture_buffer_texture_buffer_active_uniform_validation_compute_shader(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_buffer.texture_buffer_active_uniform_validation_compute_shader")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_depth_clamp_border(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_depth_clamp_border")

    def test_ES32_CTS_texture_border_clamp_texparameteri_errors(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_border_clamp.texparameteri_errors")

    def test_ES32_CTS_texture_border_clamp_sampling_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_border_clamp.sampling_texture")

    def test_ES32_CTS_shader_multisample_interpolation_render(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.shader_multisample_interpolation.render")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_depth_clamp_edge(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_depth_clamp_edge")

    def test_ES32_CTS_texture_border_clamp_samplerparameteri_with_wrong_pname(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_border_clamp.samplerparameteri_with_wrong_pname")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_depth_repeat(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_depth_repeat")

    def test_ES32_CTS_gpu_shader5_texture_gather_offset_depth_repeat_y(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.texture_gather_offset_depth_repeat_y")

    def test_ES32_CTS_tessellation_shader_compilation_and_linking_errors(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.compilation_and_linking_errors")

    def test_ES32_CTS_texture_border_clamp_samplerparameteri_non_gen_sampler_error(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_border_clamp.samplerparameteri_non_gen_sampler_error")

    def test_ES32_CTS_gpu_shader5_texture_gather_offsets_color(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.texture_gather_offsets_color")

    def test_ES32_CTS_gpu_shader5_texture_gather_offsets_depth(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.texture_gather_offsets_depth")

    def test_ES32_CTS_texture_border_clamp_samplerparameteri_border_color(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_border_clamp.samplerparameteri_border_color")

    def test_ES32_CTS_tessellation_shader_default_values_of_context_wide_properties(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.default_values_of_context_wide_properties")

    def test_ES32_CTS_gpu_shader5_uniform_blocks_array_indexing(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.gpu_shader5.uniform_blocks_array_indexing")

    def test_ES32_CTS_texture_border_clamp_gettexparameteri_errors(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_border_clamp.gettexparameteri_errors")

    def test_ES32_CTS_tessellation_shader_ext_program_interface_query_dependency(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.ext_program_interface_query_dependency")

    def test_ES32_CTS_tessellation_shader_isolines_tessellation(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.isolines_tessellation")

    def test_ES32_CTS_info(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.info")

    def test_ES32_CTS_tessellation_shader_max_patch_vertices(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.max_patch_vertices")

    def test_ES32_CTS_texture_border_clamp_border_color_errors(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.texture_border_clamp.border_color_errors")

    def test_ES32_CTS_tessellation_shader_primitive_coverage(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.primitive_coverage")

    def test_ES32_CTS_sample_shading_api(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.sample_shading.api")

    def test_ES32_CTS_tessellation_shader_xfb_captures_data_from_correct_stage(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.xfb_captures_data_from_correct_stage")

    def test_ES32_CTS_tessellation_shader_vertex_spacing(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.vertex_spacing")

    def test_ES32_CTS_tessellation_shader_program_object_properties(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.program_object_properties")

    def test_ES32_CTS_tessellation_shader_vertex_ordering(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.vertex_ordering")

    def test_ES32_CTS_tessellation_shader_tessellation_control_to_tessellation_evaluation(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.tessellation_control_to_tessellation_evaluation")

    def test_ES32_CTS_tessellation_shader_tessellation_invariance(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.tessellation_invariance")

    def test_ES32_CTS_tessellation_shader_tessellation_shader_triangles_tessellation(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.tessellation_shader_triangles_tessellation")

    def test_ES32_CTS_tessellation_shader_tessellation_shader_tessellation(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.tessellation_shader_tessellation")

    def test_ES32_CTS_tessellation_shader_tessellation_shader_point_mode(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.tessellation_shader_point_mode")

    def test_ES32_CTS_tessellation_shader_tessellation_shader_tc_barriers(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.tessellation_shader_tc_barriers")

    def test_ES32_CTS_tessellation_shader_tessellation_shader_quads_tessellation(self):
        print "[RunTest]: %s" % self.__str__()
        self.executor.run_case("ES32-CTS.tessellation_shader.tessellation_shader_quads_tessellation")

    def test_OpenGLES_32_Support_dumpsys_SurfaceFlinger(self):
        print "[RunTest]: %s" % self.__str__()
        assert self.log.check_dumpsys_SurfaceFlinger_info(keyword="gles", assertword="OpenGL ES 3.2"),\
            "OpenGL ES version is not 3.2!"

    def test_OpenGLES_32_Support_Grafika(self):
        "Same step as above test since it's separated with different case in HPALM"
        print "[RunTest]: %s" % self.__str__()
        assert self.log.check_dumpsys_SurfaceFlinger_info(keyword="gles", assertword="OpenGL ES 3.2"),\
            "OpenGL ES version is not 3.2!"

    def test_OpenGLES_32_Support_OpenGL_Extensions_Viewer(self):
        "Same step as above test since it's separated with different case in HPALM"
        print "[RunTest]: %s" % self.__str__()
        assert self.log.check_dumpsys_SurfaceFlinger_info(keyword="gles", assertword="OpenGL ES 3.2"),\
            "OpenGL ES version is not 3.2!"

    def test_EGL_get_proc_address_core_32(self):
        print "[RunTest]: %s" % self.__str__()
        deqp_impl.run_case("dEQP-EGL.functional.get_proc_address.core.*")

    def test_GLES31_android_extension_pack_extensions_32(self):
        print "[RunTest]: %s" % self.__str__()
        deqp_impl.run_case("dEQP-GLES31.functional.android_extension_pack.extensions*")

    def test_GLES31_android_extension_pack_limits_32(self):
        print "[RunTest]: %s" % self.__str__()
        deqp_impl.run_case("dEQP-GLES31.functional.android_extension_pack.limits*")

    def test_GLES31_android_extension_pack_shaders_32(self):
        print "[RunTest]: %s" % self.__str__()
        deqp_impl.run_case("dEQP-GLES31.functional.android_extension_pack.shaders*")

    def test_GLES31_info_extensions_32(self):
        print "[RunTest]: %s" % self.__str__()
        sub_case_list =  ['KHR_debug',
                          'KHR_texture_compression_astc_ldr',
                          'KHR_blend_equation_advanced',
                          'OES_sample_shading',
                          'OES_sample_variables',
                          'OES_shader_image_atomic',
                          'OES_shader_multisample_interpolation',
                          'OES_texture_stencil8',
                          'OES_texture_storage_multisample_2d_array',
                          'EXT_copy_image', 'OES_copy_image',
                          'EXT_draw_buffers_indexed','OES_draw_buffers_indexed',
                          'EXT_geometry_shader','OES_geometry_shader',
                          'EXT_gpu_shader5','OES_gpu_shader5',
                          'EXT_primitive_bounding_box','OES_primitive_bounding_box',
                          'EXT_shader_io_blocks','OES_shader_io_blocks',
                          'EXT_tessellation_shader','OES_tessellation_shader',
                          'EXT_texture_border_clamp','OES_texture_border_clamp',
                          'EXT_texture_buffer','OES_texture_buffer',
                          'EXT_texture_cube_map_array','OES_texture_cube_map_array',
                          'OES_draw_elements_base_vertex',
                          'EXT_color_buffer_float',
                          'KHR_robustness']
        deqp_impl.run_case("dEQP-GLES31.info.extensions*")
        output = g_common_obj.adb_cmd_capture_msg("cat %s" % deqp_impl.inside_output)
        for sub_c in sub_case_list:
            sub_case_name = "GL_" + sub_c
            assert sub_case_name in output, "%s is not in extension list" % sub_case_name
