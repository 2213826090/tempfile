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
@summary: dEQP VK
@since: 08/30/2017
@author: Rui
'''
from testlib.graphics.dEQP_impl import dEQPImpl
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.common.base import clearTmpDir


class RundEQP(UIATestBase):

    @classmethod
    def setUpClass(self):
        super(RundEQP, self).setUpClass()
        if g_common_obj.adb_cmd_capture_msg("ps | grep adbd")[0:4] != "root":
            g_common_obj.root_on_device()

    @classmethod
    def tearDownClass(self):
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

    def test_VK_wsi_android(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.wsi.android.*")

    def test_VK_ubo_single_struct_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.ubo.single_struct_array.*")

    def test_VK_ubo_single_struct(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.ubo.single_struct.*")

    def test_VK_ubo_single_nested_struct_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.ubo.single_nested_struct_array.*")

    def test_VK_ubo_single_nested_struct(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.ubo.single_nested_struct.*")

    def test_VK_ubo_single_basic_type(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.ubo.single_basic_type.*")

    def test_VK_ubo_single_basic_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.ubo.single_basic_array.*")

    def test_VK_ubo_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.ubo.random.*")

    def test_VK_ubo_multi_nested_struct(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.ubo.multi_nested_struct.*")

    def test_VK_ubo_multi_basic_types(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.ubo.multi_basic_types.*")

    def test_VK_ubo_link_by_binding(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.ubo.link_by_binding.*")

    def test_VK_ubo_instance_array_basic_type(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.ubo.instance_array_basic_type.*")

    def test_VK_ubo_3_level_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.ubo.3_level_array.*")

    def test_VK_ubo_2_level_struct_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.ubo.2_level_struct_array.*")

    def test_VK_ubo_2_level_array(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.ubo.2_level_array.*")

    def test_VK_synchronization_semaphores(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.synchronization.smoke.semaphores*")

    def test_VK_synchronization_fences(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.synchronization.smoke.fences*")

    def test_VK_synchronization_events(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.synchronization.smoke.events*")

    def test_VK_ssbo_layout(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.ssbo.layout.*")

    def test_VK_spirv_assembly_instruction(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.spirv_assembly.instruction.*")

    def test_VK_sparse_resources_mipmap_sparse_residency(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.sparse_resources.mipmap_sparse_residency.*")

    def test_VK_sparse_resources_image_sparse_residency(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.sparse_resources.image_sparse_residency.*")

    def test_VK_sparse_resources_image_sparse_binding(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.sparse_resources.image_sparse_binding.*")

    def test_VK_sparse_resources_buffer_sparse_residency(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.sparse_resources.buffer.*.sparse_residency.*")

    def test_VK_sparse_resources_buffer_sparse_residency_aliased(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.sparse_resources.buffer.*.sparse_residency_aliased")

    def test_VK_sparse_resources_buffer_sparse_binding(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.sparse_resources.buffer.*.sparse_binding.*")

    def test_VK_renderpass_simple(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.renderpass.*.simple.*")

    def test_VK_renderpass_formats(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.renderpass.*.formats.*")

    def test_VK_renderpass_attachment_allocation(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.renderpass.*.attachment_allocation.*")

    def test_VK_renderpass_attachment(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.renderpass.*.attachment.*")

    def test_VK_query_pool_occlusion_query(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.query_pool.occlusion_query.*")

    def test_VK_pipeline_vertex_input(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.pipeline.vertex_input.*")

    def test_VK_pipeline_timestamp(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.pipeline.timestamp.*")

    def test_VK_pipeline_stencil(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.pipeline.stencil.*")

    def test_VK_pipeline_sampler(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.pipeline.sampler.*")

    def test_VK_pipeline_push_constant(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.pipeline.push_constant.*")

    def test_VK_pipeline_multisample(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.pipeline.multisample.*")

    def test_VK_pipeline_input_assembly(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.pipeline.input_assembly.*")

    def test_VK_pipeline_image_view(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.pipeline.image_view.*")

    def test_VK_pipeline_image(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.pipeline.image.*")

    def test_VK_pipeline_early_fragment(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.pipeline.*.early_fragment*")

    def test_VK_pipeline_depth(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.pipeline.depth.*")

    def test_VK_pipeline_cache(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.pipeline.cache.*")

    def test_VK_pipeline_blend(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.pipeline.blend.*")

    def test_VK_memory_pipeline_barrier(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.memory.pipeline_barrier.*")

    def test_VK_memory_mapping(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.memory.mapping.*")

    def test_VK_memory_allocation(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.memory.allocation.*")

    def test_VK_info_platform(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.info.platform*")

    def test_VK_info_memory_limits(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.info.memory_limits*")

    def test_VK_info_device(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.info.device*")

    def test_VK_info_build(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.info.build*")

    def test_VK_image_store(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.image.store.*")

    def test_VK_image_qualifiers(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.image.qualifiers.*")

    def test_VK_image_load_store(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.image.load_store.*")

    def test_VK_image_image_size(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.functions.*")

    def test_VK_image_format_reinterpret(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.image.format_reinterpret.*")

    def test_VK_image_atomic_operations(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.image.atomic_operations.*")

    def test_VK_glsl_swizzles(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.swizzles.*")

    def test_VK_glsl_switch(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.switch.*")

    def test_VK_glsl_struct(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.struct.*")

    def test_VK_glsl_scoping(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.scoping.*")

    def test_VK_glsl_return(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.return.*")

    def test_VK_glsl_operator(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.operator.*")

    def test_VK_glsl_opaque_type_indexing(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.opaque_type_indexing.*")

    def test_VK_glsl_matrix(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.matrix.*")

    def test_VK_glsl_loops(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.loops.*")

    def test_VK_glsl_linkage(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.linkage.*")

    def test_VK_glsl_indexing(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.indexing.*")

    def test_VK_glsl_functions(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.functions.*")

    def test_VK_glsl_discard(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.discard.*")

    def test_VK_glsl_conversions(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.conversions.*")

    def test_VK_glsl_constants(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.constants.*")

    def test_VK_glsl_constant_expressions(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.constant_expressions.*")

    def test_VK_glsl_conditionals(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.conditionals.*")

    def test_VK_glsl_builtin(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.builtin.*")

    def test_VK_glsl_arrays(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.glsl.arrays.*")

    def test_VK_dynamic_state_vp_state(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.dynamic_state.vp_state.*")

    def test_VK_dynamic_state_rs_state(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.dynamic_state.rs_state.*")

    def test_VK_dynamic_state_general_state(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.dynamic_state.general_state.*")

    def test_VK_dynamic_state_ds_state(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.dynamic_state.ds_state.*")

    def test_VK_dynamic_state_cb_state(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.dynamic_state.cb_state.*")

    def test_VK_draw_simple_draw(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.draw.simple_draw.*")

    def test_VK_draw_indirect_draw(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.draw.indirect_draw.*")

    def test_VK_draw_indexed_draw(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.draw.indexed_draw.*")

    def test_VK_compute_indirect_dispatch(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.compute.indirect_dispatch.*")

    def test_VK_compute_builtin_var(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.compute.builtin_var.*")

    def test_VK_compute_basic(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.compute.basic.*")

    def test_VK_binding_model_shader_access(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.binding_model.shader_access.*")

    def test_VK_api_smoke(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.api.smoke.*")

    def test_VK_api_object_management(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.api.object_management*")

    def test_VK_api_info(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.api.info.*")

    def test_VK_api_device_init(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.api.device_init.*")

    def test_VK_api_copy_and_blit(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.api.copy_and_blit.*")

    def test_VK_api_command_buffers(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.api.command_buffers.*")

    def test_VK_api_buffer_view(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.api.buffer_view.*")

    def test_VK_api_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-VK.api.buffer.*")
