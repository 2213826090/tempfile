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

    def test_EGL_choose_config_simple(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.choose_config.simple.*")

    def test_EGL_client_extensions_base(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.client_extensions.base*")

    def test_EGL_color_clears_multi_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.color_clears.multi_context.*")

    def test_EGL_color_clears_multi_thread(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.color_clears.multi_thread.*")

    def test_EGL_color_clears_single_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.color_clears.single_context.*")

    def test_EGL_create_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.create_context.*")

    def test_EGL_create_context_ext(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.create_context_ext.*")

    def test_EGL_create_surface_pbuffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.create_surface.pbuffer.*")

    def test_EGL_create_surface_pixmap(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.create_surface.pixmap.*")

    def test_EGL_create_surface_platform_pixmap(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.create_surface.platform_pixmap.*")

    def test_EGL_create_surface_platform_window(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.create_surface.platform_window.*")

    def test_EGL_create_surface_window(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.create_surface.window.*")

    def test_EGL_fence_sync_invalid(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.fence_sync.invalid.*")

    def test_EGL_fence_sync_valid(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.fence_sync.valid.*")

    def test_EGL_get_proc_address_core(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.core.*")

    def test_EGL_image_api(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.image.api.*")

    def test_EGL_image_create(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.image.create.*")

    def test_EGL_image_modify(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.image.modify.*")

    def test_EGL_image_render_multiple_contexts(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.image.render_multiple_contexts.*")

    def test_EGL_info(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.info.*")

    def test_EGL_multithread_config(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.multithread.config*")

    def test_EGL_multithread_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.multithread.context*")

    def test_EGL_multithread_pbuffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.multithread.pbuffer*")

    def test_EGL_multithread_pixmap(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.multithread.pixmap*")

    def test_EGL_multithread_pixmap_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.multithread.pixmap_context*")

    def test_EGL_multithread_pixmap_single_window(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.multithread.pixmap_single_window*")

    def test_EGL_multithread_pixmap_single_window_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.multithread.pixmap_single_window_context*")

    def test_EGL_multithread_pixmap_window(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.multithread.pixmap_window*")

    def test_EGL_multithread_single_window(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.multithread.single_window*")

    def test_EGL_native_color_mapping_native_pixmap(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.native_color_mapping.native_pixmap.*")

    def test_EGL_native_color_mapping_native_window(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.native_color_mapping.native_window.*")

    def test_EGL_native_color_mapping_pbuffer_to_native_pixmap(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.native_color_mapping.pbuffer_to_native_pixmap.*")

    def test_EGL_native_coord_mapping_native_pixmap(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.native_coord_mapping.native_pixmap.*")

    def test_EGL_native_coord_mapping_native_window(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.native_coord_mapping.native_window.*")

    def test_EGL_native_coord_mapping_pbuffer_to_native_pixmap(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.native_coord_mapping.pbuffer_to_native_pixmap.*")

    def test_EGL_negative_api(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.negative_api.*")

    def test_EGL_preserve_swap_no_preserve(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.preserve_swap.no_preserve.*")

    def test_EGL_preserve_swap_preserve(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.preserve_swap.preserve.*")

    def test_EGL_query_config_constraints(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.query_config.constraints.*")

    def test_EGL_query_config_get_config_attrib(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.query_config.get_config_attrib.*")

    def test_EGL_query_context_get_current_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.query_context.get_current_context.*")

    def test_EGL_query_context_get_current_display(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.query_context.get_current_display.*")

    def test_EGL_query_context_get_current_surface(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.query_context.get_current_surface.*")

    def test_EGL_query_context_query_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.query_context.query_context.*")

    def test_EGL_query_context_simple(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.query_context.simple.*")

    def test_EGL_query_surface_set_attribute(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.query_surface.set_attribute.*")

    def test_EGL_query_surface_simple(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.query_surface.simple.*")

    def test_EGL_render_multi_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.render.multi_context.*")

    def test_EGL_render_multi_thread(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.render.multi_thread.*")

    def test_EGL_render_single_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.render.single_context.*")

    def test_EGL_resize_surface_size(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.resize.surface_size.*")

    def test_EGL_reusable_sync_invalid(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.reusable_sync.invalid.*")

    def test_EGL_reusable_sync_valid(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.reusable_sync.valid.*")

    def test_EGL_surfaceless_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.surfaceless_context.*")

    def test_EGL_swap_buffers(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.swap_buffers.*")

    def test_EGL_sharing_gles2_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.sharing.gles2.buffer*")

    def test_EGL_sharing_gles2_texture(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.sharing.gles2.texture*")

    def test_EGL_sharing_gles2_program(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.sharing.gles2.program*")

    def test_EGL_sharing_gles2_shader(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.sharing.gles2.shader*")

    def test_EGL_sharing_gles2_multithread_simple(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.sharing.gles2.multithread.simple*")

    def test_EGL_sharing_gles2_multithread_simple_egl_sync(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.sharing.gles2.multithread.simple_egl_sync*")

    def test_EGL_sharing_gles2_multithread_simple_egl_server_sync(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.sharing.gles2.multithread.simple_egl_server_sync*")

    def test_EGL_query_config_get_configs(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.query_config.get_configs.*")

    def test_EGL_choose_config_random(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.choose_config.random.*")

    def test_EGL_multithread_pbuffer_single_window(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.multithread.pbuffer_single_window*")

    def test_EGL_multithread_pbuffer_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.multithread.pbuffer_context*")

    def test_EGL_multithread_single_window_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.multithread.single_window_context*")

    def test_EGL_multithread_pbuffer_single_window_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.multithread.pbuffer_single_window_context*")

    def test_EGL_get_proc_address_extension(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension*")

    def test_EGL_client_extensions_disjoint(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.client_extensions.disjoint*")

    def test_EGL_client_extensions_extensions(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.client_extensions.extensions*")

    def test_EGL_resize_back_buffer(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.resize.back_buffer*")

    def test_EGL_resize_pixel_density(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.resize.pixel_density*")

    def test_EGL_buffer_age(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.buffer_age*")

    def test_EGL_partial_update(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.partial_update*")

    def test_EGL_negative_partial_update(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.negative_partial_update*")

    def test_EGL_multicontext(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.multicontext*")

    def test_EGL_sharing_gles2_context(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.sharing.gles2.context*")

    def test_EGL_get_proc_address_extension_32(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.get_proc_address.extension*")

    def test_EGL_functional_wide_color(self):
        print "[RunTest]: %s" % self.__str__()
        self._deqp.run_case("dEQP-EGL.functional.wide_color.*")