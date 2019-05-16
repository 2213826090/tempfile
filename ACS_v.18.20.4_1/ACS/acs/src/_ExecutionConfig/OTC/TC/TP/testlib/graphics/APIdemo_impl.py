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
#published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.
# Any license under such intellectual property rights must be express
# and approved by Intel in writing.
"""
@summary: ChromeExtendImpl class
@since: 02/10/2015
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""

import time

from testlib.graphics import common
from testlib.graphics.common import busybox_obj
from testlib.graphics.common import logcat, pkgmgr
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle


class APIDemoImpl(object):

    """ APIDemoImpl """

    CONFIG_FILE = 'tests.common.apidemo.conf'
    PKG_NAME = "com.example.android.apis"
    MAIN_ACTIVITY = "com.example.android.apis.ApiDemos"

    CompressedTextureActivity = "com.example.android.apis.graphics.CompressedTextureActivity"
    CubeMapActivity = "com.example.android.apis.graphics.CubeMapActivity"
    FrameBufferObjectActivity = "com.example.android.apis.graphics.FrameBufferObjectActivity"
    GLSurfaceViewActivity = "com.example.android.apis.graphics.GLSurfaceViewActivity"
    KubeActivity = "com.example.android.apis.graphics.kube.Kube"
    MatrixPaletteActivity = "com.example.android.apis.graphics.MatrixPaletteActivity"
    GLES20Activity = "com.example.android.apis.graphics.GLES20Activity"
    SpriteTextActivity = "com.example.android.apis.graphics.spritetext.SpriteTextActivity"
    TriangleActivity = "com.example.android.apis.graphics.TriangleActivity"
    TouchRotateActivity = "com.example.android.apis.graphics.TouchRotateActivity"

    def __init__(self):
        self.device = g_common_obj.get_device()

        self.configer = TestConfig()
        self.config = self.configer.read(self.CONFIG_FILE, "APIDemoImpl")
        config_handle = ConfigHandle()
        self.config["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        self.arti = Artifactory(self.config.get('artifactory_location'))

    def setup(self):
        """setup implement's resource file"""
        busybox_obj.setup()

        if not pkgmgr._package_installed(pkgName=self.PKG_NAME):
            file_path = self.config.get("apk")
            apk_path = self.arti.get(file_path)
            pkgmgr.apk_install(apk_path)

    def clean(self):
        """clean implement's resource file"""
        busybox_obj.clean()

        cmd = "uninstall %s" % (self.PKG_NAME)
        print g_common_obj.adb_cmd_common(cmd)

    def launch(self):
        """launch app"""
        g_common_obj.launch_app_am(self.PKG_NAME, self.MAIN_ACTIVITY)

    def launch_sub_activity(self, activity):
        cmdstr = "am start -n %s/%s" % (self.PKG_NAME, activity)
        output = g_common_obj.adb_cmd_capture_msg(cmdstr)
        assert 'Warning' not in output,\
            "[FAILURE] Failed launch\n%s" % (output)

    def get_pid(self):
        cmd = "busybox pidof %s" % (self.PKG_NAME)
        msg = busybox_obj.adb_busybox_cmd(cmd)
        assert msg, "[FAILURE] Failed get process id"
        print "[Debug] pid:%s" % (msg)
        return msg

    def _run_test(self, activity='', count=1, step_time=60):
        for i in range(1, count + 1):
            print "[Debug] run_compressed_texture count:%s" % (i)
            mark_time = logcat.get_device_time_mark()
            self.launch_sub_activity(activity)
            #pid = self.get_pid()
            start_time = time.time()
            while time.time() - start_time <= step_time:
                time.sleep(1)
                _, current_activity = common.get_current_focus_window()
                assert current_activity == activity,\
                    "[FAILURE] unexpected activity %s" % (current_activity)
                self.device().click()
            g_common_obj.assert_exp_happens()

    def run_compressed_texture(self, count=1, step_time=60):
        self._run_test(self.CompressedTextureActivity, count, step_time)

    def run_cube_map(self, count=1, step_time=60):
        self._run_test(self.CubeMapActivity, count, step_time)

    def run_frame_buffer_object(self, count=1, step_time=60):
        self._run_test(self.FrameBufferObjectActivity, count, step_time)

    def run_gl_surface_view(self, count=1, step_time=60):
        self._run_test(self.GLSurfaceViewActivity, count, step_time)

    def run_kube(self, count=1, step_time=60):
        self._run_test(self.KubeActivity, count, step_time)

    def run_matrix_palette_skinning(self, count=1, step_time=60):
        self._run_test(self.MatrixPaletteActivity, count, step_time)

    def run_opengles20(self, count=1, step_time=60):
        self._run_test(self.GLES20Activity, count, step_time)

    def run_sprite_text(self, count=1, step_time=60):
        self._run_test(self.SpriteTextActivity, count, step_time)

    def run_textured_triangle(self, count=1, step_time=60):
        self._run_test(self.TriangleActivity, count, step_time)

    def run_touch_rotate(self, count=1, step_time=60):
        self._run_test(self.TouchRotateActivity, count, step_time)
