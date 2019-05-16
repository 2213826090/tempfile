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
"""
@summary: ChromeExtendImpl class
@since: 04/22/2015
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""

import re
import time
import filecmp

from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.common import get_screenshot_region
from testlib.graphics.common import logcat, remove_temp_file
from testlib.graphics.common import osversion


class ImageProcessingImpl(object):

    """ ImageProcessingExtendImpl """

    CONFIG_FILE = 'tests.common.imageprocessing.conf'
    PKG_NAME = "com.android.rs.image"
    MAIN_ACTIVITY = "com.android.rs.image.ImageProcessingActivity"

    SECTION_LEVELS_VEC3_RELAXED = 'Levels Vec3 Relaxed'
    OPTIONS_LEVELS_VEC3_RELAXED = ['Saturation', 'In Black', 'Out Black', 'Out White']

    SECTION_LEVELS_VEC4_RELAXED = 'Levels Vec4 Relaxed'
    OPTIONS_LEVELS_VEC4_RELAXED = ['Saturation', 'In Black', 'Out Black', 'Out White']

    class HomeUI(object):

        """Home UI"""

        def __init__(self, device):
            self.device = device

        @property
        def image_display(self):
            """scroll to image region and return ui object"""
            self.device(scrollable=True).scroll.toBeginning(steps=20)
            time.sleep(3)
            return self.device(resourceId='com.android.rs.image:id/display')

        def set_selection(self, name):
            """change setting's section ui"""
            print "[Debug] set_selection %s" % (name)
            if self.device().scroll.to(text=name):
                print "[Debug] skip set selection %s" % (name)
                return
            resourceId = "com.android.rs.image:id/filterselection"
            self.device(scrollable=True).scroll.to(resourceId=resourceId)
            self.device(resourceId=resourceId).click()
            self.device(className="android.widget.ListView") \
                .child_by_text(
                name,
                allow_scroll_search=True,
                className="android.widget.TextView"
            ).click()
            assert self.device(text=name).exists, \
                "[FAILURE] Failed set_selection %s" % (name)

        def set_option(self, name, percent):
            """change option SeekBar value"""
            print "[Debug] set_option %s %s" % (name, percent)
            pre_image = get_screenshot_region(self.image_display.bounds)
            self.device().scroll.to(text=name)
            bounds = self.device(text=name)\
                .down(className='android.widget.SeekBar').info['bounds']
            pre_bar = get_screenshot_region(bounds)

            self.device.click(bounds['left'] + (bounds['right'] - bounds['left']) * (percent / 100.00),
                              (bounds['top'] + bounds['bottom']) / 2)
            time.sleep(1)
            end_bar = get_screenshot_region(bounds)
            g_common_obj.assert_exp_happens()
            end_image = get_screenshot_region(self.image_display.bounds)
            g_common_obj.assert_exp_happens()
            remove_temp_file(pre_bar)
            remove_temp_file(end_bar)
            remove_temp_file(pre_image)
            remove_temp_file(end_image)

    def __init__(self):
        self.device = g_common_obj.get_device()
        self.home = ImageProcessingImpl.HomeUI(self.device)

        self.configer = TestConfig()
        self.config = self.configer.read(self.CONFIG_FILE, "ImageProcessingExtendImpl")
        config_handle = ConfigHandle()
        self.config["artifactory_location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat', 'sys.conf')
        self.arti = Artifactory(self.config.get('artifactory_location'))

    def setup(self):
        """setup implement's resource file"""
        version_array = osversion.get_android_version()
        androidversion = version_array[0]
        if androidversion == 7:
            print "osversion is N"
            apk_path = self.arti.get(self.config.get("apk"))
        elif androidversion == 6:
            print "osversion is M"
            apk_path = self.arti.get(self.config.get("apk"))
        elif androidversion == 5:
            print "osversion is L"
            apk_path = self.arti.get(self.config.get("apk_l"))
        else:
            print "osversion is %s" % (androidversion)
            apk_path = self.arti.get(self.config.get("apk_l"))
        cmd = "install -r %s" % (apk_path)
        print g_common_obj.adb_cmd_common(cmd, 300)

    def clean(self):
        """clean implement's resource file"""
        cmd = "uninstall %s" % (self.PKG_NAME)
        print g_common_obj.adb_cmd_common(cmd)

    def launch(self):
        """launch app"""
        g_common_obj.launch_app_am(self.PKG_NAME, self.MAIN_ACTIVITY)
        time.sleep(3)

    def change_levels_vec3_relaxed_settings(self, options=None):
        """change levels_vec3_relaxed options"""
        if options:
            keys = [each[0] for each in options]
            for key in keys:
                assert key in self.OPTIONS_LEVELS_VEC3_RELAXED
        self._change_settings(self.SECTION_LEVELS_VEC3_RELAXED, options)

    def change_levels_vec4_relaxed_settings(self, options=None):
        """change levels_vec4_relaxed options"""
        if options:
            keys = [each[0] for each in options]
            for key in keys:
                assert key in self.OPTIONS_LEVELS_VEC4_RELAXED
        self._change_settings(self.SECTION_LEVELS_VEC4_RELAXED, options)

    def _change_settings(self, section, options=None):
        """change setting's section and set options"""
        print "[Debug] change to %s" % (section)
        self.home.set_selection(section)
        if options:
            for option in options:
                assert isinstance(option, tuple)
                key, value = option
                self.home.set_option(key, value)

    def verify_log_gen_gpu(self, time_mark):
        """verify keywords GPU in log"""
        logs = logcat.get_device_log(time_mark, "RenderScript:V *:S")
        assert len(logs) > 0, \
            "[FAILURE] Failed Catch logs"
        msg = re.findall(r'.* GPU*', logs)
        assert len(msg) > 0, \
            "[FAILURE] Failed verify_log_gen_gpu"
        print "[Debug] %s" % (''.join(msg))

    def verify_log_core_cpu(self, time_mark):
        """verify keywords CPU in log"""
        logs = logcat.get_device_log(time_mark, "RenderScript:V *:S")
        assert len(logs) > 0, \
            "[FAILURE] Failed Catch logs"
        msg = re.findall(r'.* CPU*', logs)
        assert len(msg) > 0, \
            "[FAILURE] Failed verify_log_core_cpu"
        print "[Debug] %s" % (''.join(msg))
