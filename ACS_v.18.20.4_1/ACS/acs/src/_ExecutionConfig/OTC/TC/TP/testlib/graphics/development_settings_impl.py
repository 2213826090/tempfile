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
@summary: DevelopmentSettingsImpl class
@since: 04/29/2015
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""

import time
# import threading
from testlib.util.common import g_common_obj
from testlib.graphics.common import multi_display
from testlib.androidframework.common import EnvironmentUtils


class DevelopmentSettingsImpl(object):

    """ DevelopmentSettingsImpl """

    PKG_NAME = "com.android.settings"
    ACTIVITY = ".DevelopmentSettings"

    class Options(object):

        """development options"""

        def __init__(self, device):
            self.device = device

        def search_switch_item(self, name):
            self.device(scrollable=True).scroll.to(text=name)
            assert self.device(text=name), "%s not found" % name
            y = self.device.info["displayHeight"]
            x = self.device.info["displayWidth"]
            self.device.swipe(x / 2, y / 2 + 50, x / 2, y / 2, steps=50)
            return self.device(text=name, resourceId='android:id/title')\
                .right(className="android.widget.Switch")

    def __init__(self):
        self.device = g_common_obj.get_device()
        self._options = DevelopmentSettingsImpl.Options(self.device)

    def launch(self):
        """launch APP"""
        cmdstr = "am start -S -n %s/%s" % (self.PKG_NAME, self.ACTIVITY)
        return g_common_obj.adb_cmd(cmdstr)

    def _set_switch_option(self, name, switch='ON'):
        """set switch option"""
        uiobj = self._options.search_switch_item(name)
        old_state = uiobj.info['text']
        print "[Debug] %s state before:%s after:%s"\
            % (name, old_state, switch)
        if old_state != switch:
            uiobj.click()
        uiobj = self._options.search_switch_item(name)
        changed_state = uiobj.info['text']
        assert switch == changed_state,\
            "[FAILURE] Failed change option set:%s"\
            % (switch)

    def _get_switch_option(self, name):
        """get switch option"""
        uiobj = self._options.search_switch_item(name)
        return uiobj.info['text']

    def set_disable_hw_overlays(self, switch='ON'):
        """set_disable_hw_overlays"""
        self.launch()
        self._set_switch_option('Disable HW overlays', switch)
        time.sleep(1)
        self.device.press.back()

    def get_disable_hw_overlays(self):
        """get_disable_hw_overlays"""
        self.launch()
        state = self._get_switch_option('Disable HW overlays')
        print "[Debug] get_disable_hw_overlays state:%s" % (state)
        time.sleep(1)
        self.device.press.back()
        return state

    def set_input_options(self, switch='ON'):
        """set Show taps & Pointer location"""
        if EnvironmentUtils.get_android_version() == "M":
            _opts = ['Show touches', 'Pointer location']
        else:
            _opts = ['Show taps', 'Pointer location']
        self.launch()
        for op in _opts:
            self._set_switch_option(op, switch)
            time.sleep(1)
        self.device.press.back()


class Debug_GPUoverdraw(DevelopmentSettingsImpl):

    def __init__(self):
        super(Debug_GPUoverdraw, self).__init__()
        self._GPUoverdraw_name = 'Debug GPU overdraw'

    def _show_GPU_overdraw_list(self):
        self.launch()
        self._options.search_switch_item(self._GPUoverdraw_name)
        self.device(text=self._GPUoverdraw_name).click.wait()

    def _get_option_location(self):
        return self.device(text=self._GPUoverdraw_name).bounds.values()

    def _perform_long_click(self, x, y):
        long_click_event = "input touchscreen swipe %s %s %s %s 5000" % (x, y, x, y)
        g_common_obj.adb_cmd_capture_msg(long_click_event)

    def turn_off_GPUoverdraw(self):
        self._show_GPU_overdraw_list()
        return self.device(text="Off").click.wait()

    def enable_show_overdraw_areas(self):
        self._show_GPU_overdraw_list()
        return self.device(text="Show overdraw areas").click.wait()

    def enable_show_areas_for_deuteranomaly(self):
        self._show_GPU_overdraw_list()
        return self.device(text="Show areas for Deuteranomaly").click.wait()

    def turn_off_GPUoverdraw_get_screenshot(self):
        self.turn_off_GPUoverdraw()
        top, left, right, bottom = self._get_option_location()
        self._perform_long_click(right, bottom) # Perform long click(5 sec) event.
        return multi_display.get_screenshot_multidisplay_croped(0, 0, top, left, bottom) # Start screenshot

    def enable_show_overdraw_areas_get_screenshot(self):
        self.enable_show_overdraw_areas()
        top, left, right, bottom = self._get_option_location()
        self._perform_long_click(right, bottom) # Perform long click(5 sec) event.
        return multi_display.get_screenshot_multidisplay_croped(0, 0, top, left, bottom) # Start screenshot

    def enable_show_areas_for_deuteranomaly_get_screenshot(self):
        self.enable_show_areas_for_deuteranomaly()
        top, left, right, bottom = self._get_option_location()
        self._perform_long_click(right, bottom) # Perform long click(5 sec) event.
        return multi_display.get_screenshot_multidisplay_croped(0, 0, top, left, bottom) # Start screenshot
