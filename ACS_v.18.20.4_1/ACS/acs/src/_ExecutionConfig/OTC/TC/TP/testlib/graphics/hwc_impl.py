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
#    and approved by Intel in writing.
"""
@summary: DisplayCableSwitchImpl class
@since: 03/13/2015
@author: Ding, JunnanX (junnanx.ding@intel.com)
"""

import re
import time
from testlib.util.common import g_common_obj
from testlib.graphics.common import logcat, ACCESSIBILITY, launch_settings_am, adb32

COLOR_INVERSION = 'Color inversion'
COLOR_CORRECTION = 'Color correction'
CORRECTION_MODE = 'Correction mode'
CORRECTION_MODE_LIST = ['Deuteranomaly','Protanomaly','Tritanomaly']


class HWComposerImpl(object):

    """ Hardware Composer Implement"""

    option_overlay = 'overlay'

    def __init__(self):
        self.d = g_common_obj.get_device()

    def _set_option(self, name, value):
        time_mark = logcat.get_device_time_mark()
        cmd = "service call hwc.info 3 s16 '%s' s16 '%s'" % (name, value)
        g_common_obj.adb_cmd_capture_msg(repr(cmd))
        e_msg = logcat.get_device_log(time_mark, 'Parcel:F hwc:W *:S')
        assert len(e_msg) == 0,\
            "[FAILURE] %s \n%s" % (cmd, e_msg)
        i_msg = logcat.get_device_log(time_mark, 'hwc:I *:S')
        catch_log = re.findall('.+Changed option %s.+' % (name), i_msg)
        print '-' * 60 + '\n'
        print catch_log
        print '-' * 60 + '\n'
        assert catch_log,\
            "[FAILURE] verify option failed"

    def set_overlay(self, value):
        self._set_option(self.option_overlay, value)

    def set_color_inversion_ui(self, switch='on'):
        launch_settings_am(ACCESSIBILITY)
        time.sleep(2)
        self.d().scroll.to(text=COLOR_INVERSION)
        locator = self.d(text=COLOR_INVERSION).right(className="android.widget.Switch")
        status = locator.checked
        if switch.upper() == 'ON':
            if not status:
                locator.click()
        elif switch.upper() == 'OFF':
            if status:
                locator.click()
        adb32.adb_shell_stop()
        adb32.adb_shell_start()

    def set_color_corretion_ui(self, mode=''):
        launch_settings_am(ACCESSIBILITY)
        time.sleep(2)
        self.d().scroll.to(text=COLOR_CORRECTION)
        switcher = self.d(className="android.widget.Switch")
        status = self.d(text=COLOR_CORRECTION).down().info['text'].lower()
        if mode == '':
            if status.lower() != 'off':
                self.d(text=COLOR_CORRECTION).click.wait()
                time.sleep(1)
                switcher.click()
                self.d.press.back()
        else:
            if status.lower() != 'on':
                self.d(text=COLOR_CORRECTION).click.wait()
                time.sleep(1)
                switcher.click()
                time.sleep(1)
                self.d(text=CORRECTION_MODE).click.wait()
                self.d(textStartsWith=mode).click()
            else:
                self.d(text=COLOR_CORRECTION).click.wait()
                time.sleep(1)
                self.d(text=CORRECTION_MODE).click.wait()
                self.d(textStartsWith=mode).click()
            self.d.press.back()
        adb32.adb_shell_stop()
        adb32.adb_shell_start()

hwc_impl = HWComposerImpl()