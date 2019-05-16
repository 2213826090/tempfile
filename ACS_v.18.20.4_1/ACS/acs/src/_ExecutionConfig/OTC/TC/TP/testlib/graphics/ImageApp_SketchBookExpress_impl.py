# Copyright (C) 2015  Jin, YingjunX <yingjunx.jin@intel.com>
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
@summary: Class for ImageApp SketchBookExpress
@since: 05/26/2015
@author: Xiangyi Zhao
'''

import os
import time
from testlib.util.common import g_common_obj
from testlib.graphics.common import get_current_focus_window

class Locator(object):
    """
    locator
    """

    def __init__(self, device):
        self._device = device

    @property
    def wait_exist(self):
        """ wait until the ui object exist """
        def _wait(uiobj, timeout=10):
            return uiobj.wait("exists", timeout * 500)
        return _wait

    @property
    def tour(self):
        return self._device(
            resourceId="android:id/content")\
        .child(className="android.widget.RelativeLayout")

    @property
    def menu(self):
        return self._device(
            resourceId="android:id/content")\
        .child(className="android.widget.LinearLayout")\
        .child(className="android.view.View", index=5)

    @property
    def menu_m(self):
        return self._device(
            resourceId="android:id/content")\
        .child(className="android.widget.LinearLayout")\
        .child(className="android.view.ViewGroup", index=5)

    @property
    def tool_btn(self):
        return self._device(
            resourceId="android:id/content")\
        .child(className="android.widget.LinearLayout")\
        .child(className="android.view.View", index=1)

    @property
    def tool_btn_m(self):
        return self._device(
            resourceId="android:id/content")\
        .child(className="android.widget.LinearLayout")\
        .child(className="android.view.ViewGroup", index=1)

class SketchBookX:
    '''
    classdocs
    '''

    pkg_name = "com.adsk.sketchbookhdexpress"
    activity_name = "com.adsk.sketchbook.SketchBook"

    def __init__(self):
        self._device = g_common_obj.get_device()
        self._locator = Locator(self._device)

    def launch_app_am(self):
        """ Launch SketchBookX via adb am command
        """
        print "Launch SketchBookX by adb am"
        g_common_obj.launch_app_am(\
            SketchBookX.pkg_name, SketchBookX.activity_name)
        time.sleep(2)
        if self._device(text='I accept').exists:
            self._device(text='I accept').click()
        time.sleep(4)
        self._locator.tour.click()
        time.sleep(1)
        self._locator.tour.click()
        time.sleep(1)
        self._locator.tour.click()
        time.sleep(1)
        self._locator.tour.click()
        time.sleep(1)

    @staticmethod
    def stop_app_am():
        """ Stop SketchBookX via adb am command
        """
        print "Stop SketchBookX by adb am"
        g_common_obj.stop_app_am(SketchBookX.pkg_name)

    @staticmethod
    def set_workaround():
        """ Reset the workaround to let the picture display after push
        """
        cmd = 'am broadcast -a android.intent.action.MEDIA_MOUNTED \
               -d file:///sdcard/Pictures'
        g_common_obj.adb_cmd(cmd)

    def choose_menu(self):
        """ click menu button and check if pop-up window
        """
        print "catch screenshot sketchbook1.png"
        self._device.screenshot("sketchbook1.png")
        time.sleep(1)
        if self._locator.menu.exists:
            self._locator.menu.click.wait()
        elif self._locator.menu_m.exists:
            self._locator.menu_m.click.wait()
        else:
            assert False, "Can not found memu button"
        time.sleep(2)
        print "catch screenshot sketchbook2.png"
        self._device.screenshot("sketchbook2.png")
        time.sleep(1)
        assert (os.system('diff sketchbook1.png sketchbook2.png | grep "differ"') == 0), "click menu button fail"

    def choose_quicktour(self):
        """ choose Quick Tour button and check function
        """
        if self._locator.tool_btn.exists:
            self._locator.tool_btn.click.wait()
            x = (self._locator.tool_btn.info.get("bounds").get("left") + self._locator.tool_btn.info.get("bounds").get("right")) / 2
            rule_y = self._locator.tool_btn.info.get("bounds").get("bottom") + 5
        elif self._locator.tool_btn_m.exists:
            self._locator.tool_btn_m.click.wait()
            x = (self._locator.tool_btn_m.info.get("bounds").get("left") + self._locator.tool_btn_m.info.get("bounds").get("right")) / 2
            rule_y = self._locator.tool_btn_m.info.get("bounds").get("bottom") + 5
        else:
            assert False, "Can not found tool button"
        time.sleep(2)
        count = 0
        while (count < 30):
            self._device.click(x, rule_y)
            time.sleep(2)
            _, ACTname = get_current_focus_window()
            if ACTname == "com.adsk.sketchbook.SketchBook":
                rule_y = rule_y + 20
                count = count + 1
                print "com.adsk.sketchbook.SketchBook"
                print count
                continue
            if ACTname == "com.adsk.sketchbook.helpinfo.PreferencesSetting":
                rule_y = rule_y + 20
                self._device.press.back()
                if self._locator.tool_btn.exists:
                    self._locator.tool_btn.click.wait()
                elif self._locator.tool_btn_m.exists:
                    self._locator.tool_btn_m.click.wait()
                else:
                    assert False, "Can not found tool button"
                count = count + 1
                print "com.adsk.sketchbook.helpinfo.PreferencesSetting"
                print count
                continue
            if ACTname == "com.adsk.sketchbook.SketchBookStatement":
                assert False, "can not found quick tour button"
                break
            if ACTname == "com.adsk.sketchbook.SketchBookTour":
                break
        self._locator.tour.click()
        time.sleep(1)
        self._locator.tour.click()
        time.sleep(1)
        self._locator.tour.click()
        time.sleep(1)
        self._locator.tour.click()
        time.sleep(1)

    def scroll(self):
        """ scroll up and down by function zoom in and zoom out
        """
        print "Zoom in"
        x = self._device.info.get("displayWidth")
        y = self._device.info.get("displayHeight")
        self._device().gesture((x / 2, y / 2), (x / 2, y / 2)).to((x / 8, y / 8), (x * 7 / 8, y * 7 / 8))
        time.sleep(2)
        print "catch screenshot sketchbook3.png"
        self._device.screenshot("sketchbook3.png")
        time.sleep(1)
        print "Zoom out"
        self._device().gesture((x / 8, y / 8), (x * 7 / 8, y * 7 / 8)).to((x / 2, y / 2), (x / 2, y / 2))
        time.sleep(2)
        print "catch screenshot sketchbook4.png"
        self._device.screenshot("sketchbook4.png")
        time.sleep(1)
        assert (os.system('diff sketchbook3.png sketchbook4.png | grep "differ"') == 0), "scroll up and down fail"

    def uninstall_app(self):
        """ uninstall SketchBookX
        """
        print "uninstall SketchBookX"
        cmd = 'uninstall %s' % SketchBookX.pkg_name
        g_common_obj.adb_cmd_common(cmd)
