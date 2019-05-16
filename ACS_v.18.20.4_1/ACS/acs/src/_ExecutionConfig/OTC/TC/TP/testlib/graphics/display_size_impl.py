# -*- coding: utf-8 -*-

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.common import launch_settings_am, multi_display
from testlib.common.common import g_common_obj
from testlib.graphics.compare_pic_impl import compare_pic
from time import sleep


class DisplaySizeImpl(UIATestBase):

    _d = g_common_obj.get_device()

    def setUp(self):
        super(DisplaySizeImpl, self).setUp()
        g_common_obj.root_on_device()

    def tearDown(self):
        super(DisplaySizeImpl, self).tearDown()
        g_common_obj.back_home()

    def launch_display_size_menu(self):
        launch_settings_am('DisplaySettingsActivity')
        assert self._d(textContains="Display size").exists, 'Fail to launch display option.'

        if self._d(text="Advanced").exists: self._d(text="Advanced").click.wait()
        if self._d(text="Display size").exists: self._d(text="Display size").click.wait()
        assert self._d(text="Preview").exists, 'Fail to enter display size menu.'

    def change_display_size_and_check(self, value='Default'):
        current_value = self._d(resourceId='com.android.settings:id/current_label').info['text']
        # not capture wifi area
        before_change = multi_display.get_screenshot_multidisplay_croped(0, 100, 100, 200, 200)
        self._d(className='android.widget.RadioButton', description=value).click()
        sleep(3)
        # not capture wifi area
        after_change = multi_display.get_screenshot_multidisplay_croped(0, 100, 100, 200, 200)
        rms = compare_pic.compare_pic(before_change, after_change)
        if value == current_value:
            assert rms == 0, "Display size not same."
        else:
            assert rms > 0, "Display size not take effect."
        return True

    def set_displaysize_to_default(self):
        self.launch_display_size_menu()
        self._d(className='android.widget.RadioButton', description='Default').click()
        self._d.press.back() * 3