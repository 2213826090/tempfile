#Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
#Intel Corporation All Rights Reserved.

#The source code contained or described herein and
#all documents related to the source code ("Material") are owned by
#Intel Corporation or its suppliers or licensors.

#Title to the Material remains with Intel Corporation or
#its suppliers and licensors.
#The Material contains trade secrets and proprietary and
#confidential information of Intel or its suppliers and licensors.
#The Material is protected by worldwide copyright and
#trade secret laws and treaty provisions.
#No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
#or disclosed in any way without Intel's prior express written permission.
#No license under any patent, copyright, trade secret or
#other intellectual property right is granted to
#or conferred upon you by disclosure or delivery of the Materials,
#either expressly, by implication, inducement, estoppel or otherwise.

#Any license under such intellectual property rights must be express
#and approved by Intel in writing.

"""
@summary: This file implements for test smoothness
@since: 06/09/2014
@author: Grace Yi (gracex.yi@intel.com)
"""

from testlib.util.common import g_common_obj
import time

class ScreenSmoothnessImpl:
    """
    @summary: The basic function to screen smoothness
    """

#--------- begin locator -------------
    class Locator(object):
        """
            Helper for locator UI Object
        """

        def __init__(self, device):
            self.d = device

        @property
        def btn_active(self):
            """ UI button active """
            return self.d(\
                resourceId="com.google.android.googlequicksearchbox:id/active")

        @property
        def btn_fling_panel(self):
            """ UI button apps """
            return self.d(resourceId=\
             "com.google.android.googlequicksearchbox:id/apps_customize_pane_content",\
             scrollable="true")

        @property
        def btn_home_active(self):
            """ UI button homescreen active """
            return self.d(\
                resourceId="com.google.android.googlequicksearchbox:id/page_indicator")

        @property
        def btn_widgets(self):
            """ UI button Widgets """
            return self.d(text="Widgets")

        @property
        def btn_inactive(self):
            """ UI button inactive """
            return self.d(resourceId=\
                "com.google.android.googlequicksearchbox:id/inactive")

    def __init__ (self, cfg):
        self._device = g_common_obj.get_device()
        self._locator = ScreenSmoothnessImpl.Locator(self._device)
        self.cfg = cfg

    def fling_forward(self):
        """
        @summary: horizontal and forward fling the launcher screen
        @return: None
        """
        s_locate = self.get_widget_locate()
        self._locator.btn_fling_panel.fling.horiz.forward()
        e_locate = self.get_widget_locate()
        print "Fling forward:" , e_locate
        assert s_locate != e_locate, "The widget is not moved."

    def fling_backward(self):
        """
        @summary: horizontal and backward fling the launcher screen
        @return: None
        """
        s_locate = self.get_widget_locate()
        self._locator.btn_fling_panel.fling.horiz.backward()
        e_locate = self.get_widget_locate()
        print "Fling back:" , e_locate
        assert s_locate != e_locate, "The widget is not moved."

    def goto_widgets(self):
        """ got to widgets """
        self._device.press.home()
        self._locator.btn_home_active.long_click()
        if not self._locator.btn_widgets.exists:
            self._locator.btn_home_active.long_click()
        self._locator.btn_widgets.click()
        assert self._locator.btn_fling_panel.exists, \
        "UI error"

    def get_widget_locate(self):
        """
        @summary: get widget locate
        """
        g_common_obj.restart_server()
        locate = self._locator.btn_active.info.get("bounds")["left"]
        return int(locate)

    def get_count(self):
        """ get widget fling count"""
        count = self._locator.btn_inactive.count
        return int(count)

    def set_orientation_n(self):
        """
        @summary: set orientation as n
        """
        self._device.orientation = "n"
