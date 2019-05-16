"""

:copyright: (c)Copyright 2016, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL OTC ANDROID CORE QA
:summary: This script implements the Common PyUiAutomator actions
:since: 07/01/2016
:author: apalko
"""
import time

from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Interface.Ui.IPyUiAutomator import IPyUiAutomator
from uiautomator import Device, Adb


class PyUiAutomator(BaseV2, IPyUiAutomator):
    """
    Class that implements the Common PyUiAutomator actions
    """
    HSP_PROFILE = "Phone audio"
    A2DP_PROFILE = "Media audio"
    HID_PROFILE = "Input device"
    PAN_PROFILE = "Internet access"

    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)
        IPyUiAutomator.__init__(self, device)
        self.d = Device(device.retrieve_serial_number())

    def press_home(self):
        """
        Used to press the home button
        """
        self._logger.info("Press home")
        self.d.press.home()

    def screen_on(self):
        """
        Used to set the screen on
        """
        self._logger.info("Screen on")
        self.d.screen.on()

    def dismiss_soft_keyboard(self):
        """
        Used to dismiss the soft keyboard on the screen

        """
        time.sleep(1)
        if "mInputShown=true" in \
                Adb(self._device.retrieve_serial_number()).cmd("shell dumpsys input_method").communicate()[
                    0].decode("utf-8"):
            self.d.press.back()
            time.sleep(1)
