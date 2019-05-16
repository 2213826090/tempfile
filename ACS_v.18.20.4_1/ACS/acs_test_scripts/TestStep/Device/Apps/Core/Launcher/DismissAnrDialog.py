"""

:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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

:organization: INTEL OTC ANDROID QA
:description: PyUiautomator TestStep for dismissing ANR dialog(s) if any is found,
on the screen
:since: 05/20/15
:author: apalko
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from uiautomator import Device


class DismissAnrDialog(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor: instantiates PyUiAutomator Uecmd
        """

        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.ui_actions = self._device.get_uecmd("PyUiAutomator")

    def run(self, context):
        """
        PyUiautomator TestStep for dismissing any ANR dialog from the screen if exists. Useful especially
        for setup of the Python UI tests, only pressing home does not automatically clear ANR dialogs from the
        previous tests
        """

        DeviceTestStepBase.run(self, context)

        # parse params
        self.max_attempts = self._pars.max_attempts
        if not self.max_attempts > 0:
            self._raise_config_exception("MAX_ATTEMPTS must be > 0")
        self.press_home_first = self._pars.press_home_first

        # Execute the test step

        if self.press_home_first:
            self.ui_actions.press_home()

        self.ui_actions.dismiss_anr(self.max_attempts)
