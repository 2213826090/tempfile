"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

:organization: INTEL OTC Android
:summary: This file implements a Test Step that sets a lock screen on the device
:since:3/12/2015
:author: mmaraci
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException


class EnableLockScreen(DeviceTestStepBase):
    """
    Enable the lock screen
    """
    PY_UIAUTOMATOR = "PyUiAutomator"

    def __init__(self, tc_conf, global_conf, ts_conf, factory=None):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self._applications_launcher = self._device.get_uecmd(self.PY_UIAUTOMATOR)

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        current_lockscreen_value = self._applications_launcher.check_screenlock()
        if current_lockscreen_value in ['Pattern', 'PIN', 'Password']:
            msg = 'This type of lockscreen setting has not been implemented: {0}. Please change this locking type ' \
                  'manually'.format(current_lockscreen_value)
            raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, msg)

        if not current_lockscreen_value == self._pars.lockscreen_type:
            self._applications_launcher.open_screenlock_menu()
            self._applications_launcher.set_lockscreen(self._pars.lockscreen_type)
            final_lockscreen_value = self._applications_launcher.check_screenlock()
            if final_lockscreen_value != self._pars.lockscreen_type:
                msg = "The Screen lock value is {0} instead of the intended one: {1}".format(final_lockscreen_value,
                                                                                             self._pars.lockscreen_type)
                raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, msg)
