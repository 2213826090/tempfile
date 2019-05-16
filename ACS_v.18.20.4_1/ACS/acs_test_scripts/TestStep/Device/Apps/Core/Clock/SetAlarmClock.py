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
:summary: This file implements a Test Step that sets an alarm on the device
:since:21/04/2015
:author: mmaraci
"""

from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Utilities.ApplicationUtilities import ApplicationUtilities

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase


class SetAlarmClock(DeviceTestStepBase):
    """
    Implements the test step to set an alarm clock on the device
    """
    def __init__(self, tc_conf, global_conf, ts_conf, factory=None):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self._alarm_clock = ApplicationUtilities(self._device)

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        DeviceTestStepBase.run(self, context)

        now_hour, now_minutes = self._alarm_clock.get_system_clock()
        if not now_hour or not now_minutes:
            msg = "Could not get system time"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        alarm_minutes = now_minutes + int(self._pars.alarm_minutes_from_now)
        alarm_hour = now_hour + int(self._pars.alarm_hour_from_now)

        alarm_minutes = alarm_minutes % 60
        alarm_hour = alarm_hour + alarm_minutes / 60
        alarm_hour = alarm_hour % 24

        self._alarm_clock.force_stop_clock()

        if not self._alarm_clock.set_alarm_clock(self._pars.alarm_message, alarm_hour, alarm_minutes):
            msg = "Could not properly set the alarm on the device"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        context.set_info(self._pars.save_hour, alarm_hour)
        context.set_info(self._pars.save_minutes, alarm_minutes)