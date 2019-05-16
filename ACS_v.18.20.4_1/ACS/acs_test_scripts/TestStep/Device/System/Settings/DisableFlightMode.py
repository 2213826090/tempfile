"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG PSI
:summary: This file implements a Test Step to disable the flight mode
:since:10/01/2014
:author: ssavrim
"""
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException


class DisableFlightMode(DeviceTestStepBase):
    """
    Enable the flight mode
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        if self._device.is_capable('modem or modem_LTE'):
            # Load Networking ue command
            networking_api = self._device.get_uecmd("Networking")

            networking_api.set_flight_mode(False)
            if networking_api.get_flight_mode():
                error_msg = "Fail to disable flight mode on '{0}' !".format(self._pars.device)
                raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)
        else:
            self._logger.warning('this device does not have modem, no flight mode to deactivate!')
