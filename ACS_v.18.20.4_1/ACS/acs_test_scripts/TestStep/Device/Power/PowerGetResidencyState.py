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

:organization: INTEL NDG
:summary: This file implements the system state parameters measurement
:since: 23/06/2014
:author: floeselx
"""

import time
from acs_test_scripts.TestStep.Device.Power.PowerBase import PowerBase
from ErrorHandling.DeviceException import DeviceException


class PowerGetResidencyState(PowerBase):
    """
    State measurement class
    """

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        PowerBase.run(self, context)

        mode = str(self._pars.mode).lower()
        unit = str(self._pars.pwr_meas_unit)

        # Where the information will be stored into the context?
        variable_to_set = self._pars.save_as

        self._logger.debug("Sleep Mode to measure = %s, unit type = %s" %(mode, unit))

        value = self._api.get_value(unit, mode)

        if value is None:
            raise DeviceException(DeviceException.INVALID_PARAMETER,
                                 "There is no %s sleep mode for this device model" %
                                 mode)

        # We have the value, let's save it into the context
        context.set_info(variable_to_set, str(value))
        self.ts_verdict_msg = "VERDICT: %s stored as {0}".format(value) % variable_to_set
        self._logger.debug(self.ts_verdict_msg)
