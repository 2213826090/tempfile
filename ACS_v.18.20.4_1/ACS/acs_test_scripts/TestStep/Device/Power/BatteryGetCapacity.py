"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file implements the battery get capacity test step
:since 01/12/2014
:author: jfranchx
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase


class BatteryGetCapacity(DeviceTestStepBase):
    """
    Implements the battery get capacity test step
    Attributes:
        DEVICE (string): @see DeviceTestStepBase
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        # gets handle to the API
        self._em_api = self._device.get_uecmd("EnergyManagement")

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        self._logger.info("Getting the current battery capacity...")

        # Where the information will be stored into the context?
        variable_to_set = self._pars.save_battery_capacity_as

        # Get battery capacity
        result = self._em_api.get_msic_registers()
        battery_capacity = result["BATTERY"]["CAPACITY"][0]

        # We have the value, let's save it into the context
        context.set_info(variable_to_set, battery_capacity)
        self.ts_verdict_msg = "VERDICT: %s stored as {0}".format(battery_capacity) % variable_to_set
        self._logger.debug(self.ts_verdict_msg)