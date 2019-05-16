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

:organization: INTEL MCG PSI
:summary: This file implements a Test Step to get a list of services for the BT device
:since: 26/02/2015
:author: mmaraci
"""
from acs_test_scripts.Equipment.Computer.Common.Common import GenericComputer
from acs_test_scripts.TestStep.Device.Wireless.BT.Base import BtBase
from ErrorHandling.DeviceException import DeviceException
import re

class BtListServicesSDPTool(BtBase, GenericComputer):
    """
    Implements the test step to get a list of services for the BT device
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory=None):
        """
        Constructor
        """

        BtBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        BtBase.run(self, context)
        cmd = "sdptool browse " + self._pars.name
        output = self._device.run_cmd(cmd, 20)
        control_string = "Browsing " + self._pars.name
        if self._pars.check_string and self._pars.check_string != "":
            if "'" in self._pars.check_string:
                if self._pars.check_string.replace("'", "\"") in output[1]:
                    self.ts_verdict_msg = "VERDICT: The searched service is correctly found by the sdptool: {0}".format(self._pars.check_string)
                    self._logger.debug(self.ts_verdict_msg)
            elif "Version" in self._pars.check_string:
                search1, search2 = self._pars.check_string.split("Version")
                if re.search(search1 + ".+" + search2, output[1]):
                    self.ts_verdict_msg = "VERDICT: The searched service is correctly found by the sdptool: {0}".format(self._pars.check_string)
                    self._logger.debug(self.ts_verdict_msg)
            else:
                self.ts_verdict_msg = "VERDICT: The sdptool did not find the given service listed for this device"
                raise DeviceException(DeviceException.OPERATION_FAILED, self.ts_verdict_msg)
        elif (not self._pars.check_string or self._pars.check_string == "") and control_string in output[1]:
            self.ts_verdict_msg = "VERDICT: The service list is correctly found by the sdptool: {0}".format(output[1])
            self._logger.debug(self.ts_verdict_msg)
        else:
            self.ts_verdict_msg = "VERDICT: The sdptool did not find any services listed for this device"
            raise DeviceException(DeviceException.OPERATION_FAILED, self.ts_verdict_msg)
        self._logger.debug(self.ts_verdict_msg)
