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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.

:summary: This file implements a Test Step for retrieving the wifi throughput targets
:author: jfranchx
:since 05/01/2015
:organization: INTEL QCTV
"""

from Core.TestStep.TestStepBase import TestStepBase
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
from Device.DeviceManager import DeviceManager


class GetWifiConnectionTarget(TestStepBase):

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        device = DeviceManager().get_device(self._pars.device)
        # Read the connection targets
        connection_targets = ConfigsParser("Wifi_Connection_Targets").\
            parse_wifi_connection_targets(device.get_phone_model())

        context.set_info(self._pars.target_connection+":TARGET_VALUE", connection_targets.connection_target_value)
        context.set_info(self._pars.target_connection+":TARGET_UNIT", connection_targets.connection_unit)
        context.set_info(self._pars.target_connection+":FAILURE_VALUE", connection_targets.connection_failure_value)
        context.set_info(self._pars.target_connection+":FAILURE_UNIT", connection_targets.connection_unit)

