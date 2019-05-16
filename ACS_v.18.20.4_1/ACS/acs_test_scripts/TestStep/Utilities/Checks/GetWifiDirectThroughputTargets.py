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

:summary: This file implements a Test Step for retrieving the wifi direct throughput targets
:author: jfranchx
:since 19/01/2015
:organization: INTEL NDG
"""

from Core.TestStep.TestStepBase import TestStepBase
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
from acs_test_scripts.Utilities.ThroughputMeasure import ThroughputMeasure
from Device.DeviceManager import DeviceManager


class GetWifiDirectThroughputTargets(TestStepBase):

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        device = DeviceManager().get_device(self._pars.device)

        throughput_target_unit =  ThroughputMeasure.KBPS_UNIT
        throughput_failure_unit = ThroughputMeasure.KBPS_UNIT

        # Read the throughput targets
        throughput_targets = ConfigsParser("Wifi_Direct_Throughput_Targets").\
            parse_wifi_direct_targets(device.get_phone_model(), self._pars.iperf_protocol, self._pars.bandwidth)

        if self._pars.direction in "up":
            throughput_target_value = throughput_targets.ul_target_value
            throughput_failure_value = throughput_targets.ul_failure_value
        else:
            throughput_target_value = throughput_targets.dl_target_value
            throughput_failure_value = throughput_targets.dl_failure_value

        context.set_info(self._pars.target_throughput+":TARGET_VALUE", throughput_target_value)
        context.set_info(self._pars.target_throughput+":TARGET_UNIT", throughput_target_unit)
        context.set_info(self._pars.target_throughput+":FAILURE_VALUE", throughput_failure_value)
        context.set_info(self._pars.target_throughput+":FAILURE_UNIT", throughput_failure_unit)
