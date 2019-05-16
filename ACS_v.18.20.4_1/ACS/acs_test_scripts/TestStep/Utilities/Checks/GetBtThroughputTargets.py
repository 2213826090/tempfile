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

:summary: This file implements a Test Step for retrieving the bluetooth throughput targets
:author: jfranchx
:since 13/03/2015
:organization: INTEL NDG
"""

from Core.TestStep.TestStepBase import TestStepBase
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
from Device.DeviceManager import DeviceManager


class GetBtThroughputTargets(TestStepBase):

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        protocol = self._pars.bt_protocol
        device = DeviceManager().get_device(self._pars.device)

        # Read the throughput targets
        throughput_targets = ConfigsParser("BT_Throughput_Targets").parse_bt_targets(device.get_phone_model(), protocol)

        if self._pars.direction in "up":
            throughput_target = throughput_targets.ul_target
            throughput_failure = throughput_targets.ul_failure
        else:
            throughput_target = throughput_targets.dl_target
            throughput_failure = throughput_targets.dl_failure

        context.set_info(self._pars.target_throughput+":TARGET_VALUE", throughput_target.value)
        context.set_info(self._pars.target_throughput+":TARGET_UNIT", str(throughput_target.unit))
        context.set_info(self._pars.target_throughput+":FAILURE_VALUE", throughput_failure.value)
        context.set_info(self._pars.target_throughput+":FAILURE_UNIT", str(throughput_failure.unit))