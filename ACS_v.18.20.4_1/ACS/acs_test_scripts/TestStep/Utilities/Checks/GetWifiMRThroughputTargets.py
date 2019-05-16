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

:summary: This file implements a Test Step for retrieving the wifi multi-role throughput targets
:author: jfranchx
:since 29/01/2015
:organization: INTEL NDG
"""

from Core.TestStep.TestStepBase import TestStepBase
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
from acs_test_scripts.Utilities.ThroughputMeasure import ThroughputMeasure
from Device.DeviceManager import DeviceManager


class GetWifiMRThroughputTargets(TestStepBase):

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        device = DeviceManager().get_device(self._pars.device)

        sta_throughput_target_unit = ThroughputMeasure.UNITS.KPS_UNIT
        sta_throughput_failure_unit = ThroughputMeasure.UNITS.KPS_UNIT
        p2p_throughput_target_unit = ThroughputMeasure.UNITS.KPS_UNIT
        p2p_throughput_failure_unit = ThroughputMeasure.UNITS.KPS_UNIT

        # Read the throughput targets
        sta_throughput_targets, p2p_throughput_targets = ConfigsParser("Wifi_MR_Throughput_Targets").\
            parse_wifi_mr_targets(device.get_phone_model(), self._pars.standard_frequency, self._pars.bandwidth,
                                  self._pars.sta_direction, self._pars.sta_iperf_protocol, self._pars.p2p_direction,
                                  self._pars.p2p_iperf_protocol)

        if self._pars.sta_direction in "up":
            sta_throughput_target_value = sta_throughput_targets.ul_target
            sta_throughput_failure_value = sta_throughput_targets.ul_failure
        else:
            sta_throughput_target_value = sta_throughput_targets.dl_target
            sta_throughput_failure_value = sta_throughput_targets.dl_failure

        if self._pars.p2p_direction in "up":
            p2p_throughput_target_value = p2p_throughput_targets.ul_target
            p2p_throughput_failure_value = p2p_throughput_targets.ul_failure
        else:
            p2p_throughput_target_value = p2p_throughput_targets.dl_target
            p2p_throughput_failure_value = p2p_throughput_targets.dl_failure

        context.set_info(self._pars.target_throughput+":STA_TARGET_VALUE", sta_throughput_target_value.value)
        context.set_info(self._pars.target_throughput+":STA_TARGET_UNIT", sta_throughput_target_unit)
        context.set_info(self._pars.target_throughput+":STA_FAILURE_VALUE", sta_throughput_failure_value.value)
        context.set_info(self._pars.target_throughput+":STA_FAILURE_UNIT", sta_throughput_failure_unit)
        context.set_info(self._pars.target_throughput+":P2P_TARGET_VALUE", p2p_throughput_target_value.value)
        context.set_info(self._pars.target_throughput+":P2P_TARGET_UNIT", p2p_throughput_target_unit)
        context.set_info(self._pars.target_throughput+":P2P_FAILURE_VALUE", p2p_throughput_failure_value.value)
        context.set_info(self._pars.target_throughput+":P2P_FAILURE_UNIT", p2p_throughput_failure_unit)
