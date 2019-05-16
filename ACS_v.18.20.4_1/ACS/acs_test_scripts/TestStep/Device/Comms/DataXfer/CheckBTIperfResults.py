"""
@summary: This test step evaluates the throughput over BT measured by iperf, determines a
    test verdict, and logs the results.

@since 24 June 2014
@author: Val Peterson
@organization: INTEL PEG-SVE-DSV

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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
from acs_test_scripts.Utilities.IPerfUtilities import compute_iperf_verdict
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.ThroughputMeasure import ThroughputMeasure, DuplexThroughputMeasure


class CheckBTIperfResults(DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        DeviceTestStepBase.run(self, context)

        throughput = DuplexThroughputMeasure()
        throughput.ul_throughput.set(self._pars.ul_value, ThroughputMeasure.parse_unit(self._pars.ul_units))
        throughput.dl_throughput.set(self._pars.dl_value, ThroughputMeasure.parse_unit(self._pars.dl_units))


        try:
            throughput_targets = ConfigsParser("BT_Throughput_Targets").\
                parse_bt_targets(self._device.get_phone_model(), self._pars.bt_protocol)
        except IndexError:
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, "BT_Throughput_Targets does not have targets for %s model and %s protocol"%(self._device.get_phone_model(), self._pars.protocol))

        if throughput_targets is None:
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, "Could not obtain throughput targets")

        if self._pars.strict_target:
            throughput_targets.dl_failure = throughput_targets.dl_target
            throughput_targets.ul_failure = throughput_targets.ul_target

        verdict, msg = compute_iperf_verdict(throughput, throughput_targets, self._pars.direction)
        if verdict == Global.FAILURE:
            self._logger.error("CheckBTIperfResults FAILED!")
        else:
            self._logger.info("CheckBTIperfResults PASSED!")
        self._logger.info("Bluetooth Throughput Measurement Results")
        self._logger.info(msg)
        if verdict == Global.FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED, "CheckBTIperfResults failed")
