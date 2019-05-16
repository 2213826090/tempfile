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
:summary:  This file implements the LTE Iperf testCase.
:since: 12/01/2014
:author: gcharlex
"""
import copy
import time
from LAB_LTE_BASE import LabLteBase
from LAB_LTE_IPERF import LabLteIperf
from UtilitiesFWK.Utilities import Global
from UtilitiesFWK.Utilities import format_exception_info
from acs_test_scripts.Utilities.CommunicationUtilities import TelephonyConfigsParser
from acs_test_scripts.Utilities.IPerfUtilities import compute_iperf_verdict


class LabLteIperfCaSwitch(LabLteIperf):

    """
    Lab LTE Iperf test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_LTE_IPERF Init function
        LabLteIperf.__init__(self, tc_name, global_config)

        # SWAP CARRIER
        self._swap_carrier = \
            self._tc_parameters.get_param_value("SWAP_CARRIER", False, "str_to_bool")

        # Save PCC throughput targets
        self._pcc_throughput_targets, _ = TelephonyConfigsParser("Throughput_Targets").\
            parse_lte_theoretical_targets(self._lte_category, self._bandwidth, self._antennas_number)

        # Save SCC throughput targets
        self._scc_throughput_targets, _ = TelephonyConfigsParser("Throughput_Targets").\
            parse_lte_theoretical_targets(self._lte_category, self._scc_bandwidth, self._antennas_number)

        # Iperf duration
        self._iperf_duration = 120

    def set_up(self):
        """
        Initialize the test.
        """

        # Call LAB_LTE_IPERF set_up function
        LabLteIperf.set_up(self)

        # Disable Secondary Carrier Component
        self._ns_cell_4g.set_secondary_carrier_state("OFF")

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test.
        Configuring the internal IPERF server of the equipment.
        Starting this IPERF server.
        Launching the IPERF client
        Computing the throughput to get a verdict.
        """
        result_msg = ""

        # Call LAB_LTE_BASE Run function
        LabLteBase.run_test(self)
        time.sleep(self._wait_btwn_cmd)

        # Launch the IPERF test and get throughput
        try:
            if self._ns_cell_4g.get_cell_status() == "OFF":
                self._logger.info("4G cell is OFF, restarting it")
                self._ns_cell_4g.set_cell_on()
                if self._ns_cell_4g.get_cell_status() == "OFF":
                    self._logger.error("4G cell is OFF, cannot run test iteration")
                    # raise Exception in order to get proper exit handling
                    raise Exception
                self._connect_dut_to_4g_cell()

            # Start asynchronus Iperf
            iperf_async = self._networking_api.iperf_async(self._iperf_settings)

            # Throughput Target = PCC throughput targets
            throughput_targets = copy.deepcopy(self._pcc_throughput_targets)
            throughput_targets.set_failure_throughput_from_config(self._dut_config,
                                                                  self._failure_targets)

            # Iperf measure
            throughput = iperf_async.perform_continous_measure(self._iperf_duration)

            # Compute Iperf verdict
            (result_code, result_msg_tmp) = \
                compute_iperf_verdict(throughput,
                                      throughput_targets,
                                      self._iperf_direction)
            self._logger.info(result_msg_tmp)
            result_msg += result_msg_tmp + "\n"
            if result_code == Global.FAILURE:
                return result_code, result_msg_tmp

            # Activate SCC
            self._ns_cell_4g.set_secondary_carrier_state("MACactivate")
            self._ns_cell_4g.check_secondary_carrier_state_before_timeout("MACactivate", 30)

            # Throughput Target = PCC + SCC throughput targets
            # DL = PCC DL + SCC DL --- UL = PCC UL
            throughput_targets = copy.deepcopy(self._pcc_throughput_targets)
            throughput_targets.add_secondary_carrier_throughput_targets(self._scc_throughput_targets)
            throughput_targets.set_failure_throughput_from_config(self._dut_config,
                                                                  self._failure_targets)

            # Iperf measure
            throughput = iperf_async.perform_continous_measure(self._iperf_duration)

            # Compute Iperf verdict
            (result_code, result_msg_tmp) = \
                compute_iperf_verdict(throughput,
                                      throughput_targets,
                                      self._iperf_direction)
            self._logger.info(result_msg_tmp)
            result_msg += result_msg_tmp + "\n"
            if result_code == Global.FAILURE:
                return result_code, result_msg_tmp

            if self._swap_carrier:
                # Swap SCC <-> PCC
                self._ns_cell_4g.swap_primary_and_secondary_carrier_settings()
                self._ns_cell_4g.check_secondary_carrier_state_before_timeout("MACactivate", 30)

                # Throughput Target = SCC + PCC throughput targets
                # DL = SCC DL + PCC DL --- UL = SCC UL
                self._pcc_throughput_targets, self._scc_throughput_targets = \
                    self._scc_throughput_targets, self._pcc_throughput_targets
                throughput_targets = copy.deepcopy(self._pcc_throughput_targets)
                throughput_targets.add_secondary_carrier_throughput_targets(self._scc_throughput_targets)
                throughput_targets.set_failure_throughput_from_config(self._dut_config,
                                                                      self._failure_targets)

                # Iperf measure
                throughput = iperf_async.perform_continous_measure(self._iperf_duration)

                # Compute Iperf verdict
                (result_code, result_msg_tmp) = \
                    compute_iperf_verdict(throughput,
                                          throughput_targets,
                                          self._iperf_direction)
                self._logger.info(result_msg_tmp)
                result_msg += result_msg_tmp + "\n"
                if result_code == Global.FAILURE:
                    return result_code, result_msg_tmp

            # Deactivate SCC
            self._ns_cell_4g.set_secondary_carrier_state("OFF")
            self._ns_cell_4g.check_secondary_carrier_state_before_timeout("OFF", 30)

            # Throughput Target = PCC throughput targets
            throughput_targets = copy.deepcopy(self._pcc_throughput_targets)
            throughput_targets.set_failure_throughput_from_config(self._dut_config,
                                                                  self._failure_targets)

            # Iperf measure
            throughput = iperf_async.perform_continous_measure(self._iperf_duration)

            # Compute Iperf verdict
            (result_code, result_msg_tmp) = \
                compute_iperf_verdict(throughput,
                                      throughput_targets,
                                      self._iperf_direction)
            self._logger.info(result_msg_tmp)
            result_msg += result_msg_tmp + "\n"
            if result_code == Global.FAILURE:
                return result_code, result_msg_tmp

            # Wait end of IPERF
            iperf_async.stop_iperf_async()

        except Exception:
            result_code = Global.FAILURE
            result_msg = "!!!! WARNING Exception occurred during iperf test!!! "
            exception_text = format_exception_info()
            self._logger.debug("Exception during iperf test: %s ", exception_text)
            self._logger.error("!!!! Exception occurred during iperf test!!! ")
            return result_code, result_msg

        # Return result
        return Global.SUCCESS, result_msg

# ------------------------------------------------------------------------------

    def tear_down(self):
        """
        Finishing the test.
        Stopping the IPERF server and releasing the equipment.
        """
        LabLteIperf.tear_down(self)

        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------
