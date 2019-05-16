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
:summary: This file implements the LAB HSPA IPERF UC
:since: 16/10/2010
:author: ccontreras
"""

import time
from acs_test_scripts.UseCase.Networking.LAB_TDSCDMA_BASE import LabTdscdmaBase
from acs_test_scripts.Utilities.IPerfUtilities import compute_iperf_verdict, \
    get_iperf_configuration, retrieve_parameters_from_tc_params, get_random_iperf_port
from acs_test_scripts.Utilities.CommunicationUtilities import KPIUtil, throughput_targets_string
from UtilitiesFWK.Utilities import format_exception_info
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.IPerfUtilities import set_iperf_ip_settings, compute_iperf_bw


class LabTdscdmaIperf(LabTdscdmaBase):

    """
    Lab TD-SCDMA iperf test.
    """

    def __init__(self, tc_name, global_config):

        # Call LAB_HSPA_BASE Init function
        LabTdscdmaBase.__init__(self, tc_name, global_config)

        # Read Iperf PORT
        self._port = self._tc_parameters.get_param_value("PORT")

        if self._port in (None, "", "None"):
            # If no port are specified use a randomly generated port
            self._random_port = True
        else:
            self._port = int(self._port)
            self._random_port = False

        # Read Iperf measurement duration
        self._duration = \
            int(self._tc_parameters.get_param_value("DURATION"))

        # Read Iperf protocol
        self._iperf_protocol = \
            self._tc_parameters.get_param_value("IPERF_PROTOCOL", "TCP").lower()

        self._iperf_bandwidth = \
            str(self._tc_parameters.get_param_value("IPERF_BANDWIDTH", None)).upper()

        # Get TCP window size and number of iperf client threads to run concurrently
        # window size > tput_to_reach * round_trip_time (200ms for HSPA)
        # Iperf compute automatically the windows size.
        # It is better to not set a window size as result with iperf default size are pretty good
        self._iperf_uldl_parameters = retrieve_parameters_from_tc_params(self._tc_parameters)

        # Get computer type
        self._computer_temp = self._tc_parameters.get_param_value("COMPUTER", "IPERF_SERVER")

        # Get customized failure Targets
        self._failure_targets = \
            str(self._tc_parameters.get_param_value("FAILURE_TARGETS", ""))

        # Detect if it is a KPI test or not
        self._kpi_test = self._tc_parameters.get_param_value("KPI_TEST", False, "str_to_bool")
        if self._kpi_test:
            self._logger.info("It is a KPI test")
            self._kpi_data = None
            self._current_iteration = 0

        # Read Iperf TUNE OPTIONS
        self._tune_options = self._tc_parameters.get_param_value("IPERF_TUNE_OPTIONS", "0")
        if self._tune_options in (None, "", 0, '0', 'off', 'OFF'):
            self._tune_options = 0
        else:
            self._tune_options = 1

        # Load computer equipment
        if self._computer_temp is not None:
            self._computer = self._em.get_computer(self._computer_temp)

        self._computer_type = \
            global_config.benchConfig.get_parameters(self._computer_temp)

        # Get server IP address
        self._server_ip_address = self._computer_type.get_param_value("IP")

        # Set Iperf direction
        if self._direction in ("UL", "up", None):
            self._iperf_direction = "up"
        elif self._direction in ("DL", "down"):
            self._iperf_direction = "down"
        else:
            self._iperf_direction = "both"

        self._iperf_settings = {"server_ip_address": self._server_ip_address,
                                "port_number": self._port,
                                "duration": self._duration,
                                "protocol": self._iperf_protocol,
                                "computer": self._computer}

        # Update the failure targets
        self._throughput_targets.set_failure_throughput_from_config(self._dut_config,
                                                                    self._failure_targets,
                                                                    self._kpi_test,
                                                                    tc_name._name)

        # Log Throughput targets for TDSCDMA
        self._logger.info(throughput_targets_string(self._throughput_targets))

# ------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call LAB_TDSCDMA_BASE set_up function
        LabTdscdmaBase.set_up(self)

        # Set up DUT to registered
        self._set_up_registration()

        # Set up DUT to pdp activated
        self._set_up_pdp_active()

        # Reset KPI data needed for computing KPI test result
        if self._kpi_test:
            self._kpi_data = KPIUtil()
            self._current_iteration = 0

        # Update server and DUT IP address in iperf_settings
        set_iperf_ip_settings(self._iperf_settings, self._iperf_direction, self._computer,
                              self._networking_api, self._device.get_cellular_network_interface(), self._registration_timeout)

        # Get the iperf configuration corresponding to the target throughput to reach
        if self._tune_options:
            self._iperf_settings.update(get_iperf_configuration(self._throughput_targets,
                                                                self._iperf_direction))
        else:
            self._iperf_settings.update({'direction': self._iperf_direction})

            self._iperf_settings.update(self._iperf_uldl_parameters)

            if self._iperf_protocol.upper() == "UDP":
                # Compute IPERF bandwidth from target throughput and iperf bandwidth
                compute_iperf_bw(self._iperf_settings, self._throughput_targets, self._iperf_bandwidth)

        # Force screen on to avoid end of PDP context due to fast dormancy
        self._phone_system.set_screen_timeout(0)
        self._phone_system.wake_screen()
        self._phone_system.set_phone_screen_lock_on(1)
        return Global.SUCCESS, "No errors"

# ------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        result_code = Global.SUCCESS
        result_msg = ""
        result_msg_tmp = ""
        throughput = None

        # Call LAB_HSPA_BASE Run function
        LabTdscdmaBase.run_test(self)

        # Start IPERF measurement
        time.sleep(self._wait_btwn_cmd)

        # Launch the IPERF test and get throughput
        try:
            # Force screen on to avoid end of PDP context due to fast dormancy
            self._phone_system.wake_screen()
            self._phone_system.set_phone_screen_lock_on(1)
            if self._ns_data_3g.get_data_connection_status() != "PDP_ACTIVE":
                self._ns_data_3g.check_data_connection_state("PDP_ACTIVE",
                                                             self._registration_timeout,
                                                             blocking=False)
            if self._random_port:
                # Get a randomly generated port
                self._port = get_random_iperf_port()
                self._iperf_settings["port_number"] = self._port
            throughput = self._networking_api.iperf(self._iperf_settings)

            (result_code, result_msg_tmp) = \
                compute_iperf_verdict(throughput,
                                      self._throughput_targets,
                                      self._iperf_direction)
            if self._kpi_test:
                # In case of KPI test, store measured throughput
                # for final verdict (it will depend of the median value of all measured values)
                self._kpi_data.append(throughput)
                result_msg = "KPI test Iteration: %d " \
                             % (self._current_iteration + 1) + result_msg_tmp
            else:
                result_msg = result_msg_tmp

        except Exception as e:
            result_code = Global.FAILURE
            result_msg = "!!!! WARNING Exception occurred during iperf test !!!! "
            exception_text = format_exception_info()
            self._logger.warning("!!!! WARNING Exception occurred during iperf test !!!! ")
            self._logger.debug("Exception during iperf test: %s ", exception_text)
        finally:
            if self._kpi_test:
                self._current_iteration += 1
                # For KPI test verdict is computed only on median throughput computed on last
                # iteration, other reasons(exception, iteration not run, not last iteration ...)
                # does not alter the verdict
                result_code = Global.SUCCESS
                # If we are in the latest iteration of a KPI test
                # compute throughput median value and compute verdict on this median value
                if self._current_iteration == self.get_b2b_iteration():
                    median_throughput = self._kpi_data.get_median_throughput()
                    self._logger.info("Median Throughput : DL: %s UL %s" % (str(median_throughput.dl_throughput), str(median_throughput.ul_throughput)))
                    (result_code, result_msg_tmp) = compute_iperf_verdict(median_throughput,
                                                                          self._throughput_targets,
                                                                          self._iperf_direction)
                    result_msg = "KPI median throughput: " + result_msg_tmp
        # Return result
        return result_code, result_msg
