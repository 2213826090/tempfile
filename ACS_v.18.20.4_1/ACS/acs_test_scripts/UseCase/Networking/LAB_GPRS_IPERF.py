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

:summary: Use Case GPRS IPERF for SMOKE and BAT tests.
:organization: INTEL MCG PSI
:author: cco
:since: 02/09/2010
"""

import time

from LAB_GPRS_BASE import LabGprsBase
from acs_test_scripts.Utilities.IPerfUtilities import compute_iperf_verdict, \
    get_iperf_configuration, set_iperf_ip_settings, retrieve_parameters_from_tc_params
from acs_test_scripts.Utilities.CommunicationUtilities import throughput_targets_string
from UtilitiesFWK.Utilities import Global


class LabGprsIperf(LabGprsBase):
    """
    Lab GPRS iperf test.
    """

    def __init__(self, tc_name, global_config):

        # Call LAB_GPRS_BASE Init function
        LabGprsBase.__init__(self, tc_name, global_config)

        # Read Iperf PORT
        self._port = int(self._tc_parameters.get_param_value("PORT"))

        # Read Iperf measurement duration
        self._duration = \
            int(self._tc_parameters.get_param_value("DURATION"))

        # Read Iperf protocol
        self._iperf_protocol = \
            str(self._tc_parameters.get_param_value("IPERF_PROTOCOL", "TCP"))

        # Read Iperf TUNE OPTIONS
        self._tune_options = self._tc_parameters.get_param_value("IPERF_TUNE_OPTIONS", "1")
        if self._tune_options in (None, "", 1, '1', 'on', 'ON'):
            self._tune_options = 1
        else:
            self._tune_options = 0

        # Read Iperf Options
        self._iperf_options = \
            self._tc_parameters.get_param_value("IPERF_OPTIONS")

        # Get TCP window size and number of iperf client threads to run concurrently
        # window size > tput_to_reach * round_trip_time (700ms for GPRS)
        # Iperf compute automatically the windows size.
        # It is better to not set a window size as result with iperf default size are pretty good
        self._iperf_uldl_parameters = retrieve_parameters_from_tc_params(self._tc_parameters)

        # Get computer type
        self._computer = self._tc_parameters.get_param_value("COMPUTER")

        # Get customized failure Targets
        self._failure_targets = \
            str(self._tc_parameters.get_param_value("FAILURE_TARGETS", ""))

        self._iperf_settings = \
            {"server_ip_address": self._server_ip_address,
             "port_number": self._port,
             "duration": self._duration,
             "protocol": self._iperf_protocol.lower()}

        if self._computer == "":
            self._computer = None
        # Load computer equipment
        if self._computer is not None:
            self._computer = self._em.get_computer(self._computer)

        # Update the failure targets
        self._throughput_targets.set_failure_throughput_from_config(self._dut_config,
                                                                    self._failure_targets)

        # Log Throughput targets for GPRS
        self._logger.info(throughput_targets_string(self._throughput_targets))

        # Set Iperf direction
        if self._direction in ("UL", "up", None):
            self._iperf_direction = "up"
        elif self._direction in ("DL", "down"):
            self._iperf_direction = "down"
        else:
            self._iperf_direction = "both"

    def set_up(self):
        """
        Initialize the test
        """
        # Call LAB_GPRS_BASE set_up function
        LabGprsBase.set_up(self)

        # Update server and DUT IP address in iperf_settings
        set_iperf_ip_settings(self._iperf_settings, self._iperf_direction, self._computer, self._networking_api, self._device.get_cellular_network_interface(), self._registration_timeout)

        if self._computer is not None:
            self._iperf_settings.update({"computer": self._computer})

        if self._tune_options == 1:
            # Set Iperf settings depending on the expected throughput
            self._iperf_settings.update(
                get_iperf_configuration(self._throughput_targets))
        else:
            self._iperf_settings.update({'direction': self._iperf_direction})
            self._iperf_settings.update(self._iperf_uldl_parameters)

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """

        # Call LAB_GPRS_BASE Run function
        LabGprsBase.run_test(self)

        # Start IPERF measurement using PORT
        # and DURATION and LAB_SERVER ip_address parameters
        time.sleep(self._wait_btwn_cmd)

        throughput = self._networking_api.iperf(self._iperf_settings)

        # Compute verdict depending on throughputs
        return compute_iperf_verdict(throughput, self._throughput_targets, self._iperf_direction)