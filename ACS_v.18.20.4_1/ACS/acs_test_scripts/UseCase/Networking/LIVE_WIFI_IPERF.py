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
:summary: This file implements the Live WIFI IPERF UC
:since: 03/03/2014
:author: jfranchx
"""

import time
from acs_test_scripts.UseCase.Networking.LIVE_WIFI_BASE import LiveWifiBase
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
from acs_test_scripts.Utilities.IPerfUtilities import compute_iperf_verdict, \
    get_iperf_configuration, parse_iperf_options
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.ThroughputMeasure import ThroughputMeasure


class LiveWifiIperf(LiveWifiBase):

    """
    Live Wifi Iperf test.
    """

    def __init__(self, tc_name, global_config):
        LiveWifiBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._iperf_options = \
            self._tc_parameters.get_param_value("IPERF_OPTIONS")

        self._direction = self._tc_parameters.get_param_value("DIRECTION")
        if self._direction is None or self._direction == "":
            self._direction = 'both'
        else:
            self._direction = self._direction.lower()

        mode = self._tc_parameters.get_param_value("IPERF_MODE")
        if mode is None or mode == "":
            mode = "single"
        else:
            mode = mode.lower()

        self._tune_options = self._tc_parameters.\
            get_param_value("IPERF_TUNE_OPTIONS")
        if self._tune_options in (None, 1, '1', 'on', 'ON'):
            self._tune_options = 1
        else:
            self._tune_options = 0

        self._target_ul = self._tc_parameters.get_param_value("TARGET_UL")
        self._failure_ul = self._tc_parameters.get_param_value("FAILURE_UL")
        self._target_dl = self._tc_parameters.get_param_value("TARGET_DL")
        self._failure_dl = self._tc_parameters.get_param_value("FAILURE_DL")

        # Get computer type
        self._computer = self._tc_parameters.get_param_value("COMPUTER")
        if self._computer == "":
            self._computer = None
        # Load computer equipment
        if self._computer is not None:
            self._computer = self._em.get_computer(self._computer)

        self._throughput_targets = None
        self._iperf_settings = {"server_ip_address": self._server_ip_address,
                                "mode": mode}

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call LIVE_WIFI_BASE set_up function
        LiveWifiBase.set_up(self)

        if '-u' in self._iperf_options:
            protocole = "UDP"
        else:
            protocole = "TCP"

        # Read the throughput targets
        self._throughput_targets = ConfigsParser("Wifi_Throughput_Targets").\
            parse_wifi_targets(self._device.get_phone_model(),
                               self._standard, self._security, protocole, self._bandwidth)

        # Overwrite target values with those read in TC parameter
        if self._target_ul is not None and self._target_ul != "":
            self._throughput_targets.ul_target.set(float(self._target_ul),
                                                   ThroughputMeasure.KBPS_UNIT)
        if self._failure_ul is not None and self._failure_ul != "":
            self._throughput_targets.ul_failure.set(float(self._failure_ul),
                                                    ThroughputMeasure.KBPS_UNIT)
        if self._target_dl is not None and self._target_dl != "":
            self._throughput_targets.dl_target.set(float(self._target_dl),
                                                   ThroughputMeasure.KBPS_UNIT)
        if self._failure_dl is not None and self._failure_dl != "":
            self._throughput_targets.dl_failure.set(float(self._failure_dl),
                                                    ThroughputMeasure.KBPS_UNIT)

        if self._direction not in ("both", "up", "down"):
            msg = "%s is not a valid DIRECTION."
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        self._iperf_settings.update({"direction": self._direction})

        if self._computer is not None:
            self._iperf_settings.update({"computer": self._computer})

        if self._direction == 'down':
            # Downlink: connect from host to DUT, get DUT IP address.
            ip = self._networking_api.get_wifi_ip_address()
            self._iperf_settings.update({"server_ip_address": ip})
        elif self._computer is not None:
            # Uplink/both: connect from DUT to host, get computer IP address.
            # if computer is None or localhost, use WIFI_SERVER from Bench_Config
            ip = self._computer.get_host_on_test_network()
            if ip not in ("localhost", "127.0.0.1"):
                self._iperf_settings.update({"server_ip_address": ip})

        if self._tune_options == 1:
            # Set Iperf settings depending on the expected throughput
            self._iperf_settings.update(
                get_iperf_configuration(self._throughput_targets))

        # Overwrite Iperf settings with iperf options read in TC parameter
        self._iperf_settings.update(parse_iperf_options(self._iperf_options))

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """
        # Call LIVE_WIFI_BASE Run function
        LiveWifiBase.run_test(self)
        time.sleep(self._wait_btwn_cmd)

        # Run Iperf command
        throughput = self._networking_api.iperf(self._iperf_settings)

        # Compute verdict depending on throughputs
        return compute_iperf_verdict(throughput, self._throughput_targets, self._direction)
