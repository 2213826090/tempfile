"""

:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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
:summary: This file implements LIVE WIFI TETHERING IPERF UC
:since: 10/08/2012
:author: jpstierlin RTC20900
"""

from LIVE_WIFI_TETHERING_BASE import LiveWifiTetheringBase
from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
from acs_test_scripts.Utilities.IPerfUtilities import compute_iperf_verdict, \
    parse_iperf_options
from UtilitiesFWK.Utilities import Global


class LiveWifiTetheringIperf(LiveWifiTetheringBase):

    """
    Live wifi Tethering iperf.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LIVE_WIFI_TETHERING_BASE Init function
        LiveWifiTetheringBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._direction = self._tc_parameters.get_param_value("DIRECTION")
        if self._direction is None:
            self._direction = 'both'
        else:
            self._direction = self._direction.lower()

        self._iperf_options = \
            self._tc_parameters.get_param_value("IPERF_OPTIONS")

        mode = self._tc_parameters.get_param_value("IPERF_MODE")
        if mode is None:
            mode = "single"
        else:
            mode = mode.lower()

        self._iperf_settings = {"mode": mode}

        self._throughput_targets = None
        self._server_ip_address = None

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """
        # Call LIVE_WIFI_TETHERING_BASE set_up function
        LiveWifiTetheringBase.set_up(self)

        if '-u' in self._iperf_options:
            protocole = "UDP"
        else:
            protocole = "TCP"

        # Read the throughput targets
        self._throughput_targets = ConfigsParser("Wifi_Throughput_Targets").\
            parse_wifi_targets(self._device.get_phone_model(),
                               self._hotspot_standard,
                               self._hotspot_security, protocole)

        if self._direction == 'down':
            self._server_ip_address = self._networking_api. \
                get_interface_ipv4_address(self._hotspot_ext_interface)
        else:
            if self._wifi_interface is not None:
                self._server_ip_address = self._interface_ip
            else:
                self._server_ip_address = self._networking_api2.get_wifi_ip_address()
        self._iperf_settings.update({"server_ip_address": self._server_ip_address})

        if self._direction is not None:
            self._iperf_settings.update({"direction": self._direction})
        if self._networking_api2 is not None:
            self._iperf_settings.update({"networking_api2": self._networking_api2})
        elif self._computer is not None:
            self._iperf_settings.update({"computer": self._computer})

        # Overwrite Iperf settings with iperf options read in TC parameter
        self._iperf_settings.update(parse_iperf_options(self._iperf_options))

        # Add optional interface to poll
        self._iperf_settings.update({'interface': self._dut_config.get("hotspotExtInterface")})

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call LIVE_WIFI_TETHERING_BASE Run function
        LiveWifiTetheringBase.run_test(self)

        # Run Iperf command
        throughput = self._networking_api.iperf(self._iperf_settings)

        # Compute verdict depending on throughputs
        return compute_iperf_verdict(throughput, self._throughput_targets, self._direction)
