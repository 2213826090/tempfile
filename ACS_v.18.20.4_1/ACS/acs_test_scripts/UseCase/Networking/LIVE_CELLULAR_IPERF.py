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
:summary: This file implements the LIVE WCDMA IPERF UC
:since: 30/03/2010
:author: cbresoli
"""

import time
from LIVE_CELLULAR_BASE import LiveCellularBase
from acs_test_scripts.Utilities.IPerfUtilities import compute_iperf_verdict, get_iperf_configuration
from acs_test_scripts.Utilities.CommunicationUtilities import TelephonyConfigsParser


class LiveCellularIperf(LiveCellularBase):

    """
    Live WCDMA iperf test.
    """

    def __init__(self, tc_name, global_config):
        LiveCellularBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._port = int(self._tc_parameters.get_param_value("PORT"))
        self._duration = \
            int(self._tc_parameters.get_param_value("DURATION"))
        self._iperf_protocol = \
            str(self._tc_parameters.get_param_value("IPERF_PROTOCOL","TCP"))
        # Read the throughput targets
        self._throughput_targets = TelephonyConfigsParser("Throughput_Targets").\
            parse_live_wcdma_targets()

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call LAB_WCDMA_BASE Run function
        LiveCellularBase.run_test(self)

        time.sleep(self._wait_btwn_cmd)

        # Start IPERF measurement using PORT
        # and DURATION and WIFI SERVER ip_address parameters
        time.sleep(self._wait_btwn_cmd)

        iperf_settings = \
            {"server_ip_address": self._server_ip_address,
             "port_number": self._port,
             "duration": self._duration,
             "protocol": self._iperf_protocol.lower()}
        iperf_settings.update(get_iperf_configuration(self._throughput_targets))

        throughput = self._networking_api.iperf(iperf_settings)

        # Compute verdict depending on throughputs
        return compute_iperf_verdict(throughput, self._throughput_targets)
