"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: This file implements the LIVE WIFI KPI IPERF UC
@since: 02/04/2014
@author: jfranchx
"""
import time
from ErrorHandling.AcsConfigException import AcsConfigException

from acs_test_scripts.UseCase.Networking.LIVE_WIFI_IPERF import LiveWifiIperf
from acs_test_scripts.UseCase.Networking.WiFi_KRI_KPI.LIVE_WIFI_KPI_BASE import LiveWifiKPIBase
from UtilitiesFWK.Utilities import Global


class LiveWifiKPIIperf(LiveWifiKPIBase, LiveWifiIperf):
    """
    Live WiFi KPI Iperf Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveWifiIperf.__init__(self, tc_name, global_config)
        LiveWifiKPIBase.__init__(self, tc_name, global_config)

    def set_up(self):
        """
        Initialize the test
        """
        LiveWifiIperf.set_up(self)
        LiveWifiKPIBase.set_up(self)

        # Configure RSSI
        self._rssi_value = self._networking_api.get_wifi_rssi(self._ssid)
        if self._rssi_value > self.RSSI_MAX_VALUE or self._rssi_value < self.RSSI_MIN_VALUE:
            msg = "Bad RSSI to run the test : %sdBm - require between -40dBm and -45dBm" % self._rssi_value
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Run test
        """

        LiveWifiKPIBase.run_test(self)
        # Flush TCP records
        self._networking_api.clean_tcp_records_device()
        self._computer.clean_tcp_records_computer()

        time.sleep(self._wait_btwn_cmd)

        iperf_result, iperf_msg = LiveWifiIperf.run_test(self)

        complete_msg = "RSSI value : %s -- %s" % (self._rssi_value, iperf_msg)
        return iperf_result, complete_msg

    def tear_down(self):
        """
        End and dispose the test
        """
        LiveWifiIperf.tear_down(self)

        # Reconfigure default device state
        LiveWifiKPIBase.tear_down(self)

        return Global.SUCCESS, "No errors"
