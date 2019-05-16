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
:summary: This file implements the LIVE_BT_AND_WIFI_SCAN
:author: jfranchx
:since:31/07/2013
"""

import time
import Queue

from acs_test_scripts.UseCase.LocalConnectivity.LIVE_BT_BASE import LiveBTBase
from acs_test_scripts.UseCase.Networking.LIVE_WIFI_BASE import LiveWifiBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.LocalConnectivityUtilities import ThreadScanBTRemoteDevice
from acs_test_scripts.Utilities.NetworkingUtilities import ThreadScanWiFi
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveBTAndWiFiScan(LiveBTBase, LiveWifiBase):
    """
    Live BT and WiFi scan simultaneous
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveBTBase init function
        LiveBTBase.__init__(self, tc_name, global_config)
        LiveWifiBase.__init__(self, tc_name, global_config)

        self._ref_bt_addr = str(self._tc_parameters.get_param_value("REF_BT_ADDR")).upper()
        self._duration = str(self._tc_parameters.get_param_value("DURATION"))

        self._bench_config = global_config.benchConfig.get_parameters("BT_DEVICE")
        self._thread_bt_scan = None
        self._thread_wifi_scan = None
        self._queue = Queue.Queue()

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveBTBase.set_up(self)

        if self._ssid in (None, ""):
            msg = str(self._ssid) + ": WiFi SSID is missing"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._ref_bt_addr.lower() in ["none", ""] and self._bench_config is not None:
            self._ref_bt_addr = str(self._bench_config.get_param_value("MacAddress"))

        if self._ref_bt_addr.lower() in ["none", "", "00:00:00:00:00:00"]:
            msg = "No BD address found in the TC and/or in bench_config"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, msg)

        if not str(self._duration).isdigit():
            msg = "Bad value on duration parameter : %s" % self._duration
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        self._duration = int(self._duration)

        # Configure WiFi
        self._networking_api.set_wifi_power(1)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base run_test function
        LiveBTBase.run_test(self)

        # Configure BT Scan
        self._thread_bt_scan = ThreadScanBTRemoteDevice(self._queue, self._bt_api, self._ref_bt_addr, self._duration)

        # Configure WiFi Scan
        self._thread_wifi_scan = ThreadScanWiFi(self._queue, self._networking_api, self._ssid, self._duration)

        self._thread_wifi_scan.start()
        self._thread_bt_scan.start()
        time.sleep(self._wait_btwn_cmd)
        self._exception_reader(self._queue, [self._thread_bt_scan, self._thread_wifi_scan])

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Finish the test and clear environment
        """

        # Call UseCase base tear_down function
        LiveBTBase.tear_down(self)

        # Disable WiFi
        self._networking_api.set_wifi_power(0)

        return Global.SUCCESS, "No errors"
