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
:summary: This file implements the LIVE_DUAL_PHONE_BT_TRANSFER_WITH_SCAN
:author: jfranchx
:since:26/06/2013
"""

from acs_test_scripts.UseCase.LocalConnectivity.LIVE_DUAL_PHONE_BT_BASE import LiveDualPhoneBTBase
from acs_test_scripts.UseCase.Networking.LIVE_WIFI_BASE import LiveWifiBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.LocalConnectivityUtilities import ThreadOPPTransferFile, ThreadScanBT, opp_init_configure, opp_terminate
from acs_test_scripts.Utilities.NetworkingUtilities import ThreadScanWiFi
import time
import Queue
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveDualPhoneBTTransferWithScan(LiveDualPhoneBTBase, LiveWifiBase):
    """
    Live BT Transfer with scan (BT and/or WiFi)
    """

    SCANNING_WAIT_TIME = 20.0

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LiveBTBase init function
        LiveDualPhoneBTBase.__init__(self, tc_name, global_config)
        LiveWifiBase.__init__(self, tc_name, global_config)

        # Read COMPONENT_USE from test case xml file
        self._component_use = str(self._tc_parameters.get_param_value("COMPONENT_USE"))
        # Read DUT_STATE from test case xml file
        self._dut_state = str(self._tc_parameters.get_param_value("DUT_STATE"))
        # Read OPP_LOCAL_FILE from test case xml file
        self._opp_local_file = str(self._tc_parameters.get_param_value("OPP_LOCAL_FILE"))

        # Initialize data
        self._thread_opp_transfer = None
        self._thread_scan_bt = None
        self._thread_scan_wifi = None

        self._queue = Queue.Queue()

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveDualPhoneBTBase.set_up(self)

        opp_init_configure(self._device, self._phone2)
        time.sleep(self._wait_btwn_cmd)

        if self._component_use not in ["USE_WIFI", "USE_BT", "USE_BT_AND_WIFI"]:
            msg = "COMPONENT_USE parameter unknown - must be USE_WIFI, USE_BT or USE_BT_AND_WIFI"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._dut_state not in ["DUT_CLIENT", "DUT_SERVER"]:
            msg = "DUT state configuration unknown - DUT should be client or server"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # WiFi configure if needed
        if self._component_use in ["USE_WIFI", "USE_BT_AND_WIFI"]:
            if self._ssid in (None, ""):
                msg = str(self._ssid) + ": WiFi SSID is missing"
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
            self._networking_api.set_wifi_power(1)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCase base run_test function
        LiveDualPhoneBTBase.run_test(self)

        # Wifi configure if needed
        if self._component_use in ["USE_WIFI", "USE_BT_AND_WIFI"]:
            self._thread_scan_wifi = ThreadScanWiFi(self._queue, self._networking_api, self._ssid)
        # Configure BT if needed
        if self._component_use in ["USE_BT", "USE_BT_AND_WIFI"]:
            self._thread_scan_bt = ThreadScanBT(self._queue, self._bt_api, self._bt_api2)

        # Configure BT OPP Transfer
        if self._dut_state == "DUT_CLIENT":
            self._thread_opp_transfer = ThreadOPPTransferFile(self._queue, self._device, self._phone2,
                                                              self._opp_local_file, self._device.multimedia_path)
        else:
            self._thread_opp_transfer = ThreadOPPTransferFile(self._queue, self._phone2, self._device,
                                                              self._opp_local_file, self._phone2.multimedia_path)

        # Launch threads
        self._thread_opp_transfer.start()
        time.sleep(LiveDualPhoneBTTransferWithScan.SCANNING_WAIT_TIME)

        if self._thread_scan_wifi is not None:
            self._thread_scan_wifi.start()

        if self._thread_scan_bt is not None:
            self._thread_scan_bt.start()

        self._exception_reader(self._queue, [self._thread_scan_wifi, self._thread_scan_bt, self._thread_opp_transfer])

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Execute the test
        """

        # Call UseCase base tear_down function
        LiveDualPhoneBTBase.tear_down(self)

        # Clear OPP transfer
        opp_terminate(self._device, self._phone2)

        # WiFi configure if needed
        if self._component_use == "USE_WIFI" or self._component_use == "USE_BT_AND_WIFI":
            self._networking_api.set_wifi_power(0)

        # Delete sent file
        if self._dut_state == "DUT_CLIENT":
            self._bt_api2.bt_opp_init(self._opp_local_file)
        else:
            self._bt_api.bt_opp_init(self._opp_local_file)

        return Global.SUCCESS, "No errors"
