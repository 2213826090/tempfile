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
:summary: This file implements the LIVE WIFI SCAN UC
:since: 22/08/2011
:author: szhen11
"""
import time
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LabWifiScan(LabWifiBase):

    """
    Lab Wifi Scan Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)
        if self._hidden:
            # The main goal of the test won't be reached
            self._logger.warning("The SSID has been set to 'hidden'. The " +
                                 "test will control that DUT cannot scan the AP.")

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # If Bt to be (de-)activated for FIT tests
        if self._bt_fit_used:

            self._bt_initial_state = self._localconnectivity_api.get_bt_power_status()

            # if Bt is not at the wished value, set it to the correct one.
            if self._bt_initial_state != self._bt_wished_value:
                self._localconnectivity_api.set_bt_power(self._bt_wished_value)
                time.sleep(self._wait_btwn_cmd)

        # Initiate connection to the equipment
        self._ns.init()
        # Regulatory Domain management
        self._set_regulatory_domain('AP')
        # Configure the equipment
        self._ns.set_wifi_config(self._ssid,
                                 self._hidden,
                                 self._standard,
                                 self._security,
                                 self._passphrase,
                                 self._channel,
                                 self._dtim,
                                 self._beacon,
                                 self._wmm,
                                 self._bandwidth,
                                 self._wifi_mimo,
                                 self._radiusip,
                                 self._radiusport,
                                 self._radiussecret)

        # Turn Wifi interface ON
        self._networking_api.set_wifi_power("on")
        time.sleep(self._wait_btwn_cmd)
        # Regulatory Domain management
        self._set_regulatory_domain('DUT')

        # Close the connection to AP
        self._ns.release()

        # Set frequency band to "auto"
        self._networking_api.set_wifi_frequency_band("auto", True, self._dut_wlan_iface)
        time.sleep(self._wait_btwn_cmd)

        # Configure the DUT
        self._networking_api.clean_all_data_connections()
        time.sleep(self._wait_btwn_cmd)

        # Put display on to avoid scan failure.
        self._phone_system_api.display_on()
        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        # Enable radio on AP
        self._ns.init()
        self._ns.enable_wireless()
        self._ns.release()

        # Scan and list ssids
        self._networking_api.request_wifi_scan()
        time.sleep(self._wait_btwn_cmd)

        passed = False
        start = time.time()
        # Maximum time needed to update the scan list.
        # In theory it would be 60 + 2s, but we take a little margin here.
        timeout = 70
        while not passed and time.time() < (start + timeout):
            if self._hidden:
                # Wait 60 seconds for the SSID scanned list to be refreshed
                time.sleep(timeout)
            else:
                time.sleep(3)

            ssids = self._networking_api.list_ssids("wifi", "all")
            if (not self._hidden and self._ssid in ssids) \
                    or (self._hidden and self._ssid not in ssids):
                passed = True

        if passed:
            if not self._hidden:
                msg = "No error, configurable AP successfully found"
            else:
                msg = "No error, hidden configurable AP not found"
            self._logger.info(msg)
        else:
            if not self._hidden:
                msg = "cannot find the configurable AP [%s]" % self._ssid
            else:
                msg = "Hidden SSID has been found [%s]" % self._ssid
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Disable radio on AP
        self._ns.init()
        self._ns.disable_wireless()
        self._ns.release()

        # Force a scan
        self._networking_api.request_wifi_scan()
        time.sleep(self._wait_btwn_cmd)

        passed = False
        start_time = time.time()
        # Wait for the DUT to flush its SSID cache
        while not passed and time.time() < (start_time + timeout):
            # Wait 3 seconds then scan again
            time.sleep(3)

            ssids = self._networking_api.list_ssids("wifi", "all")

            # Access Point should not be in the list
            passed = self._ssid not in ssids

        if passed:
            if not self._hidden:
                msg = "No error, configurable AP successfully disappeared"
            else:
                msg = "No error, hidden configurable AP not found"
            self._logger.info(msg)
        else:
            if not self._hidden:
                msg = "configurable AP [%s] down but still in scan list" \
                    % self._ssid
            else:
                msg = "Hidden SSID has been found [%s]" % self._ssid
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, msg

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        # Put display off.
        self._phone_system_api.display_off()

        # Set wifi power to off
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_wifi_power("off")

        # Disable radio for all Wifi APs
        self._disable_all_radios()

        # If BT has to be (de-)activated for FIT tests
        if self._bt_fit_used:

            bt_final_status = self._localconnectivity_api.get_bt_power_status()

            # if Bt is not at the initial value, set it to the correct one.
            if bt_final_status != self._bt_initial_state:
                # restore Bt in initial state
                self._localconnectivity_api.set_bt_power(self._bt_initial_state)
                time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"
