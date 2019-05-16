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
:summary: This file implements the LIVE WIFI GETBAND UC
:since: 20/11/2012
:author: emarchan - RTC32863 - Test dual band selection
"""


from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from time import sleep
import time
from ErrorHandling.DeviceException import DeviceException


class LiveWifiGetband(UseCaseBase):

    """
    Live Wifi getband Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get UECmdLayer
        self._networking_api = self._device.get_uecmd("Networking")

        self._original_band_selection = None

        # Set BT prerequisite (set ON, set OFF or nothing) for FIT BT/WIFI tests
        self._bt_fit_used, self._bt_wished_value = self._get_bt_fit_config()
        # BT initial state
        self._bt_initial_state = 'STATE_ON'
        # Instantiate generic UECmd
        self._localconnectivity_api = self._device.get_uecmd("LocalConnectivity")
        self._dut_wlan_iface = str(self._dut_config.get("wlanInterface"))
#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # run setup inherit from UseCaseBase
        UseCaseBase.set_up(self)

        # If Bt to be (de-)activated for FIT tests
        if self._bt_fit_used:

            self._bt_initial_state = self._localconnectivity_api.get_bt_power_status()

            # if Bt is not at the wished value, set it to the correct one.
            if self._bt_initial_state != self._bt_wished_value:
                self._localconnectivity_api.set_bt_power(self._bt_wished_value)
                time.sleep(self._wait_btwn_cmd)

        # Turn Wifi interface ON
        self._networking_api.set_wifi_power("on")
        sleep(self._wait_btwn_cmd)

        # Set a default regulatory domain to enable 5Ghz
        self._networking_api.set_regulatorydomain(
            self._networking_api.get_default_regulatory_domain(), self._dut_wlan_iface)

        self._original_band_selection = \
            self._networking_api.get_wifi_frequency_band(self._dut_wlan_iface)

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        for cur_band in ["auto", "5GHz", "2.4GHz"]:
            # Set the new band
            self._networking_api.set_wifi_frequency_band(cur_band, False, self._dut_wlan_iface)
            sleep(self._wait_btwn_cmd)
            # Check the new band
            res_band = self._networking_api.get_wifi_frequency_band()

            # Compare it to the ID the device gave.
            if res_band == cur_band:
                msg = "Got band %s - OK" % res_band
                self._logger.info(msg)
            else:
                msg = "Got band %s instead of %s" % (res_band, cur_band)
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        # restore original wifi band
        if self._original_band_selection is not None:
            self._logger.info(
                "Setting band back to %s" % self._original_band_selection)
            self._networking_api.set_wifi_frequency_band(self._original_band_selection, False, self._dut_wlan_iface)

        # Turn Wifi interface ON
        self._networking_api.set_wifi_power("off")
        sleep(self._wait_btwn_cmd)

        # If Bt to be (de-)activated for FIT tests
        if self._bt_fit_used:

            bt_final_status = self._localconnectivity_api.get_bt_power_status()

            # if Bt is not at the initial value, set it to the correct one.
            if bt_final_status != self._bt_initial_state:
                # restore Bt in initial state
                self._localconnectivity_api.set_bt_power(self._bt_initial_state)
                time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def _get_bt_fit_config(self):
        """
        Get BT configuration for FIT BT/WIFI tests

        :rtype: list of 2 elements (boolean, str)
        :return: true if BT/WIFI FIT is used, Bluetooth state ON or OFF for the test to be run
        """

        # Read WHILE_BLUETOOTH_ON parameter (named for retro-compatibility) from test case xml file for FIT tests
        param_while_bt_on = \
            str(self._tc_parameters.get_param_value("WHILE_BLUETOOTH_ON"))

        if param_while_bt_on.lower() in ["1", "on", "true", "yes"]:
            bt_fit_used = True
            bt_wished_value = 'STATE_ON'
        elif param_while_bt_on.lower() in ["0", "off", "false", "no"]:
            bt_fit_used = True
            bt_wished_value = 'STATE_OFF'
        else:
            bt_fit_used = False
            bt_wished_value = 'STATE_OFF'

        return bt_fit_used, bt_wished_value
#------------------------------------------------------------------------------
