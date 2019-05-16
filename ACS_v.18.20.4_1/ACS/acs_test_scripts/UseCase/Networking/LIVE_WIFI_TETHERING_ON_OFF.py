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
:summary: This file implements the LIVE WIFI TETHERING ON/OFF UC
:since: 05/09/2012
:author: jpstierlin RTC20899 - Increase wifi tethering coverage
"""

import time

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global


class LiveWifiTetheringOnOff(UseCaseBase):

    """
    Live Wifi Tethering on/off Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get host spot configuration according HOTSPOT_SSID
        self._hotspot_ssid = \
            str(self._tc_parameters.get_param_value("HOTSPOT_SSID"))

        # Get host spot configuration according HOTSPOT_SECURITY
        self._hotspot_security = \
            str(self._tc_parameters.get_param_value("HOTSPOT_SECURITY"))

        # Get host spot configuration according HOTSPOT_PASSWORD
        self._hotspot_passphrase = \
            str(self._tc_parameters.get_param_value("HOTSPOT_PASSWORD"))

        # Get host spot configuration according HOTSPOT_STANDARD
        self._hotspot_standard = \
            self._tc_parameters.get_param_value("HOTSPOT_STANDARD")

        # Get flight mode configuration according FLIGHT_MODE
        self._flight_mode = \
            self._tc_parameters.get_param_value("FLIGHT_MODE")
        if self._flight_mode in (1, '1', 'on', 'ON'):
            self._flight_mode = 1
        elif self._flight_mode in (0, '0', 'off', 'OFF'):
            self._flight_mode = 0
        else:
            self._logger.warning("UC parameter 'flight_mode' ignored: %s."
                                 % (str(self._flight_mode)))
            self._flight_mode = 0

        # Get UECmdLayer
        self._networking_api = self._device.get_uecmd("Networking")

        # init original wifi power status for phone1
        self._original_wifi_power_status = 0
        # init original flight mode for phone1
        self._original_flight_mode = 0
        self._interface_ip = None

        # Set BT prerequisite (set ON, set OFF or nothing) for FIT BT/WIFI tests
        self._bt_fit_used, self._bt_wished_value = self._get_bt_fit_config()
        # Instantiate generic UECmd
        self._localconnectivity_api = self._device.get_uecmd("LocalConnectivity")
        self._bt_initial_state = 'STATE_ON'
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
        # store original wifi power status
        self._original_wifi_power_status = \
            self._networking_api.get_wifi_power_status()

        # store original flight mode
        time.sleep(self._wait_btwn_cmd)
        self._original_flight_mode = self._networking_api.get_flight_mode()

        # Enable/Disable flight mode
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_flight_mode(self._flight_mode)

        # disable wifi for Phone1
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_wifi_power("off")

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        # enable Portable wifi hotspot for Phone1
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_wifi_hotspot("on", self._hotspot_ssid,
                                              self._hotspot_security, self._hotspot_passphrase)

        # disable Portable wifi hotspot for Phone1
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_wifi_hotspot("off")

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        # restore original flight mode
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_flight_mode(self._original_flight_mode)

        # restore original wifi power status
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_wifi_power(self._original_wifi_power_status)

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
