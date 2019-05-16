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
:summary: This file implements the LAB WIFI TETHERING BASE UC
:since: 22/09/2011
:author: szhen11
"""
import time

from acs_test_scripts.UseCase.Networking.LAB_WCDMA_BASE import LabWcdmaBase
from UtilitiesFWK.Utilities import Global
from Device.DeviceManager import DeviceManager


class LabWifiTetheringBase(LabWcdmaBase):

    """
    Lab Wifi Tethering base Test class.
    """
    _CHECK_CONNECTION_TIMEOUT = 20

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWcdmaBase.__init__(self, tc_name, global_config)

        # Get host spot configuration according HOTSPOT_SSID
        self._hotspot_ssid = \
            str(self._tc_parameters.get_param_value("HOTSPOT_SSID"))

        # Get host spot configuration according HOTSPOT_SECURITY
        self._hotspot_security = \
            str(self._tc_parameters.get_param_value("HOTSPOT_SECURITY"))

        # Get host spot configuration according HOTSPOT_PASSWORD
        self._hotspot_passphrase = \
            str(self._tc_parameters.get_param_value("HOTSPOT_PASSWORD"))

        # Get UECmdLayer
        self._networking_api = self._device.get_uecmd("Networking")

        self._phone2 = DeviceManager().get_device("PHONE2")
        if self._phone2 is not None:
            self._networking_api2 = self._phone2.get_uecmd("Networking")

        # init original wifi power status for phone1
        self._original_wifi_power_status = 0

        # Set BT prerequisite (set ON, set OFF or nothing) for FIT BT/WIFI tests
        self._bt_fit_used, self._bt_wished_value = self._get_bt_fit_config()
        # Instantiate generic UECmd
        self._localconnectivity_api = self._device.get_uecmd("LocalConnectivity")
        self._bt_initial_state = 'STATE_ON'

        # MAC Address of the STA device that will be connected to the softAP
        self._client_mac_address = ""

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # run setup inherit from LabWcdmaBase
        LabWcdmaBase.set_up(self)

        # Check if we have the second phone available
        if self._phone2 is None:
            return Global.FAILURE, "Cannot run that use case with only one phone configured."

        # Boot the other phone (the DUT is already booted)
        if not self._phone2.is_available():
            DeviceManager().boot_device("PHONE2")

        # If Bt to be (de-)activated for FIT tests
        if self._bt_fit_used:

            self._bt_initial_state = self._localconnectivity_api.get_bt_power_status()

            # if Bt is not at the wished value, set it to the correct one.
            if self._bt_initial_state != self._bt_wished_value:
                self._localconnectivity_api.set_bt_power(self._bt_wished_value)
                time.sleep(self._wait_btwn_cmd)

        # store original wifi power status
        time.sleep(self._wait_btwn_cmd)
        self._original_wifi_power_status = \
            self._networking_api.get_wifi_power_status()

        # disable wifi for Phone1
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_wifi_power("off")

        # enable Portable wifi hotspot for Phone1
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_wifi_hotspot("on", self._hotspot_ssid,
                                              self._hotspot_security, self._hotspot_passphrase)

        # set wifi power on for Phone2
        time.sleep(self._wait_btwn_cmd)
        self._networking_api2.set_wifi_power("on")

        # Clear all data connections for Phone2
        time.sleep(self._wait_btwn_cmd)
        self._networking_api2.clean_all_data_connections()

        # scan wifi for Phone2
        time.sleep(self._wait_btwn_cmd)
        self._networking_api2.request_wifi_scan()

        # set passphrase for Phone2
        if self._hotspot_security in "WPA2-PSK":
            self._networking_api2.set_wificonfiguration(self._hotspot_ssid,
                                                        self._hotspot_passphrase,
                                                        self._hotspot_security)

        # connect hotspot from Phone1 for Phone2
        time.sleep(self._wait_btwn_cmd)
        self._networking_api2.wifi_connect(self._hotspot_ssid)

        # Retrieve the PHONE2 MAC address
        time.sleep(self._wait_btwn_cmd)
        self._client_mac_address = self._networking_api2.get_interface_mac_addr()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        LabWcdmaBase.tear_down(self)

        status = Global.SUCCESS
        msg = "No error"

        # disconnect from hotspot for Phone2
        time.sleep(self._wait_btwn_cmd)
        self._networking_api2.\
            wifi_disconnect(self._hotspot_ssid)

        # set wifi power off for phone2
        time.sleep(self._wait_btwn_cmd)
        self._networking_api2.set_wifi_power("off")

        # disable Portable wifi hotspot for Phone1
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_wifi_hotspot("off")

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

        return status, msg

#------------------------------------------------------------------------------

    def _get_bt_fit_config(self):
        """
        Get BT configuration for FIT BT/WIFI tests

        :rtype: list of 2 elements (boolean, str)
        :return: true if BT/WIFI FIT is used, Bluetooth state ON or OFF for the test to be run
        """

        # Read WHILE_BLUETOOTH_ON parameter (named for retro-compatibility)
        # from test case xml file for FIT tests
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
