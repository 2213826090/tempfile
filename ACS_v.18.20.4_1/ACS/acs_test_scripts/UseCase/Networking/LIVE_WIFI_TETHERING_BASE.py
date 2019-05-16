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
:summary: This file implements the LIVE WIFI TETHERING BASE UC
:since: 26/06/2012
:author: jpstierlin BZ3235
"""

import time

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global, str_to_bool_ex
from Device.DeviceManager import DeviceManager
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.AcsBaseException import AcsBaseException


class LiveWifiTetheringBase(UseCaseBase):

    """
    Live Wifi Tethering base Test class.
    """
    _CHECK_CONNECTION_TIMEOUT = 20

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get FTP server parameters
        self._wifi_server = global_config.benchConfig.get_parameters("WIFI_SERVER")
        self._ftp_ip_address = self._wifi_server.get_param_value("IP")
        self._ftp_username = self._wifi_server.get_param_value("username")
        self._ftp_password = self._wifi_server.get_param_value("password")
        if self._wifi_server.has_parameter("ftp_path"):
            self._ftp_path = self._wifi_server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""

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
            str(self._tc_parameters.get_param_value("FLIGHT_MODE"))
        self._flight_mode = str_to_bool_ex(self._flight_mode)

        # Get Data connection mode
        self._is_pdp_context_activated = \
            str(self._tc_parameters.get_param_value("IS_PDP_CONTEXT_ACTIVATED"))
        self._is_pdp_context_activated = str_to_bool_ex(self._is_pdp_context_activated)
        # Default is True
        if self._is_pdp_context_activated == None:
            self._is_pdp_context_activated = True

        # Get the optional DUT wrong passphrase to test connection failure
        self._wrong_passphrase = self._tc_parameters. \
            get_param_value("WIFI_PASSPHRASE")

        # Get computer type
        self._computer = self._tc_parameters.get_param_value("COMPUTER")
        if self._computer == "":
            self._computer = None  # empty computer: use second DUT

        # Get UECmdLayer
        self._networking_api = self._device.get_uecmd("Networking")

        # Load computer equipment
        if self._computer is not None:
            self._computer = self._em.get_computer(self._computer)
            self._wifi_interface = self._computer.get_wifi_interface()
            if self._wifi_interface == "":
                self._wifi_interface = None
        else:
            self._wifi_interface = None

        # Get PHONE2
        self._phone2 = None
        self._networking_api2 = None
        if self._computer is None:
            self._phone2 = DeviceManager().get_device("PHONE2")
            if self._phone2 is not None:
                self._networking_api2 = self._phone2.get_uecmd("Networking")

        # init original wifi power status for phone1
        self._original_wifi_power_status = 0
        # init original flight mode for phone1
        self._original_flight_mode = 0
        self._interface_ip = None

        self._hotspot_ext_interface = \
            str(self._dut_config.get("hotspotExtInterface"))

        # Set BT prerequisite (set ON, set OFF or nothing) for FIT BT/WIFI tests
        self._bt_fit_used, self._bt_wished_value = self._get_bt_fit_config()
        # BT initial state
        self._bt_initial_state = 'STATE_ON'
        # Instantiate generic UECmd
        self._localconnectivity_api = self._device.get_uecmd("LocalConnectivity")

        # MAC Address of the STA device that will be connected to the softAP
        self._client_mac_address = ""

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # pylint: disable=E1101
        # run setup inherit from UseCaseBase
        UseCaseBase.set_up(self)

        # Check if we have second wifi interface or second phone available
        if (self._computer is None or self._wifi_interface is None) \
                and self._phone2 is None:
            msg = "Cannot run that use case without a remote PC or with only one phone configured."
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

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

        # store original flight mode
        time.sleep(self._wait_btwn_cmd)
        self._original_flight_mode = self._networking_api.get_flight_mode()

        # Enable/Disable flight mode
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_flight_mode(self._flight_mode)

        # Enable Cellular Data connection
        if self._flight_mode == False and self._is_pdp_context_activated:
            time.sleep(self._wait_btwn_cmd)
            self._networking_api.activate_pdp_context()

        # disable wifi for Phone1
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_wifi_power("off")

        # enable Portable wifi hotspot for Phone1
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_wifi_hotspot("on", self._hotspot_ssid,
                                              self._hotspot_security, self._hotspot_passphrase)

        if self._wrong_passphrase in (None, "", "NONE", "None"):
            passphrase = self._hotspot_passphrase
            wrong_passphrase = False
        else:
            passphrase = self._wrong_passphrase
            wrong_passphrase = True

        if self._computer is not None:
            try:
                self._interface_ip = \
                    self._computer.wifi_connect(self._wifi_interface,
                                                self._hotspot_standard,
                                                self._hotspot_ssid,
                                                self._hotspot_security,
                                                passphrase)
                if wrong_passphrase:
                    msg = "Connection successful with wrong passphrase."
                    self._logger.error(msg)
                    raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
            except TestEquipmentException as exc:
                # ignore the exception if the connection fails with a wrong passphrase
                if not wrong_passphrase:
                    raise
                elif not "Could not get IP address" in str(exc) \
                        and not "Could not associate device" in str(exc):
                    raise

            # Retrieve the MAC address of the COMPUTER
            time.sleep(self._wait_btwn_cmd)
            self._client_mac_address = self._computer.get_interface_mac_addr(self._wifi_interface)
            self._client_mac_address.lower()

        elif self._phone2 is not None:
            # Boot the other phone (the DUT is already booted)
            if not self._phone2.is_available():
                DeviceManager().boot_device("PHONE2")

            # set wifi power on for Phone2
            time.sleep(self._wait_btwn_cmd)
            self._networking_api2.set_wifi_power("on")

            # set auto connect state off for Phone2
            time.sleep(self._wait_btwn_cmd)
            self._networking_api2.\
                set_autoconnect_mode("all",
                                     self._uecmd_types.AUTO_CONNECT_STATE.off)

            # disconnect all wifi for Phone2
            time.sleep(self._wait_btwn_cmd)
            self._networking_api2.wifi_disconnect_all()

            # set passphrase for Phone2
            self._networking_api2.set_wificonfiguration(self._hotspot_ssid,
                                                        passphrase,
                                                        self._hotspot_security)

            # connect hotspot from Phone1 for Phone2
            time.sleep(self._wait_btwn_cmd)
            try:
                self._networking_api2.wifi_connect(self._hotspot_ssid)
                if wrong_passphrase:
                    msg = "Connection successful with wrong passphrase."
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            except AcsBaseException as exc:
                # ignore the exception if the connection fails with a wrong passphrase
                if not wrong_passphrase:
                    raise
                elif not "timeout" in str(exc):
                    raise

            # Retrieve the MAC address of the PHONE2
            time.sleep(self._wait_btwn_cmd)
            self._client_mac_address = self._networking_api2.get_interface_mac_addr().lower()

        else:
            msg = "Cannot run that use case without a remote PC or with only one phone configured."
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        status = Global.SUCCESS
        msg = "No error"

        if self._computer is not None:
            self._computer.wifi_disconnect(self._wifi_interface)

        elif self._phone2 is not None:
            # disconnect from hotspot for Phone2
            time.sleep(self._wait_btwn_cmd)
            self._networking_api2.\
                wifi_disconnect(self._hotspot_ssid)

            time.sleep(self._wait_btwn_cmd)
            self._networking_api2.wifi_remove_config(self._hotspot_ssid)

            # set wifi power off for phone2
            time.sleep(self._wait_btwn_cmd)
            self._networking_api2.set_wifi_power("off")

        # disable Portable wifi hotspot for Phone1
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.set_wifi_hotspot("off")

        # restore original flight mode
        if str_to_bool_ex(self._original_flight_mode) != str_to_bool_ex(self._flight_mode):
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
