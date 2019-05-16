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
:summary: This file implements the LAB WIFI DUAL AP SWITCH NEXT TO AP LOSS
:since: 20/04/2012 BZ2828
:author: apairex
"""

from acs_test_scripts.UseCase.Networking.LAB_WIFI_DUAL_BASE import LabWifiDualBase
from UtilitiesFWK.Utilities import Global

import time
import os
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabWifiDualApSwitchApLoss(LabWifiDualBase):

    """
    Lab Wifi Dual AP Switch Next to AP Loss Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiDualBase.__init__(self, tc_name, global_config)

        # Do we use the same SSID on both Access Points?
        self._same_ssid = self._tc_parameters.get_param_value("AP2_USES_SAME_SSID_AND_SECURITY")
        if str(self._same_ssid).upper() in ("YES", "TRUE"):
            self._same_ssid = True
            # Overwrite LabWifiDualBase parameter initialization
            self._ssid_ap2 = self._ssid
            self._security_ap2 = self._security
            self._passphrase_ap2 = self._passphrase
        else:
            self._same_ssid = False

        # Do we test roaming while FTP transfer?
        self._ftp_transfer = self._tc_parameters.\
            get_param_value("SWITCH_BETWEEN_AP_DURING_FTP_TRANSFER")
        if str(self._ftp_transfer).upper() in ("YES", "TRUE"):
            self._ftp_transfer = True
        else:
            self._ftp_transfer = False

        # Get UECmdLayer
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")

        # Parameter for ping controls
        self._packetsize = 32
        self._packnb = 4
        self._lossrate = 25

        # Parameter for FTP transfer
        self._direction = "DL"
        self._dl_filename_param = self._tc_parameters.get_param_value("FTP_DL_FILENAME")
        self._xfer_timeout = self._tc_parameters.get_param_value("XFER_TIMEOUT")
        self._dl_filename = os.path.join(self._ftp_path, self._dl_filename_param)

        self._dut_mac_address_for_connection_check = ""

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """

        LabWifiDualBase.set_up(self)

        # Check TC parameters
        if self._ftp_transfer and not self._dl_filename_param:
            msg = "Missing TC parameter: FTP_DL_FILENAME: " + str(self._dl_filename_param)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._xfer_timeout in (None, ""):
            self._xfer_timeout = 200
        else:
            self._xfer_timeout = float(self._xfer_timeout)

        # Set Wifi sleep policy to "Never Sleep"
        self._networking_api.set_wifi_sleep_policy(self._networking_api.WIFI_SLEEP_POLICY["NEVER"])

        # Warn the user if the AP are not Cisco 12xx
        if not self._ns.get_model().startswith("CISCO_12") \
                or not self._ns_ap2.get_model().startswith("CISCO_12"):
            self._logger.warning("You will have better reliability using 2 CISCO 12xx APs")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiDualBase.run_test(self)

        if self._ftp_transfer:
            self.__run_test_ftp()
        else:
            self.__run_test_ping()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __run_test_ping(self):
        """
        Execute the test with Ping data control
        """
        # Switch on AP1 and off AP2 Wifi
        self.__activate_access_point(1, self._dut_mac_address_for_connection_check)

        # Checks AP #1 is connected
        self._networking_api.check_connection_state(self._ssid)

        # Checks data is enabled
        self._check_IP_address_attributed(1)

        # Checks connection using Ping
        packetloss = self._networking_api.ping(self._wifirouter_ip,
                                               self._packetsize, self._packnb)
        if packetloss.value > self._lossrate:
            msg = "AP1: Ping command fails (%s% loss)" % str(packetloss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)

        # Switch on AP2 Wifi and switch off AP1
        self.__activate_access_point(2, self._dut_mac_address_for_connection_check)

        # Checks AP #2 is connected
        self._networking_api.check_connection_state(self._ssid_ap2)

        # Checks data is enabled
        self._check_IP_address_attributed(2)

        # Checks connection using Ping
        packetloss = self._networking_api.ping(self._wifirouter_ip_ap2,
                                               self._packetsize, self._packnb)
        if packetloss.value > self._lossrate:
            msg = "AP2: Ping command fails (%s% loss)" % str(packetloss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)

    def __run_test_ftp(self):
        """
        Execute the test with FTP transfer
        """
        # Switch on AP1 and off AP2 Wifi
        self.__activate_access_point(1, self._dut_mac_address_for_connection_check)

        # Checks AP #1 is connected
        self._networking_api.check_connection_state(self._ssid)

        # Checks data is enabled
        self._check_IP_address_attributed(1)

        # Start FTP transfer
        task_id = self._networking_api.start_ftp_xfer(self._direction,
                                                      self._ftp_ip_address,
                                                      self._ftp_username,
                                                      self._ftp_password,
                                                      self._dl_filename,
                                                      self._device.get_ftpdir_path())
        start = time.time()

        # Switch on AP2 Wifi and switch off AP1
        self.__activate_access_point(2, self._dut_mac_address_for_connection_check)

        # Checks AP #2 is connected
        self._networking_api.check_connection_state(self._ssid_ap2)

        # Checks data is enabled
        self._check_IP_address_attributed(2)

        # Checks FTP transfer is still ongoing
        ftp_status = self._networking_api.get_ftp_xfer_status()

        if ftp_status != self._networking_api.FTP_TRANSFERRING:
            if self._networking_api.is_ftp_xfer_success(self._dl_filename,
                                                        self._direction,
                                                        task_id):
                msg = "File has already been transferred. Please use a bigger file for this test!"
            else:
                msg = "FTP transfer STOPS when roaming happened. status=%s. " % str(ftp_status)

            # End the transfer
            self._networking_api.stop_ftp_xfer(task_id)

            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Wait for the FTP transfer to end.
        while start + self._xfer_timeout > time.time() \
                and ftp_status == self._networking_api.FTP_TRANSFERRING:
            time.sleep(5)
            ftp_status = self._networking_api.get_ftp_xfer_status()

        # Does Timeout occur?
        if ftp_status == self._networking_api.FTP_TRANSFERRING:
            # End the transfer
            self._networking_api.stop_ftp_xfer(task_id)

            msg = "FTP Transfer timeout. Please use a smaller file or increase the timeout value"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Control that the FTP transfer was success
        if not self._networking_api.is_ftp_xfer_success(self._dl_filename,
                                                        self._direction,
                                                        task_id):
            # End the transfer
            self._networking_api.stop_ftp_xfer(task_id)

            msg = "FTP transfer FAILS"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def __activate_access_point(self, ap_to_activate, mac_for_connection_check=""):
        """
        Function to stop 1 Access Point and to start the other.
        This function integrates the connection initialization to the AP control
        because actions between AP controls in this test case can be longer than
        the connection timeout to AP.

        :type ap_to_activate: int or str 'both'
        :param ap_to_activate: id of the AP that should be activated.
                                The other one has to be disabled (except in "both" case).
        :type mac_for_connection_check: String
        :param mac_for_connection_check: DUT MAC address to check that the connected devices
                                        list of enabled AP contains the DUT MAC address.
        """
        check_timeout = 120

        if ap_to_activate == 1:

            # Switch on AP1 wifi and switch off AP2 wifi
            self._ns.init()
            self._ns.enable_wireless()
            time.sleep(self._wait_btwn_cmd)

            self._ns_ap2.init()
            self._ns_ap2.disable_wireless()
            self._ns_ap2.release()

            if mac_for_connection_check:
                # Wait for the client to be connected to the enabled Access Point
                start = time.time()
                client_macaddr_list = self._ns.get_client_macaddr_list()
                while mac_for_connection_check not in client_macaddr_list \
                        and start + check_timeout > time.time():
                    time.sleep(2)
                    client_macaddr_list = self._ns.get_client_macaddr_list()

            self._ns.release()

        elif ap_to_activate == 2:

            # Switch on AP2 wifi and switch off AP1 wifi
            self._ns_ap2.init()
            self._ns_ap2.enable_wireless()
            time.sleep(self._wait_btwn_cmd)

            self._ns.init()
            self._ns.disable_wireless()
            self._ns.release()

            if mac_for_connection_check:
                # Wait for the client to be connected to the enabled Access Point
                start = time.time()
                client_macaddr_list = self._ns_ap2.get_client_macaddr_list()
                while mac_for_connection_check not in client_macaddr_list \
                        and start + check_timeout > time.time():
                    time.sleep(2)
                    client_macaddr_list = self._ns_ap2.get_client_macaddr_list()

            self._ns_ap2.release()

        elif ap_to_activate == 'both':

            # Switch on both AP1 and AP2 Wifi
            self._ns.init()
            self._ns.set_wifi_standard(self._standard)
            self._ns.enable_wireless()
            self._ns.release()
            time.sleep(self._wait_btwn_cmd)

            self._ns_ap2.init()
            self._ns_ap2.set_wifi_standard(self._standard_ap2)
            self._ns_ap2.enable_wireless()
            self._ns_ap2.release()

        else:
            msg = "__activate_access_point(): bad parameter [%s]" \
                % str(ap_to_activate)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

    def _check_IP_address_attributed(self, ap_number):
        """
        In case of a connection using a DHCP server we need to ensure that the IP address
        has been correctly attributed, in order for the data transfer to be enabled

        :type ap_number: int
        :param ap_number: Number (1 or 2) of the AP, DUT is connected to.
        """
        # Get the IP setting depending on the AP the DUT is connected to
        ip_setting = "static"
        if ap_number == 1:
            ip_setting = self._ip_setting
        elif ap_number == 2:
            ip_setting = self._ip_setting_ap2

        # Only check if IP has been delivered when DHCP configuration using IPv4
        if not self._use_ipv6 and ip_setting == "dhcp":
            self._networking_api.wait_for_wifi_dhcp_connected()
