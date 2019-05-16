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
:summary: This file implements the LIVE WIFI Base UC
:since: 30/03/2010
:author: cbresoli

"""

import time

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LiveWifiBase(UseCaseBase):

    """
        Live Wifi Test base class.
    """
    SUPPORTED_WIFI_STANDARD = ['a', 'b', 'g', 'n', 'an', 'bg', 'gb', 'bgn',
                               'ngb', 'n2.4G', 'n5G', 'ac', 'off']
    SUPPORTED_WIFI_BANDWIDTH = ["20", "40", "80"]

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        UseCaseBase.__init__(self, tc_name, global_config)

        # save global config into an attribute in
        # order to retrieve value in set_up method
        self.__global_config = global_config

        # Get FTP server parameters
        self._server = global_config.benchConfig.get_parameters("WIFI_SERVER")
        self._server_ip_address = self._server.get_param_value("IP")
        self._username = self._server.get_param_value("username")
        self._password = self._server.get_param_value("password")
        if self._server.has_parameter("ftp_path"):
            self._ftp_path = self._server.get_param_value("ftp_path")
        else:
            self._ftp_path = ""

        self._ip = None
        self._wifi_router_name = None
        self._standard = None
        self._ssid = ""
        self._standard_found = None
        self._bandwidth = None
        self._bandwidth_found = None
        self._security = None
        self._security_found = None
        self._passphrase = None
        self._wrong_passphrase = None
        self._isRouterNameFound = True
        self._isRouterFound = False
        self._connection_time = -1

        bench_config = self.__global_config.benchConfig
        # Retrieve wifi access point
        self._wifi_router_name = \
            str(self._tc_parameters.get_param_value("WIFI_ACCESS_POINT"))

        # if wifi router name is not specified, exit
        if self._wifi_router_name in [None, "", "NONE"]:

            self._isRouterNameFound = False

        elif(bench_config.has_parameter(self._wifi_router_name)
             and self.__global_config.benchConfig.
             get_parameters(self._wifi_router_name) not in (None, "")):

            self._isRouterFound = True
            self._wifirouter = self.__global_config.benchConfig.\
                get_parameters(self._wifi_router_name)

            ssid_found = self._wifirouter.get_param_value("SSID")
            self._standard_found = self._wifirouter.get_param_value("standard")
            if self._wifirouter.has_parameter("bandwidth"):
                self._bandwidth_found = self._wifirouter.get_param_value("bandwidth")
            self._security_found = self._wifirouter.\
                get_param_value("WIFI_SECURITY").upper()

            if ssid_found not in (None, ""):
                self._ssid = ssid_found

            if self._standard_found in self.SUPPORTED_WIFI_STANDARD:
                self._standard = self._standard_found

            if self._bandwidth_found in self.SUPPORTED_WIFI_BANDWIDTH:
                self._bandwidth = self._bandwidth_found

            # retrieve wifi router parameters
            if self._security_found in ("NONE", "OPEN"):

                self._logger.debug("OPEN" 'Wifi Security type selected,' +
                                   'getting parameters')
                self._security = self._security_found

            elif self._security_found in ("WEP", "WPA", "WPA2"):

                self._security = self._security_found
                self._logger.debug(str(self._security) +
                                   " Wifi Security selected, getting parameters")

                passphrase = self._wifirouter.get_param_value("passphrase")
                self._passphrase = passphrase

                # Get the optional DUT wrong passphrase to test connection failure
                self._wrong_passphrase = str(self._tc_parameters.
                                             get_param_value("WIFI_PASSPHRASE"))
                if (self._wrong_passphrase == "") or (self._wrong_passphrase.upper() == "NONE"):
                    self._wrong_passphrase = None

        # Set BT prerequisite (set ON, set OFF or nothing) for FIT BT/WIFI tests
        self._bt_fit_used, self._bt_wished_value = self._get_bt_fit_config()
        # BT initial state
        self._bt_initial_state = 'STATE_ON'

        # Get UECmdLayer
        self._networking_api = self._device.get_uecmd("Networking")
        self._localconnectivity_api = self._device.get_uecmd("LocalConnectivity")

        self._dut_wlan_iface = str(self._dut_config.get("wlanInterface"))
#------------------------------------------------------------------------------

    def set_up(self):

        UseCaseBase.set_up(self)

        # Check initialization and raise exception if something went wrong

        if not self._isRouterNameFound:
            msg = "No Wifi router has been specified by the user"
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, msg)

        if not self._isRouterFound:
            msg = "router name" + self._wifi_router_name + "is missing"
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, msg)

        if self._ssid in (None, ""):
            msg = str(self._ssid) + ": wifi ssid is missing"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._standard is None:
            # unknown wifi security
            if self._standard_found in ("", None):
                msg = "wifi standard is missing"
            else:
                msg = self._standard_found \
                    + ":Unknown wifi standard is missing"

            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._bandwidth is None:
            msg = "Bandwidth not found, use default value 20MHz"
            self._logger.warning(msg)
            self._bandwidth = "20"
        elif self._bandwidth not in ["20", "40", "80"]:
            msg = str(self._bandwidth) + "is invalid value for bandwidth"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        elif self._bandwidth in ["40", "80"] and self._standard in ['b', 'g', 'n', 'bg', 'gb', 'bgn', 'ngb', 'n2.4G']:
            msg = str(self._bandwidth) + "MHz bandwidth is not compatible with standard " + str(self._standard)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._security is None:
            # unknown wifi security
            if self._security_found in (None, ""):
                msg = "missing wifi security"
            else:
                msg = self._security_found + ": unknown wifi security"

            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if(self._security in ("WEP", "WPA", "WPA2")
           and self._passphrase in ("", None)):
            msg = "Passphrase %s is missing in bench configuration file"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # If Bt to be (de-)activated for FIT tests
        if self._bt_fit_used:

            self._bt_initial_state = self._localconnectivity_api.get_bt_power_status()

            # if Bt is not at the wished value, set it to the correct one.
            if self._bt_initial_state != self._bt_wished_value:
                self._localconnectivity_api.set_bt_power(self._bt_wished_value)
                time.sleep(self._wait_btwn_cmd)

        # Initialize the test
        self._networking_api.set_wifi_power("on")
        time.sleep(self._wait_btwn_cmd)

        # Set frequency band to "auto"
        self._networking_api.set_wifi_frequency_band("auto", True, self._dut_wlan_iface)
        time.sleep(self._wait_btwn_cmd)

        self._networking_api.clean_all_data_connections()
        time.sleep(self._wait_btwn_cmd)

        # 1st phase done, go to connection!

        if self._security in ("WEP", "WPA", "WPA2"):

            self._networking_api.set_wificonfiguration(self._ssid,
                                                       self._passphrase,
                                                       self._security)
            time.sleep(self._wait_btwn_cmd)

        start_time = time.time()
        self._networking_api.wifi_connect(self._ssid)

        if self._wrong_passphrase is None:
            self._networking_api.wait_for_wifi_dhcp_connected()
            end_time = time.time()
            self._connection_time = end_time - start_time

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)
        time.sleep(self._wait_btwn_cmd)

        self._networking_api.wifi_remove_config(self._ssid)
        time.sleep(self._wait_btwn_cmd)

        self._networking_api.set_wifi_power("off")

        # If Bt to be (de-)activated for FIT tests
        if self._bt_fit_used:

            bt_final_status = self._localconnectivity_api.get_bt_power_status()

            # if Bt is not at the initial value, set it to the correct one.
            if bt_final_status != self._bt_initial_state:
                # restore Bt in initial state
                self._localconnectivity_api.set_bt_power(self._bt_initial_state)
                time.sleep(self._wait_btwn_cmd)
        return Global.SUCCESS, "No errors"

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
