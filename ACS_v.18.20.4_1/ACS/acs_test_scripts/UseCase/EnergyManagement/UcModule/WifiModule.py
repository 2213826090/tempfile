"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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
:summary: This module control the WIFI, it allow advanced wifi operation like registering to AP
:author: vgomberx
:since: 27/01/2015
"""
from time import sleep
from acs_test_scripts.UseCase.EnergyManagement.UcModule.OverMind import OverMind
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from ErrorHandling.AcsConfigException import AcsConfigException


class WifiModule():
    """
    init.
    """
    __LOG_TAG = "[WIFI_MODULE]\t"
    __SECURTIY_MODE = ("NONE", "OPEN", "WEP", "WPA", "WPA2")

    def __init__(self):
        """
        parameter to initialize this module.
        :type mode: str
        """
        self.__logger = LOGGER_TEST_SCRIPT
        self.__logger.info(self.__LOG_TAG + "INIT")
        overmind = OverMind()
        device = overmind.get_instance(overmind.DEVICE)
        tc_parameters = overmind.get_instance(overmind.TC_PARAMETERS)
        self.__bench_config = overmind.get_instance(overmind.BENCH_CONFIG)

        #-----------------------------------------------------------------------
        self.__ssid = ""
        self.__security = None
        self.__security_found = None
        self.__passphrase = None
        self.__is_router_name_found = False
        self.__is_router_found = False
        self.__setup_needed = True
        self.__wifi_router_name = None

        # Retrieve wifi access point from testcase
        wifi_router_name_from_tc = tc_parameters.get_param_value("WIFI_ACCESS_POINT", "")
        if not wifi_router_name_from_tc in [ None, "", "NONE"]:
            self.configure_wifi_ap(wifi_router_name_from_tc)
        # Get UECmdLayer
        self.__networking_api = device.get_uecmd("Networking", True)

    def is_wifi_ap_defined(self):
        """
        return if a wifi access point has been well defined for this module.
        By defined, it means with all necessary parameter ( security , pass phrase etc....)
        Used by load system.

        :rtype: boolean
        :return: True if an access point has been defined , False otherwise
        """
        return self.__is_router_found

    def configure_wifi_ap(self, wifi_ap):
        """
        configure the wifi ap to use.
        """
        self.__logger.debug(self.__LOG_TAG + "Configure wifi AP to use : %s" % (wifi_ap))
        # check if the access point info are present in bench config
        if wifi_ap not in ["", None, "NONE"] :
            # set the potential router name
            self.__wifi_router_name = wifi_ap
            if self.__bench_config.has_parameter(wifi_ap):
                self.__is_router_name_found = True
                # get router info from bench config
                wifi_router = self.__bench_config.get_parameters(wifi_ap)
                self.__is_router_found = True
                self.__ssid = wifi_router.get_param_value("SSID", "")
                self.__security_found = str(wifi_router.get_param_value("WIFI_SECURITY", "")).upper()

                # retrieve wifi router parameters
                if self.__security_found in self.__SECURTIY_MODE:
                    self.__security = self.__security_found
                    if self.__security_found in ("WEP", "WPA", "WPA2"):
                        self.__logger.debug(self.__LOG_TAG + str(self.__security) + " Wifi Security selected, getting parameters")
                        self.__passphrase = wifi_router.get_param_value("passphrase", "")

    def set_up(self):
        """
        turn wifi on an setup the connection
        """
        # Check initialization and raise exception if something went wrong
        msg = ""
        if not self.__is_router_name_found:
            msg = "No Wifi router name has been specified by the user"
        elif not self.__is_router_found:
            msg = "router name" + self.__wifi_router_name + "is missing from bench configuration"
        elif self.__ssid in (None, ""):
            msg = str(self.__ssid) + ": wifi ssid is missing from bench configuration"
        elif self.__security is None:
            # unknown wifi security
            if self.__security_found in (None, ""):
                msg = "missing wifi security from bench configuration"
            else:
                msg = "unknown wifi security : %s" % str(self.__security_found)
        elif self.__security in ("WEP", "WPA", "WPA2") and self.__passphrase in ("", None):
            msg = "Wifi Passphrase %s is missing from bench configuration file"

        if msg != "":
            msg = self.__LOG_TAG + msg
            self.__logger.error()
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # turn on wifi and clean old connection
        self.__networking_api.set_wifi_power("on")
        sleep(2)
        self.__networking_api.clean_all_data_connections()
        if self.__security in ("WEP", "WPA", "WPA2"):
            self.__networking_api.set_wificonfiguration(self.__ssid,
                                                       self.__passphrase,
                                                       self.__security)

        self.__setup_needed = False

    def wifi_connect(self, check_if_setup_need=False):
        """
        connect the wifi

        :type check_if_setup_need: boolean
        :param check_if_setup_need: if set to true, the module will check if a setup is needed and will perform it
        """
        if check_if_setup_need and self.__setup_needed:
            self.set_up()

        self.__networking_api.wifi_connect(self.__ssid)
        self.__networking_api.wait_for_wifi_dhcp_connected()

    def wifi_disconnect(self):
        """
        disconnect the wifi
        """
        self.__networking_api.wifi_disconnect_all()

    def release(self):
        """
        release the wifi connection and turn wifi off
        """
        self.__networking_api.set_wifi_power("on")
        sleep(2)
        self.__networking_api.clean_all_data_connections()
        self.__setup_needed = True
        self.__networking_api.set_wifi_power("off")
