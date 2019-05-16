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
:summary: This file implements the LAB WIFI DUAL AP BAND SELECTION SCAN
        In this UC, we control that only 5GHz AP is visible when band selection
        option is set to "5GHz" and that only 2.4GHz AP is visible when band
        selection is set to "2.4GHz" option.
:since: 26/09/2012 BZ2828
:author: apairex
"""
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.UseCase.Networking.LAB_WIFI_DUAL_BASE import LabWifiDualBase
from UtilitiesFWK.Utilities import Global

import time
import os
from datetime import datetime
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabWifiDualApBandSelScan(LabWifiDualBase):

    """
    Lab Wifi Dual AP Frequency Band Selection class.

    This UseCase allows to test Wifi network scan when using Frequency Band
    selection.
    This test has been designed in order to be able to loop on every possible
    configurations.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiDualBase.__init__(self, tc_name, global_config)

        # AP must not be hidden
        self._hidden = False
        self._hidden_ap2 = False

        # Get the parameter test phases
        self._2_4ghz_test = self._tc_parameters.\
            get_param_value("RUN_2_4_GHZ_TEST")
        self._5ghz_test = self._tc_parameters.get_param_value("RUN_5_GHZ_TEST")
        self._auto_test = self._tc_parameters.get_param_value("RUN_AUTO_TEST")

        if str(self._2_4ghz_test).lower() in ["1", "true"]:
            self._2_4ghz_test = True
        else:
            self._2_4ghz_test = False
        if str(self._5ghz_test).lower() in ["1", "true"]:
            self._5ghz_test = True
        else:
            self._5ghz_test = False
        if str(self._auto_test).lower() in ["1", "true"]:
            self._auto_test = True
        else:
            self._auto_test = False

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

        # Control that the TC parameters has been correctly set.
        if self._standard not in ["a", "n5G"]:
            msg = "AP1 is not set to 5GHz. Standard set: %s" % self._standard
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if self._standard_ap2 not in ["gb", "ngb", "n2.4G"]:
            msg = "AP2 is not set to 2.4GHz. Standard set: %s" \
                % self._standard_ap2
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if not self._2_4ghz_test and not self._5ghz_test \
                and not self._auto_test:
            msg = "At least 1 test must be activated. Nothing to do."
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        if self._channel != "36":
            self._logger.warning("AP1 channel is not 36 as recommended: %s"
                                 % self._channel)
        if self._channel_ap2 != "1":
            self._logger.warning("AP2 channel is not 1 as recommended: %s"
                                 % self._channel_ap2)

        # Initiate connection to the equipment
        self._ns.init()

        if self._sniffer:
            self._sniffer.init()

        # Configure the 5GHz Access Point
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

        self._ns.enable_wireless()

        # Turn Wifi interface ON
        self._networking_api.set_wifi_power("on")
        time.sleep(self._wait_btwn_cmd)

        # Regulatory Domain management
        self._set_regulatory_domain('DUT')

        # Close the connection to AP
        self._ns.release()

        # Initiate connection to the secondary Access Point equipment
        self._ns_ap2.init()

        # Configure the 2.4GHz Access Point
        self._ns_ap2.set_wifi_config(self._ssid_ap2,
                                     self._hidden_ap2,
                                     self._standard_ap2,
                                     self._security_ap2,
                                     self._passphrase_ap2,
                                     self._channel_ap2,
                                     self._dtim_ap2,
                                     self._beacon_ap2,
                                     self._wmm_ap2,
                                     self._bandwidth_ap2,
                                     self._wifi_mimo_ap2,
                                     self._radiusip,
                                     self._radiusport,
                                     self._radiussecret)
        self._ns_ap2.enable_wireless()

        # Close the connection to the AP2
        self._ns_ap2.release()

        # Configure the DUT
        self._networking_api.clean_all_data_connections()

        # start sniffing if requested
        if self._sniffer:
            # build a unique capture file name with a timestamp inside
            # the Campaign Report directory
            filename = "capture-%s.cap" % datetime.now().strftime("%Hh%M.%S")
            pathname = self._device.get_report_tree().get_report_path()
            self._capture = os.path.join(pathname, filename)
            # start capturing
            self._sniffer.start_capture(self._channel, self._capture)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiDualBase.run_test(self)

        if self._auto_test:

            ssids = self.__run_band_scan("auto")

            if self._ssid not in ssids:
                msg = "auto: Unable to scan 2.4GHz AP"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            if self._ssid_ap2 not in ssids:
                msg = "auto: Unable to scan 5GHz AP"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if self._2_4ghz_test:

            ssids = self.__run_band_scan("2.4GHz")

            if self._ssid_ap2 not in ssids:
                msg = "2.4GHz: Unable to scan 2.4GHz AP"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            if self._ssid in ssids:
                msg = "2.4GHz: Able to scan 5GHz AP"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if self._5ghz_test:

            ssids = self.__run_band_scan("5GHz")

            if self._ssid not in ssids:
                msg = "5GHz: Unable to scan 5GHz AP"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            if self._ssid_ap2 in ssids:
                msg = "5GHz: Able to scan 2.4GHz AP"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        # stop sniffing
        if self._sniffer:
            if self._capture:
                self._sniffer.stop_capture(self.test_ok)
            self._sniffer.release()

        # Reset the Frequency Band parameter to "auto"
        self._networking_api.set_wifi_frequency_band("auto", False, self._dut_wlan_iface)
        self._networking_api.set_wifi_power("off")

        # Disable radio for all Wifi APs
        self._disable_all_radios()

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

    def __run_band_scan(self, freq_band="auto"):
        """
        Sets the WIFI frequency band, then perform a scan and return the result

        :type freq_band: str
        :param freq_band: can be "auto", "2.4GHz", "5GHz"

        :return: list of SSIDs returned by the scan
        """
        self._logger.info("Run %s Band Selection test" % freq_band)

        # Configure Wifi interface to scan given networks
        self._networking_api.set_wifi_frequency_band(freq_band, False, self._dut_wlan_iface)

        # Wait for the list to be updated
        time.sleep(90)

        # Scan Wifi networks
        self._networking_api.request_wifi_scan()
        time.sleep(self._wait_btwn_cmd)
        ssids = self._networking_api.list_ssids()

        return ssids
