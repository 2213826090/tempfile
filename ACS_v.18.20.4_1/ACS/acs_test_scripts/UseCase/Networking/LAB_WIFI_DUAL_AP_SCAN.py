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

:summary: This file implements the LIVE WIFI SCAN UC. It check dual ap with
same name and same or different security mode.
:since: 28/11/2012
:author: smaurelx
"""
import time

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.UseCase.Networking.LAB_WIFI_DUAL_BASE import LabWifiDualBase
from acs_test_scripts.UseCase.Networking.LAB_WIFI_SCAN import LabWifiScan
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LabWifiDualAPScan(LabWifiDualBase, LabWifiScan):

    """
    Lab Wifi Scan Test class.
    """

    def __init__(self, tc_name, global_config):  # pylint: disable=W0231
        """
        Constructor
        """
        LabWifiDualBase.__init__(self, tc_name, global_config)

        if self._ssid != self._ssid_ap2:
            msg = "SSID AP2 name [%s] different of SSID AP1. Force SSID AP2 name to [%s]" \
                % (self._ssid_ap2, self._ssid)
            self._logger.warning(msg)
            self._ssid_ap2 = self._ssid

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiScan.set_up(self)

        # Initiate connection to the secondary Access Point equipment
        self._ns_ap2.init()

        config_index = 0
        if self._wifirouter_ip == self._wifirouter_ip_ap2:
            config_index = 1

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
                                     self._radiussecret,
                                     config_index)

        self._ns_ap2.enable_wireless()

        # Close the connection to the AP2
        self._ns_ap2.release()

        security_ap2 = "Unknown security value"
        # Extract the security Type
        if self._security_ap2 in ("NONE", "OPEN"):
            security_ap2 = "NONE"
        elif self._security_ap2 in ("WEP64", "WEP128"):
            security_ap2 = "WEP"
        elif self._security_ap2 in ("WPA-PSK-TKIP", "WPA2-PSK-AES",
                                    "WPA-PSK-AES", "WPA2-PSK-TKIP",
                                    "WPA2-PSK-TKIP-AES"):
            security_ap2 = "WPA"
        elif self._security_ap2 in ("EAP-WPA", "EAP-WPA2"):
            security_ap2 = self._security_ap2

        self._networking_api.set_wificonfiguration(self._ssid_ap2,
                                                   self._passphrase_ap2,
                                                   security_ap2,
                                                   self._ip_setting_ap2,
                                                   self._ip_address_ap2,
                                                   self._netmask_ap2,
                                                   self._gateway_ap2,
                                                   self._dns1_ap2,
                                                   self._dns2_ap2)

        if self._hidden:
            self._hidden = False
            msg = "The dual configurable AP [%s] can't be used on hidden mode. Force SSID visible." \
                % self._ssid
            self._logger.warning(msg)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        # Force 10s settling time before scanning to give enough time to scan.
        time.sleep(10)
        self._networking_api.request_wifi_scan()

        time.sleep(self._wait_btwn_cmd)
        ssids, capabilities = self._networking_api.list_ssids_and_capabilities(
            "wifi", "all")

        current = 0
        while current < len(ssids):
            ssids[current] += " $ " + capabilities[current]
            current += 1

        ssids = list(set(ssids))

        local_count = 0
        formated_ssid = self._ssid + " $ "
        for cur_ssid in ssids:
            if cur_ssid.startswith(formated_ssid):
                local_count += 1

        same_security_mode = self._security_ap2 == self._security

        if local_count == 1:
            if same_security_mode:
                msg = "No error, dual configurable AP successfully found only one AP [%s - %s]" % (self._ssid, self._security)
            else:
                msg = "cannot find the dual configurable AP [%s] (only one found)" % self._ssid
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        elif local_count == 2:
            if same_security_mode:
                msg = "cannot find the dual configurable AP [%s] (two found)" % self._ssid
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                msg = "No error, dual configurable AP successfully found"
        else:
            msg = "cannot find the dual configurable AP [%s]" % self._ssid
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        self._logger.info(msg)

        return Global.SUCCESS, msg
