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
:summary: This file implements the LAB WIFI DIRECT BASE UC
:since: 06/06/2013
:author: smaurel
"""

from Device.DeviceManager import DeviceManager
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global


class LabWifiDirectBase(UseCaseBase):
    """
    Lab Wifi Direct class.
    """

    DEFAULT_REGULATORY_DOMAIN = "US"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        self._device1_name = self._tc_parameters.get_param_value("DEVICE1_NAME", "")
        self._device2_name = self._tc_parameters.get_param_value("DEVICE2_NAME", "")

        # Get P2p interface name
        self._device1_p2pinterface = str(self._dut_config.get("p2pInterface"))

        self._device1_supplicant = None
        self._device1_client = None
        self._device1_mac = None
        self._device2_supplicant = None
        self._device2_client = None
        self._device2_mac = None
        self._phone2 = None
        self._networking2_api = None

        self._device2_p2pinterface = None
        if self._device2_name.startswith("PHONE"):
            self.dut_config2 = DeviceManager().get_device_config("PHONE2")
            self._device2_p2pinterface = str(self.dut_config2.get("p2pInterface"))

            # Get Device 2 Instance
            self._phone2 = DeviceManager().get_device(self._device2_name)
            self._networking2_api = self._phone2.get_uecmd("Networking")

        self._networking_api = self._device.get_uecmd("Networking")

        self._dut1_wlan_iface = str(self._dut_config.get("wlanInterface"))
        self._dut2_wlan_iface = str(self.dut_config2.get("wlanInterface"))
    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        self.__check_tc_parameters()

        if self._phone2 is not None and not self._phone2.is_available():
            DeviceManager().boot_device(self._device2_name)

        # set wifi On on the DUT to allow regulatory domain to change
        self._networking_api.set_wifi_power("on")
        if self._networking2_api is not None:
            self._networking2_api.set_wifi_power("on")

        # set the regulatory domain
        self._networking_api.set_regulatorydomain(LabWifiDirectBase.DEFAULT_REGULATORY_DOMAIN, self._dut1_wlan_iface)
        if self._networking2_api is not None:
            self._networking2_api.set_regulatorydomain(LabWifiDirectBase.DEFAULT_REGULATORY_DOMAIN,
                                                       self._dut2_wlan_iface)

        # set wifi Off on the DUT
        self._networking_api.set_wifi_power("off")
        if self._networking2_api is not None:
            self._networking2_api.set_wifi_power("off")

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def _get_supplicant_instance(self, device_name):
        """
        Get the p2p supplicant instance for the device

        :rtype: UECmd Object or Equipement Object
        :return: The p2p supplicant instance for the device
        """
        device_instance = self.__get_device_instance(device_name)

        if device_name.startswith("PHONE"):
            return device_instance.get_uecmd("P2PSupplicantCLI")

        if device_name.startswith("COMPUTER"):
            return device_instance.get_p2p("P2pSupplicant")

        msg = "device not found " + device_name
        self._logger.error(msg)
        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

    def _get_client_instance(self, device_name):
        """
        return the p2p client instance for the device
        """
        device_instance = self.__get_device_instance(device_name)

        if device_name.startswith("PHONE"):
            return device_instance.get_uecmd("P2PClientCLI")

        if device_name.startswith("COMPUTER"):
            return device_instance.get_p2p("P2pClient")

        msg = "device not found " + device_name
        self._logger.error(msg)
        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

    def __get_device_instance(self, device_name):
        """
        retrieve the device instance
        """
        if device_name.startswith("COMPUTER"):
            return self._em

        if device_name == "PHONE1":
            return self._device

        if device_name.startswith("PHONE"):
            if not self._phone2.is_available():
                DeviceManager().boot_device(device_name)

            return self._phone2

        msg = "device not found " + device_name
        self._logger.error(msg)
        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

    def __check_tc_parameters(self):
        """
        Checks all TC parameters
        """
        if not self._device1_name:
            msg = "undefined device name 1."
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if not self._device2_name:
            msg = "undefined device name 2."
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
