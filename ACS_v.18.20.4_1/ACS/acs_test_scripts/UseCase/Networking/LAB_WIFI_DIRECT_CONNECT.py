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
:summary: This file implements the LAB WIFI DIRECT CONNECT UC
:since: 19/06/2013
:author: smaurel
"""

import time

from acs_test_scripts.UseCase.Networking.LAB_WIFI_DIRECT_BASE import LabWifiDirectBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.NetworkingUtilities import AcsWifiFrequencies
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabWifiDirectConnect(LabWifiDirectBase):
    """
    Lab Wifi Direct connect class.
    """

    P2P_CONNECTION_MODE = ["PBC", "PIN"]

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiDirectBase.__init__(self, tc_name, global_config)

        self._connection_type = self._tc_parameters.get_param_value("CONNECTION_MODE")

        self._device1_ip = self._tc_parameters.get_param_value("DEVICE1_IP")
        self._device1_go = self._tc_parameters.get_param_value("DEVICE1_GROUP_OWNER")
        self._device1_p2p_name = self._tc_parameters.get_param_value("DEVICE1_P2P_NAME")
        self._device1_channel = self._tc_parameters.get_param_value("DEVICE1_CHANNEL", "1")

        self._device2_ip = self._tc_parameters.get_param_value("DEVICE2_IP")
        self._device2_go = self._tc_parameters.get_param_value("DEVICE2_GROUP_OWNER")
        self._device2_host_interface = self._tc_parameters.get_param_value("DEVICE2_HOST_INTERFACE")
        self._device2_p2p_name = self._tc_parameters.get_param_value("DEVICE2_P2P_NAME")
        self._device2_channel = self._tc_parameters.get_param_value("DEVICE2_CHANNEL", "1")

        self._device1_frequency = 0
        self._device2_frequency = 0

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiDirectBase.set_up(self)

        self.__check_cnt_parameters()

        self._device1_frequency = int(AcsWifiFrequencies.get_frequency(self._device1_channel))
        self._device2_frequency = int(AcsWifiFrequencies.get_frequency(self._device2_channel))

        self._connection_type = str(self._connection_type).upper()
        if self._connection_type not in LabWifiDirectConnect.P2P_CONNECTION_MODE:
            msg = "unknown connection type."
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # init and run device 1 supplicant
        self._device1_supplicant = self._get_supplicant_instance(self._device1_name)
        self._device1_supplicant.start(None)

        # init and run device 1 client
        self._device1_client = self._get_client_instance(self._device1_name)
        self._device1_client.init()
        self._device1_mac = self._device1_client.start(self._device1_p2pinterface)
        self._device1_client.remove_all_network()
        self._device1_client.set_listen_channel(self._device1_channel)
        self._device1_client.set_device_name(self._device1_p2p_name)

        # init and run device 2 supplicant
        self._device2_supplicant = self._get_supplicant_instance(self._device2_name)
        if self._device2_name.startswith("COMPUTER"):
            self._device2_supplicant.start(self._device2_host_interface)
        else:
            self._device2_supplicant.start(None)

        # init and run device 2 client
        self._device2_client = self._get_client_instance(self._device2_name)
        self._device2_client.init()
        self._device2_mac = self._device2_client.start(self._device1_p2pinterface)
        self._device2_client.remove_all_network()
        self._device2_client.set_listen_channel(self._device2_channel)
        self._device2_client.set_device_name(self._device2_p2p_name)

        return Global.SUCCESS, "No error"

    def run_test(self):
        """
        Execute the test
        """
        LabWifiDirectBase.run_test(self)

        # both devices Start find
        self._device1_client.find_network()
        self._device2_client.find_network()

        # Search devices
        dut_name = self._device1_client.find_dut(self._device2_mac, 20)
        if dut_name is None:
            msg = "Device 1 can't find Device 2 - " + self._device2_mac
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        self._logger.info("device 1 - p2p identify " + dut_name + " " + self._device2_mac)

        dut_name = self._device2_client.find_dut(self._device1_mac, 20)
        if dut_name is None:
            msg = "Device 2 can't find Device 1 - " + self._device2_mac
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        self._logger.info("device 2 - p2p identify " + dut_name + " " + self._device1_mac)

        # Connect both devices
        if self._connection_type == "PBC":  # pbc mode
            self._device1_client.connect_pbc(self._device2_mac, self._device1_go,
                                             True, self._device1_frequency)
            time.sleep(2)
            self._device2_client.connect_pbc(self._device1_mac, self._device2_go,
                                             False, self._device2_frequency)
        else:  # PIN mode
            connection_pin = self._device1_client.connect_pin_display(
                                            self._device2_mac, self._device1_go,
                                            True, self._device1_frequency)
            time.sleep(2)
            self._device2_client.connect_pin_enter(self._device1_mac,
                                            connection_pin, self._device2_go,
                                            False, self._device2_frequency)

        time.sleep(1)
        self._device1_client.set_lan_interface_ip_address(self._device1_ip, 2)
        self._device2_client.set_lan_interface_ip_address(self._device2_ip, 2)

        return Global.SUCCESS, "No error"

    def tear_down(self):
        """
        End and dispose the test
        """
        LabWifiDirectBase.tear_down(self)

        if self._device1_client is not None:
            self._device1_client.disconnect()
        if self._device2_client is not None:
            self._device2_client.disconnect()

        # stop and desinit device 1 client and supplicant
        if self._device1_client is not None:
            self._device1_client.stop()
            self._device1_client.release()
        if self._device1_supplicant is not None:
            self._device1_supplicant.stop()

        # stop and desinit device 2 client and supplicant
        if self._device2_client is not None:
            self._device2_client.stop()
        if self._device2_supplicant is not None:
            self._device2_supplicant.stop()
        if self._device2_client is not None:
            self._device2_client.release()

        return Global.SUCCESS, "No error"

    def __check_cnt_parameters(self):
        """
        Checks all connection parameters
        """
        if not self._device1_channel:
            msg = "undefined CHANNEL."
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if (int(self._device1_channel) not in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_5G) \
        and (int(self._device1_channel) not in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_2G):
            msg = "Unknown CHANNEL " + self._device1_channel
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if not self._device2_channel:
            msg = "undefined CHANNEL."
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if (int(self._device2_channel) not in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_5G) \
        and (int(self._device2_channel) not in AcsWifiFrequencies.WIFI_CHANNELS_FREQUENCIES_2G):
            msg = "Unknown CHANNEL " + self._device2_channel
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if not str(self._device1_go).isdigit():
            msg = "device 1 invalid go."
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        self._device1_go = int(str(self._device1_go))

        if not str(self._device2_go).isdigit():
            msg = "device 2 invalid go."
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        self._device2_go = int(str(self._device2_go))
