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
:summary: This file implements the Lab Wifi Live Cellular Network swap UC
:since: 29/02/2012
:author: apairex
"""
import time
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException


class LabWifiLiveCellularSwap(LabWifiBase):

    """
    Lab-Wifi/Live-cellular swap test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._live_server = \
            self._tc_parameters.get_param_value("CELLULAR_ADDRESS_TO_PING")

        self._packetsize = 32
        self._count = 10
        self._target_ping_packet_loss_rate = 20

        self._registration_waiting_time = 15

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiBase.set_up(self)

        # Check if cellular live server is valid
        if self._live_server in [None, "", "NONE"]:
            msg = "live cellular server is missing"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Disable Wifi
        self._networking_api.set_wifi_power("off")

        # Enable Cellular Data transfer
        self._networking_api.activate_pdp_context()
        time.sleep(self._wait_btwn_cmd)

        # Check cellular connection
        self.__ping_server(self._live_server, "Cellular network")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiBase.run_test(self)

        # Enable Wifi
        self._networking_api.set_wifi_power("on")
        time.sleep(self._registration_waiting_time)

        # Ping Wifi server
        self.__ping_server(self._wifirouter_ip, "Wifi network")

        # Disable Wifi
        self._networking_api.set_wifi_power("off")
        time.sleep(self._registration_waiting_time)

        # Ping Live server
        self.__ping_server(self._live_server, "Cellular network")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def __ping_server(self, server2ping, network2test):
        """
        Function performing a ping on the server specified in parameter.
        This function raises an Exception in case the server is not reachable

        :type server2ping: str
        :param server2ping: server to ping
        :type network2test: str
        :param network2test: Type of network to test
        """
        try:
            packet_loss = self._networking_api.\
                ping(server2ping, self._packetsize, self._count)
        except AcsBaseException as error:
            if error.get_error_message().find(DeviceException.DEFAULT_ERROR_CODE.
                                              rstrip(".")) != -1:
                msg = network2test + " connection fails"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                raise error
        except:
            raise

        # Compute verdict depending on % of packet loss
        if packet_loss.value > self._target_ping_packet_loss_rate:
            msg = network2test + ": too much packet loss (%s)" \
                % str(packet_loss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
