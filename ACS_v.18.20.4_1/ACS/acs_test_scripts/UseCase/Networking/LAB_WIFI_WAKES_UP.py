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
:summary: This file implements the LIVE WIFI WAKES UP UC
:since: 30/03/2012 BZ2409
:author: apairex
"""
import time
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Device.UECmd.UECmdTypes import Measure
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException


class LabWifiWakesUp(LabWifiBase):

    """
    Lab Wifi wakes up test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        # Set the screen timeout (in sec) to a very small value
        self._screen_timeout = 15

        # Default value for time to wait for wifi to disconnect all network
        self._time_before_wifi_sleep = 18 * 60

        # Ping fixed parameters
        self._packetsize = 32
        self._count = 4
        self._lossrate_connection_ok = 50
        self._lossrate_disconnected = 99

        self._previous_wifi_sleep_policy = None
        self._previous_screen_timeout = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiBase.set_up(self)

        # Get the time to wait for wifi to disconnect all network
        self._time_before_wifi_sleep = self._device.get_time_before_wifi_sleep()

        # Set WIFI sleep policy to "when screen turns off"
        self._previous_wifi_sleep_policy = self._networking_api.\
            get_wifi_sleep_policy()
        if self._previous_wifi_sleep_policy != \
                self._networking_api.WIFI_SLEEP_POLICY["WHEN_SCREEN_OFF"]:
            self._networking_api.\
                set_wifi_sleep_policy(self._networking_api.
                                      WIFI_SLEEP_POLICY["WHEN_SCREEN_OFF"])

        # Set screen timeout to a short value
        self._previous_screen_timeout = self._phone_system_api.get_screen_timeout()
        if self._previous_screen_timeout != self._screen_timeout:
            self._phone_system_api.set_screen_timeout(self._screen_timeout)

        # Remove the potential screen lock ON
        self._phone_system_api.display_off()

        # Check WIFI connection by pinging Wifi AP
        packet_loss = self._networking_api.ping(self._wifirouter_ip,
                                                self._packetsize, self._count)
        if packet_loss.value > self._lossrate_connection_ok:
            msg = "Wifi IP connection is bad: %s packet loss" \
                % str(packet_loss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Set the Wifi power saving mode to ON
        self._networking_api.set_wifi_power_saving_mode("on")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiBase.run_test(self)

        # Wait for the Wifi to sleep
        self._logger.info("Waiting %s sec for Wifi to sleep"
                          % self._time_before_wifi_sleep)
        time.sleep(self._time_before_wifi_sleep)
        self._logger.info("Wifi should have gone to sleep")

        # Checks that Wifi is well entered into sleep mode
        try:
            packet_loss = self._networking_api.ping(self._wifirouter_ip,
                                                    self._packetsize,
                                                    self._count)
        except AcsBaseException as e:
            if "Network is unreachable" in e.get_error_message():
                # Then the Exception has been raised
                # because of a ping connection failure
                packet_loss = Measure()
                packet_loss.value = 100
                packet_loss.units = "%"
            else:
                raise e

        if packet_loss.value < self._lossrate_disconnected:
            msg = "Wifi did not manage to enter sleep mode: %s packet loss" \
                % str(packet_loss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Wake up the screen
        self._phone_system_api.display_on()
        time.sleep(6 * self._wait_btwn_cmd)
        self._phone_system_api.display_off()

        # Check WIFI connection by pinging Wifi AP
        packet_loss = self._networking_api.ping(self._wifirouter_ip,
                                                self._packetsize, self._count)
        if packet_loss.value > self._lossrate_connection_ok:
            msg = "Wifi did not manage to wakes up: %s packet loss" \
                % str(packet_loss.value)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        LabWifiBase.tear_down(self)

        # Restore WIFI sleep policy
        if self._previous_wifi_sleep_policy is not None and \
                self._previous_wifi_sleep_policy != \
                self._networking_api.WIFI_SLEEP_POLICY["WHEN_SCREEN_OFF"]:
            self._networking_api.\
                set_wifi_sleep_policy(self._previous_wifi_sleep_policy)

        # Restore the screen timeout
        if self._previous_screen_timeout is not None and \
                self._previous_screen_timeout != self._screen_timeout:
            self._phone_system_api.set_screen_timeout(self._previous_screen_timeout)

        # Make sure that the screen lock ON is disabled
        self._phone_system_api.display_off()

        return Global.SUCCESS, "No errors"
