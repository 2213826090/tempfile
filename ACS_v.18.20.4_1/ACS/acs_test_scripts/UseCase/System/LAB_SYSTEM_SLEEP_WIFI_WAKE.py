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

:organization: UMG WSIV System Tests
:summary: This file implements LAB SYSTEM SLEEP WIFI WAKE UC
    i.e. Test S3/S0i3 residency with wifi associated
    -Check that platform enters/exits S3/S0i3 while Associated
:since: 26/11/2012
:author: jpstierlin RTC24950 Add S3/S0i3 tests
"""

import time
from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabSystemSleepWifiWake(LabWifiBase):

    """
    S3/S0i3 WiFi class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        # Get TC Parameters

        # Sleep mode
        self._sleep_mode = self._tc_parameters.get_param_value("SLEEP_MODE")

        # Sleep time duration in seconds
        self._duration = int(self._tc_parameters.get_param_value("DURATION"))

        # Settle time
        self._settle_time = \
            int(self._tc_parameters.get_param_value("SETTLE_TIME"))

        # Ping count
        ping_count = self._tc_parameters.get_param_value("PING_COUNT")
        if str(ping_count).isdigit():
            self._ping_count = int(ping_count)
        else:
            self._ping_count = 1

        # Ping interval
        ping_interval = self._tc_parameters.get_param_value("PING_INTERVAL")
        if ping_interval in ["", None]:
            self._ping_interval = float(1)
        else:
            self._ping_interval = float(ping_interval)

        # Ping flood
        ping_flood = self._tc_parameters.get_param_value("PING_FLOOD")
        if ping_flood in ["", None]:
            self._ping_flood = False
        else:
            self._ping_flood = str_to_bool(ping_flood)

        # Ping loss rate
        self._target_ping_packet_loss_rate = \
            float(self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE"))

        self._packet_size = \
            int(self._tc_parameters.get_param_value("PACKET_SIZE"))

        # Get computer type
        self._computer_name = self._tc_parameters.get_param_value("COMPUTER")

        # Get UECmdLayer
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._residency_api = self._device.get_uecmd("Residencies")

        self._computer = None
        self._phone_ip = None

        # Residency rate (sleep duration / test length)
        self._target_residency_rate_min = \
            self._tc_parameters.get_param_value("TARGET_RESIDENCY_RATE_MIN")
        if self._target_residency_rate_min not in ["", None]:
            self._target_residency_rate_min = \
                float(self._target_residency_rate_min)
        else:
            self._target_residency_rate_min = 0.0
        self._target_residency_rate_max = \
            self._tc_parameters.get_param_value("TARGET_RESIDENCY_RATE_MAX")
        if self._target_residency_rate_max not in ["", None]:
            self._target_residency_rate_max = \
                float(self._target_residency_rate_max)
        else:
            self._target_residency_rate_max = 100.0

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """

        LabWifiBase.set_up(self)

        self._phone_ip = self._networking_api.get_wifi_ip_address()

        if self._ping_count != 0 and \
                (self._computer_name == "" or self._computer_name is None):
            msg = "Cannot run that use case without a local/remote PC configured."
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        else:
            self._computer = self._em.get_computer(self._computer_name)

        self._sleep_mode_api.init(self._sleep_mode)

        # Wait for the DUT to settle down
        self._logger.info("wait " + str(self._settle_time) +
                          " seconds for DUT to settle down")

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        LabWifiBase.run_test(self)

        self._residency_api.clear()

        # It's time to sleep !
        self._phonesystem_api.sleep_mode("on")

        if self._io_card is not None:
            self._device.disconnect_board()
            self._io_card.usb_host_pc_connector(False)

        result = ""

        if self._ping_count == 0:
            ping_delay = self._duration
        else:
            ping_delay = self._duration / 2

        self._logger.info("sleep " + str(ping_delay) + " seconds")
        time.sleep(ping_delay)

        try:
            if self._ping_count != 0:
                # ping the platform
                packet_loss = self._computer.ping(self._phone_ip,
                                                  self._packet_size, self._ping_count,
                                                  self._ping_interval, self._ping_flood)
                msg = "packet loss: %d%s" % (packet_loss.value, packet_loss.units)
                result += msg
                if packet_loss.value > self._target_ping_packet_loss_rate:
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.CONNECTION_LOST, msg)
                self._logger.info("sleep " + str(ping_delay) + " seconds")
                time.sleep(ping_delay)
        finally:
            if self._io_card is not None:
                self._io_card.usb_host_pc_connector(True)
                # wait x seconds
                time.sleep(self._device._usb_sleep_duration)  # pylint: disable=W0212
                self._device.connect_board()
                time.sleep(self._wait_btwn_cmd)

        count = self._residency_api.get_value("count", self._sleep_mode)
        if not isinstance(count, float):
            msg = "Can't get end count"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        self._logger.debug("Value after=%f" % count)

        if self._ping_flood == False and count == 0:
            msg = "%s counter did not increment" % self._sleep_mode
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if self._ping_flood and count > 0:
            msg = "%s counter did unexpectedly increment" % self._sleep_mode
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        residency = self._residency_api.get_value("residency", self._sleep_mode_api.get_sleep_mode())

        result += " " + self._sleep_mode + " count: %d, residency: %f " % (count, residency)

        if (residency < self._target_residency_rate_min) or \
                (residency > self._target_residency_rate_max):
            self._logger.error(result)
            raise DeviceException(DeviceException.OPERATION_FAILED, result)

        return Global.SUCCESS, result

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        LabWifiBase.tear_down(self)
        time.sleep(self._wait_btwn_cmd)
        self._sleep_mode_api.clear()
        return Global.SUCCESS, "No errors"
