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
:summary: This file implements the LAB WIFI DHCP LEASE UC
:since: Friday, July 06 2012
:author: rneu
"""
import time

from acs_test_scripts.UseCase.Networking.LAB_WIFI_DHCP_SERVER_BASE import LabDHCPServerBase
from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LabWifiDhcpLease(LabDHCPServerBase):

    """
    Lab Wifi Connect Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabDHCPServerBase.__init__(self, tc_name, global_config)

        # Sleep mode (s3/s0i3)
        self._sleep_mode = self._tc_parameters.get_param_value("SLEEP_MODE")

        # Check if USB has to be disconnected during the test
        self._disconnect_usb = str(self._tc_parameters.get_param_value("NO_USB")).lower()
        if self._disconnect_usb in ["none", ""]:
            self._disconnect_usb = "false"

        # check in TC settings if a lease parameter is given
        self._lease = str(self._tc_parameters.get_param_value("DHCP_LEASE")).lower()
        if self._lease in ["none", ""]:
            self._lease = self._dhcp_lease

        # Residency rate (sleep duration / test length)
        self._target_residency_rate_min = \
            str(self._tc_parameters.get_param_value("TARGET_RESIDENCY_RATE_MIN")).lower()
        if self._target_residency_rate_min not in ["none", ""]:
            self._target_residency_rate_min = \
                float(self._target_residency_rate_min) / 100
        else:
            self._target_residency_rate_min = 0

        self._target_residency_rate_max = \
            str(self._tc_parameters.get_param_value("TARGET_RESIDENCY_RATE_MAX")).lower()
        if self._target_residency_rate_max not in ["none", ""]:
            self._target_residency_rate_max = \
                float(self._target_residency_rate_max) / 100
        else:
            self._target_residency_rate_max = 0

        # Get computer type (don't ping if empty)
        self._computer_name = str(self._tc_parameters.get_param_value("COMPUTER", ""))
        if self._computer_name not in ["none", ""]:
            self._computer = self._em.get_computer(self._computer_name)
        else:
            self._computer = None

        self._packetsize = str(self._tc_parameters.get_param_value("PACKET_SIZE"))
        if self._packetsize in ["none", ""] or not self._packetsize.isdigit():
            self._packetsize = 56  # default ping packet size
        else:
            self._packetsize = int(self._packetsize)

        # Get UECmdLayer
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")

        self._usb_disconnected = False

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LabWifiBase.set_up(self)

        # disconnect wifi as LAB_WIFI_BASE connects it
        self._networking_api.wifi_disconnect_all()
        time.sleep(self._wait_btwn_cmd)

        # Initiate connection to the equipment
        self._ns.init()

        # turn off dhcp and clean params in any case
        self._ns.set_dhcp("off")
        # then check if it has to be turned on
        if self._dhcp_enabled in ("True", "TRUE"):
            self._ns.set_dhcp("on",
                              self._low_excluded_addr,
                              self._high_excluded_addr,
                              self._dhcp_subnet,
                              self._dhcp_subnet_mask,
                              self._lease,
                              self._dhcp_gateway_address)

        # Close the connection to AP
        self._ns.release()

        self._sleep_mode_api.init(self._sleep_mode)

        # Connect the DUT on the Wifi network
        self._networking_api.wifi_connect(self._ssid, True, True)
        time.sleep(5)

        self._dut_ip = "0.0.0.0"
        bad_ip_timeout = 15
        while (self._dut_ip == "0.0.0.0") and (bad_ip_timeout > 0):
            self._dut_ip = self._networking_api.get_wifi_ip_address()
            time.sleep(1)
            bad_ip_timeout -= 1

        if self._dut_ip == "0.0.0.0":
            msg = "Can't obtain a valid IP address (%s)" % self._dut_ip
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # set Wifi Sleep Policy to "never"
        self._networking_api.set_wifi_sleep_policy(self._networking_api.WIFI_SLEEP_POLICY["NEVER"])
        self._logger.info("Setting wifi policy to never")
        return Global.SUCCESS, "no errors"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabDHCPServerBase.run_test(self)

        value_before = self._phonesystem_api.get_sleep_wakeup_count(self._sleep_mode)
        time_before = self._phonesystem_api.get_sleep_time(self._sleep_mode)
        start_time = time.time()

        # It's time to sleep !
        self._phonesystem_api.sleep_mode("on")

        should_test_lease = True
        if self._lease == "infinite":
            should_test_lease = False

        # convert dhcp lease str in seconds
        if should_test_lease:
            lease_time = self._lease.split()
            lease_in_seconds = ((int(lease_time[0]) * 24 + int(lease_time[1])) * 60 + int(lease_time[2])) * 60
        else:
            lease_in_seconds = 300

        duration = int(1.5 * lease_in_seconds)

        # Disconnect USB
        if self._disconnect_usb == "true":

            # Redirect log file
            self._networking_api.redirect_log_on_dut()

            # Unplug the usb
            self._device.disconnect_board()
            self._io_card.usb_host_pc_connector(False)
            self._usb_disconnected = True

            try:
                if self._computer is not None:
                    # Ping during 2 or 3 leases
                    count = int(duration / 7)
                    self._computer.ping(self._dut_ip,
                                        self._packetsize, count, 7)
                else:
                    # Wait 2 or 3 leases
                    self._logger.info("Waiting %d seconds disconnected from the DUT" % duration)
                    time.sleep(float(duration))
            finally:
                # Plug the usb
                self._io_card.usb_host_pc_connector(True)
                self._device.connect_board()
                self._usb_disconnected = False
                # Stop redirecting log file
                self._networking_api.kill_log_on_dut()

            # retrieve dhcp lease interval
            average = self._networking_api.retrieve_dhcp_renewal_interval_on_dut(2)

        else:
            # retrieve dhcp lease interval
            average = self._networking_api.retrieve_dhcp_renewal_interval(2, duration)

        if should_test_lease:
            if average is None:
                msg = "Unexpected value: Renew lease interval %s" % average
                self._logger.error(msg)
                raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)
            # renew lease interval should be about 48% of router lease time
            min_interval = lease_in_seconds * 46 / 100
            max_interval = lease_in_seconds * 50 / 100

            if (average < min_interval) or (average > max_interval):
                msg = "Renew lease interval %d is beyond boundaries %d and %d" % (average, min_interval, max_interval)
                self._logger.error(msg)
                raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)
        else:
            # if lease is infinite check no Renew is done
            if average is not None:
                msg = "Renew lease are performed when it should not"
                self._logger.error(msg)
                raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)

        stop_time = time.time()
        time_after = self._phonesystem_api.get_sleep_time(self._sleep_mode)
        value_after = self._phonesystem_api.get_sleep_wakeup_count(self._sleep_mode)

        count = value_after - value_before
        if count == 0:
            msg = "%s counter did not increment" % self._sleep_mode
            self._logger.error(msg)
            raise DeviceException(DeviceException.PROHIBITIVE_MEASURE, msg)

        time_after -= time_before
        duration = stop_time - start_time
        residency = time_after / duration

        result = self._sleep_mode + " count: %d, residency: %f (%f/%f)" % \
            (count, residency, time_after, duration)

        if residency < self._target_residency_rate_min or \
           residency > self._target_residency_rate_max:
            self._logger.error(result)
            raise DeviceException(DeviceException.OPERATION_FAILED, result)

        new_dut_ip = self._networking_api.get_wifi_ip_address()
        if self._dut_ip != new_dut_ip:
            msg = "DUT ip address has been changed from %s to %s during the test" % \
                (self._dut_ip, new_dut_ip)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return Global.SUCCESS, result

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        self._logger.info("%s: entering tear_down", self._name)
        if self._usb_disconnected:
            # Plug the usb
            self._io_card.usb_host_pc_connector(True)
            self._device.connect_board()
            self._usb_disconnected = False
            # Stop redirecting log file
            self._networking_api.kill_log_on_dut()

        self._sleep_mode_api.clear()

        LabDHCPServerBase.tear_down(self)

        # set Wifi Sleep Policy to default
        self._logger.info("Setting wifi policy back to default")
        self._networking_api.set_wifi_sleep_policy(self._networking_api.WIFI_SLEEP_POLICY["DEFAULT"])

        return Global.SUCCESS, "no errors"
