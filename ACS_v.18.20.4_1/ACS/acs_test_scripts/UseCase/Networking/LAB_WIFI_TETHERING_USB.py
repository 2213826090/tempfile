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
:summary: This file implements the LAB WIFI TETHERING USB UC
The goal of this UC is to validate the tethered USB interface over wifi
:since: 20/12/2012
:author: jpstierlin RTC20935
"""

import time
import re

from acs_test_scripts.UseCase.Networking.LAB_WIFI_BASE import LabWifiBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.DeviceException import DeviceException


class LabWifiTetheringUSB(LabWifiBase):

    """
    Lab Wifi Tethering USB Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabWifiBase.__init__(self, tc_name, global_config)

        self._original_flight_mode = None
        self._computer = None
        self._disconnected = False
        self._unplugged = False
        self._changedroute = False
        self._destination = None
        self._netmask = None
        self._iface = None
        self._hotspot_ip = None
        self._network = None

        self._packetsize = \
            int(self._tc_parameters.get_param_value("PACKET_SIZE"))
        self._count = int(self._tc_parameters.get_param_value("PACKET_COUNT"))
        self._target_ping_packet_loss_rate = \
            float(self._tc_parameters.get_param_value("TARGET_PACKET_LOSS_RATE"))

        # Should we unplug USB during tethering ?
        unplug_usb = str(self._tc_parameters.get_param_value("UNPLUG_USB"))
        if unplug_usb.lower() in ["1", "on", "true", "yes"]:
            self._unplug_usb = 1
        else:
            self._unplug_usb = 0

        # Should we turn wifi off during tethering ?
        wifi_off = str(self._tc_parameters.get_param_value("WIFI_OFF"))
        if wifi_off.lower() in ["1", "on", "true", "yes"]:
            self._wifi_off = 1
        else:
            self._wifi_off = 0

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """

        # store original flight mode
        self._original_flight_mode = self._networking_api.get_flight_mode()
        time.sleep(self._wait_btwn_cmd)

        if self._original_flight_mode != self._use_flight_mode:
            self._networking_api.set_flight_mode(self._use_flight_mode)
            time.sleep(self._wait_btwn_cmd)

        # Configure and connect wifi to the AP
        LabWifiBase.set_up(self)

        # Get local computer to run dhclient
        self._computer = self._em.get_computer("COMPUTER1")

        # Get access point subnet to add its route to tethered interface
        self._network = re.sub(r'([0-9]+\.[0-9]+\.[0-9]+).*', r'\1.0',
                               self._wifirouter_ip)

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        LabWifiBase.run_test(self)

        # push a script that will run the test in adb disconnected mode
        self._networking_api.usb_tether(self._wifi_off, self._unplug_usb,
                                        self._use_flight_mode)

        # Disconnect ADB
        self._device.disconnect_board()
        self._disconnected = True

        time.sleep(10)

        start_time = time.time()

        # Tethering should have now started on DUT, wait for USB interface to come up
        # pylint: disable=W0612
        ip, self._hotspot_ip = self._computer.dhclient(
            self._computer.get_usb_interface())

        # USB is now tethered

        # remove temporarily existing route for the same network
        self._destination, self._netmask, self._iface = \
            self._computer.change_route(self._network, self._hotspot_ip)
        self._changedroute = True

        # Ping the Access Point behind the tethered USB hotspot
        packet_loss = self._computer.ping(self._wifirouter_ip, self._packetsize, self._count)
        self._logger.info("Packet loss: %s%s" % (packet_loss.value, packet_loss.units))
        if packet_loss.value > self._target_ping_packet_loss_rate:
            msg = "Could not ping AP through interface usb0"
            self._logger.error(msg)
            time.sleep(90)  # need to wait for the DUT script to end
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # wait for up to 60 seconds after starting DUT script
        sleep_time = 60 - (time.time() - start_time)
        if sleep_time > 0:
            time.sleep(sleep_time)

        if self._unplug_usb:
            self._io_card.usb_host_pc_connector(False)
            self._unplugged = True
            time.sleep(10)
            start_time = time.time()

            # try to ping again, this should fail
            packet_loss = self._computer.ping(self._wifirouter_ip, self._packetsize, self._count)
            self._logger.info("Packet loss: %s%s" % (packet_loss.value, packet_loss.units))
            if packet_loss.value != 100:
                msg = "Ping AP through interface usb0 successful after USB unplugged"
                self._logger.error(msg)
                time.sleep(60)  # need to wait for the DUT script to end
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            # plug USB
            self._io_card.usb_host_pc_connector(True)
            self._unplugged = False
            # wait x seconds
            time.sleep(self._device.get_usb_sleep_duration())

            # wait for DUT to complete script...
            sleep_time = 30 - (time.time() - start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

            # wait for USB interface to come up, this should fail !
            try:
                self._computer.dhclient(self._computer.get_usb_interface())
                msg = "Interface usb0 incorrectly still tethered"
                self._logger.error(msg)
                time.sleep(30)  # need to wait for the DUT script to end
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            except TestEquipmentException as e:
                if not "Could not get IP address" in str(e):
                    raise

        elif not self._use_flight_mode or self._wifi_off:

            # wait for DUT to activate flight_mode/turn off wifi...
            start_time = time.time()

            # try to ping again, this should fail
            packet_loss = self._computer.ping(self._wifirouter_ip,
                                              self._packetsize, self._count)
            self._logger.info("Packet loss: %s%s" % (packet_loss.value, packet_loss.units))
            if packet_loss.value != 100:
                msg = "Ping AP through interface usb0 successful"
                if self._wifi_off:
                    msg += " after turning wifi off"
                else:
                    msg += " after enabling flight mode"
                self._logger.error(msg)
                time.sleep(60)  # need to wait for the DUT script to end
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            # wait for DUT to deactivate flight_mode/turn on wifi...
            sleep_time = 30 - (time.time() - start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)

            # Ping the Access Point
            packet_loss = self._computer.ping(self._wifirouter_ip,
                                              self._packetsize, self._count)
            self._logger.info("Packet loss: %s%s" % (packet_loss.value, packet_loss.units))
            if packet_loss.value > self._target_ping_packet_loss_rate:
                msg = "Could not ping AP through interface usb0"
                self._logger.error(msg)
                time.sleep(30)  # need to wait for the DUT script to end
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # wait 30 seconds more for DUT to deactivate tethering
        time.sleep(30)

        self._device.connect_board()
        self._disconnected = False

        if self._changedroute:
            # restore original route
            self._computer.restore_route(self._network, self._hotspot_ip,
                                         self._destination, self._netmask, self._iface)
            self._changedroute = False

        return Global.SUCCESS, "No error"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """

        # need to reconnect board before calling tear_down
        if self._unplugged:
            self._io_card.usb_host_pc_connector(True)
            self._unplugged = False
            # wait x seconds
            time.sleep(self._device.get_usb_sleep_duration())

        if self._disconnected:
            self._device.connect_board()
            self._disconnected = False

        if self._changedroute:
            # restore original route
            self._computer.restore_route(self._network, self._hotspot_ip,
                                         self._destination, self._netmask, self._iface)
            self._changedroute = False

        LabWifiBase.tear_down(self)

        if self._original_flight_mode != self._networking_api.get_flight_mode():
            self._networking_api.set_flight_mode(self._original_flight_mode)

        return Global.SUCCESS, "No error"
