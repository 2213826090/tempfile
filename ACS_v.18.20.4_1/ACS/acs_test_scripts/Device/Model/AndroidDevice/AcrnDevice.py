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

:organization: INTEL SSG OTC
:summary: This file implements the Acrn AaaG platform
:since: 16/04/2018
:author: Jinliang Wang
"""

import time
import serial
import os
import sys
import pexpect.fdpexpect
import re
from UtilitiesFWK.Utilities import Global
from Device.Model.DeviceBase import DeviceBase
from Device.Model.AndroidDevice.AndroidDeviceBase import AndroidDeviceBase
from Device.Model.AndroidDevice.IntelDeviceBase import IntelDeviceBase
from Device.Model.AndroidDevice.BroxtonDevice import BroxtonDevice
from Lib.hypervisor.hypervisor import DebugCard

class AcrnDevice(BroxtonDevice):

    """
        Acrn AaaG device implementation
    """

    def __init__(self, config, logger):
        """
        Constructor
        Restart device to enter SOS, get SOS DHCP IP address, then set 
        as static IP, and store it to a temporary file for test program
        to read. At last to auto start up Android OS
        """
        DeviceBase.__init__(self, config, logger)

        self._sos_login_name = self.get_config("loginName")
        self._sos_login_password = self.get_config("loginPassword")
        self._sos_control_debug_path = self.get_config("controlDebugPath", "/dev/ttyUSB2")
        self._sos_console_debug_path = self.get_config("consoleDebugPath", "/dev/ttyUSB3")
        self._sos_acrn_network_interface = self.get_config("acrnNetworkInterface", "acrn-br0")

        self._control_serial = self.open_debug_port(self._sos_control_debug_path)
        self._console_serial = DebugCard(self._sos_console_debug_path)
        self._console_serial.open()

        retry_times = 3
        while retry_times > 0:
            try:
                # restart acrn device to make sure it only start up SOS
                self.restart_acrn_device()
                # get SOS console
                self._console_serial.switch_sos_console(self._sos_login_name, self._sos_login_password)
                # set static IP to SOS 
                self._console_serial.set_static_sos_ip(self._sos_acrn_network_interface)
                break
            except:
                retry_times -= 1
                continue

        # start up Android OS
        if retry_times > 0:
            try:
                self.get_logger().info("launching Android OS ...")
                self._console_serial.launch_uos()
            except:
                pass
            time.sleep(40)  # to wait Android OS complete to start up

        self._console_serial.close()
        self.close_debug_port(self._control_serial)
        self.get_logger().info("Closed all connected serial ports")

        BroxtonDevice.__init__(self, config, logger)

    def open_debug_port(self, dev, baud_rate=115200):
        """
        Open serial device port of debug board

        :param dev: serial device path, e.g. /dev/ttyUSB0
        :type dev: string

        :param baud_rate: serial baud rate
        :type baud_rate: int

        :rtype int
        :return ser: interface of opened serial device
        """
        self.get_logger().info("Open serial device %s ..." % dev)
        ser = serial.Serial(dev, baud_rate)
        if not ser.isOpen():
            ser.open()
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.ser.flush()  # clean any invalid buffer
        return ser

    def close_debug_port(self, ser):
        """
        Close serial device port of debug board

        :param ser: serial interface
        :type ser: int
        """
        if ser.isOpen():
            ser.close()

    def restart_acrn_device(self):
        """
        Restart acrn device
        """
        if self._control_serial.isOpen():
            self._control_serial.write("2r")
            time.sleep(3)
            self._control_serial.write("2g")
            time.sleep(10)
        else:
            self.get_logger().warning("Please connect debug card and open control port.")
