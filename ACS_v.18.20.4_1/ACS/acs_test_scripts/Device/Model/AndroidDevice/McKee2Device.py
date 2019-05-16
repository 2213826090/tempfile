"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

:organization: INTEL NDG SW
:summary: This file implements the McKee 2 platform
:since: 14/02/2014
:author: jreynaux
"""
from Device.Model.AndroidDevice.IntelDeviceBase import IntelDeviceBase
from Device.Model.AndroidDevice.AndroidDeviceBase import AndroidDeviceBase
from UtilitiesFWK.Utilities import Global, AcsConstants
from ErrorHandling.DeviceException import DeviceException
import time


class McKee2Device(IntelDeviceBase):

    """
        McKee2 platform implementation
    """

    def __init__(self, config, logger):
        """
        Constructor

        :type  config: dict
        :param config: Device configuration to use

        :type  logger: logger
        :param logger: Logger to use
        """
        IntelDeviceBase.__init__(self, config, logger)

        self._cellular_network_interface = self.get_config("cellularNetworkInterface", "rmnet0")

    def switch_on(self, boot_timeout=None, settledown_duration=None,
                  simple_switch_mode=False):
        """
        Switch ON the device
        This can be done either via the power supply
        or via IO card

        :param boot_timeout: Total time to wait for booting
        :type boot_timeout: int

        :param settledown_duration: Time to wait until start to count \
                                    for timeout,
                                    Period during which the device must \
                                    have started.
        :type settledown_duration: int

        :param simple_switch_mode: a C{boolean} indicating whether we want
        to perform a simple switch on.
        :type simple_switch_mode: bool

        :rtype: list
        :return: Output status and output log
        """
        (return_code, return_message) = IntelDeviceBase.switch_on(self, boot_timeout,
                                                                  settledown_duration,
                                                                  simple_switch_mode)
        if return_code == Global.SUCCESS:
            if self._embedded_log:
                # Initialize embedded log mechanism for MODEM if required
                self._embedded_log.start("MODEM")

        return return_code, return_message

    def get_cellular_network_interface(self):
        """
        Return the ip interface of celluar network

        this interface can be obtain from the command "busybox ifconfig"
        :rtype: str
        :return: telephony ip interface
        """
        return self._cellular_network_interface

    def set_filesystem_rw(self):
        """
        Set the file system in read/write mode.
        """
        self.get_logger().debug("set filesystem to read/write (/ partition)")
        tries = 0
        status = Global.FAILURE
        err_msg = "FAILURE"

        while tries < 3:
            status, err_msg = self.run_cmd("adb shell mount -wo remount /", 3, True)
            if status == Global.FAILURE:
                tries += 1
            else:
                break

        if status == Global.FAILURE:
            self.get_logger().error("set_filesystem_rw error: %s" % err_msg)
            raise DeviceException(
                DeviceException.FILE_SYSTEM_ERROR,
                "set_filesystem_rw error: %s" % err_msg)

    def _get_device_boot_mode(self):
        """
        get the boot mode from adb

        :rtype: string
        :return: device state : MOS or UNKNOWN
        """
        return AndroidDeviceBase._get_device_boot_mode(self)

    def _wait_for_full_boot(self, timeout):
        """
        Loop until adb confirms boot complete

        :type timeout: int
        :param timeout:timeout to wait until full boot

        :rtype: bool
        :return: True if board is fully booted during timeout, False otherwise
        """
        self.get_logger().info("Waiting for device to be fully booted ...")

        begin_time = time.time()
        end_time = begin_time + float(timeout)
        full_boot = False
        while not full_boot and time.time() < end_time:
            if self.get_property_value("dev.bootcomplete") == "1":
                self.get_logger().info("Device fully booted")
                full_boot = True
                # Wait 1 s before retrying
            time.sleep(1)

        return full_boot

    def _retrieve_imei(self, properties):
        """
        Retrieve the imei version (from adb shell getprop)

        ..attention:: For NexusS and reference device key is ril.imei
        for Mfld-Android it should be persist.radio.device.imei

        :type properties: dict
        :param properties: a dictionnary containing all system prop (gotten through getprop)
        :rtype: str
        :return: the imei, or None if unable to retrieve it
        """
        imei = None
        key_list = ("persist.radio.device.imei", "ril.IMEI", "ril.barcode", "ro.serialno")

        key_found = False
        for key in key_list:
            self.get_logger().debug("Checking " + key + " key ...")
            if key in properties:
                imei = properties[key]

            if imei not in (None, ""):
                self.get_logger().debug("Imei found in " + key + " key !")
                key_found = True
                break
            else:
                self.get_logger().debug("Imei not found, checking next key.")

        if not key_found:
            self.get_logger().debug("Impossible to retrieve Imei")
            imei = None

        return imei

    def _retrieve_kernel_version(self):
        """
        Retrieve the kernel version (from adb shell cat /proc/version)

        :rtype: str
        :return: the kernel version, or None if unable to retrieve it
        """
        kernel_version = AcsConstants.NOT_AVAILABLE
        adb_cmd_str = "adb shell cat /proc/version"

        status, status_msg = self.run_cmd(adb_cmd_str, self._uecmd_default_timeout, True)

        if status == Global.SUCCESS:
            if status_msg not in (None, "") and status_msg.lower().find("not found") == -1:
                # Check if rooted device
                if status_msg.lower().find("permission denied") != -1:
                    self.get_logger().debug("Non-rooted device, unable to retrieve Kernel Version")
                    kernel_version += " (non rooted device)"
                else:
                    status_msg = str(status_msg).strip().split(' ')
                    kernel_version = status_msg[2]
        else:
            self.get_logger().warning("Impossible to retrieve Kernel Version")

        return kernel_version

    def _retrieve_fw_version(self, properties):
        """
        Retrieve the firmware version (from adb shell getprop)

        :type properties: dict
        :param properties: a dictionnary containing all system prop (gotten through getprop)
        :rtype: str
        :return: the firmware version, or None if unable to retrieve it
        """
        key = "ro.bootloader"
        fw_version = None

        if key in properties:
            fw_version = properties[key]

        if fw_version in (None, ""):
            self.get_logger().warning("Fail to retrieve firmware version of the device")
            fw_version = None

        return fw_version

    def _cos_line_on(self):
        """
        Enable the USB host to DUT. Stay in COS mode (Go in flash mode)

        :rtype: int (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        :return: result of the switch state request
        """
        self.get_logger().info("Connect usb host to dut: enter in flash mode")
        return_code = Global.FAILURE
        # Enable flash mode by connecting usb host while in COS mode
        if self._eqts_controller.connect_usb_host_to_dut():
            return_code = Global.SUCCESS

        return return_code

    def _cos_line_off(self):
        """
        Disable the USB host to DUT. Stay in COS mode (Stay in flash mode)

        :rtype: int (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        :return: result of the switch state request
        """
        self.get_logger().info("Disconnect usb host to dut: stay in POS mode")
        return_code = Global.FAILURE
        # Disable flash mode by disabling usb host connection
        if self._eqts_controller.disconnect_usb_host_to_dut():
            return_code = Global.SUCCESS

        return return_code

    def enable_adb_root(self):
        """
        Switch adb to adb root
        :rtype: boolean
        :return: true if root is successfully set, false otherwise
        """
        self.get_logger().info("Adb root requested, enabling it ...")
        result, output = self.run_cmd("adb root", self._adb_root_cmd_timeout, force_execution=True)

        end_time = time.time() + self._adb_root_timeout
        while time.time() < end_time and output.find("already running as root") == -1:
            time.sleep(self._adb_root_timeout / 10.0)
            if self._use_adb_over_ethernet:
                # reconnect device, as "adb root" reset adb socket
                self._phone_handle.adb_ethernet_start(self._ip_address, self._adb_port,
                                                      self._adb_connect_retries_nb, self._adb_connect_timeout)
            # Unplug and replug the USB cable, if IOcard exists.
            # In case of blocking 'adb root' command, this can ends the command
            self.get_logger().info("root request has failed, unplug/plug USB cable then try again")
            self._eqts_controller.disconnect_usb_host_to_dut(raise_exception=False)
            time.sleep(1)
            self._eqts_controller.connect_usb_host_to_dut(raise_exception=False)
            time.sleep(5)
            # Add a connection timeout to 'adb root' command timeout
            result, output = self.run_cmd("adb root",
                                          self._adb_root_cmd_timeout+self._adb_connect_timeout,
                                          force_execution=True)

        return result == Global.SUCCESS and output.find("already running as root") != -1
