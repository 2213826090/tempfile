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
:summary: This file implements the FastbootTool class
:since: 29/07/2013
:author: lbavois
"""

import os
import time

from lxml import etree

from acs_test_scripts.Device.Module.Common.Flash.FlashManager.Tool.FlashToolBase import FlashToolBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import internal_shell_exec
from Core.Report.ACSLogging import LOGGER_FWK


class FastbootTool(FlashToolBase):

    """
    This class is the specific class used for FASTBOOT flash tool
    """

    def __init__(self, logger=None):
        """
        Constructor

        :type logger: object
        :param logger: logger name
        """
        FlashToolBase.__init__(self, "FastbootTool", logger)

        # Specific to Flash Tool
        self._logger = logger if logger else LOGGER_FWK
        self._flash_tool_state = "UNUSED"
        self._serial = None
        self._pos_serial = None
        self._soc_serial = None
        self._switch_device_state_fct = None
        self._first_file_flashing = False

    def _check_flash_tool_availability(self):
        """
        Assume fastboot is available
        """

        cmd_name = "fastboot"

        return cmd_name

    def _check_flash_files(self, flash_file_list):
        """
        Check flash file compatibility with flash tool

        :type flash_file_list: list
        :param flash_file_list: list of flash file (absolute paths)
        :rtype: list
        :return: list of valid flash file (absolute paths) for current flash tool
        """
        fastbootcmds_flash_file_list = []

        if len(flash_file_list) > 0:
            for flash_file in flash_file_list:
                _, fileExtension = os.path.splitext(flash_file)

                if os.path.isfile(flash_file) and (fileExtension.lower() == ".xml"):
                    self._logger.info("FastbootTool: Flash file %s exists" % str(flash_file))
                    fastbootcmds_flash_file_list.append(flash_file)
                else:
                    err_msg = "FastbootTool: No suitable flash file  (.xml file should be used) or " + \
                              "flash file does not exist: %s" % str(flash_file)
                    self._logger.error(err_msg)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
        else:
            err_msg = "FastbootTool: No file to flash"
            self._logger.error(err_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)

        return fastbootcmds_flash_file_list

    def _retrieve_device_info(self, device_flash_info):
        """
        Retrieve necessary information from the device for flash tool execution

        :type device_flash_info: FlashDeviceInfo
        :param device_flash_info: contain all device information needed by the flash tool
        """
        # Retrieve soc_serial_number and serial number (specific to PFT flash tool)
        serial = device_flash_info.device_serial_number
        self._serial = str(serial).strip()

        soc_serial = device_flash_info.device_soc_serial_number
        self._soc_serial = str(soc_serial).strip()

        pos_serial = device_flash_info.device_pos_serial_number
        self._pos_serial = str(pos_serial).strip()

        # Retrieve callback function in device class to make change its state
        self._switch_device_state_fct = device_flash_info.change_device_state_fct

    def _get_cmd(self):
        """
        Build the flash command : we do not use the parent mechanism
        """
        return None

    def _run_fastboot_cmds(self):
        android_dir = str(os.environ['ANDROID_PRODUCT_OUT'])
        self._logger.info("Waiting for the device to Power On and stabilize for 100 seconds")
        time.sleep(100)
        timeout = 10
        command = "adb connect " + self._serial
        self._logger.info(command)
        return_code, output = internal_shell_exec(command, timeout)
        if return_code == Global.SUCCESS:
            try:
                tree = etree.parse(self._flash_file)
                for df in tree.xpath('//file'):
                    conf_type = df.attrib['TYPE']
                    sf = df.getchildren()
                    if conf_type == "FASTBOOT":
                        fastboot_file = sf[0].text
                        print (conf_type + "::" + fastboot_file)
                    elif conf_type == "KERNEL":
                        kernel_file = sf[0].text
                        print (conf_type + "::" + kernel_file)
                    elif conf_type == "RECOVERY":
                        recovery_file = sf[0].text
                        print (conf_type + "::" + recovery_file)
                    elif conf_type == "SYSTEM":
                        system_file = sf[0].text
                        print (conf_type + "::" + system_file)
            except Exception as ex:
                return_code = Global.FAILURE
                self.error_message = "FastbootTool: Error in parsing xml %s" % str(ex)
        else:
            self.error_message = "FastbootTool: ADB connect has failed" % str(output)

        if return_code == Global.SUCCESS:
            # assume that device is now in POS, so we can release the pos line
            if self._flash_tool_state == "FLASH_MODE_START":
                # Deactivate provisionning line as PFT start the flashing.
                if self._switch_device_state_fct("POS_MODE_OFF") == Global.SUCCESS:
                    self._flash_tool_state = "FLASH_MODE"
                else:
                    return_code = Global.FAILURE
                    self._flash_tool_state = "UNKNOW"
                    self.error_message = "FastbootTool: Issue can not disable provisionning line on the device"

            if self._flash_tool_state == "FLASH_MODE":

                command = "fastboot devices"
                timeout = 2
                return_code, output = internal_shell_exec(command, timeout)
                if return_code == Global.SUCCESS:
                    for df in tree.xpath('//command'):
                        sf = df.getchildren()
                        command = df.find('string').text
                        timeout = df.find('timeout').text
                        flash_time = int(timeout)
                        if command.find("$fastboot_file") > -1:
                            command = command.replace("$fastboot_file", android_dir + "/" + fastboot_file)
                        elif command.find("$recovery_file") > -1:
                            command = command.replace("$recovery_file", android_dir + "/" + recovery_file)
                        elif command.find("$system_file") > -1:
                            command = command.replace("$system_file", android_dir + "/" + system_file)
                        elif command.find("$kernel_file") > -1:
                            command = command.replace("$kernel_file", android_dir + "/" + kernel_file)
                        else:
                            print command
                        command = command.replace("fastboot", "fastboot -t " + self._pos_serial, 1)
                        if command.find("system") > -1:
                            return_code, output = internal_shell_exec(command, flash_time)
                        else:
                            return_code, output = internal_shell_exec(command, flash_time)
                        if return_code != Global.SUCCESS:
                            self.error_message = "Fastboot flash commands fail" % str(output)
                else:
                    self.error_message = "Fastboot devices has failed" % str(output)
            else:
                self.error_message = "FastbootTool: unable to switch to flash mode"

        return return_code

    def _run(self):
        """
        Flash the target.
        """
        # Parse the input xml file and retrieve the flash file
        # information . Flashing of the partition.tbl will break the uefi Android
        # image as the current UEFI creates partition in its own way and partition.tbl
        # file is not supported

        result = self._run_fastboot_cmds()

        if result == Global.SUCCESS:
            return_code, error_message = self._switch_device_state_fct("WAIT_BOOT")
            if return_code == Global.SUCCESS:
                self.exec_result = True
            else:
                self.error_message = "FastbootTool: Flash failure - device fails to boot : %s" % error_message
                self._flash_tool_state = "UNUSED"
        else:
            self.exec_result = False
            self._flash_tool_state = "UNUSED"

    def _analyze_output(self, data):
        """
        log the output from flash tool command execution

        :type data: str
        :param data: string to analyze
        """
        for d in data:
            self._logger.debug(d)

    def flash(self, flash_file_list, device_info, timeout):
        """
        Start the flash execution for the flash tool (flash procedure with device state change)

        :type flash_file_list: list
        :param flash_file_list: list containing flash file absolute paths
        :type device_info: FlashDeviceInfo
        :param device_info: contain all device information needed by the flash tool
        :type timeout: int
        :param timeout: max time in seconds to do flash procedure
        :rtype: int
        :return: result of the flash procedure (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        """

        # Check flash file availability and format
        flash_files = self._check_flash_files(flash_file_list)

        self._retrieve_device_info(device_info)

        # Flash all flash files passed
        for flash_file in flash_files:

            # Device state will depend of the flash file type and the current flash tool state
            if self._flash_tool_state == "UNUSED":
                # Change the device state => Trig the flash mode
                if self._switch_device_state_fct("FLASH_SWITCH_OFF") == Global.SUCCESS and \
                    self._switch_device_state_fct("POS_MODE_ON") == Global.SUCCESS:
                    self._flash_tool_state = "FLASH_MODE_START"
                else:
                    self._logger.error(
                        "FastbootTool: Issue can not switch off + enable provisionning mode on the device")
                    flash_return_code = Global.FAILURE

            if self._flash_tool_state == "FLASH_MODE_START" or self._flash_tool_state == "FLASH_MODE":
                self._flash_file = flash_file
                flash_return_code = FlashToolBase._flash(self, timeout)

            if flash_return_code == Global.FAILURE:
                # Flash has failed on a flash file => stop the flash procedure (do not flash other flash files)
                break

        return flash_return_code
