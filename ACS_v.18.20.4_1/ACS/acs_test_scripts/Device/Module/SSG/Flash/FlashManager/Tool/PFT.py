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

:organization: INTEL OTC SQE
:summary: This file implements the PhoneFlashTool (PFT) class
:since: 24/07/2015
:author: mceausu
"""

import os
import platform
import shlex

from acs_test_scripts.Device.Module.SSG.Flash.FlashManager.Tool.FlashToolBase import FlashToolBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.CommandLine import CommandLine
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


class PFT(FlashToolBase):

    """
    This class is the specific class used for PFT flash tool
    """

    def __init__(self, logger=None):
        """
        Constructor

        :type logger: object
        :param logger: logger name
        """
        FlashToolBase.__init__(self, "PFT", logger)

        # Specific to Flash Tool
        self._logger = logger if logger else LOGGER_TEST_SCRIPT
        self._flash_tool_state = "INIT"
        self._serial = None
        self._soc_serial = None
        self._switch_device_state_fct = None
        self._first_file_flashing = False
        self._pos_mode_on_exec_thread = None
        self._flash_file = None

    def _check_flash_tool_availability(self):
        """
        Check if PFT Flash Tool is installed over current ACS bench

        :rtype: str
        :return: string containing flash tool name to use for flash execution
        """
        if platform.system() == "Windows":
            cmd_name = "cflasher"
        else:
            cmd_name = "phoneflashtool"

        # Check the phone flash tool installation over ACS bench
        flash_tool_cmd_path = CommandLine.which(cmd_name)

        if flash_tool_cmd_path is None:
            error_msg = "PFT: Check if PHONE FLASH TOOL is installed on ACS bench (Command %s not found !)" % cmd_name
            if platform.system() == "Windows":
                error_msg += " - Check also if PHONE FLASH TOOL folder is referenced on ACS bench in PATH environment variable"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.EXTERNAL_LIBRARY_ERROR, error_msg)

        return cmd_name

    def _check_flash_files(self, flash_file_list):
        """
        Check flash file compatibility with flash tool

        :type flash_file_list: list
        :param flash_file_list: list of flash file (absolute paths)

        :rtype: list
        :return: list of valid flash file (absolute paths) for current flash tool
        """
        pft_flash_file_list = []

        if len(flash_file_list) > 0:
            for flash_file in flash_file_list:
                if os.path.isfile(flash_file):
                    filename, fileExtension = os.path.splitext(flash_file)

                    if fileExtension.lower() == ".zip":
                        flash_file_to_use = os.path.join(filename, "flash.xml")
                        if not os.path.isfile(flash_file_to_use):
                            flash_file_to_use = os.path.join(filename, "flash.json")
                        if not os.path.isfile(flash_file_to_use):
                            err_msg = "PFT: Flash file %s not found inside %s" % (flash_file_to_use, str(flash_file))
                            self._logger.error(err_msg)
                            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
                        else:
                            pft_flash_file_list.append(flash_file_to_use)
                    elif fileExtension.lower() == ".xml":
                        self._logger.info("PFT: Flash file %s exists" % str(flash_file))
                        pft_flash_file_list.append(flash_file)
                    else:
                        err_msg = "PFT: No suitable flash file %s (.xml or .zip file should be used)" % str(flash_file)
                        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
                else:
                    err_msg = "PFT: flash file %s does not exist: %s" % str(flash_file)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
        else:
            err_msg = "PFT: No file to flash"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)

        return pft_flash_file_list

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

        # Retrieve callback function in device class to make change its state
        self._switch_device_state_fct = device_flash_info.change_device_state_fct

        # Retrieve flash configuration characteristics
        self.flash_configuration = device_flash_info.flash_configuration
        self.flash_configuration_type = device_flash_info.flash_configuration_type

    def _get_cmd(self):
        """
        Build the flash command

        :rtype: str
        :return: string containing flash tool full command to use for flash execution
        """
        cmd = "%s --cli" % self._flash_tool_cmd

        cmd = "%s -l 4 -f %s" % (cmd, self._flash_file)

        if self.flash_configuration and self.flash_configuration_type:
            cmd += " --configuration %s_%s"% (self.flash_configuration_type, self.flash_configuration)

        if self._serial.upper() not in ["", "NONE", "NOT AVAILABLE", "UNKNOWN"]:
            cmd += " --os-sn %s" % self._serial

        if self._soc_serial not in ["", "NONE", "None"]:
            cmd += " --soc-sn %s" % self._soc_serial

        if platform.system() != "Windows":
            cmd = shlex.split(cmd)

        return cmd

    def _run(self):
        """
        Flash the target.

        :rtype: int
        :return: return code for flash tool command execution or None
        :rtype: str
        :return: error message if any
        """
        return_code, error_message = FlashToolBase._run(self)

        # Valid return_code from flashing command
        if return_code is not None:
            if return_code == 0:

                # if we flash a blankphone, PFT does not restart the board
                # we stay in the current state (should be "FLASH_MODE")
                # else PFT tool reboot the board
                # We suppose that first file flashed is a blankphone
                # In the future we shall ask the device what is its state?
                if self._first_file_flashing is False:
                    # Wait for the boot (after flash, the device is booting)
                    return_code, error_message = self._switch_device_state_fct("WAIT_BOOT")
                    if return_code == Global.SUCCESS:
                        self.exec_result = True
                        self._logger.info("PFT: Flash succeeds - device succeeds to boot : %s" % error_message)
                        self._flash_tool_state = "FLASHED"
                    else:
                        self.exec_result = False
                        self.error_message = "PFT: Flash failure - device fails to boot : %s" % error_message
                        self._flash_tool_state = "UNUSED"
                else:
                    self.exec_result = True
            elif chr(return_code) == "\x11":
                self.exec_result = False
                self.error_message = "Some PFT arguments are not supported. " + \
                                     "Please check if your PhoneFlashTool version is up-to-date"
                self._flash_tool_state = "UNUSED"
            else:
                self.exec_result = False
                self.error_message = "Unexpected result: %d" % return_code
                self._flash_tool_state = "UNUSED"
        else:
            self.exec_result = False
            self.error_message = error_message
            self._flash_tool_state = "UNUSED"

    def _analyze_output(self, data):
        """
        Analyze in real time the output log from flash tool command execution
        and switch the device state when needed

        :type data: str
        :param data: string to analyze
        """
        for d in data:
            self._logger.debug(d)
            # Search in logging trigger for flashing event
            if d != "":
                if any(word in d for word in ["Ready to flash", "Waiting for devices"]):
                    # Switch the device in FLASH MODE state if not already in
                    if self._flash_tool_state == "FLASH_INIT_STATE":
                        # Activate provisionning line to allow PFT to connect device as soon as it
                        # is ready (device in DNX or POS mode)
                        if self._switch_device_state_fct("POS_MODE_ON") == Global.SUCCESS:
                            self._flash_tool_state = "FLASH_MODE_START"
                        else:
                            self.exec_result = False
                            self.error_message = "PFT: Issue can not enable provisionning line on the device"
                            self._flash_tool_state = "UNUSED"
                            FlashToolBase._stop(self)

                if any(word in d for word in ["New Android device", "New SOC device", "Android device detected"]):
                    # Change the device state => Stop the flash mode as now started
                    if self._flash_tool_state == "FLASH_MODE_START":
                        # Deactivate provisionning line as PFT start the flashing.
                        if self._switch_device_state_fct("POS_MODE_OFF") == Global.SUCCESS:
                            self._flash_tool_state = "FLASH_MODE"
                        else:
                            self.exec_result = False
                            self.error_message = "PFT: Issue can not disable provisionning line on the device"
                            self._flash_tool_state = "UNUSED"
                            FlashToolBase._stop(self)

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

        flash_return_code = Global.FAILURE
        # Change the device state => Trig the flash mode
        if self._switch_device_state_fct("FLASH_SWITCH_OFF") == Global.SUCCESS:
            # Device can be flashed
            self._flash_tool_state = "FLASH_INIT_STATE"
            self._first_file_flashing = True

            # Flash all flash files passed
            for flash_file in flash_files:

                self._flash_file = flash_file
                flash_return_code = FlashToolBase._flash(self, timeout)

                self._first_file_flashing = False

                if flash_return_code == Global.FAILURE:
                    # Flash has failed on a flash file => stop the flash procedure (do not flash other flash files)
                    break
        else:
            self._logger.error("PFT: Issue can not switch off the device state")

        if flash_return_code == Global.SUCCESS:
            flash_return_code, _ = self._switch_device_state_fct("WAIT_BOOT")

        return flash_return_code
