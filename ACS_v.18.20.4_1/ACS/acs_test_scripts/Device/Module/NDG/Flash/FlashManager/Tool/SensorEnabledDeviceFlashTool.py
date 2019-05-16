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

:organization: INTEL NDG SW Dev
:summary: This file implements the Flash Tool for Nordic devices (Clark, Lois, Diana ...)
:since: 15/09/2014
:author: jreynaux
"""

import os
import stat
import platform
import re
import zipfile
import tempfile

from ErrorHandling.AcsToolException import AcsToolException
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Device.Module.NDG.Flash.FlashManager.Tool.FlashToolBase import FlashToolBase
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.CommandLine import CommandLine
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT

BOARD_TYPE_LOIS = "lois"
BOARD_TYPE_CLARK = "clark"


class SensorEnabledDeviceFlashTool(FlashToolBase):
    """
    This class is the specific class used for Flash tool used on Nordic device.

    Basically we should ensure that JLinkExe is installed on linux host pc.
    Then unzip the given file, a flash.sh should be present on folder.
    The flash.sh script should be called in order to start flash procedure.
    """

    def __init__(self, logger=None):
        """
        Constructor

        :type logger: object
        :param logger: logger name
        """
        FlashToolBase.__init__(self, "SED_FT", logger)

        # Specific to Flash Tool
        self._logger = logger if logger else LOGGER_TEST_SCRIPT
        self._flash_tool_state = "INIT"
        self._serial = None
        self._soc_serial = None
        self._switch_device_state_fct = None
        self._first_file_flashing = False
        self._pos_mode_on_exec_thread = None
        self._flash_file = None

        self._board_type = None

    def _check_flash_tool_availability(self):
        """
        Check if Flash Tool is installed over current ACS bench

        :rtype: str
        :return: string containing flash tool name to use for flash execution
        """
        if str(platform.system()) in ["Windows", "Linux"]:
            cmd_name = "cflasher"
        else:
            error_msg = "SED_FT: Unknown platform (Only Windows and Linux supported)"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.EXTERNAL_LIBRARY_ERROR, error_msg)

        # Check the phone flash tool installation over ACS bench
        phone_flash_tool_cmd_path = CommandLine.which(cmd_name)
        if phone_flash_tool_cmd_path is None:
            error_msg = "SED_FT: Check if PhoneFlashTool software is installed on ACS bench (Command %s not found !)" % cmd_name
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.EXTERNAL_LIBRARY_ERROR, error_msg)

        # Check the JLink tool installation over ACS bench
        jlink_tool_cmd_path = CommandLine.which("JLinkExe")
        if jlink_tool_cmd_path is None:
            error_msg = "SED_FT: Check if Segger J-Link software is installed on ACS bench (Command %s not found !)" % cmd_name
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
        if len(flash_file_list) > 1:

            err_msg = "SED_FT: Need a single zip file as input"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)

        elif len(flash_file_list) == 1:
            # If only one file given, should be a zip
            flash_file = flash_file_list[0]
            filename, file_extension = os.path.splitext(flash_file)
            if ".zip" in file_extension.lower():
                self._logger.debug("SED_FT: Checking integrity of flash file")
                file_ok = self.__check_zip_file_integrity(flash_file)
                if not os.path.isfile(flash_file) or not file_ok:
                    err_msg = "SED_FT: Flash file archive %s not found or corrupted !" % (str(flash_file))
                    self._logger.error(err_msg)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)

                # # Unzip file then return path
                # self._logger.debug("SED_FT: Building files list from zip archive")
                # nft_flash_files_arg = self.__get_path_from_zip(flash_file)
            else:
                err_msg = "SED_FT: No suitable flash file %s (.fls list or zip file should be used)" % str(flash_file)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
        else:
            err_msg = "SED_FT: No file to flash"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)

        # return nft_flash_files_arg
        return flash_file

    def _retrieve_build_info_from_flash_files(self, flash_files):
        """
        Retrieve build information from flash files.

        :type flash_files: list
        :param flash_files: path to flash files
        :rtype: dict
        :return: Dictionary containing build properties
        """
        # Initialize variables
        return {}

    def _retrieve_device_info(self, device_flash_info):
        """
        Retrieve necessary information from the device for flash tool execution

        :type device_flash_info: FlashDeviceInfo
        :param device_flash_info: contain all device information needed by the flash tool
        """
        # Retrieve callback function in device class to make change its state
        self._switch_device_state_fct = device_flash_info.change_device_state_fct
        # Retrieve board type because may used on flash command
        self._board_type = device_flash_info.board_type

    def _get_cmd(self):
        """
        Build the flash command

        :rtype: str
        :return: string containing flash tool full command to use for flash execution
        """
        # Append -f <flash file> -x auto.json parameter to blank the device on first flash
        cmd = "{0} -f {1} -x auto.json" .format(self._flash_tool_cmd, self._flash_file)

        # then configure device type
        if BOARD_TYPE_LOIS in self._board_type.lower():
            cmd = "{0} -c {1}".format(cmd, BOARD_TYPE_LOIS)
        elif BOARD_TYPE_CLARK in self._board_type.lower():
            cmd = "{0} -c {1}".format(cmd, BOARD_TYPE_CLARK)

        # If specific flash options given, append it to command
        if self._flash_options not in [None, ""]:
            cmd += " {0}".format(str(self._flash_options).strip())

        return cmd

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
        self._flash_file = self._check_flash_files(flash_file_list)

        self._retrieve_device_info(device_info)

        cmd = self._get_cmd()

        self._logger.info("SED_FT: Initiate flashing procedure")
        # Device can be flashed
        self._flash_tool_state = "FLASH_INIT_STATE"
        self._first_file_flashing = True

        # Flash flash file passed
        flash_return_code = self._flash(timeout)

        self._first_file_flashing = False

        if flash_return_code == Global.FAILURE:
            # Flash has failed on a flash file => stop the flash procedure (do not flash other flash files)
            err_msg = "SED_FT: Unable to flash %s (%s)" % (str(self._flash_file), str(self.error_message))
            raise AcsToolException(AcsToolException.CRITICAL_FAILURE, err_msg)

        return flash_return_code

    def _flash(self, timeout):
        """
        Start the flash execution for the flash tool

        :type timeout: int
        :param timeout: max time in seconds to do flash procedure
        :rtype: int
        :return: result of the flash procedure(Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        """

        # Start the flash tool thread
        # return_code, error_message = CmdExecManager._start(self, timeout, execute_in_thread=False)
        return_code, error_message = super(FlashToolBase, self)._start(timeout, execute_in_thread=False)

        # Check result
        if self.exec_result and return_code == 0:
            self._logger.info("%s: FLASH SUCCESS (file= %s)" % (self._flash_tool_name, self._flash_file))
            flash_operation = Global.SUCCESS
        else:
            self._logger.error("%s: FLASH FAILURE (file= %s, error=%s)" % (
                self._flash_tool_name, self._flash_file, self.error_message))
            flash_operation = Global.FAILURE

        return flash_operation

    def _analyze_output(self, data):
        """
        Analyze in real time the output log from flash tool command execution
        and switch the device state when needed

        :type data: str
        :param data: string to analyze
        """
        # Flash start here
        start_ifw_flash_trigger = ["Start flashing"]
        # Final verdict at end of script
        flash_success_trigger = ["Flash success"]

        for d in data:
            self._logger.debug(d)
            # Search in logging trigger for flashing event
            if d != "":

                # Trigger for Flash initialization (FLASH_INIT_STATE -> FLASH_MODE_START)
                if any(word in d for word in start_ifw_flash_trigger):
                    # Switch the device in FLASH_MODE_START state if not already in
                    if self._flash_tool_state == "FLASH_INIT_STATE":
                        # Change the device state => Trig the flash mode (reboot device)
                        # if reboot ok should flash, see next step
                        self._logger.debug("Changing flash state to FLASH_MODE_START")
                        self._flash_tool_state = "FLASH_MODE"
                        # Log it
                        self._logger.info("SED_FT: Starting flashing procedure")

                # Trigger final flash verdict (FLASH_MODE -> FLASH_MODE_STOP)
                if any(word in d for word in flash_success_trigger):
                    self._logger.info("SED_FT: Flashing procedure done successfully")
                    # Change the device state => Stop the flash mode as it finish
                    if self._flash_tool_state == "FLASH_MODE":
                        if self._switch_device_state_fct("WAIT_BOOT") == Global.SUCCESS:
                            # If boot ok, then full flash procedure is ok
                            self._logger.debug("Changing flash state to FLASH_MODE_STOP")
                            self._flash_tool_state = "FLASH_MODE_STOP"
                            # Set final exec result
                            self.exec_result = True

    @staticmethod
    def __check_zip_file_integrity(zip_file):
        """
        Check if the given file is a valid zip file for flashing.

        :type zip_file: str
        :param zip_file: The zip file to check

        :rtype: bool
        :return: True or False whether the given file is valid or not.
        """
        file_state = False
        try:
            the_zip_file = zipfile.ZipFile(zip_file)
            ret = the_zip_file.testzip()

            if ret is None:
                file_state = True
        except Exception as ex:
            print str(ex)

        return file_state