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
:summary: This file implements the Flash Tool for Edison class
:since: 18/02/2014
:author: jreynaux
"""
import os
import stat
import platform
import zipfile
import tempfile

from ErrorHandling.AcsToolException import AcsToolException
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Device.Module.NDG.Flash.FlashManager.Tool.FlashToolBase import FlashToolBase
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.CommandLine import CommandLine
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


class EdisonFlashTool(FlashToolBase):

    """
    This class is the specific class used for Flash tool used on Edison.

    Basically we should ensure that phoneflashtool and dfu-utils are installed on linux devices.
    Then unzip the given file, a flashall.sh should be present on folder.
    The flashall.sh script should be called in order to start flash procedure.
    Then reboot the device, the flash process should start automatically.
    """

    def __init__(self, logger=None):
        """
        Constructor

        :type logger: object
        :param logger: logger name
        """
        FlashToolBase.__init__(self, "EFT", logger)

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
        Check if Flash Tool is installed over current ACS bench

        :rtype: str
        :return: string containing flash tool name to use for flash execution
        """
        if platform.system() == "Windows":
            error_msg = "Flashing of Edison devices with ACS actually not supported !"
            raise AcsConfigException(AcsConfigException.EXTERNAL_LIBRARY_ERROR, error_msg)

        cmd_name = "phoneflashtool"

        # Check the phone flash tool installation over ACS bench
        phone_flash_tool_cmd_path = CommandLine.which(cmd_name)

        if phone_flash_tool_cmd_path is None:
            error_msg = "EFT: Check if PHONE FLASH TOOL is installed on ACS bench (Command %s not found !)" % cmd_name
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.EXTERNAL_LIBRARY_ERROR, error_msg)

        cmd_name = "dfu-util"

        # Check the phone flash tool installation over ACS bench
        dfu_cmd_path = CommandLine.which(cmd_name)

        if dfu_cmd_path is None:
            error_msg = "EFT: Check if dfu-util is installed on ACS bench (Command %s not found !)" % cmd_name
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.EXTERNAL_LIBRARY_ERROR, error_msg)

        # Now set cmd_name as script to be used for flash
        cmd_name = "flashall.sh"

        return cmd_name

    def _check_flash_files(self, flash_file_list):
        """
        Check flash file compatibility with flash tool

        :type flash_file_list: list
        :param flash_file_list: list of flash file (absolute paths)

        :rtype: list
        :return: list of valid flash file (absolute paths) for current flash tool
        """
        eft_flash_files_arg = ""
        if len(flash_file_list) > 1:

            err_msg = "EFT: Need a single zip file as input"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)

        elif len(flash_file_list) == 1:
            # If only one file given, should be a zip
            flash_file = flash_file_list[0]
            filename, file_extension = os.path.splitext(flash_file)
            if ".zip" in file_extension.lower():
                self._logger.debug("EFT: Checking integrity of flash file")
                file_ok = self.__check_zip_file_integrity(flash_file)
                if not os.path.isfile(flash_file) or not file_ok:
                    err_msg = "EFT: Flash file archive %s not found or corrupted !" % (str(flash_file))
                    self._logger.error(err_msg)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
                # Unzip file then return path
                self._logger.debug("EFT: Building files list from zip archive")
                eft_flash_files_arg = self.__get_path_from_zip(flash_file)
            else:
                err_msg = "EFT: No suitable flash file %s (.fls list or zip file should be used)" % str(flash_file)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
        else:
            err_msg = "EFT: No file to flash"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)

        return eft_flash_files_arg

    def _retrieve_device_info(self, device_flash_info):
        """
        Retrieve necessary information from the device for flash tool execution

        :type device_flash_info: FlashDeviceInfo
        :param device_flash_info: contain all device information needed by the flash tool
        """
        # Retrieve callback function in device class to make change its state
        self._switch_device_state_fct = device_flash_info.change_device_state_fct

    def _get_cmd(self):
        """
        Build the flash command

        :rtype: str
        :return: string containing flash tool full command to use for flash execution
        """
        # Now append both to obtain full path to sh script
        cmd = os.path.join(self._flash_file, self._flash_tool_cmd)

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

        self.__check_flash_script()

        self._retrieve_device_info(device_info)

        self._logger.info("EFT: Initiate flashing procedure")
        # Device can be flashed
        self._flash_tool_state = "FLASH_INIT_STATE"
        self._first_file_flashing = True

        # Flash flash file passed
        flash_return_code = FlashToolBase._flash(self, timeout)

        self._first_file_flashing = False

        if flash_return_code == Global.FAILURE:
            # Flash has failed on a flash file => stop the flash procedure (do not flash other flash files)
            err_msg = "EFT: Unable to flash %s (%s)" % (str(self._flash_file), str(self.error_message))
            raise AcsToolException(AcsToolException.CRITICAL_FAILURE, err_msg)

        return flash_return_code

    def _analyze_output(self, data):
        """
        Analyze in real time the output log from flash tool command execution
        and switch the device state when needed

        :type data: str
        :param data: string to analyze
        """
        # FLash initialization
        start_ifw_flash_trigger = ["Detecting Intel Device - Attempt ", "Please plug and reboot the board"]
        # Flash start here
        ifw_flash_trigger = ["IFW flash started", "Flashing IFWI", "Initiating download..."]
        # Final verdict at end of script
        flash_success_trigger = ["complete the flashing procedure"]

        for d in data:
            self._logger.debug(d)
            # Search in logging trigger for flashing event
            if d != "":
                # Trigger for Flash initialization (FLASH_INIT_STATE -> FLASH_MODE_START)
                if any(word in d for word in start_ifw_flash_trigger):
                    # Switch the device in FLASH_MODE_START state if not already in
                    if self._flash_tool_state == "FLASH_INIT_STATE":
                        # Change the device state => Trig the flash mode (reboot device)
                        if self._switch_device_state_fct("FLASH_SWITCH_OFF") == Global.SUCCESS:
                            # if reboot ok should flash, see next step
                            self._logger.debug("Changing flash state to FLASH_MODE_START")
                            self._flash_tool_state = "FLASH_MODE_START"
                            # Log it
                            self._logger.info("EFT: Starting flashing procedure")
                        else:
                            msg = "EFT: Issue can not switch off the device"
                            self._logger.error(msg)
                            self._logger.debug("Changing flash state to FLASH_MODE_STOP")
                            self._flash_tool_state = "FLASH_MODE_STOP"
                            self.exec_result = False
                            self.error_message = msg
                            FlashToolBase._stop(self)
                    else:
                        self._logger.warning("EFT: Wrong flash tool state %s when \"%s\" should be %s" %
                                             (self._flash_tool_state, start_ifw_flash_trigger, "FLASH_INIT_STATE"))

                # Trigger for Flash Start (FLASH_MODE_START -> FLASH_MODE)
                if any(word in d for word in ifw_flash_trigger):
                    # Change the device state => Stop the flash mode as now started
                    if self._flash_tool_state == "FLASH_MODE_START":
                        # Change the device state => Stop the flash mode as now started
                        self._logger.debug("Changing flash state to FLASH_MODE")
                        self._flash_tool_state = "FLASH_MODE"
                        # Log it
                        self._logger.info("EFT: Flashing procedure in progress")
                    else:
                        self._logger.warning("EFT: Wrong flash tool state %s when \"%s\" should be %s" %
                                             (self._flash_tool_state, ifw_flash_trigger, "FLASH_MODE_START"))

                # Trigger final flash verdict (FLASH_MODE -> FLASH_MODE_STOP)
                if any(word in d for word in flash_success_trigger):
                    self._logger.info("EFT: Flashing procedure done successfully")
                    # Change the device state => Stop the flash mode as it finish
                    if self._flash_tool_state == "FLASH_MODE":
                        if self._switch_device_state_fct("WAIT_BOOT") == Global.SUCCESS:
                            # If boot ok, then full flash procedure is ok
                            self._logger.debug("Changing flash state to FLASH_MODE_STOP")
                            self._flash_tool_state = "FLASH_MODE_STOP"
                            # Set final exec result
                            self.exec_result = True
                        else:
                            self._logger.debug("Changing flash state to UNUSED")
                            self._flash_tool_state = "UNUSED"
                            # Set final exec result and error message
                            self.exec_result = False
                            self.error_message = "EFT: Issue can not check board status after flash success"
                            FlashToolBase._stop(self)
                    else:
                        self._logger.warning("EFT: Wrong flash tool state %s when \"%s\" should be %s" %
                                             (self._flash_tool_state, flash_success_trigger[0], "FLASH_MODE"))

    # -----------------------------------------------------------------------------------------------------

    def __get_path_from_zip(self, flash_file):
        # Check
        if self.__check_zip_file_integrity(flash_file):
            return self.__unzip_file_get_path(flash_file)
        else:
            err_msg = "EFT: Unable to treat %s zip flash file" % str(self._flash_file)
            raise AcsToolException(AcsToolException.CRITICAL_FAILURE, err_msg)

    def __check_flash_script(self):
        # Now check availability of flashing script (mean full cmd with no parameters if any)
        cmd = self._get_cmd().split()[0]
        flash_all_cmd_path = CommandLine.exists(cmd)
        if not flash_all_cmd_path:
            error_msg = "EFT: Check if %s script is packaged with build file (not found !))" % self._flash_tool_cmd
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.EXTERNAL_LIBRARY_ERROR, error_msg)
        else:
            if platform.system() == "Linux":
                self._logger.debug("Fix file permission on %s" % cmd)
                # Add execution mode to current mode
                mode = os.stat(cmd).st_mode | stat.S_IEXEC
                # fix file permission
                CommandLine.chmod(cmd, mode)

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

    @staticmethod
    def __unzip_file_get_path(zip_file):
        """
        Unzip the file given as parameter on system temp folder.
        Then provide full path of files.

        :type zip_file: str
        :param zip_file: The path to the zip file

        :rtype: str
        :return: The concatenated str containing the list of file extracted
        """
        # get the filename of file in order to define extract folder
        filename = str(os.path.splitext(os.path.basename(zip_file))[0])
        # build an extract folder path from system temporary folder and filename
        extract_path = os.path.join(tempfile.gettempdir(), filename)
        # Extract the file on it
        with zipfile.ZipFile(zip_file, "a") as z:
            z.extractall(extract_path)

        # return the path where files are extracted
        return extract_path
