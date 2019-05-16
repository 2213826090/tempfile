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
:summary: This file implements the FlashTool (FT) class
:since: 18/02/2014
:author: jreynaux
"""

import os
import platform
import shlex
import zipfile
import tempfile

from ErrorHandling.AcsToolException import AcsToolException
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Device.Module.NDG.Flash.FlashManager.Tool.FlashToolBase import FlashToolBase
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.CommandLine import CommandLine
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


class FlashToolE2(FlashToolBase):

    """
    This class is the specific class used for Flash tool
    """

    def __init__(self, logger=None):
        """
        Constructor

        :type logger: object
        :param logger: logger name
        """
        FlashToolBase.__init__(self, "FTE2", logger)

        # Specific to Flash Tool
        self._logger = logger if logger else LOGGER_TEST_SCRIPT
        self._flash_tool_state = "INIT"
        self._serial = None
        self._soc_serial = None
        self._switch_device_state_fct = None
        self._first_file_flashing = False
        self._pos_mode_on_exec_thread = None
        self._flash_file = None
        self._flash_user_data = None

    def _check_flash_tool_availability(self):
        """
        Check if Flash Tool is installed over current ACS bench

        :rtype: str
        :return: string containing flash tool name to use for flash execution
        """
        cmd_name = "DownloadTool"

        # Check the phone flash tool installation over ACS bench
        flash_tool_cmd_path = CommandLine.which(cmd_name)

        if flash_tool_cmd_path is None:
            error_msg = "FTE2: Check if FLASH TOOL E2 is installed on ACS bench (Command %s not found !)" % cmd_name
            if platform.system() == "Windows":
                error_msg += " - Check also if FLASH TOOL E2 folder is referenced " \
                             "on ACS bench in PATH environment variable"
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
        fte2_flash_files_arg = ""

        if len(flash_file_list) > 1:
            for flash_file in flash_file_list:
                if os.path.isfile(flash_file):
                    filename, file_extension = os.path.splitext(flash_file)

                    if ".fls" in file_extension.lower():
                        if not os.path.isfile(flash_file):
                            err_msg = "FTE2: Flash file %s not found !" % (str(flash_file))
                            self._logger.error(err_msg)
                            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
                        # Build a single arg line with all files
                        fte2_flash_files_arg += flash_file + " "
                    else:
                        err_msg = "FTE2: No suitable flash file %s (.fls list or single zip file should be used)" % \
                                  str(flash_file)
                        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
                else:
                    err_msg = "FTE2: flash file %s does not exist: %s" % str(flash_file)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)

        elif len(flash_file_list) == 1:
            # If only one file given, should be a zip
            flash_file = flash_file_list[0]
            filename, file_extension = os.path.splitext(flash_file)
            if ".zip" in file_extension.lower():
                self._logger.debug("FTE2: Checking integrity of flash file")
                file_ok = self.__check_zip_file_integrity(flash_file)
                if not os.path.isfile(flash_file) or not file_ok:
                    err_msg = "FTE2: Flash file archive %s not found or corrupted !" % (str(flash_file))
                    self._logger.error(err_msg)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
                # Build a list of fls files to flash from zip file given
                self._logger.debug("FTE2: Building files list from zip archive")
                fte2_flash_files_arg = self.__build_list_from_zip(flash_file)
            else:
                err_msg = "FTE2: No suitable flash file %s (.fls list or zip file should be used)" % str(flash_file)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
        else:
            err_msg = "FTE2: No file to flash"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)

        return fte2_flash_files_arg

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
        if platform.system() == "Windows":
            # If windows need to specify usb port to use, here the first one
            cmd = "%s -cu1" % self._flash_tool_cmd
        else:
            cmd = self._flash_tool_cmd

        cmd = "%s %s" % (cmd, self._flash_file)

        if platform.system() != "Windows":
            cmd = shlex.split(cmd)

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
        flash_file = self._check_flash_files(flash_file_list)

        if not self._flash_user_data:
            self._logger.info("FTE2: Flashing procedure will not flash userdata !")
            ff_list = str(flash_file).split(" ")
            flash_file = []
            for f in ff_list:
                if "userdata" not in f:
                    flash_file.append(f)
            flash_file = " ".join(flash_file)

        self._flash_file = flash_file

        self._retrieve_device_info(device_info)

        flash_return_code = Global.FAILURE

        # Disconnect USB
        if self._switch_device_state_fct("COS_MODE_OFF") == Global.FAILURE:
            err_msg = "FTE2: Unable to switch to COS_MODE_OFF"
            raise AcsToolException(AcsToolException.CRITICAL_FAILURE, err_msg)

        # Change the device state => Trig the flash mode
        if self._switch_device_state_fct("FLASH_SWITCH_OFF") == Global.SUCCESS:
            self._logger.info("FTE2: Initiate flashing procedure")
            # Device can be flashed
            self._flash_tool_state = "FLASH_INIT_STATE"
            self._first_file_flashing = True

            # Flash all flash files passed
            flash_return_code = FlashToolBase._flash(self, timeout)

            self._first_file_flashing = False

            if flash_return_code == Global.FAILURE:
                # Flash has failed on a flash file => stop the flash procedure (do not flash other flash files)
                err_msg = "FTE2: Unable to flash %s" % str(self._flash_file)
                raise AcsToolException(AcsToolException.CRITICAL_FAILURE, err_msg)
        else:
            self._logger.error("FTE2: Issue can not switch off the device state")

        return flash_return_code

    def _analyze_output(self, data):
        """
        Analyze in real time the output log from flash tool command execution
        and switch the device state when needed

        :type data: str
        :param data: string to analyze
        """
        # FLash initialization
        flash_mode_start_trigger = ["Please reboot your phone device"]
        # Flash start here
        flash_mode_trigger = ["Device synchronized"]
        # Final verdict at end of script
        flash_mode_stop_trigger = ["Success!"]

        flash_error_trigger = ["Error opening device"]

        for d in data:
            self._logger.debug(d)
            # Search in logging trigger for flashing event
            if d != "":
                if any(word in d for word in flash_mode_start_trigger):
                    # Switch the device in FLASH MODE state if not already in
                    if self._flash_tool_state == "FLASH_INIT_STATE":
                        # Activate usb host line to allow FTE2 to connect device as soon as it
                        # is ready (device in DNX or COS mode)
                        if self._switch_device_state_fct("COS_MODE_ON") == Global.SUCCESS:
                            self._logger.debug("Changing flash state to FLASH_MODE_START")
                            self._flash_tool_state = "FLASH_MODE_START"
                            # Log it
                            self._logger.info("FTE2: Starting flashing procedure")
                        else:
                            self.exec_result = False
                            self.error_message = "FTE2: Issue can not enable usb host line on the device"
                            self._flash_tool_state = "UNUSED"
                            FlashToolBase._stop(self)

                if any(word in d for word in flash_mode_trigger):
                    # Change the device state => Stop the flash mode as now started
                    if self._flash_tool_state == "FLASH_MODE_START":
                        # Change the device state => Stop the flash mode as now started
                        self._logger.debug("Changing flash state to FLASH_MODE")
                        self._flash_tool_state = "FLASH_MODE"
                        # Log it
                        self._logger.info("FTE2: Flashing procedure in progress")

                if any(word in d for word in flash_mode_stop_trigger):
                    self._logger.info("FTE2: Flashing procedure done successfully")
                    # Change the device state => Stop the flash mode as it finish
                    if self._flash_tool_state == "FLASH_MODE":
                        code, message = self._switch_device_state_fct("WAIT_BOOT")
                        if code == Global.SUCCESS:
                            self._logger.debug("Changing flash state to FLASH_MODE_STOP")
                            self._flash_tool_state = "FLASH_MODE_STOP"
                            self.exec_result = True
                        else:
                            self.exec_result = False
                            self.error_message = "FTE2: Issue can not check board status after flash success (%s)" \
                                                 % str(message)
                            self._flash_tool_state = "UNUSED"
                            FlashToolBase._stop(self)

                if any(word in d for word in flash_error_trigger):
                    self._logger.error("FTE2: An error occur during flash procedure !")
                    # Change the device state => Stop the flash mode as it finish
                    self.exec_result = False
                    self.error_message = "FTE2: An error occur during flash procedure ! (%s)" \
                                         % str(d)
                    self._flash_tool_state = "UNUSED"
                    FlashToolBase._stop(self)

    # -----------------------------------------------------------------------------------------------------

    def __build_list_from_zip(self, flash_file):
        # Check
        if self.__check_zip_file_integrity(flash_file):
            return self.__unzip_file_get_list(flash_file)
        else:
            err_msg = "FTE2: Unable to treat %s zip flash file" % str(self._flash_file)
            raise AcsToolException(AcsToolException.CRITICAL_FAILURE, err_msg)

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
    def __unzip_file_get_list(zip_file):
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
        with zipfile.ZipFile(zip_file, "r") as z:
            z.extractall(extract_path)
        # return the list of files extracted
        # list_of_files = os.listdir(extract_path)
        list_of_files = [os.path.join(extract_path, f) for f in os.listdir(extract_path)]
        return " ".join(list_of_files)
