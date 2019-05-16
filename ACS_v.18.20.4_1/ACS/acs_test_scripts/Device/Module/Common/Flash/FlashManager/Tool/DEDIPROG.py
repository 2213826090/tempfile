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
:summary: This file implements the Dediprog (DEDIPROG) class (bios flash tool)
:since: 09/07/2013
:author: lbavois
"""

import os
import platform
import shlex

from lxml import etree

from acs_test_scripts.Device.Module.Common.Flash.FlashManager.Tool.FlashToolBase import FlashToolBase
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.CommandLine import CommandLine
from UtilitiesFWK.Utilities import internal_shell_exec
from UtilitiesFWK.Utilities import Global
from Core.Report.ACSLogging import LOGGER_FWK


class DEDIPROG(FlashToolBase):

    """
    This class is the specific class used for DEDIPROG flash tool
    """

    def __init__(self, logger=None, voltage="1.8V"):
        """
        Constructor

        :type logger: object
        :param logger: logger name
        """
        FlashToolBase.__init__(self, "DEDIPROG", logger)

        # Specific to Flash Tool
        self._logger = logger if logger else LOGGER_FWK
        self._flash_tool_state = "UNUSED"
        self._switch_device_state_fct = None
        self._flash_file = None
        self._voltage = str(voltage)

    def _check_flash_tool_availability(self):
        """
        Check if Dediprog Flash Tool is installed over current ACS bench

        :rtype: str
        :return: string containing flash tool name to use for flash execution
        """
        cmd_name = ""

        # Check the dediprog flash tool installation over ACS bench
        if platform.system() == "Windows":
            cmd_name = "dpcmd"
        else:
            cmd_name = "flashrom"

        flash_tool_cmd_path = CommandLine.which(cmd_name)

        if flash_tool_cmd_path is None:
            error_msg = "DEDIPROG: Check if DEDIPROG is installed on ACS bench (Command %s not found !)" % cmd_name
            if platform.system() == "Windows":
                error_msg += " - Check also if DEDIPROG folder is referenced on ACS bench in PATH environment variable"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.EXTERNAL_LIBRARY_ERROR, error_msg)
        else:
            # DEDIPROG software is installed
            try:
                # Try to launch DEDIPROG flash tool

                # Analyze for LINUX and WINDOWS, return code to see if DEDIPROG flash tool is launchable
                if platform.system() == "Windows":
                    cmd = cmd_name + " -d"
                else:
                    cmd = cmd_name + " --version"

                return_code, return_message = internal_shell_exec(cmd, 1, silent_mode=True)

            except Exception as ex:
                err_msg = "DEDIPROG: Flash tool (%s) execution issue over ACS bench - error: %s" % (cmd, str(ex))
                self._logger.error(err_msg)
                raise AcsConfigException(AcsConfigException.EXTERNAL_LIBRARY_ERROR, err_msg)

            if return_code == 0:
                # DEDIPROG flash tool is installed and available
                # Print the flash tool version
                self._logger.info("DEDIPROG: Flash tool is installed and running over ACS bench - version : %s" %
                                  str(return_message))
            else:
                # DEDIPROG flash tool issue when launching it
                err_msg = "DEDIPROG: Flash tool (command= %s) execution issue over ACS bench - " % (cmd) + \
                          " - error: %s" % (str(return_message))
                self._logger.error(err_msg)
                raise AcsConfigException(AcsConfigException.EXTERNAL_LIBRARY_ERROR, err_msg)

        return cmd_name

    def _check_flash_files(self, flash_file_list):
        """
        Check flash file compatibility with flash tool

        :type flash_file_list: list
        :param flash_file_list: list of flash file (absolute paths)

        :rtype: str
        :return: absolute path for the file that shall be flashed
        """

        # Capture all error while parsing flash files
        dediprog_flash_file_path = ""

        for flash_file in flash_file_list:
            filename, file_extension = os.path.splitext(flash_file)

            if file_extension.lower() == ".zip":
                flash_file_to_use = os.path.join(filename, "flash.xml")
                if not os.path.isfile(flash_file_to_use):
                    err_msg = "DEDIPROG: Flash file %s not found inside %s" % (flash_file_to_use, str(flash_file))
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
                else:
                    # Update flash file to use
                    flash_file = flash_file_to_use
                    filename, file_extension = os.path.splitext(flash_file)

            if file_extension.lower() == ".xml":
                if os.path.isfile(flash_file):
                    # Only parse .xml file
                    try:
                        flash_xml_doc = etree.parse(flash_file)
                        dediprog_flash_file = flash_xml_doc.xpath(
                            '//flashfile/code_group[@name="FIRMWARE"]/file[starts-with(@TYPE, "DEDIPROG")]/name')
                        if dediprog_flash_file:
                            flash_file = os.path.join(os.path.dirname(flash_file), dediprog_flash_file[0].text)
                            _, file_extension = os.path.splitext(flash_file)
                    except Exception as ex:  # pylint: disable=W0703
                        self._logger.warning("DEDIPROG: Error while parsing %s - Error= %s" % (
                            str(flash_file), str(ex)))
                else:
                    self._logger.warning("DEDIPROG: Flash file %s not found!" % str(flash_file))

            if file_extension.lower() == ".bin":

                if os.path.isfile(flash_file):
                    # Go out the loop as file has been found, and it should have only one .bin file to flash
                    # and put first in the flash file list
                    self._logger.info("DEDIPROG: Flash file used for flash : %s" % str(flash_file))
                    dediprog_flash_file_path = flash_file
                    break
                else:
                    self._logger.warning("DEDIPROG: Flash file %s not found!" % str(flash_file))
            else:
                self._logger.warning("DEDIPROG: Flash input file %s, " % str(flash_file) +
                                     "wrong file extension (it should be .zip, .xml, .bin)")

        # Test if file exists
        if dediprog_flash_file_path == "":
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "DEDIPROG: No suitable flash file or flash file does not exist")
        else:
            return dediprog_flash_file_path

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

        # WARNING: 1.8V voltage IS MANDATORY, IF NOT, HARDWARE CAN BE DAMAGED
        if platform.system() == "Windows":
            # Chip will be auto-detected to avoid the Windows/Linux naming misalignment
            if str(self._voltage).endswith('V'):
                vcc = dict()
                vcc["3.5V"] = 0
                vcc["2.5V"] = 1
                vcc["1.8V"] = 2
                try:
                    cmd = "%s  --vcc %s --log -u %s" % (vcc[str(self._voltage)], self._flash_tool_cmd, self._flash_file)
                except KeyError:
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                             "DEDIPROG: No suitable voltage value %s, should be 3.5V / 2.5V / 1.8V - " % str(
                                                 self._voltage))
            else:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "DEDIPROG: No suitable voltage value %s - Ex : 1.8V" % str(self._voltage))

        else:
            # Chip will be auto-detected to avoid the Windows/Linux naming misalignment
            if str(self._voltage).endswith('V'):
                cmd = "%s -V -p dediprog:voltage=%s -w %s" % (self._flash_tool_cmd,
                                                              self._voltage, self._flash_file)
                cmd = shlex.split(cmd)
            else:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "DEDIPROG: No suitable voltage value %s - Ex : 1.8V" % str(self._voltage))

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
            if return_code != 0:
                self.exec_result = False
                self.error_message = "DEDIPROG: Unexpected result: %d" % return_code
            else:
                self.exec_result = True
        else:
            self.exec_result = False
            self.error_message = error_message

    def _analyze_output(self, data):
        """
        Analyze log output from flash tool

        :type data: str
        :param data: string to analyze
        """
        for d in data:
            # Don't log elapse time in dediprog to avoid overflowing buffer
            if d != "" and not any(word in d for word in ["Elapse", "elapse"]):
                if any(word in d for word in ["Error", "error"]):
                    self._logger.error(d)
                else:
                    self._logger.debug(d)

    def set_voltage(self, voltage):

        self._voltage = str(voltage)

    def reset_voltage(self):

        self._voltage = "1.8V"

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

        # Change the device state => Trig the flash mode
        if self._switch_device_state_fct("FLASH_SWITCH_OFF") == Global.SUCCESS:
            # Device can be flashed
            self._flash_tool_state = "FLASH_MODE"
            flash_return_code = FlashToolBase._flash(self, timeout)
        else:
            self._logger.error("DEDIPROG: Issue can not switch off the device state")
            flash_return_code = Global.FAILURE

        return flash_return_code

    def flash_binary(self, flash_file, timeout):
        """
        Start the flash execution for the current flash tool (DEDIPROG)

        .. note:: This flash procedure does not trig any device state change

        :type flash_file: str
        :param flash_file: flash file absolute path
        :type timeout: int
        :param timeout: max time in seconds to do flash procedure

        :rtype: int
        :return: result of the flash procedure (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        """

        self._flash_file = self._check_binary_file(flash_file)

        if self._flash_file:
            # Flash file exist => flash it
            flash_return_code = FlashToolBase._flash(self, timeout)
        else:
            flash_return_code = Global.FAILURE

        return flash_return_code

    def _check_binary_file(self, file_path):
        """
        Check if input file is a binary one

        :type file_path: str
        :param file_path: binary file absolute path

        :rtype: str
        :return: binary file absolute path if it is a binary file else None
        """

        return_file_path = None

        # Check if flash file exist
        if not isinstance(file_path, str):
            self._logger.error("DEDIPROG: Invalid flash file, should be a string!")
        else:
            if file_path in [None, "None", ""]:
                self._logger.error("DEDIPROG: Flash file not defined!")
            else:
                # Normalize path name, also for relative paths
                file_path = os.path.abspath(str(file_path).strip())

                # Check flash file availability and format
                _, file_extension = os.path.splitext(file_path)

                if file_extension.lower() == ".bin":

                    if os.path.isfile(file_path):
                        # Go out the loop as file has been found, and it should have only one .bin file to flash
                        # and put first in the flash file list
                        self._logger.info("DEDIPROG: Flash file used for flash : %s" % str(file_path))
                        return_file_path = file_path
                    else:
                        self._logger.error("DEDIPROG: Flash file %s not found!" % str(file_path))
                else:
                    self._logger.error(
                        "DEDIPROG: Flash input file %s, wrong file extension (it should be .bin)" % str(file_path))

        return return_file_path

    def increase_bin_size(self, flashFilePath, newSize):
        """
        Increase the binary size for a .bin extension file to the size specified in entry (in Mbytes)

        :type flashFilePath: str
        :param flashFilePath: contain binary file absolute path

        :type newSize: int
        :param newSize: contain new final size requested for binary file (in Mbytes)

        :rtype: int
        :return: result of the procedure (Global.SUCCESS, Global.FAILURE)

        """
        result_file_path = ""
        result_new_size = 0
        return_result = Global.FAILURE

        result_file_path = self._check_binary_file(flashFilePath)

        if newSize < 0:
            self._logger.error("DEDIPROG: Can not decrease the size of the file %s (new size = %s)" % (
                str(flashFilePath), str(newSize)))
        else:
            if newSize == 0:
                self._logger.warning("DEDIPROG: Size of the file %s is not impacted" % str(flashFilePath))
            else:
                result_new_size = newSize

        if result_file_path and result_new_size > 0:
            size_before = os.path.getsize(result_file_path)
            added_size = 1024 * 1024 * result_new_size - size_before
            if added_size <= 0:
                self._logger.error(
                    "DEDIPROG: binary file size (%s bytes) greater or equal to the requested size (%s Mo)"
                    % (str(size_before), str(result_new_size)))
            else:
                try:
                    self._logger.info("DEDIPROG: increase binary file size (%s bytes) to the requested size (%s) Mo"
                                      % (str(size_before), str(result_new_size)))
                    # Add 00 ascii parameters
                    with open(result_file_path, 'ab') as f:
                        f.write((added_size - 1) * '\x00' + "\n")

                except:
                    self._logger.error("DEDIPROG: Issue while writing the file %s" % result_file_path)
                    pass

                size_after = os.path.getsize(result_file_path)

                if size_after - size_before == added_size:
                    return_result = Global.SUCCESS
                else:
                    return_result = Global.FAILURE

        return return_result
