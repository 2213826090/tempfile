#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: Pupdr Library - DediprogModule
@since: 11/17/2014
@author: travenex
"""

import os
import platform
import LoggerModule
import HostModule
import MiscModule

class DediprogModule(object):

    __instance = None
    __biosFile = None
    __flashToolCmd = None
    __globalConf = None
    __logger = None
    __host = None
    __misc = None
    __voltage = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def init(self, globalConf=None):
        self.__globalConf = globalConf
        self.__logger = LoggerModule.LoggerModule()
        self.__host = HostModule.HostModule()
        self.__misc = MiscModule.MiscModule()
        self.__biosFile = None
        self.__voltage = "1.8V"
        self.__flashToolCmd = None

    def __checkDediprogSwAvailability(self):
        """
        :rtype: str
        :return: string containing flash tool name to use for flash execution
        """

        # Check the dediprog flash tool installation on bench

        log = "checkDediprogSwAvailability(): "

        if platform.system() == "Windows":
            cmd_name = "dpcmd"
        else:
            cmd_name = "flashrom"

        flashToolCmdPath = self.__misc.which(cmd_name)

        if flashToolCmdPath is None:
            error_msg = log + "Check if DEDIPROG is installed on bench (Command {0} not found !)".format(cmd_name)
            if platform.system() == "Windows":
                error_msg += " - Check also if DEDIPROG folder is referenced on bench in PATH environment variable"
            self.__logger.printLog("WARNING", error_msg)
        else:
            # DEDIPROG software is installed
            cmd = ""
            try:
                # Try to launch DEDIPROG flash tool

                # Analyze for LINUX and WINDOWS, return code to see if DEDIPROG flash tool can be launched
                if platform.system() == "Windows":
                    cmd = cmd_name + " -d"
                else:
                    cmd = cmd_name + " --version"

                return_code, return_message = self.__host.commandExec(cmd, 1, silent_mode=True)

            except Exception as ex:
                err_msg = log + "Flash tool ({0}) execution issue on bench - error: {1}".format(cmd, ex)
                self.__logger.printLog("WARNING", err_msg)
                return None

            if return_code == 0:
                # DEDIPROG flash tool is installed and available
                # Print the flash tool version
                self.__logger.printLog("INFO", log + "Flash tool is installed and running on bench - version : {0}".format(return_message))
            else:
                # DEDIPROG flash tool issue when launching it
                err_msg = log + "Flash tool (command= {0}) execution issue on bench - error: {1}".format(cmd, return_message)
                self.__logger.printLog("WARNING", err_msg)
                return None

        return flashToolCmdPath

    def __runDediprog(self, timeout):
        """
        Build the flash command

        :rtype: str
        :return: string containing flash tool full command to use for flash execution
        """
        log = "runDediprog(): "

        # WARNING: 1.8V voltage IS MANDATORY, IF NOT, HARDWARE CAN BE DAMAGED
        if platform.system() == "Windows":
            # Chip will be auto-detected to avoid the Windows/Linux naming misalignment
            if str(self.__voltage).endswith('V'):
                vcc = dict()
                vcc["3.5V"] = 0
                vcc["2.5V"] = 1
                vcc["1.8V"] = 2
                try:
                    cmd = "{0}  --vcc {1} --log -u {2}".format(vcc[str(self.__voltage)], self.__flashToolCmd, self.__biosFile)
                except KeyError:
                    self.__logger.printLog("WARNING",
                                           log + "Not suitable voltage value {0}, should be 3.5V / 2.5V / 1.8V - "
                                           .format(self.__voltage))
                    return False
            else:
                self.__logger.printLog("WARNING", log + "Not suitable voltage value {0} - Ex : 1.8V".format(self.__voltage))
                return False

        else:
            # Chip will be auto-detected to avoid the Windows/Linux naming misalignment
            if str(self.__voltage).endswith('V'):
                cmd = "{0} -V -p dediprog:voltage={1} -w {2}".format(self.__flashToolCmd, self.__voltage, self.__biosFile)
            else:
                self.__logger.printLog("WARNING", log + "Not suitable voltage value {0} - Ex : 1.8V".format(self.__voltage))
                return False

        # launch command and return
        exec_status, _ = self.__host.commandExec(cmd, timeout)
        if exec_status == 0:
            return True
        else:
            return False

    def flashBinary(self, flash_file, timeout):
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

        self.__flashToolCmd = self.__checkDediprogSwAvailability()
        if not self.__flashToolCmd:
            return False

        self.__biosFile = self.__checkBinaryFile(flash_file)

        if self.__biosFile:
            # Flash file exist => flash it
            flash_return_code = self.__runDediprog(timeout)
        else:
            flash_return_code = False

        return flash_return_code

    def __checkBinaryFile(self, file_path):
        """
        Check if input file is a binary one

        :type file_path: str
        :param file_path: binary file absolute path

        :rtype: str
        :return: binary file absolute path if it is a binary file else None
        """
        log = "checkBinaryFile(): "
        return_file_path = None

        # Check if flash file exist
        if not isinstance(file_path, str):
            self.__logger.printLog("WARNING", log + "Invalid flash file, should be a string!")
        else:
            if file_path in [None, "None", ""]:
                self.__logger.printLog("WARNING", log + "Flash file not defined!")
            else:
                # Normalize path name, also for relative paths
                file_path = os.path.abspath(str(file_path).strip())

                # Check flash file availability and format
                _, file_extension = os.path.splitext(file_path)

                if file_extension.lower() == ".bin":

                    if os.path.isfile(file_path):
                        # Go out the loop as file has been found, and it should have only one .bin file to flash
                        # and put first in the flash file list
                        self.__logger.printLog("INFO", log + "Flash file used for flashing: {0}".format(file_path))
                        return_file_path = file_path
                    else:
                        self.__logger.printLog("WARNING", log + "Flash file {0} not found!".format(file_path))
                else:
                    self.__logger.printLog("WARNING",
                        log + "Flash input file {0}, wrong file extension (it should be .bin)".format(file_path))

        return return_file_path

    def increaseBinSize(self, flashFilePath, newSize):
        """
        Increase the binary size for a .bin extension file to the size specified in entry (in Mbytes)

        :type flashFilePath: str
        :param flashFilePath: contain binary file absolute path

        :type newSize: int
        :param newSize: contain new final size requested for binary file (in Mbytes)

        :rtype: int
        :return: result of the procedure (Global.SUCCESS, Global.FAILURE)

        """
        result_new_size = 0
        return_result = False
        log = "increaseBinSize(): "

        result_file_path = self.__checkBinaryFile(flashFilePath)

        if newSize < 0:
            self.__logger.printLog("WARNING", log + "Can not decrease the size of file: {0} (new size={1})".format(str(flashFilePath), str(newSize)))
        else:
            if newSize == 0:
                self.__logger.printLog("DEBUG", log + "Size of the file '{0}' is not impacted".format(str(flashFilePath)))
            else:
                result_new_size = newSize

        if result_file_path and result_new_size > 0:
            size_before = os.path.getsize(result_file_path)
            added_size = 1024 * 1024 * result_new_size - size_before
            if added_size <= 0:
                self.__logger.printLog("WARNING",
                    log + "binary file size ({0} bytes) greater or equal to the requested size ({1} Mo)".format(str(size_before), str(result_new_size)))
            else:
                try:
                    self.__logger.printLog("INFO", log + "increase binary file size ({0} bytes) to the requested size: {1} Mo".format(str(size_before), str(result_new_size)))
                    # Add 00 ascii parameters
                    with open(result_file_path, 'ab') as f:
                        f.write((added_size - 1) * '\x00' + "\n")

                except Exception as e:
                    self.__logger.printLog("WARNING", log + "Issue while writing the file: {0} ({1})".format(result_file_path, e))
                    pass

                size_after = os.path.getsize(result_file_path)

                if size_after - size_before == added_size:
                    return_result = True
                else:
                    return_result = False

        return return_result