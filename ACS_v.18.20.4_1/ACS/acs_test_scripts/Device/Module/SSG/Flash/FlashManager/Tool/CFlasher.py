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
:summary: This file implements the Cflasher (CFlasher) class
:since: 12/19/2014
:author: syang5
"""

import os
import shlex
import platform
import tarfile
import zipfile
import glob

from acs_test_scripts.Device.Module.SSG.Flash.FlashManager.Tool.FlashToolBase import FlashToolBase
from acs_test_scripts.Device.Module.SSG.Flash.FlashManager.Tool.PFT import PFT
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.CommandLine import CommandLine
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


class CFlasher(PFT):

    """
    This class is the specific class used for CFlasher flash tool
    """

    def __init__(self, logger=None):
        """
        Constructor

        :type logger: object
        :param logger: logger name
        """
        FlashToolBase.__init__(self, "CFlasher", logger)

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
        Check if CFlasher Flash Tool is installed over current ACS bench

        :rtype: str
        :return: string containing flash tool name to use for flash execution
        """
        cmd_name = "cflasher"

        # Check the phone flash tool installation over ACS bench
        flash_tool_cmd_path = CommandLine.which(cmd_name)

        if flash_tool_cmd_path is None:
            error_msg = "Cflasher Check if PHONE FLASH TOOL is installed on ACS bench (Command %s not found !)" % cmd_name
            if platform.system() == "Windows":
                error_msg += " - Check also if PHONE FLASH TOOL folder is referenced on ACS bench in PATH environment variable"
            self._logger.error(error_msg)
            raise AcsConfigException(AcsConfigException.EXTERNAL_LIBRARY_ERROR, error_msg)
        return cmd_name

    def _get_cmd(self):
        """
        Build the flash command

        :rtype: str
        :return: string containing flash tool full command to use for flash execution
        """
        cmd = "%s" % self._flash_tool_cmd

        cmd = "%s -l 5 -f %s --reboot 0" % (cmd, self._flash_file)

        if self._serial.upper() not in ["", "NONE", "NOT AVAILABLE", "UNKNOWN"]:
            cmd += " --os-sn %s" % self._serial

        if self._soc_serial not in ["", "NONE", "None"]:
            cmd += " --soc-sn %s" % self._soc_serial

        if platform.system() != "Windows":
            cmd = shlex.split(cmd)

        return cmd

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

                    if zipfile.is_zipfile(flash_file) or tarfile.is_tarfile(flash_file):
                        flash_files_to_use = glob.glob(os.path.join(filename, 'flash.*'))
                        if len(flash_files_to_use) > 0:
                            for fname in flash_files_to_use:
                                _, fileExtension = os.path.splitext(fname)
                                if fileExtension.lower() in ['.json', '.xml']:
                                    flash_file_to_use = os.path.join(filename, fname)
                                    self._logger.info("CFlasher: Flash file %s exists" % str(flash_file))
                                    pft_flash_file_list.append(flash_file_to_use)
                                else:
                                    err_msg = "CFlasher: No suitable flash file %s (.xml, .json, or .zip file should be used)" % str(flash_file)
                                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
                        else:
                            err_msg = "CFlasher: No suitable flash file %s (.xml, .json, or .zip file should be used)" % str(flash_file)
                            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
                    elif fileExtension.lower() == ".xml":
                        self._logger.info("CFlasher: Flash file %s exists" % str(flash_file))
                        pft_flash_file_list.append(flash_file)
                    else:
                        err_msg = "CFlasher: No suitable flash file %s (.xml or .zip file should be used)" % str(flash_file)
                        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
                else:
                    err_msg = "CFlasher: flash file %s does not exist: %s" % str(flash_file)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
        else:
            err_msg = "CFlasher: No file to flash"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)

        return flash_file_list
