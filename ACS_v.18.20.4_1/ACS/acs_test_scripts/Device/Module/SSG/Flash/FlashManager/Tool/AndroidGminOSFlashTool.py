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

:organization: INTEL MCG PSI
:summary: This file implements the GMIN TCP-IP target platforms
:since: 3/4/2014
:author: mceausu
"""

import os
import platform
import shlex

from acs_test_scripts.Device.Module.SSG.Flash.FlashManager.Tool.FlashToolBase import FlashToolBase
from acs_test_scripts.Device.Module.SSG.Flash.FlashManager.Tool.AndroidOSFlashTool import AndroidOSFlashTool
from ErrorHandling.AcsConfigException import AcsConfigException


class AndroidGminOSFlashTool(AndroidOSFlashTool):

    """
    This class is the specific class used for Gmin flash tool
    """
    DEFAULT_CI_SCRIPT = ["flash-ci.sh", "flash-ci.bat"]
    DEFAULT_OS_IMG = []

    def _check_flash_tool_availability(self):
        """
        Assume fastboot command is available on all Android phone
        """
        cmd_name = 'flash-ci.sh'
        if platform.system() == "Windows":
            cmd_name = "flash-ci.bat"
        return cmd_name

    def _check_flash_files(self, flash_file_list):
        """
        Check flash file compatibility with flash tool

        :type flash_file_list: list
        :param flash_file_list: list of flash file (absolute paths)
        :rtype: list
        :return: folder path (absolute path) containing all flash files
        """
        android_flash_file_list = []
        flash_file_folder = ""
        folder_flash_path = None
        if len(flash_file_list) > 0:
            for flash_file in flash_file_list:
                filename, file_extension = os.path.splitext(flash_file)
                if flash_file.endswith((".tgz", ".zip")):
                    flash_file_folder = filename
                    for file in os.listdir(flash_file_folder):
                        if file.endswith((".sh", ".bat")):
                            android_flash_file_list.append(file)
                else:
                    err_msg = "AndroidGminOSFlashTool: flash file format is not suitable (should be .tgz) for file %s" % str(
                        flash_file)
                    self._logger.warning(err_msg)
        if not android_flash_file_list:
            err_msg = "AndroidGminOSFlashTool: No files to flash found"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
        else:
            # check boot.img,recovery.img, system.img file presence
            all_img_presence = set(self.DEFAULT_CI_SCRIPT).issubset(android_flash_file_list)
            if not all_img_presence:
                missing_ff = set(self.DEFAULT_CI_SCRIPT).difference(android_flash_file_list)
                err_msg = "AndroidGminOSFlashTool: missing a flash file : %s" % " ,".join(missing_ff)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
            else:
                # all needed files are present
                # Return folder name containing all the files
                # Assumption: all files are in the same folder
                folder_flash_path = flash_file_folder
        return folder_flash_path

    def _get_cmd(self):
        """
        Build the flash command
        """
        if self._adb_over_ethernet:
            cmd = self._flash_tool_cmd + " -t %s" % self._pos_serial
        else:
            cmd = self._flash_tool_cmd + " -s %s" % self._pos_serial
        if platform.system() != "Windows":
            cmd = "sh %s" % cmd
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
        cur_dir = os.getcwd()
        try:
            os.chdir(self._flash_file)
            return_code, error_message = FlashToolBase._run(self)
        finally:
            os.chdir(cur_dir)

        # Valid return_code from flashing command
        if return_code is not None:
            if return_code != 0:
                self.exec_result = False
                self.error_message = "AndroidGminOSFlashTool: Unexpected result: %d" % return_code
            else:
                self.exec_result = True
                self._logger.info("AndroidGminOSFlashTool: Flash succeeds")
                self._flash_tool_state = "FLASHED"

        else:
            self.exec_result = False
            self.error_message = error_message
