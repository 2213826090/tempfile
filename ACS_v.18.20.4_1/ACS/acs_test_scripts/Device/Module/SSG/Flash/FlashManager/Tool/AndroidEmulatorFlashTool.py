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
:since: 5/27/2014
:author: rcstamat
"""

import os
import platform
import zipfile
import tarfile
import subprocess
import fnmatch

from acs_test_scripts.Device.Module.SSG.Flash.FlashManager.Tool.FlashToolBase import FlashToolBase
from acs_test_scripts.Device.Module.SSG.Flash.FlashManager.Tool.AndroidOSFlashTool import AndroidOSFlashTool
from ErrorHandling.AcsConfigException import AcsConfigException


class AndroidEmulatorFlashTool(AndroidOSFlashTool):
    """
    This class is the specific class used for flashing the emulator
    """

    def __init__(self, logger=None):
        """
        Constructor

        :type logger: object
        :param logger: logger name
        """
        AndroidOSFlashTool.__init__(self, logger)
        self.path_to_android = ""
        self.path_to_emulator = ""
        self.path_to_image = ""

    def _check_flash_tool_availability(self):
        """
        Need to overwrite this function. It doesn't have any pupose.
        """
        return "dir" if platform.system() == "Windows" else "ls"

    def _check_flash_files(self, flash_file_list):
        """
        Check flash file compatibility with flash tool

        :type flash_file_list: list
        :param flash_file_list: list of flash file (absolute paths)
        :rtype: list
        :return: folder path (absolute path) containing all flash files
        """

        flash_file_folder = ""
        folder_flash_path = None

        if len(flash_file_list) > 0:
            for flash_file in flash_file_list:
                filename, file_extension = os.path.splitext(flash_file)
                if flash_file.endswith((".tgz", ".zip")):
                    if zipfile.is_zipfile(flash_file):
                        try:
                            zip_file = zipfile.ZipFile(flash_file, "r")
                            self._logger.info("AndroidEmulatorFlashTool: unzip flash file (%s)" % flash_file)
                            zip_file.extractall(filename)
                            zip_file.close()
                        except Exception as ex:
                            err_msg = "AndroidEmulatorFlashTool: Flash input file %s, unzip issue, error : %s" % (
                            flash_file, str(ex))
                            self._logger.error(err_msg)
                            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
                    elif tarfile.is_tarfile(flash_file):
                        try:
                            tar_file = tarfile.open(flash_file, "r")
                            self._logger.info("AndroidEmulatorFlashTool: untar flash file (%s)" % flash_file)
                            tar_file.extractall(filename)
                            tar_file.close()
                        except Exception as ex:
                            err_msg = "AndroidEmulatorFlashTool: Flash input file %s, untar issue, error : %s" % (
                            flash_file, str(ex))
                            self._logger.error(err_msg)
                            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
                    else:
                        err_msg = "AndroidEmulatorFlashTool: Not a valid archive"
                        raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
                    # Check availability of image folder, the android and emulator executables.
                    # Also give all permisions to the folder.
                    flash_file_folder = "".join(filename)
                    os.system("chmod -R 777 %s" %flash_file_folder)
                    for subdir, dirs, files in os.walk(flash_file_folder):
                        for dir in dirs:
                            if ((dir == "x86") or (dir == "armeabi-v7a")) and ("system-images" in subdir):
                                self.path_to_image = os.path.join(subdir, dir)
                                os.environ['DIR_PATH'] = self.path_to_image
                                break;
                        for file in files:
                            if (file == "android"):
                                self.path_to_android = os.path.join(subdir, file)
                                #self._logger.info("The 'android' tool is located here: ", path_to_android)
                                os.chmod(self.path_to_android, 0755)
                            # Search for all files that contain emulator but are not .jpg or .html files
                            # Example emulator, emulator-x86, emulator64-arm ...
                            if fnmatch.fnmatch(file, 'emulator*') and not fnmatch.fnmatch(file, 'emulator*.*'):
                                path_to_generic = os.path.join(subdir, file)
                                if (file == 'emulator'):
                                    self.path_to_emulator = os.path.join(subdir, file)
                                    os.environ['EMULATOR_PATH'] = self.path_to_emulator
                else:
                    err_msg = "AndroidEmulatorFlashTool: flash file format is not suitable (should be .tgz) for file %s" % str(
                        flash_file)
                    self._logger.warning(err_msg)
        if not self.path_to_android or not self.path_to_emulator:
            err_msg = "AndroidEmulatorFlashTool: android or emulator not found in the archive"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
        else:
            folder_flash_path = flash_file_folder
        return folder_flash_path

    def _get_cmd(self):
        """
        No verification for flashing. It will be done at start-up.
        """
        cmd = "ls"
        return cmd if platform.system() != "Windows" else None

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
                self.error_message = "AndroidEmulatorFlashTool: Unexpected result: %d" % return_code
            else:
                self.exec_result = True
                self._logger.info("AndroidEmulatorFlashTool: Flash succeeds")
                self._flash_tool_state = "FLASHED"

        else:
            self.exec_result = False
            self.error_message = error_message

    def flash(self, flash_file_list, device_info, timeout):
        self._flash_file = self._check_flash_files(flash_file_list)

        #Force creating a new emulator configuration named x86
        proc = subprocess.Popen([self.path_to_android, 'create', 'avd', '-n', "x86", '-t', '1', '-f'],
                                stdin=subprocess.PIPE)
        proc.communicate(input="no")
        flash_return_code = FlashToolBase._flash(self, timeout)

        return flash_return_code
