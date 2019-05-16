#!/usr/bin/env python
# -*- coding: utf-8 -*-

#-----------------------------------------------------------------------------
# :copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
# The source code contained or described here in and all documents related
# to the source code ("Material") are owned by Intel Corporation or its
# suppliers or licensors. Title to the Material remains with Intel Corporation
# or its suppliers and licensors. The Material contains trade secrets and
# proprietary and confidential information of Intel or its suppliers and
# licensors.
#
# The Material is protected by worldwide copyright and trade secret laws and
# treaty provisions. No part of the Material may be used, copied, reproduced,
# modified, published, uploaded, posted, transmitted, distributed, or disclosed
# in any way without Intel's prior express written permission.
#
# No license under any patent, copyright, trade secret or other intellectual
# property right is granted to or conferred upon you by disclosure or delivery
# of the Materials, either expressly, by implication, inducement, estoppel or
# otherwise. Any license under such intellectual property rights must be express
# and approved by Intel in writing.
#
# :organization: INTEL MCG PSI
# :summary: This module implements Sphinx Auto-generator of Documentation
# :since: 2/7/14
# :author: lbavois
#-----------------------------------------------------------------------------

"""

------

Sphinx doc goes here ...
"""

import os
import platform
import shlex

from lxml import etree

from acs_test_scripts.Device.Module.SSG.Flash.FlashManager.Tool.FlashToolBase import FlashToolBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


class AndroidOSFlashTool(FlashToolBase):

    """
    This class is the specific class used for AndroidFlash flash tool
    """
    DEFAULT_OS_IMG = ["boot.img", "recovery.img", "system.img", "android-info.txt"]

    def __init__(self, logger=None):
        """
        Constructor

        :type logger: object
        :param logger: logger name
        """
        FlashToolBase.__init__(self, self.__class__.__name__, logger)

        # Specific to Flash Tool
        self._logger = logger if logger else LOGGER_TEST_SCRIPT
        self._flash_tool_state = "UNUSED"
        self._serial = None
        self._pos_serial = None
        self._soc_serial = None
        self._switch_device_state_fct = None
        self._adb_over_ethernet = False
        self._flash_file = None

    def _check_flash_tool_availability(self):
        """
        Assume fastboot command is available on all Android phone
        """
        cmd_name = "fastboot"

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

                if file_extension.lower() == ".xml":
                    if os.path.isfile(flash_file):
                        # Parse .xml file
                        flash_file_folder = os.path.join(os.path.dirname(flash_file))

                        try:
                            flash_xml_doc = etree.parse(flash_file)
                            file_node_list = flash_xml_doc.xpath('//flashfile/code_group[@name]/file[@TYPE]/name')

                            if file_node_list:
                                for file_node in file_node_list:
                                    if file_node.text:
                                        img_flash_file = os.path.join(flash_file_folder, file_node.text)
                                        if os.path.isfile(img_flash_file):
                                            android_flash_file_list.append(file_node.text)
                        except Exception as ex:
                            err_msg = "AndroidOSFlashTool: Error in parsing .xml file %s , exception= %s" % str(
                                flash_file), str(ex)
                            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)

                        # Add all .txt file associated to .xml file in same folder in flash file list
                        for file in os.listdir(flash_file_folder):
                            if file.endswith(".txt"):
                                android_flash_file_list.append(file)

                elif file_extension.lower() == ".zip":
                    # Do we have .img files to launch fastboot -flashall in folder where .zip has been extracted
                    flash_file_folder = filename
                    for file in os.listdir(flash_file_folder):
                        if file.endswith(".img") or file.endswith(".txt"):
                            android_flash_file_list.append(file)
                else:
                    err_msg = "%s : flash file format is not suitable (should be .zip or .xml) for file %s" % (
                        self.__class__.__name__, str(flash_file))
                    self._logger.warning(err_msg)

        if not android_flash_file_list:
            err_msg = "%s: No files to flash found" % self.__class__.__name__
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
        else:
            # check boot.img,recovery.img, system.img file presence
            all_img_presence = set(self.DEFAULT_OS_IMG).issubset(android_flash_file_list)
            if not all_img_presence:
                missing_ff = set(self.DEFAULT_OS_IMG).difference(android_flash_file_list)
                err_msg = "%s: missing a flash file : %s" % (self.__class__.__name__, " ,".join(missing_ff))
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
            else:
                # all needed files are present
                # Return folder name containing all the files
                # Assumption: all files are in the same folder
                folder_flash_path = flash_file_folder

        return folder_flash_path

    def _retrieve_device_info(self, device_flash_info):
        """
        Retrieve necessary information from the device for flash tool execution

        :type device_flash_info: FlashDeviceInfo
        :param device_flash_info: contain all device information needed by the flash tool
        """
        # Retrieve soc_serial_number and serial number (specific to PFT flash tool)
        serial = device_flash_info.device_pos_serial_number
        self._serial = str(serial).strip()

        # Retrieve adb connection type
        self._adb_over_ethernet = device_flash_info.adb_over_ethernet

        soc_serial = device_flash_info.device_soc_serial_number
        self._soc_serial = str(soc_serial).strip()

        pos_serial = device_flash_info.device_pos_serial_number
        self._pos_serial = str(pos_serial).strip()

        # Retrieve callback function in device class to make change its state
        self._switch_device_state_fct = device_flash_info.change_device_state_fct

    def _get_cmd(self):
        """
        Build the flash command
        """
        if self._adb_over_ethernet:
            cmd = "%s -t %s flashall" % (self._flash_tool_cmd, self._pos_serial)
        else:
            cmd = "%s -s %s flashall" % (self._flash_tool_cmd, self._pos_serial)

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
        # Set environment variable ANDROID_PRODUCT_OUT to be able to support fastboot flashall command
        os.environ["ANDROID_PRODUCT_OUT"] = self._flash_file

        return_code, error_message = FlashToolBase._run(self)

        # Delete environment variable ANDROID_PRODUCT_OUT cause no more used
        del os.environ["ANDROID_PRODUCT_OUT"]

        # Valid return_code from flashing command
        if return_code is not None:
            if return_code != 0:
                self.exec_result = False
                self.error_message = "%s: Unexpected result: %d" % (self.__class__.__name__, return_code)
            else:
                self.exec_result = True
                self._logger.info("%s: Flash succeeds" % (self.__class__.__name__))
                self._flash_tool_state = "FLASHED"

        else:
            self.exec_result = False
            self.error_message = error_message

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

        # Check flash files availability and format
        self._flash_file = self._check_flash_files(flash_file_list)

        self._retrieve_device_info(device_info)

        # Change the device state => Trig the flash mode
        if self._switch_device_state_fct("FLASH_SWITCH_OFF") == Global.SUCCESS \
                and self._switch_device_state_fct("POS_MODE_ON") == Global.SUCCESS:
            self._flash_tool_state = "FLASH_MODE"
            flash_return_code = FlashToolBase._flash(self, timeout)
            if flash_return_code == Global.SUCCESS:
                flash_return_code, _ = self._switch_device_state_fct("WAIT_BOOT")
        else:
            self._logger.error("%s: Can not switch in fastboot mode on the device" % self.__class__.__name__)
            flash_return_code = Global.FAILURE

        return flash_return_code
