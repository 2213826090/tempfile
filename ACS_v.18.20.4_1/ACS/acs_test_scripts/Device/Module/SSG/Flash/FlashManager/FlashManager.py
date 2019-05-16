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
:summary: This file implements the Flash Manager for a device (flash bios+os)
:since: 09/07/2013
:author: lbavois
"""

import time
import os
import zipfile
import tarfile

from lxml import etree

from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from acs_test_scripts.Device.Module.SSG.Flash.FlashManager.Tool.AndroidOSFlashTool import AndroidOSFlashTool
from acs_test_scripts.Device.Module.SSG.Flash.FlashManager.Tool.FastbootTool import FastbootTool
from acs_test_scripts.Device.Module.SSG.Flash.FlashManager.Tool.PFT import PFT
from acs_test_scripts.Device.Module.SSG.Flash.FlashManager.Tool.CFlasher import CFlasher
from acs_test_scripts.Device.Module.SSG.Flash.FlashManager.Tool.AndroidGminOSFlashTool import AndroidGminOSFlashTool
from acs_test_scripts.Device.Module.SSG.Flash.FlashManager.Tool.AndroidEmulatorFlashTool import AndroidEmulatorFlashTool
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.LockManager.LockManagerClient import LockEquipementManagerClient
from ErrorHandling.AcsConfigException import AcsConfigException


class FlashManager(object):

    """
    This class is the ACS flash module in charge of flash management
    It allow to launch all flash methods (bios / os flashing) needed to flash a given device
    Using flash tools supported by ACS (for instance: DEDIPROG, PFT, FASTBOOT TOOL, ...)
    """
    # Define the list of flash tool supported in ACS
    BIOS_FLASH_TOOL_LIST = {}

    OS_FLASH_TOOL_LIST = {"PFT": PFT,
                          "CFlasher": CFlasher,
                          "FastbootTool": FastbootTool,
                          "AndroidFlash": AndroidOSFlashTool,
                          "AndroidGminFlash": AndroidGminOSFlashTool,
                          "AndroidEmulator": AndroidEmulatorFlashTool}

    def __init__(self, os_flash_tool, logger, bios_flash_request=False, bios_flash_tool="", voltage="1.8V"):
        """
        Constructor

        :type os_flash_tool: str
        :param os_flash_tool: Name of the flash tool used for os flashing
        :type logger: object
        :param logger: Name of the logger used for flash module
        :type bios_flash_request: bool
        :param bios_flash_request: request bios flashing (=True) , no bioas flashing (=False)
        :type bios_flash_request: str
        :type bios_flash_tool: bios flashing tool name
        """
        self._os_flash_tool = str(os_flash_tool)
        self._logger = logger if logger else LOGGER_TEST_SCRIPT
        self._allow_flashing_bios = bios_flash_request
        self._bios_flash_tool = str(bios_flash_tool)
        self._flash_files = []
        self._biosFlashObject = None
        self._osFlashObject = None
        self._voltage = str(voltage)
        # Check flash tool availability when creating FlashManager instance
        self._check_flash_tools()

    def _check_flash_tools(self):
        """
        Check the different flash tools availability
        """
        if self._allow_flashing_bios:
            # We want to flash device bios
            if self._bios_flash_tool in self.BIOS_FLASH_TOOL_LIST.iterkeys():
                # bios flash tool is supported by ACS FlashManager
                # Create bios_flash_tool instance
                # Set a correct voltage value for dediprog

                # For the moment, DEDIPROG is the only available Bios Flash Tool, voltage value is mandatory
                self._biosFlashObject = self.BIOS_FLASH_TOOL_LIST[self._bios_flash_tool](self._logger, self._voltage)
            else:
                raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG,
                                         "FlashManager: BIOS flash (%s) "
                                         "tool not supported by ACS" % self._bios_flash_tool)

        # if BIOS flash tool present when bios flashing requested or no bios flashing requested
        # => Test OS flash tool presence
        if self._os_flash_tool in self.OS_FLASH_TOOL_LIST.iterkeys():
            # os flash tool is supported by ACS FlashManager
            # Create os_flash_tool instance
            self._osFlashObject = self.OS_FLASH_TOOL_LIST[self._os_flash_tool](self._logger)
        else:
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG,
                                     "FlashManager: OS flash tool (%s) not supported by ACS" % self._os_flash_tool)

    def _check_flash_files(self, file_list):
        """
        Check that given flash files exist

        :type file_list: str
        :param file_list: path to flash files, separated by ';'
        :rtype: list
        :return: flash file list to use with flash tools
        """
        flash_files_list = []
        for flash_file in str(file_list).split(";"):

            # Check if flash file exist
            if not isinstance(flash_file, str):
                err_msg = "Invalid flash file, should be a string!"
                self._logger.error(err_msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
            if flash_file in [None, "None", ""]:
                err_msg = "Flash file not defined!"
                self._logger.error(err_msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
            else:
                # Normalize path name, also for relative paths
                flash_file = os.path.abspath(str(flash_file).strip())

                # Extract final flash file if needed (.zip file case)
                try:
                    final_flash_file = self._extract_flash_file(flash_file)
                except IOError as e:
                    raise AcsConfigException(AcsConfigException.FILE_NOT_FOUND,
                                             "{0} is not valid flash file : {1}".format(flash_file,
                                                                                        e.strerror))
                flash_files_list.append(final_flash_file)

        # update FlashManager file list flashed
        if flash_files_list:
            # We need at least 1 file to flash the device
            return flash_files_list
        else:
            err_msg = ("FlashManager: no flash file provided to flash BIOS and OS,"
                       " please check your flash files list {0}".format(" ".join(file_list)))
            self._logger.error(err_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)

    def _extract_flash_file(self, flash_file):
        """
        Extract flash file path to use with flash tools

        :type flash_file: str
        :param flash_file: path to flash file given by user
        :rtype: str
        :return: flash file path to use with flash tools
        """

        # Check we have the correct flash file entry point for ACS
        filename, file_extension = os.path.splitext(flash_file)

        # First unzip flash file if needed
        if zipfile.is_zipfile(flash_file):
            try:
                zip_file = zipfile.ZipFile(flash_file, "r")
                self._logger.info("FlashManager: unzip flash file (%s)" % flash_file)
                zip_file.extractall(filename)
                zip_file.close()
                flash_file_to_use = flash_file
            except Exception as ex:
                err_msg = "FlashManager: Flash input file %s, unzip issue, error : %s" % (flash_file, str(ex))
                self._logger.error(err_msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
        elif tarfile.is_tarfile(flash_file):
            try:
                tar_file = tarfile.open(flash_file, "r")
                self._logger.info("FlashManager: untar flash file (%s)" % flash_file)
                tar_file.extractall(filename)
                tar_file.close()
                flash_file_to_use = flash_file
            except Exception as ex:
                err_msg = "FlashManager: Flash input file %s, untar issue, error : %s" % (flash_file, str(ex))
                self._logger.error(err_msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
        elif file_extension.lower() == ".xml" or file_extension.lower() == ".bin":
            if os.path.isfile(flash_file):
                self._logger.info("FlashManager: Flash file %s exists" % flash_file)
                flash_file_to_use = flash_file
            else:
                err_msg = "FlashManager: Flash file %s not found!" % flash_file
                self._logger.error(err_msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
        else:
            err_msg = "FlashManager: Flash file %s format is not suitable (.xml, .bin or .zip file should be used)" \
                      % str(flash_file)
            self._logger.error(err_msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, err_msg)
        return flash_file_to_use

    def _retrieve_build_info_from_flash_files(self, flash_files):
        """
        Retrieve build information from flash files.

        :type flash_files: list
        :param flash_files: path to flash files
        :rtype: dict
        :return: Dictionary containing build properties
        """
        # Initialize variables
        build_info_dict = {}

        for flash_file in self._flash_files:
            filename, file_extension = os.path.splitext(flash_file)

            # First unzip flash file if needed
            if file_extension.lower() == ".zip":
                if not os.path.isdir(filename):
                    # Unzip has not been done do it
                    try:
                        zip_file = zipfile.ZipFile(flash_file, "r")
                        self._logger.info("FlashManager: unzip flash file (%s)" % flash_file)
                        zip_file.extractall(filename)
                        zip_file.close()
                    except Exception as ex:
                        err_msg = "FlashManager: Flash input file %s, unzip issue, error : %s" % (
                            str(flash_file), str(ex))
                        self._logger.error(err_msg)

                # Unzip has already been done, check if there is flash.xml file present to retrieve build information
                xml_flash_file = os.path.join(filename, "flash.xml")
                if os.path.isfile(xml_flash_file):
                    # Update flash_file to retrieve build information
                    flash_file = xml_flash_file
                    filename, file_extension = os.path.splitext(flash_file)

            if file_extension.lower() == ".xml":
                # Analyze flash file first
                try:
                    # Retrieve build archive to extract information from file build.prop
                    flash_xml_doc = etree.parse(flash_file)
                except (AcsConfigException, etree.XMLSyntaxError) as ex_msg:
                    error_msg = "FlashManager: Unable to retrieve all build informations from flash files (exception = " + \
                                str(ex_msg) + ")"
                    self._logger.warning(error_msg)
                    continue

                # Extract build properties
                info_key = flash_xml_doc.xpath("//buildproperties/property")
                for prop in info_key:
                    property_name = prop.get("name")
                    if property_name not in build_info_dict:
                        build_info_dict.update({property_name: prop.get("value")})
                        if "ro.build.description" in property_name:
                            build_info_dict["software_release"] = prop.get("value")
                        elif "ro.product.name" in property_name:
                            build_info_dict["hardware_model"] = prop.get("value")
                        elif "gsm.version.baseband" in property_name:
                            build_info_dict["baseband_version"] = prop.get("value")
                        elif "sys.ifwi.version" in property_name:
                            build_info_dict["firmware_version"] = prop.get("value")

        return build_info_dict

    def check_flash_file(self, flash_files):
        """
        Check and get flash file properties

        :type flash_files: list
        :param flash_files: contain flash file absolute path list (split with a ';' character)
        """
        self._flash_files = self._check_flash_files(flash_files)
        build_info = self._retrieve_build_info_from_flash_files(self._flash_files)
        return build_info

    def flash_single_device(self, device_info, timeout):
        """
        Flash BIOS and OS to device memories.


        :type device_info: FlashDeviceInfo
        :param device_info: device parameters needed by flash tools to flash the board
        :type timeout: int
        :param timeout: Flash execution timeout in s
        :rtype: int
        :return: result of the flash procedure (Global.SUCCESS, Global.FAILURE, Global.BLOCKED)
        :rtype: dict
        :return: Dictionary containing build properties
        """

        flash_operation = Global.SUCCESS

        # Flash bios/firmware first
        if self._biosFlashObject is not None:
            # Flash the BIOS
            t0 = time.time()
            # Run the flasher
            flash_operation = self._biosFlashObject.flash(self._flash_files, device_info, timeout)
            t1 = time.time()
            # Decrease global flash time, by the time spent to flash the bios
            timeout -= (t1 - t0)

            if flash_operation != Global.SUCCESS:
                # Stop the global flash procedure as bios/firmware has failed
                # log an error
                self._logger.error("FlashManager: bios/firmware flashing has failed, abort flash procedure")

        # Flash OS only if boot flash procedure is successful or not required
        if flash_operation == Global.SUCCESS and self._osFlashObject is not None:
            # Flash the OS
            t0 = time.time()
            # Run the flasher
            flash_operation = self._osFlashObject.flash(self._flash_files, device_info, timeout)
            t1 = time.time()
            # Decrease global flash time, by the time spent to flash the bios
            timeout -= (t1 - t0)
        return flash_operation

    def flash(self, device_info, timeout, multiple_devices=False):
        result = Global.FAILURE
        if not multiple_devices:
            result = self.flash_single_device(device_info, timeout)
        else:
            flash_eq_client = LockEquipementManagerClient(device_info.lock_manager_ipAdress,
                                                          device_info.lock_manager_port,
                                                          self._logger)
            result = flash_eq_client.exec_function_on_resource('FLASH', timeout, self.flash_single_device,
                                                               (device_info, timeout))
        return result
