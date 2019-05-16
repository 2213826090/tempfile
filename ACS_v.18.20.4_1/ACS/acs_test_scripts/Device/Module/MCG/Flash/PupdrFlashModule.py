#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=missing-docstring, invalid-name, unused-argument
"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
@summary: Pupdr flashing module
@since: 5/16/2014
@author: sfusilie
"""
try:
    from acs_test_scripts.Device.Module.MCG.Flash.IFlashModule import IFlashModule
except:
    from Device.Module.Flash.IFlashModule import IFlashModule

from Device.Module.DeviceModuleBase import DeviceModuleBase
from UtilitiesFWK.ExecScriptCtx import init_ctx
from UtilitiesFWK.Utilities import Global
from UtilitiesFWK.AttributeDict import AttributeDict
try:
    import Lib.pupdr.pupdr_lib_loader as pupdr
except:
    import Lib.pupdr.pupdr_common as pupdr
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import AcsConstants
import xml.etree.ElementTree as ET
import os
import re

REGEX_TAG_FASTBOOT = ".*-([^-]*-[a-z]*-[0-9]*)\.zip"

class PupdrFlashModule(IFlashModule, DeviceModuleBase):

    def __init__(self):
        super(PupdrFlashModule, self).__init__()
        self._globals = globals()
        self._flash_input = None
        self._flash_files = None
        self._device_info = {}
        self._flash_info = {}
        self._flash_bios = False
        self._check_tag = False
        self._check_ifwi = True
        self._modem_flash = True
        self._files_handler = pupdr.FlashFileHandler()

    def extract_flash_file_info(self):
        properties = pupdr.FLASH_MANAGER.get_build_properties(self._flash_files[0], pft_info_file=False, exit=False)
        if properties != {}:
            self._flash_info = properties
        else:
            self.logger.error("Cannot update flash_build_properties !")

    def init(self, flash_input):
        verdict = Global.SUCCESS

        def update_global_config():
            """
            For now, pupdr_common is based on exec script
            Update globals with fake exec script in order to make this code retro compliant
            """
            self._globals["EXEC_UC"] = AttributeDict()
            fake_tc_params = AttributeDict()
            fake_tc_params._attrs_override = {"TcExpectedResult": ""}

            def fake_get_name():
                return "FLASH_UC"

            fake_tc_params.get_name = fake_get_name
            self._globals["EXEC_UC"]._tc_parameters = fake_tc_params

        init_ctx(self._globals, self.logger, self.global_conf)
        # in order to hack pupdr common
        update_global_config()
        pupdr.init(self._globals)

        if not flash_input:
            raise AcsConfigException("No flash files provided")
        else:
            self._flash_input = flash_input

        self._modem_flash = self.configuration.get_value("MODEM_FLASH", True, "str_to_bool")
        if self._modem_flash:
            pupdr.TEST = "PUPDR FLASH MODULE - "
        else:
            pupdr.TEST = "PUPDR FLASH MODULE (no modem) - "

        self.logger.info("Flash({0})".format(flash_input))
        # pupdr.FLASH_FILES = flash_input
        self._flash_files = flash_input.split(";")
        if not self._flash_files:
            raise AcsConfigException("Nothing to flash")
        else:
            # check all entries are present on disk and of proper format
            flash_file_check_passed = True
            for flash_file in self._flash_files:
                if not os.path.isfile(flash_file):
                    flash_file_check_passed = False
                    self.logger.error("Flash file {0} not present on disk".format(flash_file))
                if not any(flash_file_type in os.path.basename(flash_file)
                           for flash_file_type in ("blankphone", "fastboot", "ota", "flashfiles")):
                    # check directory name in case flash.xml is passed
                    if not any(flash_file_type in os.path.basename(os.path.dirname(flash_file))
                               for flash_file_type in ("blankphone", "fastboot", "ota", "flashfiles")):
                        flash_file_check_passed = False
                        self.logger.error("Flash file {0} is neither of blankphone, fastboot nor ota type".format(flash_file))
            if not flash_file_check_passed:
                raise AcsConfigException("Flash files are not present or of bad type")
            self.extract_flash_file_info()

        #$ Deactivate MOS init
        pupdr.NOMOS = True
        pupdr.setup()

        self.update_device_info_with_build_info(self.flash_properties)
        return verdict

    @property
    def flash_properties(self):
        return self._flash_info

    @property
    def device_properties(self):
        return self._device_info

    def flash(self, timeout):
        # >$ If only fastboot file provided
        flash_key = "force;blank"
        self._flash_info["flash_list"] = self._files_handler.flash_file_list_2_dictionary_list(self._flash_files)
        # compute tag
        for f in self._flash_info["flash_list"]:
            local_flash_file = pupdr.FlashFiles(f)
            tag = re.search(REGEX_TAG_FASTBOOT, local_flash_file.get_localFlashFile())
            if tag:
                self._flash_info["tag"] = tag.group(1)
                self._check_tag = True
                break

        if len(self._flash_files) == 1:
            if "blankphone" in self._flash_files[0]:
                self.logger.info("Only blankphone provided")
                raise AcsConfigException("Only blankphone provided")
            else:
                # do not check ifwi if only fastboot file provided because ifwi is in blankphone
                if "fastboot" in self._flash_files[0]:
                    self._check_ifwi = False

        if not self._modem_flash:
            # > retrieve flash.xml file
            flash_file = pupdr.FLASH_MANAGER.get_zip_content_list(self._flash_info["fastboot"], "flash.xml")[0]

            # > Remove the modem flashing command from the xml
            flash_xml = ET.ElementTree()
            xml = flash_xml.parse(flash_file)
            for cmd in xml.findall("command"):
                if "modem" in cmd.find("string").text:
                    flash_xml.getroot().remove(cmd)
            flash_xml.write(flash_file)

            # > Rezip the fastboot package
            z = pupdr.unzip(self._flash_info["fastboot"], "a")
            z.write(flash_file, arcname="flash.xml")
            z and z.close()

        # >$ Run flashing
        verdict = pupdr.FLASH_MANAGER.flash(self._flash_info,
                                            flash_key,
                                            check_tag=self._check_tag,
                                            check_ifwi=self._check_ifwi,
                                            check_modem=False,
                                            check_img=True,
                                            check_ssn=False)
        if verdict == SUCCESS:
            pupdr.finalStep()
            self.update_device_info_with_build_info(self.device_properties)
        else:
            pupdr.failure(pupdr.OUTPUT)

        return verdict

    def update_device_info_with_build_info(self, dict_info):
        """
        Update device info with build properties from flash files
        """

        if dict_info:
            self.logger.info("Update device info with build properties from flash files")

            # Update properties which can be updated
            self.device.device_properties.sw_release_flashed = dict_info.get("software_release", AcsConstants.NOT_AVAILABLE)
            self.device.device_properties.model_number = dict_info.get("hardware_model", AcsConstants.NOT_AVAILABLE)
            self.device.device_properties.baseband_version = dict_info.get("baseband_version", AcsConstants.NOT_AVAILABLE)
            self.device.device_properties.fw_version = dict_info.get("firmware_version", AcsConstants.NOT_AVAILABLE)
