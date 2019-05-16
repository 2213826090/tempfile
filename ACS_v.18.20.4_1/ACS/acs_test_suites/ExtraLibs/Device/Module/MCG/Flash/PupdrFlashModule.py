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
except (NameError, ImportError, AttributeError):
    from Device.Module.Flash.IFlashModule import IFlashModule

from Device.Module.DeviceModuleBase import DeviceModuleBase
from UtilitiesFWK.ExecScriptCtx import init_ctx
from UtilitiesFWK.Utilities import Global
from UtilitiesFWK.AttributeDict import AttributeDict
import Lib.pupdr.pupdr_lib_loader as pupdr
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import AcsConstants
import xml.etree.ElementTree as ET
import os
import re
import json

# Regex to find the IFWI Version if possible
REGEX_IFWI_VERSION = re.compile(r'([A-Z0-9]{2}\.[A-Z0-9]{2})')


class PupdrFlashModule(IFlashModule, DeviceModuleBase):
    """
    PUPDR Flash module implementation

    """

    def __init__(self):
        """
        Module Constructor

        """
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

    def extract_flash_file_info(self):
        """
        Extracts Flash info from different sources::

            * PUPDR library
            * PhoneFlashTool build info generated JSON file

        """
        properties = pupdr.PL.Pupdr().Flash.getBuildProperties(self._flash_files[0], pft_info_file=False, isExit=False)
        if properties != {}:
            self._flash_info = properties
        else:
            self.logger.error("Cannot update flash_build_properties !")

        # preferring "fastboot" file against "blankphone"
        flash_file = self._flash_files[1] if len(self._flash_files) == 2 else self._flash_files[0]
        if flash_file.endswith(".json"):
            if pupdr.PL.Pupdr().Misc.isExpectedJsonFormat(flash_file, ["build_target","build_variant","board_type","url_buildinfo"],top_keys=["provisioning_properties"]):
                with open(flash_file) as f:
                    provisioning_json_data = json.load(f)["provisioning_properties"]
                branch = pupdr.PL.Pupdr().Download.getBranchFromTag(provisioning_json_data["url_buildinfo"])
                builder = pupdr.PL.Pupdr().Download.getBuilderFromTag(provisioning_json_data["url_buildinfo"])
                number = pupdr.PL.Pupdr().Download.getNumberFromTag(provisioning_json_data["url_buildinfo"])
                if any(not element for element in [branch, builder, number]):
                    tag = provisioning_json_data["url_buildinfo"]
                else:
                    tag = "{0}-{1}-{2}".format(branch, builder, number)
                self._flash_info["SwRelease"] = "{}/{}-{}-{}".format(provisioning_json_data["board_type"],
                                                                     provisioning_json_data["build_target"],
                                                                     provisioning_json_data["build_variant"],
                                                                     tag)
                self._flash_info["BranchBuildTypeNumber"] = tag
                return
            try:
                with open(flash_file) as f:
                    props = json.load(f)

                build_props = props.get("buildProperties", {})
                components_props = props.get("components", {})
                # To be sure we get all expected properties no matter what the context

                board_type = props.get('board')
                if board_type:
                    ifwi_filename = components_props.get('ifwi_{0}_file'.format(board_type), "").split('/')[-1]
                    ifwi_version = REGEX_IFWI_VERSION.findall(ifwi_filename)
                    ifwi_version = ifwi_version[0] if ifwi_version else AcsConstants.NOT_AVAILABLE
                    self._flash_info["FwVersion"] = ifwi_version

                self._flash_info["SwRelease"] = build_props.get("ro.build.description", AcsConstants.NOT_AVAILABLE)
                self._flash_info["BranchBuildTypeNumber"] = build_props.get("ro.build.version.incremental",
                                                                            AcsConstants.NOT_AVAILABLE)
            except (ValueError, IOError, OSError, AttributeError, UnicodeError) as error:
                self.logger.error("Cannot get flashing info "
                                  "from input json file ({0})!"
                                  "\nError Details:\n\jn{1}".format(self._flash_files[0], error))

    def init(self, flash_input):
        """
        Module Initialization

        :param flash_input: The Flash input(s)
        :type flash_input: str

        .. note:: Flash input(s) are either the blankphone;fastboot images filenames or the fastboot only

            When passing both blankphone and fastboot, always pass the blankphone first separated with semi-colon

            **/path/to/blankphone.json;/path/to/fastboot.json**

        :return: The initialization verdict
        :rtype: int

        """

        verdict = Global.SUCCESS

        def update_global_config():
            """
            For now, pupdr_common is based on exec script
            Update globals with fake exec script in order to make this code retro compliant
            """
            self._globals["EXEC_UC"] = AttributeDict()
            fake_tc_params = AttributeDict()
            fake_tc_params._attrs_override = {"TcExpectedResult": ""}
            fake_tc_params.get_name = lambda: "FLASH_UC"
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
                           for flash_file_type in ("blankphone", "fastboot", "ota", "firmware", "provisioning")):
                    # check directory name in case flash.xml is passed
                    if not any(flash_file_type in os.path.basename(os.path.dirname(flash_file))
                               for flash_file_type in ("blankphone", "fastboot", "ota")):
                        flash_file_check_passed = False
                        self.logger.error("Flash file {0} is neither of blankphone, "
                                          "fastboot nor ota type".format(flash_file))
            if not flash_file_check_passed:
                raise AcsConfigException("Flash files are not present or of bad type")
            self.extract_flash_file_info()

        # $ Deactivate MOS init
        pupdr.NOMOS = True
        pupdr.setup()

        self.update_device_flash_properties()

        return verdict

    @property
    def flash_properties(self):
        """
        Property holding Flash properties

        :return: Flash properties
        :rtype: dict

        """
        return self._flash_info

    @property
    def device_properties(self):
        """
        Property holding Device properties

        :return: Device properties
        :rtype: dict

        """
        return self._device_info

    def flash(self, timeout):
        """
        Flashing behavior implementation

        :param timeout: A timeout after which the Flashing process must consider as a failure
        :type timeout: float or int

        :return: The Flashing verdict
        :rtype: int

        """
        if len(self._flash_files) == 1:
            if "blankphone" in self._flash_files[0]:
                self.logger.info("Only blankphone provided")
                raise AcsConfigException("Only blankphone provided")
        self.update_device_flash_properties()
        local_verdict, output = pupdr.PL.Pupdr().Flash_scripts.SystemFlashTestCase(self._flash_files)
        if local_verdict:
            verdict = pupdr.SUCCESS
            pupdr.append_OUTPUT(output["output_log"], split=" # ")
            pupdr.finalStep()
            self.update_device_info_with_build_info(self.device_properties)
        else:
            verdict = pupdr.FAILURE
            pupdr.failure(output["output_log"])

        return verdict

    def update_device_flash_properties(self):
        """
        Updates the Device `flash_properties` property with build info
        Those information are gathered from PhoneFlashTool (--build-info-output)

        Properties mapped::

            * sw_release (ro.build.description)
            * branch_buildtype_number (ro.build.version.incremental)

        """

        # Mapping the data value as expected into AWR
        self.device.flash_device_properties = self._flash_info.copy()

    def update_device_info_with_build_info(self, dict_info):
        """
        Update device info with build properties from flash files

        :param dict_info: Device information
        :type dict_info: dict

        """
        if dict_info:
            self.logger.info("Update device info with build properties from flash files")
            # Update properties which can be updated
            self.device.device_properties.sw_release = dict_info.get("software_release", AcsConstants.NOT_AVAILABLE)
            self.device.device_properties.model_number = dict_info.get("hardware_model", AcsConstants.NOT_AVAILABLE)
            self.device.device_properties.baseband_version = dict_info.get("baseband_version",
                                                                           AcsConstants.NOT_AVAILABLE)
            self.device.device_properties.fw_version = dict_info.get("firmware_version", AcsConstants.NOT_AVAILABLE)

    def get_build_name(self):
        """
        Get the build name (from json build file)

        :return: The build name
        :rtype: str

        """
        return pupdr.BUILD_NAME
