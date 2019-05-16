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
@summary: Pupdr Library - Flash scripts
@since: 07/01/2015
@author: travenex
"""

import os
import re
import json
import shutil
from .. import TestCaseModule
from .. import ConfigurationModule

class Flash(object):

    __instance = None
    __testCase = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def __init__(self):
        pass

    @classmethod
    def init(cls, globalConf=None):
        cls.__globalConf = globalConf
        cls.__testCase = TestCaseModule.TestCaseModule()
        cls.__configuration = ConfigurationModule.ConfigurationModule()

    def IncrementalFotaTestCase(self):
        TestCase = Flash.IncrementalFotaScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def FastbootForwardFlashingTestCase(self):
        TestCase = Flash.ForwardFlashingScript(self.__globalConf)
        TestCase(file_type="fastboot")
        return TestCase.verdict, TestCase.output_dict

    def OtaForwardFlashingTestCase(self):
        TestCase = Flash.ForwardFlashingScript(self.__globalConf)
        TestCase(file_type="ota")
        return TestCase.verdict, TestCase.output_dict

    def FastbootFlashingTestCase(self):
        TestCase = Flash.FlashingScript(self.__globalConf)
        TestCase(file_type="fastboot")
        return TestCase.verdict, TestCase.output_dict

    def UserFlashingWithoutAdbTestCase(self):
        TestCase = Flash.UserFlashingWithoutAdbScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def OtaFlashingTestCase(self):
        TestCase = Flash.FlashingScript(self.__globalConf)
        TestCase(file_type="ota")
        return TestCase.verdict, TestCase.output_dict

    def OtaSideloadTestCase(self):
        TestCase = Flash.OtaSideloadScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def OtaForwardSideloadTestCase(self):
        TestCase = Flash.OtaForwardSideloadScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def SystemFlashTestCase(self, flash_files, force=True, build_tag_override=""):
        TestCase = Flash.SystemFlashScript(self.__globalConf)
        TestCase(flash_files=flash_files, force=force, build_tag_override=build_tag_override)
        return TestCase.verdict, TestCase.output_dict

    def ReferenceFlashingTestCase(self):
        TestCase = Flash.ReferenceFlashingScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def ExternalBuildFlashingTestCase(self):
        TestCase = Flash.ExternalBuildFlashingScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def Dessert2DessertFlashingTestCase(self):
        TestCase = Flash.Dessert2DessertFlashingScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    def ButFlashFilesDownloadTestCase(self, provisioning_json_file):
        TestCase = Flash.ButFlashFilesDownloadScript(self.__globalConf)
        TestCase(provisioning_json_file=provisioning_json_file)
        return TestCase.verdict, TestCase.output_dict

    def CheckFlashedBuildTestCase(self):
        TestCase = Flash.CheckFlashedBuildScript(self.__globalConf)
        TestCase()
        return TestCase.verdict, TestCase.output_dict

    class SystemFlashScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "PUPDR FLASH - "

        def runAllSteps(self, flash_files=list(), force=True, build_tag_override=""):
            if "bxtp" in self.configuration.board:
                self.logger.printLog("INFO", "Disable init and final")
                self.enable_init = False
                self.enable_final = False
            if len(flash_files) == 1 and self.misc.isExpectedJsonFormat(
                    flash_files[0], ["build_target", "build_variant", "board_type",
                                     "url_buildinfo", "timeout","download_timeout"],
                    top_keys=["provisioning_properties"]):
                self.runStep(Flash.ProcessProvisioningJsonStep, parameter={"provisioning_json_file": flash_files[0]})
                self.runStep(Flash.DownloadStep)
            else:
                self.runStep(Flash.BuildButFromFlashFilesStep, parameter={"flash_files": flash_files})
            if not force:
                flash_key = "recover"
                self.enable_init = False
            else:
                flash_key = "force;recover"
            self.runStep(Flash.FlashStep, parameter={"build": self.download.build_but,"flash_key": flash_key})
            if self.configuration.branch == "mmsd":
                self.runStep(Flash.CheckFlashBuildStep, parameter={"build": self.download.build_but,"check_tag": False,
                                                                      "check_ifwi": True, "check_modem": False,
                                                                      "check_img": True, "check_ssn": False,
                                                                      "build_tag_override": build_tag_override})
            else:
                self.runStep(Flash.CheckFlashBuildStep, parameter={"build": self.download.build_but, "check_tag": True,
                                                                   "check_ifwi": True, "check_modem": False,
                                                                   "check_img": True, "check_ssn": False,
                                                                   "build_tag_override": build_tag_override})

    class ButFlashFilesDownloadScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "PUPDR DOWNLOAD - "
        def runAllSteps(self, provisioning_json_file=""):
            self.enable_init = False
            self.enable_final = False
            self.runStep(Flash.ProcessProvisioningJsonStep, parameter={"provisioning_json_file": provisioning_json_file})
            self.runStep(Flash.PhoneFlashToolPurgeStep)
            self.runStep(Flash.WorkdirCleaningStep)
            self.runStep(Flash.DownloadStep)

    class ForwardFlashingScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "FW FLASH - "
        def runAllSteps(self, file_type="fastboot"):
            if "bxtp" in self.configuration.board:
                self.logger.printLog("INFO", "Disable init and final")
                self.enable_init = False
                self.enable_final = False
            if not self.name:
                self.name = "{0} FW FLASH - ".format(file_type.upper())
            self.runStep(Flash.DownloadStep, parameter={"but_ota_files_needed":file_type=="ota","create_ref": True})
            self.runStep(Flash.CheckLocalFlashFilePresenceStep, parameter={"build": self.download.build_but,
                            "mandatory_files": "fastboot{0}{1}"
                            .format(";blankphone" if any(("pft_key" in element and element["pft_key"] =="blankphone") for element in self.configuration.download.INTERNAL_DOWNLOAD) else "",
                            ";ota" if file_type=="ota" else "")})
            if self.skipping:
                self.toggleVerdict(self.VALID)
                self.output_dict["valid_log"] = "no Full OTA file found"
            # Disabeling ifwi check for bxtp #
            if "bxtp" not in self.configuration.board:
                self.runStep(Flash.StoreIfwiVersionStep, impact_tc_verdict=False)
            self.runStep(Flash.CheckLocalFlashFilePresenceStep, parameter={"build": self.download.build_ref,
                            "mandatory_files": "fastboot{0}"
                            .format(";blankphone" if any(("pft_key" in element and element["pft_key"] =="blankphone") for element in self.configuration.download.INTERNAL_DOWNLOAD) else "")})
            self.runStep(Flash.FlashBuildIfNeededStep,
                            parameter={"build":self.download.build_ref, "flash_key":"force;recover"})
            self.runStep(Flash.FlashStep,
                            parameter={"build": self.download.build_but,
                                       "flash_key": "update" if file_type == "fastboot" else file_type})
            self.runStep(Flash.CheckFlashBuildStep, parameter={"build": self.download.build_but,"check_tag": True,
                                                                  "check_ifwi": True, "check_modem": False,
                                                                  "check_img": True, "check_ssn": False})
            if not self.verdict:
                self.runStep(Flash.LeaveFlashingSoftlyStep, parameter={"build": self.download.build_but},
                             run_on_failure=True,
                             impact_tc_verdict=False)

    class OtaSideloadScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "SIDELOAD - "
        def runAllSteps(self):
            self.runStep(Flash.DownloadStep, parameter={"but_ota_files_needed": True})
            self.runStep(Flash.CheckLocalFlashFilePresenceStep, parameter={"build": self.download.build_but,
                                                                           "mandatory_files": "ota"})
            if self.skipping:
                self.toggleVerdict(self.VALID)
                self.output_dict["valid_log"] = "no Full OTA file found"
            self.runStep(Flash.StoreIfwiVersionStep, impact_tc_verdict=False)
            self.runStep(Flash.FlashBuildIfNeededStep, parameter={"build":self.download.build_but,
                                                                  "flash_key":"force;recover"})
            self.runStep(Flash.SideloadStep, parameter={"build": self.download.build_but})
            self.runStep(Flash.CheckFlashBuildStep, parameter={"build": self.download.build_but,"check_tag": True,
                                                               "check_ifwi": True, "check_modem": False,
                                                               "check_img": True, "check_ssn": False})
            if not self.verdict:
                self.runStep(Flash.LeaveFlashingSoftlyStep, parameter={"build": self.download.build_but},
                             run_on_failure=True,
                             impact_tc_verdict=False)

    class OtaForwardSideloadScript(TestCaseModule.TestCaseModule.TC_structure):
         def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "FORWARD SIDELOAD - "
         def runAllSteps(self):
            self.runStep(Flash.DownloadStep, parameter={"but_ota_files_needed": True,"create_ref": True})
            self.runStep(Flash.CheckLocalFlashFilePresenceStep, parameter={"build": self.download.build_but,
                                                                           "mandatory_files": "ota"})
            if self.skipping:
                self.toggleVerdict(self.VALID)
                self.output_dict["valid_log"] = "no Full OTA file found"
            self.runStep(Flash.StoreIfwiVersionStep, impact_tc_verdict=False)
            self.runStep(Flash.FlashBuildIfNeededStep, parameter={"build":self.download.build_ref,
                                                                  "flash_key":"force;recover"})
            self.runStep(Flash.SideloadStep, parameter={"build": self.download.build_but})
            self.runStep(Flash.CheckFlashBuildStep, parameter={"build": self.download.build_but,"check_tag": True,
                                                               "check_ifwi": True, "check_modem": False,
                                                               "check_img": True, "check_ssn": False})
            if not self.verdict:
                self.runStep(Flash.LeaveFlashingSoftlyStep, parameter={"build": self.download.build_but},
                             run_on_failure=True,
                             impact_tc_verdict=False)


    class CheckFlashedBuildScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "CHECK FLASHED BUILD - "
        def runAllSteps(self):
            self.runStep(Flash.DownloadStep)
            self.runStep(Flash.CheckFlashBuildStep, parameter={"build": self.download.build_but,"check_tag": True,
                                                                  "check_ifwi": True, "check_modem": False,
                                                                  "check_img": True, "check_ssn": False})

    class ReferenceFlashingScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "REF FLASH - "
        def runAllSteps(self):
            self.runStep(Flash.DownloadStep, parameter={"create_but":False, "create_ref": True})
            self.runStep(Flash.CheckLocalFlashFilePresenceStep, parameter={"build": self.download.build_ref,
                            "mandatory_files": "fastboot{0}"
                            .format(";blankphone" if any(("pft_key" in element and element["pft_key"] =="blankphone") for element in self.configuration.download.INTERNAL_DOWNLOAD) else "")})
            self.runStep(Flash.FlashStep,
                            parameter={"build": self.download.build_ref,
                                       "flash_key": "force;recover"})
            self.runStep(Flash.CheckFlashBuildStep, parameter={"build": self.download.build_ref,"check_tag": True,
                                                               "check_ifwi": True, "check_modem": False,
                                                               "check_img": True, "check_ssn": False})

    class FlashingScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "FLASH - "
        def runAllSteps(self, file_type="fastboot"):
            if "_" not in self.name:
                self.name = "{0} FLASH - ".format(file_type.upper())
            self.runStep(Flash.DownloadStep, parameter={"but_ota_files_needed":file_type=="ota"})
            self.runStep(Flash.CheckLocalFlashFilePresenceStep, parameter={"build": self.download.build_but,
                            "mandatory_files": "fastboot{0}{1}"
                            .format(";blankphone" if any(("pft_key" in element and element["pft_key"] =="blankphone") for element in self.configuration.download.INTERNAL_DOWNLOAD) else "",
                            ";ota" if file_type=="ota" else "")})
            if self.skipping:
                self.toggleVerdict(self.VALID)
                self.output_dict["valid_log"] = "no Full OTA file found"
            self.runStep(Flash.FlashBuildIfNeededStep,
                            parameter={"build":self.download.build_but, "flash_key":"force;recover"})
            self.runStep(Flash.FlashStep,
                            parameter={"build": self.download.build_but,
                                       "flash_key": "update" if file_type == "fastboot" else file_type})
            self.runStep(Flash.CheckFlashBuildStep, parameter={"build": self.download.build_but,"check_tag": True,
                                                               "check_ifwi": True, "check_modem": False,
                                                               "check_img": True, "check_ssn": False})
            if not self.verdict:
                self.runStep(Flash.LeaveFlashingSoftlyStep, parameter={"build": self.download.build_but},
                             run_on_failure=True,
                             impact_tc_verdict=False)

    class UserFlashingWithoutAdbScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "USER FLASHING - "
        def runAllSteps(self):
            self.enable_init = False
            self.enable_final = False
            self.runStep(Flash.DownloadStep)
            self.runStep(Flash.FlashStep,
                         parameter={"build": self.download.build_but, "flash_key": "force;recover", "adb": False})

    class ExternalBuildFlashingScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "EXTERNAL FLASHING - "
        def runAllSteps(self):
            self.runStep(Flash.DownloadStep, parameter={"create_external":True})
            self.runStep(Flash.CheckLocalFlashFilePresenceStep, parameter={"build": self.download.build_external,
                            "mandatory_files": "fastboot{0}"
                            .format(";blankphone" if any(("pft_key" in element and element["pft_key"] =="blankphone") for element in self.configuration.download.INTERNAL_DOWNLOAD)
                                                                                   else "")})
            self.runStep(Flash.FlashStep,
                            parameter={"build": self.download.build_external, "flash_key": "force;recover"})
            ifwi_check = True
            if any(element in self.configuration.board for element in ["byt_"]):
                ifwi_check = False
            self.runStep(Flash.CheckFlashBuildStep, parameter={"build": self.download.build_external,
                                                                  "check_tag": True, "check_ssn": False,
                                                                  "check_ifwi": ifwi_check, "check_modem": False,
                                                                  "check_img": True})
            if not self.verdict:
                self.runStep(Flash.LeaveFlashingSoftlyStep, parameter={"build": self.download.build_but},
                             run_on_failure=True,
                             impact_tc_verdict=False)

    class Dessert2DessertFlashingScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "D2D FLASHING - "
        def runAllSteps(self):
            self.runStep(Flash.DownloadStep, parameter={"create_d2d":True, "but_ota_files_needed": True})
            self.runStep(Flash.CheckLocalFlashFilePresenceStep, parameter={"build": self.download.build_but,
                "mandatory_files": "fastboot{0};ota"
                .format(";blankphone" if any(("pft_key" in element and element["pft_key"] =="blankphone") for element in self.configuration.download.INTERNAL_DOWNLOAD) else "")})
            self.runStep(Flash.CheckD2DLocalFlashFilePresenceStep, parameter={"build": self.download.build_d2d})
            if self.skipping:
                self.toggleVerdict(self.VALID)
                self.output_dict["valid_log"] = "no source build"
            self.runStep(Flash.StoreIfwiVersionStep, impact_tc_verdict=False)
            self.runStep(Flash.FlashBuildIfNeededStep, parameter={"build": self.download.build_d2d["source"]})
            self.runStep(Flash.FlashStep, parameter={"build":self.download.build_d2d["destination"],
                                                        "flash_key":"ota"})
            self.runStep(Flash.CheckFlashBuildStep, parameter={"build": self.download.build_d2d["destination"],
                                                                  "check_tag": True, "check_ssn": False,
                                                                  "check_ifwi": True, "check_modem": False,
                                                                  "check_img": True})
            if not self.verdict:
                self.runStep(Flash.FlashBuildIfNeededStep,
                             parameter={"build": self.download.build_d2d["destination"], "try_goto_mos": True},
                             impact_tc_verdict=False,
                             run_on_failure=True)
            self.allowed_TC_upon_success = ["all_TCs"]
            self.allowed_TC_upon_failure = ["Dessert2DessertFlashingTestCase"]
            self.removed_TC_upon_failure = ["OtaFlashingTestCase", "FastbootRebootBootloaderTestCase",
                                            "PkpOnInCosTestCase", "KernelWatchdogTestCase",
                                            "AdbRebootInRosTestCase"]
            self.allowed_TC_upon_skip = ["Dessert2DessertFlashingTestCase"]
            self.removed_TC_upon_skip = ["OtaFlashingTestCase", "FastbootRebootBootloaderTestCase",
                                         "PkpOnInCosTestCase", "KernelWatchdogTestCase",
                                         "AdbRebootInRosTestCase"]

    class IncrementalFotaScript(TestCaseModule.TestCaseModule.TC_structure):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_structure.__init__(self, conf)
            if not self.name:
                self.name = "IOTA - "
        def runAllSteps(self, flash_files=list()):
            if "bxtp" in self.configuration.board:
                self.logger.printLog("INFO", "Disable init and final")
                self.enable_init = False
                self.enable_final = False
            self.runStep(Flash.DownloadStep, parameter={"create_fota": True})
            self.runStep(Flash.CheckIotaLocalFlashFilePresenceStep, parameter={"build": self.download.build_fota})
            if self.skipping:
                self.toggleVerdict(self.VALID)
                self.output_dict["valid_log"] = "No Incremental OTA file found"
            # Disabeling ifwi check for bxtp #
            if "bxtp" not in self.configuration.board:
                self.runStep(Flash.StoreIfwiVersionStep, impact_tc_verdict=False)
            self.runStep(Flash.RunFotaFlashStep, parameter={"build": self.download.build_fota})

    class BuildButFromFlashFilesStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Flash BUT Data"
            self.description = "building Build Under Test data from flash file list"
            self.defaultParameter = {"flash_files": None}
        def step(self, parameter):
            flash_files = parameter["flash_files"]
            if not flash_files or not isinstance(flash_files, list):
                self.output = "invalid flash file list: {0} (python list expected)".format(flash_files)
                self.verdict = False
            else:
                self.download.build_but["flash_list"] = self.flashFile.flashFileList2DictionaryList(flash_files)
                # compute tag
                tag_regexp = ""
                tag_group = 0
                download_data = self.misc.downloadConfiguration2dictionary(
                        self.configuration.download.INTERNAL_DOWNLOAD,
                        ota=False,
                        multi_variant_prefix=self.configuration.download.MULTI_VARIANT_PREFIX)
                for file_type in download_data:
                    if file_type["tag_regexp"]:
                        tag_regexp = file_type["tag_regexp"]
                        tag_group = file_type.get("tag_group", 1)
                for f in self.download.build_but["flash_list"]:
                    local_flash_file = self.flashFile.createFile(f)
                    tag = self.download.getBuildTag(os.path.basename(local_flash_file.get_localFlashFile()),
                                                    extra_regexp=tag_regexp, extra_group=tag_group)
                    if tag:
                        self.download.build_but["tag"] = tag
                        self.description += " for {} build".format(tag)
                        break

            # fill cflasher version in TCR
            exec_status, version = self.host.commandExec("cflasher --version")
            if exec_status == 0 and re.search("Phone Flash Tool", version.split("\n")[0]):
                self.campaign.Campaign_information.updateJsonWithPftVersion(version.split("\n")[0])

    class PhoneFlashToolPurgeStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Purge PFT"
            self.description = "empty PFT download directory"
        def step(self, parameter):
            exec_status, output = self.host.commandExec("cflasher --purge-cache 0", 60)
            if exec_status != 0:
                self.output = "failure to purge PFT directory"
                self.verdict = False

    class WorkdirCleaningStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Clean Workdir"
            self.description = "cleaning working directory from previous download directories"
        def step(self, parameter):
            dir_list = ("BUT", "REF", "EXTERNAL", "USER", "FOTA", "D2D")
            workdir = self.globalConf.get("WORKDIR", "")
            if not workdir:
                self.output = "'WORKDIR' entry not found in global configuration, no directory set to download flash files"
                self.verdict = False
            else:
                if os.path.isdir(workdir):
                    self.logger.printLog("DEBUG", "working directory content: {0}".format(", ".join(os.listdir(workdir))))
                for directory in dir_list:
                    local_directory = os.path.join(workdir, directory)
                    if os.path.isdir(local_directory):
                        shutil.rmtree(local_directory)
                        self.logger.printLog("INFO", "cleaning... {0}".format(local_directory))

    class ProcessProvisioningJsonStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Process Provisioning Json"
            self.description = "parse provisioning json"
            self.defaultParameter = {"provisioning_json_file": ""}
        def step(self, parameter):
            provisioning_json_file = parameter["provisioning_json_file"]

            # check input type
            if not provisioning_json_file or not isinstance(provisioning_json_file, str):
                self.output = "invalid provisioning file: {0} (file name expected)".format(provisioning_json_file)
                self.verdict = False
                return

            if not self.misc.isExpectedJsonFormat(provisioning_json_file, ["build_target", "build_variant",
                                                                           "board_type", "url_buildinfo",
                                                                           "timeout", "download_timeout"],
                                                  top_keys=["provisioning_properties"]):
                self.output = "invalid provisioning file format: {0}".format(provisioning_json_file)
                self.verdict = False
                return

            # check json loading
            try:
                with open(provisioning_json_file) as f:
                    provisioning_data = json.load(f)
            except Exception as e:
                self.output = "impossible to load json file, invalid format for {0} ({1})"\
                    .format(provisioning_json_file, e)
                self.verdict = False
                return

            # process data
            self.download.provisioning_data = self.misc.loadProvisioningProperties(provisioning_data["provisioning_properties"])

            # check credentials
            if not self.host.is_credential:
                self.output = "credentials are not set, cannot download"
                self.verdict = False
                return

            try:
                self.download.provisioning_data["global_pft_timeout"] = int(self.download.provisioning_data["global_pft_timeout"])
                self.download.provisioning_data["artifactory_to_local_artifactory_download_timeout"] = int(self.download.provisioning_data["artifactory_to_local_artifactory_download_timeout"])
            except Exception as e:
                self.output = "issue while converting 'timeout' or 'download_timeout' into integer (error={0})".format(e)
                self.verdict = False

            self.logger.printLog("DEBUG", "Provisioning Data = {0}".format(self.download.provisioning_data))

    class FlashStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Flash Build"
            self.description = "flash specified build"
            self.defaultParameter = {"build": None,"flash_key": "force;recover", "adb": True}
        def step(self, parameter):
            build = parameter["build"]
            adb = parameter["adb"]
            if "tag" in build:
                self.description = "flash {} build".format(build.get("tag"))
            flash_key = parameter["flash_key"]
            if "recover" in flash_key:
                self.description = "flash {} build using recovery/blank procedure".format(build.get("tag", "specified"))
            elif "update" in flash_key:
                self.description = "flash {} build using fastboot/update procedure".format(build.get("tag", "specified"))
            elif "ota" in flash_key:
                self.description = "flash {} build using OTA procedure".format(build.get("tag", "specified"))
            elif "tag" in build:
                self.description = "flash {} build".format(build["tag"])

            if not isinstance(build, dict):
                self.output = "no build provided"
                self.verdict = False
            else:
                self.verdict = self.flash.flash(build,
                                                flash_key,
                                                check_tag=False,
                                                check_ifwi=False,
                                                check_modem=False,
                                                check_img=False,
                                                check_ssn=False,
                                                adb_after_flashing=adb)

    class FlashBuildIfNeededStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Flash Build If Needed"
            self.description = "flash specified build if needed"
            self.defaultParameter = {"build": None,"flash_key": "force;recover", "try_goto_mos": False,
                                     "pre_check_img": False, "pre_check_ifwi": False}
            self.specific = ""
        def step(self, parameter):
            pre_check_ifwi = parameter["pre_check_ifwi"]
            pre_check_img = parameter["pre_check_img"]
            build = parameter["build"]
            self.description = "flash {} build if needed using recovery/blank procedure".format(
                build.get("tag", "specified"))
            try_goto_mos = parameter["try_goto_mos"]
            flash_key = parameter["flash_key"]
            if not isinstance(build, dict) or not build:
                self.output = "no build provided"
                self.verdict = False
            else:
                if pre_check_ifwi:
                    self.specific = "and check ifwi"
                if pre_check_img:
                    self.specific += "{0}and check images".format(" " if self.specific else "")

                if "pre_flash_configuration" in build:
                    self.configuration.switchConfig(build["pre_flash_configuration"])
                if try_goto_mos:
                    self.osManager.gotoMos()
                if not self.flash.checkFlashedBuild(build, check_tag=True, check_ifwi=pre_check_ifwi,
                                                    check_img=pre_check_img, check_modem=False, check_ssn=False)\
                        or not self.flash.checkFlashPartitionFreeSpace(build):
                    if "bxtp" in self.configuration.board:
                        self.verdict = self.flash.flash(build,
                                                        flash_key,
                                                        check_tag=False,
                                                        check_ifwi=False,
                                                        check_modem=False,
                                                        check_img=True,
                                                        check_ssn=False)
                    else:
                        self.verdict = self.flash.flash(build,
                                                        flash_key,
                                                        check_tag=True,
                                                        check_ifwi=True,
                                                        check_modem=False,
                                                        check_img=True,
                                                        check_ssn=False)

    class LeaveFlashingSoftlyStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Leave Flashing Softly"
            self.description = "flash build if needed before leaving TC in case of script failure"
            self.defaultParameter = {"build":None, "flash_key":"force;recover",
                                     "pre_check_img":False, "pre_check_ifwi":False}
            self.specific = ""
        def step(self, parameter):
            pre_check_ifwi = parameter["pre_check_ifwi"]
            pre_check_img = parameter["pre_check_img"]
            build = parameter["build"]
            self.description = "flash {} build if needed before leaving TC in case of script failure".format(build.get("tag"))
            flash_key = parameter["flash_key"]
            if not isinstance(build, dict) or not build:
                self.output = "no build provided"
                self.verdict = False
            else:
                if pre_check_ifwi:
                    self.specific = "and check ifwi"
                if pre_check_img:
                    self.specific += "{0}and check images".format(" " if self.specific else "")
                if not self.flash.checkFlashedBuild(build, check_tag=True, check_ifwi=pre_check_ifwi,
                                                    check_img=pre_check_img):
                    self.verdict = self.flash.flash(build,
                                                    flash_key,
                                                    check_tag=True,
                                                    check_ifwi=False,
                                                    check_modem=False,
                                                    check_img=True,
                                                    check_ssn=False)
                else:
                    self.logger.printLog("INFO", "Flashing before leaving not needed as board properly flashed and booting")
            # do not execute further testing
            self.skip = True

    class CheckFlashBuildStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check Flash Build"
            self.description = "check flashed build"
            self.defaultParameter = {"build": None, "check_tag": False, "check_ifwi": False,
                                     "check_modem": False, "check_img": False, "check_ssn": False,
                                     "build_tag_override": ""}
        def step(self, parameter):
            build = parameter["build"]
            if "tag" in build:
                self.description = "check {} build is properly flashed".format(build["tag"])
            check_tag = parameter["check_tag"]
            check_ifwi = parameter["check_ifwi"]
            check_modem = parameter["check_modem"]
            check_img = parameter["check_img"]
            check_ssn = parameter["check_ssn"]
            build_tag_override = parameter["build_tag_override"]
            if not isinstance(build, dict) or not build:
                self.output = "no build provided"
                self.verdict = False
            else:
                if build_tag_override:
                    # override build tag:
                    self.logger.printLog("DEBUG", "overriding build tag: previous={0}, next={1}".format(build["tag"], build_tag_override))
                    build["tag"] = build_tag_override
                if "bxtp" in self.configuration.board:
                    self.verdict = self.flash.checkFlashedBuildBxtpMrb(build,
                                                                check_tag=False,
                                                                check_ifwi=False,
                                                                check_modem=False,
                                                                check_img=True,
                                                                check_ssn=False)
                else:
                    self.verdict = self.flash.checkFlashedBuild(build,
                                                                check_tag=check_tag,
                                                                check_ifwi=check_ifwi,
                                                                check_modem=check_modem,
                                                                check_img=check_img,
                                                                check_ssn=check_ssn)

    class StoreIfwiVersionStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Store Ifwi Version"
            self.description = "get and store ifwi version if BUT flashed on the device"
            self.defaultParameter = {}
        def step(self, parameter):
            if self.download.build_d2d:
                build = self.download.build_d2d["destination"]
            else:
                build = self.download.build_but
            self.verdict = self.flash.checkFlashedBuild(build,
                                                        check_tag=True,
                                                        check_ifwi=True,
                                                        check_modem=False,
                                                        check_img=False,
                                                        check_ssn=False)

            if self.download.build_fota and \
                    not any("fota_update" not in self.download.build_fota[fota] for fota in self.download.build_fota) and \
                    "valid_ifwi_version" in self.download.build_but:
                self.logger.printLog("DEBUG", "storing ifwi value in FOTA dict")
                for fota in self.download.build_fota:
                    self.download.build_fota[fota]["fota_update"]["valid_ifwi_version"] = self.download.build_but["valid_ifwi_version"]

            if self.download.build_d2d and \
                    "destination" in self.download.build_d2d and \
                    "valid_ifwi_version" in self.download.build_but:
                self.logger.printLog("DEBUG", "storing ifwi value in D2D dict")
                self.download.build_d2d["destination"]["valid_ifwi_version"] = self.download.build_but["valid_ifwi_version"]

    class DownloadStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Download Build"
            self.description = "Download flash files from artifactory"
            self.defaultParameter = {"create_but": True, "create_ref": False,
                                     "but_ota_files_needed": False, "ref_ota_files_needed": False,
                                     "create_fota": False,
                                     "but_url": "", "ref_url": "", "create_external": False, "create_d2d":False, "download_rma_tools":False}
        def step(self, parameter):
            create_but = parameter["create_but"]
            create_ref = parameter["create_ref"]
            create_external = parameter["create_external"]
            but_ota_files_needed = parameter["but_ota_files_needed"]
            ref_ota_files_needed = parameter["ref_ota_files_needed"]
            create_fota = parameter["create_fota"]
            create_d2d = parameter["create_d2d"]
            but_url = parameter["but_url"]
            ref_url = parameter["ref_url"]
            download_rma_tools = parameter["download_rma_tools"]
            self.verdict = self.download.newDownload(create_but=create_but,
                                                     create_ref=create_ref,
                                                     but_ota_files_needed=but_ota_files_needed,
                                                     ref_ota_files_needed=ref_ota_files_needed,
                                                     create_fota=create_fota,
                                                     but_url=but_url,
                                                     ref_url=ref_url,
                                                     create_external=create_external,
                                                     create_d2d=create_d2d,
                                                     download_rma_tools=download_rma_tools)

    class CheckLocalFlashFilePresenceStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check Presence"
            self.description = "Check local flash file presence on disk"
            self.defaultParameter = {"build": {}, "mandatory_files": "fastboot{0}".format(";blankphone" if any(("pft_key" in element and element["pft_key"] =="blankphone") for element in self.configuration.download.INTERNAL_DOWNLOAD) else "")}
        def step(self, parameter):
            build = parameter["build"]
            mandatory_files = parameter["mandatory_files"]
            if not build or not "flash_list" in build:
                self.verdict = False
                self.output = "invalid build: {0}".format(build)
            else:
                for f in build["flash_list"]:
                    local_flash_file = self.flashFile.createFile(f)
                    if local_flash_file.get_localFlashFile() == "" or not os.path.isfile(local_flash_file.get_localFlashFile()):
                        self.verdict = False
                        if not self.output:
                            self.output = "{1}invalid flash file: {0}".format(local_flash_file.get_localFlashFile(),
                                                                              " | " if self.output else "")
                    else:
                        self.logger.printLog("INFO", "flash file presence checked: {0}"
                                             .format(local_flash_file.get_localFlashFile()))
                for files_type in mandatory_files.split(";"):
                    if not any (f["file_type"] == files_type for f in build["flash_list"]):
                        if files_type == "ota":
                            self.output += "No FULL OTA file found"
                            self.logger.printLog("INFO", "No Full OTA file found: skipping TC")
                            self.skip = True
                            continue
                        else:
                            self.verdict = False
                            self.output = "some files missing among: {0}".format(", ".join(mandatory_files.split(";")))
                            continue

    class SideloadStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Flash With Sideload"
            self.description = "used adb sideload command to flash OTA package"
            self.defaultParameter = {"build": {}}
        def step(self, parameter):
            build = parameter["build"]
            #root the device
            if not self.device.adbRoot():
                self.verdict = False
                return
            #adb reboot sideload-auto-reboot
            if not self.osManager.adbRebootSideloadAutoReboot():
                self.verdict = False
                return
            #adb sideload <ota package> ()
            ota_package = ""
            for element in build["flash_list"]:
                local_flash_file = self.flashFile.createFile(element)
                if local_flash_file.get_file_type() == "ota":
                    ota_package = local_flash_file.get_localFlashFile()
            if not ota_package:
                self.verdict = False
                self.output = "no package 'ota' found"
                return
            exec_status, output = self.host.commandExecAdb("sideload {}".format(ota_package), 2000)
            if exec_status != 0:
                self.verdict = False
                self.output = "failure with 'adb sideload {}'".format(ota_package)
                return
            if not self.osManager.waitOs("main",
                                         timeout=1400,
                                         blind=True,
                                         waitWdExp=False):
                self.verdict = False
                return

    class CheckIotaLocalFlashFilePresenceStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check Presence For Incremental OTA Flash Files"
            self.description = "Check local flash file presence on disk"
            self.defaultParameter = {"build": None}
        def step(self, parameter):
            build = parameter["build"]
            if not isinstance(build, dict):
                self.verdict = False
                self.output = "invalid build: {0} (dictionary expected)".format(build)
            else:
                if not build.keys():
                    self.output = "No Incremental OTA file found"
                    self.logger.printLog("INFO", "No Incremental OTA file found: skipping TC")
                    self.skip = True
                else:
                    for fota in build:
                        check_list = ("fota", "flash_list", "fota_update")
                        missing_keys = [key for key in check_list if key not in build[fota]]
                        if "fota_update" in build[fota] and not "flash_list" in build[fota]["fota_update"]:
                            missing_keys.append("fota_update_flash_list")
                        if missing_keys:
                            self.output = "{0} flash files missing".format(", ".join(missing_keys))
                            self.verdict = False
                        else:
                            if not os.path.isfile(build[fota]["fota"]):
                                self.output = "file missing: {0}".format(build[fota]["fota"])
                                self.verdict = False
                            else:
                                for f in build[fota]["flash_list"]:
                                    local_flash_file = self.flashFile.createFile(f)
                                    if not os.path.isfile(local_flash_file.get_localFlashFile()):
                                        self.output += "{1}file missing: {0}".format(local_flash_file.get_localFlashFile(), " | " if self.output else "")
                                        self.verdict = False
                                    else:
                                        self.logger.printLog("INFO", "flash file presence checked: {0}".format(local_flash_file.get_localFlashFile()))
                                for f in build[fota]["fota_update"]["flash_list"]:
                                    local_flash_file = self.flashFile.createFile(f)
                                    if not os.path.isfile(local_flash_file.get_localFlashFile()):
                                        self.output += "{1}file missing: {0}".format(local_flash_file.get_localFlashFile(), " | " if self.output else "")
                                        self.verdict = False
                                    else:
                                        self.logger.printLog("INFO", "flash file presence checked: {0}".format(local_flash_file.get_localFlashFile()))

    class CheckD2DLocalFlashFilePresenceStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Check Presence For Dessert To Dessert Flash Files"
            self.description = "Check local flash file presence on disk"
            self.defaultParameter = {"build": None}
        def step(self, parameter):
            build = parameter["build"]
            if not isinstance(build, dict):
                self.verdict = False
                self.output = "invalid build: {0} (dictionary expected)".format(build)
            else:
                if not build.keys() or ("output_log" in build and build["output_log"] != ""):
                    self.output += "No source build provided"
                    self.logger.printLog("INFO", "No source build provided, skipping TC{0}"
                    .format(" ({0})".format(build.get("output_log", "")) if build.get("output_log", "") else ""))
                    self.skip = True
                else:
                    check_list = ("source", "destination")
                    missing_keys = [key for key in check_list if (key not in build or "flash_list" not in build[key])]
                    if missing_keys:
                        self.output = "{0} flash key missing".format(", ".join(missing_keys))
                        self.verdict = False
                    else:
                        for f in build["source"]["flash_list"]:
                            local_flash_file = self.flashFile.createFile(f)
                            if not os.path.isfile(local_flash_file.get_localFlashFile()):
                                self.output += "{1}file missing: {0}".format(local_flash_file.get_localFlashFile(), " | " if self.output else "")
                                self.verdict = False
                            else:
                                self.logger.printLog("INFO", "flash file presence checked: {0}".format(local_flash_file.get_localFlashFile()))
                        for f in build["destination"]["flash_list"]:
                            local_flash_file = self.flashFile.createFile(f)
                            if not os.path.isfile(local_flash_file.get_localFlashFile()):
                                self.output += "{1}file missing: {0}".format(local_flash_file.get_localFlashFile(), " | " if self.output else "")
                                self.verdict = False
                            else:
                                self.logger.printLog("INFO", "flash file presence checked: {0}".format(local_flash_file.get_localFlashFile()))

    class RunFotaFlashStep(TestCaseModule.TestCaseModule.TC_element):
        def __init__(self, conf):
            TestCaseModule.TestCaseModule.TC_element.__init__(self, conf)
            if not self.name:
                self.name = "Flash Incremental OTA Packages"
            self.description = "flash source build then flash packages one by one"
            self.defaultParameter = {"build": None}
        def step(self, parameter):
            build = parameter["build"]

            output_list = list()
            number_of_packages = len(build)

            for index, fota in enumerate(build):
                self.subTestIndex("FOTA {0}/{1}".format(index+1, number_of_packages))

                # OTA file info
                text = "*** Incremental OTA update file: {0} ***".format(os.path.basename(build[fota]["fota"].replace(".zip", "")))
                self.logger.printLog("INFO", "*"*len(text))
                self.logger.printLog("INFO", text)
                self.logger.printLog("INFO", "*"*len(text))
                self.logger.printLog("INFO", "Incremental OTA update from {0} to {1}".format(build[fota]["tag"],
                                                                                             build[fota]["tag_update"]))

                self.subTestIndex("FOTA {0}/{1} - 1/2".format(index+1, number_of_packages))
                # flash source build if needed
                if not self.flash.checkFlashedBuild(build[fota], check_tag=True, check_ifwi=False, check_modem=False,
                                                    check_img=False, check_ssn=False):
                    if not self.flash.flash(build[fota],
                                            "force;recover",
                                            check_tag   = True,
                                            check_ifwi  = False,
                                            check_modem = False,
                                            check_img   = False,
                                            check_ssn   = False):
                        self.flash.leaveFlashingSoftly(self.download.build_but)
                        output_list.append(fota + ": failure to flash source build")
                        self.verdict = False
                        continue

                self.subTestIndex("FOTA {0}/{1} - 2/2".format(index+1, number_of_packages))
                # run ota update
                if not self.flash.flash(build[fota]["fota_update"], "fota"):
                    self.flash.leaveFlashingSoftly(self.download.build_but)
                    output_list.append(fota + ": failure to flash or boot after Incremental OTA update")
                    self.verdict = False
                else:
                    output_list.append(fota + ": success")

            # reset index
            self.subTestIndex(0)
            if output_list:
                self.output += "{1}{0}".format(" --- ".join(output_list), " | " if self.output else "")
