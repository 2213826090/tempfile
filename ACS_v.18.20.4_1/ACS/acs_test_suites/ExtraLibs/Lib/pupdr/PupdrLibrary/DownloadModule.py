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
@summary: Pupdr Library - DownloadModule
@since: 11/17/2014
@author: travenex
"""

import os
import re
import json
import shutil
import zipfile
import urllib2
import tempfile
import OutputModule
import HostModule
import LoggerModule
import LogsModule
import FlashFileModule
import DeviceModule
import FlashModule
import MiscModule
import ConfigurationModule
import CampaignModule

class DownloadModule(object):

    __instance = None
    __globalConf = None
    __logger = None
    __device = None
    build_but = None
    build_ref = None
    build_fota = None
    build_external = None
    build_d2d = None
    provisioning_data = None
    __path_FOTA = None
    __flashFile = None
    __host = None
    __output = None
    __campaign = None
    __flash = None
    __misc = None
    __configuration = None
    __logs = None
    __code_tag_regexp = None
    __provisioning_build_info_json = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.__instance:
            cls.__instance = object.__new__(cls)
        return cls.__instance

    def init(self, globalConf):
        self.__globalConf = globalConf
        self.__logger = LoggerModule.LoggerModule()
        self.__host = HostModule.HostModule()
        self.__output = OutputModule.OutputModule()
        self.__device = DeviceModule.DeviceModule()
        self.__flashFile = FlashFileModule.FlashFileModule()
        self.__misc = MiscModule.MiscModule()
        self.__flash = FlashModule.FlashModule()
        self.__logs = LogsModule.LogsModule()
        self.__campaign = CampaignModule.CampaignModule()
        self.build_but = dict()
        self.provisioning_data = dict()
        self.build_ref = dict()
        self.build_fota = dict()
        self.build_external = dict()
        self.build_d2d = dict()
        self.build_d2d["source"] = dict()
        self.build_d2d["destination"] = dict()
        self.build_d2d["output_log"] = ""
        self.__path_FOTA = ""
        self.__configuration = ConfigurationModule.ConfigurationModule()
        self.__code_tag_regexp = r'([a-zA-Z][a-zA-Z0-9]+[0-9][0-9][0-9][0-9](phonedoctor)?)'
        if self.__logs.bootota_log_path:
            self.__provisioning_build_info_json = os.path.join(self.__logs.bootota_log_path, "provisioning_build_info.json")
        else:
            self.__provisioning_build_info_json = ""

    def newDownload(self, create_but=True,
                          create_ref=False,
                          but_ota_files_needed=False,
                          ref_ota_files_needed=False,
                          create_fota=False,
                          but_url="",
                          ref_url="",
                          create_external=False,
                          create_d2d=False,
                          download_rma_tools=False):
        """ init
        """
        # Get PFT download dir from opt
        verdict = True
        output = ""

        # clean dictionaries
        self.build_but.clear()
        self.build_ref.clear()
        self.build_fota.clear()
        self.build_external.clear()
        self.build_d2d["source"].clear()
        self.build_d2d["destination"].clear()
        self.build_d2d["output_log"] = ""
        global DOWNLOAD_RMA_TOOLS
        DOWNLOAD_RMA_TOOLS = download_rma_tools

        if not self.provisioning_data:
            self.__logger.printLog("INFO", "downloading data with data from 'PROVISIONING_DATA' entry in global conf")
            if not isinstance(self.__globalConf.get("PROVISIONING_DATA"), dict):
                self.__logger.printLog("WARNING", "invalid input in 'PROVISIONING_DATA' global configuration parameter "
                                                  "(python dictionary expected)")
                self.provisioning_data = self.__misc.loadProvisioningProperties({})
            else:
                self.provisioning_data = self.__misc.loadProvisioningProperties(self.__globalConf.get("PROVISIONING_DATA"))

            if but_url:
                self.__logger.printLog("INFO", "forcing build url {}".format(but_url))
                self.provisioning_data["url_buildinfo"] = but_url

            if not self.provisioning_data["board_type"]:
                self.__logger.printLog("INFO", "forcing board type {}".format(self.__configuration.board))
                self.provisioning_data["board_type"] = self.__configuration.board

        if create_but:
            self.build_but.update(self.createDictionary(os.path.join(self.__globalConf.get("WORKDIR"), "BUT"),
                                              "",
                                              self.provisioning_data["board_type"],
                                              self.provisioning_data["buildvariant"], "BUT",
                                              but_ota_files_needed,
                                              optional_url=self.provisioning_data["url_buildinfo"]))
            if DOWNLOAD_RMA_TOOLS:
                verdict = True
                return verdict
            if not self.build_but:
                output = "failure to build BUT dictionary"
                verdict = False
            else:
                # Output variables
                self.__logger.printLog("INFO", "Build under test tag: '{0}'".format(self.build_but["tag"]))

        # also prepare ref dict if needed
        if verdict and create_ref:
            # create a dictionary that contains 'key, values' with downloaded files
            self.build_ref.update(self.createDictionary(os.path.join(self.__globalConf.get("WORKDIR"), "REF"),
                                              self.build_but["tag"] if (self.build_but and "tag" in self.build_but) else "",
                                              self.provisioning_data["board_type"],
                                              self.provisioning_data["buildvariant"],
                                              "REF",
                                              ref_ota_files_needed,
                                              optional_url=ref_url))
            if DOWNLOAD_RMA_TOOLS:
                verdict = True
                return verdict
            if not self.build_ref:
                output = "failure to build REF dictionary"
                verdict = False
            elif self.build_but and (self.build_but["tag"] == self.build_ref["tag"] != ""):
                output = "REF and BUT have same build incremental tag: {0}".format(self.build_but["tag"])
                verdict = False

        if verdict and create_fota:
            verdict, fota_dict = self.__createFotaDict(self.provisioning_data["buildvariant"])

            if not verdict:
                output = "failure to build FOTA dictionary"
            else:
                self.build_fota.update(fota_dict)

        if verdict and create_external:
            self.build_external.update(self.createDictionary(os.path.join(self.__globalConf.get("WORKDIR"), "EXTERNAL"),
                                              "",
                                              self.provisioning_data["board_type"],
                                              self.provisioning_data["buildvariant"],
                                              "EXTERNAL",
                                              False))
            if DOWNLOAD_RMA_TOOLS:
               verdict = True
               return verdict
            if not self.build_external:
                output = "failure to build EXTERNAL dictionary"
                verdict = False

        if verdict and create_d2d:
            verdict, d2d_dict = self.__createD2DDict()

            if not verdict:
                output = "failure to build D2D dictionary"
            else:
                self.build_d2d["source"].update(d2d_dict["source"])
                self.build_d2d["destination"].update(d2d_dict["destination"])
                self.build_d2d["output_log"] = d2d_dict["output_log"]

        # fill cflasher version in TCR
        exec_status, version = self.__host.commandExec("cflasher --version")
        if exec_status == 0 and re.search("Flash Tool", version.split("\n")[0]):
            self.__campaign.Campaign_information.updateJsonWithPftVersion(version.split("\n")[0])

        self.__output.appendOutput(output, verdict)
        return verdict

    # inner function to download build_info.json and parse it
    def __downloadAndParseBuildInfo(self, url):
        log = "downloadAndParseBuildInfo(): "
        verdict = True
        output = ""
        json_data = {}
        self.__logger.printLog("INFO", log + "starting for {}".format(url))
        global local_build_info_path
        local_build_info_path = ""
        # download build_info.json
        if url == self.provisioning_data["url_buildinfo"]:
            local_build_info_path = self.getBuildInfoJsonLocation(url)
        if not local_build_info_path:
            if not self.__logs.bootota_log_path:
                local_build_info_path = os.path.join(tempfile.gettempdir(), "tmp_build_info_{}.json".format(self.__misc.getUser()))
                self.__logger.printLog("DEBUG", log + "no local report path to save temporary data into, choosing tmp dir")
            else:
                local_build_info_path = os.path.join(self.__logs.bootota_log_path, "tmp_build_info.json")
            self.__misc.local_files_handler.addEntry(local_build_info_path)
            self.__logger.printLog("INFO", log + "no local copy found, downloading to {}".format(local_build_info_path))
            if os.path.isfile(local_build_info_path):
                os.remove(local_build_info_path)
            self.__host.commandExecArtifactorySecureDownload(url, os.path.dirname(local_build_info_path), file_name=os.path.basename(local_build_info_path))
            if not os.path.isfile(local_build_info_path):
                verdict = False
                output = "failure to download build_info {}".format(url)
        # read json
        if verdict:
            try:
                with open(local_build_info_path) as f:
                    json_data = json.load(f)
            except Exception as e:
                output = "failure to parse local copy of {} (error={})".format(url, e)
                json_data = {}
                verdict = False
        if os.path.isfile(local_build_info_path):
            os.remove(local_build_info_path)

        self.__output.appendOutput(output, verdict)
        return json_data

    # inner PFT download method
    def __downloadFlashfile(self, url, build_info_json_data, board_type, buildvariant, flash_files_entry, info_path, fallback=False):
        log = "downloadFlashfile(): "
        verdict = True
        output = ""
        local_flash_file_name = ""
        relative_artifactory_path = ""
        local_pft_key = ""
        info_json_data = {}
        local_configuration = ""
        local_buildvariant = buildvariant
        local_groups = {}

        # extract file location from build_info.json data
        if "hardwares" not in build_info_json_data:
            output = "'hardwares' entry missing in build_info.json"
            verdict = False
        else:
            hardwares_entry = build_info_json_data["hardwares"]
            if board_type not in hardwares_entry:
                output = "'{}' entry missing in 'hardwares' entry " \
                         "in build_info.json".format(board_type)
                verdict = False
            else:
                board_type_entry = hardwares_entry[board_type]
                if "variants" not in board_type_entry:
                    output = "'variants' entry missing in 'hardwares/{}' entry " \
                             "in build_info.json".format(board_type)
                    verdict = False
                else:
                    variants_entry = board_type_entry["variants"]
                    if flash_files_entry.get("forced_variant"):
                        local_buildvariant = local_buildvariant.split("-")[0] + "-" + flash_files_entry["forced_variant"]
                        self.__logger.printLog("INFO", log + "overriding '{}' original buildvariant by '{}'".format(buildvariant, local_buildvariant))

                    if local_buildvariant not in variants_entry and "cht_ffd-userdebug" not in variants_entry and "cht51-2016" not in self.__misc.otaBuild:
                        output = "'{1}' entry missing in 'hardwares/{0}/variants' entry " \
                                "in build_info.json".format(board_type, local_buildvariant)
                        verdict = False
                    else:
                        if "cht_ffd-userdebug" in variants_entry and "cht51-2016" in self.__misc.otaBuild:
                            self.__logger.printLog("INFO", log + "Found cht51-2016 D2D test - overriding {} with cht_ffd-userdebg".format(local_buildvariant))
                            local_buildvariant = "cht_ffd-userdebug"

                        buildvariant_entry = variants_entry[local_buildvariant]
                        if "flashfiles" not in buildvariant_entry:
                            output = "'flashfiles' entry missing in 'hardwares/{0}/variants/{1}' entry " \
                                     "in build_info.json".format(board_type, buildvariant)
                            verdict = False
                        else:
                            flashfiles_entry = buildvariant_entry["flashfiles"]
                            if fallback and flash_files_entry.get("pft_fallback", {}).get("pft_key"):
                                local_pft_key = flash_files_entry["pft_fallback"]["pft_key"]
                            else:
                                local_pft_key = flash_files_entry["pft_key"]
                            if local_pft_key not in flashfiles_entry:
                                output = "'{2}' entry missing in 'hardwares/{0}/variants/{1}/flashfiles' entry " \
                                         "in build_info.json".format(board_type, local_buildvariant, local_pft_key)
                                verdict = False
                            else:
                                raw_data = str(flashfiles_entry[local_pft_key][0])
                                if not raw_data:
                                    output = "empty entry at 'hardwares/{0}/variants/{1}/flashfiles/{2}' entry " \
                                             "in build_info.json".format(board_type, local_buildvariant, local_pft_key)
                                    verdict = False
                                else:
                                    self.__logger.printLog("DEBUG", log + "raw json data = {}".format(raw_data))
                                    relative_artifactory_path = raw_data.split(":")[0]
                                    if len(raw_data.split(":")) == 2:
                                        local_flash_file_name = raw_data.split(":")[1]
                                        self.__logger.printLog("DEBUG", log + "original flash file = {}".format(local_flash_file_name))
                                    if flash_files_entry["flash_file"]:
                                        if local_flash_file_name:
                                            self.__logger.printLog("DEBUG", log + "original flash file overridden by {}".format(flash_files_entry["flash_file"]))
                                        local_flash_file_name = flash_files_entry["flash_file"]
                                    if fallback and flash_files_entry.get("pft_fallback", {}).get("flash_file"):
                                        if local_flash_file_name:
                                            self.__logger.printLog("DEBUG", log + "original flash file overridden by {}".format(flash_files_entry["pft_fallback"]["flash_file"]))
                                        local_flash_file_name = flash_files_entry["pft_fallback"]["flash_file"]

        if DOWNLOAD_RMA_TOOLS:
           rma_file_name="platform-rma-tools-linux.zip"
           full_url_rma_tools=(url.split("/build_info")[0] + relative_artifactory_path).split("/user")[0] + "/tools/linux-x86/" + rma_file_name
           rma_destination_dir =os.path.dirname(local_build_info_path)
           rma_destination_file=os.path.join(rma_destination_dir, rma_file_name)
           self.__host.commandExecArtifactorySecureDownload(full_url_rma_tools, rma_destination_dir, file_name=rma_file_name)
           if not os.path.isfile(rma_destination_file):
               verdict = False
               output = "failure to download rma_tools {}".format(full_url_rma_tools)
           else:
               try:
                    z = zipfile.ZipFile(rma_destination_file, 'r')
                    z.extractall(path = '../../tools/rma/')
                    z.close()
                    verdict = True
               except zipfile.BadZipfile as e:
                    output = "failure to unzip {} (error={})".format(zipfile, e)
                    verdict = False
           return verdict

        # add build_info.json link
        if output:
            output + " {}".format(url)

        # generate pft download command
        if verdict:
            full_url_to_artifact = url.split("/build_info")[0] + relative_artifactory_path
            start_download_timeout = int(self.provisioning_data["artifactory_to_local_artifactory_download_timeout"] / 60) + 1
            local_configuration = flash_files_entry["configuration"]
            if fallback and flash_files_entry.get("pft_fallback", {}).get("configuration"):
                local_configuration = flash_files_entry["pft_fallback"]["configuration"]
            local_groups = flash_files_entry["groups"]
            if fallback and flash_files_entry.get("pft_fallback", {}).get("groups"):
                local_groups = flash_files_entry["pft_fallback"]["groups"]

            # file to download
            cmd = "-f {0}".format(full_url_to_artifact)
            cmd += " --download-only"
            cmd += " --buildvariant {0}".format(local_buildvariant)
            cmd += " --board {0}".format(board_type)
            cmd += " --build-info-output {0}".format(info_path)
            cmd += " --start-download-timeout {0}".format(start_download_timeout)
            cmd += " --download-stuck-timeout 5"
            if local_configuration:
                cmd += " -c {0}".format(local_configuration)
            if local_flash_file_name:
                cmd += " -x {0}".format(local_flash_file_name)
            if not isinstance(local_groups, dict):
                self.__logger.printLog("WARNING", "'groups' parameter expects dictionary (actual value = {0})".format(local_groups))
            else:
                for element in local_groups:
                    if local_groups[element]:
                        cmd += " --enable-group {0}".format(element)
                    else:
                        cmd += " --disable-group {0}".format(element)

            # launch command and check status
            exec_status = self.__host.commandExecCflasher(cmd, self.provisioning_data["global_pft_timeout"])[0]
            if exec_status != 0:
                output = "failure with PFT download command for '{}' flashfile type".format(local_pft_key)
                verdict = False

            elif not os.path.isfile(info_path):
                output = "PFT failed to generate build-info-output json while downloading '{}' flashfile type".format(local_pft_key)
                verdict = False

            else:
                try:
                    with open(info_path) as f:
                        info_json_data = json.load(f)
                except Exception as e:
                    output = "failure to parse PFT json output file {} (error={})".format(info_path, e)
                    verdict = False

        # check generated data
        if verdict and local_configuration:
            if "flashConfiguration" not in info_json_data:
                output = "missing 'flashConfiguration' entry in PFT output json"
                verdict = False
            elif info_json_data["flashConfiguration"] != local_configuration:
                output = "downloaded configuration issue: actual='{}', expected='{}'".format(info_json_data["flashConfiguration"], local_configuration)
                verdict = False
            else:
                self.__logger.printLog("DEBUG", log + "downloaded configuration checked ({})".format(local_configuration))
        # flashFileName check
        if verdict and local_flash_file_name:
            if "flashFileName" not in info_json_data:
                output = "missing 'flashFileName' entry in PFT output json"
                verdict = False
            elif info_json_data["flashFileName"] != local_flash_file_name:
                output = "downloaded flash file name issue: actual='{}', expected='{}'".format(info_json_data["flashFileName"], local_flash_file_name)
                verdict = False
            else:
                self.__logger.printLog("DEBUG", log + "downloaded flash file name checked ({})".format(local_flash_file_name))
        # groups check
        if verdict and local_groups:
            if "flashGroupsState" not in info_json_data:
                output = "missing 'flashGroupsState' entry in PFT output json"
                verdict = False
            elif not isinstance(local_groups, dict):
                self.__logger.printLog("WARNING", "'groups' parameter expects dictionary (actual value = {0})".format(local_groups))
            else:
                # get flash file
                if not local_flash_file_name:
                    local_flash_file_name = info_json_data.get("flashFileName")
                    if not local_flash_file_name:
                        local_flash_file_name = "flash.json"
                # read and parse flash file
                zip_file = info_json_data.get("localFlashFile")
                dirname = ""
                if not zip_file:
                    output = "missing 'flashGroupsState' entry in PFT output json"
                    verdict = False
                else:
                    dirname = zip_file.replace(".zip", "")
                    if not os.path.dirname(dirname):
                        try:
                            z = zipfile.ZipFile(zip_file, 'r')
                            z.extractall(dirname)
                            z.close()
                        except zipfile.BadZipfile as e:
                            output = "failure to unzip {} (error={})".format(zipfile, e)
                            verdict = False
                expected_groups = {}
                if verdict:
                    flash_file_path = os.path.join(dirname, local_flash_file_name)
                    if not os.path.isfile(flash_file_path):
                        verdict = False
                        output = "'{}' file not present in {}".format(local_flash_file_name, dirname)
                    else:
                        try:
                            self.__logger.printLog("DEBUG", log + "parsing {}".format(flash_file_path))
                            with open(flash_file_path) as f:
                                local_flash_file_data = json.load(f)
                        except Exception as e:
                            verdict = False
                            output = "failure to parse flash file {} (error={})".format(local_flash_file_name, e)
                        else:
                            local_configuration_data = local_flash_file_data.get("flash", {}).get("configurations")
                            if not local_configuration_data:
                                self.__logger.printLog("DEBUG", log + "missing 'configurations' entry in flash file {}".format(local_flash_file_name))
                            elif not local_configuration:
                                for single_config in local_configuration_data:
                                    if local_configuration_data[single_config].get("default"):
                                        local_configuration = single_config
                                        self.__logger.printLog("DEBUG", log + "found '{}' default configuration".format(single_config))
                                        break
                            if local_configuration:
                                expected_groups = local_configuration_data.get(local_configuration).get("groupsState", {})
                                self.__logger.printLog("DEBUG", log + "expected groups in flash file for '{}' "
                                                                "configuration :\n{}".format(local_configuration,
                                                                                             json.dumps(expected_groups, sort_keys=True, indent=4)))

                if verdict:
                    for element in local_groups:
                        if element not in info_json_data["flashGroupsState"]:
                            actual_element_state = "not present"
                        elif info_json_data["flashGroupsState"][element]:
                            actual_element_state = "enabled"
                        else:
                            actual_element_state = "disabled"

                        if element not in expected_groups:
                            self.__logger.printLog("INFO", log + "'{}' group not in flash file, skipping".format(element))
                        else:
                            expected_element_state = "enabled" if local_groups[element] else "disabled"
                            if expected_element_state != actual_element_state:
                                verdict = False
                                output = "downloaded group issue for '{}' group: actual='{}', expected='{}'".format(element, actual_element_state, expected_element_state)
                                break
                            else:
                                self.__logger.printLog("DEBUG", log + "downloaded group checked ({}={})".format(element, actual_element_state))

        #remove file if failure
        if not verdict and os.path.isfile(info_path):
            os.remove(info_path)

        self.__output.appendOutput(output, verdict)
        return verdict

    # get_flashfiles()
    # url_buildinfo  : weekly root url of the build
    # board_type     : board_type from slave_configuration.xml
    # target_dir     : local working directory path
    # path_pft       : Download dir of pft
    # buildvariant   : full name of buildvariant
    # file_type_list : download needed files only
    def getFlashfiles(self,
                       url_buildinfo,
                       board_type,
                       target_dir,
                       buildvariant,
                       download_data,
                       device=""):
        """ Downloads the zip flashfiles from url by using provided json dictionary.
                Create symbolic link in BUT and REF directory that point to downloaded files.
                Return a dictionary with keys 'blankphone', 'ota' and 'fastboot'.
                Values of returned dictionary are the paths of corresponding symbolic links.
        """
        log = "getFlashfiles(): "
        self.__logger.printLog("INFO", log + "start")
        verdict = True
        output_issue = list()

        buildinfo_link = url_buildinfo.replace("/build_info.json", "")

        # Create REF/BUT directory
        if os.path.isdir(target_dir):
            shutil.rmtree(target_dir)
        os.makedirs(target_dir)

        alternative_artifactory_list = ["buildbot", "jenkins", "sit"]
        direct_download_allowed = False
        for artifactory in alternative_artifactory_list:
            detects = []
            for flash_files_entry in download_data:
                if isinstance(flash_files_entry.get("{0}_artifactory_path".format(artifactory)), dict):
                    if flash_files_entry["{0}_artifactory_path".format(artifactory)].get("detection"):
                        detects.append(flash_files_entry["{0}_artifactory_path".format(artifactory)]["detection"])
            if len(detects) != len(download_data):
                break
            if detects and not any(detect not in buildinfo_link for detect in detects):
                direct_download_allowed = True

        build = list()
        for flash_files_entry in download_data:
            # execute PFT download if pft_key provided
            if flash_files_entry["pft_key"]:
                # define pft output name
                if flash_files_entry["local_key"]:
                    pft_info_json = os.path.join(target_dir, flash_files_entry["local_key"] + ".json")
                    link = os.path.join(target_dir, flash_files_entry["local_key"])
                else:
                    pft_info_json = os.path.join(target_dir, flash_files_entry["pft_key"] + ".json")
                    link = os.path.join(target_dir, flash_files_entry["pft_key"])
                # download build_info
                build_info_json_data = self.__downloadAndParseBuildInfo(url_buildinfo)
                if not build_info_json_data:
                    output = "failure to get json data from {0}".format(url_buildinfo)
                    self.__logger.printLog("WARNING", output)
                    output_issue.append(output)
                    local_verdict = False
                else:
                    # Download flash file
                    local_verdict = self.__downloadFlashfile(url_buildinfo, build_info_json_data, board_type, buildvariant, flash_files_entry, pft_info_json)
                    if not local_verdict and flash_files_entry["pft_fallback"]:
                        self.__logger.printLog("WARNING", "failure with PFT command, trying with fallback parameters =\n{}".format(json.dumps(flash_files_entry["pft_fallback"])))
                        local_verdict = self.__downloadFlashfile(url_buildinfo, build_info_json_data, board_type, buildvariant, flash_files_entry, pft_info_json, fallback=True)

                if DOWNLOAD_RMA_TOOLS:
                   return local_verdict

                # check if output file exist, because may not exist on older pft version
                if local_verdict:
                    dest = pft_info_json
                    # create link to actual file
                    if os.path.exists(link):
                        os.remove(link)
                    with open(pft_info_json) as f:
                        json_data = json.load(f)
                        if "localFlashFile" in json_data:
                            if os.path.isdir(json_data["localFlashFile"].replace(".zip", "")):
                                download_dir = json_data["localFlashFile"].replace(".zip", "")
                            else:
                                download_dir = os.path.dirname(json_data["localFlashFile"])
                            if "windows" in os.environ.get('OS', '').lower():
                                os.system('mklink /D {0} {1}'.format(link, download_dir))
                            else:
                                os.symlink(download_dir, link)
                            self.__logger.printLog("INFO", "creating link to '{0}' at: {1}".format(download_dir, link))
                        else:
                            self.__logger.printLog("WARNING", "'localFlashFile' key not found in pft json: {0}".format(pft_info_json))
                else:
                    dest = ""

            # file cannot be downloaded with PFT
            elif (flash_files_entry["local_key"] and not flash_files_entry["pft_key"]) or direct_download_allowed:
                key = flash_files_entry["local_key"]
                if not key:
                    key = flash_files_entry["pft_key"]
                self.__logger.printLog("WARNING", "artifactory direct download for {}".format(key))
                dest = ""
                pft_info_json = os.path.join(target_dir, key + ".json")
                for builder in alternative_artifactory_list:
                    if flash_files_entry.get("{0}_artifactory_path".format(builder), {})\
                            and isinstance(flash_files_entry["{0}_artifactory_path".format(builder)], dict)\
                            and flash_files_entry["{0}_artifactory_path".format(builder)].get("detection", "") in \
                                    url_buildinfo:
                        self.__logger.printLog("INFO", "trying '{0}' builder direct download".format(builder))
                        relative_artifactory_path = \
                            flash_files_entry["{0}_artifactory_path".format(builder)].get("path")
                        artifactory_path = url_buildinfo.split("build_info")[0] + relative_artifactory_path
                        dest = ""
                        status, artifactory_output = self.__host.commandExecArtifactorySecureContent(artifactory_path)
                        if status == 0:
                            regexp = "href=\"({0})\"".format(flash_files_entry["regexp_key"])
                            search = re.search(regexp, artifactory_output)
                            if not search:
                                self.__logger.printLog("WARNING", "not possible to get matching '{0}' flash file from: {1}".format(regexp, artifactory_path))
                                output_issue.append(("not possible to get matching '{0}' flash file from: {1}".format(regexp, artifactory_path)))
                            else:
                                if not artifactory_path.endswith("/"):
                                    artifactory_path += "/"
                                download_file = artifactory_path + search.group(1)
                                self.__logger.printLog("DEBUG", "file matching '{0}': {1}"
                                                       .format(regexp,
                                                               download_file))
                                self.__host.commandExecArtifactorySecureDownload(download_file, target_dir, timeout=1000)
                                destination_file = os.path.join(target_dir, search.group(1))
                                if not os.path.isfile(destination_file):
                                    self.__logger.printLog("INFO", "downloaded file not found, issue with download ({0})".format(destination_file))
                                    output_issue.append("downloaded file not found, issue with download ({0})".format(destination_file))
                                else:
                                    self.__flashFile.zipFlashFile2PftJsonFile(destination_file, pft_info_json,
                                                                              key,
                                                                              target_flash_file=flash_files_entry["flash_file"],
                                                                              target_configuration=flash_files_entry["configuration"],
                                                                              groups=flash_files_entry["groups"],
                                                                              start_state=flash_files_entry["start_state"])
                                    if os.path.isfile(pft_info_json):
                                        dest = pft_info_json
                                        break
                                    else:
                                        output = "failure to generate PFT info file from zip file"
                                        self.__logger.printLog("WARNING", output)
                                        output_issue.append(output)
                        else:
                            self.__logger.printLog("WARNING", "artifactory content does not exist: {0}".format(artifactory_path))
                            output_issue.append("artifactory content does not exist: {0}".format(artifactory_path))
                    else:
                        tmp_output = "invalid {}_artifactory_path input ({})".format(
                                builder,
                                flash_files_entry.get("{0}_artifactory_path".format(builder)))
                        self.__logger.printLog("DEBUG", tmp_output)
            else:
                self.__logger.printLog("DEBUG", log + "empty download entry: {0} (skipping)".format(flash_files_entry))
                dest = ""

            # Create key/value in build dict
            if dest:
                build.append(dest)
            elif "ota" in [flash_files_entry["local_key"], flash_files_entry["pft_key"]]:
                self.__logger.printLog("DEBUG", log + "non-critical download error: no OTA file found")
            elif flash_files_entry["pft_key"] == "" and flash_files_entry["local_key"] == "":
                self.__logger.printLog("DEBUG", log + "non-critical download error: empty download element")
            else:
                verdict = False
            if os.path.isfile(dest):
                self.__logger.printLog("DEBUG", log + "adding file {0}".format(dest))

        # IFWI download if needed
        if isinstance(self.__configuration.flash.DEDIPROG, dict) and \
            not any(element not in self.__configuration.flash.DEDIPROG
                    for element in ["stage1_spi_location", "stage2_emmc_location"]):
            for element in ["stage1_spi_location", "stage2_emmc_location"]:
                ifwi_stage = element.replace("_location", "")
                self.__logger.printLog("INFO", log + "downloading {0}".format(ifwi_stage.replace("_", " ").capitalize()))
                relative_artifactory_path = "/".join(self.__configuration.flash.DEDIPROG[element].split("/")[0:-1]).replace("device_id", device)
                regexp = self.__configuration.flash.DEDIPROG[element].split("/")[-1].replace("variant_id", buildvariant.split("-")[1])
                regexp = "href=\"({0})\"".format(regexp)
                artifactory_path = buildinfo_link + "/" + relative_artifactory_path
                status, artifactory_output = self.__host.commandExecArtifactorySecureContent(artifactory_path)
                if status == 0:
                    search = re.search(regexp, artifactory_output)
                    if not search:
                        self.__logger.printLog("WARNING", log + "not possible to get matching '{0}' flash file from: {1}".format(regexp, artifactory_path))
                    else:
                        if not artifactory_path.endswith("/"):
                            artifactory_path += "/"
                        download_file = artifactory_path + search.group(1)
                        self.__logger.printLog("DEBUG", "file matching '{0}': {1}"
                                               .format(regexp, artifactory_path))
                        self.__host.commandExecArtifactorySecureDownload(download_file, target_dir, timeout=1000)
                        destination_file = os.path.join(target_dir, search.group(1))
                        if not os.path.isfile(destination_file):
                            self.__logger.printLog("WARNING", log + "failure with ifwi download {0}".format(download_file))
                        else:
                            self.__logger.printLog("INFO", log + "{0} successfully downloaded".format(ifwi_stage))
                            copy_file = os.path.join(target_dir, ifwi_stage + ".bin")
                            if os.path.exists(copy_file):
                                os.remove(copy_file)
                            shutil.copy(destination_file, copy_file)
                else:
                    self.__logger.printLog("WARNING", log + "artifactory content does not exist: {0}".format(artifactory_path))

        if not verdict:
            output_log =  "download failure"
            build = list()
            if output_issue:
                output_log += ": {0}".format(", ".join(output_issue))
        else:
            output_log = ""

        self.__output.appendOutput(output_log, verdict)

        return build

    def selectRefWeeklyBuild(self, tag_but, buildvariant):
        log = "selectRefWeeklyBuild({0}/{1}): ".format(self.__configuration.board, buildvariant)
        self.__logger.printLog("INFO", log + "starting for {0} build".format(tag_but))

        # erase previously written output json file
        if not self.__logs.bootota_log_path:
            pft_output_file = os.path.join(tempfile.gettempdir(), "pft_output_{}.json".format(self.__misc.getUser()))
        else:
            pft_output_file = os.path.join(self.__logs.bootota_log_path, "pft_output.json")
        if os.path.isfile(pft_output_file):
            os.remove(pft_output_file)
        self.__misc.local_files_handler.addEntry(pft_output_file)

        # launch pft to get buildbot metadata json
        branch = self.getBranchFromTag(tag_but)
        cmd = "--search --non-interactive " \
              "--property buildbot.props.branch_name={0} " \
              "--property buildbot.props.buildername=*-{2} " \
              "--property buildbot.props.upload-weekly-branch=true " \
              "--search-output {1}" \
              .format(branch, pft_output_file, self.__configuration.download.RELEASE_BUILDER)
        self.__host.commandExecCflasher(cmd, 400)

        if not os.path.isfile(pft_output_file):
            # retry PFT command
            self.__logger.printLog("WARNING", "PFT command failure, retrying")
            self.__host.commandExecCflasher(cmd, 400)
            if not os.path.isfile(pft_output_file):
                self.__logger.printLog("WARNING", "PFT command failure, cannot retrieve buildbot data")
                return "-1"

        # try to read pft output json. exit when failed reading file
        try:
            f = open(pft_output_file)
            data = json.load(f)
            f.close()
            os.remove(pft_output_file)
        except Exception as e:
            os.remove(pft_output_file)
            self.__logger.printLog("WARNING", "could not read json file '{0}' ({1})".format(pft_output_file, e))
            return "-1"

        if "results" not in data:
            self.__logger.printLog("WARNING", "missing 'results' key in '{0}'".format(pft_output_file))
            return "-1"

        # parse results and find all proposed weekly build
        WW_list_names = list()
        for index, element in enumerate(data["results"]):
            if "properties" in element and \
                not any(entry not in element["properties"] for entry in ("buildbot.props.release_tag", "buildbot.props.short_build_number"))\
                and "downloadUri" in element:
                weekly_tag = element["properties"]["buildbot.props.release_tag"][0]
                # disregard intermediate weeklies (like 14WW01B)
                if not re.search(".*[A-Z]$", weekly_tag) and \
                    element["properties"]["buildbot.props.short_build_number"][0] not in [weekly[2] for weekly in WW_list_names]:
                    WW_list_names.append([index,
                                          weekly_tag,
                                          element["properties"]["buildbot.props.short_build_number"][0],
                                          element["downloadUri"]])

        def getKey(item):
            return item[1]

        WW_list_names = sorted(WW_list_names, key=getKey)
        if len(WW_list_names) == 0:
            self.__logger.printLog("WARNING", "no build found in '{0}':\n{1}".format(pft_output_file, data))
            return "-1"
        else:
            self.__logger.printLog("INFO", "names of WW found: {0}".format(", ".join([entry[1] for entry in WW_list_names])))

        # for weekly build or when only one build found, select last build, else the previous one
        if len(WW_list_names) == 1:
            selected_index = len(WW_list_names) - 1
        elif self.getBuilderFromTag(tag_but) == self.__configuration.download.RELEASE_BUILDER:
            selected_index = None
            for index, entry in enumerate(sorted(WW_list_names, key=getKey, reverse=True)):
                # find first tag older that BUT tag
                if "-" in entry[2]:
                    current_entry_number = entry[2].split("-")[-1]
                    but_number = tag_but.split("-")[-1]
                else:
                    search = re.search("[0-9]+$", entry[2])
                    if search:
                        current_entry_number = search.group(0)
                    search = re.search("[0-9]+$", tag_but)
                    if search:
                        but_number = search.group(0)
                try:
                    int(current_entry_number)
                    int(but_number)
                except Exception as e:
                    self.__logger.printLog("WARNING", log + "unexpected build tags: '{0}' and '{1}' "
                                                            "(error={2})".format(entry[2], tag_but, e))
                    continue
                if int(current_entry_number) < int(but_number):
                    selected_index = len(WW_list_names) - index - 1
                    self.__logger.printLog("INFO", log + "anterior build found, {0} < {1}".format(entry[2], tag_but))
                    break
                else:
                    self.__logger.printLog("INFO", log + "anterior build not found yet, {0} >= {1}".format(entry[2], tag_but))
            # if nothing found, fall back to first element
            if selected_index is None:
                selected_index = len(WW_list_names) - 1
        else:
            selected_index = len(WW_list_names) - 2

        if WW_list_names[selected_index][1] in self.__configuration.download.REFERENCE_BUILD_BLACKLIST:
            workaround = self.__configuration.download.REFERENCE_BUILD_BLACKLIST[WW_list_names[selected_index][1]]
            self.__logger.printLog("DEBUG", log + "blacklisted build: {} "
                                                  "(applying workaround: {})".format(WW_list_names[selected_index][1],
                                                                                     workaround))
            if "http" in workaround:
                if not workaround.endswith(".json"):
                    workaround += "/build_info.json"
                WW_list_names.append([-1, "CUSTOM",
                                      "unknown",
                                      self.cleanUrl(workaround)])
                selected_index = len(WW_list_names) - 1
            elif workaround in [ww[1] for ww in WW_list_names]:
                for index, ww in enumerate(WW_list_names):
                    if ww[1] == workaround:
                        selected_index = index
                        break
        else:
            # checking board type
            original_index = selected_index
            isBoardTypeInBuildInfo = False
            if not self.__logs.bootota_log_path:
                local_build_info = os.path.join(tempfile.gettempdir(), "build_info_{}.json".format(self.__misc.getUser()))
            else:
                local_build_info = os.path.join(self.__logs.bootota_log_path, "build_info.json")
            self.__misc.local_files_handler.addEntry(local_build_info)
            while selected_index >= 0 and not isBoardTypeInBuildInfo:
                # download build_info.json
                if os.path.isfile(local_build_info):
                    os.remove(local_build_info)
                self.__host.commandExecArtifactorySecureDownload(WW_list_names[selected_index][3],
                                                                 os.path.dirname(local_build_info),
                                                                 timeout=1000,
                                                                 file_name=os.path.basename(local_build_info))
                if not os.path.isfile(local_build_info):
                    self.__logger.printLog("WARNING", log + "issue with build_info.json download, trying previous weekly ({0} not found)".format(local_build_info))
                    selected_index -= 1
                    continue
                try:
                    with open(local_build_info) as f:
                        json_data = json.load(f)
                    os.remove(local_build_info)
                except Exception as e:
                    self.__logger.printLog("WARNING", log + "issue with build_info.json reading and parsing, "
                                                            "trying previous weekly ({0})".format(e))
                    selected_index -= 1
                    os.remove(local_build_info)
                    continue
                if "hardwares" in json_data:
                    available_hw = [f for f in json_data["hardwares"]]
                else:
                    available_hw = list()
                if "hardwares" not in json_data or self.__configuration.board not in json_data["hardwares"]:
                    self.__logger.printLog("WARNING", log + "'{0}' board type not found among available hardwares ({1}), "
                                                            "trying previous weekly".format(self.__configuration.board,
                                                                                            ", ".join(available_hw)))
                    selected_index -= 1
                elif "variants" not in json_data["hardwares"][self.__configuration.board] \
                    or buildvariant not in json_data["hardwares"][self.__configuration.board]["variants"]:
                    available_bv = [f for f in json_data["hardwares"][self.__configuration.board]["variants"]]
                    self.__logger.printLog("WARNING", log +"'{0}' buildvariant not found among available buildvariants ({1}), "
                                                            "trying previous weekly".format(buildvariant,
                                                                                            ", ".join(available_bv)))
                    selected_index -= 1
                else:
                    isBoardTypeInBuildInfo = True

            if not isBoardTypeInBuildInfo:
                self.__logger.printLog("WARNING", log + "board type '{0}' not found in any build_info.json, fallback to original one")
            elif selected_index != original_index:
                self.__logger.printLog("DEBUG", log + "switching from {0} to {1}".format(WW_list_names[original_index][1],
                                                                                         WW_list_names[selected_index][1]))

        selected_WW = WW_list_names[selected_index]
        self.__logger.printLog("INFO", "WW selected for download = {0}".format(selected_WW))

        # get build_info.json url
        url_buildinfo = self.cleanUrl(selected_WW[3])

        self.__logger.printLog("INFO", log + "build_info.json found '{0}'".format(url_buildinfo))

        return url_buildinfo

    def cleanUrl(self, url):
        original = url
        url = url.replace("jfstor001.jf.intel.com", "mcg-depot.intel.com")
        url = url.replace("jfstor002.jf.intel.com", "mcg-depot.intel.com")
        url = url.replace("shstor001.sh.intel.com", "mcg-depot.intel.com")
        url = url.replace("tlsstor001.tl.intel.com", "mcg-depot.intel.com")
        # url = url.replace("cactus-absp-jf1", "cactus-absp")
        # url = url.replace("cactus-absp-jf", "cactus-absp")
        # url = url.replace("cactus-absp-sh", "cactus-absp")
        # url = url.replace("cactus-absp-tl", "cactus-absp")
        self.__logger.printLog("DEBUG", "cleanUrl({}) = {}".format(original, url))
        return url

    # createDictionary()
    # path         : local working directory path
    # path_pft     : local pft download directory path
    # tag_but      : build version tag of but (ie main-weekly-50) - needed for ref download only
    # board_type   : board_type from slave_configuration.xml
    # buildvariant : full name of buildvariant
    # build_type   : USER, REF or BUT
    def createDictionary(self, path, tag_but, board_type, buildvariant, build_type, ota_file_needed=False,
                         optional_url="", local=False, device=""):
        """ Returns ref dictionary that contains flashfiles keys with corresponding paths
        """
        log = "createDictionary(): "
        download_needed = False
        url_buildinfo = ""

        if not device and "buildvariant" in self.provisioning_data:
            device = self.provisioning_data["buildvariant"].split("-")[0]

        if not build_type:
            if os.path.basename(os.path.dirname(path)) == "FOTA":
                build_type = os.path.basename(os.path.dirname(path)) + "/" + os.path.basename(path)
            else:
                build_type = os.path.basename(path)
        self.__logger.printLog("INFO", log + "starting for {0}".format(build_type))
        function_args = "build={}".format(build_type)

        if not board_type:
            self.__logger.printLog("DEBUG", log + "board type not specified, assuming '{0}'".format(self.__configuration.board))
            board_type = self.__configuration.board

        if buildvariant:
            variant = buildvariant.split("-")[-1]
        else:
            variant = ""

        if build_type == "EXTERNAL":
            file_type_list = self.__configuration.download.EXTERNAL_DOWNLOAD
        elif variant == "user" and self.__configuration.download.USER_DOWNLOAD:
            file_type_list = self.__configuration.download.USER_DOWNLOAD
        else:
            file_type_list = self.__configuration.download.INTERNAL_DOWNLOAD

        download_data = self.__misc.downloadConfiguration2dictionary(
            file_type_list, ota=ota_file_needed,
            board_type=board_type,
            device=device,
            variant=variant,
            multi_variant_prefix=self.__configuration.download.MULTI_VARIANT_PREFIX)

        if not local:
            if optional_url:
                url_buildinfo = optional_url
            elif "url_buildinfo" in self.provisioning_data and self.provisioning_data["url_buildinfo"]:
                url_buildinfo = self.provisioning_data["url_buildinfo"]
            else:
                self.__logger.printLog("DEBUG", log + "no URL provided for download")

            # Determine if download is needed or files are already there
            if (any(single_type in build_type for single_type in ["BUT", "EXTERNAL", "D2D"]) and url_buildinfo) \
                or build_type in ["REF"]:
                if not os.path.isdir(path) or not os.listdir(path) or \
                    (ota_file_needed and not "ota" in ";".join(os.listdir(path))):
                        # only download is files are not already downloaded
                        download_needed = True

            # Dynamically get url of ref in list of official weekly builds
            if build_type == "REF" and download_needed:
                if optional_url:
                    url_buildinfo = optional_url
                elif tag_but:
                    url_buildinfo = self.selectRefWeeklyBuild(tag_but, buildvariant)
                else:
                    self.__logger.printLog("WARNING", log + "cannot compute reference build URL")
                    url_buildinfo = "-1"
                # If ref url cannot be computed
                if url_buildinfo == "-1":
                    self.__output.appendOutput("cannot compute reference build URL", False, argument=function_args)
                    return {}

            if  "FOTA" in build_type and "fastboot" not in ";".join(os.listdir(path)):
                download_needed = True

        local_build = list()

        # If url is valid, proceed to download
        if download_needed:
            self.__logger.printLog("INFO", "Get build files from Artifactory")
            if not url_buildinfo:
                self.__output.appendOutput("no provided URL", False, argument=function_args)
                return {}
            local_build = self.getFlashfiles(url_buildinfo,
                                             board_type,
                                             os.path.join(self.__globalConf.get("WORKDIR"), build_type),
                                             buildvariant,
                                             download_data,
                                             device=device)
            if not local_build:
                self.__output.appendOutput("failure to get {0} files".format(build_type), False, argument=function_args)
                return {}
            if DOWNLOAD_RMA_TOOLS:
                return {}
        # If files already there
        elif os.path.isdir(path):
            # check presence of json files first
            for file_type in download_data:
                if not ((file_type["local_key"] == "ota" or file_type["pft_key"] == "ota") and not ota_file_needed):
                    if file_type["local_key"]:
                        json_file = os.path.join(path, file_type["local_key"]+".json")
                    elif file_type["flash_list_processing"]:
                        self.__logger.printLog("INFO", log + "processing input, no associated flash files, skipping")
                        continue
                    else:
                        json_file = os.path.join(path, file_type["pft_key"]+".json")
                    if os.path.isfile(json_file):
                        local_build.append(json_file)
                    elif not file_type["regexp_key"]:
                        self.__output.appendOutput("empty regexp, cannot find file related to {}".format(file_type["local_key"] if file_type["local_key"] else file_type["pft_key"]), False, argument=function_args)
                        return {}
                    else:
                        self.__logger.printLog("INFO", "Check presence of '{0}' files in '{1}'".format(file_type["regexp_key"], path))
                        files = self.__misc.getPathFile(file_type["regexp_key"], path)
                        if not files:
                            if file_type["local_key"] != "ota":
                                self.__output.appendOutput("no file matching '{0}' in {1}".format(file_type["regexp_key"], path), False, argument=function_args)
                                return {}
                            else:
                                self.__logger.printLog("INFO", log + "OTA file missing, skipping")
                        elif len(files) != 1:
                            self.__output.appendOutput("too many files matching '{0}' in {1}".format(file_type["regexp_key"], files), False, argument=function_args)
                            return {}
                        else:
                            self.__flashFile.zipFlashFile2PftJsonFile(files[0], json_file, file_type["local_key"],
                                                                      target_flash_file=file_type["flash_file"],
                                                                      target_configuration=file_type["configuration"],
                                                                      start_state=file_type["start_state"])
                            local_build.append(json_file)
        else:
            self.__output.appendOutput("{0} does not exist (files or url must be provided)".format(path), False, argument=function_args)
            return  {}

        # build output dictionary
        output = dict()
        output["flash_list"] = self.__flashFile.flashFileList2DictionaryList(local_build)
        output["original_list"] = local_build

        # check if flash list processing needed
        if download_data[-1].get("flash_list_processing"):
            processing_script = download_data[-1]["flash_list_processing"].split(":")[0]
            if len(download_data[-1]["flash_list_processing"].split(":")) == 2:
                extra_args = download_data[-1]["flash_list_processing"].split(":")[1].split(",")
            else:
                extra_args = []
            try:
                if extra_args:
                    local_verdict = \
                        eval("self.{}(output['flash_list'], {})".format(
                                processing_script,
                                ", ".join(extra_args)
                        ))
                else:
                    local_verdict = \
                        eval("self.{}(output['flash_list'])".format(processing_script))
            except Exception as e:
                local_output = "failure with '{}' flash list processing " \
                               "(error={})".format(download_data[-1]["flash_list_processing"], e)
                self.__output.appendOutput(local_output, False, argument=function_args)
                return {}
            else:
                if not local_verdict:
                    local_output = "failure while executing '{}'".format(download_data[-1]["flash_list_processing"])
                    self.__output.appendOutput(local_output, False, argument=function_args)
                    return {}

        # add complementary elements:
        output["ssn"] = self.__device.getProperty("ro.serialno")
        # if not found, search in flash file name
        tag = ""
        tag_regexp = ""
        tag_group = 0
        for file_type in download_data:
            if file_type["tag_regexp"]:
                tag_regexp = file_type["tag_regexp"]
                tag_group = file_type.get("tag_group", 1)
        for f in output["flash_list"]:
            local_flash_file = self.__flashFile.createFile(f)
            tag = self.getBuildTag(os.path.basename(local_flash_file.get_localFlashFile()),
                                   extra_regexp=tag_regexp, extra_group=tag_group)
            if tag:
                self.__logger.printLog("INFO", log + "'{0}' tag found in flash file name".format(tag))
                break
        if tag == "" or tag == "notag":
            # check tag in latest flash file xml
            fastboot_index = None
            for index, flash_file in enumerate(output["flash_list"]):
                if "flash_file" in flash_file and flash_file["file_type"] == "fastboot":
                    fastboot_index = index
                    break
            if fastboot_index:
                local_flash_file = self.__flashFile.createFile(output["flash_list"][fastboot_index])
            else:
                local_flash_file = self.__flashFile.createFile(output["flash_list"][-1])
            tag = self.__flash.getVersionFromXml(zip_file=local_flash_file.get_localFlashFile(),
                                                 flash_xml=local_flash_file.get_pft_flash_file())
        output["tag"] = tag

        # Convert dictionary element into string
        self.__logger.printLog("INFO", "BUILD {0} =\n{1}".format(build_type, json.dumps(output, sort_keys=True, indent=4)))
        self.__output.appendOutput("", True, argument=function_args)
        return output

    def buildUserdebugReprovisionFlashFile(self, flash_list):
        log = "buildUserdebugReprovisionFlashFileStep(): "
        self.__logger.printLog("INFO", log + "start")
        local_flash_file = self.__flashFile.createFile(flash_list[0])
        osloaders = self.__flash.getZipContentList(local_flash_file.get_localFlashFile(), "loader\.efi")
        fastboots = self.__flash.getZipContentList(local_flash_file.get_localFlashFile(), self.__configuration.flash.FASTBOOT_FILE)
        if not osloaders:
            output = "could not find 'loader.efi' element in flash file"
            verdict = False
        elif not fastboots:
            output = "could not find '{}' element in flash file".format(self.__configuration.flash.FASTBOOT_FILE)
            verdict = False
        else:
            osloader = osloaders[0]
            fastboot = fastboots[0]
            pre_flash_json = os.path.join(self.__globalConf.get("WORKDIR"), "BUT", "userdebug_pre_flash.json")
            self.__flashFile.buildUserdebugReprovisionFlashFile(osloader, fastboot, pre_flash_json)
            flash_list.insert(0, self.__flashFile.flashFileList2DictionaryList([pre_flash_json])[0])
            flash_list.pop(1)
            output = ""
            verdict = True
        self.__output.appendOutput(output, verdict)
        return verdict

    def oc6ToOtcFlashFile(self, flash_list, url):
        log = "oc6ToOtcFlashFile(): "
        self.__logger.printLog("INFO", log + "start")
        verdict = True
        output = ""
        local_flash_file = self.__flashFile.createFile(flash_list[0])
        search = re.search("_([0-9]+_[0-9]_[0-9]+)\\.", url)
        if search:
            tag = search.group(1)
        else:
            tag = ""
        name = "flashfiles-" + tag
        directory_name = os.path.abspath(os.path.join(self.__globalConf.get("WORKDIR"), "BUT", name))
        flashfile_json = os.path.abspath(os.path.join(self.__globalConf.get("WORKDIR"), "BUT", "fastboot.json"))
        ota_json = os.path.abspath(os.path.join(self.__globalConf.get("WORKDIR"), "BUT", "ota.json"))
        if os.path.isdir(directory_name):
            shutil.rmtree(directory_name)
        os.makedirs(directory_name)
        zipname = directory_name + ".zip"

        # copy flashfile
        original_json = os.path.abspath(os.path.join(self.__misc.extra_path, "flashSofiaOc6.json"))
        copy_json = os.path.join(directory_name, "flash.json")
        shutil.copy(original_json, copy_json)

        # copy fls
        fls_list = {
            "boot": {},
            "cache": {},
            "fwu_image": {},
            "mobilevisor": {},
            "mvconfig_smp": {},
            "psi_flash": {},
            "recovery": {},
            "secvm": {},
            "slb": {},
            "splash_img": {},
            "system": {},
            "ucode_patch": {},
            "userdata": {}
        }
        fls_directory = ""
        for fls in fls_list:
            original_fls_list = self.__flash.getZipContentList(local_flash_file.get_localFlashFile(), fls + "_signed.fls")
            if len(original_fls_list) == 1:
                fls_directory = os.path.dirname(original_fls_list[0])
                self.__logger.printLog("DEBUG", log + "found fls path = {}".format(fls_directory))
                break
        for fls in fls_list:
            original_fls_list = self.__flash.getDirectoryContentList(fls_directory, fls + "_signed.fls", 0, recursive=False)
            if not original_fls_list:
                output = "failure to find {}.fls in OC6 build".format(fls)
                self.__output.appendOutput(output, False)
                return False
            fls_copy = os.path.join(directory_name, fls + "_signed.fls")
            shutil.copy(original_fls_list[0], fls_copy)

        self.__misc.zipDirectory(zipname, directory_name)

        # fill dictionary
        output_dict = dict()
        output_dict["flashConfigurationStartState"] = "dnx"
        output_dict["flashFileName"] = os.path.basename(copy_json)
        output_dict["flashConfiguration"] = "smp_fls_config"
        output_dict["localFlashFile"] = zipname

        # write file
        try:
            with open(flashfile_json, 'wb') as outfile:
                json.dump(output_dict, outfile)
        except Exception as e:
            output = "failure to write json file: {0} (error={1})".format(flashfile_json, e)
            self.__output.appendOutput(output, False)
            return False

        flash_list.insert(0, self.__flashFile.flashFileList2DictionaryList([flashfile_json])[0])
        flash_list.pop(1)

        # ota
        ota_list = self.__flash.getZipContentList(local_flash_file.get_localFlashFile(), ".*ota.*.zip")
        if ota_list:
            self.__flashFile.zipFlashFile2PftJsonFile(
                    ota_list[0],
                    ota_json,
                    "ota",
                    start_state="mos")
            flash_list.append(self.__flashFile.flashFileList2DictionaryList([ota_json])[0])

        self.__output.appendOutput(output, verdict)
        return verdict

    def __geIndexD2D(self):
        return_index = -1
        search = re.search(".*_([0-9]+)$", self.__logs.tc_name_number)
        if search:
            try:
                return_index = int(search.group(1))
            except Exception as e:
                self.__logger.printLog("INFO", "geIndexD2D():failure to compute TC index from "
                                               "{0} ({1})".format(self.__logs.tc_name_number, e))
                return_index = -1
        return return_index

    # dessert to dessert download
    def __createD2DDict(self):
        log = "createD2DDict(): "
        D2D = dict()
        D2D["source"] = dict()
        D2D["destination"] = dict()
        D2D["output_log"] = ""
        output_list = list()
        path_D2D = os.path.join(self.__globalConf.get("WORKDIR"), "D2D")
        self.__logger.printLog("INFO", log + "start")
        if os.path.isdir(path_D2D):
            shutil.rmtree(path_D2D)
        os.makedirs(path_D2D)

        if not self.__configuration.download.DESSERT_TO_DESSERT_OTA:
            output = "DESSERT_TO_DESSERT_OTA configuration parameter not defined"
            self.__output.appendOutput(output, True)
            D2D["output_log"] = output
            return True, D2D

        download_data = self.__misc.xmlD2DDownloadConfiguration2dictionary(self.__configuration.download.DESSERT_TO_DESSERT_OTA)

        origin_config_name = "BUT"
        self.__configuration.storeCurrentConfig(origin_config_name)

        current_iteration = self.__geIndexD2D()
        if 0 <= current_iteration < len(download_data):
            download_data = [download_data[current_iteration]]
            self.__logger.printLog("DEBUG", log + "iteration {0} = {1}".format(current_iteration, download_data))
        else:
            download_data = {}
            self.__logger.printLog("DEBUG", log + "invalid index ({0}), no more download needed".format(current_iteration))

        if not download_data:
            output = "No more D2D flashing to process"
            self.__output.appendOutput(output, True)
            D2D["output_log"] = output
            return True, D2D

        for flash_file in download_data:

            origin_branch = flash_file["build"].split("-")[0]
            origin_ww = flash_file["build"].split("-")[1]
            origin_board_type = flash_file["fallback_board"] if flash_file["fallback_board"] else self.__configuration.board
            origin_device = flash_file["fallback_device"] if flash_file["fallback_device"] else self.provisioning_data["buildvariant"].split("-")[0]
            origin_buildvariant = origin_device + "-" + self.provisioning_data["buildvariant"].split("-")[1]
            update_name = flash_file["build"] + "-" + self.build_but["tag"]
            source_directory = update_name + "_source"
            destination_directory = update_name + "_destination"
            source_path = os.path.join(path_D2D, source_directory)
            destination_path = os.path.join(path_D2D, destination_directory)
            artifactory_link = "https://mcg-depot.intel.com/artifactory/cactus-absp/build/eng-builds/{0}/PSI/weekly/{1}/build_info.json".format(origin_branch, origin_ww)
            D2D[update_name] = dict()

            # update configuration
            self.__configuration.updateConfig(origin_branch, origin_board_type, config_name=update_name)

            if os.path.isdir(source_path):
                shutil.rmtree(source_path)
            os.makedirs(source_path)
            if os.path.isdir(destination_path):
                shutil.rmtree(destination_path)
            os.makedirs(destination_path)

            D2D["source"] = self.createDictionary(source_path,
                                                  "",
                                                  origin_board_type,
                                                  origin_buildvariant,
                                                  os.path.join("D2D", source_directory),
                                                  False,
                                                  optional_url=artifactory_link,
                                                  device=origin_device)
            if not D2D["source"]:
                output_list.append(update_name + " - failure to download source files")
                continue

            # download BUT:
            ota_flash_file = self.__configuration.flash.FULL_OTA_LOCAL_FLASH_FILE
            self.__configuration.switchConfig(origin_config_name)
            if self.__configuration.flash.FULL_OTA_LOCAL_FLASH_FILE and ota_flash_file:
                self.__logger.printLog("INFO", "overriding '{}' FULL_OTA_LOCAL_FLASH_FILE configuration "
                                               "parameter with '{}'".format(
                    self.__configuration.flash.FULL_OTA_LOCAL_FLASH_FILE,
                    ota_flash_file))
                self.__configuration.flash.FULL_OTA_LOCAL_FLASH_FILE = ota_flash_file
            D2D["destination"] = self.createDictionary(destination_path,
                                                       "",
                                                       self.provisioning_data["board_type"],
                                                       self.provisioning_data["buildvariant"],
                                                       os.path.join("D2D", destination_directory),
                                                       True)
            if not D2D["destination"]:
                output_list.append(update_name + " - failure to download destination files")
                continue

            D2D["source"]["pre_flash_configuration"] = update_name
            D2D["destination"]["update_name"] = update_name
            D2D["destination"]["change_configuration"] = origin_config_name

        # reset original configuration
        self.__configuration.switchConfig(origin_config_name)

        # check if dictionary complete, if not, invalidate all packages
        if output_list:
            output = ", ".join(output_list)
            verdict = False
        else:
            output = ""
            verdict = True
        self.__output.appendOutput(output, verdict)
        return verdict, D2D

    # FOTA specific method
    def __createFotaDict(self, buildvariant):
        self.__logger.printLog("INFO", "createFotaDict({0}): start".format(buildvariant))
        fota_url_prod = "http://fota.tl.intel.com/ota_packages/r3"
        fota_url_test = fota_url_prod + "/test"
        fota_url_external = "https://fota.intel.com/api/ota1/"
        artifactory_ota_url = self.provisioning_data["url_buildinfo"].replace( "build_info.json", "ota")
        fota_url = ""
        fota_html_external = ""
        artifactory_html_content = ""
        sh = 0

        self.__path_FOTA = os.path.join(self.__globalConf.get("WORKDIR"), "FOTA")

        # Check if local FOTA folder is already there and not empty
        ota_list = list()
        local = False
        if os.path.isdir(self.__path_FOTA) and os.listdir(self.__path_FOTA):
            local = True
            listdir = os.listdir(self.__path_FOTA)
            self.__logger.printLog("INFO", "FOTA folder is already there")
            for ota in listdir:
                if os.path.isdir(os.path.join(self.__path_FOTA, ota)) and "ota" in ota:
                    ota_list.append(ota + ".zip")

        # Else create FOTA
        else:
            if os.path.isdir(self.__path_FOTA):
                shutil.rmtree(self.__path_FOTA)
            os.makedirs(self.__path_FOTA)

            try:
                urllib2.urlopen(fota_url_external).read()
            except Exception as e:
                self.__logger.printLog("WARNING", "use proxy in shanghai,China")
                sh = 1

            # Get list of matching OTA
            try:
                fota_html_prod = urllib2.urlopen(fota_url_prod).read()
            except Exception as e:
                self.__logger.printLog("WARNING", "Prod Server unreachable: {0} ({1})".format(fota_url_prod, e))
                fota_html_prod = ""

            try:
                fota_html_test = urllib2.urlopen(fota_url_test).read()
            except Exception as e:
                self.__logger.printLog("WARNING", "Test Server unreachable: {0} ({1})".format(fota_url_test, e))
                fota_html_test = ""

            try:
                exec_status, fota_html_external = self.__host.commandExecExternalFotaSecureContent(fota_url_external, sh)
            except Exception as e:
                self.__logger.printLog("WARNING", "External Server unreachable: {0} ({1})".format(fota_url_external, e))
                exec_status = 1
                fota_html_external = ""
            if exec_status != 0:
                fota_html_external = ""

            try:
                exec_status, artifactory_html_content = self.__host.commandExecArtifactorySecureContent(artifactory_ota_url)
            except Exception as e:
                self.__logger.printLog("WARNING", "Artifactory unreachable: {0} ({1})".format(artifactory_ota_url, e))
                exec_status = 1
                artifactory_html_content = ""
            if exec_status != 0:
                artifactory_html_content = ""

            if fota_html_prod == "" and fota_html_test == "" and fota_html_external == "" and \
                            artifactory_html_content == "":
                output = "no reachable FOTA Servers: {0}, {1}".format(fota_url_prod, fota_url_test)
                self.__output.appendOutput(output, False)
                return False, {}

            tag = self.build_but["tag"]
            pattern = buildvariant.split("-")[0] + "[^_.]*-.*" + tag + ".zip"
            fota_list_prod = re.findall(pattern, fota_html_prod, re.I)
            fota_list_test = re.findall(pattern, fota_html_test, re.I)
            # old names: cht_cr_rvp-ota-cht51-week-103_userdebug-cht51-week-105_userdebug.zip
            pattern_external = buildvariant.split("-")[0] + "-ota-.*" + tag + "_" + buildvariant.split("-")[1] + ".zip"
            fota_list_external = re.findall(pattern_external, fota_html_external, re.I)
            artifactory_pattern = buildvariant.split("-")[0] + "-ota-[a-zA-Z-_0-9]+" + buildvariant.split("-")[1] + "-" + tag + ".zip"
            fota_list_artifactory = re.findall(artifactory_pattern, artifactory_html_content, re.I)
            fota_list_artifactory = set(fota_list_artifactory)
            # new names: cht_ffd-ota-cht_ffd-userdebug-L1c000011-cht_ffd-userdebug-L10000014.zip
            if not fota_list_external:
                updated_pattern = buildvariant.split("-")[0] + "-ota-.*" + buildvariant.split("-")[1] + "-" + tag + ".zip"
                fota_list_external = re.findall(updated_pattern, fota_html_external, re.I)

            # PROD has priority
            if fota_list_prod:
                self.__logger.printLog("INFO", "Use incremental fota from PROD !!")
                ota_list = fota_list_prod
                fota_url = fota_url_prod
            # pupdr.TEST
            elif fota_list_test:
                self.__logger.printLog("INFO", "Use incremental fota from TEST !!")
                ota_list = fota_list_test
                fota_url = fota_url_test
            elif fota_list_artifactory:
                self.__logger.printLog("INFO", "Use incremental fota from Artifactory !!")
                ota_list = fota_list_artifactory
                fota_url = artifactory_ota_url
            else:
                self.__logger.printLog("INFO", "Use incremental fota from External Fota link !")
                ota_list = fota_list_external
                fota_url = fota_url_external

        self.__logger.printLog("INFO", "list of ota: {0}".format(ota_list))

        FOTA = dict()

        #
        # For each matching ota incremental:
        # - Download ota incremental
        # - Download build source blankphone / fastboot
        #
        for ota in ota_list:
            self.__logger.printLog("INFO", "manage ota incremental: {0}".format(ota))
            FOTA[ota] = dict()

            # Create path FOTA/<ota_filename>/
            path_fota_file = os.path.join(self.__path_FOTA, ota).strip(".zip")
            if not os.path.isdir(path_fota_file):
                os.makedirs(path_fota_file)

            # ota_target is FOTA/<ota_filename>/<ota_filename>
            ota_target = os.path.join(path_fota_file, ota)
            fota_json = os.path.join(path_fota_file, "fota.json")

            # Target tag

            # SSN
            self.__device.waitAdb(timeout=20)
            if not self.build_but["ssn"]:
                self.build_but["ssn"] = self.__device.getProperty("ro.serialno")

            # If local files then get them into dictionary
            if local:
                # Ota file
                if not os.path.isfile(ota_target):
                    output = ota + " - missing ota file in FOTA directory ({0})".format(os.path.dirname(ota_target))
                    self.__logger.printLog("WARNING", output)
                    FOTA[ota]["failure_log"] = output
                    continue

                # compute flash dictionary from local files
                FOTA[ota] = self.createDictionary(path_fota_file,
                                                  "",
                                                  self.__configuration.board,
                                                  self.provisioning_data["buildvariant"],
                                                  os.path.join("FOTA", os.path.basename(path_fota_file)),
                                                  ota_file_needed=False,
                                                  optional_url="")

                if not FOTA[ota]:
                    output = "{0} - failure to get REF files in local directory".format(ota)
                    self.__logger.printLog("WARNING", output)
                    FOTA[ota]["failure_log"] = output
                    continue

                # add FOTA file to list for conversion
                if not os.path.isfile(fota_json):
                    self.__flashFile.zipFlashFile2PftJsonFile(ota_target, fota_json, "fota", start_state="mos")
                FOTA[ota]["fota_update"] = self.build_but
                FOTA[ota]["fota_update"]["flash_list"].append(self.__flashFile.flashFileList2DictionaryList([fota_json])[0])

            # Else download all files (FOTA package + source build)
            else:
                # compute source build name
                index = 0
                build_from_name = ""
                while index != (len(ota.split("-"))-3):
                    build_from_name = self.getBuildTag("-".join(ota.split("-")[index:index+3]))
                    if build_from_name:
                        break
                    else:
                        index += 1
                if not build_from_name:
                    output = ota + " - failure computing source build name"
                    self.__logger.printLog("WARNING", output)
                    FOTA[ota]["failure_log"] = output
                    continue
                link_artifactory = self.buildTag2ArtifactoryUrl(build_from_name)[0]
                if not link_artifactory:
                    output = ota + " - no url found for '{0}'".format(build_from_name)
                    FOTA[ota]["failure_log"] = output
                    self.__logger.printLog("WARNING", output)
                    continue

                # download ref flash files
                FOTA[ota] = self.createDictionary(path_fota_file,
                                             self.build_but["tag"],
                                             self.__configuration.board,
                                             self.provisioning_data["buildvariant"],
                                             os.path.join("FOTA", os.path.basename(path_fota_file)),
                                             False,
                                             optional_url=link_artifactory)
                # if download failure:
                if not FOTA[ota]:
                    output = ota + " - failure to download source flash files for '{0}'".format(build_from_name)
                    self.__logger.printLog("WARNING", output)
                    FOTA[ota] = dict()
                    FOTA[ota]["failure_log"] = output
                    continue
                else:
                    FOTA[ota]["fota"] = ota_target
                    self.__device.waitAdb(timeout=20)

                # download FOTA package with new FOTA gen builder
                if fota_url == fota_url_external:
                    url_search = re.search("href=\"([0-9a-zA-Z/\\.:]+{0})\"".format(ota), fota_html_external)
                    if not url_search:
                        output = ota + " - file not found on FOTA server ({0})".format(fota_url)
                        self.__logger.printLog("WARNING", output)
                        FOTA[ota]["failure_log"] = output
                        continue
                    else:
                        found_url = url_search.group(1)
                        if found_url.startswith("/api"):
                            found_url = "https://fota.intel.com{0}".format(found_url)
                        if self.__host.commandExecExternalFotaSecureDownload(sh, found_url, path_fota_file)[0] != 0:
                            output = ota + " - no such file: {0} (download issue)".format(ota_target)
                            self.__logger.printLog("WARNING", output)
                            FOTA[ota]["failure_log"] = output
                            continue
                # download FOTA package from Artifactory
                elif fota_url == artifactory_ota_url:
                    exec_status = self.__host.commandExecArtifactorySecureDownload(
                            artifactory_ota_url + "/{}".format(ota),
                            path_fota_file,
                            timeout=1200)[0]
                    if exec_status != 0:
                        output = ota + " - no such file: {0} (download issue)".format(ota_target)
                        self.__logger.printLog("WARNING", output)
                        FOTA[ota]["failure_log"] = output
                        continue
                # download FOTA package with older FOTA gen builder
                elif not self.__misc.downloadFile(fota_url + "/" + ota, ota_target):
                        output = ota + " - no such file: {0} (download issue)".format(ota_target)
                        self.__logger.printLog("WARNING", output)
                        FOTA[ota]["failure_log"] = output
                        continue

                self.__flashFile.zipFlashFile2PftJsonFile(ota_target, fota_json, "fota", start_state="mos")
                FOTA[ota]["fota_update"] = self.build_but
                FOTA[ota]["fota_update"]["flash_list"].append(self.__flashFile.flashFileList2DictionaryList([fota_json])[0])

            # Build reference tag from ref flash file name
            tag = FOTA[ota]["tag"]

            # if not found, search in flash file name
            if tag == "" or tag == "notag":
                for f in FOTA[ota]["flash_list"]:
                    local_flash_file = self.__flashFile.createFile(f)
                    tag = self.getBuildTag(os.path.basename(local_flash_file.get_localFlashFile()))
                    if tag:
                        break
            FOTA[ota]["tag"] = tag
            FOTA[ota]["tag_update"] = self.build_but["tag"]
            FOTA[ota]["fota"] = ota_target

        # check if dictionary complete
        verdict = not any("failure_log" in FOTA[ota] for ota in ota_list)
        # if not, invalidate all packages
        if not verdict:
            output = ", ".join(FOTA[ota]["failure_log"] for ota in ota_list if "failure_log" in FOTA[ota])
        else:
            output = ""
        self.__output.appendOutput(output, verdict)
        return verdict, FOTA

    def getBuildTag(self, string_input, extra_regexp="", extra_group=1):
        string_input = str(string_input)
        # old regexp "cht51-week-95"
        result = re.search("([a-z0-9_]+-[a-z_]+-[0-9]+)", string_input)
        # new L1e000304
        result_new = re.search(self.__code_tag_regexp, string_input)
        # extra
        if extra_regexp and isinstance(extra_group, int):
            result_extra = re.search(extra_regexp, string_input)
        else:
            result_extra = False
        if result_extra:
            output = result_extra.group(extra_group)
        elif result:
            output = result.group(1)
        elif result_new:
            output = result_new.group(1)
        else:
            output = ""
        self.__logger.printLog("INFO", "getBuildTag(): found '{0}' from: {1}".format(output, string_input))
        return output

    def getBranchFromTag(self, string_input):
        output = ""

        # classical name: "cht51-week-95"
        result = re.search("([a-z0-9_]+)-(prei|eng|week|release_candidate|maintainer|mergerequest|latest|lst|mr|rc)", string_input)
        # new codename: L1e000304
        result_new = re.search(self.__code_tag_regexp, string_input)
        # PSI urls: https://mcg-depot.intel.com/artifactory/cactus-absp/build/eng-builds/cht51/PSI/weekly/2015_WW20_C/
        result_psi = re.search("/([a-z0-9_]+)/PSI", string_input)

        # reject modem entry to be able to launch icdg testing
        if result and result.group(1) not in ["modem"]:
            output = result.group(1)
        elif result_psi:
            output = result_psi.group(1)
        elif result_new:
            code_tag = result_new.group(1)
            if code_tag.startswith("L1"):
                output = "llp_mr1"
            elif code_tag.startswith("M0") or code_tag.startswith("MM") or code_tag.startswith("M1") or code_tag.startswith("N0"):
                output = "n_master"
            elif code_tag.startswith("NC"):
                output = "n_mr0_cht"
            elif code_tag.startswith("LS"):
                output = "llp_mr1_cht"
            elif code_tag.startswith("MS"):
                output = "m_byt_cr"
            elif code_tag.startswith("MB"):
                output = "m_mr1"
            elif code_tag.startswith("MG"):
                output = "m_sf3gr_g"
            elif code_tag.startswith("G1"):
                output = "m_mr1_sf3gr_g"
            elif code_tag.startswith("MA"):
                output = "m_mr1_sf3gr_g_maint"
            elif code_tag.startswith("MD"):
                output = "m_mr1_sf3gr_maint"
            elif code_tag.startswith("r6"):
                output = "r6"
            elif code_tag.startswith("MQ"):
                output = "m_mr1_sflte_q"
            elif code_tag.startswith("ME"):
                output = "silentlake2"
            elif code_tag.startswith("MI"):
                output = "iotg_mmr1_sf3gr"
            elif code_tag.startswith("MT"):
                output = "iotg_mmr1_sf3gr_trusty"

        if not output and "modem-sit-sofialte_m-imc-mu" in string_input:
            output = "icdg"

        self.__logger.printLog("INFO", "getBranchFromTag(): found '{0}' from: {1}".format(output, string_input))
        return output

    def getBuilderFromTag(self, string_input):
        output = ""

        # classical name: "cht51-week-95"
        result = re.search("([a-z0-9_]+)-(preint|eng|week|release_candidate|maintainer|mergerequest|latest|lst|mr|rc)", string_input)
        # new codename: L1e000304
        result_new = re.search(self.__code_tag_regexp, string_input)

        if result:
            output = result.group(2)
            if output == "week":
                output = "weekly"
            elif output == "lst":
                output = "latest"
            elif output == "preint":
                output = "preintegration"
            elif output == "eng":
                output = "engineering"
            elif output == "rc":
                output = "release_candidate"
        elif result_new:
            code_tag = result_new.group(1)
            if code_tag[2] in ["c", "0"]:
                output = self.__configuration.download.RELEASE_BUILDER
            elif code_tag[2] == "l":
                output = "latest"
            elif code_tag[2] == "e":
                output = "engineering"
            elif code_tag[2] == "p":
                output = "preintegration"
            elif code_tag[2] == "m":
                output = "mergerequest"
            elif code_tag[2] == "w":
                output = "weekly"
            else:
                self.__logger.printLog("WARNING", "unknown builder for char '{0}'".format(code_tag[2]))

        self.__logger.printLog("INFO", "getBuilderFromTag(): found '{0}' from: {1}".format(output, string_input))
        return output

    def getNumberFromTag(self, string_input):
        output = ""

        # classical name: "cht51-week-95"
        branch_name = self.getBranchFromTag(string_input)
        builder_name = self.getBuilderFromTag(string_input)
        result = re.search("{0}-[a-z_][-/a-z]+([0-9]+)".format(branch_name, builder_name), string_input)
        # new codename: L1e000304
        result_new = re.search(self.__code_tag_regexp, string_input)

        if result_new:
            output = result_new.group(1)
            output = re.search("[0-9]+$", output[3:]).group(0)
        elif result:
            output = result.group(1)

        try:
            output = int(output)
        except Exception as e:
            self.__logger.printLog("WARNING", "getNumberFromTag(): failure to convert '{0}' as integer (error={1})".format(output, e))

        self.__logger.printLog("INFO", "getNumberFromTag(): found '{0}' from: {1}".format(output, string_input))
        return output

    def buildTag2ArtifactoryUrl(self, build_tag):
        # erase previously written output json file
        if self.__globalConf.get("WORKDIR"):
            pft_output_file = os.path.join(self.__globalConf.get("WORKDIR"), "pft_output.json")
        else:
            pft_output_file = os.path.join(tempfile.gettempdir(), "pft_output_{}.json".format(self.__misc.getUser()))
        self.__misc.local_files_handler.addEntry(pft_output_file)
        if os.path.isfile(pft_output_file):
            os.remove(pft_output_file)

        branch = self.getBranchFromTag(build_tag)
        builder = self.getBuilderFromTag(build_tag)
        number = self.getNumberFromTag(build_tag)

        if branch == "" or builder == "" or number == "":
            return None, "missing data among branch='{0}', builder='{1}', number='{2}'".format(branch, builder, number)

        # search link with build tag source
        cmd = "--search --non-interactive " \
              "--property buildbot.props.branch_name={0} " \
              "--property buildbot.props.buildername=*-{1} " \
              "--property buildbot.props.buildnumber={2} " \
              "--use-latest-published " \
              "--search-output {3}" \
              .format(branch, builder, number, pft_output_file)
        self.__host.commandExecCflasher(cmd, 150)

        if not os.path.isfile(pft_output_file):
            self.__logger.printLog("WARNING", "PFT command failure, cannot retrieve buildbot data")
            return None, "PFT command failure, cannot retrieve buildbot data for build tag: {0}".format(build_tag)

        # try to read pft output json. exit when failed reading file
        try:
            f = open(pft_output_file)
            data = json.load(f)
            f.close()
            os.remove(pft_output_file)
        except Exception as e:
            self.__logger.printLog("WARNING", "could not read json file '{0}' ({1})".format(pft_output_file, e))
            os.remove(pft_output_file)
            return None, "could not read json file '{0}' ({1})".format(pft_output_file, e)

        if "results" not in data:
            self.__logger.printLog("WARNING", "missing 'results' key in '{0}'".format(pft_output_file))
            return None, "missing 'results' key in '{0}'".format(pft_output_file)

        if len(data["results"]) != 1:
            self.__logger.printLog("WARNING", "a single result has not been found in '{0}' (none or multiple found)".format(pft_output_file))
            return None, "a single result has not been found in '{0}' (none or multiple found)".format(pft_output_file)

        if "downloadUri" not in data["results"][0]:
            self.__logger.printLog("WARNING", "missing downloadUri key in '{0}'".format(data["results"][0]))
            return None, "missing 'downloadUri' key in '{0}'".format(data["results"][0])

        output_url = self.cleanUrl(data["results"][0]["downloadUri"])
        output_url = output_url.replace("build_info_v3.json", "build_info.json")
        self.__logger.printLog("INFO", "url found for '{0}' build: {1}".format(build_tag, output_url))
        return output_url, ""

    def getLatestBuild(self, branch="master", builder="latest", build_reason=""):
        # erase previously written output json file
        if self.__globalConf.get("WORKDIR"):
            pft_output_file = os.path.join(self.__globalConf.get("WORKDIR"), "temp_pft_output.json")
        else:
            pft_output_file = os.path.join(tempfile.gettempdir(), "temp_pft_output_{}.json".format(self.__misc.getUser()))
        self.__misc.local_files_handler.addEntry(pft_output_file)
        if os.path.isfile(pft_output_file):
            os.remove(pft_output_file)

        if branch == "" or builder == "":
            return None, "missing data among branch='{0}', builder='{1}'".format(branch, builder), {}

        # search link with build tag source
        cmd = "--search --non-interactive " \
              "--property buildbot.props.branch_name={0} " \
              "--property buildbot.props.buildername=*-{1} " \
              "--use-latest-published " \
              "--search-output {2}" \
              .format(branch, builder, pft_output_file)
        if build_reason:
            cmd += " --property buildbot.props.reason={0}".format(build_reason)
        self.__host.commandExecCflasher(cmd, 150)

        if not os.path.isfile(pft_output_file):
            self.__logger.printLog("WARNING", "PFT command failure, cannot retrieve buildbot data")
            return None, "PFT command failure, cannot retrieve buildbot data for builder: {0}-{1}".format(branch, builder), {}

        # try to read pft output json. exit when failed reading file
        try:
            f = open(pft_output_file)
            data = json.load(f)
            f.close()
            os.remove(pft_output_file)
        except Exception as e:
            os.remove(pft_output_file)
            self.__logger.printLog("WARNING", "could not read json file '{0}' ({1})".format(pft_output_file, e))
            return None, "could not read json file '{0}' ({1})".format(pft_output_file, e), {}

        if "results" not in data:
            self.__logger.printLog("WARNING", "missing 'results' key in '{0}'".format(pft_output_file))
            return None, "missing 'results' key in '{0}'".format(pft_output_file), {}

        if len(data["results"]) != 1:
            self.__logger.printLog("WARNING", "a single result has not been found in '{0}' (none or multiple found)".format(pft_output_file))
            return None, "a single result has not been found in '{0}' (none or multiple found)".format(pft_output_file), {}

        if "downloadUri" not in data["results"][0]:
            self.__logger.printLog("WARNING", "missing downloadUri key in '{0}'".format(data["results"][0]))
            return None, "missing 'downloadUri' key in '{0}'".format(data["results"][0]), {}

        output_url = self.cleanUrl(data["results"][0]["downloadUri"])
        output_url = output_url.replace("build_info_v3.json", "build_info.json")
        self.__logger.printLog("INFO", "latest url found for '{}-{}' builder: {}".format(branch, builder, output_url))
        return output_url, "", data["results"][0]

    # download file if not already available
    def __getBuildInfoJson(self, url):
        if not self.__provisioning_build_info_json:
            self.__logger.printLog("WARNING", "getBuildInfoJson(): no path to save build_info.json to")
            return False
        if not os.path.isfile(self.__provisioning_build_info_json):
            self.__host.commandExecArtifactorySecureDownload(url, os.path.dirname(self.__provisioning_build_info_json), file_name=os.path.basename(self.__provisioning_build_info_json))
        else:
            self.__logger.printLog("DEBUG", "getBuildInfoJson(): local copy found")
        if os.path.isfile(self.__provisioning_build_info_json):
            return True
        else:
            self.__logger.printLog("WARNING", "getBuildInfoJson(): download failure")
            return False

    def getBuildInfoJsonData(self, url):
        if not self.__getBuildInfoJson(url):
            return {}
        try:
            with open(self.__provisioning_build_info_json) as f:
                json_data = json.load(f)
        except Exception as e:
            self.__logger.printLog("WARNING", "getBuildInfoJsonData(): failure to parse {} "
                                              "(error={})".format(self.__provisioning_build_info_json,
                                                                  e))
            json_data = {}
        return json_data

    def getBuildInfoJsonLocation(self, url):
        if self.__getBuildInfoJson(url):
            return self.__provisioning_build_info_json
        else:
            return ""
