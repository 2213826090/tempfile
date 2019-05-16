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
@summary: Pupdr Library - FlashFileModule
@since: 11/17/2014
@author: travenex
"""

import os
import re
import json
import shutil
import zipfile
import LoggerModule
import MiscModule
import ConfigurationModule

class FlashFileModule(object):

    __instance = None
    __globalConf = None
    __logger = None
    __configuration = None
    __misc = None
    __user_flashing = None

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
        self.__configuration = ConfigurationModule.ConfigurationModule()
        self.__misc = MiscModule.MiscModule()
        self.__user_flashing = False

    # get data from json file provided by PFT
    def __get_json_data(self, json_file, data):
        # check is file has json format
        if not json_file.endswith(".json"):
            self.__logger.printLog("INFO", "file has no json extension: %s" % json_file)
            return ""
        # check if json has key 'localFlashFile'
        try:
            with open(json_file) as f:
                json_data = json.load(f)
                if data in json_data:
                    return json_data[data]
                else:
                    self.__logger.printLog("INFO", "key '%s' not found in: %s" % (data, json_file))
                    return ""
        # catch issue when json is of bad format
        except Exception as e:
            self.__logger.printLog("INFO", "failed to read json '%s' (%s)" % (json_file, e))
            return ""

    def zipFlashFile2PftJsonFile(self, zip_file, json_output, file_type, target_flash_file="", target_configuration="",
                                 groups=None, start_state=""):
        log = "zipFlashFile2PftJsonFile({0}): ".format(file_type)
        self.__logger.printLog("INFO", log + "converting zip file to PFT json format: {0}".format(zip_file))
        output = dict()
        # unzip file
        output_directory = zip_file.split(".zip")[0] + "/"
        try:
            z = zipfile.ZipFile(zip_file, 'r')
            z.extractall(output_directory)
            z.close()
        except zipfile.BadZipfile as e:
            self.__logger.printLog("WARNING", log + "failure to unzip {} ({})".format(zipfile, e))
            return
        flash_file = ""

        # select proper xml file if present
        if target_flash_file:
            if os.path.isfile(os.path.join(output_directory, target_flash_file)):
                flash_file = target_flash_file
            else:
                self.__logger.printLog("WARNING", log + "'{0}' not found in: {1}".format(target_flash_file, output_directory))
                zip_file = None
        if file_type == "ota" and self.__configuration.flash.FULL_OTA_LOCAL_FLASH_FILE:
             flash_file = self.__createFlashFileForOta(zip_file, os.path.dirname(json_output), "full")
        if file_type == "fota":
            if self.__configuration.flash.INCREMENTAL_OTA_LOCAL_FLASH_FILE or \
                    self.__configuration.flash.INCREMENTAL_OTA_USER_LOCAL_FLASH_FILE:
                flash_file = self.__createFlashFileForOta(zip_file, os.path.dirname(json_output), "incremental")

        # in case no proper file found
        if not zip_file:
            self.__logger.printLog("WARNING", log + "aborting because {1} not found "
                                                    "in {0}".format(output_directory, target_flash_file))
            return None
        # fill output dictionary
        output["flashConfigurationStartState"] = start_state
        output["flashFileName"] = flash_file
        if target_configuration:
            output["flashConfiguration"] = target_configuration
            if not os.path.isfile(os.path.join(output_directory, "flash.json")):
                output["flashConfiguration"] = ""
            else:
                json_file= os.path.join(output_directory, "flash.json")
                with open(json_file, 'r') as f:
                    json_data = json.load(f)
                    if target_configuration in json_data["flash"]["configurations"] and "startState" in json_data["flash"]["configurations"][target_configuration]:
                            output["flashConfigurationStartState"] = json_data["flash"]["configurations"][target_configuration]["startState"]
                    else:
                        output["flashConfigurationStartState"] = "dnx"
        else:
            output["flashConfiguration"] = ""
        output["localFlashFile"] = zip_file
        output["board"] = self.__configuration.board
        # groups management
        if groups and isinstance(groups, dict):
            output["flashGroupsState"] = groups

        if "windows" in os.environ.get('OS','').lower():
            output["localFlashFile"] = output["localFlashFile"].replace("/data", "C:/cygwin64/data").replace("/", "\\")
        # write dictionary to file
        try:
            if os.path.isfile(json_output):
                os.remove(json_output)
            with open(json_output, 'wb') as outfile:
                outfile.write(json.dumps(output, sort_keys=True, indent=4))

        except Exception as e:
            self.__logger.printLog("WARNING", log + "failure to write json file: {0} ({1})".format(json_output, e))

    def buildUserdebugReprovisionFlashFile(self, userdebug_osloader, userdebug_fastboot, pre_flash_json):
        log = "buildUserdebugReprovisionFlashFile(): "

        # create fake zip file
        name = "userdebug-reprovision"
        dirname = os.path.abspath(os.path.join(os.path.dirname(pre_flash_json), name))
        if os.path.isdir(dirname):
            shutil.rmtree(dirname)
        os.makedirs(dirname)
        zipname = dirname + ".zip"
        # copy osloader
        osloader_copy = os.path.join(dirname, os.path.basename(userdebug_osloader))
        shutil.copy(userdebug_osloader, osloader_copy)
        # copy fastboot
        fastboot_copy = os.path.join(dirname, os.path.basename(userdebug_fastboot))
        shutil.copy(userdebug_fastboot, fastboot_copy)
        flash_file = self.__createFlashFileForUserdebugReprovision(os.path.basename(userdebug_osloader),
                                                                   os.path.basename(userdebug_fastboot),
                                                                   dirname)
        # copy blpolicy_erase
        blpolicy_erase = os.path.join(self.__misc.extra_path, "blpolicy-delete.txt")
        blpolicy_erase_copy = os.path.join(dirname, os.path.basename(blpolicy_erase))
        shutil.copy(blpolicy_erase, blpolicy_erase_copy)
        self.__misc.zipDirectory(zipname, dirname)

        # fill dictionary
        self.__logger.printLog("INFO", log + "start")
        output = dict()
        output["flashConfigurationStartState"] = "dnx"
        output["flashFileName"] = os.path.basename(flash_file)
        output["flashConfiguration"] = ""
        output["localFlashFile"] = zipname

        # write file
        try:
            with open(pre_flash_json, 'wb') as outfile:
                json.dump(output, outfile)
        except Exception as e:
            self.__logger.printLog("WARNING", log + "failure to write json file: {0} ({1})".format(pre_flash_json, e))

    def __createFlashFileForOta(self, local_zip_file, destination_directory, ota_type):
        """ Method to create the flash file, when it is not inside the ota package
        """
        log = "createFlashFileForOta(): "
        self.__user_flashing = self.__globalConf.get("PROVISIONING_DATA", {}).get("build_variant", "") == "user"
        if not os.path.exists(local_zip_file):
            self.__logger.printLog("WARNING", log + "local file does not exist: {0}".format(local_zip_file))
            return
        if not os.path.isdir(destination_directory):
            self.__logger.printLog("WARNING", log + "destination directory does not exist: {0}".format(destination_directory))
            return
        if ota_type == "full":
            if self.__configuration.flash.FULL_OTA_LOCAL_FLASH_FILE:
                json_file = os.path.abspath(os.path.join(os.path.dirname(__file__), self.__configuration.flash.FULL_OTA_LOCAL_FLASH_FILE))
            else:
                json_file = ""
                self.__logger.printLog("INFO", log + "Flash.FULL_OTA_LOCAL_FLASH_FILE configuration value is empty")
        elif ota_type == "incremental":
            if self.__user_flashing:
                if self.__configuration.flash.INCREMENTAL_OTA_USER_LOCAL_FLASH_FILE:
                    json_file = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                             self.__configuration.flash.INCREMENTAL_OTA_USER_LOCAL_FLASH_FILE))
                else:
                    json_file = ""
                    self.__logger.printLog("INFO",
                                           log + "Flash.INCREMENTAL_OTA_USER_LOCAL_FLASH_FILE configuration value is empty")
            else:
                if self.__configuration.flash.INCREMENTAL_OTA_LOCAL_FLASH_FILE:
                    json_file = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                                             self.__configuration.flash.INCREMENTAL_OTA_LOCAL_FLASH_FILE))
                else:
                    json_file = ""
                    self.__logger.printLog("INFO", log + "Flash.INCREMENTAL_OTA_LOCAL_FLASH_FILE configuration value is empty")
        else:
            json_file = ""
            self.__logger.printLog("WARNING", log + "invalid ota type: {0} ('full' or 'incremental' expected)".format(ota_type))
        if not json_file:
            self.__logger.printLog("WARNING", log + "cannot create flash file for local file: {0}".format(local_zip_file))
            return ""
        dest_file = os.path.join(destination_directory, os.path.basename(json_file))
        self.__logger.printLog("INFO", log + "copy '{0}' to '{1}' and fill with value: {2}".format(json_file, dest_file, local_zip_file))
        shutil.copy(json_file, dest_file)
        with open(dest_file, 'r') as f:
            json_data = json.load(f)
            json_data["flash"]["parameters"]["ota_file"]["value"] = local_zip_file
        with open(dest_file, 'w') as f:
            f.write(json.dumps(json_data, sort_keys=True, indent=4))
            f.close()
        return dest_file

    def __createFlashFileForUserdebugReprovision(self, userdebug_osloader, userdebug_fastboot, destination_directory):
        """ Method to create the flash file, when it is not inside the ota package
        """
        log = "createFlashFileForUserdebugReprovision(): "
        if not os.path.isdir(destination_directory):
            self.__logger.printLog("WARNING", log + "destination directory does not exist: {0}".format(destination_directory))
            return
        if self.__configuration.boot.GVB_ACTIVATION.endswith("-M"):
            json_file = os.path.abspath(os.path.join(os.path.dirname(__file__), "Extras", "flashUserdebugReprovisionM.json"))
        else:
            json_file = os.path.abspath(os.path.join(os.path.dirname(__file__), "Extras", "flashUserdebugReprovision.json"))
        dest_file = os.path.join(destination_directory, os.path.basename(json_file))
        self.__logger.printLog("INFO", log + "copy '{0}' to '{1}' and fill with values: "
                                             "osloader={2}, "
                                             "fastboot={3}".format(json_file,
                                                                   dest_file,
                                                                   userdebug_osloader,
                                                                   userdebug_fastboot))
        shutil.copy(json_file, dest_file)
        with open(dest_file, 'r') as f:
            json_data = json.load(f)
            json_data["flash"]["parameters"]["osloader_file"]["value"] = userdebug_osloader
            json_data["flash"]["parameters"]["fastboot_file"]["value"] = userdebug_fastboot
        with open(dest_file, 'w') as f:
            f.write(json.dumps(json_data, sort_keys=True, indent=4))
            f.close()
        return dest_file

    # convert flash files into proper dictionaries
    def flashFileList2DictionaryList(self, file_list):
        log = "flashFileList2DictionaryList(): "
        self.__logger.printLog("INFO", log + "start ({0})".format(", ".join(file_list)))
        # this dictionary will contain a list of dictionaries, one for each flash file
        final_dictionary = list()
        for singleFile in file_list:
            local_dict = self.createFile()
            # check format
            if self.__misc.isExpectedJsonFormat(singleFile, ["localFlashFile"]):
                local_dict.set_configuration(self.__get_json_data(singleFile, "flashConfiguration"))
                local_dict.set_start_state(self.__get_json_data(singleFile, "flashConfigurationStartState"))
                local_dict.set_file_location(self.__get_json_data(singleFile, "localFlashFile"))
                local_dict.set_pft_flash_file(self.__get_json_data(singleFile, "flashFileName"))
                local_dict.set_board(self.__get_json_data(singleFile, "board"))
                group_state = self.__get_json_data(singleFile, "flashGroupsState")
                if group_state and isinstance(group_state, dict):
                    for entry in group_state:
                        if self.__globalConf.get("PROVISIONING_DATA", {}).get("build_variant", "") == "user" and "erase-efivars" in entry:
                            group_state[entry] = False
                            local_dict.set_group(entry, group_state[entry])
                        else:
                            local_dict.set_group(entry, group_state[entry])
                local_dict.set_pft_json(singleFile)
                local_dict.set_file_type(os.path.basename(singleFile).replace(".json", ""))
            else:
                if "blankphone" in os.path.basename(singleFile) or (not os.path.basename(singleFile).endswith(".zip") and os.path.basename(os.path.dirname(singleFile))):
                    local_dict.set_file_type("blankphone")
                    local_dict.set_start_state("dnx")
                    # if path to json or xml inside zip file is provided, separate both data
                    if any (singleFile.endswith(ext) for ext in ("xml", "json")):
                        # flashing on windows, blankphone unzip directory is changed by test_runner
                        zip_file_name = os.path.dirname(singleFile) + ".zip"
                        if os.path.isfile(zip_file_name):
                            local_dict.set_file_location(zip_file_name)
                            local_dict.set_pft_flash_file(os.path.basename(singleFile))
                        else:
                            local_dict.set_file_location(singleFile)
                    # else provide link to zip file
                    else:
                        local_dict.set_file_location(singleFile)
                elif "fastboot" in os.path.basename(singleFile) or (not os.path.basename(singleFile).endswith(".zip") and os.path.basename(os.path.dirname(singleFile))):
                    local_dict.set_file_type("fastboot")
                    local_dict.set_start_state("pos")
                    # if path to json or xml inside zip file is provided, separate both data
                    if any (singleFile.endswith(ext) for ext in ("xml", "json")):
                        local_dict.set_file_location(os.path.dirname(singleFile) + ".zip")
                        local_dict.set_pft_flash_file(os.path.basename(singleFile))
                    # else provide link to zip file
                    else:
                        local_dict.set_file_location(singleFile)
                elif "ota" in os.path.basename(singleFile) or (not os.path.basename(singleFile).endswith(".zip") and os.path.basename(os.path.dirname(singleFile))):
                    local_dict.set_file_type("ota")
                    local_dict.set_start_state("mos")
                    local_dict.set_file_location(singleFile)
            # update output dictionary with latest entry
            final_dictionary.append(local_dict.get_flash_dictionary())
        self.__logger.printLog("INFO", log + "built list:\n{0}".format(json.dumps(final_dictionary)))
        return final_dictionary

    def createFile(self, inputElement=None):
        return self.flashFiles(self.__logger, inputElement)

    class flashFiles():
        """ Class to group methods to gather flashfiles parameters
        """
        # init: an input can be provided
        def __init__(self,logger, inputElement=None):
            self.__logger = logger
            self.__output = dict()
            self.__output["flashGroupsState"] = dict()
            # path to zip file
            if inputElement and inputElement.get("localFlashFile") not in ("", None):
                self.set_file_location(inputElement["localFlashFile"])
            else:
                self.__output["localFlashFile"] = ""
            # flash file inside zip
            if inputElement and inputElement.get("pft_flash_file") not in ("", None):
                self.set_pft_flash_file(inputElement["pft_flash_file"])
            else:
                self.__output["pft_flash_file"] = ""
            # configuration
            if inputElement and inputElement.get("flashConfiguration") not in ("", None):
                self.set_configuration(inputElement["flashConfiguration"])
            else:
                self.__output["flashConfiguration"] = ""
            # file/flash type
            if inputElement and inputElement.get("file_type") not in ("", None):
                self.set_file_type(inputElement["file_type"])
            else:
                self.__output["file_type"] = ""
            # board start state
            if inputElement and inputElement.get("flashConfigurationStartState") not in ("", None):
                self.set_start_state(inputElement["flashConfigurationStartState"])
            else:
                self.__output["flashConfigurationStartState"] = ""
            # detect if PFT json format
            if inputElement and inputElement.get("pft_json"):
                self.set_pft_json(inputElement["pft_json"])
            else:
                self.__output["pft_json"] = ""
            # board
            if inputElement and inputElement.get("board"):
                self.set_board(inputElement["board"])
            else:
                self.__output["board"] = ""
            # flashGroupEnabled
            if inputElement and inputElement.get("flashGroupsState"):
                for element in inputElement.get("flashGroupsState"):
                    self.set_group(element, inputElement["flashGroupsState"][element])

        # list of function to set dictionary parameters. minors are also made to check inputs are of proper format

        def set_file_location(self, local_file):
            self.__output["localFlashFile"] = local_file
        def set_pft_flash_file(self, pft_file):
            extension = (".json", ".xml")
            if not any (pft_file.endswith(ext) for ext in extension):
                self.__logger.printLog("WARNING", "set_pft_flash_file: file '{0}' not in list of allowed files ({1})".format(pft_file, ", ".join(extension)))
                self.__output["pft_flash_file"] = ""
            else:
                self.__output["pft_flash_file"] = pft_file
        def set_configuration(self, config):
            config_list = ("update", "blank", "recover", "fw_update", "fls")
            if config and not any(single_config in config for single_config in config_list):
                self.__logger.printLog("DEBUG", "set_configuration: config '{0}' not in list of known configs ({1})".format(config, ", ".join(config_list)))
            self.__output["flashConfiguration"] = config
        def set_file_type(self, file_type):
            self.__output["file_type"] = file_type
        def set_start_state(self, state):
            state_list = ("dnx_fw", "dnx_os", "dnx", "pos", "mos", "dnx_pos")
            if state not in state_list:
                self.__logger.printLog("WARNING", "set_start_state: start state '{0}' not in list of allowed states ({1})".format(state, ", ".join(state_list)))
                self.__output["flashConfigurationStartState"] = ""
            else:
                self.__output["flashConfigurationStartState"] = state
        def set_pft_json(self, json_file):
            json_file = str(json_file)
            if not json_file.endswith(".json"):
                self.__logger.printLog("WARNING", "set_pft_json: invalid value '{0}', json file path expected".format(json_file))
                self.__output["pft_json"] = ""
            else:
                self.__output["pft_json"] = json_file
        def set_board(self, board):
            self.__output["board"] = str(board)
        def set_group(self, group, value):
            if not isinstance(value, bool):
                self.__logger.printLog("WARNING", "set_group: invalid value '{0}', boolean expected".format(value))
            else:
                self.__output["flashGroupsState"][group] = value

        # get to whole dictionary

        def get_flash_dictionary(self):
            return self.__output

        # get single parameters

        def get_localFlashFile(self):
            return self.__output["localFlashFile"]
        def get_pft_flash_file(self):
            return self.__output["pft_flash_file"]
        def get_flashConfiguration(self):
            return self.__output["flashConfiguration"]
        def get_file_type(self):
            return self.__output["file_type"]
        def get_flashConfigurationStartState(self):
            return self.__output["flashConfigurationStartState"]
        def get_pft_json(self):
            return self.__output["pft_json"]
        def get_board(self):
            return self.__output["board"]
        def get_flash_group_state(self):
            return self.__output["flashGroupsState"]
