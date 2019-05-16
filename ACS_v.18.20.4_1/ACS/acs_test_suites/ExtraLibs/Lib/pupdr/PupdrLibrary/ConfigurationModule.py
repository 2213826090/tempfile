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
@summary: Pupdr Library - ConfigurationModule
@since: 12/4/2014
@author: travenex
"""

import os
import json
import copy
import LoggerModule
import OutputModule
import MiscModule

class ConfigurationModule(object):

    __instance = None
    __globalConf = None
    __logger = None
    __output = None
    flash = None
    boot = None
    logs = None
    timeout = None
    download = None
    custom = None
    boot_vars = None
    workaround = None
    test_description = None
    board = None
    branch = None
    __configuration_list = dict()
    __branchList = None
    __boardList = None
    __current_config_name = None

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
        self.__output = OutputModule.OutputModule()
        self.flash = None
        self.boot = None
        self.logs = None
        self.timeout = None
        self.download = None
        self.custom = None
        self.boot_vars = None
        self.workaround = None
        self.board = None
        self.branch = None
        self.test_description = None
        self.__configuration_list = dict()
        self.__branchList = None
        self.__boardList = None
        self.__current_config_name = ""

    def __getInheritBoards(self, branch, board, build_target):
        if not board and not build_target:
            self.__logger.printLog("DEBUG", "no board nor build target for configuration loading on '{}' "
                                            "branch".format(branch))
            return
        elif not board and build_target:
            self.__logger.printLog("INFO", "loading configuration for '{}' build target on '{}' "
                                           "branch".format(build_target, branch))
            board = build_target
            build_target = ""
        else:
            self.__logger.printLog("INFO", "loading configuration for '{}' board on '{}' branch".format(board, branch))

        # load build target first
        if build_target:
            self.__logger.printLog("INFO", "including '{}' build target inheritance on '{}'"
                                           "branch".format(build_target, branch))
            self.__getInheritBoards(branch, "", build_target)

        board_conf = os.path.join(os.path.dirname(__file__), "Configurations", branch, board + ".json")
        if not os.path.isfile(board_conf):
            # try to find boards with same name to fallback to
            if os.path.isdir(os.path.join(os.path.dirname(__file__), "Configurations", branch)):
                fallback_boards = list()
                all_boards = os.listdir(os.path.join(os.path.dirname(__file__), "Configurations", branch))
                for board_json in all_boards:
                    board_name = board_json.replace(".json", "")
                    if board_name in board:
                        fallback_boards.append(board_name)
                if not fallback_boards:
                    self.__logger.printLog("INFO", "board file does not exist and no fallback board found for '{0}' board in '{1}' branch".format(board, branch))
                    board_conf = None
                else:
                    candidate_board = max(fallback_boards, key=len)
                    self.__logger.printLog("INFO", "'{0}' fallback board found in '{3}': {1} (selected among: {2})"\
                                           .format(board, candidate_board, ", ".join(fallback_boards), branch))
                    board_conf = os.path.join(os.path.dirname(__file__), "Configurations", branch, candidate_board + ".json")
            else:
                board_conf = None

        if board_conf:
            if board_conf in self.__boardList:
                self.__boardList.remove(board_conf)
                self.__logger.printLog("DEBUG", "removing older reference to {}".format(board_conf))
            self.__boardList.insert(0, board_conf)

            try:
                with open(board_conf) as f:
                    board_conf_data = json.load(f)
            except Exception as e:
                self.__logger.printLog("WARNING", "failure to parse json: {0} ({1})".format(board_conf, e))
            else:
                if "Configuration" in board_conf_data and "Inherit" in board_conf_data["Configuration"] and "BOARD" in board_conf_data["Configuration"]["Inherit"]:
                    inherit_board = board_conf_data["Configuration"]["Inherit"]["BOARD"]
                else:
                    inherit_board = None
                if inherit_board:
                    new_board =  os.path.join(os.path.dirname(__file__), "Configurations", branch, inherit_board + ".json")
                    if not os.path.isfile(new_board):
                        self.__logger.printLog("WARNING", "board configuration '{0}' does not exist (cannot inherit from it)".format(new_board))
                    else:
                        self.__logger.printLog("INFO", "board '{0}' inheriting from '{1}' in '{2}' branch".format(board, inherit_board, branch))
                        self.__getInheritBoards(branch, inherit_board, "")

    def __getJsonFiles(self, branch, board, build_target):
        outputList = list()

        # read branch default if it exist and add to list
        branch_conf = os.path.join(os.path.dirname(__file__), "Configurations", branch, "branch_default.json")
        if not os.path.isdir(os.path.dirname(branch_conf)):
            self.__logger.printLog("INFO", "'{0}' branch directory missing at: {1}".format(branch, os.path.dirname(branch_conf)))
            branch_conf = None
        elif not os.path.isfile(branch_conf):
            self.__logger.printLog("INFO", "branch configuration missing ({0})".format(branch_conf))
            branch_conf = None

        # read board configuration and get inheritance
        self.__boardList = list()
        self.__getInheritBoards(branch, board, build_target)

        # build list and return
        if branch_conf:
            outputList.append(branch_conf)
        if self.__boardList:
            for element in self.__boardList:
                outputList.append(element)
        return outputList

    def __getInheritBranches(self, branch):
        branch_conf = os.path.join(os.path.dirname(__file__), "Configurations", branch, "branch_default.json")
        self.__branchList.insert(0, branch)
        if os.path.isfile(branch_conf):
            try:
                with open(branch_conf) as f:
                    branch_conf_data = json.load(f)
            except Exception as e:
                self.__logger.printLog("WARNING", "failure to parse json: {0} ({1})".format(branch_conf, e))
            else:
                if "Configuration" in branch_conf_data and "Inherit" in branch_conf_data["Configuration"] and "BRANCH" in branch_conf_data["Configuration"]["Inherit"]:
                    inherit_branch = branch_conf_data["Configuration"]["Inherit"]["BRANCH"]
                else:
                    inherit_branch = None
                if inherit_branch:
                    self.__logger.printLog("INFO", "branch '{0}' inheriting from '{1}'".format(branch, inherit_branch))
                    self.__getInheritBranches(inherit_branch)
        else:
            # try to find branches with same name to fallback to
            fallback_branches = list()
            all_branches = os.listdir(os.path.join(os.path.dirname(__file__), "Configurations"))
            for branch_name in all_branches:
                if branch_name in branch:
                    fallback_branches.append(branch_name)
            if not fallback_branches:
                self.__logger.printLog("INFO", "branch file does not exist and no fallback branch found for '{0}'".format(branch))
            else:
                candidate_branch = max(fallback_branches, key=len)
                self.__logger.printLog("INFO", "'{0}' fallback branch found: {1} (selected among: {2})".format(branch, candidate_branch, ", ".join(fallback_branches)))
                self.__getInheritBranches(candidate_branch)

    def __printJsonList(self, jsonList):
        output = "Json Configuration Files List (from higher to lower priority):\n"
        for element in jsonList:
            output += element + "\n"
        self.__logger.printLog("INFO", output)

    def storeCurrentConfig(self, config_name):
        self.__logger.printLog("INFO", "storeCurrentConfig({0}): start".format(config_name))
        self.__configuration_list[config_name] = {"branch": self.branch,
                                                  "board": self.board,
                                                  "flash_params": self.flash,
                                                  "boot_params": self.boot,
                                                  "logs_params": self.logs,
                                                  "timeout_params": self.timeout,
                                                  "download_params": self.download,
                                                  "boot_vars_params": self.boot_vars,
                                                  "workaround_params": self.workaround,
                                                  "test_description_params": self.test_description,
                                                  "custom_params": self.custom}
        self.__current_config_name = config_name

    def switchConfig(self, config_name):
        log = "switchConfig({0}): ".format(config_name)
        self.__logger.printLog("INFO", log + "start")
        if config_name not in self.__configuration_list:
            self.__logger.printLog("WARNING", log + "configuration name does not exist, available configuration are: {1}")\
                .format(config_name, ", ".join([f for f in self.__configuration_list]))
        else:
            if config_name !=  self.__current_config_name:
                self.branch = self.__configuration_list[config_name]["branch"]
                self.board = self.__configuration_list[config_name]["board"]
                self.flash = self.__configuration_list[config_name]["flash_params"]
                self.boot = self.__configuration_list[config_name]["boot_params"]
                self.logs = self.__configuration_list[config_name]["logs_params"]
                self.timeout = self.__configuration_list[config_name]["timeout_params"]
                self.download = self.__configuration_list[config_name]["download_params"]
                self.boot_vars = self.__configuration_list[config_name]["boot_vars_params"]
                self.workaround = self.__configuration_list[config_name]["workaround_params"]
                self.custom = self.__configuration_list[config_name]["custom_params"]
                self.test_description = self.__configuration_list[config_name]["test_description_params"]
                self.__current_config_name = config_name
            else:
                self.__logger.printLog("INFO", log + "parameters already in the right configuration")

    @property
    def loaded_config_list(self):
        return [config_name for config_name in self.__configuration_list]

    def updateConfig(self, branch, board, print_config=True, config_name="", build_target=""):
        jsonList = list()

        self.__logger.printLog("INFO", "updateConfig(): loading parameters for branch='{0}' and board='{1}'".format(branch, board))
        if build_target:
            self.__logger.printLog("INFO", "updateConfig(): build target fallback '{0}' "
                                           "for '{1}' board".format(build_target, board))

        # store data:
        self.branch = branch
        self.board = board

        if not config_name and branch and board:
            config_name = "-".join([branch, board])
            if build_target:
                config_name += "-" + build_target

        # if config already loaded, switch config
        if config_name in self.loaded_config_list:
            self.switchConfig(config_name)
            return True

        # get global configuration:
        global_conf = os.path.join(os.path.dirname(__file__), "Configurations", "global_default.json")
        if not os.path.isfile(global_conf):
            self.__logger.printLog("WARNING", "global configuration missing ({0})".format(global_conf))
            return False
        jsonList.append(global_conf)

        # check if branch inheritance list
        self.__branchList = list()
        self.__getInheritBranches(branch)

        # get all configuration Json:
        for singleBranch in self.__branchList:
            for jsonFile in self.__getJsonFiles(singleBranch, board, build_target):
                jsonList.insert(0, jsonFile)

        self.__printJsonList(jsonList)

        self.flash = jsonParser(self.__globalConf, jsonList, "Flash", print_config=print_config)
        self.boot = jsonParser(self.__globalConf, jsonList, "Boot", print_config=print_config)
        self.logs = jsonParser(self.__globalConf, jsonList, "Logs", print_config=print_config)
        self.timeout = jsonParser(self.__globalConf, jsonList, "Timeout", print_config=print_config)
        self.download = jsonParser(self.__globalConf, jsonList, "Download", print_config=print_config)
        self.boot_vars = jsonParser(self.__globalConf, jsonList, "BootVars", print_config=print_config)
        self.workaround = jsonParser(self.__globalConf, jsonList, "WorkAround", specific_type="WA", print_config=print_config)
        self.test_description = jsonParser(self.__globalConf, jsonList, "TestDescription", print_config=print_config)
        self.custom = jsonParser(self.__globalConf, jsonList, "Custom", print_config=print_config)

        if config_name:
            self.__logger.printLog("INFO", "updateConfig(): storing current configuration into '{0}'".format(config_name))
            self.__configuration_list[config_name] = {"branch": self.branch,
                                                     "board": self.board,
                                                     "flash_params": self.flash,
                                                     "boot_params": self.boot,
                                                     "logs_params": self.logs,
                                                     "timeout_params": self.timeout,
                                                     "download_params": self.download,
                                                     "boot_vars_params": self.boot_vars,
                                                     "workaround_params": self.workaround,
                                                     "test_description_params": self.test_description,
                                                     "custom_params": self.custom}
            self.__current_config_name = config_name

        return True

class jsonParser():
    def __init__(self, globalConf, jsonList, config, specific_type=None, print_config=True):
        self.__globalConf = globalConf
        self.__misc = MiscModule.MiscModule()
        self.__logger = LoggerModule.LoggerModule()
        self.__configType = config
        self.__jsonList = jsonList
        self.__specific_type = specific_type
        self.DATA = dict()
        self.__print_config = print_config
        self.parseJson()

    def __checkOverriding(self, child):
        isOverridden = False
        external_config = self.__globalConf.get("EXTERNAL_CONFIGURATION")
        if external_config is not None:
            if "Configuration" in external_config and self.__configType in external_config["Configuration"] and child in external_config["Configuration"][self.__configType]:
                override_value = external_config["Configuration"][self.__configType][child]
                previous_value = getattr(self, child, external_config["Configuration"][self.__configType][child])
                if override_value != previous_value:
                    # for timeouts, replace only if overridden value is greater than original one
                    if self.__configType == "Flash" and child == "INTERNAL_DOWNLOAD":
                        try:
                            updated_override_value = copy.deepcopy(previous_value)
                            for index in range(len(override_value)):
                                for item in override_value[index]:
                                    if isinstance(override_value[index][item], dict):
                                        if not updated_override_value[index].get(item):
                                            updated_override_value[index][item] = {}
                                        for element in override_value[index][item]:
                                            updated_override_value[index][item][element] = override_value[index][item][element]
                                    else:
                                        updated_override_value[index][item] = override_value[index][item]
                        except Exception as e:
                            self.__logger.printLog("WARNING", "issue with override value '{0}' (error={1})".format(override_value, e))
                        else:
                            setattr(self, child, updated_override_value)
                            isOverridden = True
                    elif isinstance(previous_value, int) and override_value > previous_value:
                        setattr(self, child, override_value)
                        isOverridden = True
                    elif not isinstance(previous_value, int):
                        setattr(self, child, override_value)
                        isOverridden = True
                    if isOverridden and self.__print_config:
                        self.__logger.printLog("INFO", "{2}.{0}={1} (original value '{3}' overridden by external-framework value)".format(child, str(getattr(self, child)), self.__configType, previous_value))
        return isOverridden

    def parseJson(self):
        for i in range(len(self.__jsonList)):
            try:
                with open(str(self.__jsonList[i])) as f:
                    json_data = json.load(f)
                data_root = json_data["Configuration"]
            except Exception as e:
                self.__logger.printLog("WARNING", "failure to parse json: {0} ({1})".format(self.__jsonList[i], e))
                continue
            if self.__configType in data_root and data_root[self.__configType]:
                element = data_root[self.__configType]
                for child in element:
                    if self.__specific_type == "WA":
                        # WA format
                        for data_type in ("TC", "Function", "TestCase", "Step"):
                            if child == data_type:
                                if data_type not in self.DATA:
                                    self.DATA[data_type] = list()
                                if not isinstance(element[child], list):
                                    element[child] = [element[child]]
                                for entry in element[child]:
                                    skip = entry.get("Skip", False)
                                    pattern = entry.get("Pattern", "")
                                    issue = entry.get("Issue", "")
                                    # print log
                                    if i == (len(self.__jsonList) - 1):
                                        outputName = os.path.basename(self.__jsonList[i])
                                    else:
                                        dirName = os.path.basename(os.path.dirname(self.__jsonList[i]))
                                        fileName = os.path.basename(self.__jsonList[i])
                                        outputName = os.path.join(dirName, fileName)
                                    if any(single_element["Name"] == entry.get("Name") for single_element in self.DATA[data_type]):
                                        continue
                                    self.DATA[data_type].append({"Name": entry.get("Name"), "Pattern": pattern, "Skip": skip, "Origin": outputName, "Issue": issue})
                                    if self.__print_config:
                                        self.__logger.printLog("INFO", "{0}.DATA['{1}']={2}".format(self.__configType, data_type, self.DATA[data_type][-1]))
                    else:
                        # if setting is not already done
                        if not hasattr(self, child):
                            setattr(self, child, element[child])
                            # print log
                            if i == (len(self.__jsonList) - 1):
                                outputName = os.path.basename(self.__jsonList[i])
                            else:
                                dirName = os.path.basename(os.path.dirname(self.__jsonList[i]))
                                fileName = os.path.basename(self.__jsonList[i])
                                outputName = os.path.join(dirName, fileName)
                            # overriding mechanism
                            if not self.__checkOverriding(child) and self.__print_config:
                                self.__logger.printLog("INFO", "{3}.{0}={1} (from: {2})".format(child, str(getattr(self, child)), outputName, self.__configType))