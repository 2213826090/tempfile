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

import re
import os
import json
import HostModule
import RelayCardModule
import DeviceModule
import WorkaroundModule
import MiscModule
import OsManagerModule
import FlashFileModule
import LogsModule
import EfiVarModule
import DediprogModule
import OutputModule
import ConfigurationModule
import LoggerModule
import BootDataModule
import ResetIrqModule
import WatchdogModule
import DownloadModule
import FlashModule
import CampaignModule
import ReportModule
import EventsModule

class TestCaseModule(object):

    instance = None
    globalConf = None
    logger = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.instance:
            cls.instance = object.__new__(cls)
        return cls.instance

    def init(self, globalConf=None):
        self.globalConf = globalConf
        self.logger = LoggerModule.LoggerModule()

    class TC_structure:
        def __init__(self, conf):
            self.globalConf = conf
            self.logger = LoggerModule.LoggerModule()
            self.host = HostModule.HostModule()
            self.relayCard = RelayCardModule.RelayCardModule()
            self.device = DeviceModule.DeviceModule()
            self.workaround = WorkaroundModule.WorkaroundModule()
            self.misc = MiscModule.MiscModule()
            self.osManager = OsManagerModule.OsManagerModule()
            self.flashFile = FlashFileModule.FlashFileModule()
            self.logs = LogsModule.LogsModule()
            self.efiVar = EfiVarModule.EfiVarModule()
            self.dediprog = DediprogModule.DediprogModule()
            self.output = OutputModule.OutputModule()
            self.configuration = ConfigurationModule.ConfigurationModule()
            self.bootData = BootDataModule.BootDataModule()
            self.resetIrq = ResetIrqModule.ResetIrqModule()
            self.download = DownloadModule.DownloadModule()
            self.flash = FlashModule.FlashModule()
            self.campaign = CampaignModule.CampaignModule()
            self.watchdog = WatchdogModule.WatchdogModule()
            self.report = ReportModule.ReportModule()
            self.events = EventsModule.EventsModule()
            self.step_list = list()
            self.verdict = True
            self.output = ""
            self.name = ""
            self.skipping = False
            self.tc_crashlogs = None
            self.allowed_TC_upon_success = []
            self.removed_TC_upon_success = []
            self.allowed_TC_upon_failure = []
            self.removed_TC_upon_failure = []
            self.allowed_TC_upon_skip = []
            self.removed_TC_upon_skip = []
            self.campaign_constraint_file = ""
            self.campaign_constraint_file_name = "PupdrCampaignConstraints.json"
            self.enable_init = True
            self.enable_final = True
            self.INVALID = "INVALID"
            self.VALID = "VALID"
            self.BLOCKED = "BLOCKED"
            self.FAILURE = "FAILURE"
            self.SUCCESS = "SUCCESS"
            self.verdict_name = self.SUCCESS
            self.step_number = 0
            self.output_dict = {}

        def TC_init(self):
            self.logger.testStep = self.name + "TC INIT - "

            # Add start flag in aplog and uart console
            self.logs.printFlags("START")

            # Save start time
            self.campaign.Campaign_information.current_TC.start()

            # Start crashlogs check except for System Flash TC
            if self.configuration.logs.CRASHLOGS:
                self.tc_crashlogs = self.watchdog.HistoryEventHandler(self.globalConf)
                self.tc_crashlogs.start()

            # Print device vars for debug
            self.device.getDumpsysBattery()
            self.device.getChargerType()
            self.device.getAllProperties()

        def TC_final_step(self, verdict=True):
            self.globalConf["TC_END_PHASE"] = True
            self.globalConf["TC_VERDICT_BOOL"] = self.verdict
            # replug in case the TC left things unplugged
            self.relayCard.powerSupply(True)
            if not self.device.adbDevices()[0]:
                self.relayCard.usbConnection(True)
            # go to mos if needed
            currentOs = self.osManager.getOs()
            if "main" not in currentOs:
                if self.osManager.gotoMos():
                    currentOs = "main"
                else:
                    self.logger.printLog("WARNING", "Unable for the DUT to be in MOS")
                    currentOs = self.osManager.getOs()
            elif currentOs == "main" and not self.verdict:
                self.logger.printLog("WARNING", "test case failure but board in MOS, power cycle")
                self.osManager.forceShutdown(plug=False)
                if not self.device.adbDevices()[0]:
                    self.relayCard.powerButtonPress(self.configuration.timeout.PKPON)
                    self.relayCard.usbConnection(True)
                if not self.osManager.waitOs("main", waitWdExp=False):
                    self.osManager.gotoMos()
                    currentOs = self.osManager.getOs()
                else:
                    currentOs = "main"

            # Print battery and charger info
            self.device.getDumpsysBattery()
            self.device.getChargerType()

            # upload data of TC
            if "main" in currentOs:
                self.logs.printFlags("END")
                # Add crashlogs into list of element to pull
                if self.tc_crashlogs:
                    crashlogs = self.tc_crashlogs.crashlogs
                    if len(crashlogs) > 5:
                        self.logger.printLog("INFO", "too many crashlogs found, taking only last 5 ones ({0})".format(", ".join(crashlogs[-5:])))
                        crashlogs = crashlogs[-5:]
                    for crash in crashlogs:
                        self.logs.logsFiles.append(crash)
                    self.logger.printLog("DEBUG", "list of crashlogs found in the history_event : {0}".format(" ".join(crashlogs)))
                    # get ramdump on concerned devices
                    self.watchdog.getRamdump()
                else:
                    self.logger.printLog("DEBUG", "The global TC_CRASHLOG is None")

            # pulling logs
            if self.device.waitAdbDevices(20):
                self.logs.pullAll(verdict=verdict)
            else:
                self.logger.printLog("INFO", "failure to pull TC logs, no adb")

        def __get_campaign_constraint_file(self):
            if not self.campaign_constraint_file:
                if self.logs.bootota_log_path:
                    self.campaign_constraint_file = os.path.join(self.logs.bootota_log_path,
                                                                 self.campaign_constraint_file_name)

        def getCampaignConstraints(self):
            log = "readCampaignConstraints(): "
            self.__get_campaign_constraint_file()
            if not self.campaign_constraint_file or not os.path.isfile(self.campaign_constraint_file):
                self.logger.printLog("INFO", log + "no file found, nothing to do")
                return True
            self.logger.printLog("INFO", log + "start parsing {0}".format(self.campaign_constraint_file))
            try:
                with open(self.campaign_constraint_file) as f:
                    constraint_data = json.load(f)
            except Exception as e:
                self.logger.printLog("WARNING", log + "failure to parse constraint file "
                                                      "{0} (error={1})".format(self.campaign_constraint_file, e))
                return True
            if "all_TCs" in constraint_data["allowed_TC"] or self.misc.getCaller(4) in constraint_data["allowed_TC"]:
                self.logger.printLog("INFO", log + "{0} TC allowed".format(self.misc.getCaller(4)))
                return True
            if "all_TCs" in constraint_data["removed_TC"] or self.misc.getCaller(4) in constraint_data["removed_TC"]:
                self.logger.printLog("INFO", log + "{0} TC removed because of {1} {2} verdict"\
                                     .format(self.misc.getCaller(4), constraint_data["test_case"], constraint_data["reason"]))
                if constraint_data["reason"] == "failure":
                    self.skipping = True
                    return False
                elif constraint_data["reason"] == "skipped":
                    self.skipping = True
                    return True

        def writeCampaignConstraints(self):
            log = "writeCampaignConstraints(): "
            self.__get_campaign_constraint_file()
            if not any([self.allowed_TC_upon_failure, self.allowed_TC_upon_skip,
                        self.allowed_TC_upon_success, self.removed_TC_upon_failure,
                        self.removed_TC_upon_skip, self.removed_TC_upon_success]):
                self.logger.printLog("INFO", log + "no constraint specified for {0}".format(self.misc.getCaller(3)))
                return
            if not self.campaign_constraint_file:
                self.logger.printLog("WARNING", log + "cannot write campaign constraint file "
                                                      "because no report path specified")
                return
            if os.path.isfile(self.campaign_constraint_file):
                os.remove(self.campaign_constraint_file)
            json_data = dict()
            json_data["allowed_TC"] = list()
            json_data["removed_TC"] = list()
            json_data["reason"] = ""
            json_data["test_case"] = self.logs.tc_name_number
            if not self.verdict:
                if self.allowed_TC_upon_failure or self.removed_TC_upon_failure:
                    json_data["allowed_TC"] = self.allowed_TC_upon_failure
                    json_data["removed_TC"] = self.removed_TC_upon_failure
                    json_data["reason"] = "failure"
                else:
                    self.logger.printLog("INFO", log + "nothing to do upon failed TC verdict")
            elif self.skipping:
                if self.allowed_TC_upon_skip or self.removed_TC_upon_skip:
                    json_data["allowed_TC"] = self.allowed_TC_upon_skip
                    json_data["removed_TC"] = self.removed_TC_upon_skip
                    json_data["reason"] = "skipped"
                else:
                    self.logger.printLog("INFO", log + "nothing to do upon skipped TC verdict")
            else:
                if self.allowed_TC_upon_success or self.removed_TC_upon_success:
                    json_data["allowed_TC"] = self.allowed_TC_upon_success
                    json_data["removed_TC"] = self.removed_TC_upon_success
                    json_data["reason"] = "success"
                else:
                    self.logger.printLog("INFO", log + "nothing to do upon successful TC verdict")
            with open(self.campaign_constraint_file, 'w') as f:
                self.logger.printLog("INFO", log + "setting constraints according to {0} "
                                                   "{1} verdict...".format(json_data["test_case"], json_data["reason"]))
                if json_data["allowed_TC"]:
                    self.logger.printLog("INFO", log + "allowed TC list = {0}".format(", ".join((json_data["allowed_TC"]))))
                else:
                    self.logger.printLog("INFO", log + "no constraint on allowed TC")
                if json_data["removed_TC"]:
                    self.logger.printLog("INFO", log + "removed TC list = {0}".format(", ".join((json_data["removed_TC"]))))
                else:
                    self.logger.printLog("INFO", log + "no constraint on removed TC")
                self.logger.printLog("INFO", log + "writing file {0}".format(self.campaign_constraint_file))
                f.write(json.dumps(json_data, sort_keys=True, indent=4))
                f.close()

        def toggleVerdict(self, verdict):
            log = "toggleVerdict(): "
            self.logger.printLog("INFO", log + "updating TC verdict to {}".format(verdict))
            if verdict not in [self.BLOCKED, self.SUCCESS, self.FAILURE, self.INVALID, self.VALID]:
                self.logger.printLog("WARNING", log + "unknown verdict")
            else:
                self.verdict_name = verdict
                if self.verdict_name in [self.FAILURE, self.INVALID]:
                    self.verdict = False
                else:
                    self.verdict = True

        def runStep(self, testStep, parameter=None, skip=False, run_on_failure=False,
                    impact_tc_verdict=True):
            local_step = {
                "step": testStep(self.globalConf),
                "output": "",
                "output_list": []
            }
            local_verdict = True
            local_step["verdict"] = "NOT EXECUTED"
            self.step_number += 1
            self.logger.testStep = self.name + "STEP {0} - ".format(self.step_number)
            self.logger.printLog("INFO", "***{0}***".format("*" * (len(local_step["step"].name) + 12)))
            self.logger.printLog("INFO", "**  Starting: {0}  **".format(local_step["step"].name))
            self.logger.printLog("INFO", "*  {0}  *".format(" " * (len(local_step["step"].name) + 12)))

            if not self.verdict and not run_on_failure:
                output = "TC verdict is failure and step only executed upon TC success"
                self.logger.printLog("INFO", output)
                self.logger.printLog("INFO", "***{0}***".format(" " * (len(output) + 2)))
                self.logger.printLog("INFO", "*** {0} ***".format(output))
                self.logger.printLog("INFO", "***{0}***".format("*" * (len(output) + 2)))
                local_step["output"] = output
            elif skip or self.skipping:
                local_step["verdict"] = "SKIPPED"
                output = "Step skipped because previous step made all end sequence skipped"
                self.logger.printLog("INFO", output)
                self.logger.printLog("INFO", "***{0}***".format(" " * (len(output) + 2)))
                self.logger.printLog("INFO", "*** {0} ***".format(output))
                self.logger.printLog("INFO", "***{0}***".format("*" * (len(output) + 2)))
                local_step["output"] = output
            else:
                stepInfo = "Step Info: {0}".format(local_step["step"].description)
                self.logger.printLog("INFO", stepInfo)
                if parameter:
                    local_step["step"](parameter)
                else:
                    local_step["step"]()
                verdict = local_step["step"].verdict
                self.skipping = local_step["step"].skip
                local_step["isWorkaround"] = local_step["step"].isWorkaround
                local_step["output"] = local_step["step"].output
                local_step["output_list"] = local_step["step"].output_list
                if local_step["step"].output:
                    self.logger.printLog("INFO", "Step output: {0}".format(local_step["step"].output))
                if verdict:
                    if local_step["isWorkaround"]:
                        local_step["verdict"] = "PASS (WORKAROUND APPLIED)"
                        output = "Step successful (Workaround Applied)"
                        self.logger.printLog("INFO", "*  {0}  *".format(" " * (len(output) + 2)))
                        self.logger.printLog("INFO", "**  {0}  **".format(output))
                        self.logger.printLog("INFO", "***{0}***".format("*" * (len(output) + 2)))
                    else:
                        local_step["verdict"] = "PASS"
                        output = "Step successful"
                        self.logger.printLog("INFO", "*  {0}  *".format(" " * (len(output) + 2)))
                        self.logger.printLog("INFO", "**  {0}  **".format(output))
                        self.logger.printLog("INFO", "***{0}***".format("*" * (len(output) + 2)))
                else:
                    local_step["verdict"] = "FAIL"
                    if impact_tc_verdict:
                        self.toggleVerdict(self.FAILURE)
                    else:
                        self.logger.printLog("DEBUG", "step verdict not impacting TC verdict")
                    local_verdict = False
                    output = "Step failure"
                    self.logger.printLog("INFO", "*  {0}  *".format(" " * (len(output) + 2)))
                    self.logger.printLog("INFO", "**  {0}  **".format(output))
                    self.logger.printLog("INFO", "***{0}***".format("*" * (len(output) + 2)))

            self.step_list.append(local_step)
            return local_verdict

        # method containing test code and defined at children level
        def runAllSteps(self, **kwargs):
            pass

        def __call__(self, **kwargs):
            self.globalConf["DEFAULT_INIT"] = False
            self.logger.testStep = self.name
            self.logs.createReport()
            if self.logs.tc_name_number:
                self.name = self.logs.tc_name_number + " - "
                self.logger.testStep = self.name

            is_workaround, wa_output = self.workaround.isWorkaround(data_type=self.workaround.TestCase, name=self.misc.getCaller(2), check_skip_only=True)
            if is_workaround:
                self.output = wa_output
                self.verdict_name = self.VALID
                self.skipping = True
                self.output_dict["valid_log"] = self.output
                self.enable_init = False

            if self.enable_init:
                self.TC_init()
            else:
                self.logger.printLog("INFO", "TC init skipped")

            self.logger.printLog("INFO", "****************")
            self.logger.printLog("INFO", "*** starting ***")
            self.logger.printLog("INFO", "****************")

            self.runAllSteps(**kwargs)

            self.logger.testStep = self.name
            self.logger.printLog("INFO", "finished")
            self.displayTcExecution()
            self.buildOutput()
            self.logger.testStep = self.name.replace(" - ", " ") + "({}) TC FINAL - ".format(self.verdict_name)
            self.writeCampaignConstraints()
            if self.enable_final and self.verdict_name not in [self.INVALID, self.VALID]:
                self.TC_final_step(verdict=self.verdict)
            else:
                self.logger.printLog("INFO", "TC final skipped")
            # Push data on TCR
            self.campaign.Campaign_information.updateJsonWithTcInformationData(verdict=self.verdict_name, output=self.output)
            self.report.TCRDataPush(self.campaign.Campaign_information.TCR_data_handler.getDictionary())
            self.report.TCRMetricsPush(self.campaign.Campaign_information.TCR_data_handler.getMetrics())
            # set tag of PUPDR log in FWK
            self.logger.testStep = self.name.replace(" - ", " ") + "({}) FWK FINAL - ".format(self.verdict_name)

            # erasing TC temporary files
            self.misc.local_files_handler.cleanEntries()

        def buildOutput(self):
            if not self.output:
                outputList = list()
                for index, element in enumerate(self.step_list):
                    line = "{0}: {1}".format(element["step"].name, element["verdict"])
                    if element["verdict"] == "FAIL":
                        line += " ({0})".format(element["output"])
                    outputList.append(line)
                self.output = " ||| ".join(outputList)
            self.output_dict.update(
                {
                    "output_log": self.output,
                    "verdict_name": self.verdict_name,
                    "verdict": self.verdict
                }
            )

        def displayTcExecution(self):
            log = "TC Execution Summary"
            printText = "*** {0} ***".format(log)
            outputLog = "\n  {0}  ".format("*" * (len(printText) - 4)) + "\n"
            outputLog += " {0} ".format("*" * (len(printText) - 2)) + "\n"
            outputLog += "{0}".format("*" * len(printText)) + "\n"
            outputLog += "{0}".format(printText) + "\n"
            outputLog += "{0}".format(printText.replace(log, "{0}".format(" " * len(log)))) + "\n"
            TCR_output = dict()
            outputLog += "*** Final Verdict: {}\n".format(self.verdict_name)
            TCR_output["Verdict"] = self.verdict_name
            if self.verdict_name == self.INVALID and self.output_dict.get("invalid_log"):
                TCR_output["Invalid Reason"] = self.output_dict.get("invalid_log")
                outputLog += "*** Invalid Because: {}\n".format(self.output_dict.get("invalid_log"))
            if self.verdict_name == self.VALID and self.output_dict.get("valid_log"):
                TCR_output["Valid Reason"] = self.output_dict.get("valid_log")
                outputLog += "*** Valid Because: {}\n".format(self.output_dict.get("valid_log"))
            outputLog += "*** Step By Step Test Detail:" + "\n"
            for index, element in enumerate(self.step_list):
                line = "*** STEP {0:2} {1:13}".format(index + 1, element["verdict"])
                TCR_single_tc_content = {}
                line += " {0:25} - {1}".format("*** " + element["step"].name, element["step"].description)
                TCR_line = "{0} | {1}".format(element["step"].name, element["step"].description)
                if element["step"].specific:
                    line += " ({0})".format(element["step"].specific)
                    TCR_line += " ({0})".format(element["step"].specific)
                TCR_single_tc_content["info2_Description"] = TCR_line
                if element["output_list"]:
                    for local_index, sub_element in enumerate(element["output_list"]):
                        TCR_single_tc_content["info3_FailureStack{}".format(local_index)] = sub_element
                else:
                    TCR_single_tc_content["info3_Output"] = element["output"]
                if element["output"]:
                    line += " : {0}".format(element["output"])
                outputLog += line + "\n"
                if index >= 9:
                    string_index = str(index + 1)
                else:
                    string_index = "0" + str(index + 1)
                TCR_single_tc_content["info1_Verdict"] = element["verdict"]
                TCR_output["STEP{0}".format(string_index)] = TCR_single_tc_content
            if self.logs.tc_name_number:
                TCR_output["Test Case"] = self.logs.tc_name_number
            outputLog += "{0}".format(printText.replace(log, "{0}".format(" " * len(log)))) + "\n"
            outputLog += "{0}".format("*" * len(printText)) + "\n"
            outputLog += " {0} ".format("*" * (len(printText) - 2)) + "\n"
            outputLog += "  {0}  ".format("*" * (len(printText) - 4))
            if self.verdict:
                self.logger.printLog("INFO", outputLog)
            else:
                self.logger.printLog("ERROR", outputLog)
            self.campaign.Campaign_information.TCR_data_handler.setTCSummary(TCR_output)

    class TC_element:

        def __init__(self, conf):
            self.globalConf = conf
            self.verdict = True
            self.output = ""
            self.output_list = []
            self.name = ""
            self.description = ""
            self.specific = ""
            self.isWorkaround = False
            self.skip = False
            self.backup_name = ""
            self.defaultParameter = dict()
            self.logger = LoggerModule.LoggerModule()
            self.host = HostModule.HostModule()
            self.relayCard = RelayCardModule.RelayCardModule()
            self.device = DeviceModule.DeviceModule()
            self.workaround = WorkaroundModule.WorkaroundModule()
            self.misc = MiscModule.MiscModule()
            self.osManager = OsManagerModule.OsManagerModule()
            self.flashFile = FlashFileModule.FlashFileModule()
            self.logs = LogsModule.LogsModule()
            self.efiVar = EfiVarModule.EfiVarModule()
            self.dediprog = DediprogModule.DediprogModule()
            self.outputModule = OutputModule.OutputModule()
            self.configuration = ConfigurationModule.ConfigurationModule()
            self.bootData = BootDataModule.BootDataModule()
            self.resetIrq = ResetIrqModule.ResetIrqModule()
            self.watchdog = WatchdogModule.WatchdogModule()
            self.download = DownloadModule.DownloadModule()
            self.campaign = CampaignModule.CampaignModule()
            self.flash = FlashModule.FlashModule()
            self.events = EventsModule.EventsModule()

        # method containing test code and defined at children level
        def step(self, parameter):
            pass

        def subTestIndex(self, index):
            if not self.backup_name:
                self.backup_name = self.logger.testStep
            if index == 0:
                self.logger.testStep = self.backup_name
            else:
                self.logger.testStep = re.sub("(- STEP [0-9]+)( -.*$)", r'\1'+" - {0}".format(index)+r'\2', self.backup_name)

        def __call__(self, parameter=None):
            if not parameter:
                parameter = dict()
            parameter = self.checkParameter(parameter, self.defaultParameter)
            self.output = ""
            self.verdict = True
            self.step(parameter)
            self.finalizeOutput()
            # check workaround
            if not self.verdict:
                self.isWorkaround = self.workaround.isWorkaround(data_type=self.workaround.Step, name=self.__class__.__name__, output=self.output)[0]
                if self.isWorkaround:
                    self.verdict = True

        def finalizeOutput(self):
            if not self.verdict and not self.output:
                self.output, self.output_list = self.outputModule.getErrorCallStacks()

        # method to check parameter input (parameter) and assign default value (from checkList) if not provided
        def checkParameter(self, parameter, checkList):
            for element in checkList:
                if element not in parameter:
                    # only print log if assigning not None value
                    if checkList[element] is not None:
                        self.logger.printLog("INFO", "'{0}' parameter update: (actual: {1}, fallback: {2})".format(element, parameter.get(element, None), checkList[element]))
                    # fallback to default parameter value
                    parameter[element] = checkList[element]
            if checkList:
                self.logger.printLog("INFO", "step parameter list: {0}".format(parameter))
            return parameter