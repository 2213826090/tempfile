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
@summary: Pupdr Library - CampaignModule
@since: 11/17/2014
@author: travenex
"""

import os
import json
import time
import socket
import datetime
import platform
import MiscModule
import LoggerModule
import HostModule
import LogsModule
import DeviceModule
import ConfigurationModule

class CampaignModule(object):

    instance = None
    globalConf = None
    __logger = None
    __misc = None
    __host = None
    Campaign_information = None

    def __new__(cls):
        """
        Constructor that always returns the same instance
        """
        if not cls.instance:
            cls.instance = object.__new__(cls)
        return cls.instance

    def init(self, globalConf=None):
        self.globalConf = globalConf
        self.__logger = LoggerModule.LoggerModule()
        self.__misc = MiscModule.MiscModule()
        self.__host = HostModule.HostModule()
        self.Campaign_information = CampaignInformation(self.globalConf)

#------------------------------------------------------------------------------#

class TCRDataHandler():

    def __init__(self, globalConf):
        self.__globalConf = globalConf
        self.__TCR_dict = dict()
        self.__metrics = []
        self.__logger = LoggerModule.LoggerModule()

    def getDictionary(self):
        return self.__TCR_dict

    def getMetrics(self):
        return self.__metrics

    def addMetric(self, name, value, unit):
        if not isinstance(name, str) or not (isinstance(value, float) or isinstance(value, int)) or not isinstance(unit, str):
            self.__logger.printLog("WARNING", "addMetric(): invalid input data {}, {}, {}".format(name, value, unit))
        else:
            self.__metrics.append({
                "name": name,
                "value": float("{0:.2f}".format(value)),
                "unit": unit
            })
            self.__logger.printLog("DEBUG", "addMetric(): adding {}, {}, {}".format(name, value, unit))

    def setCampaignData(self, dictionary):
        if not isinstance(dictionary, dict):
            return
        self.__TCR_dict["ssn"] = dictionary.get("SSN", self.__TCR_dict.get("ssn", ""))
        self.__TCR_dict["build"] = dictionary.get("Build", self.__TCR_dict.get("build", ""))
        self.__TCR_dict["host"] = dictionary.get("Host", self.__TCR_dict.get("host", ""))
        self.__TCR_dict["boardType"] = dictionary.get("Board_type", self.__TCR_dict.get("boardType", ""))
        self.__TCR_dict["os"] = dictionary.get("Os", self.__TCR_dict.get("os", ""))
        self.__TCR_dict["codeVersion"] = dictionary.get("Code_Version", self.__TCR_dict.get("codeVersion", ""))
        self.__TCR_dict["frameworkVersion"] = dictionary.get("Framework_Version", self.__TCR_dict.get("frameworkVersion", "unknown"))
        self.__TCR_dict["buildTarget"] = dictionary.get("Build_Target", self.__TCR_dict.get("buildTarget", ""))
        self.__TCR_dict["buildVariant"] = dictionary.get("build_Variant", self.__TCR_dict.get("buildVariant", ""))
        self.__TCR_dict["buildUrl"] = dictionary.get("Build_Url", self.__TCR_dict.get("buildUrl", ""))
        self.__TCR_dict["pftVersion"] = dictionary.get("pftVersion", self.__TCR_dict.get("pftVersion", ""))
        self.__TCR_dict["ifwiVersion"] = dictionary.get("ifwiVersion", self.__TCR_dict.get("ifwiVersion", ""))

    def setTCSummary(self, summary=""):
        self.__TCR_dict["summary"] = summary

#------------------------------------------------------------------------------#

class TCInformation():
    """ Structure that stores current TC info
    """
    def __init__(self):
        self.host_start_time = ""
        self.host_end_time = ""
        self.device_start_time = ""
        self.device_end_time = ""
        self.host_start_date = None
        self.host_end_date = None
        self.verdict = ""
        self.output = ""
        self.device_start_time_seconds = ""
        self.device_end_time_seconds = ""
        self.__host = HostModule.HostModule()

    def start(self):
        self.host_start_date = datetime.datetime.now()
        self.host_start_time = time.strftime("%Y-%m-%d %H:%M:%S")
        self.device_start_time = self.__host.commandExecAdb('shell "date \\"+%Y-%m-%d %H:%M:%S\\""')[1]
        self.device_start_time_seconds = self.__host.commandExecAdb('shell date "+%s"')[1]

    def stop(self, verdict, output):
        self.host_end_date = datetime.datetime.now()
        self.host_end_time = time.strftime("%Y-%m-%d %H:%M:%S")
        self.device_end_time = self.__host.commandExecAdb('shell "date \\"+%Y-%m-%d %H:%M:%S\\""')[1]
        self.device_end_time_seconds = self.__host.commandExecAdb('shell date "+%s"')[1]
        self.verdict = verdict
        self.output = output

#------------------------------------------------------------------------------#

class CampaignInformation():
    """ Class that creates and updates an xml file for quick failure analysis
    """

    def __init__(self, globalConf):
        self.__logger = LoggerModule.LoggerModule()
        self.__device = DeviceModule.DeviceModule()
        self.__host = HostModule.HostModule()
        self.__misc = MiscModule.MiscModule()
        self.__logs = LogsModule.LogsModule()
        self.__configuration = ConfigurationModule.ConfigurationModule()
        self.__globalConf = globalConf
        self.current_TC = TCInformation()
        self.__json_path = ""
        self.__json_data = dict()
        self.__json_top_key = "BootOtaCampaignData"
        self.__json_name = self.__json_top_key + ".json"
        self.TCR_data_handler = TCRDataHandler(self.__globalConf)

    def __updateInfoWithSystemData(self, parameter):
        tag = parameter[0]
        flash_key = parameter[1]
        data = parameter[2]
        log = "updateInfoWithSystemData(): "

        if not data:
            self.__logger.printLog("WARNING", log + "nothing to do, no input data")
        else:
            if self.__logs.tc_name_number not in self.__json_data["flash_data"]:
                self.__json_data["flash_data"][self.__logs.tc_name_number] = {}
            if tag not in self.__json_data["flash_data"][self.__logs.tc_name_number]:
                self.__json_data["flash_data"][self.__logs.tc_name_number][tag] = {}
            if flash_key not in self.__json_data["flash_data"][self.__logs.tc_name_number][tag]:
                self.__json_data["flash_data"][self.__logs.tc_name_number][tag][flash_key] = {}
            self.__json_data["flash_data"][self.__logs.tc_name_number][tag][flash_key]["system_partition"] = data

    def __updateInfoWithWaitOsData(self, parameter):
        # parameter = [duration, timeout, expected_os, actual_os, output, reason]
        duration = parameter[0]
        timeout = parameter[1]
        expected_os = parameter[2]
        actual_os = parameter[3]
        output = parameter[4]
        reason = parameter[5]

        if expected_os not in self.__json_data["boot_data"]:
            self.__json_data["boot_data"][expected_os] = []

        data = {
            "duration": "{0:.2f}".format(duration),
            "timeout": "{0:.2f}".format(timeout),
            "TC": self.__logs.tc_name_number
        }
        if actual_os != expected_os:
            data["actual_os"] = str(actual_os)
        if output:
            data["output"] = str(output)
        if reason:
            data["reason"] = str(reason)
        self.__json_data["boot_data"][expected_os].append(data)

    def __updateInfoWithFirstBootTimeData(self, parameter):
        # parameter = [boot_time, flash_key]
        boot_time = parameter[0]
        flash_key = parameter[1]
        tag = parameter[2]
        if self.__logs.tc_name_number not in self.__json_data["flash_data"]:
            self.__json_data["flash_data"][self.__logs.tc_name_number] = {}
        if tag not in self.__json_data["flash_data"][self.__logs.tc_name_number]:
            self.__json_data["flash_data"][self.__logs.tc_name_number][tag] = {}
        if flash_key not in self.__json_data["flash_data"][self.__logs.tc_name_number][tag]:
            self.__json_data["flash_data"][self.__logs.tc_name_number][tag][flash_key] = {}
        self.__json_data["flash_data"][self.__logs.tc_name_number][tag][flash_key]["first_boot_time"] = "{0:.2f}".format(boot_time)

    def __updatePftVersion(self, parameter):
        # parameter = [boot_time, flash_key]
        self.__json_data["campaign"]["pftVersion"] = parameter[0]

    def __updateIfwiVersion(self, parameter):
        # parameter = [boot_time, flash_key]
        self.__json_data["campaign"]["ifwiVersion"] = parameter[0]

    def __updateInfoWithPftTimeData(self, parameter):
        # parameter = [boot_time, flash_file]
        zip_file = parameter[1]
        boot_time = parameter[0]
        configuration = parameter[2]
        flash_file = parameter[3]
        tag = parameter[4]
        flash_key = parameter[5]

        data = {
            "duration": "{0:.2f}".format(boot_time),
            "zip_file": zip_file,
            "flash_file": flash_file,
            "configuration": configuration,
        }
        if self.__logs.tc_name_number not in self.__json_data["flash_data"]:
            self.__json_data["flash_data"][self.__logs.tc_name_number] = {}
        if tag not in self.__json_data["flash_data"][self.__logs.tc_name_number]:
            self.__json_data["flash_data"][self.__logs.tc_name_number][tag] = {}
        if flash_key not in self.__json_data["flash_data"][self.__logs.tc_name_number][tag]:
            self.__json_data["flash_data"][self.__logs.tc_name_number][tag][flash_key] = {}
        if "PFT_commands" not in self.__json_data["flash_data"][self.__logs.tc_name_number][tag][flash_key]:
            self.__json_data["flash_data"][self.__logs.tc_name_number][tag][flash_key]["PFT_commands"] = []
        self.__json_data["flash_data"][self.__logs.tc_name_number][tag][flash_key]["PFT_commands"].append(data)

    def __updateInfoWithCurrentTc(self):
        """ update class and parser values
        """
        self.__json_data["campaign"]["TC_executed"] += 1
        # get expected TC results.
        if self.current_TC.verdict == "FAILURE":
            self.__json_data["campaign"]["TC_failure"] += 1
        elif self.current_TC.verdict == "BLOCKED":
            self.__json_data["campaign"]["TC_blocked"] += 1
        elif self.current_TC.verdict == "INVALID":
            self.__json_data["campaign"]["TC_invalid"] += 1
        elif self.current_TC.verdict == "VALID":
            self.__json_data["campaign"]["TC_valid"] += 1
        else:
            self.__json_data["campaign"]["TC_success"] += 1
        # in case of failure/blocked, also add new entry in list of failures
        if not any(element is None for element in [self.current_TC.host_end_date, self.current_TC.host_start_date]):
            time_diff = self.current_TC.host_end_date - self.current_TC.host_start_date
            formatted_time_diff = self.__misc.int2StringDelay(time_diff.total_seconds())
        else:
            formatted_time_diff = ""
        if self.current_TC.verdict not in ["SUCCESS", "VALID"]:
            data = {
                "TC": self.__logs.tc_name_number,
                "Verdict": self.current_TC.verdict,
                "Output": self.current_TC.output,
                "StartDateHost": self.current_TC.host_start_time,
                "EndDateHost": self.current_TC.host_end_time,
                "StartDateDevice": self.current_TC.device_start_time,
                "EndDateDevice": self.current_TC.device_end_time,
                "ExecutionTime": formatted_time_diff
            }
            try:
                data["StartDateDeviceSec"] = int(self.current_TC.device_start_time_seconds) * 1000
            except:
                data["StartDateDeviceSec"] = 0
            try:
                data["StartDateDeviceSec"] = int(self.current_TC.device_end_time_seconds) * 1000
            except:
                data["EndDateDeviceSec"] = 0
            self.__json_data["failure"].append(data)
        elif self.current_TC.verdict == "VALID":
            self.__json_data["success"].append({"TC": self.__logs.tc_name_number + " (VALID)",
                                                "ExecutionTime": formatted_time_diff})
        else:
            self.__json_data["success"].append({"TC": self.__logs.tc_name_number,
                                                "ExecutionTime": formatted_time_diff})
        if not self.__json_data["campaign"]["SSN"]:
            self.__json_data["campaign"]["SSN"] = \
                self.__device.getProperty("ro.serialno")
        if not self.__json_data["campaign"]["Build"]:
            self.__json_data["campaign"]["Build"] = \
                self.__globalConf.get("BUILD_NAME", "")
        if not self.__json_data["campaign"]["Board_type"]:
            self.__json_data["campaign"]["Board_type"] = \
                self.__configuration.board
        if not self.__json_data["campaign"]["Build_Target"]:
            self.__json_data["campaign"]["Build_Target"] = \
                self.__globalConf.get("PROVISIONING_DATA", {}).get("build_target", "")
        if not self.__json_data["campaign"]["build_Variant"]:
            self.__json_data["campaign"]["build_Variant"] = \
                self.__globalConf.get("PROVISIONING_DATA", {}).get("build_variant", "")
        if not self.__json_data["campaign"]["Build_Url"]:
            self.__json_data["campaign"]["Build_Url"] = \
                self.__globalConf.get("PROVISIONING_DATA", {}).get("url_buildinfo", "")
        if not self.__json_data["campaign"]["Code_Version"]:
            self.__json_data["campaign"]["Code_Version"] = \
                self.__globalConf.get("BOOTOTA_LIBRARY_CODE_VERSION", "")
        if not self.__json_data["campaign"]["Framework_Version"]:
            self.__json_data["campaign"]["Framework_Version"] = \
                self.__globalConf.get("FRAMEWORK_CODE_VERSION", "")

    def __openJson(self):
        """ upload current xml info and update class inner variables
        """
        try:
            with open(self.__json_path) as f:
                json_data = json.load(f)
        except Exception as e:
            self.__logger.printLog("WARNING", "openJson(): failure to parse {0} (error={1})".format(self.__json_path, e))
            json_data = {}

        if self.__json_top_key not in json_data:
            self.__logger.printLog("WARNING", "openJson(): missing '{1}' key in {0}".format(self.__json_path, self.__json_top_key))
            self.__json_data = {}
        else:
            self.__json_data = json_data[self.__json_top_key]

    def __closeJson(self):
        """ write pretty xml
        """
        if os.path.exists(self.__json_path):
            os.remove(self.__json_path)
        with open(self.__json_path, "w") as local_file:
            local_file.write(json.dumps({self.__json_top_key: self.__json_data}, sort_keys=True, indent=4))

    def __createNewJson(self):
        """ create new and empty xml file
        """
        json_data = dict()
        json_data["campaign"] = {
            "SSN": self.__device.getProperty("ro.serialno"),
            "Build": self.__globalConf.get("BUILD_NAME", ""),
            "Host": socket.gethostname(),
            "Board_type": self.__configuration.board,
            "Os": " - ".join((platform.system(), platform.release())),
            "Start": time.strftime("%Y-%m-%d %H:%M:%S"),
            "Code_Version": self.__globalConf.get("BOOTOTA_LIBRARY_CODE_VERSION", ""),
            "Framework_Version": self.__globalConf.get("FRAMEWORK_CODE_VERSION", ""),
            "Build_Target": self.__globalConf.get("PROVISIONING_DATA", {}).get("build_target", ""),
            "build_Variant": self.__globalConf.get("PROVISIONING_DATA", {}).get("build_variant", ""),
            "Build_Url": self.__globalConf.get("PROVISIONING_DATA", {}).get("url_buildinfo", ""),
            "TC_executed": 0,
            "TC_success": 0,
            "TC_failure": 0,
            "TC_blocked": 0,
            "TC_invalid": 0,
            "TC_valid": 0
        }
        json_data["failure"] = []
        json_data["success"] = []
        json_data["flash_data"] = {}
        json_data["boot_data"] = {}
        with open(self.__json_path, "w") as local_file:
            local_file.write(json.dumps({self.__json_top_key: json_data}, sort_keys=True, indent=4))

    def updateJsonWithTcInformationData(self, verdict, output):
        """ update sequence launched in final step
        """
        log = "updateJsonWithTcInformationData(): "
        self.current_TC.stop(verdict, output)
        self.__updateJsonStructure(self.__updateInfoWithCurrentTc, log)

    def updateJsonWithPftVersion(self, version):
        """ update sequence launched in final step
        """
        log = "updateJsonWithPftVersion(): "
        self.__updateJsonStructure(self.__updatePftVersion, log, param=[version])

    def updateJsonWithIfwiVersion(self, version):
        """ update sequence launched in final step
        """
        log = "updateJsonWithIfwiVersion(): "
        self.__updateJsonStructure(self.__updateIfwiVersion, log, param=[version])

    def updateJsonWithFirstBootTimeData(self, boot_time, key, tag):
        """ update sequence launched in final step
        """
        log = "updateJsonWithFirstBootTimeData(): "
        self.__updateJsonStructure(self.__updateInfoWithFirstBootTimeData, log, param=[boot_time, key, tag])

    def updateJsonWithPftTimeData(self, duration, zip_file, configuration, flash_file, tag, flash_key):
        """ update sequence launched in final step
        """
        log = "updateJsonWithPftTimeData(): "
        self.__updateJsonStructure(self.__updateInfoWithPftTimeData, log, param=[duration, zip_file, configuration, flash_file, tag, flash_key])

    def updateJsonWithSystemData(self, tag, flash_key, data):
        """ update sequence launched in final step
        """
        log = "updateJsonWithSystemData(): "
        self.__updateJsonStructure(self.__updateInfoWithSystemData, log, param=[tag, flash_key, data])

    def updateJsonWithWaitOsData(self, duration, timeout, expected_os, actual_os, output, reason):
        """ update sequence launched in final step
        """
        log = "updateJsonWithWaitOsData(): "
        self.__updateJsonStructure(self.__updateInfoWithWaitOsData, log, param=[duration, timeout, expected_os, actual_os, output, reason])

    def createCampaignJson(self):
        """ update sequence launched in final step
        """
        log = "createCampaignJson(): "
        self.__updateJsonStructure(None, log)

    def getJsonData(self):
        log = "getJsonData(): "
        self.__updateJsonStructure(None, log)
        return self.__json_data

    def __updateJsonStructure(self, update_method, log="", param=None):
        if not self.__logs.bootota_log_path:
            self.__logger.printLog("INFO", log + "skipped (because 'REPORT_PATH' not provided in global configuration)")
        else:
            try:
                if param:
                    self.__logger.printLog("INFO", log + "start with parameters={0}".format(param))
                else:
                    self.__logger.printLog("INFO", log + "start")
                # open json file
                self.__json_path = os.path.join(self.__logs.bootota_log_path, self.__json_name)
                if not os.path.exists(self.__json_path):
                    self.__logger.printLog("INFO", log + "creating new json campaign file at {0}".format(self.__json_path))
                    self.__createNewJson()
                self.__openJson()
                if not update_method:
                    return
                if not self.__json_data:
                    self.__logger.printLog("WARNING", log + "issue with json loading, aborting")
                    return
                if param:
                    update_method(param)
                else:
                    update_method()
                self.__closeJson()
                # push campaign json data to TCR dictionary for all TCs
                self.TCR_data_handler.setCampaignData(self.__json_data["campaign"].copy())
                self.__logger.printLog("INFO", log + "a new entry has been made in '{0}'".format(self.__json_name))
            except Exception as e:
                self.__logger.printLog("WARNING", log + "issue while updating json campaign file (error={0})".format(e))

    def sendFlashDataEmail(self):
        log = "sendFlashDataEmail(): "
        self.__logger.printLog("INFO", log + "sending email disabled")
        # try:
        #     table = "<table>\n"
        #     email_core_text = "<h2>Campaign Data</h2>\n<table>\n"
        #     for tag in ("Build","Board", "Host", "Campaign_Start_Date", "SSN", "Os", "Code_Version"):
        #         email_core_text += "<tr><td>{0}</td><td>{1}</td></tr>\n".format(tag, self.__xml_root.find("Campaign").find(tag).get("value"))
        #     for tag in ("Build","Board"):
        #         table += "<tr><td>{0}</td><td>{1}</td></tr>\n".format(tag, self.__xml_root.find("Campaign").find(tag).get("value"))
        #
        #     email_core_text += "</table>\n<h2>Detailed Flash Data</h2>\n<h3>Phone Flash Tool Execution Duration</h3><table>\n"
        #     email_core_text += "<tr><td>Zip File</td><td>Flash Time</td></tr>\n"
        #     for element in self.__xml_root.find("FlashData").find("PFT_command").find(self.__logs.tc_name_number):
        #         email_core_text += "<tr><td>{1}</td><td>{0}s</td></tr>\n".format(element.find("flash_0").get("value"), element.tag)
        #         table += "<tr><td>{1} flash time</td><td>{0}s</td></tr>\n".format(element.find("flash_0").get("value"), element.tag)
        #
        #     email_core_text += "</table>\n<h3>First Boot Data</h3>\n<table>\n"
        #     flash_key = self.__xml_root.find("FlashData").find("First_boot").find(self.__logs.tc_name_number).find("Data").get("flash_key")
        #     email_core_text += "<tr><td>Flash key</td><td>{0}</td></tr>\n".format(flash_key)
        #     boot_time = self.__xml_root.find("FlashData").find("First_boot").find(self.__logs.tc_name_number).find("Data").get("boot_time")
        #     email_core_text += "<tr><td>1st MOS boot time</td><td>{0}s</td></tr>\n".format(boot_time)
        #     table += "<tr><td>1st MOS boot time</td><td>{0}s</td></tr>\n".format(boot_time)
        #
        #     email_core_text += "</table>\n<h3>System Partition Data</h3>\n<table>\n"
        #     for element in self.__xml_root.find("FlashData").find("System_partition").find(self.__logs.tc_name_number):
        #         email_core_text += "<tr><td>/system - {0}</td><td>{1}</td></tr>\n".format(element.tag, element.get("value"))
        #         table += "<tr><td>/system - {0}</td><td>{1}</td></tr>\n".format(element.tag, element.get("value"))
        #     email_core_text += "</table>\n"
        #     table += "</table>\n"
        #     subject = "Flash Data for {0} - {1} on host {2} ".format(self.__xml_root.find("Campaign").find("Build").get("value"),
        #                                                    self.__xml_root.find("Campaign").find("Board").get("value"),
        #                                                    self.__xml_root.find("Campaign").find("Host").get("value"))
        #
        #     html_content = """<html>
        #                     <head>
        #                     <style>
        #                     table, th, td {
        #                         border: 1px solid black;
        #                         border-collapse: collapse;
        #                     }
        #                     </style>
        #                     </head>
        #                     <body>""" + \
        #                    "<h2>Report Data</h2>" + table +\
        #                     email_core_text +\
        #                     "</body></html>"
        #
        #     self.__misc.sendMail(subject, html_content, MIMEType="html", format_html=False)
        # except Exception as e:
        #     self.__logger.printLog("WARNING", log + "failure to format or send email ({0})".format(e))