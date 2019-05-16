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
@summary: Pupdr Library - Misc
@since: 11/17/2014
@author: travenex
"""

import re
import os
import time
import json
import shutil
import urllib2
import inspect
import zipfile
import LoggerModule
import OutputModule
import HostModule
import smtplib
import getpass
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText

class MiscModule(object):

    __instance = None
    __globalConf = None
    __output = None
    __logger = None
    __host = None
    local_files_handler = None
    extra_path = None

    """
    Global variable used for D2D force cht51 to M on cht_ffd board.
    """
    otaBuild = None

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
        self.__output = OutputModule.OutputModule()
        self.__host = HostModule.HostModule()
        self.local_files_handler = self.__FilesHandler()
        self.extra_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "Extras"))
        self.otaBuild = ""

    def str2bool(self, strbool):
        """ Convert a string into a boolean.
            This function != case sensitive.
        """
        if strbool.lower() == "true":
            return True
        elif strbool.lower() == "false":
            return False
        else:
            self.__logger.printLog("WARNING", "str2bool(): unexpected value: {0}".format(strbool))

    @staticmethod
    def getUser():
        return getpass.getuser()

    def unzip(self, zip_file, mode='r'):
        """ Return zipfile object
        """
        z = None
        try:
            z = zipfile.ZipFile(zip_file, mode)
        except Exception as e:
            self.__logger.printLog("WARNING", "failure to unzip '{0}' ({1})".format(zip_file, e))
        return z

    def zipDirectory(self, zip_file, directory):
        self.__logger.printLog("INFO", "zipDirectory(): zipping '{0}' to '{1}'".format(directory, zip_file))
        relative_path = os.path.abspath(directory)
        with zipfile.ZipFile(zip_file, "w", zipfile.ZIP_DEFLATED) as z:
            for root, dirs, files in os.walk(directory):
                z.write(root, os.path.relpath(root, relative_path))
                for unit_file in files:
                    filename = os.path.join(root, unit_file)
                    # skip pyc files
                    if os.path.isfile(filename) and not filename.endswith(".pyc"):
                        arc_name = os.path.join(os.path.relpath(root, relative_path), unit_file)
                        z.write(filename, arc_name)

    def getPathFile(self, pattern, path_list):
        """ Get the full path of filename by searching in path_list.
            Return None != found.
        """
        if type(path_list) != type(list()):
            path_list = [path_list]

        f = list()

        for path in path_list:
            for root, dirs, files in os.walk(path):
                for name in files:
                    if re.search(pattern, name):
                        f.append(os.path.join(root, name))
                        self.__logger.printLog("INFO", "getPathFile({0}): found ".format(pattern) + f[-1])

        return f if f else None

    def sendMail(self, subject="", text="", From="", To=list(), MIMEType="plain", format_html=True):
        """ Send an email
            MIMEText is expected to be either html or plain
        """

        # Force default values
        subject = "[BootOtaLibrary] {0}".format(subject if subject else "Email from BootOta scripts")
        if not From:
            From = "BootOta Test Team <bootota@intel.com>"

        # Create message container - the correct MIME type is multipart/alternative.
        msg = MIMEMultipart('alternative')
        msg['Subject'] = subject
        msg['From']    = From
        msg['To'] = ""
        for email_address in To:
            if email_address.endswith("@intel.com"):
                if msg['To']:
                    msg['To'] += ", " + email_address
                else:
                    msg['To'] = email_address
            else:
                self.__logger.printLog("WARNING", "non-Intel address, disregarding: {0}".format(email_address))

        # Create the body of the message (a plain-text and an HTML version).
        if MIMEType not in ('plain', 'html'):
            MIMEType = 'plain'
            text = "Hello,\n\n" + text + "\n\nRegards,\nBootOta Test Team"
        if MIMEType == 'html':
            if format_html:
                text = """\
                      <html>
                      <head></head>
                      <body>
                        <p>""" + "Hello,<br><br>" + text + "<br><br>Regards,<br>BootOta Test Team" + """
                        </p>
                      </body>
                      </html>
                      """

        # Record the MIME type - text/plain or text/html.
        part = MIMEText(text, MIMEType)

        # Attach parts into message container.
        # According to RFC 2046, the last part of a multipart message, in this case
        # the HTML message, is best and preferred.
        msg.attach(part)

        # Send the message via "smtp.intel.com" server.
        s = smtplib.SMTP("smtp.intel.com")

        # sendmail function takes 3 arguments: sender's address, recipient's address
        # and message to send - here it is sent as one string.
        To.append("tonyx.raveneau@intel.com")
        s.sendmail(From, To, msg.as_string())
        s.quit()

    @staticmethod
    def which(fileName):
        """
        Shows the full path of (shell) commands.
        Find full path of a command/file from PATH environment

        :rtype: string
        :return: Full path of the command
                Return None in case PATH environment is not defined or command not found
        """
        # Reformat command name regarding os
        fileName = fileName if os.name in ['posix'] else fileName + ".exe"

        # Separator is different in Linux
        path_separator = ":" if os.name in ['posix'] else ";"
        fullPathCmd = None

        # Get PATH environment value
        os_env_path = os.environ.get("PATH")
        if os_env_path:
            for path in os_env_path.split(path_separator):
                try:
                    if fileName in os.listdir(r'%s' % path):
                        fullPathCmd = os.path.join(path, fileName)
                        break
                except OSError:
                    # Skip if current path is not found
                    # It could arrives that some path defined in the PATH environment is not found.
                    continue

        return fullPathCmd

    @staticmethod
    def getCaller(index):
        # index=0 return getCaller
        # index=1 return getCaller's caller
        # index=2 return caller of getCaller's caller
        return inspect.getouterframes(inspect.currentframe(), 2)[index][3]

    def downloadFile(self, url, to):
        """ Download a file from url
        """
        output = ""
        verdict = True
        self.__logger.printLog("INFO", "download {0} to {1}".format(url, to))
        try:
            url_obj = urllib2.urlopen(url)
            with open(to, "wb") as f:
                f.write(url_obj.read())
            if not os.path.isfile(to):
                output = "downloaded file not present at expected path: {0}".format(to)
                verdict = False
        except Exception as e:
            output = "failed to download ({0})".format(e)
            verdict = False
        self.__output.appendOutput(output, verdict)
        return verdict

    def checkProcess(self, process):
        """ Return True if the process given in parameter == found running on the device.
            adb needed.
        """
        self.__logger.printLog("INFO", "checkProcess({0}): start".format(process))
        exec_status, grepOutput = self.__host.commandExecAdb("shell ps | grep -a {0}".format(process))
        if process in grepOutput:
            output = ""
            verdict = True
        else:
            output = "no process '{0}' found".format(process)
            verdict = False
        self.__output.appendOutput(output, verdict)
        return verdict

    def int2StringDelay(self, delay):
        # format in minute + seconds if delay superior to 1 minute
        try:
            if delay >= 60:
                if (delay % 60) != 0:
                    if int(delay % 60) > 9:
                        strDelay = "{0}min{1}s".format(int(delay) / 60, int(delay % 60))
                    else:
                        strDelay = "{0}min0{1}s".format(int(delay) / 60, int(delay % 60))
                else:
                    strDelay = "{0}min".format(int(delay / 60))
            else:
                strDelay = "{0}s".format(int(delay))
        except Exception as e:
            strDelay = ""
            self.__logger.printLog("WARNING", "int2StringDelay(): issue with formatting (error={0})".format(e))
        return strDelay

    def waitDelay(self, log, delay):
        strDelay = self.int2StringDelay(delay)
        # print message
        self.__logger.printLog("INFO", "waitDelay(delay={1}): {0}".format(log, strDelay))
        # sleep
        time.sleep(delay)
        # exit
        self.__output.appendOutput("", None, argument="delay={0}".format(strDelay))

    def downloadConfiguration2dictionary(self, file_type_list, ota=True, board_type="", device="", variant="",
                                         multi_variant_prefix=""):
        log = "downloadConfiguration2dictionary(): "
        # convert input into dictionary
        download_data = list()
        length = len(file_type_list)
        for index, flash_file in enumerate(file_type_list):
            if not ota and (flash_file.get("pft_key", "") == "ota" or flash_file.get("local_key", "") == "ota"):
                self.__logger.printLog("INFO", log +
                                       "skipping OTA flash file download (file {0}/{1})".format(index+1, length))
                continue
            local_dict = dict()
            valid_entries = ["pft_key", "buildbot_artifactory_path", "jenkins_artifactory_path", "regexp_key",
                             "flash_file", "configuration", "local_key", "groups", "pft_fallback", "start_state",
                             "forced_variant", "flash_list_processing", "tag_regexp", "tag_group",
                             "sit_artifactory_path"]
            for data in flash_file:
                if flash_file[data]:
                    local_dict[str(data)] = flash_file[data]
                else:
                    local_dict[str(data)] = ""
            for entry in valid_entries:
                # fill found entries not filled yet
                if entry not in local_dict:
                    if entry in ["groups"]:
                        local_dict[entry] = {}
                    else:
                        local_dict[entry] = ""
                if not isinstance(local_dict[entry], list) and not isinstance(local_dict[entry], dict) \
                        and not isinstance(local_dict[entry], int):
                    local_dict[entry] = str(local_dict[entry]
                                            .replace("board_id",
                                                     re.sub(multi_variant_prefix, "", board_type))
                                            .replace("device_name_id", device)
                                            .replace("variant_id", variant))
                elif isinstance(local_dict[entry], list):
                    for sub_index, element in enumerate(local_dict[entry]):
                        local_dict[entry][sub_index] = str(element
                                                           .replace("board_id",
                                                                    re.sub(multi_variant_prefix, "", board_type))
                                                           .replace("device_name_id", device)
                                                           .replace("variant_id", variant))
                elif isinstance(local_dict[entry], dict) and entry in ["pft_fallback", "sit_artifactory_path",
                                                                       "buildbot_artifactory_path",
                                                                       "jenkins_artifactory_path"]:
                    for element in local_dict[entry]:
                        if not isinstance(local_dict[entry][element], list) \
                                and not isinstance(local_dict[entry][element], dict):
                            local_dict[entry][element] = str(local_dict[entry][element]
                                                             .replace("board_id",
                                                                      re.sub(multi_variant_prefix, "", board_type))
                                                             .replace("device_name_id", device)
                                                             .replace("variant_id", variant))

            # override parameters
            key = local_dict["local_key"] if local_dict.get("local_key") else local_dict.get("pft_key", "")
            if key and isinstance(self.__globalConf.get("INTERNAL_DOWNLOAD_OVERRIDE"), dict) and \
                            key in self.__globalConf["INTERNAL_DOWNLOAD_OVERRIDE"]:
                override_params = self.__globalConf["INTERNAL_DOWNLOAD_OVERRIDE"][key]
                for item in override_params:
                    if item not in valid_entries:
                        self.__logger.printLog("WARNING", log + "invalid override parameter")
                        continue
                    if isinstance(override_params[item], dict):
                        if not local_dict.get(item):
                            local_dict[item] = dict()
                        for element in override_params[item]:
                            local_dict[item][element] = override_params[item][element]
                    else:
                        local_dict[item] = override_params[item]
                    self.__logger.printLog("INFO", log + "adding to '{}' flash files: "
                                                         "{}={}".format(key, item,override_params[item]))

            if local_dict.get("local_key") or local_dict.get("pft_key") or local_dict.get("flash_list_processing"):
                download_data.append(local_dict)
            else:
                self.__logger.printLog("WARNING", log + "no 'local_key' or 'pft_key' entry, skipping file "
                                                        "{0}/{1}".format(index+1, length))

        self.__logger.printLog("INFO", log + "download data =\n{0}".format(json.dumps(download_data,
                                                                                      sort_keys=True,
                                                                                      indent=4)))
        return download_data

    def xmlD2DDownloadConfiguration2dictionary(self, file_type_list):
        log = "xmlD2DDownloadConfiguration2dictionary(): "
        # convert input into dictionary
        download_data = list()
        for flash_file in file_type_list:
            local_dict = dict()
            valid_entries = ("build", "fallback_device", "fallback_board")
            for entry in valid_entries:
                local_dict[entry] = flash_file.get(entry, "")
            download_data.append(local_dict)

        self.otaBuild = local_dict["build"]

        self.__logger.printLog("INFO", log + "download data =\n{0}".format(json.dumps(download_data,
                                                                                      sort_keys=True,
                                                                                      indent=4)))
        return download_data

    class __FilesHandler():
        def __init__(self):
            self.__logger = LoggerModule.LoggerModule()
            self.__file_list = list()

        def addEntry(self, name):
            self.__logger.printLog("INFO", "FilesHandler.addEntry({0}): adding file".format(name))
            if name not in self.__file_list:
                self.__file_list.append(name)

        def cleanEntries(self):
            self.__logger.printLog("INFO", "FilesHandler.cleanEntries(): start")
            for element in self.__file_list:
                if os.path.isfile(element):
                    self.__logger.printLog("INFO", "cleanEntries(): erasing file {0}".format(element))
                    os.remove(element)
                elif os.path.isdir(element):
                    self.__logger.printLog("INFO", "cleanEntries(): erasing directory {0}".format(element))
                    shutil.rmtree(element)
                else:
                    self.__logger.printLog("DEBUG", "cleanEntries(): file not found {0}".format(element))

    def isExpectedJsonFormat(self, json_file, mandatory_keys=list(), top_keys=list()):
        log = "isExpectedJsonFormat(): "
        # check is file has json format
        if not json_file.endswith(".json"):
            self.__logger.printLog("INFO", log + "file has no json extension: {0}".format(json_file))
            return False
        # check if json has specific keys
        try:
            with open(json_file) as f:
                json_data = json.load(f)
        # catch issue when json is of bad format
        except Exception as e:
            self.__logger.printLog("WARNING", log + "failed to read json '{0}' ({1})".format(json_file, e))
            return False
        if not mandatory_keys:
            # return if not specific keys required
            return True

        if top_keys:
            for element in top_keys:
                available_keys = [e for e in json_data]
                if element not in json_data:
                    self.__logger.printLog("WARNING", log + "missing '{0}' key in json data, available keys are {1}".format(element, ", ".join(available_keys)))
                    return False
                else:
                    json_data = json_data[element]

        missing_keys = [key for key in mandatory_keys if key not in json_data]
        if not missing_keys:
            self.__logger.printLog("INFO", log + "file of proper json format {0} (checked key: {1})".
                                   format(json_file, ", ".join(mandatory_keys)))
            return True
        else:
            self.__logger.printLog("WARNING", log + "missing keys in json file {0} are {1} (among {2})".
                                   format(json_file, ", ".join(missing_keys), ", ".join(mandatory_keys)))
            return False

    def loadProvisioningProperties(self, input_dict):
        log = "loadProvisioningProperties(): "
        self.__logger.printLog("INFO", log + "start")
        pft_timeout = 1020
        if not isinstance(input_dict, dict):
            self.__logger.printLog("WARNING", log + "failure with input, not a dictionary")
            return {}
        output_dict = dict()
        output_dict["build_target"] = input_dict.get("build_target", "")
        output_dict["build_variant"] = input_dict.get("build_variant", "")
        output_dict["url_buildinfo"] = input_dict.get("url_buildinfo", "")
        output_dict["board_type"] = input_dict.get("board_type", "")
        output_dict["global_pft_timeout"] = input_dict.get("timeout", pft_timeout)
        output_dict["artifactory_to_local_artifactory_download_timeout"] = input_dict.get("download_timeout", pft_timeout+60)
        elements = [f for f in [input_dict["build_target"],
                                input_dict["build_variant"]] if f]
        output_dict["buildvariant"] = "-".join(elements)
        try:
            output_dict["artifactory_to_local_artifactory_download_timeout"] = int(output_dict["artifactory_to_local_artifactory_download_timeout"])
            output_dict["global_pft_timeout"] = int(output_dict["global_pft_timeout"])
        except Exception as e:
            self.__logger.printLog("WARNING", log + "failure to convert into integer timeouts: '{}', '{}' "
                                                    "(error={})".format(output_dict["artifactory_to_local_artifactory_download_timeout"],
                                                                        output_dict["global_pft_timeout"],
                                                                        e))
            output_dict["artifactory_to_local_artifactory_download_timeout"] = pft_timeout
            output_dict["global_pft_timeout"] = pft_timeout + 60

        self.__logger.printLog("INFO", log + "provisioning data loaded:\n{}".format(json.dumps(output_dict, sort_keys=True, indent=4)))
        return output_dict.copy()