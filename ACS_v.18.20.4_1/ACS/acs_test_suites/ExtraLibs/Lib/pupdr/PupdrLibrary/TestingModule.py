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
@summary: Pupdr Library - TestingModule
@since: 07/30/2015
@author: travenex
"""

import os
import re
import json
import datetime
import tempfile
import urllib
import time
import OutputModule
import HostModule
import LoggerModule
import DownloadModule
import ConfigurationModule
import MiscModule

class TestingModule(object):

    __instance = None
    __globalConf = None
    __logger = None
    __configuration = None
    __host = None
    __download = None
    __output = None
    __misc = None
    ipath = None

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
        self.__download = DownloadModule.DownloadModule()
        self.__misc = MiscModule.MiscModule()
        self.__configuration = ConfigurationModule.ConfigurationModule()
        try:
            import requests
            self.__logger.printLog("INFO", "TestingModule init: request module properly loaded")
        except Exception as e:
            self.__logger.printLog("WARNING", "TestingModule init: error while importing request python module, "
                                              "TCR requests disabled (error={})".format(e))
        self.ipath = False

    class __timeZoneOffset(datetime.tzinfo):
        """Fixed offset in minutes: `time = utc_time + utc_offset`."""
        def __init__(self, offset):
            datetime.tzinfo.__init__(offset)
            self.__offset = datetime.timedelta(minutes=offset)
            hours, minutes = divmod(offset, 60)
            #NOTE: the last part is to remind about deprecated POSIX GMT+h timezones
            #  that have the opposite sign in the name;
            #  the corresponding numeric value is not used e.g., no minutes
            self.__name = '<%+03d%02d>%+d' % (hours, minutes, -hours)
        def utcoffset(self, dt=None):
            return self.__offset
        def tzname(self, dt=None):
            return self.__name
        def dst(self, dt=None):
            return datetime.timedelta(0)
        def __repr__(self):
            return 'FixedOffset(%d)' % (self.utcoffset().total_seconds() / 60)

    def getBuildsToTest(self, board_type_list, skip_tags=list(), force_tags=list(), force_only=False):
        log = "getBuildsToTest(): "
        if board_type_list:
            self.__logger.printLog("INFO", "starting for {0} board".format(", ".join(board_type_list)))
        output_dict = {"error_list": [], "tests": []}

        # find branch list
        if "global" not in self.__configuration.loaded_config_list:
            self.__configuration.updateConfig("", "", print_config=False, config_name="global")
        else:
            self.__configuration.switchConfig("global")
        builders = {}
        for builder in self.__configuration.test_description.builder_list:
            builders[str(builder)] = {}
        if not builders:
            output = "failure to get branches to test"
            self.__output.appendOutput(output, False)
            return {"critical_error": output}
        self.__logger.printLog("INFO", log + "{0} builders found".format(", ".join([builder for builder in builders])))

        if not board_type_list:
            self.__logger.printLog("INFO", log + "getting defaults boards for all branches")
            for builder in builders:
                builders[builder]["board_type_list"] = []
                for board_type in self.__configuration.test_description.builder_list[builder].get("mandatory_boards", {}):
                    if board_type not in builders[builder]["board_type_list"]:
                        builders[builder]["board_type_list"].append(board_type)
                self.__logger.printLog("INFO", log + "looking for {0} boards on {1} "
                                                     "builder".format(", ".join(builders[builder]["board_type_list"]),
                                                                     builder))
        else:
            for builder in builders:
                builders[builder]["board_type_list"] = board_type_list

        # find campaigns to run
        campaigns = list()
        builders_requiring_test = {}
        for builder in builders:
            if len(builder.split("-")) != 2:
                self.__logger.printLog("WARNING", log + "'{0}' invalid builder name, <branch>-<build_type> format expected".format(builder))
                continue
            branch = builder.split("-")[0]
            build_type = builder.split("-")[1]
            if not builders[builder]["board_type_list"]:
                output_log = "no board types for {0} builder".format(builder)
                self.__logger.printLog("WARNING", output_log)
                output_dict["error_list"].append(output_log)
                continue
            for board_type in builders[builder]["board_type_list"]:
                self.__configuration.updateConfig(branch, board_type, print_config=False)
                if build_type == "engineering":
                    campaign_list = self.__configuration.test_description.engineering
                elif build_type == "release_candidate":
                    campaign_list = self.__configuration.test_description.release_candidate
                elif build_type == "latest":
                    campaign_list = self.__configuration.test_description.latest
                elif build_type == "preintegration":
                    campaign_list = self.__configuration.test_description.preintegration
                elif build_type == "weekly":
                    campaign_list = self.__configuration.test_description.weekly
                else:
                    self.__logger.printLog("WARNING", log + "'{0}' unknown build type".format(build_type))
                    continue
                if not campaign_list:
                    self.__logger.printLog("INFO", "no tests required on {0} builder and {1} board".format(builder, board_type))
                    continue
                for single_campaign in campaign_list:
                    if builder not in builders_requiring_test:
                        builders_requiring_test[builder] = {"regexp": [], "board_type_list": []}
                    builders_requiring_test[builder]["board_type_list"].append(board_type)
                    builders_requiring_test[builder]["regexp"].append(single_campaign.get("build_tag", ""))
                    for test in single_campaign.get("testing"):
                        campaigns.append({"builder": builder,
                                          "campaign": test.get("campaign", ""),
                                          "flash": test.get("flash", True),
                                          "regexp": single_campaign.get("build_tag", ""),
                                          "name": single_campaign.get("name", ""),
                                          "build_target": test.get("build_target", ""),
                                          "board_type": board_type,
                                          "warnings": test.get("warnings", [])})

        if not campaigns:
            output = "no tests found"
            self.__output.appendOutput(output, False)
            return {"critical_error": output}

        # display found campaigns
        for index in range(len(campaigns)):
            log_info = "{0:20} campaign on {1:25} and {2:35} " \
                       "targets".format(campaigns[index]["campaign"],
                                        campaigns[index]["builder"],
                                        campaigns[index]["board_type"]+"/"+campaigns[index]["build_target"])
            if campaigns[-1]["warnings"]:
                log_info += " (warning on TC: {0})".format(", ".join(campaigns[-1]["warnings"]))
            self.__logger.printLog("DEBUG", log_info)

        # set time threshold
        threshold = datetime.datetime.now()
        threshold = threshold.replace(hour=10, minute=0, second=0, microsecond=0, tzinfo=self.__timeZoneOffset(120))
        if datetime.datetime.now().hour < 12:
            threshold = threshold.replace(hour=14)
            threshold -= datetime.timedelta(1)

        all_build_results = list()

        if force_only:
            builders_requiring_test = {}
            self.__logger.printLog("INFO", log + "removing all builder because only using forced builds, "
                                                 "{0}".format(", ".join(force_tags)))
        else:
            self.__logger.printLog("DEBUG", log + "looking for build after {0}".format(threshold))

        # find builds to test
        for builder in builders_requiring_test:
            branch = builder.split("-")[0]
            build_type = builder.split("-")[1]

            # erase previously written output json file
            pft_output_file = os.path.join(tempfile.gettempdir(), "test_pft_output_{}.json".format(self.__misc.getUser()))
            if os.path.isfile(pft_output_file):
                os.remove(pft_output_file)

            # launch pft to get buildbot metadata json
            cmd = "--search --non-interactive " \
                  "--property buildbot.props.branch_name={0} " \
                  "--property buildbot.props.buildername=*-{1} " \
                  "--search-output {2}" \
                  .format(branch, build_type, pft_output_file)
            self.__host.commandExecCflasher(cmd, 400)
            if not os.path.isfile(pft_output_file):
                # retry PFT command if command fails
                self.__logger.printLog("DEBUG", "failure with PFT command, retrying request")
                self.__host.commandExecCflasher(cmd, 400)
                if not os.path.isfile(pft_output_file):
                    output_log = "failure to get build data for '{0}' builder with PFT request".format(builder)
                    self.__logger.printLog("WARNING", output_log)
                    output_dict["error_list"].append(output_log)
                    continue

            # try to read pft output json, exit when failed reading file
            try:
                with open(pft_output_file) as local_pft_file:
                    data = json.load(local_pft_file)
                os.remove(pft_output_file)
            except Exception as exception_log:
                os.remove(pft_output_file)
                output_log = "failure with json parsing of '{0}' ({1})".format(pft_output_file, exception_log)
                self.__logger.printLog("WARNING", output_log)
                output_dict["error_list"].append(output_log)
                continue

            if "results" not in data or not data.get("results"):
                output_log = "empty data in '{0}'".format(pft_output_file)
                self.__logger.printLog("WARNING", output_log)
                output_dict["error_list"].append(output_log)
                continue

            # parse results and find all builds
            build_results = list()
            for regexp in builders_requiring_test[builder]["regexp"]:
                self.__logger.printLog("INFO", log + "searching artifactory for {0} build with '{1}' matching boot"
                                                     "reason".format(builder, regexp))
                for index, element in enumerate(data["results"]):
                    if isinstance(element["properties"].get("buildbot.props.reason"), list):
                        reason = element["properties"]["buildbot.props.reason"][0]
                    else:
                        reason = element["properties"].get("buildbot.props.reason", "")
                    reason = str(reason)
                    if "properties" in element and \
                        not any(entry not in element["properties"] for entry in ["buildbot.props.short_build_number"])\
                        and "downloadUri" in element\
                        and element["downloadUri"].endswith("build_info.json")\
                        and "created" in element\
                        and re.search(regexp, reason):
                        found_build_numbers = [single_element["tag"] for single_element in build_results]
                        if element["properties"]["buildbot.props.short_build_number"][0] not in found_build_numbers:
                            # parse date
                            str_date = element["created"][:-6]
                            timezone = element["created"][-6:].replace(":", "")
                            offset = int(timezone[-4:-2])*60 + int(timezone[-2:])
                            if timezone.startswith("-"):
                                offset = -offset
                            date = datetime.datetime.strptime(str_date, "%Y-%m-%dT%H:%M:%S.%f")
                            date = date.replace(tzinfo=self.__timeZoneOffset(offset))
                            build_results.append({"builder": builder,
                                                  "reason": reason,
                                                  "tag": element["properties"]["buildbot.props.short_build_number"][0],
                                                  "url": self.__download.cleanUrl(element["downloadUri"]),
                                                  "date": date})

            # remove older builds and already tested builds
            for element in build_results:
                if element["date"] < threshold:
                    element["skipped"] = "older builds"
                if skip_tags and element["tag"] in skip_tags:
                    element["skipped"] = "already tested"

            # check entry in build_info.json
            for element in [build for build in build_results if not build.get("skipped", "")]:
                # store build_info.json in tmp dir
                temp_json = os.path.join(tempfile.gettempdir(), "build_info_{}.json".format(self.__misc.getUser()))
                if os.path.isfile(temp_json):
                    os.remove(temp_json)
                self.__misc.local_files_handler.addEntry(temp_json)
                self.__host.commandExecArtifactorySecureDownload(element["url"], os.path.dirname(temp_json),
                                                                 file_name=os.path.basename(temp_json))
                if not os.path.isfile(temp_json):
                    output_log = "failure to download {0}".format(element["url"])
                    self.__logger.printLog("WARNING", output_log)
                    output_dict["error_list"].append(output_log)
                    continue
                try:
                    with open(temp_json) as f:
                        json_data = json.load(f)
                    os.remove(temp_json)
                except Exception as e:
                    os.remove(temp_json)
                    output_log = "failure to parse {0} downloaded from {1} (error={2})".format(temp_json, element["url"], e)
                    self.__logger.printLog("WARNING", output_log)
                    output_dict["error_list"].append(output_log)
                    continue
                else:
                    missing_target_list = list()
                    check_list = []
                    for campaign in campaigns:
                        # extract data
                        board_type = campaign["board_type"]
                        buildvariant = campaign["build_target"]
                        full_target = board_type + "/" + buildvariant
                        branch = campaign["builder"].split("-")[0]
                        # skip if check already performed
                        if full_target in check_list:
                            # already tested
                            continue
                        else:
                            # add
                            check_list.append(full_target)
                        self.__configuration.switchConfig(branch+"-"+board_type)
                        if not self.__checkEntryInBuildInfo(json_data, board_type, buildvariant):
                            if full_target not in missing_target_list:
                                missing_target_list.append(full_target)
                    # if no target found, discarding build
                    if len(missing_target_list) == len(check_list):
                        element["skipped"] = "no entry found in build_info.json for {0} targets".format(", ".join(missing_target_list))

            # print skipped builds
            skipped = {}
            for element in build_results:
                skip_reason = element.get("skipped", "")
                if skip_reason:
                    if skip_reason not in skipped:
                        skipped[skip_reason] = []
                    skipped[skip_reason].append(element["tag"])
            for skip_reason in skipped:
                self.__logger.printLog("INFO", log + "{0} - skipping {1}".format(skip_reason, ", ".join(skipped[skip_reason])))

            # adding builds to list of all builds from all builders
            for build in build_results:
                all_build_results.append(build)

            # print warning if no new build found
            if not [build for build in build_results if not build.get("skipped", "")]:
                output_log = "no new build found on {0} builder".format(builder)
                self.__logger.printLog("WARNING", output_log)
                output_dict["error_list"].append(output_log)

        # adding forced tag:
        local_index = 0
        for element in force_tags:
            tag = self.__download.getBuildTag(element)
            if not tag:
                output_log = "failure to get build tag from {0}".format(element)
                self.__logger.printLog("WARNING", output_log)
                output_dict["error_list"].append(output_log)
                continue
            if any((tag == build["tag"] and not build.get("skipped", "")) for build in all_build_results):
                self.__logger.printLog("WARNING", "{0} build already added from artifactory builds".format(tag))
                continue
            url, output = self.__download.buildTag2ArtifactoryUrl(tag)
            if not url:
                output_log = "failure to get build url from {0} ({1})".format(tag, output)
                self.__logger.printLog("WARNING", output_log)
                output_dict["error_list"].append(output_log)
                continue
            branch = self.__download.getBranchFromTag(tag)
            build_type = self.__download.getBuilderFromTag(tag)
            if not branch or not build_type:
                output_log = "failure to get builder from {0}".format(tag)
                self.__logger.printLog("WARNING", output_log)
                output_dict["error_list"].append(output_log)
                continue
            builder = "-".join([branch, build_type])
            temp_json = os.path.join(tempfile.gettempdir(), "build_info_{}.json".format(self.__misc.getUser()))
            if os.path.isfile(temp_json):
                os.remove(temp_json)
            self.__misc.local_files_handler.addEntry(temp_json)
            self.__host.commandExecArtifactorySecureDownload(url, os.path.dirname(temp_json), file_name=os.path.basename(temp_json))
            if not os.path.isfile(temp_json):
                output_log = "failure to download {0}".format(url)
                self.__logger.printLog("WARNING", output_log)
                output_dict["error_list"].append(output_log)
                continue
            try:
                with open(temp_json) as f:
                    json_data = json.load(f)
                os.remove(temp_json)
            except Exception as e:
                os.remove(temp_json)
                output_log = "failure to parse {0} downloaded from {1} (error={2})".format(temp_json, url, e)
                self.__logger.printLog("WARNING", output_log)
                output_dict["error_list"].append(output_log)
                continue
            # force build will be tested first
            if isinstance(json_data["buildbot_properties"].get("buildbot.props.reason"), list):
                reason = json_data["buildbot_properties"]["buildbot.props.reason"][0]
            else:
                reason = json_data["buildbot_properties"].get("buildbot.props.reason", "")
            reason = str(reason)
            all_build_results.insert(
                local_index,
                {
                    "builder": builder,
                    "reason": reason,
                    "forced": True,
                    "tag": tag,
                    "url": url,
                    "date": None
                }
            )
            local_index += 1

        # generate output test dictionary with build that haven't been skipped
        for build in [build for build in all_build_results if not build.get("skipped", "")]:
            found_campaign = False
            match_found = True
            if build.get("forced"):
                check_build_reason = any(re.search(campaign["regexp"], build["reason"]) for campaign in campaigns if campaign["builder"] == build["builder"])
                if not any(campaign["builder"] == build["builder"] for campaign in campaigns):
                    match_found = False
            else:
                check_build_reason = True
            for campaign in campaigns:
                build_reason_matches = re.search(campaign["regexp"], build["reason"])
                if campaign["builder"] == build["builder"] and\
                    (check_build_reason and build_reason_matches or
                         (not check_build_reason and "default" in campaign["name"])) or\
                         (not match_found and campaign["name"] == "default" and campaign["builder"].split("-")[0] == build["builder"].split("-")[0]):
                    found_campaign = True
                    output_dict["tests"].append({"url": build["url"],
                                                 "campaign": campaign["campaign"],
                                                 "build_target": campaign["build_target"],
                                                 "flash": campaign["flash"],
                                                 "board_type": campaign["board_type"],
                                                 "tag": build["tag"],
                                                 "warnings": campaign["warnings"],
                                                 "skip": False})
                    self.__logger.printLog("INFO", "found {0:20} on {1:15} build and {2:35} "
                                                   "target".format(output_dict["tests"][-1]["campaign"],
                                                                   output_dict["tests"][-1]["tag"],
                                                                   output_dict["tests"][-1]["board_type"]+"/"+output_dict["tests"][-1]["build_target"]))
            if not found_campaign:
                self.__logger.printLog("WARNING", log + "no campaign found for "
                                                        "'{}' on '{}' branch".format(build["tag"], build["builder"]))


        # add skipped tests in output as well so further test do not check them again
        for build in [build for build in all_build_results if build.get("skipped", "")]:
            output_dict["tests"].append({"tag": build["tag"],
                                         "skip": True})

        if not output_dict["tests"]:
            output_log = "no tests founds on all branches"
            self.__logger.printLog("WARNING", output_log)
            output_dict["error_list"].append(output_log)

        self.__output.appendOutput("", True)
        return output_dict

    def __checkEntryInBuildInfo(self, build_info_data, board, buildvariant):
        log = "checkEntryInBuildInfo(): "
        verdict = True
        output = ""
        self.__logger.printLog("INFO", log + "checking {0}/{1} entry in {2} "
                                       "buildinfo".format(board, buildvariant,
                                                          build_info_data.get("buildbot_properties", {}).get("buildbot.props.short_build_number", "")))
        if "hardwares" not in build_info_data:
            output = "'hardwares' entry missing in build_info.json"
            verdict = False
        elif board not in build_info_data["hardwares"]:
            output = "'{0}' entry missing in 'hardwares' entry".format(board)
            verdict = False
        elif buildvariant not in build_info_data["hardwares"][board]["variants"]:
            output = "'{0}' entry missing in 'hardwares/{1}/variants' entry".format(buildvariant, board)
            verdict = False
        else:
            self.__logger.printLog("INFO", log + "check passed for {0}/{1}".format(board, buildvariant))
        if not any(element.get("pft_key") for element in self.__configuration.download.INTERNAL_DOWNLOAD):
            verdict = True
            self.__logger.printLog("DEBUG", log + "bypass check because no 'flash_file' entry present in INTERNAL_DOWNLOAD parameter")

        self.__output.appendOutput(output, verdict)
        return verdict

    def tcrBuildStatusRequest(self, build_tag):
        log = "tcrBuildStatusRequest({0}): ".format(build_tag)
        self.__logger.printLog("INFO", log + "starting")

        base_url = 'http://api.tcr.sh.intel.com/1'

        # Make the GET request
        if self.ipath:
            tc = "BOOTOTA - SETUP - SYSTEM FLASH"
        else:
            tc = "PUPDR_SYSTEM_FLASH"

        nowIso = datetime.datetime.utcnow() - datetime.timedelta(days=30)
        nowIsoEpochMilliSeconds = int(round(time.mktime(nowIso.timetuple()) * 1000))

        query = {"query": {"creation": {">": nowIsoEpochMilliSeconds},
                           "result.bootOta.build": {"==": str(build_tag)},
                           "testCase": {"==": tc}}}

        try:
            import requests
        except Exception as e:
            self.__logger.printLog("WARNING", log + "cannot import request python lib ({})".format(e))
            self.__output.appendOutput("", False)
            return {}
        else:
            ret = requests.get(base_url + "/tests?" + urllib.urlencode(query),
                               headers={"content-type":"application/json"})

            try:
                ret.raise_for_status() # Equivalent to threw an exception if r.status_code != 200
            except Exception as e:
                self.__logger.printLog("WARNING", "failure to get results from TCR (error={0})".format(e))
                results = {}
                verdict = False
            else:
                results = ret.json()
                self.__logger.printLog("DEBUG", log + "TCR request returned {0} results".format(results.get("totalCount", 0)))
                verdict = True

            self.__output.appendOutput("", verdict)
            return results

    def tcrCampaignStatusRequest(self, campaign_id):
        log = "tcrCampaignStatusRequest({0}): ".format(campaign_id)
        self.__logger.printLog("INFO", log + "starting")

        base_url = 'http://api.tcr.sh.intel.com/1'

        try:
            import requests
        except Exception as e:
            self.__logger.printLog("WARNING", log + "cannot import request python lib ({})".format(e))
            self.__output.appendOutput("", False)
            return {}
        else:
            ret = requests.get(base_url + "/campaigns/" + str(campaign_id),
                               headers={"content-type":"application/json"})

            try:
                ret.raise_for_status() # Equivalent to threw an exception if r.status_code != 200
            except Exception as e:
                self.__logger.printLog("WARNING", "failure to get results from TCR (error={0})".format(e))
                results = {}
                verdict = False
            else:
                results = ret.json()
                verdict = True

            output_dict = {}
            if not any(element not in results for element in ["id", "name", "status", "verdict"]):
                output_dict["id"] = results["id"]
                output_dict["name"] = results["name"]
                output_dict["status"] = results["status"]
                output_dict["verdict"] = results["verdict"]
                try:
                    output_dict["rate"] = int(float(results.get("_meta", {}).get("rates", {}).get("status", {}).get("COMPLETED", 0))/
                                              float(results.get("_meta", {}).get("nbTests", 0)) * 100.0)
                except:
                    output_dict["rate"] = 0

            if output_dict:
                self.__logger.printLog("INFO", log + "found campaigns with '{0}' status and '{1}' verdict"
                                       .format(output_dict["status"],
                                               output_dict["verdict"]))
            else:
                self.__logger.printLog("INFO", log + "no found campaigns")

            self.__output.appendOutput("", verdict)
            return output_dict

    def tcrTcListFromCampaignIdRequest(self, campaign_id):
        log = "tcrTcListFromCampaignIdRequest({0}): ".format(campaign_id)
        self.__logger.printLog("INFO", log + "starting")

        base_url = 'http://api.tcr.sh.intel.com/1'

        try:
            import requests
        except Exception as e:
            self.__logger.printLog("WARNING", log + "cannot import request python lib ({})".format(e))
            self.__output.appendOutput("", False)
            return {}
        else:
            ret = requests.get(base_url + "/campaigns/" + str(campaign_id) + "/tests",
                               headers={"content-type":"application/json"})

            try:
                ret.raise_for_status() # Equivalent to threw an exception if r.status_code != 200
            except Exception as e:
                self.__logger.printLog("WARNING", "failure to get results from TCR (error={0})".format(e))
                results = {}
                verdict = False
            else:
                results = ret.json()
                self.__logger.printLog("DEBUG", log + "TCR request returned {0} results".format(len(results)))
                verdict = True

            self.__output.appendOutput("", verdict)
            return results

    def tcrTcRetestRequest(self, tc_name, board_type, target, variant, build_tag):
        log = "tcrTcRetestRequest({0}, {1}, {2}, {3}, {4}): ".format(tc_name, board_type, target, variant, build_tag)
        self.__logger.printLog("INFO", log + "starting")

        base_url = 'http://api.tcr.sh.intel.com/1'

        nowIso = datetime.datetime.utcnow() - datetime.timedelta(days=30)
        nowIsoEpochMilliSeconds = int(round(time.mktime(nowIso.timetuple()) * 1000))

        # Make the GET request
        query = {
            "query": {
                "result.bootOta.build": {"==": str(build_tag)},
                "result.bootOta.boardType": {"==": str(board_type)},
                "result.bootOta.buildTarget": {"==": str(target)},
                "result.bootOta.buildVariant": {"==": str(variant)},
                "creation": {">": nowIsoEpochMilliSeconds},
                "testCase": {"==": str(tc_name)}
            }
        }

        try:
            import requests
        except Exception as e:
            self.__logger.printLog("WARNING", log + "cannot import request python lib ({})".format(e))
            self.__output.appendOutput("", False)
            return []
        else:
            ret = requests.get(base_url + "/tests?" + urllib.urlencode(query),
                               headers={"content-type":"application/json"})

            try:
                ret.raise_for_status() # Equivalent to threw an exception if r.status_code != 200
            except Exception as e:
                self.__logger.printLog("WARNING", "failure to get results from TCR (error={0})".format(e))
                results = {}
                verdict = False
            else:
                results = ret.json()
                self.__logger.printLog("DEBUG", log + "TCR request returned {0} results".format(results.get("totalCount", 0)))
                verdict = True

            self.__output.appendOutput("", verdict)
            return results.get("results")

    def getAllTcrCampaigns(self, campaign_dict, force_tag_list=list(), ipath=False):
        log = "getAllTcrCampaigns(): "
        self.__logger.printLog("INFO", log + "starting")
        self.ipath = ipath

        if not campaign_dict:
            self.__logger.printLog("INFO", log + "no tests found in campaign dictionary")
            return
        elif not isinstance(campaign_dict, list):
            self.__logger.printLog("INFO", log + "list input expected")
            return

        # tcr request for all builds
        tcr_results = []
        checked_build = []
        tags_to_check = [single_campaign["tag"] for single_campaign in campaign_dict]
        for tag in tags_to_check:
            if tag not in checked_build:
                checked_build.append(tag)
                tag_list = list(force_tag_list)
                tag_list.append(tag)
                for single_tag in tag_list:
                    test_results = self.tcrBuildStatusRequest(single_tag)
                    for element in test_results.get("results"):
                        campaign_results = self.tcrCampaignStatusRequest(element.get("campaignId"))
                        if campaign_results:
                            branch = self.__download.getBranchFromTag(single_tag)
                            board_type = element.get("result", {}).get("bootOta", {}).get("boardType", "")
                            self.__configuration.updateConfig(branch, board_type, print_config=False)
                            local_campaign_name = campaign_results.get("name", "").replace("_Campaign.xml", "")
                            tcr_results.append({
                                "campaign": local_campaign_name,
                                "status": campaign_results.get("status", ""),
                                "tag": tag,
                                "original_tag": single_tag,
                                "campaignId": element["campaignId"],
                                "verdict": campaign_results.get("verdict", ""),
                                "board_type": board_type,
                                "build_target": element.get("result", {}).get("bootOta", {}).get("buildTarget", "") +
                                    "-" + element.get("result", {}).get("bootOta", {}).get("buildVariant", ""),
                                "execution_rate": campaign_results.get("rate", 0)
                            })
                        else:
                            self.__logger.printLog("DEBUG", "no BOOTOTA campaign found with campaignId={0}".format(element.get("campaignId")))

        for element in tcr_results:
            self.__logger.printLog("INFO", "found {0:20} on {1:12} build and {2:30} "
                                           "target ({3:10}/ {4})"
                                   .format(element["campaign"],
                                           element["original_tag"],
                                           element["board_type"]+" / "+element["build_target"],
                                           element["status"],
                                           element["verdict"]))

        # checking campaign in tcr for each required campaign
        for campaign in campaign_dict:
            for result in tcr_results:
                branch = self.__download.getBranchFromTag(result["tag"])
                board_type = result["board_type"]
                self.__configuration.updateConfig(branch, board_type, print_config=False, config_name=branch+"-"+board_type)
                if self.__configuration.download.ROOT_BOARD_TYPE:
                    result_board_type = self.__configuration.download.ROOT_BOARD_TYPE
                else:
                    result_board_type = result["board_type"]
                if campaign["campaign"] == result["campaign"] and \
                    campaign["build_target"] == result["build_target"] and \
                    campaign["board_type"] == result_board_type and \
                    campaign["tag"] == result["tag"]:
                    if "tcr_reports" not in campaign:
                        campaign["tcr_reports"] = []
                    tcr_url = "http://tcr.sh.intel.com/#/campaigns/" + result["campaignId"] + "/detail"
                    if any(tcr_url == single_report["url"] for single_report in campaign["tcr_reports"]):
                        # skip if campaign already stored
                        continue
                    campaign["tcr_reports"].append(
                        {
                            "url": tcr_url
                        }
                    )
                    campaign["tcr_reports"][-1]["status"] = result["status"]
                    campaign["tcr_reports"][-1]["verdict"] = result["verdict"]
                    campaign["tcr_reports"][-1]["execution_rate"] = result["execution_rate"]
                    TC_list = self.tcrTcListFromCampaignIdRequest(result["campaignId"])
                    # remove ipath duplicates
                    TC_list = [tc for tc in TC_list if "- 0" not in tc["testCase"]]
                    failed_TC_list = []
                    for element in TC_list:
                        if element["verdict"] != "PASSED" and element["status"] == "COMPLETED":
                            retest = self.tcrTcRetestRequest(element["testCase"].rstrip(" "),
                                                             result["board_type"],
                                                             result["build_target"].split("-")[0],
                                                             result["build_target"].split("-")[-1],
                                                             result["original_tag"])
                            updated_retest_list = []
                            for single_retest in retest:
                                if single_retest["testCase"].rstrip(" ") == element["testCase"].rstrip(" ") and \
                                    single_retest["result"]["bootOta"]["boardType"] == result["board_type"] and \
                                    single_retest["result"]["bootOta"]["buildTarget"] == result["build_target"].split("-")[0] and \
                                    single_retest["result"]["bootOta"]["buildVariant"] == result["build_target"].split("-")[-1] and \
                                    single_retest["result"]["bootOta"]["build"] == result["original_tag"]:
                                    updated_retest_list.append(single_retest)
                            try:
                                success_rate = float(len([single_retest for single_retest in updated_retest_list if single_retest["verdict"] == "PASSED"])) /\
                                    float(len(updated_retest_list))
                            except:
                                success_rate = 0
                            failed_TC_list.append({
                                 "testCase": element["testCase"].rstrip(" "),
                                 "success_rate": int(success_rate * 100),
                                 "retry_number": len(updated_retest_list),
                                 "tcList": list()
                            })
                            for single_retest in updated_retest_list:
                                failed_TC_list[-1]["tcList"].append(
                                    {
                                        "verdict": single_retest["verdict"],
                                        "campaign_url": "http://tcr.sh.intel.com/#/campaigns/" + single_retest["campaignId"] + "/detail"
                                    }
                                )

                    campaign["tcr_reports"][-1]["executed"] = len(TC_list)
                    if campaign["tcr_reports"][-1]["status"] == "COMPLETED":
                        campaign["tcr_reports"][-1]["failed_tc_list"] = failed_TC_list
                    else:
                        campaign["tcr_reports"][-1]["failed_tc_list"] = []

    def __gerritPatchSetQuery(self, patchId):
        log = "gerritPatchSetQuery({0}): ".format(patchId)
        self.__logger.printLog("INFO", log + "starting")

        # gerrit query to get
        ssh_cmd = "ssh android.intel.com gerrit query --current-patch-set {0} --format=JSON".format(patchId)
        exec_status, output = self.__host.commandExec(ssh_cmd, timeout=60)
        json_data = {}
        if exec_status == 0:
            for line in output.split("\n"):
                try:
                    json_data = json.loads(line)
                except:
                    pass
                if json_data:
                    break
            if json_data:
                self.__logger.printLog("DEBUG", log + "json data properly extracted from ssh output")
            else:
                self.__logger.printLog("WARNING", log + "no json data could be extracted from ssh output")
        else:
            self.__logger.printLog("WARNING", log + "error with ssh command")

        return json_data

    def __reviewPatch(self, commitId, cr1, message=""):
        log = "reviewPatch({0}): ".format(commitId)
        self.__logger.printLog("INFO", log + "starting setting V{0}1 review".format("+" if cr1 else "-"))

        if message:
            message = "-m '{0}'".format(message)

        ssh_cmd = "ssh -p 29418 android.intel.com gerrit review --verified {0}1 {2} {1}".format("+" if cr1 else "-",
                                                                                                  commitId,
                                                                                                  message)

        self.__host.commandExec(ssh_cmd, timeout=60)

    def generateResults(self, campaign_dict, review=False, force_review=False, emails=""):
        log = "generateResults(): "
        self.__logger.printLog("INFO", log + "starting")

        content_per_build = {}
        warning_list = []

        # sort campaign per board type
        def getKey(item):
            return item.get("board_type", "").lower()

        for campaign in sorted(campaign_dict, key=getKey):
            builder = self.__download.getBranchFromTag(campaign["tag"]) + "-" + self.__download.getBuilderFromTag(campaign["tag"])
            if self.__configuration.test_description.builder_list[builder].get("mandatory_boards", {}).get(campaign["board_type"]) not in [True, False]:
                branch = self.__download.getBranchFromTag(campaign["tag"])
                board_type = campaign["board_type"]
                self.__configuration.updateConfig(branch, board_type, print_config=False)

                self.__logger.printLog("DEBUG", log + "skipping '{0}' board because associated with "
                                                      "'{1}'".format(campaign["board_type"],
                                                      self.__configuration.download.ROOT_BOARD_TYPE))
                continue

            if campaign["tag"] not in content_per_build:
                # get patch list:
                local_json = os.path.join(tempfile.gettempdir(), "build_info_{0}.json".format(campaign["tag"]))
                if not os.path.isfile(local_json):
                    self.__misc.local_files_handler.addEntry(local_json)
                    self.__host.commandExecArtifactorySecureDownload(campaign["url"], tempfile.gettempdir(), file_name=os.path.basename(local_json))
                patch_list = list()
                if os.path.isfile(local_json):
                    try:
                        with open(local_json) as f:
                            json_data = json.load(f)
                        patch_list = json_data.get("buildbot_properties", {}).get("buildbot.props.force_build_changeids", [])
                    except Exception as e:
                        self.__logger.printLog("WARNING", "failure to parse {0} (error={1})".format(local_json, e))
                        patch_list = []

                content_per_build[campaign["tag"]] = {}
                content_per_build[campaign["tag"]]["raw_patch_list"] = patch_list
                content_per_build[campaign["tag"]]["patch_list"] = []
                content_per_build[campaign["tag"]]["test_verdict"] = True
                content_per_build[campaign["tag"]]["buildbot_url"] = campaign["url"].split("build_info")[0] + "buildbot_link.htm"
                content_per_build[campaign["tag"]]["artifactory_url"] = campaign["url"].split("build_info")[0]
                content_per_build[campaign["tag"]]["html_data"] = \
                    ["<div style='margin-left: 30px'>",
                     "<a href='{0}' target='_blank'>Buildbot Link for {1}</a>".format(content_per_build[campaign["tag"]]["buildbot_url"], campaign["tag"]) + " - " +\
                     "<a href='{0}' target='_blank'>Artifactory Link for {1}</a><br>".format(content_per_build[campaign["tag"]]["artifactory_url"], campaign["tag"])]
                # get patch data
                jira_list = []
                for patch in patch_list:
                    # get author and commit
                    patch_json_data = self.__gerritPatchSetQuery(patch)
                    # store data in dict
                    content_per_build[campaign["tag"]]["patch_list"].append({
                        "id": patch,
                        "author": patch_json_data.get("currentPatchSet",{}).get("author", {}).get("name", ""),
                        "subject": patch_json_data.get("subject", ""),
                        "commitId": patch_json_data.get("currentPatchSet", {}).get("revision", ""),
                        "jira": ";".join(sorted([element.get("id") for element in patch_json_data.get("trackingIds", []) if "id" in element]))
                    })
                    if not content_per_build[campaign["tag"]]["patch_list"][-1]["jira"]:
                        content_per_build[campaign["tag"]]["patch_list"][-1]["jira"] = "UNKNOWN-JIRA"
                    # if new jira name found, store in list of jira
                    if content_per_build[campaign["tag"]]["patch_list"][-1]["jira"] not in jira_list:
                        jira_list.append(content_per_build[campaign["tag"]]["patch_list"][-1]["jira"])
                content_per_build[campaign["tag"]]["jira_list"] = jira_list
                # print patch list per jira
                for jira in jira_list:
                    # print JIRA
                    for single_jira in jira.split(";"):
                        if single_jira and single_jira != "UNKNOWN-JIRA":
                            content_per_build[campaign["tag"]]["html_data"].append("<a href='https://jira01.devtools.intel.com/browse/{0}' target='_blank'>{0}</a>".format(single_jira))
                        elif single_jira == "UNKNOWN-JIRA":
                            content_per_build[campaign["tag"]]["html_data"].append(single_jira)
                    content_per_build[campaign["tag"]]["html_data"].append("<div style='margin-left: 30px'>")
                    # print patch associated to each jira
                    for patch in content_per_build[campaign["tag"]]["patch_list"]:
                        if patch["jira"] == jira:
                            author = patch["author"]
                            subject = patch["subject"]
                            if author:
                                author = " - " + author
                            if subject:
                                subject = " ({0})".format(subject)
                            content_per_build[campaign["tag"]]["html_data"].append("<a href='{0}' target='_blank'>Patch {1}{2}{3}</a><br>"
                            .format("https://android.intel.com/#/c/"+str(patch["id"]),
                                    patch["id"],
                                    author,
                                    subject))
                    content_per_build[campaign["tag"]]["html_data"].append("</div>")
                content_per_build[campaign["tag"]]["html_data"].append("</div>")
                content_per_build[campaign["tag"]]["html_data"].append("<br>")
                content_per_build[campaign["tag"]]["html_data"].append("<div style='margin-left: 30px'>")
                content_per_build[campaign["tag"]]["html_data"].append("<table>")
                content_per_build[campaign["tag"]]["html_data"].append("<tr bgcolor=#00BFFF><td><b>Board</b></td><td><b>Variant</b></td><td><b>Campaign</b></td><td><b>Result</b></td><td><b>Report</b></td></tr>")
                content_per_build[campaign["tag"]]["text_data"] = ["<div style='margin-left: 30px'>"]
            if not campaign.get("tcr_reports"):
                warning = False
                if builder not in self.__configuration.test_description.builder_list:
                    self.__logger.printLog("WARNING", log + "invalid builder name '{0}'".format(builder))
                else:
                    if not isinstance(self.__configuration.test_description.builder_list[builder].get("mandatory_boards", {}).get(campaign["board_type"]), str):
                        warning = not self.__configuration.test_description.builder_list[builder].get("mandatory_boards", {}).get(campaign["board_type"], True)
                if not warning:
                    content_per_build[campaign["tag"]]["test_verdict"] = False
                    content_per_build[campaign["tag"]]["text_data"].append("<p style='color:red'>ERROR: No campaign results found on TCR for "
                                                                           "{}/{}/{}</p>"
                                                                           .format(campaign["board_type"],
                                                                                   campaign["build_target"],
                                                                                   campaign["campaign"]))
                    content_per_build[campaign["tag"]]["html_data"].append("<tr><td>{0}</td><td>{1}</td><td>{2}</td><td bgcolor=#990066>Not found in TCR</td><td>N/A</td></tr>".format(campaign["board_type"], campaign["build_target"], campaign["campaign"]))
                else:
                    data = "<p>INFO: '{0}' status not mandatory</p>".format(campaign["board_type"])
                    if data not in content_per_build[campaign["tag"]]["text_data"]:
                        content_per_build[campaign["tag"]]["text_data"].append(data)
                    content_per_build[campaign["tag"]]["html_data"].append("<tr><td>{0}</td><td>{1}</td><td>{2}</td><td bgcolor=#FFFF33><b>WARNING</b> - Not found in TCR</td><td>N/A</td></tr>".format(campaign["board_type"], campaign["build_target"], campaign["campaign"]))
            else:
                for element in campaign["tcr_reports"]:
                    if element["status"] == "COMPLETED":
                        # successful campaign
                        if element["verdict"] == "PASSED" and not any(any(single_tc["verdict"] != "VALID" for single_tc in tc["tcList"]) for tc in element["failed_tc_list"]):
                            result = "<b>SUCCESS</b>"
                            color = " bgcolor=#66CC00"
                        elif element["execution_rate"] != 100 and \
                                [tc["testCase"] for tc in element["failed_tc_list"]] == ["BOOTOTA - SETUP - SYSTEM FLASH"]:
                            # flash failure and only executed TC
                            result = "<b>FLASHING FAILURE</b>"
                            color = " bgcolor=#CC0000"
                        # fully executed but with failures
                        elif element["failed_tc_list"]:
                            warning_tc_list = []
                            failed_tc_list = []

                            # get warning configuration for specific board/campaign
                            branch = self.__download.getBranchFromTag(campaign["tag"])
                            board_type = campaign["board_type"]
                            self.__configuration.updateConfig(branch, board_type, print_config=False)
                            warning_conf = self.__configuration.test_description.warnings
                            if warning_conf:
                                for single_warning in warning_conf.copy():
                                    if "|" in single_warning:
                                        for sub_element in single_warning.split("|"):
                                            warning_conf[sub_element] = warning_conf[single_warning]
                                        warning_conf.pop(single_warning)
                                self.__logger.printLog("INFO", log + "found warning for {}/{}: {}".format(
                                    campaign["board_type"],
                                    campaign["campaign"],
                                    warning_conf))
                            else:
                                warning_conf = {}
                            # split between failed and warning TC
                            for tc in element["failed_tc_list"]:
                                if not any(single_tc["verdict"] != "VALID" for single_tc in tc["tcList"]):
                                    # do not show invalid tcs which are not failures
                                    continue
                                tcr_links = []
                                for retry in tc.get("tcList"):
                                    if retry["campaign_url"] not in tcr_links and retry["campaign_url"] != element["url"]:
                                        tcr_links.append(retry["campaign_url"])
                                if tc["testCase"] in warning_conf:
                                    warning_tc_list.append({
                                        "tc": tc["testCase"],
                                        "reason": re.sub("([A-Z]+-[0-9]+)(:.*)", "<a href='https://jira01.devtools.intel.com/browse/{0}' target='_blank'>{0}</a>".format(r'\1') + r'\2', warning_conf[tc["testCase"]])
                                    })
                                elif tc["success_rate"] > 0:
                                    warning_log = "{0}% SUCCESS on {1} iterations".format(tc["success_rate"], tc["retry_number"])
                                    for link in tcr_links:
                                        warning_log += " - <a href='{0}' target='_blank'>TCR Link</a>".format(link)
                                    warning_tc_list.append({
                                        "tc": tc["testCase"],
                                        "reason": warning_log
                                    })
                                    content_per_build[campaign["tag"]]["text_data"].append("<p style='color:orange'>WARNING: Multiple iterations found for "
                                                                               "{}/{}/{} (success rate = {}%)</p>"
                                                                               .format(campaign["board_type"],
                                                                                       campaign["build_target"],
                                                                                       tc["testCase"],
                                                                                       tc["success_rate"]))
                                else:
                                    if tc["retry_number"] > 1:
                                        failure_log = "{0}% SUCCESS on {1} iterations".format(tc["success_rate"], tc["retry_number"])
                                        for link in tcr_links:
                                            failure_log += " - <a href='{0}' target='_blank'>TCR Link</a>".format(link)
                                    else:
                                        failure_log = ""
                                    failed_tc_list.append({
                                        "tc": tc["testCase"],
                                        "reason": failure_log
                                    })
                            if failed_tc_list:
                                content_per_build[campaign["tag"]]["test_verdict"] = False
                                result = "<b>FAILURES:</b><br>{0}".format("<br>".join(["{0}{1}".format(single_failure["tc"], (" -> "+single_failure["reason"]) if single_failure["reason"] else "") for single_failure in failed_tc_list]))
                                color = " bgcolor=#CC3333"
                                if warning_tc_list:
                                    result += "<br><b>WARNINGS:</b><br>{0}".format("<br>".join(["{0} -> {1}".format(single_warning["tc"], single_warning["reason"]) for single_warning in warning_tc_list]))
                            elif warning_tc_list:
                                result = "<b>WARNINGS:</b><br>{0}".format("<br>".join(["{0} -> {1}".format(single_warning["tc"], single_warning["reason"]) for single_warning in warning_tc_list]))
                                color = " bgcolor=#FFFF33"
                            else:
                                content_per_build[campaign["tag"]]["test_verdict"] = False
                                result = "<b>FAILURE (TCR={})</b>".format(element["verdict"])
                                color = " bgcolor=#CC3333"
                        else:
                            content_per_build[campaign["tag"]]["test_verdict"] = False
                            result = "<b>FAILURE (TCR={})</b>".format(element["verdict"])
                            color = " bgcolor=#CC3333"

                    # still running
                    elif element["status"] == "RUNNING":
                        content_per_build[campaign["tag"]]["test_verdict"] = False
                        result = "<b>RUNNING</b>"
                        color = " bgcolor=#FF3300"
                        content_per_build[campaign["tag"]]["text_data"].append("<p style='color:red'>ERROR: Some TC still ongoing on {}/{}</p>".format(campaign["board_type"], campaign["build_target"]))
                    else:
                        content_per_build[campaign["tag"]]["test_verdict"] = False
                        result = "<b>{}</b>".format(element["status"].upper())
                        color = " bgcolor=#CC3333"
                        content_per_build[campaign["tag"]]["text_data"].append("<p style='color:red'>ERROR: {} status found</p>".format(element["status"].upper()))

                    for tc in element["failed_tc_list"]:
                        if tc["testCase"] == "BOOTOTA - FLASH - OTA FORWARD FLASHING" and tc["tcList"] and not any(single_tc["verdict"] != "VALID" for single_tc in tc["tcList"]):
                            result += "<br>(No OTA)"
                            content_per_build[campaign["tag"]]["text_data"].append("<p style='color:orange'>WARNING: No Full OTA test for "
                                                                                   "{}/{}</p>"
                                                                                   .format(campaign["board_type"],
                                                                                           campaign["build_target"]))
                        if tc["testCase"] == "BOOTOTA - FOTA - INCREMENTAL FOTA" and tc["tcList"] and not any(single_tc["verdict"] != "VALID" for single_tc in tc["tcList"]):
                            result += "<br>(No Incr FOTA)"
                            content_per_build[campaign["tag"]]["text_data"].append("<p style='color:orange'>WARNING: No Incremental OTA test for "
                                                                                   "{}/{}</p>"
                                                                                   .format(campaign["board_type"],
                                                                                           campaign["build_target"]))

                    content_per_build[campaign["tag"]]["html_data"].append("<tr><td>{0}</td><td>{1}</td><td>{2}</td><td{5}>{3}</td><td><a href='{4}' target='_blank'>TCR Link</a></td></tr>".
                                                                           format(campaign["board_type"],
                                                                                  campaign["build_target"],
                                                                                  campaign["campaign"] + " ({0} run TC)".format(element["executed"]),
                                                                                  result,
                                                                                  element["url"],
                                                                                  color))
                    single_tuple = [campaign["board_type"], campaign["build_target"], campaign["campaign"]]
                    if len(campaign["tcr_reports"]) > 1 and single_tuple not in warning_list:
                        content_per_build[campaign["tag"]]["text_data"].append("<p style='color:orange'>WARNING: Multiple campaign results found on TCR for "
                                                                               "{}/{}/{}</p>"
                                                                               .format(campaign["board_type"],
                                                                                       campaign["build_target"],
                                                                                       campaign["campaign"]))
                        warning_list.append(single_tuple)

        for tag in content_per_build:
            if content_per_build[tag]["test_verdict"]:
                content_per_build[tag]["text_data"].append("<p style='color:green'>INFO: Test Verdict = SUCCESS</p>")
            else:
                content_per_build[tag]["text_data"].append("<p style='color:red'>ERROR: Test Verdict = FAILURE</p>")

        # review if needed
        if review or force_review:
            for tag in content_per_build:
                if not content_per_build[tag]["test_verdict"] and force_review:
                    # print warning message in case the verdict is forced by test team
                    content_per_build[tag]["text_data"].append("<p style='color:orange'>WARNING: V+1 enforced by Test Team</p>")

                if content_per_build[tag]["test_verdict"] or force_review:
                    # review with V+1 if all tests successful or review forced by test team
                    cr1 = True
                else:
                    # review with V-1 if not all tests successful
                    cr1 = False

                message = "Patch tested in build: {0}".format(content_per_build[tag]["buildbot_url"])
                for campaign in [campaign for campaign in campaign_dict if campaign["tag"] == tag]:
                    for result in campaign.get("tcr_reports", []):
                        message += " - Results for {0}/{1}: {2}".format(campaign["board_type"], campaign["build_target"], result["url"])

                for patch in content_per_build[tag]["patch_list"]:
                    if cr1:
                        content_per_build[tag]["text_data"].append("<p style='color:green'>INFO: V+1 review for {0} patch</p>".format(patch["id"]))
                    else:
                        content_per_build[tag]["text_data"].append("<p style='color:red'>ERROR: V-1 review for {0} patch</p>".format(patch["id"]))
                    self.__reviewPatch(patch["commitId"], cr1, message=message)

        email_subject = "BootOta Test Status"
        email_core_text = ""
        for tag in content_per_build:
            # add end tags
            content_per_build[tag]["html_data"].append("</table>")
            content_per_build[tag]["html_data"].append("</div>")
            content_per_build[tag]["html_data"].append("<br>")
            content_per_build[tag]["text_data"].append("</div>")
            # compute email core data
            email_core_text += "<h3>{0}</h3>\n".format(tag)
            email_core_text += "\n".join(content_per_build[tag]["html_data"])
            email_core_text += "\n".join(content_per_build[tag]["text_data"])
            # add tag and jira in subject
            email_subject += " - {0} ({1})".format(tag, ", ".join(content_per_build[tag]["jira_list"]))

        html_content = """
            <html>
            <head>
            <style>
            table, th, td {
                border: 1px solid black;
                border-collapse: collapse;
                padding: 5px;
            }
            p {
                margin: 0em;
            }
            </style>
            </head>
            <body>
            <h2>BootOta Test Status</h2>\n""" + \
            email_core_text +\
            "</body></html>"

        # print html_content
        if emails:
            self.__logger.printLog("INFO", log + "email send to {0}".format(", ".join(emails)))
            self.__misc.sendMail(email_subject, html_content, MIMEType="html", format_html=False, To=emails)
        temp_output_file = os.path.join(tempfile.gettempdir(), "BootOtaStatus.html")
        if os.path.isfile(temp_output_file):
            os.remove(temp_output_file)
        with open(temp_output_file, "w") as f:
            f.write(html_content)
        self.__logger.printLog("INFO", log + "content stored at {0}".format(temp_output_file))
        self.__logger.printLog("INFO", log + "done")
