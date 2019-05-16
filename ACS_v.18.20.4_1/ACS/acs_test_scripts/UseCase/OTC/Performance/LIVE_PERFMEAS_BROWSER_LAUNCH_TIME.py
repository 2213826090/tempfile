"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG PSI
:summary: Use Case base for applications based power measurement
:since: 20/02/2013
:author: pbluniex
"""

import os
import time
import re
import math
import numpy
import subprocess
import requests
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.PnPUtilities import PnPResults
from Core.Report.SecondaryTestReport import SecondaryTestReport
from lxml import etree


class BrowserLaunchTime(UseCaseBase):

    """
    Class live to run benchmarks application
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor

        :type tc_name: BaseConf
        :param tc_name: Configuration of the usecase

        :type global_config: Dictionnary
        :param global_config: Global configuration of the campaign
        """
        # Call power measurement base Init function
        UseCaseBase.__init__(self, tc_name, global_config)


        self.__failure_file = os.path.join(self._execution_config_path,
                                           self._device.get_config("FailureFile"))
        self.__target_file = os.path.join(self._execution_config_path,
                                          self._device.get_config("TargetFile"))
        self._report_tree = global_config.campaignConfig.get("campaignReportTree")
        self._tc_name = os.path.basename(self.get_name())
        self._tc_date = ""
        self._secondary_report = SecondaryTestReport(
            self._device.get_report_tree().get_report_path())
        self._sysdebug_apis = self._device.get_uecmd("SysDebug")
        self.screen_off_time = 0
        attributes = {"id": self._tc_name,
                      "date": self._tc_date,
                      "verdict": "NOT EXECUTED"}

        self._secondary_report = SecondaryTestReport(
            self._device.get_report_tree().get_report_path())

        self.__results = PnPResults(self._report_tree,
                                    self._dut_config.get("Name"),
                                    self.__failure_file,
                                    self.__target_file,
                                    attributes)
        self._results = {}

    def __fetch_artifactory_file(self, appuri):
        """
        Retrieve a file from artifactory
        """
        artifact_url_base = "https://mcg-depot.intel.com/artifactory/acs_test_artifacts/"
        artifact_url = artifact_url_base + appuri

        local_path = os.path.join(self._execution_config_path, "EMBD")
        local_filename = os.path.join(local_path, os.path.basename(appuri))
        local_filename = local_filename.split(";")[0]

        if not os.path.exists(local_path):
            os.mkdir(local_path)

        if os.path.exists(local_filename):
            return local_filename

        self._device.get_logger().info("Download %s => %s" % (artifact_url, local_filename))

        start = time.time()
        r = requests.get(artifact_url, proxies={'http': ''})
        r.raise_for_status()

        length = float(r.headers['content-length'])
        progressize = 0

        with open(local_filename, "wb") as f:
            for chunk in r.iter_content(chunk_size=1024000):
                if chunk:
                    progressize += len(chunk)
                    percent = int(progressize / length * 100)
                    self._logger.debug("progress of downloading %s file: %d%%" %
                                       (appuri, percent))
                    f.write(chunk)

        dlduration = time.time() - start
        self._device.get_logger().info("Download %s in %f seconds (birate: %d MiO/s)" %
                          (appuri, dlduration, length / (dlduration * 8000000)))

        return local_filename


    def get_score(self, stat_type="MEDIAN"):
        """
        Get the score of the application
        """
        if stat_type == "ARITHMETIC_MEAN":
            functor = numpy.mean
        elif stat_type == "GEOMETRIC_MEAN":
            functor = scipy.stats.mstats.gmean
        else:
            stat_type = "MEDIAN"
            functor = numpy.median

        xmlresult = etree.Element("scores")
        keys = sorted(self._results.keys())
        if "score" in keys:
            keys.remove("score")
            keys.insert(0, "score")

        for key in keys:
            value = self._results[key]

            xmlnode = etree.Element("item")
            result = str(functor(value))
            runs = ";".join([str(x) for x in value])
            xmlnode.attrib["name"] = key
            xmlnode.attrib["value"] = str(result)
            xmlnode.attrib["runs"] = runs
            self._device.get_logger().debug("%s: %s => %s" % (key, str(value), result))

            xmlresult.append(xmlnode)

            if key == "score":
                self._device.get_logger().info("Result Browser score : %s (%s)"% (result, stat_type))
        return xmlresult


    def set_up(self):
        """
        Set up the test configuration

        :rtype: tuple
        :return: tuple of Verdict and comment
        """
        #Download and push uiautomator jar on the device
        arguments = self._tc_parameters.get_param_value("ARGUMENTS")
        addr = self.__fetch_artifactory_file(arguments.split("artifact://")[1])

        result, output = self._device.push(addr.split(";")[0], "/data/local/tmp/", 60)

        res, output = self._device.run_cmd("adb shell sqlite3 /data/data/com.android.providers.settings/databases/settings.db\
                                           \"select value from system where name='screen_off_timeout'\"", 5)
        if not res == Global.SUCCESS:
            return Global.FAILURE, "No output detected"

        self.screen_off_time = int(output)

        #Set screen off timeout to 30 min
        self._device.run_cmd("adb shell sqlite3 /data/data/com.android.providers.settings/databases/settings.db\
                             \"update system set value='1800000' where name='screen_off_timeout'\"", 5)


        #Cold measurement, reboot before measure
        self._device.reboot()

        #unlock screen
        self._device.run_cmd("adb shell input keyevent 82", 5)
        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Run the test

        :rtype: tuple
        :return: tuple of Verdict and comment
        """
        #Launch Browser as user
        self._device.run_cmd("adb shell uiautomator runtest Performance.jar -c other.BrowserLaunch", 120)

        #Filter the output
        res, out = self._device.run_cmd("adb logcat -d | \
                        grep \"Displayed com.android.browser/.BrowserActivity\" | cut -d ' ' -f5-", 5)

        if not res == Global.SUCCESS:
            return Global.FAILURE, "No output detected"
        out = out.replace("\r\n", "\n")

        err = re.search( r'Exception', out)
        if not err == None:
            return Global.FAILURE, "uiautomator test faield"

        #Get raw result
        out = re.search( r'[0-9]+', out)
        if out == None:
            return Global.FAILURE, "No answer detected"

        #Compare result with the target file's entry
        self._device.get_logger().info("Result Settings score : %s MEDIAN", out.group(0))
        stat_type = self._tc_parameters.get_param_value("STAT_TYPE")
        scores = self.get_score(stat_type)
        xmltree = self._sysdebug_apis.report()

        self.__results.append(scores)
        self.__results.append(xmltree)

        try:
            ilb = True
            verdict, output = self.__results.get_performance_verdict(scores, ilb,
                                                                     self._secondary_report,
                                                                     self.tc_order)
            self._device.get_logger().info(output)
        finally:
            self.__results.write()

        return verdict, output


    def tear_down(self):
        """
        Tear down

        :rtype: tuple
        :return: tuple of Verdict and comment
        """
        self._device.run_cmd("adb shell sqlite3 /data/data/com.android.providers.settings/databases/settings.db\
                             \"update system set value='"+ str(self.screen_off_time) + "' where name='screen_off_timeout'\"", 5)
        return Global.SUCCESS, "No errors"
