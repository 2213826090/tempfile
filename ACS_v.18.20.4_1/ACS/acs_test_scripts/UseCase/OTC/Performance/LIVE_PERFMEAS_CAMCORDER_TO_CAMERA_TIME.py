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

:organization: OTC
:summary: Use Case base for camera launch to preview
:since: 20/05/2014
:author: cimihaix
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


class CamcorderToCameraSwitching(UseCaseBase):

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

        self._artifact_manager = self._em.get_artifact_manager("ARTIFACT_MANAGER")

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
                self._device.get_logger().info("Result Contacts score : %s (%s)"% (result, stat_type))
        return xmlresult

    def runBashCommand(self, command):
        '''
        Brief:
        Runs a bash command and waits for it
        Param:
        command - the command to be executed

        Atention!: Take care to escape properly the special characters
        '''

        p = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        retval = p.wait()
        return p.stdout.readlines()

    def set_up(self):
        """
        Set up the test configuration

        :rtype: tuple
        :return: tuple of Verdict and comment
        """

       #Download and push uiautomator jar on the device
        arguments = self._tc_parameters.get_param_value("ARGUMENTS")

        local_artifact = self._artifact_manager.get_artifact(artifact_name=arguments, transfer_timeout=3600)

        if os.path.isfile(local_artifact):
            result, output = self._device.push(local_artifact, "/data/local/tmp/", 60)
            if result != Global.SUCCESS:
                return result, output
        else:
            return Global.BLOCKED, "Cannot find {0}".format(local_artifact)

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Run the test

        :rtype: tuple
        :return: tuple of Verdict and comment
        """
        ITERATIONS = 3
        CCtoCavg = 0
        self.runBashCommand("adb shell uiautomator runtest Performance.jar -c other.StartCamera")

        for x in range(0, ITERATIONS):
            self.runBashCommand("adb shell uiautomator runtest Performance.jar -c other.CameraCamcorderSwitch")

            time.sleep(3)

            out = self.runBashCommand("adb logcat -d -v time | grep \"startDevice\\|camera_cancel_auto_focus\\|closeCamera\"")
            rawtimestamps = []


            for line in out:
                res = re.findall( r'[0-9]+:[0-9]+:[0-9]+.[0-9]+', line)

                rawtimestamps.append(res[0])


            numericTimestamp = re.findall( r'[0-9]+', rawtimestamps[2])

            initialTimestamp = 3600 * 1000 * int(numericTimestamp[0]) + 60 * 1000 * int(numericTimestamp[1]) + 1000 * int(numericTimestamp[2]) + int(numericTimestamp[3]);

            numericTimestamp = re.findall( r'[0-9]+', rawtimestamps[3])

            finalTimestamp = 3600 * 1000 * int(numericTimestamp[0]) + 60 * 1000 * int(numericTimestamp[1]) + 1000 * int(numericTimestamp[2]) + int(numericTimestamp[3]);

            CCtoCavg += (finalTimestamp - initialTimestamp)


        self._device.get_logger().info("Result Camera to preview launch time : %s MEDIAN", str(CCtoCavg/ITERATIONS))
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
        self._device.run_cmd("adb shell input keyevent KEYCODE_ESCAPE", 5)
        self._device.run_cmd("adb shell input keyevent KEYCODE_ESCAPE", 5)
        return Global.SUCCESS, "No errors"
