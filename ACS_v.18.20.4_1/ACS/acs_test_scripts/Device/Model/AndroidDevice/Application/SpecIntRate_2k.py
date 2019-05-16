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

:organization: INTEL PCCG PSTV
:summary: This script implements the SpecIntRate 2k
:since: 21/08/2014
:author: nssumalx
"""
import os
from lxml import etree
from scipy import stats as scistats
import json
import time
import subprocess

from acs_test_scripts.Device.Model.AndroidDevice.Application.Ispec import Ispec
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


class SpecIntRate_2k(Ispec):
    """
     Implementation of SpecIntRate as IApplication
     """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        self._logger = device.get_logger()
        Ispec.__init__(self, device)
        self._result_file_builder = "output_specInt_rate_4t"
        self._result_file = "final-4T.txt"
        self._run_no = 0
        self._path_ref = os.path.join("BENCHMARKS", "SPEC2000")

    def _prepare_subtest_command(self, sub_benchmark_name):

        sub_benchmark_details = self.subTest_details[sub_benchmark_name]
        command = "../" + self._test_runner + " "
        subtest_cmd = []
        for key, values in sub_benchmark_details.iteritems():
            subtests_args = sub_benchmark_details[key]["arguments"]
            subtests_logs = sub_benchmark_details[key]["logs"]
            command = command + "./" + key + " "
            parameters = ""
            index = 0
            sub_cmd = ""
            for arguments in subtests_args:
                for count in [1, 2, 3, 4]:
                    for argument in arguments:
                        parameters += argument
                        parameters += " "
                    log_file = subtests_logs[index][0]
                    parameters += "> "
                    parameters += log_file + "." + str(count) + ".out "
                    parameters += "2>> "
                    parameters += log_file + "." + str(count) + ".err "
                    mycommand = command + parameters
                    parameters = ""
                    if count != 4:
                        mycommand += "&"
                    sub_cmd += mycommand
                    sub_cmd += "\n"
                index += 1
            subtest_cmd.append(sub_cmd)
        self.subTest_commands[sub_benchmark_name] = subtest_cmd

    def fetch_result(self, result_file):
        """
        Fetch result of the benchmark and store the result file in
        report tree

        :type result_file: str
        :param result_file: The name of the output file of the benchmark
        """
        command = "%s.sh" % self._result_file_builder
        self.adb_shell("cd %s &&  ./%s" % (self._root_dir, command), 100)
        self.adb_shell("echo %s" % (command), 100)

        self.adb_pull("%s/%s" % (self._root_dir, result_file), self.ispec_report_path, 180)
        score_list = {}
        with open(self.ispec_report_path + "/" + result_file, 'r') as outputfile:
            score = 0
            subtest_score =0
            subtest = ""
            subtest_score_list=[]
            for line in outputfile:
                if line.startswith("<=- "):
                    if len(subtest_score_list) > 0:
                        subtest_score = max(subtest_score_list)
                        score += subtest_score
                        self._logger.debug("score for subtest %s is %f" % (subtest, subtest_score))
                    subtest_score_list =[]
                    if len(subtest) is not 0:
                        self._logger.debug("Generated Score for category %s is  %f" %(test_category,score))
                        score_list[test_category] = score
                        subtest = ""
                        score = 0
                        subtest_score = 0
                    test_category = line.split()[1]
                    self._logger.debug("Test category is %s " % test_category)

                elif line.startswith("real"):
                    if len(subtest) == 0:
                        score = 0
                    splitted = line.split("\t")
                    score_sec = self.toseconds(splitted[1].strip("\n"))
                    subtest_score_list.append(score_sec)

                elif len(line) == 1:
                    score += subtest_score

                elif len(line) > 0:
                    if len(subtest_score_list) > 0:
                        subtest_score = max(subtest_score_list)
                        self._logger.debug("score for subtest %s is %f" % (subtest, subtest_score))

                    subtest_score_list = []
                    for test in self.ispec.iterkeys():
                        if test == test_category:
                            for sub_test in self.ispec[test]:
                                if line.startswith(sub_test):
                                    subtest = sub_test
                                    break

            """For last line in file"""
            subtest_score = max(subtest_score_list)
            score += subtest_score
            self._logger.debug("score for subtest %s is %f" % (subtest, subtest_score))
            self._logger.debug("Generated Score for category %s is  %f" % (test_category, score))
            score_list[test_category] = score

        score_category_list = []
        for category, score in score_list.iteritems():
            category_score = (1.16*4*(self.ispec_const[category] / score))
            score_category_list.append(category_score)

        benchmark_result = scistats.gmean(score_category_list)
        self._logger.debug("Score for %d tests %d", len(score_category_list), benchmark_result)
        self._results["score"] = benchmark_result

    def _run_command(self, benchmark, commands, timeout):
        script_path = self._root_dir + "/" + benchmark
        counter = 0
        for command in commands.split("\n"):
            counter += 1
            command = command.strip()
            if counter != 4:
                self.adb_shell("cd %s && nohup  %s& echo OK" % (script_path, command), 1)
            if counter == 4:
                self.adb_shell("cd %s && nohup  %s && echo OK" % (script_path, command), 1)
                self._logger.info("Wait started")
                self._wait_for_application("../" + self._test_runner, timeout)
                counter = 0

    def wait(self, timeout):
        """
            Drive the application to run the benchmark

            :type timeout: integer
            :parame timeout: Timeout until the benchmark should end
            """
        self._run_no += 1
        benchmarks = ""
        cur = 0
        nb = len(benchmarks)
        self.remove_previous_output()
        benchmark_count = len(self.available_tests)
        for cur_benchmark in self.available_tests:
            cur += 1
            self._logger.info("Start running %s (%d of %d)" % (cur_benchmark, cur, benchmark_count))
            self._prepare_subtest_command(cur_benchmark)
            self._run_benchmark(cur_benchmark, timeout)

        result_file = "%s" % self._result_file
        self.fetch_result(result_file)