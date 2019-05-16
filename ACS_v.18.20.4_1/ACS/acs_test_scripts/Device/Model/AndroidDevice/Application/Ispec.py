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

from acs_test_scripts.Device.Model.AndroidDevice.Application.ITarball import ITarball
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


class Ispec(ITarball):
    """
    Implementation of Ispec as IApplication
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        ITarball.__init__(self, device)
        self.is_lower_better = False
        self._logger = device.get_logger()
        self._root_dir = "/data/spec"
        self._test_runner = "time"
        self._result = {"score": []}
        self._result_file = "final"
        self._result_file_builder = "outputNew"
        self.subTest_commands = {}
        self.available_tests = ["164.gzip", "176.gcc", "186.crafty", "252.eon", "254.gap", "256.bzip2", "175.vpr",
                                "181.mcf", "197.parser", "253.perlbmk", "255.vortex", "300.twolf"]

        self.subTest_details = {
            "164.gzip": {
                "gzip_base.cpu2000.v1.3.ic11.1.linux32.atom.oct192009": {
                    "arguments": [
                        ["input.source", "60"],
                        ["input.log", "60"],
                        ["input.graphic", "60"],
                        ["input.random", "60"],
                        ["input.program", "60"]
                    ],
                    "logs": [
                        ["input.source"],
                        ["input.log"],
                        ["input.graphic"],
                        ["input.random"],
                        ["input.program"]
                    ]
                }
            },
            "176.gcc": {
                "cc1_base.cpu2000.v1.3.ic11.1.linux32.atom.oct192009": {
                    "arguments": [
                        ["166.i", "-o", "166.s"],
                        ["200.i", "-o", "200.s"],
                        ["expr.i", "-o", "expr.s"],
                        ["integrate.i", "-o", "integrate.s"],
                        ["scilab.i", "-o", "scilab.s"]
                    ],
                    "logs": [
                        ["166"],
                        ["200"],
                        ["expr"],
                        ["integrate"],
                        ["scilab"]
                    ]
                }
            },
            "186.crafty": {
                "crafty_base.cpu2000.v1.3.ic11.1.linux32.atom.oct192009": {
                    "arguments": [
                        ["<", "crafty.in"]
                    ],
                    "logs": [
                        ["crafty"]
                    ]
                }
            },
            "252.eon": {
                "eon_base.cpu2000.v1.3.ic11.1.linux32.atom.oct192009": {
                    "arguments": [
                        ["chair.control.cook", "chair.camera", "chair.surfaces", "chair.cook.ppm", "ppm",
                         "pixels_out.cook"],
                        ["chair.control.rushmeier", "chair.camera chair.surfaces", "chair.rushmeier.ppm",
                         "ppm pixels_out.rushmeier"],
                        ["chair.control.kajiya", "chair.camera", "chair.surfaces", "chair.kajiya.ppm", "ppm",
                         "pixels_out.kajiya"]
                    ],
                    "logs": [
                        ["cook_log"],
                        ["rushmeier_log"],
                        ["kajiya_log"]
                    ]
                }
            },

            "254.gap": {
                "gap_base.cpu2000.v1.3.ic11.1.linux32.atom.oct192009": {
                    "arguments": [
                        ["-l", "./", "-q", "-m", "192M", "<", "ref.in"]
                    ],
                    "logs": [
                        ["ref"]
                    ]
                }
            },

            "256.bzip2": {
                "bzip2_base.cpu2000.v1.3.ic11.1.linux32.atom.oct192009": {
                    "arguments": [
                        ["input.source", "58"],
                        ["input.graphic", "58"],
                        ["input.program", "58"]
                    ],
                    "logs": [
                        ["input.source"],
                        ["input.graphic"],
                        ["input.program"]
                    ]
                }
            },

            "175.vpr": {
                "vpr_base.cpu2000.v1.3.ic11.1.linux32.atom.oct192009": {
                    "arguments": [
                        ["net.in", "arch.in", "place.out", "dum.out", "-nodisp", "-place_only", "-init_t", "5",
                         "-exit_t", "0.005", "-alpha_t", "0.9412", "-inner_num", "2"],
                        ["net.in", "arch.in", "place.in", "route.out", "-nodisp", "-route_only", "-route_chan_width",
                         "15", "-pres_fac_mult", "2", "-acc_fac", "1", "-first_iter_pres_fac", "4",
                         "-initial_pres_fac 8"]
                    ],
                    "logs": [
                        ["place_log"],
                        ["route_log"]
                    ]
                }
            },

            "181.mcf": {
                "mcf_base.cpu2000.v1.3.ic11.1.linux32.atom.oct192009": {
                    "arguments": [
                        ["inp.in"]
                    ],
                    "logs": [
                        ["inp"]
                    ]
                }
            },

            "197.parser": {
                "parser_base.cpu2000.v1.3.ic11.1.linux32.atom.oct192009": {
                    "arguments": [
                        ["2.1.dict", "-batch", "<", "ref.in"]
                    ],
                    "logs": [
                        ["ref"]
                    ]
                }
            },

            "253.perlbmk": {
                "perlbmk_base.cpu2000.v1.3.ic11.1.linux32.atom.oct192009": {
                    "arguments": [
                        ["-I", "./lib", "diffmail.pl", "2 550 15 24 23 100"],
                        ["-I", "./lib", "makerand.pl"],
                        ["-I", "./lib", "perfect.pl", "b 3 m 4"],
                        ["-I", "./lib", "splitmail.pl", "850 5 19 18 1500"],
                        ["-I", "./lib", "splitmail.pl", "704 12 26 16 836"],
                        ["-I", "./lib", "splitmail.pl", "535 13 25 24 1091"],
                        ["-I", "./lib", "splitmail.pl", "957 12 23 26 1014"]
                    ],
                    "logs": [
                        ["2.550.15.24.23.100"],
                        ["makerand"],
                        ["b.3.m.4"],
                        ["850.5.19.18.1500"],
                        ["704.12.26.16.836"],
                        ["535.13.25.24.1091"],
                        ["957.12.23.26.1014"]
                    ]
                }
            },
            "255.vortex": {
                "vortex_base.cpu2000.v1.3.ic11.1.linux32.atom.oct192009": {
                    "arguments": [
                        ["lendian1.raw"],
                        ["lendian2.raw"],
                        ["lendian3.raw"]
                    ],
                    "logs": [
                        ["vortex1"],
                        ["vortex2"],
                        ["vortex3"]
                    ]
                }
            },
            "300.twolf": {
                "twolf_base.cpu2000.v1.3.ic11.1.linux32.atom.oct192009": {
                    "arguments": [
                        ["ref"]
                    ],
                    "logs": [
                        ["ref"]
                    ]
                }

            }
        }

        self.ispec_const = {
            "164.gzip": 1400,
            "176.gcc": 1100,
            "186.crafty": 1000,
            "252.eon": 1300,
            "254.gap": 1100,
            "256.bzip2": 1500,
            "175.vpr": 1400,
            "181.mcf": 1800,
            "197.parser": 1800,
            "253.perlbmk": 1800,
            "255.vortex": 1900,
            "300.twolf": 3000
        }

        self.ispec = {
            "164.gzip": ["input.source", "input.log", "input.graphic", "input.random", "input.program"],
            "176.gcc": ["166.i", "200.i", "expr.i", "integrate.i", "scilab.i"],
            "186.crafty": ["crafty.in"],
            "252.eon": ["cook", "rushmeier", "kajiya"],
            "254.gap": ["ref.in"],
            "256.bzip2": ["input.source", "input.graphic", "input.program"],
            "175.vpr": ["place.out", "route.out"],
            "181.mcf": ["inp.in"],
            "197.parser": ["ref.in"],
            "253.perlbmk": ["diffmail.pl", "makerand.pl", "perfect.pl", "splitmail1.pl", "splitmail2.pl",
                            "splitmail3.pl", "splitmail4.pl"],
            "255.vortex": ["lendian1.raw", "lendian2.raw", "lendian3.raw"],
            "300.twolf": ["ref"]
        }

        self.threads_nb = 1
        self.ispec_report_path = None
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

            for arguments in subtests_args:
                for argument in arguments:
                    parameters += argument
                    parameters += " "

                log_file = subtests_logs[index][0]
                index += 1
                parameters += "> "
                parameters += log_file + "." + str(self._run_no) + ".out "
                parameters += "2>> "
                parameters += log_file + "." + str(self._run_no) + ".err "
                mycommand = command + parameters
                parameters = ""
                subtest_cmd.append(mycommand)

        self.subTest_commands[sub_benchmark_name] = subtest_cmd

    def _getSubtest_cmd(self, sub_benchmark_name):
        """
        Compute current command to launch depending on current benchmark
        :type sub_benchmark_name: str
        :param sub_benchmark_name: The name of the executable to launch
        """
        return self.subTest_commands[sub_benchmark_name]


    def _run_command(self, benchmark, command, timeout):
        """
        Run benchmark command

        :type benchmark: str
        :param benchmark: The name of the benchmark to run

        :type command: str
        :param command: The command to run

        :type timeout: integer
        :parame timeout: Timeout until the benchmark should end
        """
        script_path = self._root_dir + "/" + benchmark
        self._logger.info("Run %s with %d threads %d" %
                          (benchmark, self.threads_nb, self._run_no))

        self.adb_shell("cd %s && nohup  %s " % (script_path, command), 120)

        self._wait_for_application("../" + self._test_runner, timeout)

    def _run_benchmark(self, sub_benchmark_name, timeout):
        """
        Run the benchmark in parameter

        :type sub_benchmark_name: str
        :param sub_benchmark_name: The name of the benchmark to run

        :type timeout: integer
        :parame timeout: Timeout until the benchmark should end
        """

        cmdList = self._getSubtest_cmd(sub_benchmark_name)
        for cmd in cmdList:
            testcmd = cmd
            self._run_command(sub_benchmark_name, testcmd, 1800)

    def fetch_result(self, result_file):
        """
        Fetch result of the benchmark and store the result file in
        report tree

        :type result_file: str
        :param result_file: The name of the output file of the benchmark
        """
        command = "%s.sh" % self._result_file_builder
        self.adb_shell("cd %s &&  ./%s" % (self._root_dir, command), 100)
        self.adb_shell("echo %s" % command, 100)
        self.adb_pull("%s/%s" % (self._root_dir, result_file), self.ispec_report_path, 180)
        score_list = {}
        with open(self.ispec_report_path + "/" + result_file, 'r') as outputfile:
            subtest = {}
            score = 0
            subtest = ""

            for line in outputfile:
                if line.startswith("<=- "):
                    if len(subtest) is not 0:
                        self._logger.debug("Generate Score for %s" % line.split()[1])
                        score_list[subtest] = score
                        subtest = ""
                        score = 0

                    subtest = line.split()[1]
                elif line.startswith("real"):
                    splitted = line.split("\t")
                    score_sec = self.toseconds(splitted[1].strip("\n"))
                    self._logger.debug("score ==>  score: %f", score_sec)
                    score += score_sec

            """For last line in file"""
            score_list[subtest] = score

        score_category_list = []
        for category, score in score_list.iteritems():
            self._logger.debug("Category score  (%s) => score (%f)", category, score)
            category_score = (self.ispec_const[category] / score) * 100
            score_category_list.append(category_score)

        benchmark_result = scistats.gmean(score_category_list)
        self._logger.debug("Score for %d tests %d", len(score_category_list), benchmark_result)
        self._results["score"] = benchmark_result

    def _get_TestDetails(self, sub_benchmark_name):
        sub_benchmark_details = self.subTest_details[sub_benchmark_name]
        command = ""
        subtest_cmd = []
        for key, values in sub_benchmark_details.iteritems():
            subtests_args = sub_benchmark_details[key]["arguments"]
            subtests_logs = sub_benchmark_details[key]["logs"]
            command = command + "./" + key + " "
            parameters = ""
            index = 0

            for arguments in subtests_args:
                for argument in arguments:
                    parameters = parameters + argument
                    parameters += " "

                log_file = subtests_logs[index][0]
                index += 1
                parameters += "> "
                parameters = parameters + log_file + "." + str(self._run_no) + ".out "
                parameters += "2>> "
                parameters = parameters + log_file + "." + str(self._run_no) + ".err "
                mycommand = command + parameters
                parameters = ""
                subtest_cmd.append(mycommand)

        self.subTest_commands[sub_benchmark_name] = subtest_cmd

    def toseconds(self, atime):
        atime = atime.strip()
        splitted = atime.split()
        minutes = 0.0
        seconds = 0.0
        for s in splitted:
            if s.endswith("m"):
                minutes = float(s.strip("m"))
            elif s.endswith("s"):
                seconds = float(s.strip("s"))

        time_insec = 60 * minutes + seconds

        return time_insec

    def __get_section_score(self, section):
        """
        Get the score of the session

        :type section: str
        :param section: The name of the section to compute score
        """
        accum = 1
        self._logger.info("Score of %s is %f" % (section, self._results["score"]))
        return self._results["score"]

    def get_score(self, _stat_type=None):
        """
        Return the score of benchmark run
        """
        xmlresult = etree.Element("scores")
        xmlnode = etree.Element("item")
        xmlnode.attrib["name"] = "score"
        xmlnode.attrib["value"] = str(self._results["score"])
        xmlresult.append(xmlnode)
        return xmlresult

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

        result_file = "%s%d.txt" % (self._result_file, self._run_no)
        self.fetch_result(result_file)

    def post_install(self):
        """
        Post install actions
        """
        self.threads_nb = int(self._phonesystem.get_cpu_info(["cpu cores"])[0])
        result_folder = "ispec_result_%dt" % self.threads_nb
        report_dir = self._get_report_tree()
        report_dir.create_subfolder(result_folder)
        self.ispec_report_path = report_dir.get_subfolder_path(result_folder)

    def remove_previous_output(self):
        """
            Remove old result files
        """
        cmd = "cd %s && rm -f final*" % self._root_dir
        output = self.adb_shell(cmd, 10)
        for test_folder in self.available_tests:
            path = self._root_dir + "/" + test_folder
            cmd = "cd %s && rm -f *.out" % (path)
            output = self.adb_shell(cmd, 10)
            cmd = "cd %s && rm -f *.err" % (path)
            output = self.adb_shell(cmd, 10)