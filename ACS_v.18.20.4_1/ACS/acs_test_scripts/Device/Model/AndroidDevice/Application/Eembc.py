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
:summary: This script implements the EEMBC benchmark for performance measurement
:since: 17/04/2013
:author: pbluniex
"""
from acs_test_scripts.Device.Model.AndroidDevice.Application.ITarball import ITarball
import os
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from lxml import etree


class Eembc(ITarball):

    """
    Implementation of caffeinemark as Application
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        ITarball.__init__(self, device)
        self.is_lower_better = False
        self._path_ref = os.path.join("BENCHMARKS", "EEMBC")
        self.__search_pattern = "Iterations\/Sec[ ]*=[ ]*[0-9.]*"

        self.__available_tests = ["AUT", "CON", "NET", "OFW", "TEL"]

        self.__eembc_const = {"AUT": 307.455,
                              "CON": 2.192,
                              "NET": 395.184,
                              "OFW": 1.139,
                              "TEL": 785.138}

        # pktflowb4m_lite, bezier01fixed_lite, and bezier01fixed_float are not
        # run for now.
        self.__eembc = {"AUT": ["a2time01_lite", "aifftr01_lite", "aifirf01_lite",
                        "aiifft01_lite", "basefp01_lite", "bitmnp01_lite",
                        "cacheb01_lite", "canrdr01_lite", "idctrn01_lite",
                        "iirflt01_lite", "matrix01_lite", "pntrch01_lite",
                        "puwmod01_lite", "rspeed01_lite", "tblook01_lite",
                        "ttsprk01_lite"],

                        "CON": ["cjpeg_lite", "djpeg_lite", "rgbcmy01_lite",
                                "rgbhpg01_lite", "rgbyiq01_lite"],

                        "NET": ["ospf_lite", "routelookup_lite", "pktflowb512k_lite",
                                "pktflowb1m_lite", "pktflowb2m_lite"],

                        "OFW": ["text01_lite", "rotate01_lite", "dither01_lite"],

                        "TEL": ["autcor00data_1_lite", "autcor00data_2_lite",
                                "autcor00data_3_lite", "conven00data_1_lite",
                                "conven00data_2_lite", "conven00data_3_lite",
                                "fbital00data_2_lite", "fbital00data_3_lite",
                                "fbital00data_6_lite", "fft00data_1_lite",
                                "fft00data_2_lite", "fft00data_3_lite",
                                "viterb00data_1_lite", "viterb00data_2_lite",
                                "viterb00data_3_lite", "viterb00data_4_lite"]}

        self.__threads_nb = 1
        self.__eembc_report_path = None
        self._parameters = None

    def __prepare_cmd(self, benchmark_name):
        """
        Compute current command to launch depending on current benchmark

        :type benchmark_name: str
        :param benchmark_name: The name of the executable to launch
        """
        script = ""
        command = "%s.sh" % benchmark_name
        thread_cmd = "./%s.exe &\n" % benchmark_name

        for _ in range(0, self.__threads_nb):
            script += thread_cmd

        if not script:
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                     "No command to run.")

        self.adb_shell("echo '%s' > %s/%s" % (script, self._root_dir, command), 3)
        return command

    def __run_command(self, benchmark, command, result_file, timeout):
        """
        Run benchmark command

        :type benchmark: str
        :param benchmark: The name of the benchmark to run

        :type command: str
        :param command: The command to run

        :type result_file: str
        :param result_file: The name of the file where the result of the
                            benchmark will be dumped

        :type timeout: integer
        :parame timeout: Timeout until the benchmark should end
        """
        cmd = "cd %s && chmod 755 %s.exe" % (self._root_dir, benchmark)
        self.adb_shell(cmd, 3)

        self._logger.info("Run %s with %d threads %d" %
                          (benchmark, self.__threads_nb, self._run_no))

        self.adb_shell("cd %s && nohup sh %s > %s" %
                       (self._root_dir, command, result_file), 3)

        self._wait_for_application(benchmark, timeout)

    def __run_benchmark(self, benchmark_name, timeout):
        """
        Run the benchmark in parameter

        :type benchmark: str
        :param benchmark: The name of the benchmark to run

        :type timeout: integer
        :parame timeout: Timeout until the benchmark should end
        """
        cmd = self.__prepare_cmd(benchmark_name)
        self._results[benchmark_name] = []
        result_file = "%s-t%d-r%d.log" % (benchmark_name,
                                          self.__threads_nb,
                                          self._run_no)

        self.__run_command(benchmark_name, cmd, result_file, timeout)
        self.__fetch_result(benchmark_name, result_file)

    def __fetch_result(self, benchmark, result_file):
        """
        Fetch result of the benchmark and store the result file in
        report tree

        :type benchmark: str
        :param benchmark: The name of the benchmark

        :type result_file: str
        :param result_file: The name of the output file of the benchmark
        """
        output = self.adb_shell("grep -o -e '%s' %s/%s||echo NOk" %
                                (self.__search_pattern, self._root_dir,
                                 result_file), 3)

        if output == "NOk":
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Can not fetch result for '%s'" % benchmark)

        lines = output.splitlines()
        bench_result = []
        for line in lines:
            result = line.split("=")[1]
            result = result.strip(" \n")
            bench_result.append(float(result))

        self._results[benchmark].append(sum(bench_result))
        self._logger.debug("Intermediate result for %s : %f (%s)" %
                          (benchmark, sum(bench_result), str(bench_result)))

        self.adb_pull("%s/%s" % (self._root_dir, result_file),
                      self.__eembc_report_path, 60)

    def __get_section_score(self, section):
        """
        Get the score of the session

        :type section: str
        :param section: The name of the section to compute score
        """
        accum = 1

        for key in self.__eembc[section]:
            scores = self._results.get(key)
            if scores is None:
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Can not get score for %s" % key)

            accum = accum * max(scores)

        sec_score = accum ** (1.0 / len(self.__eembc[section])) / self.__eembc_const[section]
        self._logger.info("Score of %s is %f" % (section, sec_score))

        return sec_score

    def get_score(self, _stat_type=None):
        """
        Return the score of benchmark run
        """
        scores = []
        for key in self.__available_tests:
            sec_score = self.__get_section_score(key)
            scores.append(sec_score)

        eembc_score = 1
        for score in scores:
            eembc_score = eembc_score * score

        xmlresult = etree.Element("scores")
        xmlnode = etree.Element("item")
        xmlnode.attrib["name"] = "score"
        xmlnode.attrib["value"] = str(eembc_score ** (1.0 / len(scores)))
        xmlresult.append(xmlnode)

        return xmlresult

    def __parse_arguments(self):
        """
        Parse arguments
        """
        if self._arguments is None:
            return

        configs = ("".join(line.strip()
                           for line in self._arguments.strip(" ;\n").splitlines())).split(";")

        arguments = {}
        for conf in configs:
            args = [x.strip(" \n").lower() for x in conf.split(":")]
            arguments[args[0]] = args[1]
        self._logger.info(arguments)
        self._parameters = arguments

    def wait(self, timeout):
        """
        Drive the application to run the benchmark

        :type timeout: integer
        :parame timeout: Timeout until the benchmark should end
        """
        self._run_no += 1
        benchmarks = []
        for key in self.__available_tests:
            benchmarks = benchmarks + self.__eembc[key]
        base_name = os.path.split(self._application_uri)[1]
        self._root_dir = "/data/%s" % base_name.split(".")[0]
        self._logger.debug("Root directory : %s" % self._root_dir)
        self._logger.debug("Run benchmark : %s" % str(benchmarks))

        cur = 0
        nb = len(benchmarks)
        for cur_benchmark in benchmarks:
            cur += 1
            self._logger.info("Start running %s (%d of %d)" %
                              (cur_benchmark, cur, nb))
            self.__run_benchmark(cur_benchmark, timeout)

    def post_install(self):
        """
        Post install actions
        """
        self.__threads_nb = int(self._phonesystem.get_cpu_info(["cpu cores"])[0])

        result_folder = "eembc_result_%dt" % self.__threads_nb
        self.__parse_arguments()
        report_dir = self._get_report_tree()
        report_dir.create_subfolder(result_folder)
        self.__eembc_report_path = report_dir.get_subfolder_path(result_folder)
