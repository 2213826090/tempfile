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

:organization: OTC ANDROID QA
:summary: Enable running CTS in ACS
:since: 15/10/14
:author: rcstamat
"""

import __builtin__
import os
import stat
import shutil
import time

import lxml.etree as et

import UtilitiesFWK.Utilities as Utils
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from Core.Report.SecondaryTestReport import SecondaryTestReport
from ErrorHandling.AcsToolException import AcsToolException
from UtilitiesFWK.Utilities import Global
from Core.PathManager import Folders
from acs_test_scripts.Utilities.HttpDownloaderUtil import HttpDownloaderUtil

SysErrors = __builtin__.OSError, __builtin__.IOError,

CTS_RESULT_FILENAME = "testResult.xml"
CTS_EXEC_FILENAME = "cts-tradefed"
CTS_CONTINUE_SESSION_COMMAND = " run cts --continue-session 0"

class CTS(UseCaseBase):

    """
    Execute CTS tests on a device.

    First of all, have a look to:
    http://source.android.com/compatibility/cts-intro.html

    CTS test plan is divided by package (full plan ~ 80 packages).
    CTS release AND plan evolve for each android version.

    To run the full test plan properly, we need:
    - wifi AP with internet access
    - cts framework zip file
    - cts media zip file
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

    def initialize(self):
        """
        Process the **<Initialize>** section of the XML file and execute defined test steps.
        """
        UseCaseBase.initialize(self)
        result, output = Global.SUCCESS, ""

         # Retrieve artifact manager
        self._cts_path = os.path.join(Folders.ACS_CACHE, 'Artifacts', self._tc_parameters.get_param_value("CTS_LOCAL_PATH"))
        self._cts_media_path = self._tc_parameters.get_param_value("CTS_MEDIA_PATH")
        self._cts_result_comparison = self._tc_parameters.get_param_value("CTS_REPORT_COMPARISON_PATH")
        self._test_cmd_lines = self._tc_parameters.get_param_value("TEST_CMD_LINES")
        self._test_timeout = self._tc_parameters.get_param_value("TEST_TIMEOUT", default_cast_type=float)
        self._test_failed_retry_no = self._tc_parameters.get_param_value("TEST_FAILED_RETRY_NO", default_value=0, default_cast_type=int)
        # Add a secondary report
        self.__tc_report = SecondaryTestReport(self._device.get_report_tree().get_report_path())
        self._cts_exec_path = ""

        return result, output

    def _extract_cts_results_from_cts_report(self, path_to_cts_report, previous_cts_results=None, publishInExternalReport=False):
        """
        Extract in a list the cts tests results from cts xml file

        return structure will be like this:
        {
        package1:
                  {
                  test1:
                        {
                         "NOT_EXECUTED": ["test7", "test8"],
                         "PASS": ["test4", "test6"],
                         "FAIL": ["test1", "test2"],
                  test2: ...
                  ...
                        }
        package2: ...
        }
        :type  path_to_cts_report: str
        :param path_to_cts_report: path to CTS report where result will be extracted

        :type  previous_cts_results: str
        :param previous_cts_results: path to a PREVIOUS CTS report in order to compare results

        :type  publishInExternalReport: str
        :param publishInExternalReport: do we add result in external report

        :rtype: dict
        :return: full cts result by package

        """
        cts_results = None

        self._logger.debug("CTS - _extract_cts_results_from_cts_report starts...")

        if not isinstance(previous_cts_results, dict):
            previous_cts_results = {}

        cts_report_path = ""
        if HttpDownloaderUtil.is_http_uri(path_to_cts_report):
            # Test is available thru URL => download it localy
            result, full_path = self._download_file(path_to_cts_report)
        else:
            result, full_path = self._get_file_path(path_to_cts_report)

        if result != Global.SUCCESS:
            self._logger.error("Cannot find %s" % path_to_cts_report)
        else:
            self._logger.debug("CTS - _extract_cts_results_from_cts_report - CTS report Download OK")
            cts_report_path = full_path

            try:
                cts_parsed_result = et.parse(cts_report_path)
                self._logger.debug("CTS - _extract_cts_results_from_cts_report - Parsing of the CTS report completed")
            except et.XMLSyntaxError:
                error_msg = "CTS report file " + str(
                    cts_report_path) + "- parsing-reading issue (exception= " + str(Utils.format_exception_info()) + ")"
                raise AcsToolException(AcsToolException.XML_PARSING_ERROR, error_msg)
            xpath_request_pck = "//TestPackage"
            package_list = cts_parsed_result.getroot().xpath(xpath_request_pck)
            cts_results = previous_cts_results
            results_tc = {}
            for package in package_list:
                self._logger.debug("CTS - _extract_cts_results_from_cts_report - Packages results processing")
                package_name = package.get('appPackageName')
                if package_name is not None:
                    xpath_request_tc = ".//TestCase"
                    tcs = package.xpath(xpath_request_tc)
                    cts_results[package_name] = {}
                    for tc_node in tcs:
                        tc_name = tc_node.get('name')
                        if tc_name is not None:
                            default_value = {"PASS": [], "FAIL": [], "NOT_EXECUTED": []}
                            results_tc = cts_results[package_name].get(tc_name, default_value)
                            xpath_request_test = ".//Test"
                            tests = tc_node.xpath(xpath_request_test)
                            for test_node in tests:
                                test_name = test_node.get('name')
                                if test_name is not None:
                                    test_result = test_node.get("result")
                                    if test_result is not None:
                                        test_result = test_result.lower()
                                    if test_result == "fail":
                                        if publishInExternalReport:
                                            self.__tc_report.add_result(test_name, self.__tc_report.verdict.FAIL,
                                                                        "cts test is FAIL", self.get_name(), self.tc_order)
                                        results_tc["FAIL"].append(test_name)
                                    elif test_result == "pass":
                                        if publishInExternalReport:
                                            self.__tc_report.add_result(test_name, self.__tc_report.verdict.PASS,
                                                                        "cts test is PASS", self.get_name(), self.tc_order)
                                        results_tc["PASS"].append(test_name)
                                    else:
                                        if publishInExternalReport:
                                            self.__tc_report.add_result(test_name, self.__tc_report.verdict.BLOCKED,
                                                                        "cts test has not been executed", self.get_name(
                                                                        ),
                                                                        self.tc_order)
                                        results_tc["NOT_EXECUTED"].append(test_name)
                            cts_results[package_name][tc_name] = results_tc
        return cts_results

    def _compute_cts_results(self, cts_results):
        """
        Compute ACS result based on CTS result

        :type  cts_results: dict
        :param cts_results: full cts result as dict.
                            look at _extract_cts_results_from_cts_report comment, for structure description

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        output = []
        old_cts_result = None
        verdict = Global.SUCCESS

        if not cts_results:
            verdict, output = Global.FAILURE, "No results to compute"
        else:
            if self._cts_result_comparison:
                old_cts_result = self._extract_cts_results_from_cts_report(self._cts_result_comparison)

            if old_cts_result is None:
                old_cts_result = {}

            for package, tcs in cts_results.items():
                tmp_output = []
                nb_test_pass = 0
                nb_test_fail = 0
                nb_test_not_exec = 0
                package_status = "PASS"
                pck_old_container = {}
                if package in old_cts_result:
                    pck_old_container = old_cts_result[package]

                for tc, test_status in tcs.items():
                    nb_test_pass += len(test_status["PASS"])
                    nb_test_fail += len(test_status["FAIL"])
                    nb_test_not_exec += len(test_status["NOT_EXECUTED"])
                    tc_old_container = {}

                    if tc in pck_old_container:
                        tc_old_container = pck_old_container[tc]

                    test_verdict = "pass"

                    for test in test_status["FAIL"]:
                        test_verdict = "fail"
                        # Comparison is needed
                        # Report all failed tests
                        if ("PASS" in tc_old_container) & ("FAIL" in tc_old_container):
                            if test in tc_old_container["PASS"]:
                                test_verdict += "(regression)"
                            elif test in tc_old_container["FAIL"]:
                                test_verdict += "(same status)"
                            else:
                                test_verdict += "(no status)"
                        tmp_output.append("%s - %s %s" % (test_verdict, tc, test))

                    for test in test_status["PASS"]:
                        # Comparison is needed
                        # Only report pass test which fails in the previous run
                        if ("PASS" in tc_old_container) & ("FAIL" in tc_old_container):
                            if test in tc_old_container["FAIL"]:
                                test_verdict = "pass(improvment)"
                                tmp_output.append("%s - %s %s" % (test_verdict, tc, test))
                nb_test_fail += nb_test_not_exec
                if nb_test_fail:
                    verdict = Global.FAILURE
                    package_status = "FAIL"
                sum_up = "%s : %s : %s pass %s fail" % (package_status, package, nb_test_pass, nb_test_fail)
                if nb_test_not_exec:
                    sum_up += "(includes %s not executed tests)" % nb_test_not_exec
                output.append(sum_up)
                output = output + tmp_output

        return verdict, output

    def continue_session(self, cts_r_file):
        '''
        This function is used to compute the "continue session command"
        '''
        command = self.test_path.split('run')[0] + " run cts --continue-session 0"
        command += " run cts --continue-session 0 --serial "
        command += self.device_serial
        return command

    def set_up(self):
        """
        Initialize the test

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        verdict = Global.SUCCESS
        msg = ""

        UseCaseBase.set_up(self)

        return verdict, msg

    def run_cts(self):
        '''
        This function will deal with running CTS
        '''

        current_time = time.time()
        test_cmd_line = "%s %s" % (self._cts_exec_path, self._test_cmd_lines)
        result, output = Utils.internal_shell_exec(test_cmd_line, self._test_timeout)
        if result != Global.SUCCESS:
            return Global.FAILURE
        for root, _, filenames in os.walk(self._cts_path):
            for file_ in filenames:
                if CTS_RESULT_FILENAME in file_:
                    cts_results = os.path.join(root, file_)
                    left_time = time.time() - current_time
                    self._test_timeout = self._test_timeout - left_time
                    xml_file = open(cts_results, 'r')
                    xml_cont = xml_file.read()
                    xml_file.close()
                    if 'result="notExecuted"' in xml_cont:
                        if self._test_timeout > 0:
                            self._test_cmd_lines = CTS_CONTINUE_SESSION_COMMAND
                            self._logger.debug("CTS - Not Executed retry")
                            self.run_cts()
                    elif 'result="fail"' in xml_cont:
                        if self._test_timeout > 600 and self._test_failed_retry_no > 0 :
                            self._test_failed_retry_no -= 1
                            x_res = os.path.join(os.path.dirname(cts_results), 'backup_cts_res_' + str(self._test_failed_retry_no))
                            shutil.copyfile(cts_results, x_res)
                            xml_cont = xml_cont.replace('result="fail"', \
                                                        'result="notExecuted"')
                            xml_file = open(cts_results, 'w')
                            xml_file.write(xml_cont)
                            xml_file.close()
                            #self.motc_library.reboot_device()
                            self._test_cmd_lines = CTS_CONTINUE_SESSION_COMMAND
                            self._logger.debug("CTS - Failed retry:%s"%self._test_failed_retry_no)
                            self.run_cts()

    def run_test(self):
        """
        Execute the CTS test
        Compute result based on CTS xml result output

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        UseCaseBase.run_test(self)
        cts_results = {}
        result = Global.SUCCESS
        time_remaining = 0

        cts_exec_found = False
        for root, _, file_names in os.walk(self._cts_path):
            for file_ in file_names:
                if file_ == CTS_EXEC_FILENAME:
                    self._cts_exec_path = os.path.join(root, file_)
                    os.chmod(self._cts_exec_path, stat.S_IRWXU)
                    cts_exec_found = True

            if not cts_exec_found:
                verdict, msg = Global.FAILURE, "Cannot find the CTS executable binary"
            else:
                # clean up old previous results
                for root, _, file_names in os.walk(self._cts_path):
                    for file_ in file_names:
                        if CTS_RESULT_FILENAME in file_:
                            shutil.rmtree(root)
        self.run_cts()
        self._logger.debug("CTS - run_test completed")
        for root, _, filenames in os.walk(self._cts_path):
            for file_ in filenames:
                if CTS_RESULT_FILENAME in file_:
                    cts_results = self._extract_cts_results_from_cts_report(os.path.join(root, file_),
                                                                            cts_results,
                                                                            publishInExternalReport=True)
                    acs_cts_results = os.path.join(self._device.get_report_tree().get_report_path(), "cts_results")
                    if not os.path.exists(acs_cts_results):
                        os.makedirs(acs_cts_results)
                    self._logger.info("Move CTS results into ACS result dir")
                    shutil.move(root, acs_cts_results)
        result, output = self._compute_cts_results(cts_results)

        return result, output

    def tear_down(self):
        """
        End and dispose the test

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        UseCaseBase.tear_down(self)

        return Global.SUCCESS, "No errors"
