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

:organization: OTC ANDROID
:summary: Run CTS test plan in ACS
:since: 09/12/15
:author: mfagerst
"""

import __builtin__
import os
import stat
import shutil
import time
import zipfile
import traceback
import subprocess
from tempfile import gettempdir

import lxml.etree as et

import UtilitiesFWK.Utilities as Utils
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from Core.Report.SecondaryTestReport import SecondaryTestReport
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global
from Core.PathManager import Folders
from acs_test_scripts.Utilities.HttpDownloaderUtil import HttpDownloaderUtil
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager
from Core.Report.Live.LiveReporting import LiveReporting

SysErrors = __builtin__.OSError, __builtin__.IOError,

CTS_RESULT_FILENAME = "testResult.xml"
CTS_EXEC_DIR = "android-cts/tools"
CTS_EXEC_FILENAME = "cts-tradefed"
CTS_EXEC_TEST_PLAN = "run cts --disable-reboot --plan"
CTS_TEST_PLANS_DIR = "android-cts/repository/plans"
CTS_RESULT_DIR = "android-cts/repository/results"
CTS_RETEST_PLAN_NAME = "my_failed_tests"
CTS_EXEC_DERIVED_PLAN_1 = "add derivedplan --plan"
CTS_EXEC_DERIVED_PLAN_2 = "--session 0 --result fail"
ACS_CTS_RESULTS_DIR = "cts_results"


class CTSTestPlan(UseCaseBase):

    """
    Execute CTS test plan on a device.

    First of all, have a look to:
    http://source.android.com/compatibility/cts-intro.html

    CTS test plan is divided by package (full plan ~ 80 packages).
    CTS release AND plan evolve for each android version.

    To run the specified test plan properly, we need:
    - cts framework zip file
    - cts test plan
    - required prerequisits executed in preceeding test steps
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

        self._cts_version = self._tc_parameters.get_param_value("CTS_VERSION")
        self._test_plan = self._tc_parameters.get_param_value("TEST_PLAN")
        self._artifactory_uri = self._tc_parameters.get_param_value("ARTIFACTORY_URI")
        self._test_failed_retry_no = self._tc_parameters.get_param_value("TEST_FAILED_RETRY_NO", default_value=0, default_cast_type=int)
        self._test_timeout = self._tc_parameters.get_param_value("TEST_TIMEOUT", default_cast_type=float)
        self._cts_result_comparison = self._tc_parameters.get_param_value("CTS_REPORT_COMPARISON_PATH")
        self._test_cmd_lines = self._tc_parameters.get_param_value("TEST_CMD_LINES")
        # Add a secondary report
        self.__tc_report = SecondaryTestReport(self._device.get_report_tree().get_report_path())
        self.devnull = file(os.devnull, "w")
        return result, output

    def _download_file(self, url):
        """
        Download a file from http server

        :type  url: str
        :param url: url to the file to download

        :rtype: tuple
        :return: ACS verdict and local path to downloaded file
        """
        result = Global.FAILURE
        file_name = os.path.join(gettempdir(),
                                 url.rstrip('/').split('/')[-1])

        http_downloader = HttpDownloaderUtil(url=url,
                                             destination=file_name,
                                             override=True,
                                             download_timeout=self._test_timeout,
                                             logger=self._logger)

        try:
            # check args
            response = http_downloader.init()
            # then, process download
            result, _, file_name = http_downloader.download(response)
        except (SysErrors, AcsConfigException):
            msg = ("Exception occurred while Downloading {0}"
                   "Detailed exception below:\n{1}".format(url, traceback.format_exc()))
            file_name = ""
            result = Global.FAILURE
            self._logger.error(msg)

        return result, file_name

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


    def _derive_retest_plan(self):
        """
        Derive test plan including failed tests from last run
        """
        self._logger.info("Derive test plan")

        # Remove any previously derived test plan
        try:
            derived_test_plan = "%s%s" % (CTS_RETEST_PLAN_NAME, ".xml")
            os.remove(os.path.join(self._cts_path, CTS_TEST_PLANS_DIR, derived_test_plan))
        except OSError:
            pass

        cmd_line = "%s %s %s %s" % (self._cts_exec_path, CTS_EXEC_DERIVED_PLAN_1, CTS_RETEST_PLAN_NAME, CTS_EXEC_DERIVED_PLAN_2)
        Utils.internal_shell_exec(cmd_line, self._test_timeout)


    def _run_cts(self, test_plan):
        """
        Execute a CTS test plan
        Compute result based on CTS xml result output
        Add cts results to test case report

        :type test_plan: str
        :type parameter: Test plan to be executed

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        cts_results = {}
        result = Global.SUCCESS
        result_folder = ""

        # Execute test plan
        test_cmd_line = "%s %s %s" % (self._cts_exec_path, CTS_EXEC_TEST_PLAN, test_plan)
        result, output = Utils.internal_shell_exec(test_cmd_line, self._test_timeout)
        if result != Global.SUCCESS:
            return Global.FAILURE, output
        else:
            self._logger.debug("CTS test plan execution completed")

        for root, _, filenames in os.walk(os.path.join(self._cts_path, CTS_RESULT_DIR)):
            for file_ in filenames:
                if CTS_RESULT_FILENAME in file_:
                    result_folder = root
                    cts_results = self._extract_cts_results_from_cts_report(os.path.join(root, file_),
                                                                            cts_results,
                                                                            publishInExternalReport=True)

        result, output = self._compute_cts_results(cts_results)

        # Derive test plan to be used at retry before moving results folder
        if (result != Global.SUCCESS) and (self._test_run_no < self._test_failed_retry_no):
            self._derive_retest_plan()

        # Add ziped result to report
        live_report = LiveReporting.instance()
        filename = "%s%s" % (result_folder,".zip")
        live_report.send_test_case_resource(filename)
        # Remove cts results folder
        shutil.rmtree(result_folder)

        return result, output


    def set_up(self):
        """
        Initialize the test

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        verdict = Global.SUCCESS
        msg = ""

        UseCaseBase.set_up(self)

        if self._cts_version is None:
            return (Global.FAILURE, "You need to specify a CTS_NAME value.")

        if self._test_plan is None:
            return (Global.FAILURE, "You need to specify a TEST_PLAN value.")

        if self._artifactory_uri is None:
            self._logger.info("Default artifactory path will be used")

        if self._test_timeout is None:
            self._test_timeout = 360000
            self._logger.info("Default test timeout will be used: %s.", self._test_timeout)

        if self._cts_result_comparison is None:
            self._logger.info("No comparison will be done with previous CTS run")

        if self._test_failed_retry_no is None:
            self._logger.info("No test retries will be run at failure.")
            self._test_failed_retry_no = 0

        art_mgr = EquipmentManager().get_artifact_manager("ARTIFACT_MANAGER")

        # Download CTS package
        try:
            self._cts_path = art_mgr.get_artifact(self._cts_version, self._artifactory_uri)
        except Exception as artmgr_exception:
            return (Global.FAILURE, "Failed to download CTS package")

        # Check if zipfile or local cache
        fileName, fileExtension = os.path.splitext(self._cts_path)
        if fileExtension.lower() == ".zip":
            # Unzip CTS package
            self._cts_path = art_mgr.unzip_artifact(self._cts_path)
            self._logger.debug("Unzip with ArtifactoryManager, self._cts_path: %s", self._cts_path)

        # Set CTS execution path and chmod execution files
        self._cts_exec_path = os.path.join(self._cts_path, CTS_EXEC_DIR)
        self._cts_exec_path = os.path.join(self._cts_exec_path, CTS_EXEC_FILENAME)
        os.chmod(self._cts_exec_path, stat.S_IRWXU)

        #turn on stay awake on the device
        cmd = ['adb', 'shell', 'svc', 'power', 'stayon', 'usb']
        subprocess.call(cmd, stdout=self.devnull, stderr=self.devnull)

        # Download Test plan
        try:
            test_plan = "%s%s" % (self._test_plan,".xml")
            test_plan_path = art_mgr.get_artifact(test_plan, self._artifactory_uri)
        except Exception as artmgr_exception:
            return (Global.FAILURE, "Failed to download Test plan")

        # Copy Test plan into CTS
        test_plan_dir = os.path.join(self._cts_path, CTS_TEST_PLANS_DIR)
        shutil.copy(test_plan_path, test_plan_dir)

        # Remove previous result if it exists
        results_path = os.path.join(self._cts_path, CTS_RESULT_DIR)
        if os.path.exists(results_path):
            for item in os.listdir(results_path):
                item_path = os.path.join(results_path, item)
                if os.path.isfile(item_path):
                    os.unlink(item_path)
                else:
                    shutil.rmtree(item_path)

        return verdict, msg


    def run_test(self):
        """
        Execute the CTS test
        Compute result based on CTS xml result output

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        UseCaseBase.run_test(self)
        result = Global.SUCCESS

        self._test_run_no = 0

        result, output = self._run_cts(self._test_plan)

        while (result != Global.SUCCESS) and (self._test_run_no < self._test_failed_retry_no):
            self._logger.info("Rerun test due to failure")
            result, output =  self._run_cts(CTS_RETEST_PLAN_NAME)
            self._test_run_no+=1

        return result, output


    def tear_down(self):
        """
        End and dispose the test

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        #turn off stay awake on the device
        cmd = ['adb', 'shell', 'svc', 'power', 'stayon', 'false']
        subprocess.call(cmd, stdout=self.devnull, stderr=self.devnull)
        UseCaseBase.tear_down(self)

        return Global.SUCCESS, "No errors"
