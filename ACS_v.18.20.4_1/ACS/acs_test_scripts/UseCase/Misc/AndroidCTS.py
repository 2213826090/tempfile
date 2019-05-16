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
:summary: Enable running CTS in ACS
:since: 08/09/2012
:author: kturban
"""

import __builtin__
import os
import stat
import zipfile
import shutil
import traceback
from tempfile import gettempdir

import lxml.etree as et

from acs_test_scripts.UseCase.Networking.LIVE_WIFI_BASE import LiveWifiBase
from Core.Report.SecondaryTestReport import SecondaryTestReport
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global, format_exception_info, internal_shell_exec
from acs_test_scripts.Utilities.HttpDownloaderUtil import HttpDownloaderUtil

SysErrors = __builtin__.OSError, __builtin__.IOError,

CTS_RESULT_FILENAME = "testResult.xml"
CTS_EXEC_FILENAME = "cts-tradefed"


class AndroidCTS(LiveWifiBase):

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
        LiveWifiBase.__init__(self, tc_name, global_config)
        # Get TC Parameters
        self._cts_path = self._tc_parameters.get_param_value("CTS_PATH")
        self._cts_media_path = self._tc_parameters.get_param_value("CTS_MEDIA_PATH")
        self._cts_result_comparison = self._tc_parameters.get_param_value("CTS_REPORT_COMPARISON_PATH")
        self._test_cmd_lines = self._tc_parameters.get_param_value("TEST_CMD_LINES")
        self._test_timeout = self._tc_parameters.get_param_value("TEST_TIMEOUT", default_cast_type=float)

        self._ui_api = self._device.get_uecmd("Ui")
        self._ui_api.set_global_config(global_config)

        self._misc_api = self._device.get_uecmd("PhoneSystem")
        self._system_api = self._device.get_uecmd("System")
        self._app_api = self._device.get_uecmd("AppMgmt")
        # Add a secondary report
        self.__tc_report = SecondaryTestReport(self._device.get_report_tree().get_report_path())

        self._cts_exec_path = ""

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

    def _set_apk_as_admin(self):
        """
        Set CTS device application as admin

        :rtype: tuple
        :return: ACS verdict and msg output.
        """
        verdict = Global.FAILURE
        error_msg = ""
        self._ui_api.run_operation_set("go_home")
        verdict, msg = self._ui_api.run_operation_set("set_2_apks_as_admin")
        if verdict == Global.SUCCESS:
            self._ui_api.run_operation_set("go_home")
            error_msg = "Wizard deactivated !"
        else:
            error_msg = msg
        return verdict, error_msg

    def _disable_wizard(self):
        """
        Disable device wizard on the device (device wizard activated on user/userdebug builds)

        :rtype: tuple
        :return: ACS verdict and msg output.
        """
        verdict = Global.FAILURE
        msg = ""

        verdict, msg = self._ui_api.run_operation_set("phone_unlock")
        if verdict == Global.SUCCESS:
            self._ui_api.run_operation_set("go_home")
            verdict, msg = self._ui_api.run_operation_set("unlock_wizard")
            if verdict == Global.SUCCESS:
                self._ui_api.run_operation_set("go_home")
                verdict, msg = Global.SUCCESS, "Wizard deactivated !"
        return verdict, msg

    def _disable_lock_screen(self):
        """
        Deactivate the device lockscreen
        Set the device lock screen to None

        :rtype: tuple
        :return: ACS verdict and msg output.
        """
        verdict = Global.FAILURE
        msg = ""

        verdict, msg = self._ui_api.run_operation_set("phone_unlock")
        if verdict == Global.SUCCESS:
            self._ui_api.run_operation_set("go_home")
            verdict, msg = self._ui_api.run_operation_set("deactivate_lockscreen")
            if verdict == Global.SUCCESS:
                self._ui_api.run_operation_set("go_home")
                verdict, msg = Global.SUCCESS, "Lock screen removed !"
        return verdict, msg

    def _install_cts_apks(self):
        """
        Push APKs needed by CTS tests
            - retrieve apk(s) path located on the CTS fwk
            - install them on the device

        :rtype: tuple
        :return: ACS verdict and msg output.
        """
        # depends on android version
        # before 4.2, both are necessaries and provided in CTS package
        # after 4.2, only DelegatingAccessibilityService is necessary

        apk_to_install = ["CtsDelegatingAccessibilityService.apk",
                          "CtsDeviceAdmin.apk"]
        error_msg = ""
        verdict = Global.SUCCESS

        for apk in apk_to_install:
            found = False
            for root, _, file_names in os.walk(self._cts_path):
                for file_ in file_names:
                    if apk in file_:
                        found = True
                        apk_full_path = os.path.join(root, file_)
                        (verdict, error_msg) = self._app_api.install_device_app(apk_full_path)
                        if verdict != Global.SUCCESS:
                            break
                if verdict != Global.SUCCESS:
                    break
            if verdict != Global.SUCCESS:
                break

        if not found:
            verdict = Global.FAILURE
            error_msg = "None of these necessaries APKs has been found: %s" % (" , ".join(apk_to_install))

        return verdict, error_msg

    def _push_cts_medias(self):
        """
        Push videos needed by CTS tests
            - retrieve cts media files if specified by the user
            - push them on the phone

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        verdict = Global.SUCCESS
        error_msg = ""
        if not self._cts_media_path:
            self._logger.warning("No media specified, some CTS packages require such files and may be failed")
            verdict = Global.SUCCESS
            error_msg = "Nothing to do, no media specified"
        else:
            self._logger.info("Retrieve cts media from %s" % self._cts_media_path)
            if HttpDownloaderUtil.is_http_uri(self._cts_media_path):
                # Test is available thru URL => download it localy
                result, full_path = self._download_file(self._cts_media_path)
            else:
                result, full_path = self._get_file_path(self._cts_media_path)

            if result != Global.SUCCESS:
                verdict, error_msg = result, "Cannot find %s" % self._cts_media_path
            else:
                self._cts_media_path = full_path

            if verdict == Global.SUCCESS:
                # Set cts directory path
                # can be a path to a dir or a zip file
                # in both case, resulting filename will be a directory
                fileName, fileExtension = os.path.splitext(self._cts_media_path)
                if fileExtension.lower() == ".zip":
                    zip_file = zipfile.ZipFile(self._cts_media_path, "r")
                    zip_file.extractall(fileName)
                    zip_file.close()
                # cts path should be a dir.
                if os.path.isdir(fileName):
                    self._cts_media_path = fileName
                    # Get path to multimedia files
                    # CTS expects video files in /sdcard/test dir
                    # mmPath = self._device.multimedia_path
                    mmPath = "/sdcard/test"
                    (error_code, _error_msg) = self._device.run_cmd("adb shell mkdir %s" % mmPath, 30)
                    push_cmd = "adb push \"%s\" \"%s\"" % (self._cts_media_path, mmPath)
                    (error_code, _error_msg) = self._device.run_cmd(push_cmd, self._test_timeout)
                    if not error_code == Global.SUCCESS:
                        verdict, error_msg = Global.FAILURE, "Cannot push %s on %s" % (self._cts_media_path, mmPath)
                    else:
                        verdict, error_msg = Global.SUCCESS, "CTS Medias pushed properly"
                else:
                    verdict, error_msg = Global.FAILURE, "Cannot find CTS media"

        return verdict, error_msg

    def _setup_device_for_cts(self):
        """
        Do all setup actions required to run CTS properly.
        Official Setup Instructions are available here:
        http://static.googleusercontent.com/media/source.android.com/en//compatibility/android-cts-manual.pdf

        Optional (depends on the cts package to run):
            medias: if specified by user, push all media on the phone

        :rtype: tuple
        :return: ACS verdict and msg output
        """

        verdict = Global.SUCCESS
        msg = "Set up the board for CTS : OK"
        # No previous setup done on the board
        cts_setup_status = self._device.get_property_value('CTS_SETUP_STATUS')
        if cts_setup_status is None or cts_setup_status in ("", "0"):
            # disable user wizard
            verdict, msg = self._disable_wizard()
            if verdict == Global.SUCCESS:
                self._logger.info(msg)
                # remove lock screen
                verdict, msg = self._disable_lock_screen()
                if verdict == Global.SUCCESS:
                    self._logger.info(msg)
                    # remove android security verification about application installation
                    self._misc_api.set_verify_application(False)

                    # install CTS necessaries APKs to run the tests
                    self._install_cts_apks()
                    if verdict == Global.SUCCESS:
                        self._set_apk_as_admin()
                        # Activate mock location
                        self._misc_api.set_mock_location(True)
                        # Stay on if plugged
                        self._misc_api.set_stay_on_while_plugged_in(3)
                        # Remove screen off timeout
                        self._misc_api.set_screen_timeout(3600)
                        # Activate accessibility
                        self._misc_api.set_accessibility_services("DELEGATE_SERVICE")
                        # Change country to US and language to english
                        self._misc_api.change_language_country_informations("en", "US")
                        # push videos files
                        verdict, msg = self._push_cts_medias()
                        if verdict == Global.SUCCESS:
                            self._logger.info("All settings necessaries to CTS have been applied")
                            self._device.run_cmd("adb shell setprop CTS_SETUP_STATUS 1", 1)
        else:
            self._logger.info("A previous CTS setup has already been done")
        return verdict, msg

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
                    cts_report_path) + "- parsing-reading issue (exception= " + str(format_exception_info()) + ")"
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

    def set_up(self):
        """
        Initialize the test

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        verdict = Global.SUCCESS
        msg = ""

        LiveWifiBase.set_up(self)

        if self._test_timeout is None:
            return (Global.FAILURE, "You need to specify a "
                                    "TESTS_TIMEOUT value.")

        if self._cts_path is None:
            return (Global.FAILURE, "You need to specify a "
                                    "CTS_PATH value.")

        if self._test_cmd_lines is None:
            return (Global.FAILURE, "You need to specify a "
                                    "TEST_PACKAGES_NAMES value.")

        if self._cts_media_path is None:
            self._logger.info("No media specified for CTS tests")

        if self._cts_result_comparison is None:
            self._logger.info("No comparison will be done with previous CTS run")

        full_path = ""
        if HttpDownloaderUtil.is_http_uri(self._cts_path):
            # Test is available thru URL => download it localy
            result, full_path = self._download_file(self._cts_path)
        else:
            result, full_path = self._get_file_path(self._cts_path)

        if result != Global.SUCCESS:
            verdict = result
            msg = "Cannot get the cts version from %s" % self._cts_path
        else:
            self._cts_path = full_path

            # Set cts directory path
            # can be a path to a dir or a zip file
            # in both case, resulting filename will be a directory
            cts_exec_found = False
            fileName, fileExtension = os.path.splitext(self._cts_path)
            if fileExtension.lower() == ".zip":
                zip_file = zipfile.ZipFile(self._cts_path, "r")
                zip_file.extractall(fileName)
                zip_file.close()
            else:
                fileName = self._cts_path

            # cts path should be a dir
            if os.path.isdir(fileName):
                self._cts_path = fileName
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

                # Initialize UI api
                self._ui_api.init()

                # setup the board if it has not been done previously
                verdict, msg = self._setup_device_for_cts()

        return verdict, msg

    def run_test(self):
        """
        Execute the CTS test
        Compute result based on CTS xml result output

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        LiveWifiBase.run_test(self)
        cts_results = {}
        result = Global.SUCCESS

        for test_cmd_line in self._test_cmd_lines.split(";"):
            test_cmd_line = "%s %s" % (self._cts_exec_path, test_cmd_line.strip())
            result, output = internal_shell_exec(test_cmd_line, self._test_timeout)  # pylint: disable=W0212
            if result != Global.SUCCESS:
                return Global.FAILURE, output
            else:
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
        LiveWifiBase.tear_down(self)

        # Release UI api
        if self._ui_api is not None and self._ui_api.isInitialized:
            self._ui_api.release()

        return Global.SUCCESS, "No errors"
