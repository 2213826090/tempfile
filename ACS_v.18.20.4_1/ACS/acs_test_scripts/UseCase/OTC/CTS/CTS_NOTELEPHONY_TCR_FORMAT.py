"""

:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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
:since: 2/12/15
:author: mceausu
"""

import __builtin__
import os
import stat
import shutil
import time
import lxml.etree as et
import copy

import UtilitiesFWK.Utilities as Utils
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsToolException import AcsToolException
from UtilitiesFWK.Utilities import Global
from UtilitiesFWK.Utilities import Verdict
from Core.PathManager import Folders
from Core.Report.Live.LiveReporting import LiveReporting

SysErrors = __builtin__.OSError, __builtin__.IOError,

CTS_RESULT_FILENAME = "testResult.xml"
CTS_RESULT_XSL = "cts_result.xsl"
CTS_RESULT_CSS = "cts_result.css"
CTS_RESULT_HTML_FILENAME = "CTS_RESULTS.html"
CTS_RESULT_FOLDER = "results"
CTS_PLANS_FOLDER = "plans"
CTS_EXEC_FILENAME = "cts-tradefed"
CTS_CONTINUE_SESSION_COMMAND = " run cts --continue-session 0"

class CTS(UseCaseBase):

    """
    Execute Android CTS tests on a device
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

        self._cts_path = os.path.join(Folders.ACS_CACHE, 'Artifacts',
                                      self._tc_parameters.get_param_value("CTS_LOCAL_PATH"))
        if not os.path.exists(self._cts_path):
            return Global.FAILURE, "CTS path from CTS_LOCAL_PATH test case parameter is not valid: %s"%self._cts_path
        self._test_cmd_lines = self._tc_parameters.get_param_value("TEST_CMD_LINES")
        self._test_timeout = self._tc_parameters.get_param_value("TEST_TIMEOUT", default_cast_type=float)
        self._test_failed_retry_no = self._tc_parameters.get_param_value("TEST_FAILED_RETRY_NO",
                                                                         default_value=0, default_cast_type=int)
        self._cts_exec_path = ""
        self._cts_custom_plan_path = self._tc_parameters.get_param_value("CTS_CUSTOM_PLAN_LOCATION")

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

        return verdict, msg

    def get_cts_custom_plan(self):
        """
        This function will check if we have a CTS custom path and move it to
        the right location inside CTS default plans path so that CTS will be able
        to run it
        :return:
        """

        if self._cts_custom_plan_path:
            self._logger.info("CTS custom plan path: %s" % self._cts_custom_plan_path)
            if os.path.isfile(self._cts_custom_plan_path):
                for root, folders, file_names in os.walk(self._cts_path):
                    for elem in folders:
                        if CTS_PLANS_FOLDER in elem:
                            p_folder = os.path.join(root, elem)
                            self._logger.debug("CTS default Plan path: %s"%p_folder)
                            shutil.move(self._cts_custom_plan_path, p_folder)
                            return

    def run_cts(self):
        """
        This function will deal with running CTS
        """
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
                        i = 0
                        if self._test_timeout > 600 and self._test_failed_retry_no > 0:
                            i += 1
                            self._logger.debug("CTS - Failed Tests Retry: %s/%s" % (i, self._test_failed_retry_no))
                            self._test_failed_retry_no -= 1
                            x_res = os.path.join(os.path.dirname(cts_results),
                                                 'backup_cts_res_' + str(self._test_failed_retry_no))
                            shutil.copyfile(cts_results, x_res)
                            xml_cont = xml_cont.replace('result="fail"', 'result="notExecuted"')
                            xml_file = open(cts_results, 'w')
                            xml_file.write(xml_cont)
                            xml_file.close()
                            #self.motc_library.reboot_device()
                            self._test_cmd_lines = CTS_CONTINUE_SESSION_COMMAND
                            self.run_cts()

    def _extract_cts_results_summary(self, cts_result_file_path):
        """
        Extract summary report data from CTS XML report:
        <Summary failed="0" notExecuted="0" timeout="0" pass="11" />
        :return:
        """
        verdict = Global.SUCCESS
        msg = ""
        try:
            tree = et.parse(cts_result_file_path)
            self._logger.debug("CTS - Parsing of the CTS report XML file completed")
        except et.XMLSyntaxError:
            error_msg = "CTS report file: " + str(cts_result_file_path) + \
                        "- parsing-reading issue (exception= " + str(Utils.format_exception_info()) + ")"
            raise AcsToolException(AcsToolException.XML_PARSING_ERROR, error_msg)
        summary = tree.find('Summary')
        if summary is not None:
            failed_t = int(summary.get("failed"))
            not_executed_t = int(summary.get("notExecuted"))
            timeout_t = int(summary.get("timeout"))
            passed_t = int(summary.get("pass"))
            msg = "CTS results Summary: failed=%s notExecuted=%s timeout=%s pass=%s"\
                  %(failed_t, not_executed_t, timeout_t, passed_t)
            if (failed_t + not_executed_t + timeout_t) > 0:
                verdict = Global.FAILURE
        else:
            verdict = Global.FAILURE
            msg = "Summary section was not found in CTS result XML file: %s" %cts_result_file_path
        return verdict, msg

    def _extract_cts_results_for_bulk_tcr_upload(self, cts_result_file_path):
        """
        This function will extract CTS results from default XML into acceptable
        TCR for bulk results upload
        :return:
         payload - results structure to be uploaded into TCR as bulk update
        """
        verdict = Global.SUCCESS
        msg = ""
        payload = []
        rt = {}
        try:
            tree = et.parse(cts_result_file_path)
            self._logger.debug("CTS - Parsing of the CTS report XML file completed")
        except et.XMLSyntaxError:
            error_msg = "CTS report file: " + str(cts_result_file_path) + \
                        "- parsing-reading issue (exception= " + str(Utils.format_exception_info()) + ")"
            raise AcsToolException(AcsToolException.XML_PARSING_ERROR, error_msg)
        package_list = tree.getroot().xpath("//TestPackage")
        for package in package_list:
            self._logger.debug("CTS - Packages results processing")
            package_name = package.get('appPackageName')
            if package_name is not None:
                rt['useCase'] = package_name
            test_case = ""
            element = package.find("TestSuite")
            while element is not None:
                test_case += '.' + element.get('name')
                element = element.find("TestSuite")
            tcs = package.xpath(".//TestCase")
            for tc_node in tcs:
                tc_name = tc_node.get('name')
                if tc_name is not None:
                    tests = tc_node.xpath(".//Test")
                    for test_node in tests:
                        test_name = test_node.get('name')
                        if test_name is not None:
                            test_name = '.' + tc_name + '.' + test_name
                            result = test_node.get('result')
                            if result == "pass":
                                result = Verdict.PASS
                            elif result == "fail":
                                result = Verdict.FAIL
                            elif result == "timeout":
                                result = Verdict.INTERRUPTED
                            elif result == "notExecuted":
                                result = "NA"
                            rt['verdict'] = result
                            rt['testCase'] = package_name + test_case + test_name
                            payload.append(copy.deepcopy(rt))
        nr_elem = len(payload)
        if nr_elem == 0:
            msg = "Calculated CTS bulk upload payload is empty. Was this an empty CTS test plan execution?"
            verdict = Global.FAILURE
        else:
            msg = "calculated CTS bulk upload payload contains %s elements"% nr_elem
            LiveReporting.instance().create_bulk_tc(payload)
        return verdict, msg

    def search_for_zip(self, path):
        """
        :return:
        """
        for root, _, filenames in os.walk(path):
            for file_ in filenames:
                file_path = os.path.join(root, file_)
                if file_path.endswith(".zip") and os.path.isfile(file_path):
                    return file_path
        return None

    def convert_xmls_html(self, cts_result_folder):
        """
        This function will apply xsl to XML and then convert the result to HTML
        :return:
        """
        html_filename = os.path.join(cts_result_folder, CTS_RESULT_HTML_FILENAME)
        xml_filename = os.path.join(cts_result_folder, CTS_RESULT_FILENAME)
        xsl_filename = os.path.join(cts_result_folder, CTS_RESULT_XSL)
        css_filename = os.path.join(cts_result_folder, CTS_RESULT_CSS)
        dom = et.parse(xml_filename)
        xslt = et.parse(xsl_filename)
        transform = et.XSLT(xslt)
        newdom = transform(dom)
        outfile = open(html_filename, 'w')
        outfile.write(et.tostring(newdom, pretty_print=True))
        outfile.close()
        cts_file_c = open(html_filename, 'r')
        css_file_c = open(css_filename, 'r')
        cts_file_s = cts_file_c.read()
        css_file_s = css_file_c.read()
        cts_file_s = cts_file_s.replace('@import "cts_result.css";', css_file_s)
        css_file_c.close()
        cts_file_c.close()
        cts_file_c = open(html_filename, 'w')
        cts_file_c.write(cts_file_s)
        cts_file_c.close()
        return html_filename

    def run_test(self):
        """
        Execute the CTS test
        Compute result based on CTS xml result output

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        UseCaseBase.run_test(self)
        cts_report_file = None
        result = Global.SUCCESS
        output0 = ""
        output1 = ""
        output = ""

        cts_exec_found = False
        for root, _, file_names in os.walk(self._cts_path):
            for fileu in file_names:
                if fileu == CTS_EXEC_FILENAME:
                    self._cts_exec_path = os.path.join(root, fileu)
                    os.chmod(self._cts_exec_path, stat.S_IRWXU)
                    cts_exec_found = True
                    break
            if cts_exec_found:
                break
        if not cts_exec_found:
            return Global.FAILURE, "Cannot find the CTS executable binary"
        else:
            # clean up old previous results
            for root, folders, file_names in os.walk(self._cts_path):
                for elem in folders:
                    if CTS_RESULT_FOLDER in elem:
                        r_folder = os.path.join(root, elem)
                        self._logger.debug("Clean old CTS results from: %s"% r_folder)
                        shutil.rmtree(r_folder)
                        break

        # Move customized CTS plans to default CTS plan location, if any
        self.get_cts_custom_plan()
        # run CTS
        self.run_cts()
        self._logger.debug("CTS - run_test completed: %s" % self._cts_path)
        for root, _, filenames in os.walk(self._cts_path):
            for file_ in filenames:
                if CTS_RESULT_FILENAME in file_:
                    cts_report_file = os.path.join(root, file_)
                    break
            if cts_report_file:
                break
        if not cts_report_file or not os.path.isfile(cts_report_file):
            result = Global.FAILURE
            output0 = "CTS report file not found in CTS report folder"
        else:
            self._logger.debug("CTS report file found: %s" % cts_report_file)
            #upload CTS raw results into TCR test case
            cts_root = (os.path.sep).join(root.rstrip(os.path.sep).split(os.path.sep)[:-1])
            cts_html_result = self.convert_xmls_html(root)
            # tcr_attachement = self.search_for_zip(cts_root)
            # if tcr_attachement:
            #     self._logger.debug("TCR CTS zip file to be uploaded to test case results: %s" %tcr_attachement)
            #     LiveReporting.instance().send_test_case_resource(tcr_attachement, display_name="")

            if cts_html_result and os.path.isfile(cts_html_result):
                self._logger.debug("TCR CTS HTML file to be uploaded to test case results: %s" %cts_html_result)
                LiveReporting.instance().send_test_case_resource(cts_html_result, display_name="")
            # extract CTS results summary
            result, output1 = self._extract_cts_results_summary(cts_report_file)
            result, output = self._extract_cts_results_for_bulk_tcr_upload(cts_report_file)
            self._logger.info("Move CTS results into ACS result dir")
            acs_cts_results = os.path.join(self._device.get_report_tree().get_report_path(), "cts_raw_results")
            if not os.path.exists(acs_cts_results):
                os.makedirs(acs_cts_results)
            shutil.move(cts_root, acs_cts_results)
        return result, output0 + output1 + "\n" + output

    def tear_down(self):
        """
        End and dispose the test

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        UseCaseBase.tear_down(self)

        return Global.SUCCESS, "No errors"
