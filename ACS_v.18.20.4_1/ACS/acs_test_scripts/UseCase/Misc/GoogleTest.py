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
:summary: Enable running Google test in ACS
:since: 23/08/2013
:author: lbavois
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from Core.Report.SecondaryTestReport import SecondaryTestReport
from UtilitiesFWK.Utilities import Global, Verdict, get_timestamp
import UtilitiesFWK.Utilities as Utils
import os
import lxml.etree as et

# String const used to initialize a default result file path over the device
# sdcard directory should always be present and no need to be a root user to write inside
GTEST_OUTPUT = "/sdcard/gtest_report.xml"
# Timeout in seconds to list google test command exec file over the device
ADB_LS_FILE_TIMEOUT = 2
# Timeout in seconds to remove result file path over the device
ADB_RM_FILE_TIMEOUT = 5
# Timeout in seconds to copy google test result file from the device to ACS host
ADB_PULL_FILE_TIMEOUT = 10


class GoogleTest(UseCaseBase):

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base __init__ method
        UseCaseBase.__init__(self, tc_name, global_config)
        # Enable ACS secondary reports for ACS AWR
        self._secondary_report = SecondaryTestReport(self._device.get_report_tree().get_report_path())

    def set_up(self):
        """
        Initialize the test
        """

        status = Global.FAILURE
        msg = "Undefined error during setup"
        UseCaseBase.set_up(self)

        self._google_test_timeout = self._tc_parameters.get_param_value(
            param="GTEST_TIMEOUT", default_value=0, default_cast_type=int)
        self._google_test_command = self._tc_parameters.get_param_value(
            "GTEST_BINARY", default_value="", default_cast_type=str)

        self._device.get_logger().debug(
            "Google test output files while be generated in %s (.xml and .txt)" % GTEST_OUTPUT)

        if self._google_test_timeout is None or self._google_test_timeout <= 0:
            msg = "You need to specify a GTEST_COMMAND_TIMEOUT value superior to 0 in seconds"
        else:
            if self._google_test_command is None or self._google_test_command == "":
                msg = "You need to specify a valid path for GTEST_COMMAND file (%s) loaded on the device" % self._google_test_command
            else:
                # Check whether google test file exists or not
                result, output = self._device.run_cmd(
                    "adb shell ls %s" % self._google_test_command, ADB_LS_FILE_TIMEOUT)

                if result == Global.SUCCESS:
                    self._device.get_logger().debug(
                        "Google test command file %s exists on the device" % self._google_test_command)
                    # Delete xml output file in case old one exists on the device
                    # No matter if the command fails (in case of none existing file)
                    self._device.run_cmd("adb shell rm %s" % GTEST_OUTPUT, ADB_RM_FILE_TIMEOUT, silent_mode=True)
                    status = Global.SUCCESS
                    msg = ""
                else:
                    msg = "You need to specify a valid path for GTEST_COMMAND file (%s) loaded on the device (error = %s)" % self._google_test_command % output
        return status, msg

    def run_test(self):
        """
        Executes the test.
        """
        UseCaseBase.run_test(self)

        # Format the command
        # The google test case result is stored inside GTEST_OUTPUT file
        cmd = "adb shell %s --gtest_output=\"xml:%s\"" % (self._google_test_command, GTEST_OUTPUT)

        # Run the test
        result, output = self._device.run_cmd(cmd, self._google_test_timeout)

        # In all the cases store in ACS report folder google test command stdout
        # Build a report file path is ACS report folder
        txt_result_file_path = self.__build_report_file_name(self._google_test_command, "txt")

        with open(txt_result_file_path, "w") as text_file:
            text_file.write(output)

        if result == Global.SUCCESS:

            # Check adb command output after running google test
            if "Segmentation fault" in output:
                # Handle Segmentation fault error case
                # No xml report file generated => no copy to do in ACS report folder
                # Inform user that test is FAILED with Segmentation fault
                result = Global.FAILURE
                comment = "Segmentation Fault detected during google test command execution: " + \
                    self._google_test_command + " - see log file " + os.path.basename(txt_result_file_path)
            else:
                # Build a report file path is ACS report folder
                xml_result_file_path = self.__build_report_file_name(self._google_test_command, "xml")

                # Pull the google test result xml file on the ACS report folder
                self._device.pull(GTEST_OUTPUT, xml_result_file_path, ADB_PULL_FILE_TIMEOUT)

                # If we retrieve the google test command xml report file
                if os.path.isfile(xml_result_file_path):

                    # Put all the test case data inside a dictionnary after parsing the google xml report file
                    gtest_report_dict = {}
                    status, msg = self.__google_test_xml_report_as_dict(xml_result_file_path, gtest_report_dict)

                    if status:
                        # Parse result file
                        # Add to secondary reports all sub test cases
                        result, comment = self.__parse_google_test_report_data(gtest_report_dict)
                    else:
                        result = Global.FAILURE
                        comment = msg
                else:
                    result = Global.FAILURE
                    comment = "Cannot retrieve google test xml report file associated to test execution" % self._google_test_command

                if "ERROR:" in output:
                    # Handle possible case where all test case are passed but an error is detected
                    result = Global.FAILURE
                    comment = "ERROR detected during google test command execution: " + \
                        self._google_test_command + " - see log file " + os.path.basename(txt_result_file_path)
        else:
            result = Global.FAILURE
            comment = "ISSUE detected during google test command execution: " + \
                self._google_test_command + " - see log file " + os.path.basename(txt_result_file_path)

        return result, comment

    def __build_report_file_name(self, file_test_path, file_extension):
        """
        This private function generates a unique file name inside the ACS report folder
        The name is built from a timestamp for the current time.
        The function return a file path str
         """
        # Format the test result file name stored in ACS report folder
        # We add the date as we can have same test passed several times in a test
        # campaign, so we need a unique filename after each execution
        file_name = "%s_report_%s.%s" % (os.path.basename(file_test_path), get_timestamp(), file_extension)

        # Add the ACS report folder to the result file name
        result_file_path = os.path.join(self._device.get_report_tree().get_report_path(), file_name)

        return result_file_path

    def __parse_node(self, node):
        """
        This private function parse a node from gtest output xml file test result

        :type node: object
        :param node: node of the xml document parsed by etree.

        :rtype: dict
        :return: Data stocked into a dictionnary.
        """
        dico = {}
        name = node.get('name', "")
        if name:
            # store all keys (except 'name')/value in a dict
            for key in [x for x in node.attrib if x != "name"]:
                dico[key] = node.attrib[key]

        return dico

    def __parse_tc_nodes(self, testsuite_node_list, testsuite_dict):
        """
        This function takes a lost of test case nodes and put parsing result in dictionnary
        it returns the number of test cases

        :type testsuite_node_list: list
        :param testsuite_node_list: list of nodes of the xml document parsed by etree.
        :type testsuite_node_list: dict
        :param testsuite_dict: dictionnary containing output parsing data.

        :rtype: integer, str
        :return: number of tc and error message
        """
        # count the total number of test case nodes
        testcase_count = 0
        message = "Undefined error while parsing with etree"
        for testsuite_node in testsuite_node_list:
            testsuite_name = testsuite_node.get('name', "")
            if testsuite_name:
                testsuite_dict[testsuite_name] = self.__parse_node(testsuite_node)
            try:
                # Extract testcase in testsuite node
                testcase_list = {}
                testcase_node_list = testsuite_node.xpath('./testcase')
                if testcase_node_list:
                    for testcase_node in testcase_node_list:
                        testcase_name = testcase_node.get('name', "")
                        testcase_count += 1
                        if testcase_name:
                            testcase_list[testcase_name] = self.__parse_node(testcase_node)
                            # Extract failure in testcase node
                            failure_node_list = testcase_node.xpath('./failure')
                            failure_message = ""
                            if failure_node_list:
                                for failure_node in failure_node_list:
                                    failure_message += failure_node.get('message', "")
                                    if failure_message:
                                        testcase_list[testcase_name].update({'comment': failure_message})
            except et.XPathError as e:
                message = str(e)
                break
            testsuite_dict[testsuite_name].update({'testcases': testcase_list})
        return testcase_count, message

    def __google_test_xml_report_as_dict(self, xml_gtest_report_file, testsuite_dict):
        """
        This private function parses xml result file generated by google test command
        add put all the data inside a dictionnary
        This function returns a dictionary with all google test result execution data
        and the status of google test report file parsing

        :type xml_gtest_report_file: str
        :param xml_gtest_report_file: name of the local gtest report file

        :type testsuite_dict: dictionnary
        :param testsuite_dict: gtest report data dict

        :rtype: boolean, str
        :return: status and message
        """

        status_message = "Undefined error while parsing gtest xml file %s" % xml_gtest_report_file
        status = False

        try:
            gtest_result_doc = et.parse(xml_gtest_report_file)

        except et.XMLSyntaxError:
            _, error_msg, _ = Utils.get_exception_info()
            status_message = "Cannot parse gtest xml result file for test %s" % self._google_test_command

        try:
            # Parse etree tool result
            testsuites_nodes = gtest_result_doc.xpath('//testsuites')

            if not testsuites_nodes:
                # Inform the user that there is no results inside gtest result file
                status_message = "Error while parsing gtest xml result file for test %s" % self._google_test_command + \
                    " - file => no <testsuites> ...  </testsuites> node found"
            else:
                # Parse the testsuites node, it should be uniq
                if len(testsuites_nodes) == 1:

                    # EXTRACT DATAS FROM GTEST RESULT FILE
                    ##########################################################################
                    # Extract the test suite nodes + test case nodes for every test suite
                    # put all values in testsuite_list dictionnary => contain testsuite name +
                    # testsuite attributes + testcase dictionnary

                    testsuite_node_list = testsuites_nodes[0].xpath('./testsuite')
                    if testsuite_node_list:
                        testcase_count = self.__parse_tc_nodes(testsuite_node_list, testsuite_dict)

                        # Test if a test case has been run
                        if testcase_count:
                            status = True
                        else:
                            # no test case inside the gtest result file report so no test case run
                            status_message = "Error while parsing gtest xml result file for test %s" % self._google_test_command + \
                                " - no <testscase> node found, no test case run in current google test"
                    else:
                        # no test suite inside the gtest result file report so no test case run
                        status_message = "Error while parsing gtest xml result file for test %s" % self._google_test_command + \
                            " - no <testsuite> node, no test case run in current google test"
                else:
                    # We do not handle several <testsuites> in google test report parsing
                    status_message = "Error while parsing gtest xml result file for test %s" % self._google_test_command + \
                        " - file => several <testsuites> ...  </testsuites> node found"

        except Exception:
            _, error_msg, _ = Utils.get_exception_info()
            status_message = "Error while parsing gtest xml result file for test %s - error = %s" % (
                self._google_test_command, error_msg)

            if not testsuite_dict:
                status_message = "Error while parsing gtest xml result file for test %s" % self._google_test_command + \
                    " - no testsuite succesfully parsed "

        return status, status_message

    def __parse_google_test_report_data(self, gtest_report_dict):
        """
        Compute test results from gtest output xml file test result
        add each test result to a secondary report
        Return the secondary report
        """

        test_case_verdict = Global.SUCCESS
        status_message = ""

        # CONVERT DATAS FROM GTEST RESULT dictionnary into test case verdict + secondary result for ACS AWR
        #####################################################################################################

        # Set test verdict as FAILURE if one sub test is BLOCKED or FAILURE
        # Parse the testsuite list present in the google test result dictionnary
        for testsuite_key, testsuite_value in gtest_report_dict.items():
            if "testcases" in testsuite_value:
                for testcase_name, testcase_attr in testsuite_value['testcases'].items():

                    # Compute secondary result test case name adding testsuite name and testcase name
                    secondary_report_testcase_name = testsuite_key + "." + testcase_name

                    # Compute secondary result verdict and comment associated
                    secondary_report_testcase_verdict = Verdict.FAIL
                    secondary_report_testcase_comment = ""

                    if "status" in testcase_attr:
                        status = testcase_attr['status']
                        # Check if run or not
                        if status == "notrun":
                            # If the attribues equals "notrun" consider the test as skipped
                            secondary_report_testcase_verdict = Verdict.BLOCKED
                            secondary_report_testcase_comment = "Test %s has been skipped (notrun)"
                            test_case_verdict = Global.FAILURE
                        elif status == "run":
                            # Mean success
                            secondary_report_testcase_verdict = Verdict.PASS
                            secondary_report_testcase_comment = ""

                            # Unless a comment has been added for specific failure case
                            if "comment" in testcase_attr:
                                failure_message = testcase_attr['comment']
                                if failure_message is not None:
                                    # Mean failure
                                    secondary_report_testcase_verdict = Verdict.FAIL
                                    secondary_report_testcase_comment = failure_message
                                    test_case_verdict = Global.FAILURE
                    else:
                        secondary_report_testcase_verdict = Verdict.FAIL
                        secondary_report_testcase_comment = "Error while parsing gtest xml result file for test %s" % self._google_test_command + \
                            " - no status for test case " + secondary_report_testcase_name
                        test_case_verdict = Global.FAILURE

                    self._secondary_report.add_result(
                        secondary_report_testcase_name, secondary_report_testcase_verdict, secondary_report_testcase_comment, self.get_name(), self.tc_order)

                if test_case_verdict == Global.FAILURE and not status_message:
                    status_message = "At least one test has a FAILURE - BLOCKED status for gtest %s" % self._google_test_command
            else:
                test_case_verdict = Global.FAILURE
                status_message = "No test found inside inside gtest result file for gtest %s" % self._google_test_command

        return test_case_verdict, status_message

    def tear_down(self):
        UseCaseBase.tear_down(self)
        # Delete xml output file
        self._device.run_cmd("adb shell rm %s" % GTEST_OUTPUT, ADB_RM_FILE_TIMEOUT, silent_mode=True)

        return Global.SUCCESS, "No error"
