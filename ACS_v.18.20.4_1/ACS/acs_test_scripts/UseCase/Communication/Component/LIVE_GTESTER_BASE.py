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
:summary: This file implements base methods for I{GTester} Use Cases.
:since: 12/02/2013
:author: asebbanx
"""
# Module name follows ACS conventions
# pylint: disable=C0103

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.GTester.Parser import GCampaignParser
from acs_test_scripts.Utilities.GTester.PhoneWrapper import AndroidDeviceWrapper
from acs_test_scripts.Utilities.GTester.Runner import GTesterRunner
from UtilitiesFWK.Utilities import Global, Verdict
import os
from ErrorHandling.DeviceException import DeviceException
# We locally disable some errors that PyDev may see
# pylint: disable=F0401,E0611
# We restore the previously disabled errors.
# pylint: enable=F0401,E0611


class LiveGTesterBase(UseCaseBase):

    """
    Live GTester base Use Case class.
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Call UseCase base __init__ method
        UseCaseBase.__init__(self, tc_name, global_config)

        # We will need a phone wrapper
        self.__phone_wrapper = AndroidDeviceWrapper.get_instance(self._device)

        # We may need a runner to execute the commands
        self.__runner = None

        # We may need a parser to process text and/or XML
        self.__parser = None

    def set_up(self):
        """
        Initializes this test.
        """
        # Run the inherited 'set_up' method
        UseCaseBase.set_up(self)

        # Some attributes are needed
        self.__runner = GTesterRunner(self._device)
        self.__parser = GCampaignParser()

        # Create the result directory if needed
        # Remount the device's filesystem
        self.phone_wrapper.remount()
        # Create the directory
        self.__phone_wrapper.make_result_directory()

        # Return the verdict
        return Global.SUCCESS, "No error."

    def get_runner(self):
        """
        Returns the C{GTesterRunner} instance used by this
        Use Case.

        :rtype: GTesterRunner
        :return: the C{GTesterRunner} instance
        """
        return self.__runner

    def get_parser(self):
        """
        Returns the C{GCampaignParser} instance used by this
        Use Case.

        This parser is used to parse XML files, commands and
        others text/str elements that may require processing.

        :rtype: GCampaignParser
        :return: the C{GCampaignParser} instance
        """
        return self.__parser

    def get_phone_wrapper(self):
        """
        Returns the C{AndroidDeviceWrapper} instance used by this
        Use Case.

        :rtype: AndroidDeviceWrapper
        :return: the C{AndroidDeviceWrapper} instance
        """
        return self.__phone_wrapper

    def _count_verdict_in_multiple_result(self, verdict, results):
        """
        Count the number of occurrences of the given verdict in
        the given multiple results.

        :param verdict: The verdict to count
        :type verdict: UtilitiesFWK.Utilities.Verdict

        :param results: The results to use
        :type results: dict

        .. seealso:: Report.SecondaryTestReport.SecondaryTestReport

        :return: The count of verdict in results
        :rtype: int
        """
        count = 0
        for el in results:
            if 'status' in results[el] and results[el]['status'] == verdict:
                count += 1
        return count

    def _parse_and_compute_result(self, result_object, test_name, secondary_report=None):
        """
        Parse the given result object and compute verdict.

        :param result_object: The result object to parse
        :type result_object: Utilities.GTester.Parser.GTestFwkResult

        :param test_name: The test name to be used for logging
        :type test_name: str

        :param secondary_report: (optional) The secondary report to use in case
                                of multiple results
        :type secondary_report: Report.SecondaryTestReport.SecondaryTestReport

        :rtype: None
        """

        # List of tests fail
        tests_fail_list = []
        # List of tests blocked
        tests_blocked_list = []
        # List of tests pass
        tests_pass_list = []

        if result_object is None:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                   "There is no result object to compute !")
        if test_name in [None, ""]:
            test_name = "unknown test"

        # Retrieve the remote file path
        remote_file_path = result_object.remote_file_path

        # Compute the local file path
        local_file_path = result_object.local_file_path
        local_file_path = os.path.normpath(local_file_path)

        # Retrieve the result file
        self._logger.debug("Retrieve result file.")
        self.phone_wrapper.pull(remote_file_path, local_file_path)

        # Parse the file to generate list of pass, fail and blocked tests.
        self._logger.debug("Parse the file to extract verdict.")
        (tests_pass_list, tests_fail_list, tests_blocked_list) = result_object.parse_document_for_result()

        # Parse the result file
        self._logger.debug("Parse the file.")
        result_object.parse_gtester()

        # Copy local file to report dir
        self._logger.debug("Copy local file.")
        result_object.copy_local_file(self._device.get_report_tree().get_report_path())

        # Delete the local file
        self._logger.debug("Delete local file.")
        result_object.delete_local_file()

        # Compute the verdict
        self._logger.debug("Compute verdict.")
        verdict = result_object.verdict

        result_object.get_verdict()

        output = str(test_name) + " execution successfull."
        if verdict != Global.SUCCESS:
            output = result_object.log_messages
            # Convert the output to a str if needed
            if isinstance(output, list):
                output = " ".join(map(str, output))  # pylint: disable=W0141

        result = result_object.get_multiple_results()

        # In case of multiple execution
        if result is not None and secondary_report is not None:
            # Add the results to test reports
            secondary_report.add_result_from_dict(result, self.get_name(), self.tc_order)

            # Create str with name of all tests fail
            str_tests_fail = ""
            for i in range(len(tests_fail_list)):
                str_tests_fail += (str(tests_fail_list[i]) + " : FAIL; ")

             # Create str with name of all tests blocked
            str_tests_blocked = ""
            for i in range(len(tests_blocked_list)):
                str_tests_blocked += (str(tests_blocked_list[i]) + " : BLOCKED; ")

            # Adjust verdict according to
            if self._count_verdict_in_multiple_result(Verdict.BLOCKED, result) > 0:
                verdict = Global.BLOCKED

                if self._count_verdict_in_multiple_result(Verdict.FAIL, result) > 0:
                    # generating the output message for pass, fail and blocked tests
                    output = str(test_name) + " execution failed. Global result : " + str(len(tests_pass_list)) + \
                        " tests pass, " + str(len(tests_fail_list)) + " tests fail and " + str(len(tests_blocked_list)) + \
                            " tests blocked. " + str_tests_fail + str_tests_blocked
                else:
                   # generating the output message for pass and blocked test
                   output = str(test_name) + " execution blocked. Global result : " + str(len(tests_pass_list)) + \
                        " tests pass and " + str(len(tests_blocked_list)) + " tests blocked. " + str_tests_blocked

            elif self._count_verdict_in_multiple_result(Verdict.FAIL, result) > 0:
                verdict = Global.FAILURE

                # generating the output message for pass and fail tests
                output = str(test_name) + " execution failed. Global result : " + str(len(tests_pass_list)) + \
                    " tests pass and " + str(len(tests_fail_list)) + " tests fail. " + str_tests_fail

            else:
                verdict = Global.SUCCESS
                output = str(test_name) + " execution successfull."

        return verdict, output

    def _get_test_name(self, command):
        """"
        Parse the given command then extract the test name

        :type command: str
        :param command: The command to be used for extraction

        :rtype: str
        :return: The extracted test name as the test app binary
        """
        splitted_command = command.split(" ")
        test_name = os.path.basename(splitted_command[0])
        return str(test_name)

    # Declare properties that will help us
    # to write other Component tests Use Case more easily.
    runner = property(
        get_runner,
        None,
        None,
        "The runner (GTesterRunner) instance.")
    parser = property(
        get_parser,
        None,
        None,
        "The parser (GCampaignParser) instance.")
    phone_wrapper = property(
        get_phone_wrapper,
        None,
        None,
        "The parser (AndroidDeviceWrapper) instance.")
