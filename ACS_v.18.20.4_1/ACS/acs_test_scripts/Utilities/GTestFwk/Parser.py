"""
:copyright: (c)Copyright 2012, Intel Corporation All Rights Reserved.
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
:summary: Parsing utilities for GTest Framework UC
:author: jreynaux
:since: 07/04/2013
"""

from datetime import datetime
import os.path
from xml.dom import minidom

from UtilitiesFWK.Utilities import Verdict
from acs_test_scripts.Utilities.GTester.Parser import GParserBase, GResult
import shutil
from ErrorHandling.DeviceException import DeviceException


class GTestFwkResult(GParserBase):

    """
    A class to handle operations for C{GTestFwk} results.
    """
    # Note that value is usually set on UC parameters
    GTEST_FWK_RESULT_ROOT = "/logs"
    """
    The root directory where all GTest Framework test results
    will be stored.
    """

    def __init__(
            self,
            file_name=None,
            remote_dir_path=None,
            local_dir_path="."):
        """
        Initializes this instance.
        """
        GParserBase.__init__(self)
        if file_name is None:
            file_name = GTestFwkResult.build_file_name()

        if remote_dir_path is None:
            remote_dir_path = GTestFwkResult.GTEST_FWK_RESULT_ROOT

        self._file_name = file_name
        self._remote_dir_path = remote_dir_path
        self._local_dir_path = local_dir_path
        self._remote_file_path = None
        self._local_file_path = None
        self._verdict = GResult.FAILED
        self._duration = 0.0
        self._messages = []

        self._multiple_results = None
        # Current Testsuite name
        self._current_ts_name = None
        # Current Testcase name
        self._current_tc_name = None

    @classmethod
    def create(cls, file_name=None, remote_dir_path=None):
        """
        Returns a new C{GTesterResult} instance built
        with the given file name if any or with a
        generated file name otherwise.

        :param file_name: The file name to use for Result (optional)
        :type file_name: str

        :param remote_dir_path: The remote directory where the file is (optional)
        :type remote_dir_path: str

        :rtype: GTesterResult
        :return: a new C{GTesterResult} instance
        """
        if file_name is None:
            file_name = GTestFwkResult.build_file_name()
        instance = GTestFwkResult(file_name, remote_dir_path)
        return instance

    @classmethod
    def build_file_name(cls):
        """
        Returns a file name that can be used as output argument
        for I{GTester}.
        The name is built from a timestamp for the current time.
        """
        right_now = datetime.now()
        timestamp = "%04d%02d%02d-%02d%02d%02d%06d" % (
            right_now.year,
            right_now.month,
            right_now.day,
            right_now.hour,
            right_now.minute,
            right_now.second,
            right_now.microsecond)
        file_name = "gtest_fwk_result_%s.xml" % timestamp
        return file_name

    def get_log_messages(self):
        """
        Returns this test log messages.

        :rtype: list
        :return: the log messages as list
        """
        return self._messages

    def get_verdict(self):
        """
        Returns this test result's verdict if any test
        has actually been executed (and C{GResult.FAILED}
        otherwise).
        """
        return self._verdict

    def get_file_name(self):
        """
        Returns this result file name.
        :rtype: str
        :return: the file name
        """
        return self._file_name

    def get_remote_file_path(self):
        """
        Returns this result's remote file path.
        :rtype: str
        :return: the file path
        """
        if self._remote_file_path is None:
            # Remote file path is Linux only
            # so we force the file path separator
            # to /
            self._remote_file_path = "%s/%s" % (
                self._remote_dir_path,
                self.file_name)
        return self._remote_file_path

    def delete_local_file(self):
        """
        Removes the local result file if it exists.
        """
        if self.does_local_file_exist:
            os.unlink(self.local_file_path)

    def copy_local_file(self, destination_dir):
        """
        Copys the local result file if it exists.

        :param destination_dir: The directory where to copy current GTester result file.
        :type destination_dir: str

        :rtype: None
        """
        if self.does_local_file_exist:
            shutil.copy2(self.local_file_path, destination_dir)

    def does_local_file_exist(self):
        """
        Returns a boolean indicating whether the local file
        associated to this test exists or not.
        :rtype: bool
        :return: whether the local file exists or not:
            - C{True} if the local file exists
            - C{False} otherwise
        """
        return os.path.exists(self.local_file_path)

    def get_local_file_path(self):
        """
        Returns this result's local file path.
        :rtype: str
        :return: the file path
        """
        if self._local_file_path is None:
            self._local_file_path = os.path.join(
                self._local_dir_path,
                self.file_name)
        return self._local_file_path

    def _set_remote_file_path(self, remote_file_path):
        """
        Sets this result's file path to the given value.
        :param remote_file_path: the path to the result file
        :type remote_file_path: str
        """
        self._remote_file_path = remote_file_path

    def get_duration(self):
        """
        Returns this test's global execution time.
        :rtype: float
        :return: the test duration
        """
        return self._duration

    def _set_duration(self, duration):
        """
        Sets this test's duration to the given value.
        :param duration: the new durations value
        :type duration: float
        """
        self._duration = duration

    def get_multiple_results(self):
        """
        Returns this multiple result for this test.

        :return: The multiple result dict
        :rtype: dict of dict

        .. seealso:: Report.SecondaryTestReport.SecondaryTestReport
        """
        return self._multiple_results

    def _get_verdict_from_result(self, verdict):
        """
        Compute verdict from GtesterResult into ACS verdict.

        :param verdict: The verdict as text from the GTester result file
        :type verdict: str

        :return: The equivalent verdict useable for ACS
        :rtype: Utilities.Verdict
        """
        if verdict is None:
            raise DeviceException(DeviceException.XML_PARSING_ERROR,
                                  "Unable to statute on \"None\" verdict in GTester result")

        v = Verdict.FAIL
        if verdict.lower() == "success":
            v = Verdict.PASS
        elif verdict.lower() == "skipped":
            v = Verdict.BLOCKED
        return v

    def parse(self):
        """
        Parses the result file and returns a boolean indicating
        whether anything has been done or not.

        :rtype: bool
        :return:
            - C{True} if a result file has been found
            - C{False} otherwise
        """
        parsed = False
        if self.does_local_file_exist():
            document = minidom.parse(self.local_file_path)
            self.parse_document(document)
            parsed = True
        return parsed

    def parse_document(self, document):
        """
        Parses the given XML document.
        """
        # Retrieve the testsuites nodes
        testsuites_nodes = document.getElementsByTagName("testsuites")
        # We consider only the first one
        if len(testsuites_nodes) > 0:
            testsuites_node = testsuites_nodes[0]
            # Parse the node
            self.parse_testsuites_node(testsuites_node)
        else:
            self.skip(document)

    def parse_testsuites_node(self, node):
        """
        Parses the I{testsuites} XML node.

        .. note:: Contains multiple "testsuite" nodes

        :type node: xml.dom.minidom.Node
        :param node: the XML C{Node} to parse.
        """
        # Retrieve the attributes
        attributes_dict = \
            self.from_attributes_to_dictionary(node)

        # Parse the duration attribute
        if "time" in attributes_dict:
            duration_string = attributes_dict["time"]
            self._duration = float(duration_string)

        # Parse the testsuite nodes
        testsuite_nodes = node.getElementsByTagName("testsuite")
        if len(testsuite_nodes) == 0:
            # Retrieve the first node
            testsuite_node = testsuite_nodes[0]
            self.parse_testsuite_node(testsuite_node)
        else:
            # Init the result dict
            if self._multiple_results is None:
                self._multiple_results = {}
            for testsuite_node in testsuite_nodes:
                self.parse_testsuite_node(testsuite_node)

    def parse_testsuite_node(self, node):
        """
        Parses the I{testsuite} XML node.

        :type node: xml.dom.minidom.Node
        :param node: the XML C{Node} to parse.

        """
        # Retrieve the attributes
        attributes_dict = \
            self.from_attributes_to_dictionary(node)
        # Update useful information
        if "name" in attributes_dict:
            name = attributes_dict["name"]
            self._current_ts_name = name

        # Parse the testcase nodes
        testcase_nodes = node.getElementsByTagName("testcase")
        if len(testcase_nodes) == 0:
            # Retrieve the first node
            testcase_node = testcase_nodes[0]
            self.parse_testcase_node(testcase_node)
        else:
            # Init the result dict
            if self._multiple_results is None:
                self._multiple_results = {}
            for testcase_node in testcase_nodes:
                self.parse_testcase_node(testcase_node)

    def parse_testcase_node(self, node):
        """
        Parses the I{testcase} XML node.

        :type node: xml.dom.minidom.Node
        :param node: the XML C{Node} to parse.
        """
        attributes_dict = \
            self.from_attributes_to_dictionary(node)
        # Update useful information
        if "name" in attributes_dict:
            name = attributes_dict["name"]
            # store the current tc id
            self._current_tc_name = str(self._current_ts_name) + '-' + str(name)
            # Init the sub result dic
            if self._multiple_results is not None:
                self._multiple_results[self._current_tc_name] = {}

        # check a failure
        failure_message = None
        failure = None
        failure_node = node.getElementsByTagName("failure")
        if len(failure_node) > 0:
            failure = failure_node[0].firstChild
            if failure is not None:
                failure_message = str(failure.nodeValue)

        # retrieve verdict
        verdict = "failure"
        comment = "Test succeed"
        if "status" in attributes_dict:
            status = attributes_dict["status"]
            # Check if run or not
            if status == "notrun":
                # If the attribues equals "notrun" consider the test as skipped
                verdict = "skipped"
                comment = "Test %s has been skipped (notrun)" % self._current_tc_name

            elif status == "run":
                if failure_message is not None:
                    # Mean fail
                    verdict = "failure"
                    comment = failure_message
                else:
                    # Mean success
                    verdict = "success"
                    comment = "Test %s succeed" % self._current_tc_name

            # Compute verdict
            self._verdict = GResult.from_string(verdict)
            self._messages.append(comment)

            # Compute verdict for multiple results
            if self._multiple_results is not None and \
                    self._current_tc_name in self._multiple_results:
                #  Compute for multiple verdicts
                self._multiple_results[self._current_tc_name]['status'] = self._get_verdict_from_result(verdict)
                self._multiple_results[self._current_tc_name]['comment'] = comment

    # Declare properties
    verdict = property(
        get_verdict,
        None,
        None,
        "This test verdict.")

    file_name = property(
        get_file_name,
        None,
        None,
        "The result file name.")

    remote_file_path = property(
        get_remote_file_path,
        None,
        None,
        "The path to the result file on the DUT.")

    local_file_path = property(
        get_local_file_path,
        None,
        None,
        "The path to the local result file.")

    duration = property(
        get_duration,
        None,
        None,
        "The test global evaluation time.")

    log_messages = property(
        get_log_messages,
        None,
        None,
        "The log messages.")

    multiple_results = property(
        get_multiple_results,
        None,
        None,
        "The multiple results dict used when many test cases ran.")
