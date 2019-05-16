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
:summary: Parsing utilities for GTester
:author: asebbanx
:since: 29/10/2012
"""

from datetime import datetime
import os.path
import re
import shutil
from xml.dom import minidom

from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from UtilitiesFWK.Utilities import Global, Verdict
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class GParserBase(object):

    """
    A base class for XML parsers.
    """

    def __init__(self, logger=None):
        """
        Initializes this instance.

        :type logger: logging
        :param logger: the logger instance to use
        """
        self.__logger = logger

    def from_attributes_to_dictionary(self, node):
        """
        Returns a Python C{dict} build from the given node's attributes.

        :type node: xml.dom.minidom.Node
        :param node: the XML C{Node} in which to search.

        :rtype: dict
        :return: a dictionary having attribute names as keys and their
            matching str values as values.
        """
        # Initialize the return value
        attributes = {}
        # Iterate on the node's attributes
        for attribute_name in node.attributes.keys():
            # Retrieve the attribute's value
            value = self.get_attribute_value(node, attribute_name)
            # Update the result dictionary
            attributes[str(attribute_name)] = str(value)
        # Return the dictionary
        return attributes

    def get_attribute_value(self, node, attribute_name):
        """
        Returns the attribute value for the given name in
        the given node.

        :type node: xml.dom.minidom.Node
        :param node: the XML C{Node} in which to search.

        :type attribute_name: str
        :param attribute_name: the attribute's name for which we want
            the corresponding value.

        :rtype: str
        :return: the attributes value if it can be found (or
            C{None} otherwise.
        """
        # Initialize the return value
        value = None
        # Retrieve the node's attributes
        attrs = node.attributes
        # Check that the attributes are not empty
        if attrs is not None:
            # Check that an attribute with the given name exists
            if attribute_name in attrs.keys():
                # Retrieve the attribute's value
                value = attrs[attribute_name].value
            else:
                self.skip(attribute_name)
        # Return the attribute value
        return value

    def skip(self, node):
        """
        Peforms any useful action when a C{Node} shall not
        be taken into account by this object.
        :rtype: None
        """
        # Do nothing
        self.logger.debug("Skipping node: %s." % (str(node)))

    def get_logger(self):
        """
        Returns the logger instance used by this parser.
        """
        # If the logger was not provided create an instance
        if self.__logger is None:
            self.__logger = LOGGER_TEST_SCRIPT
        # Return the logger
        return self.__logger

    logger = property(
        get_logger,
        None,
        None,
        "The logger instance.")


class GCampaignParser(GParserBase):

    """
    A class that is useful to parse C{GTester} campaigns.
    """

    DEFAULT_GTESTER_RESULT_FILE = "gtester_result.xml"
    """
    The name of the result file to be given to C{GTester}.
    This is used as a default value and should be replaced
    with a better suited name whenever it is possible.
    """

    def __init__(self, logger=None):
        """
        Initializes this instance.

        :type logger: Report.Logging.Logging
        :param logger: the logger instance to use
        """
        GParserBase.__init__(self, logger)
        self.__commands = []

    def get_commands(self):
        """
        Returns the list of all the commands that have been
        parsed so far.

        Note that this method returns a copy of the command list
        so that the original cannot be modified outside of this class.

        :rtype: list
        :return: the commands that have been parsed by this object.
        """
        return list(self.__commands)

    def parse_campaign(self, campaign_file):
        """
        Parses the given C{GTester} campaign file and updates the
        C{commands} property.

        :type campaign_file: str
        :param campaign_file: the path to the campaign_file
        """
        # Initialize a variable that will hold all the
        # lines from the campaign file.
        command_lines = None
        try:
            # Open the file in read mode
            with open(campaign_file, 'r') as file_object:
                # As the GTester campaign files are rather small,
                # we can allow ourselves to store all the lines
                # of the file at the same time in a variable without
                # risking memory issues.
                command_lines = file_object.readlines()
        # Catch possible IO Error
        except IOError as io_error:
            message = "I/O error({0}): {1}".format(
                io_error.errno,
                io_error.strerror)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, message)

        # Catch other kind of error/exceptions
        # (errors/exceptions possibly raised by 'realines' are not documented)
        except:
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                     "Could not parse the campaign file (%s)." % str(campaign_file))
        # Parse the retrieved lines
        self.parse_lines(command_lines)

    def parse_lines(self, lines):
        """
        Parses the given C{lines} and builds C{GTester} commands from them,
        returns the number of processed commands.

        The complete C{GTester} commands are updated to the C{commands} property.

        :param lines: the lines to parse.
        :type lines: list

        :rtype: int
        :return: the number of commands that have been processed.
        """
        line_count = 0
        if lines is not None:
            for command in lines:
                if command is not None and command != "":
                    gtester_command = self.build_gtester_command(command)
                    self.__commands.append(gtester_command)
                    line_count += 1
        return line_count

    def is_command_valid(self, command):
        """
        Returns a boolean indicating whether the given command can
        be a valid candidate for a C{GTester} command or not.
        :param command: the command to test
        :type command: str

        :rtype: bool
        :return: whether the command can be used to build a C{GTester}
            command or not:
            - C{True} is the command can be used for C{GTester}
            - C{False} otherwise
        """
        # Initialize the return value
        valid = False
        # Consider that the command is not a comment by default
        comment = False
        # If command is not None or empty str
        if command:
            # Remove leading spaces
            command = command.lstrip(" \t")
            # Test whether the command is a comment or not
            if command != "":
                if command.startswith("#"):
                    comment = True
        # Update the valid status if command is not None or empty str
        if command and not comment:
            valid = True
        # Return the status
        return valid

    def build_gtester_command(self, campaign_command):
        """
        Returns a well constructed C{Gtester} command built from
        the given command element retrieved from the campaign file.
        :param campaign_command: the command read from the campaign file.
        :type campaign_command: str

        :rtype: str
        :return: the complete C{GTester} command.
        """
        # Find and extract the binary path
        bin_regexp = "(\/[\w-]+)+"
        regexp = re.compile(bin_regexp)
        matcher = regexp.match(campaign_command)
        binary_path = matcher.group(0)
        # Find where the binary path is present in the original command
        index = campaign_command.index(binary_path)
        # Add the ' -- ' option at the appropriate position
        index += len(binary_path)
        campaign_command = "%s --%s" % \
            (campaign_command[:index], campaign_command[index:])
        # Add the gtester command call and appropriate options
        gtester_command = "%s %s %s" % (
            "gtester -k -o",
            GCampaignParser.DEFAULT_GTESTER_RESULT_FILE,
            campaign_command)
        # Return the command
        return gtester_command

    # Declare properties
    commands = property(
        get_commands,
        None,
        None,
        "The list of all commands that have been parsed by this object.")


class GResult(object):

    """
    A class to hold the result and significant information
    about a C{GTester} result or an I{AT} command result.

    We want to avoid inheritance for simplicity reasons.
    But some utility method are provided to create C{GResult}
    instances in an easy way.
    """

    SUCCESS = Global.SUCCESS
    """
    Class attribute representing a GTester success.
    """

    FAILED = Global.FAILURE
    """
    Class attribute representing a GTester failure.
    """

    SKIPPED = Global.BLOCKED
    """
    Class attribute representing a test that has
    been skipped.
    """

    def __init__(self, verdict=None, messages=None):
        """
        Initializes this instance.
        """
        # Process the verdict parameter
        if verdict is None:
            self.__verdict = GResult.FAILED
        else:
            self.__verdict = verdict
        # Setting a parameter's default value to [] is discouraged
        # by PyLint so we handle that specific case here.
        if messages is None:
            self.__messages = []
        else:
            self.__messages = messages

    def get_log_messages(self):
        """
        Returns this test log messages.

        :rtype: list
        :return: the log messages as list
        """
        return self.__messages

    def get_verdict(self):
        """
        Returns this test result's verdict if any test
        has actually been executed (and C{GResult.FAILED}
        otherwise).
        """
        return self.__verdict

    @classmethod
    def to_string(cls, numeric_value):
        """
        Returns the C{str} equivalent of the given value.
        C{numeric_value} is expected to be the return value
        of a gtester verdict.

        :type numeric_value: int
        :param numeric_value: the numeric value corresponding
            to a gtester verdict.

        :rtype: str
        :return: the str value corresponding to C{numeric_value} or C{None}.
        """
        for key, value in vars(cls).iteritems():
            if value == numeric_value:
                return key.lower()
        return None

    @classmethod
    def from_string(cls, string_value):
        """
        Returns the C{int} equivalent of the given value.
        C{string_value} is expected to be the textual representation
        of a gtester verdict.

        :type string_value: str
        :param string_value: the str value explaining the gtester result.

        :rtype: int
        :return: the integer value corresponding to C{string_value} or C{None}
        """
        upper_case_value = str(string_value).upper()
        return getattr(cls, upper_case_value, None)

    @classmethod
    def from_gtester_result(cls, gtester_result):
        """
        Returns a C{GResult} instance built from the given
        C{GTesterResult} instance.

        :type gtester_result: GTesterResult
        :param gtester_result: any C{GTesterResult} instance.

        :rtype: GResult
        :return: a C{GResult} instance containing the useful data
            held by the given C{GTesterResult} instance.
        """
        # Create a new instance
        instance = GResult(
            gtester_result.get_verdict(),
            gtester_result.get_log_messages())
        # Return the instance
        return instance


class GTesterResult(GParserBase):

    """
    A class to handle operations for C{GTester} results.
    """

    GTESTER_RESULT_ROOT = "/data/component_testing"
    """
    The root directory where all GTester test results
    will be stored.
    """

    def __init__(
            self,
            file_name=None,
            remote_dir_path="/data/component_testing",
            local_dir_path="."):
        """
        Initializes this instance.
        """
        GParserBase.__init__(self)
        if file_name is None:
            file_name = GTesterResult.build_file_name()
        self._file_name = file_name
        self._remote_dir_path = remote_dir_path
        self._local_dir_path = local_dir_path
        self._remote_file_path = None
        self._local_file_path = None
        self._verdict = GResult.FAILED
        self._binary_path = None
        self._duration = 0.0
        self._messages = []

        self._multiple_results = None
        self._current_tc_id = None

    @classmethod
    def create(cls, file_name=None):
        """
        Returns a new C{GTesterResult} instance built
        with the given file name if any or with a
        generated file name otherwise.
        :rtype: GTesterResult
        :return: a new C{GTesterResult} instance
        """
        if file_name is None:
            file_name = GTesterResult.build_file_name()
        instance = GTesterResult(file_name)
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
        file_name = "gtester_result_%s.xml" % timestamp
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

    def get_binary_path(self):
        """
        Returns this test's binary path.
        :rtype: str
        :return: the binary path
        """
        return self._binary_path

    def _set_binary_path(self, binary_path):
        """
        Sets this test's binary path to the given value.
        :param binary_path: the path to the binary file used for this test
        :type binary_path: str
        """
        self._binary_path = binary_path

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
        elif verdict.lower() == "blocked":
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

    def parse_gtester(self):
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
            self.parse_document_gtester(document)
            parsed = True
        return parsed

    def parse_document_for_result(self):
        """
        Parses the given XML document for extract verdict for all test.

        :return: - list_test_pass : list of pass test
                 - list_test_blocked : list of blocked test
                 - list_test_fail : list of fail test
        :rtype: list
        """

        # parse the result document with minidom
        if self.does_local_file_exist():
            document = minidom.parse(self.local_file_path)

        # init of list permit to return list of fail and blocked test.
        list_test_pass = []
        list_test_fail = []
        list_test_blocked = []

        # Retrieve the testbinary nodes
        testbinary_nodes = document.getElementsByTagName("gtester")

        # We consider only the first one
        if len(testbinary_nodes) > 0:
            testbinary_node = testbinary_nodes[0]

        # Research testcase node
        for node in document.getElementsByTagName('testcase'):
            # Research status node in testcase
            for child in node.getElementsByTagName('status'):
                # find value of result attribute
                result = child.attributes['result']
                # find value of path attribute
                test = node.attributes['path']
                # adds the test to the corresponding list
                if result.value == 'success':
                    list_test_pass.append(test.value)
                elif result.value == 'failed':
                    list_test_fail.append(test.value)
                elif result.value == 'blocked':
                    list_test_blocked.append(test.value)

        return list_test_pass, list_test_fail, list_test_blocked


    def parse_document(self, document):
        """
        Parses the given XML document.
        """
        # Retrieve the testbinary nodes
        testbinary_nodes = document.getElementsByTagName("testbinary")

        # We consider only the first one
        if len(testbinary_nodes) > 0:
            testbinary_node = testbinary_nodes[0]
            # Parse the node
            self.parse_testbinary_node(testbinary_node)
        else:
            self.skip(document)


    def parse_document_gtester(self, document):
        """
        Parses the given XML document.
        """
        # Retrieve the testbinary nodes
        gtester_nodes = document.getElementsByTagName("gtester")

        # We consider only the first one
        if len(gtester_nodes) > 0:
            gtester_node = gtester_nodes[0]
            # Parse the node
            self.parse_gtester_node(gtester_node)
        else:
            self.skip(document)


    def parse_testbinary_node(self, node):
        """
        Parses the I{testbinary} XML node.

        :type node: xml.dom.minidom.Node
        :param node: the XML C{Node} to parse.
        """
        # Retrieve the attributes
        attributes_dict = \
            self.from_attributes_to_dictionary(node)
        # Update useful information
        if "path" in attributes_dict:
            path = attributes_dict["path"]
            self._binary_path = path

        # Parse the test case node
        testcase_nodes = node.getElementsByTagName("testcase")
        if len(testcase_nodes) == 0:
            # Retrieve the first node
            testcase_node = testcase_nodes[0]
            self.parse_testcase_node(testcase_node)
        else:
            # Init the result dict
            self._multiple_results = {}
            for testcase_node in testcase_nodes:
                self.parse_testcase_node(testcase_node)

        # Parse the duration node
        # Retrieve THE child node named duration
        # and not a node that could have been found elsewhere
        for current_node in node.childNodes:
            if current_node.nodeType == node.ELEMENT_NODE:
                if current_node.tagName == "duration":
                    self.parse_duration_node(current_node)
                elif current_node.tagName == "message":
                    self.parse_message_node(current_node)
                else:
                    self.skip(current_node)


    def parse_gtester_node(self, node):
        """
        Parses the I{gtester} XML node.

        :type node: xml.dom.minidom.Node
        :param node: the XML C{Node} to parse.
        """
        # Retrieve the attributes
        attributes_dict = \
            self.from_attributes_to_dictionary(node)
        # Update useful information
        if "path" in attributes_dict:
            path = attributes_dict["path"]
            self._binary_path = path

        # Parse the test case node
        testcase_nodes = node.getElementsByTagName("testcase")
        if len(testcase_nodes) == 0:
            # Retrieve the first node
            testcase_node = testcase_nodes[0]
            self.parse_testcase_node(testcase_node)
        else:
            # Init the result dict
            self._multiple_results = {}
            for testcase_node in testcase_nodes:
                if testcase_node.getAttribute('skipped') == '1':
                    pass
                else:
                    self.parse_testcase_node(testcase_node)

        # Parse the duration node
        # Retrieve THE child node named duration
        # and not a node that could have been found elsewhere
        for current_node in node.childNodes:
            if current_node.nodeType == node.ELEMENT_NODE:
                if current_node.tagName == "duration":
                    self.parse_duration_node(current_node)
                elif current_node.tagName == "message":
                    self.parse_message_node(current_node)
                else:
                    self.skip(current_node)

    def parse_testcase_node(self, node):
        """
        Parses the I{testcase} XML node.

        :type node: xml.dom.minidom.Node
        :param node: the XML C{Node} to parse.
        """
        # Retrieve the verdict attribute
        attributes_dict = self.from_attributes_to_dictionary(node)

        # Note the path as tc id for multiple verdict
        if self._multiple_results is not None:
            if "path" in attributes_dict:
                tc_id = attributes_dict["path"]
                # store the current tc id
                self._current_tc_id = tc_id
                # Init the sub result dic
                self._multiple_results[self._current_tc_id] = {}

        # Check whether this test has been skipped or not
        if "skipped" in attributes_dict:
            verdict = attributes_dict["skipped"]
            if verdict == "1":
                # If the attribues equals 1 consider the test as skipped
                verdict = "skipped"
                self._verdict = GResult.from_string(verdict)
                # Compute for multiple results
                if self._multiple_results is not None and \
                        self._current_tc_id in self._multiple_results:
                    self._multiple_results[self._current_tc_id]['status'] = \
                        self._get_verdict_from_result(verdict)

        # Retrieve the status node (the first one if several are present)
        status_nodes = node.getElementsByTagName("status")
        if len(status_nodes) > 0:
            # Retrieve the first node
            status_node = status_nodes[0]
            self.parse_status_node(status_node)

        # Retrieve the message nodes
        message_nodes = node.getElementsByTagName("message")
        if len(message_nodes) > 0:
            for message_node in message_nodes:
                self.parse_message_node(message_node)
        else:
            if self._multiple_results is not None and \
                self._current_tc_id in self._multiple_results:
                self._multiple_results[self._current_tc_id]['comment'] = 'no comment'

    def parse_message_node(self, node):
        """
        Parses the I{message} XML node.

        :type node: xml.dom.minidom.Node
        :param node: the XML C{Node} to parse.
        """
        message = ""
        text_node = node.firstChild
        if text_node is not None:
            message = text_node.nodeValue
        self._messages.append(message)

        if self._multiple_results is not None and \
                self._current_tc_id in self._multiple_results:
            #  Compute for multiple verdicts
            if 'comment' in self._multiple_results[self._current_tc_id]:
                self._multiple_results[self._current_tc_id]['comment'] \
                    += ' ' + message
            else:
                self._multiple_results[self._current_tc_id]['comment'] = message

    def parse_status_node(self, node):
        """
        Parses the I{status} XML node.

        :type node: xml.dom.minidom.Node
        :param node: the XML C{Node} to parse.
        """
        # Retrieve the verdict attribute
        attributes_dict = self.from_attributes_to_dictionary(node)
        # Retrieve the verdict attribute value
        if "result" in attributes_dict:
            verdict = attributes_dict["result"]
            self._verdict = GResult.from_string(verdict)
            if self._multiple_results is not None and \
                    self._current_tc_id in self._multiple_results:
                #  Compute for multiple verdicts
                self._multiple_results[self._current_tc_id]['status'] = \
                    self._get_verdict_from_result(verdict)

    def parse_duration_node(self, node):
        """
        Parses the I{duration} XML node.

        :type node: xml.dom.minidom.Node
        :param node: the XML C{Node} to parse.
        """
        # Retrieve the text node
        text_node = node.firstChild
        duration_string = text_node.nodeValue
        self._duration = float(duration_string)

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

    binary_path = property(
        get_binary_path,
        None,
        None,
        "The path to the binary file used for this test.")

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
