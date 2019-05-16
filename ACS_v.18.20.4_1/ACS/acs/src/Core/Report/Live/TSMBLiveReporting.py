#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG/AMPS/SI/SIS/CTA
@summary: The module will send live message events via TSMB
@since: 03/11/2015
@author: lbavois
"""

import time
import logging
import os
import socket
import platform
import uuid
from Core.CampaignMetrics import CampaignMetrics
from UtilitiesFWK.Utilities import get_acs_release_version
from UtilitiesFWK.Utilities import Verdict, Status
from UtilitiesFWK.DateUtilities import utctime_iso8601


try:
    from sis_test_services_tsmb import tsmb
    TSMB_INSTALLED = True
except ImportError:
    TSMB_INSTALLED = False

from Core.Report.ACSLogging import ACS_LOGGER_NAME, FORMATER_CONF, FORMATER_DATE_CONF


class TSMBLiveReporting(object):

    """
    Class that will push live reporting event to Test Campaign Report
    through a message bus (TSMB)
    """

    # Enable or disable live reporting for test suite on TSMB
    _reporting_enabled = False
    # Test suite execution owner
    _userId = ""

    # Information on ACS test bench
    __bench_info = dict()

    # Test suite report message current instance used
    __test_suite_report_msg = None

    # Test case report message current instance used
    __test_case_report_msg = None

    # Logger for this class
    __logger = None

    # Two possible mode for sending test case report message exist:
    # testCaseDetailedReportMode = True (send start + update + stop test case messages during test case execution)
    # testCaseDetailedReportMode = False (send only stop test case message during test case execution)
    # Same information are finally sent to reporting system
    # testCaseDetailedReportMode = True allows to report information when available during test case execution
    # testCaseDetailedReportMode = False allows to report all information at test case execution completion
    # By default choose to only send test case report message when test case execution is complete
    __testCaseDetailedReportMode = True

    if TSMB_INSTALLED:
        acsTcVerdictToTsmbTcVerdictMapping = {Verdict.BLOCKED: tsmb.TEST_VERDICT.BLOCKED,
                                              Verdict.FAIL: tsmb.TEST_VERDICT.FAILED,
                                              Verdict.PASS: tsmb.TEST_VERDICT.PASSED,
                                              Verdict.INTERRUPTED: tsmb.TEST_VERDICT.NOT_COMPLETED}

        acsTestSuiteStatusToTsmbTestSuiteStatusMapping = {Status.INIT: tsmb.TEST_STATUS.INIT,
                                                          Status.ONGOING: tsmb.TEST_STATUS.ONGOING,
                                                          Status.ABORTED: tsmb.TEST_STATUS.ABORTED,
                                                          Status.COMPLETED: tsmb.TEST_STATUS.COMPLETED}
    else:
        acsTcVerdictToTsmbTcVerdictMapping = {}
        acsTestSuiteStatusToTsmbTestSuiteStatusMapping = {}

    @property
    def is_reporting_enabled(self):
        """
        Is reporting enabled?

        :rtype: bool
        """
        return TSMB_INSTALLED and self._reporting_enabled

    @is_reporting_enabled.setter
    def is_reporting_enabled(self, value):
        """
        Set reporting status

        :param value: reporting status
        :type value: bool
        """
        self._reporting_enabled = value

    @staticmethod
    def __retrieve_bench_info():
        """
        Return all information from current ACS test bench
        :rtype: bench info
        """
        user_home = os.path.split(os.path.expanduser('~'))

        if platform.system() == "Windows":
            release = platform.release()
        elif platform.dist():
            release = "{0}_{1}".format(platform.dist()[0], platform.dist()[1])

        # List bench information
        bench_info = {"name": socket.getfqdn(),
                      "user": user_home[-1],
                      "os": "{0}_{1}".format(platform.system(), release),
                      "acsVersion": get_acs_release_version(),
                      "pythonVersion": "{0}_{1}".format(platform.python_version(), platform.architecture()[0])}

        return bench_info

    def __init__(self, report_folder):
        """
        Constructor
        """
        if report_folder:
            # Create the logger file when the reporting folder is passed
            self.__logger = self._create_live_logger(report_folder)

        # Retrieve current ACS bench info (need to be done one time on current test bench)
        self.__bench_info = TSMBLiveReporting.__retrieve_bench_info()

        if TSMB_INSTALLED:
            # Connect to staging RabbitMQ message bus
            # these credentials allow only access to staging environment for testing purpose
            tsmb.set_credentials(server="cortex.tl.intel.com",
                                 port=5673,
                                 vhost="taas",
                                 user="sys_taasmqst",
                                 password="sys_tsmb_staging_31")

        self._msg_send_meantime = 0
        self._msg_nb = 0

    def _create_live_logger(self, report_dir):
        """
        Create live reporting instance with a dedicated log file in the campaign report dire
        Disable requests log and write them in live reporting logger file

        :param report_dir: campaign report folder
        :type report_dir: str

        :return: live logger report instance
        """
        logger = None

        if report_dir:
            logger = logging.getLogger("%s.FWK.TSMBLiveReporting" % ACS_LOGGER_NAME)
            logger.propagate = False
            # Configure logger
            logger.handlers = []
            logfile = os.path.join(report_dir, "TSMB_live_reporting.log")
            file_handler = logging.FileHandler(logfile)
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(logging.Formatter(FORMATER_CONF, FORMATER_DATE_CONF))
            logger.addHandler(file_handler)

        return logger

    def _init_test_suite_report_message(self, test_suite_uuid, test_suite_name, build_ref):
        """
        Constructor for test suite report message send on TSMB bus
        """
        # Create test suite report message for TSMBLiveReporting class instance
        # This test suite report message instance will be used and updated during all test suite execution time

        # Test if tsmb library import is successful before using it
        if TSMB_INSTALLED:
            msg = self.__get_message_from_bus("taas.test.suite")

            if msg:
                # Set test suite configuration information one time for all test suite execution
                msg.testRequestId = test_suite_uuid
                msg.name = test_suite_name
                msg.userId = self._userId
                msg.buildName = build_ref

                # Initialize test suite execution information
                msg.status = tsmb.TEST_VERDICT.RUNNING
                msg.comment = ""

            else:
                if self.__logger:
                    self.__logger.debug("Failed to connect to the AQMP Server!")
        else:
            msg = None

        # Keep test suite report message information stored during all test suite execution time
        self.__test_suite_report_msg = msg

    def _init_test_case_report_message(self, test_suite_uuid, test_case_name, test_case_sequence_number):
        """
        Constructor for test case report message send on TSMB bus
        """
        # Create test case report message for TSMBLiveReporting class instance
        # This test case report message instance will be used and updated during all test suite execution time

        # Test if tsmb library import is successful before using it
        if TSMB_INSTALLED:
            msg = self.__get_message_from_bus("taas.test.case")

            if msg:
                # Set test case configuration information one time for all test case execution
                # -----------------------------------------------------------------------------
                msg.testRequestId = test_suite_uuid
                msg.id = uuid.uuid4()
                msg.name = test_case_name
                msg.sequenceNumber = test_case_sequence_number
                msg.type = tsmb.TEST_TYPE.AUTO
                msg.automationTool = tsmb.AUTOMATION_TOOL.ACS
                msg.userId = self._userId
                msg.benchInfo = self.__bench_info

                # Initialize test case execution information
                # -------------------------------------------
                msg.verdict = tsmb.TEST_VERDICT.NO_RUN
                msg.comment = ""
                # Build test case report location
                msg.reportLocation = ""
                # Init crash list
                msg.crashList = []
                # Init device info (only retrieve at test case execution)
                msg.dutInfo = {}

                # Add potential test suite configuration or execution information
                msg.additionalInfo = {}
                msg.additionalResults = {}
            else:
                if self.__logger:
                    self.__logger.debug("Failed to connect to the AQMP Server!")
        else:
            msg = None

        # Keep test case report message information stored during all test case execution time
        self.__test_case_report_msg = msg

    def __send_message(self, tsmb_message):
        """
        Send report message on tsmb bus

        :param tsmb_message: tsmb message

        """
        # Test if tsmb library import is successful before using it
        if TSMB_INSTALLED:
            self.__logger and self.__logger.info("Send message on TSMB bus: {0})".format(tsmb_message.dict))

            try:
                t0 = time.time()
                tsmb_message.send()
                t1 = time.time()
                delta = (t1 - t0) * 1000
                self._msg_send_meantime += delta
                self._msg_nb += 1

                # @TODO: return code not handled in TSMB library for the moment after message sending
                # Shall be done when available
                # send_status = tsmb_message.send()
                # self.__logger and self.__logger.info("Sending status: {0})"
                #                                      .format("SUCCESS" if send_status else "FAILURE"))
            except Exception as exception:
                if self.__logger:
                    self.__logger.error("Sending status: ERROR (error:{0})".format(str(exception)))

    def send_start_campaign_info(self, testRequestId, test_suite_name, email):
        """
        Send start test suite message on TSMB bus

        :param testRequestId: test suite execution unique identifier
        :param test_suite_name: test suite name
        :param email: user email
        """
        if self.is_reporting_enabled:
            # Store userId that initiate the test suite execution
            self._userId = email

            # Initialize test suite report message
            self._init_test_suite_report_message(testRequestId, test_suite_name, "unknown")

            if self.__test_suite_report_msg:
                # Test suite execution is starting
                self.__test_suite_report_msg.status = tsmb.TEST_VERDICT.RUNNING
                # Put test suite starting execution time
                self.__test_suite_report_msg.startTime = utctime_iso8601()

                # Send test suite report message
                self.__logger and self.__logger.info("Start test suite uuid:{0})"
                                                     .format(str(self.__test_suite_report_msg.testRequestId)))
                self.__send_message(self.__test_suite_report_msg)
            else:
                self.__logger and self.__logger.error("Start test suite uuid:{0} "
                                                      "but no test suite report message created)"
                                                      .format(str(testRequestId)))

    def send_stop_campaign_info(self, status):
        """
        Send stop test suite message on TSMB bus

        :param status: test suite execution status

        """
        if self.is_reporting_enabled:

            if self.__test_suite_report_msg:
                status = tsmb.TEST_VERDICT.FAILED if bool(status) else tsmb.TEST_VERDICT.PASSED
                # build name has been retrieve at test case execution ending phase
                # Convert test suite status to report system value
                # Feature not ready
                # if status in TSMBLiveReporting.acsTestSuiteStatusToTsmbTestSuiteStatusMapping:
                #    tsmb_status = TSMBLiveReporting.acsTestSuiteStatusToTsmbTestSuiteStatusMapping[status]
                # else:
                #    tsmb_status = ""

                self.__test_suite_report_msg.status = status
                # Retrieve comment associated to test suite status
                self.__test_suite_report_msg.comment = ""
                self.__test_suite_report_msg.stopTime = utctime_iso8601()

                # Add test suite execution information
                # Compute test suite execution rate / test case pass rate / test case failure rate
                # TO DO: Shall be done in the reporting system to take into account all
                # results from different test benches
                execution_rate = CampaignMetrics.instance().execution_rate
                pass_rate = CampaignMetrics.instance().pass_rate
                fail_rate = CampaignMetrics.instance().fail_rate

                # Specific ACS data for test suite execution
                # @TODO: Shall be done in the reporting system (info shall be captured at test case level)
                # to take into account all results from different test benches
                acs_stats = {"TotalBootCount": CampaignMetrics.instance().total_boot_count,
                             "ConnectFailureCount": CampaignMetrics.instance().connect_failure_count,
                             "MeanTimeBeforeFailure": CampaignMetrics.instance().mtbf,
                             "TimeToCriticalFailure": CampaignMetrics.instance().time_to_first_critical_failure,
                             "CriticalFailureCount": CampaignMetrics.instance().critical_failure_count,
                             "UnexpectedRebootCount": CampaignMetrics.instance().unexpected_reboot_count,
                             "BootFailureCount": CampaignMetrics.instance().boot_failure_count}

                self.__test_suite_report_msg.additionalData.update({"executionRate": execution_rate,
                                                                    "passRate": pass_rate,
                                                                    "failRate": fail_rate,
                                                                    "ACS": acs_stats})

                # Send test suite report message
                self.__logger and self.__logger.info("Stop test suite uuid:{0})"
                                                     .format(str(self.__test_suite_report_msg.testRequestId)))
                self.__send_message(self.__test_suite_report_msg)

                # Test suite execution is completed on the test bench, delete test suite message context
                self.__test_suite_report_msg = None

            else:
                self.__logger and self.__logger.error("Stop test suite, "
                                                      "but no test suite starting message previously sent)")

    def send_start_tc_info(self, tc_name, tc_order_in_test_suite):
        """
        Send start test case message on TSMB bus

        :param tc_name: test case name
        :param tc_order_in_test_suite: test case order define in test suite
        """
        # Check that tsmb reporting is turned on and test suite report message has already been sent
        if self.is_reporting_enabled:
            if self.__test_suite_report_msg:
                # Initialize test case report message
                self._init_test_case_report_message(self.__test_suite_report_msg.testRequestId,
                                                    tc_name, tc_order_in_test_suite)

                if self.__test_case_report_msg:
                    # Set test case execution information
                    # -------------------------------------
                    # Test case execution is starting
                    self.__test_case_report_msg.verdict = tsmb.TEST_VERDICT.RUNNING
                    # Put test case starting execution time
                    self.__test_case_report_msg.startTime = utctime_iso8601()

                # Only send start test case report message, if required
                if self.__testCaseDetailedReportMode:
                    # Send test case report message
                    self.__logger and self.__logger.info("Start test case (name={0}, sequence order={1})"
                                                         .format(str(tc_name), str(tc_order_in_test_suite)))
                    self.__send_message(self.__test_case_report_msg)

            else:
                self.__logger and self.__logger.error("Start test case {0}, "
                                                      "but no test suite starting message previously sent)"
                                                      .format(str(tc_name)))

    def send_stop_tc_info(self, verdict, comment=""):
        """
        Send stop test case message on TSMB bus

        :param verdict: test case verdict
        :param comment: test case execution comments
        """

        # Check that tsmb reporting is turned on and test suite report message has already been sent
        if self.is_reporting_enabled:
            if self.__test_suite_report_msg:
                if self.__test_case_report_msg:
                    # Set test case execution information
                    # -------------------------------------
                    # compute test case verdict for test suite reporting system
                    if verdict in TSMBLiveReporting.acsTcVerdictToTsmbTcVerdictMapping:
                        tsmb_verdict = TSMBLiveReporting.acsTcVerdictToTsmbTcVerdictMapping[verdict]
                    else:
                        tsmb_verdict = tsmb.TEST_VERDICT.NA

                    # Set test case verdict
                    self.__test_case_report_msg.verdict = tsmb_verdict
                    # Set comment associated to test case execution
                    self.__test_case_report_msg.comment = comment
                    # Put test case starting execution time
                    self.__test_case_report_msg.stopTime = utctime_iso8601()

                    # Send test case report message
                    self.__logger and self.__logger.info("Stop test case (name={0}, order={1}, verdict={2} )"
                                                         .format(str(self.__test_case_report_msg.name),
                                                                 str(self.__test_case_report_msg.sequenceNumber),
                                                                 str(self.__test_case_report_msg.verdict)))

                    self.__send_message(self.__test_case_report_msg)
                else:
                    self.__logger and self.__logger.error("Stop test case, but no test case starting procedure done")
            else:
                self.__logger and self.__logger.error("Stop test case, "
                                                      "but no test suite starting message previously sent)")

    def update_running_tc_info(self, crash_list=list(), test_info=None, device_info=None):
        """
        Update a test case (during b2b iterations)

        :type crash_list: crash id list
        :param crash_list: Current crash list

        :param test_info (optional): Additional test case result info (can test step, external
                                     test case results, other ..)
                                    (Dictionnary key = section name, data = json serializable value data)

        :param device_info (optional): Additional device info of the dut.
        :type device_info (optional) : dict
        """

        # Check that tsmb reporting is turned on and test suite report message has already been sent
        if self.is_reporting_enabled:
            if self.__test_suite_report_msg:
                if self.__test_case_report_msg:
                    # Set test case execution information
                    # -------------------------------------
                    # Update crash list events for current test case
                    if crash_list:
                        self.__test_case_report_msg.crashList.extend(crash_list)

                    # Update additional test case result information for current test case
                    if test_info:
                        self.__test_case_report_msg.additionalResults.update(test_info)

                    # Update dut information for current test case
                    if device_info:
                        self.__test_case_report_msg.dutInfo = device_info

                    # Send test case report message
                    self.__logger and self.__logger.info("Update test case (name={0}, order={1})"
                                                         .format(str(self.__test_case_report_msg.name),
                                                                 str(self.__test_case_report_msg.sequenceNumber)))
                else:
                    self.__logger and self.__logger.error("Update test case, but no test case starting procedure done!")
            else:
                self.__logger and self.__logger.error("Update test case, but no test suite starting message "
                                                      "previously sent!)")

    def close(self):
        """
            Close Message Bus connection
        @return:
        """
        if self.is_reporting_enabled:
            if self._msg_nb:
                self._msg_send_meantime /= self._msg_nb
                self.__logger.debug("Meantime of bus message sending is "
                                    "{0} ms".format(self._msg_send_meantime))
            tsmb.close()

    def __get_message_from_bus(self, message_id):
        """
            Get Message from Bus
        :param message_id: Name of the message to instantiate (shall be defined in the spec)
        :type message_id: str

        :return: Instance of the message if found, else AttributeError
        """
        try:
            message = tsmb.get_message(message_id) if TSMB_INSTALLED else None
        except Exception as exception_text:
            message = None
            self.__logger and self.__logger.error("Sending status: ERROR (error:{0})".format(str(exception_text)))

        return message
