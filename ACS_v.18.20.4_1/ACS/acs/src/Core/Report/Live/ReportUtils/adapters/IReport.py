"""
@copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

@organization: INTEL SSG/OTC/QSI/SIS/CTA
@summary: The module describe the interface to use by all the adapters to send events
to the corresponding report server
@since: 25/09/15
@author: ssavrimoutou
"""
import abc


class IReport(object):

    """
    Report module interface
    """

    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def start_campaign(self, campaign_uuid, campaign_name, email):
        """
        Send start campaign event to the server

        :param uuid: campaign execution instance unique identifier
        :type uuid: str

        :param campaign_name: campaign name
        :type campaign_name: str

        :param email: user email
        :type email: str
        """

    @abc.abstractmethod
    def send_campaign_resource(self, resource, display_name=None, retention="SHORT"):
        """
        Push a resource onto TCR REST API for a given Campaign

        :param resource: Local resource to be pushed onto TCR at Campaign level.
        :type resource: str

        :param display_name (optional): Filename to be displayed in the UI
        :type display_name: str

        :param retention (optional): "SHORT" or "LONG"
        :type retention: str
        """

    @abc.abstractmethod
    def stop_campaign(self, status, verdict, cp_info=None):
        """
        Send stop campaign event to the server

        :param status: campaign status
        :type status: str

        :param verdict: campaign verdict
        :type verdict: str

        :param cp_info: additional campaign information
        :type cp_info: dict
        """

    @abc.abstractmethod
    def create_testcase(self, tc_name, uc_name, tc_phase, tc_type, tc_domain, tc_order, is_warning, tc_parameters):
        """
         Send http request to server to create test case entry

        :param tc_name: test case name
        :type tc_name: str

        :param uc_name: use case name
        :type uc_name: str

        :param tc_order: the test case order
        :type tc_order: int

        :param is_warning: is TC marked as warning or not
        :type is_warning: bool

        :param tc_parameters: parameters of the test case
        :type tc_parameters: str
        """

    @abc.abstractmethod
    def create_bulk_testcases(self, tc_data):
        """
         Send http request to server to create a test case bulk

        :param tc_data: list of dict
        :type tc_data: list
        """

    @abc.abstractmethod
    def start_testcase(self, tc_name, tc_order=1, device_info=None, iteration=False):
        """
        Send start test case event to the server

        :param tc_name: test case name
        :type tc_name: str

        :param tc_order: the test case order
        :type tc_order: int

        :param device_info (optional): Additional device info of the dut.
        :type device_info: dict

        :param iteration (optional): True if the test case has at least two iterations and is the "parent" test case
        :type iteration: boolean
        """

    @abc.abstractmethod
    def update_testcase(self, crash_list=None, test_info=None, device_info=None, iteration=False):
        """
        Update a test case (during b2b iterations) and send to TCR

        :param tc_name: The name of the campaign test case
        :type tc_name: str

        :param iteration (optional): True if the test case has at least two iterations and is the "parent" test case
        :type iteration: boolean


        :param crash_list (optional): Current crash list
        :type crash_list: crash id list

        :param test_info (optional): Additional test case result info (can test step, external test case results, ...)
                                    (Dictionary key = section name, data = json serializable value data)
        :type test_info: dict

        :param device_info (optional): Additional device info of the dut.
        :type device_info: dict
        """

    @abc.abstractmethod
    def send_testcase_resource(self, resource, display_name=None, retention="SHORT", iteration=False):
        """
        Push a resource onto TCR REST API for a given Test Case.

        :param resource: Local resource to be pushed onto TCR at Test Case level.
        :type resource: str

        :param display_name (optional): Filename to be displayed in the UI
        :type display_name: str

        :param retention (optional): "SHORT" or "LONG"
        :type retention: str

        :param iteration (optional): True if the test case has at least two iterations
        :type iteration: bool
        """

    @abc.abstractmethod
    def stop_testcase(self, verdict, execution_nb, tc_parameters=None, tc_properties=None, tc_comments=None,
                      iteration=False):
        """
        Send stop test case event to the server

        :param verdict: tc verdict
        :type verdict: str

        :param execution_nb: Number of test execution
        :type execution_nb: int

        :param tc_parameters (optional): dict of test execution parameters
        :type tc_parameters: dict

        :param tc_properties (optional): properties of the test case (b2b iteration, retries ..)
        :type tc_properties: dict

        :param tc_comments (optional): list of test execution comments
        :type tc_comments: list

        :param iteration (optional): True if the test case has at least two iterations and is the "parent" test case
        :type iteration: boolean
        """
