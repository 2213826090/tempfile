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

:organization: INTEL SSG OTC
:summary: Implements automatic Campaign generator from failed tests extracted
from results folder provided as input to ACS
:author: mceausu
"""
import os
from lxml import etree
import UtilitiesFWK.Utilities as Util
import Core.Report.XMLUtilities as XMLUtil
from Core.PathManager import Paths


class FailedTestCampaignGenerator(object):

    def __init__(self, failed_folder_path=""):
        """
        This is the Class Constructor
        :param failed_folder_path: string
        :param failed_folder_path: Report folder from which we want to generate
        a Test Campaign from its failed tests
        """
        self.failed_folder_path = failed_folder_path
        self.report_file = None
        self.input_campaign_name = None
        self.output_campaign_name = None
        self.failed_tc_names = []
        self.all_tc_name = []

    def find_report_file(self):
        """
        This function will search for the ACS Report XML file inside ACS Report
        folder provided as input

        :rtype: string
        :return: Report file found if found in input Report folder
        """
        for root, dirnames, filenames in os.walk(self.failed_folder_path):
            for filename in filenames:
                if filename.endswith(".xml"):
                    filename = os.path.join(root, filename)
                    if Util.is_report_file(filename):
                        return filename
        return ""

    def get_campaign_name(self):
        """
        :rtype: string
        :return: Original test campaign name that was executed to obtain this
                Report folder
        """
        tree = etree.parse(self.report_file)
        camp_name = tree.find("CampaignInfo/CampaignName")
        campaign_name = os.path.join(Paths.EXECUTION_CONFIG, camp_name.get("relative_path"), camp_name.text)
        # compute output Campaign Name
        self.output_campaign_name = os.path.join(Paths.EXECUTION_CONFIG, camp_name.text) + "_FAILED_TESTS.xml"
        return campaign_name + ".xml"

    def get_failed_tc(self):
        """
        This function will create a list from the failed tests inside the ACS test report
        """
        tree = etree.parse(self.report_file)
        root = tree.getroot()
        for test_case in root.findall('TestCase'):
            tc_name = os.path.join(test_case.get('relative_path'), test_case.get('id'))
            if test_case.find('Verdict').text not in Util.Verdict.PASS:
                self.failed_tc_names.append(tc_name)
            self.all_tc_name.append(tc_name)

    def create_new_campaign(self):
        """
        This function will generate the new test campaign based on failed tests
        from input ACS results folder
        """
        # extract campaign header: Parameters and Targets
        tree = etree.parse(self.input_campaign_name)
        root = tree.getroot()
        comment = etree.Comment(
            "This Test Campaign was AUTOMATICALLY generated from input ACS report file: %s" % self.failed_folder_path)
        root.insert(0, comment)
        # remove all test cases
        test_cases = root.findall('TestCases')[0]
        test_cases.clear()
        # replace them with the ones in the Test Report
        children = []
        for element in self.all_tc_name:
            if element not in self.failed_tc_names:
                children.append(etree.Comment(etree.tostring(etree.Element('TestCase', Id=element))))
            else:
                children.append(etree.Element('TestCase', Id=element))
        test_cases.extend(children)
        tree.write(self.output_campaign_name)
        XMLUtil.pretty_print_xml(self.output_campaign_name)
        print "Output Failed Tests Campaign can be found here: ", self.output_campaign_name

    def create_test_campaign(self):
        """
        This function implements the main logic of Campaign Generator
        """
        # extract Test Report File
        self.report_file = self.find_report_file()
        if not self.report_file:
            return (Util.Global.FAILURE, "No valid Test Report found in: %s" % self.failed_folder_path)
        print "Input Test Report File Found: %s" % self.report_file

        # extract input Campaign name from ACS Test Report
        self.input_campaign_name = self.get_campaign_name()
        if not os.path.isfile(self.input_campaign_name):
            return (Util.Global.FAILURE, "Targeted Test Campaign is not a valid file path: %s" % self.input_campaign_name)
        print "Input Test Campaign Found: %s" % self.input_campaign_name

        # extract tests from ACS Test Report
        self.get_failed_tc()
        if len(self.failed_tc_names) == 0:
            return (Util.Global.FAILURE, "This Test Report folder contains no failed tests")

        # generate ACS test campaign for the failed tests
        self.create_new_campaign()

        return Util.Global.SUCCESS, ""
