#Copyright (C) 2014  Yi, GraceX <gracex.yi@intel.com>
#Intel Corporation All Rights Reserved.

#The source code contained or described herein and
#all documents related to the source code ("Material") are owned by
#Intel Corporation or its suppliers or licensors.

#Title to the Material remains with Intel Corporation or
#its suppliers and licensors.
#The Material contains trade secrets and proprietary and
#confidential information of Intel or its suppliers and licensors.
#The Material is protected by worldwide copyright and
#trade secret laws and treaty provisions.
#No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
#or disclosed in any way without Intel's prior express written permission.
#No license under any patent, copyright, trade secret or
#other intellectual property right is granted to
#or conferred upon you by disclosure or delivery of the Materials,
#either expressly, by implication, inducement, estoppel or otherwise.

#Any license under such intellectual property rights must be express
#and approved by Intel in writing.

"""
@summary: Test google play application
@since: 09/15/2014
@author: Grace Yi (gracex.yi@intel.com)
"""

from testlib.util.common import g_common_obj
from testlib.contacts.contacts_impl import ContactsImpl
from testlib.browser.browser_impl import BrowserImpl
from testlib.google_search.search_impl import SearchImpl
from testlib.util.uiatestbase import UIATestBase
import os

class SeachLowBatteryTest(UIATestBase):
    """
    @summary: Test google play application
    """
    def setUp(self):
        cfg_file = 'tests.tablet.google_fast.conf'
        super(SeachLowBatteryTest, self).setUp()
        self._test_name = __name__
        self.search = SearchImpl(\
            self.config.read(cfg_file, 'google_search'))
        self.browser = BrowserImpl(self.config.read(cfg_file, 'browser'))
        self.contacts = ContactsImpl(self.config.read(cfg_file, 'contacts'))
        print "[Setup]: %s" % self._test_name
        self.__search_setup()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.__search_teardown()
        super(SeachLowBatteryTest, self).tearDown()

    def testSearchLowBattery(self):
        """
        This test case is to test search while battery low status

        Test Case Precondition:
        1. DUT at battery low status

        Test Case Step:
        1.Launch Search.Verify that users can access to Search,
        2.Input some search keyword to search.Verify that the searched
        result with keyword can be show

        Expect Result:
        1.Verify that users can access to Search,
        2.Verify that the searched result with keyword can be shown.

        """
        print "[RunTest]: %s" % self.__str__()

        search_key = self.search.cfg.get("search_key")
        check_app = self.search.cfg.get("check_app")
        check_web = self.search.cfg.get("check_web")
        check_contact = self.search.cfg.get("check_contact")

        self.search.search(search_key)
        self.search.check_search(check_app, check_web, check_contact)

    def __search_setup(self):
        """
        @summary: setup search function
        """

        website = self.search.cfg.get("check_web")
        webcheck = self.search.cfg.get("check_webcheck")
        contact = self.search.cfg.get("check_contact")
        #check battery
        low_battery_level = self.search.cfg.get("low_battery_level")
        self.search.check_low_battery(low_battery_level)

        if website != "" and webcheck != "":
            self.browser.browser_setup()
            self.browser.open_website(website)
            self.browser.web_check(webcheck)
        if contact != "":
            self.contacts.contact_add(contact)
        g_common_obj.back_home()

    def __search_teardown(self):
        """
        @summary: clear data
        """
        self.search.stop_from_am()
        self.browser.clear_data()