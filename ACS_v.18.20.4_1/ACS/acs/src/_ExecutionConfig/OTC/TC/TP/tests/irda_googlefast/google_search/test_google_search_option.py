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
@summary: Test google search application
@since: 08/21/2014
@author: Grace Yi (gracex.yi@intel.com)
"""

from testlib.util.common import g_common_obj
from testlib.contacts.contacts_impl import ContactsImpl
from testlib.browser.browser_impl import BrowserImpl
from testlib.google_search.search_impl import SearchImpl
from testlib.util.uiatestbase import UIATestBase
import os

class SeachTest(UIATestBase):
    """
    @summary: Test google search application
    """
    def setUp(self):
        cfg_file = 'tests.tablet.google_fast.conf'
        super(SeachTest, self).setUp()
        self._test_name = __name__
        self.search = SearchImpl(\
            self.config.read(cfg_file, 'google_search'))
        self.browser = BrowserImpl(self.config.read(cfg_file, 'browser'))
        self.contacts = ContactsImpl(self.config.read(cfg_file, 'wifisetting'))
        print "[Setup]: %s" % self._test_name
        self.__search_setup()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.__search_teardown()
        super(SeachTest, self).tearDown()

    def testSearchOption(self):
        """
        This test case is to test search app

        Test Case Step:
        1.Google Now: Verify user can opt in in Google Now and Cards are displayed

        Expect Result:
        1.Verify that users can access to Search,
        2.Verify that the searched result with keyword can be shown.
        3.The search result can link to application

        """
        print "[RunTest]: %s" % self.__str__()

        app_key = self.search.cfg.get("app_key")
        check_app = self.search.cfg.get("check_app")
        web_key = self.search.cfg.get("web_key")
        check_web = self.search.cfg.get("check_web")
        contact_key = self.search.cfg.get("contact_key")
        check_contact = self.search.cfg.get("check_contact")
        app_package = self.search.cfg.get("app_package")
        web_package = self.search.cfg.get("web_package")
        contact_package = self.search.cfg.get("contact_package")

        self.search.search(app_key)
        self.search.check_search(check_app)
        self.search.launch_app_from_search(check_app, app_package)
        self.search.search(web_key)
        self.search.check_search(check_web)
        self.search.launch_app_from_search(check_web, web_package)
        self.search.search(contact_key)
        self.search.check_search(check_contact)
        self.search.launch_app_from_search(check_contact, contact_package)


    def __search_setup(self):
        """
        @summary: setup search function
        """

        website = self.search.cfg.get("check_web")
        webcheck = self.search.cfg.get("check_webcheck")
        contact = self.search.cfg.get("check_contact")

        if website != "" and webcheck != "":
            self.browser.browser_setup()
            self.browser.open_website(website)
            self.browser.web_check(webcheck)
        if contact != "":
            self.contacts.contact_add(contact)
        #self.wifi.launch_from_am()
        #self.wifi.turn_off_wifi()
        g_common_obj.back_home()

    def __search_teardown(self):
        """
        @summary: clear data
        """
        self.search.stop_from_am()
        self.browser.clear_data()
