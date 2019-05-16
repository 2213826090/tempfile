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
@summary: Test browser
@since: 09/20/2014
@author: Grace Yi (gracex.yi@intel.com)
"""

import os
from testlib.browser.browser_impl import BrowserImpl
from testlib.util.uiatestbase import UIATestBase

class SwitchTabandSaveBookmarkTest(UIATestBase):
    """
    @summary: Test switch tab and bookmark
    """
    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(SwitchTabandSaveBookmarkTest, self).setUp()
        cfg_file = 'tests.tablet.google_fast.conf'
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.browser = BrowserImpl(self.config.read(cfg_file, 'browser'))
        self.browser.browser_setup()
        self.browser.delete_all_bookmark()

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """

        print "[Teardown]: %s" % self._test_name
        self.browser.delete_all_bookmark()
        self.browser.browser_setup()
        super(SwitchTabandSaveBookmarkTest, self).tearDown()
        self.browser = None

    def testSwitchTabandSaveBookmark(self):
        """
            switch tab and save bookmark
        """
        print "[RunTest]: %s" % self.__str__()
        web1 = self.browser.cfg.get("web1")
        web1check = self.browser.cfg.get("web1check")
        web2 = self.browser.cfg.get("web2")
        web2check = self.browser.cfg.get("web2check")
        bookmarktitle = self.browser.cfg.get("bookmarktitle")
        switchcount = self.browser.cfg.get("switchcount")

        self.browser.switch_tab(web1, web1check, web2, web2check, switchcount)
        self.browser.add_bookmark_and_check(bookmarktitle)
