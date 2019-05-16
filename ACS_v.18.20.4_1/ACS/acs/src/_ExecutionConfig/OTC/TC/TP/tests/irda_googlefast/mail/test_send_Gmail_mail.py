#Copyright (C) 2014  Lan, SamX <samx.lan@intel.com>
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
@summary: Test Mail app
@since: 09/12/2014
@author: Sam Lan (samx.lan@intel.com)
"""

import os
from testlib.util.uiatestbase import UIATestBase
from testlib.mail.mail_impl import MailImpl


class TestSendGmailMail(UIATestBase):
    """
    @summary: Test send gmail email function
    """
    def setUp(self):
        """
        @summary: set up
        @return: None
        """
        super(TestSendGmailMail, self).setUp()
        cfg_file = 'tests.tablet.google_fast.conf'
        self.mail = MailImpl()
        self.mailCfg = self.config.read(cfg_file, 'gmail_mail')

    def tearDown(self):
        """
        @summary: tear tearDown
        @return: None
        """
        super(TestSendGmailMail, self).tearDown()
        self.contacts = None

    def testSendGmailMail(self):
        """
        This test used to test contacts app

        The test case spec is following:
        1. launch mail app
        2. send Gmail Mail
        """
        sender_account=self.mailCfg.get("sender_account")
        receiver_account=self.mailCfg.get("receiver_account")
        subject=self.mailCfg.get("subject")
        body=self.mailCfg.get("body")
        attachment_file_name=self.mailCfg.get("attachment")
        
        self.mail.launch_by_am()
        self.mail.select_account(sender_account)
        self.mail.compose_a_new_email(receiver_account, subject, body)
        #attachment_file_name = "aaa.png"
        #base_path = os.path.split(os.path.realpath(__file__))[0]
        base_path="/sdcard/Download/"
        self.mail.screenshot_and_attach(base_path, attachment_file_name)
        self.mail.send_email()
        self.mail.delete_screenshot_file(base_path, attachment_file_name)
