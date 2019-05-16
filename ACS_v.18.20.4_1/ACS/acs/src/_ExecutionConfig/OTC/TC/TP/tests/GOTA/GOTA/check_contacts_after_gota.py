#Copyright (C) 2015 Song, GuimeiX Z
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

'''
@summary: test contacts after gota
@since: 27/5/2015
@author: Song, GuimeiX Z < guimeix.z.song@intel.com>
'''
import os
import string
import commands
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.system.system_impl import SystemImpl
from testlib.contacts.contacts_impl import ContactsImpl


class CheckContactsAfterGOTA(UIATestBase):

    def setUp(self):
        super(CheckContactsAfterGOTA, self).setUp()
        self._test_name = __name__
        self.gota= gotaImpl(self)
        self.contacts=ContactsImpl()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckContactsAfterGOTA, self).tearDown()
        self.cfg = None

    def testCheckContactsAfterGOTA(self):
        print "[RunTest]: %s" % self.__str__()
        self.gota.unlock_screen_gota()
        self.contacts.launch_by_am()
        self.contacts.add_a_new_contact("newcontact", account=None)