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
@summary: test charging function after gota
@since: 3/6/2015
@author: Song, GuimeiX Z < guimeix.z.song@intel.com>
'''
import os
import string
import commands
from testlib.util.uiatestbase import UIATestBase
from testlib.systemui.systemui_impl import SystemUI
from testlib.common.common import g_common_obj2

class CheckChargingAfterGOTA(UIATestBase):

    def setUp(self):
        super(CheckChargingAfterGOTA, self).setUp()
        self._test_name = __name__
        self.serial=g_common_obj2.getSerialNumber()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckChargingAfterGOTA, self).tearDown()

    def testCheckChargingAfterGOTA(self):
        print "[RunTest]: %s" % self.__str__()
        charging_status=os.popen("adb -s %s shell cat /sys/class/power_supply/dollar_cove_charger/uevent |grep 'POWER_SUPPLY_PRESENT'" %self.serial).read()
        print "%s" %charging_status
        assert not -1==charging_status.find("1")
