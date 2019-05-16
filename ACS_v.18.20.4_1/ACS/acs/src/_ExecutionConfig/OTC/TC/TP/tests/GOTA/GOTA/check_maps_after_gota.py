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
@summary: test maps function after gota
@since: 2/6/2015
@author: @author: Song, GuimeiX Z < guimeix.z.song@intel.com>
'''
import os
import string
import commands
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.maps.maps_impl import MapsImpl
from testlib.util.common import g_common_obj

class CheckMapsAfterGOTA(UIATestBase):

    def setUp(self):
        super(CheckMapsAfterGOTA, self).setUp()
        self._test_name = __name__
        self.gota= gotaImpl(self)
        self.maps=MapsImpl()
        self.d=g_common_obj.get_device()
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckMapsAfterGOTA, self).tearDown()

    def testCheckMapsAfterGOTA(self):
        print "[RunTest]: %s" % self.__str__()
        self.gota.unlock_screen_gota()
        self.maps.launch_from_am("googlemap")
        if self.d(text="Accept & continue").exists:
            self.d(text="Accept & continue").click()
        if self.d(text="Skip").exists:
            self.d(text="Skip").click()
        if self.d(text="Got it").exists:
            self.d(text="Got it").click()
        time.sleep(2)
        self.maps.open_location()
        self.maps.gps_location()
