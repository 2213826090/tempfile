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
@summary: test calculator function after gota
@since: 26/5/2015
@author: Song, GuimeiX Z < guimeix.z.song@intel.com>
'''
import os
import string
import commands
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.domains.calculator_impl import CalculatorImpl
from testlib.gota.gota_impl import gotaImpl
class CheckCalculatorAfterGOTA(UIATestBase):

    def setUp(self):
        super(CheckCalculatorAfterGOTA, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.gota.conf')
        self._test_name = __name__
        self.cfg = self.config.read(cfg_file, 'gota')
        self.d = g_common_obj.get_device()
        self.calculator=CalculatorImpl()
        self.gota= gotaImpl(self.cfg)
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(CheckCalculatorAfterGOTA, self).tearDown()

    def testCheckCalculatorAfterGOTA(self):
        print "[RunTest]: %s" % self.__str__()
        self.gota.launch_calculator()
        self.calculator.input_number()
        self.d(text="del").long_click()
        self.d(text="2").click()
        self.d(text="+").click()
        self.d(text="3").click()
        self.d(text="=").click()
        assert self.d(text="5").exists