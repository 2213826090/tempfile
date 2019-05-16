#Copyright (C) 2015 haley.han
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
@summary: test suspend resume during gota
@since: 27/4/2015
@author: haley.han(yi.a.han@intel.com)
'''
import os
import string
import commands
from testlib.util.uiatestbase import UIATestBase
from testlib.gota.gota_impl import gotaImpl
from testlib.util.common import g_common_obj
from testlib.systemui.systemui_impl import SystemUI
from testlib.system.system_impl import SystemImpl
from testlib.common.common import g_common_obj2
class SuspendResumeDuringGOTA(UIATestBase):

    def setUp(self):
        super(SuspendResumeDuringGOTA, self).setUp()
        cfg_file = os.path.join(os.environ.get('TEST_DATA_ROOT', ''), \
            'tests.tablet.gota.conf')
        self._test_name = __name__
        self.cfg = self.config.read(cfg_file, 'gota')
        self.gota= gotaImpl(self.cfg)
        self.system=SystemImpl(self.cfg)
	self.serial=g_common_obj2.getSerialNumber()
        print "[Setup]: %s" % self._test_name
    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(SuspendResumeDuringGOTA, self).tearDown()
        self.cfg = None
    def testSuspendResumeDuringGOTA(self):
        print "[RunTest]: %s" % self.__str__()
	#need common function to control cutter //xue shu qiang
	before_cutter_str=os.popen("adb -s %s shell cat /d/pmc_atom/sleep_state |grep Idle" %self.serial).read()
	fplist=before_cutter_str.split()
	before_cutter=float(fplist[1].encode('utf-8'))
	print "before %s" %before_cutter
	#cutteroff
	#time.sleep(300)
	#cutteron
	after_cutter_str=os.popen(" adb -s %s shell cat /d/pmc_atom/sleep_state |grep Idle" %self.serial).read()
	fplist=after_cutter_str.split()
	after_cutter=float(fplist[1].encode('utf-8'))
	print "before %s" %after_cutter
	assert before_cutter<after_cutter
