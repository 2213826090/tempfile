# -*- coding: utf-8 -*-
'''
@since: 06/24/2015
@author: ZhangroX
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.clock_impl import ClockImpl

class Clock(UIATestBase):

    def setUp(self):
        super(Clock, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._clock = ClockImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(Clock, self).tearDown()

    def test_AppTransparency_Clock(self):
        ''' refer TC test_AppTransparency_Clock
        '''
        print "[RunTest]: %s" % self.__str__()
        self._clock.launch_clock_am()
        self._clock.check_imge_and_graphics()
        self._clock.stop_clock_am()

