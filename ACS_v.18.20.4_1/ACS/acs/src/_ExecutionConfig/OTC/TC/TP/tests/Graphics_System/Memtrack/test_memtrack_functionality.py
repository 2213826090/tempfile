# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 02/05/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.memtrack_impl import MemTrackImpl

class MemTrack(UIATestBase):

    def setUp(self):
        super(MemTrack, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._memtrack = MemTrackImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(MemTrack, self).tearDown()

    def test_memtrack_functionality(self):
        ''' refer TC test_Memtrack_functionality
        '''
        self._memtrack.check_graphics_mem()