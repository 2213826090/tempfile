# -*- coding: utf-8 -*-
'''
@since: 06/25/2015
@author: ZhangroX
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.adobeAir_impl import AdobeAirImpl

class AdobeAir(UIATestBase):

    def setUp(self):
        super(AdobeAir, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._adobe = AdobeAirImpl()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(AdobeAir, self).tearDown()

    def test_AdobeAirEnvironment(self):
        ''' refer TC test_AdobeAirEnvironment
        '''
        print "[RunTest]: %s" % self.__str__()
        self._adobe.install_adobeAir()
        self._adobe.check_adobeAir_in_All_Apps()
        self._adobe.launch_adobeAir_am()
        self._adobe.stop_adobeAir_am()
