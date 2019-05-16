"""
@summary: Test share youtube video
@since: 10/1/2014
@author: Mingmin Liu (mingminx.liu@intel.com)
"""
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.google_plus.google_plus_impl import GooglePlusImpl

class GooglePlus(UIATestBase):
    """
    @summary: youtube used to test share video via gmail function
    """
    def setUp(self):
        super(GooglePlus, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.googleplus = GooglePlusImpl()
        self.googleplus.set_orientation_n()

    def tearDown(self):
        super(GooglePlus, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        self.googleplus = None

    def testGooglePlus(self):
        """
        This test used to test google plus function.
        The test case spec is following:
        1. Launch the "google+" .
        """
        print "[RunTest]: %s" % self.__str__()
        self.googleplus.google_plus_view()
