"""
@summary: Test google map gps location
@since: 10/1/2014
@author: Mingmin Liu (mingminx.liu@intel.com)
"""
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.maps.maps_impl import MapsImpl

class GPSLocation(UIATestBase):
    """
    youtube used to test google maps gps location function
    """
    def setUp(self):
        super(GPSLocation, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.maps = MapsImpl()
        self.maps.set_orientation_n()

    def tearDown(self):
        super(GPSLocation, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        self.maps = None

    def testGPSLocation(self):
        """
        This test used to test GPS function.
        The test case spec is following:
        1. Launch the "google map".
        2. Check GPS lock is acquired by finding my location in Maps.
        """

        print "[RunTest]: %s" % self.__str__()
        self.maps.open_location()
        self.maps.gps_location()