"""
@summary: Test share youtube video
@since: 10/1/2014
@author: Mingmin Liu (mingminx.liu@intel.com)
"""
import os
from testlib.util.uiatestbase import UIATestBase
from testlib.keep.keep_impl import KeepImpl

class CreateaNote(UIATestBase):
    """
    @summary: create a note in keep app function
    """

    def setUp(self):
        super(CreateaNote, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.keep = KeepImpl()
        self.keep.set_orientation_n()

    def tearDown(self):
        super(CreateaNote, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        self.keep = None

    def testCreateaNote(self):
        """
        This test used to test create a note function.
        The test case spec is following:
        1. Launch the "keep" and add a images.
        2. save file then delete archive notes.
        """

        print "[RunTest]: %s" % self.__str__()
        self.keep.create_a_note()


