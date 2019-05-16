import unittest
from testlib.em.settings import SecuritySetting
from constants_def import *

class SetSleepLock(unittest.TestCase):

    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.security = SecuritySetting()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name

    def testSetSleepLock(self):
        print "[RunTest]: %s" % self.__str__()
        for i in range(3):
            try:
                product = self.get_product()
                if product in [BXT_M, BXT_O]:
                    self.change_lock_screen_status("None")
                else:
                    self.change_lock_screen_status("Swipe")
                break
            except Exception as e:
                print e.message
        else:
            assert succeed

