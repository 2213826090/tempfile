from adb_helper.adb_utils import *
from test_utils import *


class StorageUSBTests(unittest.TestCase):
    TAG = "StorageUSBTests"

    def setUp(self):
        print self.TAG, "setUp"

    def tearDown(self):
        print self.TAG, "tearDown"

    def test_timing_spec_parameter(self):
        output = AdbUtils.run_adb_cmd("cat /d/mmc0/ios")
        print output
        lines = output.split("\n")
        for line in lines:
            if "timing spec:" in line:
                self.assertTrue("HS200" in line)
                return
        print "timing spec not found"
        self.assertTrue(False)

    def test_adb_bugreport(self):
        output = AdbUtils.run_adb_cmd("bugreport")
        print len(output)
        self.assertTrue(len(output) > 3000000)  # magic number empirically determined
        bugreport_sections = ["MEMORY INFO", "CPU INFO", "ZONEINFO", "SYSTEM LOG"]
        for section in bugreport_sections:
            self.assertTrue(section in output)

if __name__ == "__main__":
    test_result = SingleMethodRunner.run_single_test(StorageUSBTests, "test_adb_bugreport")
    print test_result.wasSuccessful()