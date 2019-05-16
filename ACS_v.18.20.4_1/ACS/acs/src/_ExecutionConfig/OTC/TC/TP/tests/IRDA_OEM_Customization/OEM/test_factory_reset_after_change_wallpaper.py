import os
import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase
from testlib.oem.oem_impl import OEMImpl


class TestResetAfterChange(UIATestBase):
    """
    Testing wifi enable
    """
    def setUp(self):
        super(TestResetAfterChange, self).setUp()
        cfg_file = os.path.join(os.environ.get(
            'TEST_DATA_ROOT', ''), 'tests.tablet.oem.conf')
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self.oem = OEMImpl(self.config.read(cfg_file, 'default'))
        self.oem.setup_connection()
        self.oem.set_orientation_n()
        self.oem.wake_up()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(TestResetAfterChange, self).tearDown()
        self.wifi = None

    def testResetAfterChange(self):
        """
        Factory reset after Change OEM wallpaper to another
        """
        print "[RunTest] %s" % self.__str__()

        self.oem.launch_wallpaper()
        self.oem.change_wallpaper(5)

        self.oem.launch_wallpaper()
        self.oem.change_wallpaper(3, "start.png")
        self.oem.reset_device()
        self.oem.take_picture("end.png")
        print"Compare the picture"
        base_path = self.oem.host_file_path + '/file/' + "start.png"
        new_path = self.oem.host_file_path + '/file/' + "end.png"
        print base_path
        from igascomparator import igascomparator
        comp = igascomparator()
        rate = comp.getsimilarityrate(new_path, base_path)
        print "Rate: %s" % rate
        print "Expect that pictures are not similar"
        assert (rate < 0.95)
        os.remove(base_path)
        os.remove(new_path)
