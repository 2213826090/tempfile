from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.templerun.templerun_impl import TempleRunImpl
import time


class TempleRun(RenderAppTestBase):

    def setUp(self):
        super(TempleRun, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        cfg_file = 'tests.tablet.templerun.conf'
        self.timeout = self.config.read(cfg_file, 'param').get("timeout")

        self.templerun = TempleRunImpl()
        self.templerun.close_wifi()
        self.templerun.install_templerun_app(self.config.read(cfg_file, 'artifactory'))

    def tearDown(self):
        self.templerun.stop_playing()
        super(TempleRun, self).tearDown()
        print "[Teardown]: %s" % self._test_name
        self.templerun.open_wifi()

    def testTempleRun(self):
        """
        Play 3D game temple run 2 2 hours
        """
        print "[RunTest]: %s" % self.__str__()
        self.templerun.launch_templerun()
        time.sleep(30)
        try:
            self.templerun.press_main_menu()
            self.templerun.start_play()
            time.sleep(10)
        except:
            raise Exception("Fail to play templerun")
