import os
import time
from testlib.util.uiatestbase import UIATestBase
from testlib.util.repo import Artifactory
from testlib.util.config import TestConfig
from testlib.util.common import g_common_obj
from testlib.graphics.glbenckmark_impl import GLBenchmarkImpl
from testlib.graphics.extend_glbenckmark_impl import GLBenchmarkExtendImpl
from acs_test_scripts.Lib.hypervisor.hypervisor import DebugCard

ARTIFACTORY_URL = TestConfig().getConfValue(section='artifactory',
                                            key='location')
GRAPHICS_REPO_URL = os.path.join(ARTIFACTORY_URL, 'Graphics')


def get_resource(f):
    arti = Artifactory(GRAPHICS_REPO_URL)
    return arti.get(f)


class AaaGTest(UIATestBase):
    @classmethod
    def setUpClass(self):
        apk = get_resource("GLBenchmark_2_7_0.apk")
        g_common_obj.adb_cmd_common('install -r ' + apk, 300)

    def setUp(self):
        super(AaaGTest, self).setUp()
        self.debug_card = DebugCard()
        self.debug_card.open()
        self._glBenchmark = GLBenchmarkImpl()
        self.benchmark = GLBenchmarkExtendImpl()

    def tearDown(self):
        super(AaaGTest, self).tearDown()
        self._glBenchmark.stop_app_am()
        self.benchmark.clean()
        self.debug_card.close()

    def get_dom0_ssh(self):
        return self.debug_card.get_ssh()

    def test_stress_wayland_tool_GFXBench(self):
        # launch and run GL_Benchmark in DomU
        self.benchmark.launch()

        # download and copy wayland stress tool to Dom0
        fname = "stress_wayland.tar.gz"
        stress_tool_pkg = get_resource(fname)
        dom0_ssh = self.get_dom0_ssh()
        self.assertIsNotNone(dom0_ssh, 'Failed to get dom0 ssh client.')
        dom0_ssh.run_cmd('mkdir -p /test')
        dom0_ssh.push(stress_tool_pkg, '/test/' + fname)
        dom0_ssh.run_cmd('cd /test/; tar -xf ' + fname)

        # run wayland stress tool
        cmds = [
            'cd /test/stress_wayland',
            './gpuhighpri2 params/dials.160',
        ]
        proc = dom0_ssh.run_cmd_async(' && '.join(cmds))

        def get_latest_fps():
            if proc.stdout:
                line = proc.stdout.splitlines()[-1]
                print line
                return float(line.split()[-2])

        self.benchmark.run_performance_test_async("test5")
        try:
            for _ in range(6):
                assert get_latest_fps() > 50 # check fps in Dom0
                time.sleep(5)
        finally:
            proc.kill()  # kill and cleanup
