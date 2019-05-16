# coding: UTF-8
import os
import time
from testlib.util.common import g_common_obj

class BenchmarkImpl:
    """
    Implements Settings app UI actions.
    """

    def __init__ (self, cfg = None):
        self.d = g_common_obj.get_device()

    def launch_basemark(self):
        '''
        Launch Basemark app.
        '''
        print "[Info] ---Launch Basemark app."
        os.system("adb shell am start -S com.rightware.tdmm2v10jnifree/.SplashView")
        for i in range(10):
            if self.d(text="Run Benchmark").exists:
                break
            time.sleep(1)
        assert self.d(text="Run Benchmark").exists

    def run_benchmark(self):
        '''
        Run benchmark.
        '''
        print "[Info] ---Run benchmark."
        self.d(text="Run Benchmark").click.wait()
        time.sleep(1)
        assert not self.d(text="Run Benchmark").exists

    def get_score(self):
        '''
        Get benchmark score.
        '''
        print "[Info] ---Get benchmark score."
        assert self.d(resourceId="com.rightware.tdmm2v10jnifree:id/compare_online").exists

