import sys
import os

from testlib.util.instrumentedtestbase  import InstrumentedCtsBaseImpl
from testlib.util.repo import  ARTI_LOCAL_REPO



class OsImpl(InstrumentedCtsBaseImpl):
    """
    Instrumented Test Suite
    """
    test_pkg = 'com.android.cts.os'
    apk_repo_path =  ARTI_LOCAL_REPO

    cts_class = 'android.os.cts.PowerManager'

    def instr_run_my_cts_class(self, name):
        return self.instr_run(self.cts_class +name)

    def testOsPowerManagerWakeLock(self):
        self.instr_run_my_cts_class('_WakeLockTest')


    def testOsPowerManager(self):
        self.instr_run_my_cts_class('Test')

