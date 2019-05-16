import sys
import os

from testlib.util.instrumentedtestbase  import InstrumentedCtsBaseImpl
from testlib.util.repo import  ARTI_LOCAL_REPO



class BluetoothImpl(InstrumentedCtsBaseImpl):
    """
    Instrumented Test Suite
    """
    test_pkg = 'com.android.cts.bluetooth'
    apk_repo_path =  ARTI_LOCAL_REPO

    cts_class = 'android.bluetooth.cts.BasicAdapterTest'

    def instr_run_my_cts_class(self):
        return self.instr_run(self.cts_class)

    def testBasicAdapter(self):
        self.instr_run_my_cts_class()
