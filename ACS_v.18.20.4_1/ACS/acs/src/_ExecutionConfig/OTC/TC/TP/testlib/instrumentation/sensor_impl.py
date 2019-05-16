import sys
import os

from testlib.util.repo import Artifactory
from testlib.util.uiatestbase import UIATestBase
from testlib.util.instrumentedtestbase  import InstrumentedCtsBaseImpl
from testlib.util.repo import  ARTI_LOCAL_REPO



class HardwareImpl(InstrumentedCtsBaseImpl):
    """
    Instrumented Test Suite
    """
    test_pkg = 'com.android.cts.hardware'
    apk_repo_path =  ARTI_LOCAL_REPO

    cts_class = 'android.hardware.cts.SensorTest'

    def instr_run_my_cts_class(self, name):
        return self.instr_run(self.cts_class + '#'+name)

    def testValuesForAllSensors(self):
        self.instr_run_my_cts_class('testValuesForAllSensors')


    def testSensorOperations(self):
        self.instr_run_my_cts_class('testSensorOperations')

