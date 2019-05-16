import sys
import os

from testlib.util.instrumentedtestbase import InstrumentedCtsBaseImpl
from testlib.util.repo import  ARTI_LOCAL_REPO



class GraphicImpl(InstrumentedCtsBaseImpl):
    """
    Instrumented Test Suite
    """
    test_pkg = 'com.android.cts.graphics2'
    apk_repo_path =  ARTI_LOCAL_REPO

    cts_class = 'android.graphics2.cts.TextureViewTest'

    def instr_run_my_cts_class(self):
        return self.instr_run(self.cts_class)

    """
    graphics2 TextureViewTest
    """
    def testTextureView(self):
        self.instr_run_my_cts_class()
