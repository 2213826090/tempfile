import sys
import os

from testlib.util.instrumentedtestbase  import InstrumentedCtsBaseImpl


class MediaPlayerStressImpl(InstrumentedCtsBaseImpl):
    """
    Instrumented Test Suite
    """
    test_pkg = 'com.android.cts.mediastress'
    pkg_names = ['com.android.cts.mediastress']
    pkg_files = ['CtsMediaStressTestCases.apk']
    apk_repo_path = os.path.join(os.path.split(os.path.realpath(__file__))[0], '../../../support/prebuild')

    def do_H263QcifLongPlay(self):
        self.instr_run('android.mediastress.cts.H263QcifLongPlayerTest')

    def do_H263QcifShortPlay(self):
        self.instr_run('android.mediastress.cts.H263QcifShortPlayerTest')

class MediaPlayerStressExImpl(InstrumentedCtsBaseImpl):
    """
    Instrumented Test Suite, based on enhanced APK
    """
    test_pkg = 'com.android.cts.mediastressex'
    pkg_names = ['com.android.cts.mediastressex']
    pkg_files = ['CtsMediaStressTestCasesEx.apk']
    apk_repo_path = os.path.join(os.path.split(os.path.realpath(__file__))[0], '../../../support/prebuild')

    def do_scanningShortPlay(self):
        self.instr_run('android.mediastressex.cts.ScanningShortPlayerTest')