from testlib.util.instrumentedtestbase import InstrumentedBaseImpl


class BatImpl(InstrumentedBaseImpl):
    test_pkg = 'com.intel.tests.bat'

    class_prefix = 'com.intel.tests.bat'

    def instr_run_class(self, classname):
        return self.instr_run(self.class_prefix + '.' + classname)
