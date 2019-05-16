from testlib.util.instrumentedtestbase import InstrumentedCtsBaseImpl


class LocationImpl(InstrumentedCtsBaseImpl):
    """
    Instrumented Test Suite
    """
    test_pkg = 'com.android.cts.location'

    class_prefix = 'android.location.cts'

    def instr_run_cts_class(self, classname):
        return self.instr_run(self.class_prefix + '.' + classname)
