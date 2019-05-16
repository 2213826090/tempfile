from testlib.util.instrumentedtestbase import InstrumentedBaseImpl


class GraphicsImpl(InstrumentedBaseImpl):
    inst_runner = 'android.support.test.runner.AndroidJUnitRunner'

    test_pkg = 'com.android.graphicstest.image'

    class_prefix = 'android.graphicstest.image'

    def instr_run_class(self, classname):
        return self.instr_run(self.class_prefix + '.' + classname)

class ColorModeImpl(InstrumentedBaseImpl):

    inst_runner = 'android.support.test.runner.AndroidJUnitRunner'
    test_pkg = 'android.colormode.cts'
    class_prefix = 'android.colormode.cts'

    def instr_run_class(self, case_name):
        return self.instr_run(self.class_prefix + '.' + case_name)
