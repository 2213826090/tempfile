from .common import BatTestBase


class TestGraphic(BatTestBase):
    '''
    Graphic Test Class
    '''

    def testActivityBrightnessChange(self):
        self.bat.instr_run_class('GraphicTest#testActivityBrightnessChange')

    def testSystemBrightnessChange(self):
        self.bat.instr_run_class('GraphicTest#testSystemBrightnessChange')
