from .common import BatTestBase


class TestSensor(BatTestBase):
    '''
    Sensor Test Class
    '''
    classname = 'SensorTest'

    def __run_function(self, func):
        self.bat.instr_run_class(self.classname + '#' + func)

    def testAccelerometerSensor(self):
        self.__run_function('testAccelerometerSensor')

    def testCompassSensor(self):
        self.__run_function('testCompassSensor')

    def testGyroscopeSensor(self):
        self.__run_function('testGyroscopeSensor')

    def testAlsSensor(self):
        self.__run_function('testAlsSensor')
