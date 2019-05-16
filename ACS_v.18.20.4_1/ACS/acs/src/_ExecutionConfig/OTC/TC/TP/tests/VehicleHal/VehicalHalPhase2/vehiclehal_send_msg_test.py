# coding: utf-8
"""
@summary: Vehicle Hal Send Message test class
@since: 12/05/2017
@author: Jinliang Wang (jinliang.wang@intel.com)
"""
import time

from testlib.util.common import g_common_obj
from testlib.vehicle.vehicle_test_base import VehicleTestBase
from testlib.multimedia.multimedia_canbox_helper import MultiMediaCanboxHelper
from testlib.util.log import Logger
g_logger = Logger.getlogger()

class VehicleHalSendMsgSensorTest(VehicleTestBase):
    """
    @summary: this test used to test vehicle hal sensors
    """

    __sensor_key = {
        "night": "isNight", # Night Mode
        "fuelLevel": "isFuelLevelLow", # Fuel Level
        "signalState": "state", # Turn signal state
        "rpm": "rpm", # Engine RPM
        "speed": "speed", # Speed
        "oilTemp": "temperature" # Engine Oil Temperature
    }

    @classmethod
    def setUpClass(cls):
        super(VehicleHalSendMsgSensorTest, cls).setUpClass()
        cls.canbox_helper = MultiMediaCanboxHelper()

    @classmethod
    def tearDownClass(cls):
        super(VehicleHalSendMsgSensorTest, cls).tearDownClass()

    def setUp(self):
        super(VehicleHalSendMsgSensorTest, self).setUp()
        self.launch_kitchensink_sensors()
        time.sleep(2)

    def tearDown(self):
        self.close_kitchensink_app()
        super(VehicleHalSendMsgSensorTest, self).tearDown()

    def get_sensor_info(self, sensor):
        import xml.etree.ElementTree as ET
        value = None
        sensors_string = self.d.dump()
        if not sensors_string:
            return None
        root = ET.fromstring(sensors_string)
        childs = root.findall('.//node[@resource-id="com.google.android.car.kitchensink:id/sensor_info"]')
        if childs:
            sensor_info = map(lambda ele: ele.attrib['text'].strip(), childs)
            g_logger.info("Sensor info: %s" % sensor_info)
            if sensor in self.__sensor_key.keys():
                key_str = self.__sensor_key[sensor] + '='
                g_logger.debug("sensor key string is %s" % key_str)
                value = sensor_info[0][sensor_info[0].find(key_str) + len(key_str):].split('\n')[0]
                g_logger.debug("sensor is %s" % sensor)
                g_logger.debug("value is %s" % value)
        return value

    def check_night_mode(self):
        self.canbox_helper.cansend("00A#01") # turn on night mode
        time.sleep(0.5)
        self.assertEqual(self.get_sensor_info("night"), "true")
        self.canbox_helper.cansend("00A#00") # turn off night mode
        time.sleep(0.5)
        self.assertEqual(self.get_sensor_info("night"), "false")

    def check_fuel_level(self):
        self.canbox_helper.cansend("00B#01") # change fuel level to low
        time.sleep(0.5)
        self.assertEqual(self.get_sensor_info("fuelLevel"), "true")
        self.canbox_helper.cansend("00B#00") # change fuel level to not low
        time.sleep(0.5)
        self.assertEqual(self.get_sensor_info("fuelLevel"), "false")

    def check_signal_state(self):
        states = [0, 1, 2, 4]
        self.canbox_helper.cansend("00C#00") # turn signal state 
        time.sleep(1)
        state1 = int(self.get_sensor_info("signalState"))
        self.assertTrue(state1 in states)
        self.canbox_helper.cansend("00C#00") # turn signal state
        time.sleep(1)
        state2 = int(self.get_sensor_info("signalState"))
        self.assertTrue(state2 in states)
        self.assertNotEqual(state2, state1)

    def check_rpm(self):
        self.canbox_helper.cansend("00D#00") # select RPM mode
        self.canbox_helper.cansend("011#00") # set the initial value
        time.sleep(0.5)
        rpm0 = float(self.get_sensor_info("rpm"))
        self.canbox_helper.cansend("011#01") # raise RPM, like #01, #02, #03, ...
        time.sleep(0.5)
        rpm1 = float(self.get_sensor_info("rpm"))
        self.assertGreater(rpm1, rpm0)
        self.canbox_helper.cansend("011#02") # raise RPM, like #01, #02, #03, ...
        time.sleep(0.5)
        rpm2 = float(self.get_sensor_info("rpm"))
        self.assertGreater(rpm2, rpm1)
        self.canbox_helper.cansend("011#01") # drop RPM, like #03, #02, #01 ...
        time.sleep(0.5)
        rpm3 = float(self.get_sensor_info("rpm"))
        self.assertLess(rpm3, rpm2)

    def check_speed(self):
        self.canbox_helper.cansend("00E#00") # select Speed mode
        self.canbox_helper.cansend("011#00") # set the initial value
        time.sleep(0.5)
        speed0 = float(self.get_sensor_info("speed"))
        self.canbox_helper.cansend("011#01") # raise speed, like #01, #02, #03, ...
        time.sleep(0.5)
        speed1 = float(self.get_sensor_info("speed"))
        self.assertGreater(speed1, speed0)
        self.canbox_helper.cansend("011#02") # raise speed, like #01, #02, #03, ...
        time.sleep(0.5)
        speed2 = float(self.get_sensor_info("speed"))
        self.assertGreater(speed2, speed1)
        self.canbox_helper.cansend("011#01") # drop speed, like #03, #02, #01 ...
        time.sleep(0.5)
        speed3 = float(self.get_sensor_info("speed"))
        self.assertLess(speed3, speed2)

    def check_oil_temperature(self):
        self.canbox_helper.cansend("00F#00") # select Oil Temperature mode
        self.canbox_helper.cansend("011#00") # set the initial value
        time.sleep(0.5)
        temp0 = float(self.get_sensor_info("oilTemp"))
        self.canbox_helper.cansend("011#01") # raise oil temperature, like #01, #02, #03, ...
        time.sleep(0.5)
        temp1 = float(self.get_sensor_info("oilTemp"))
        self.assertGreater(temp1, temp0)
        self.canbox_helper.cansend("011#02") # raise oil temperature, like #01, #02, #03, ...
        time.sleep(0.5)
        temp2 = float(self.get_sensor_info("oilTemp"))
        self.assertGreater(temp2, temp1)
        self.canbox_helper.cansend("011#01") # drop oil temperature, like #03, #02, #01 ...
        time.sleep(0.5)
        temp3 = float(self.get_sensor_info("oilTemp"))
        self.assertLess(temp3, temp2)


    def test_VehicalHal_SendMsgToCanBox_SensorTurnSignalState(self):
        self.check_signal_state()

    def test_VehicalHal_SendMsgToCanBox_SensorEngineOilTemp(self):
        self.check_oil_temperature()

    def test_VehicalHal_SendMsgToCanBox_SensorEngineRPM(self):
        self.check_rpm()

    def test_VehicalHal_SendMsgToCanBox_SensorFuelLevelLow(self):
        self.check_fuel_level()

    def test_VehicalHal_SendMsgToCanBox_SensorNightMode(self):
        self.check_night_mode()

    def test_VehicalHal_SendMsgToCanBox_SensorSpeed(self):
        self.check_speed()

    def test_VehicalHal_SendMsgToCanBox_SensorsData(self):
        self.check_night_mode()
        self.check_fuel_level()
        self.check_signal_state()
        self.check_rpm()
        self.check_speed()
        self.check_oil_temperature()

    def test_VehicalHal_SendMsgToCanBox_SensorIterative(self):
        night_mode = self.get_sensor_info("night")
        fuel_level = self.get_sensor_info("fuelLevel")
        state = self.get_sensor_info("signalState")
        eng_rpm = self.get_sensor_info("rpm")
        speed = self.get_sensor_info("speed")
        oil_temp = self.get_sensor_info("oilTemp")

        for i in range(3): # repeat below tests for 3 cycles
            # send msg to change night mode, then check other sensor values should keep original
            self.canbox_helper.cansend("00A#01") # turn on night mode
            time.sleep(0.5)
            night_mode = self.get_sensor_info("night")
            self.assertEqual(self.get_sensor_info("fuelLevel"), fuel_level)
            self.assertEqual(self.get_sensor_info("signalState"), state)
            self.assertEqual(self.get_sensor_info("rpm"), eng_rpm)
            self.assertEqual(self.get_sensor_info("speed"), speed)
            self.assertEqual(self.get_sensor_info("oilTemp"), oil_temp)
            # send msg to change fuel level, then check other sensor values should keep original
            self.canbox_helper.cansend("00B#01") # change fuel level to low
            time.sleep(0.5)
            fuel_level = self.get_sensor_info("fuelLevel")
            self.assertEqual(self.get_sensor_info("night"), night_mode)
            self.assertEqual(self.get_sensor_info("signalState"), state)
            self.assertEqual(self.get_sensor_info("rpm"), eng_rpm)
            self.assertEqual(self.get_sensor_info("speed"), speed)
            self.assertEqual(self.get_sensor_info("oilTemp"), oil_temp)
            # send msg to change signal state, then check other sensor values should keep original
            self.canbox_helper.cansend("00C#00")
            time.sleep(1)
            state = self.get_sensor_info("signalState") # turn signal state
            self.assertEqual(self.get_sensor_info("night"), night_mode)
            self.assertEqual(self.get_sensor_info("fuelLevel"), fuel_level)
            self.assertEqual(self.get_sensor_info("rpm"), eng_rpm)
            self.assertEqual(self.get_sensor_info("speed"), speed)
            self.assertEqual(self.get_sensor_info("oilTemp"), oil_temp)
            # send msg to change engine RPM, then check other sensor values should keep original
            self.canbox_helper.cansend("00D#00") # select RPM mode
            self.canbox_helper.cansend("011#00") # set the initial value
            time.sleep(0.5)
            self.canbox_helper.cansend("011#01") # raise RPM
            time.sleep(0.5)
            eng_rpm = self.get_sensor_info("rpm")
            self.assertEqual(self.get_sensor_info("night"), night_mode)
            self.assertEqual(self.get_sensor_info("fuelLevel"), fuel_level)
            self.assertEqual(self.get_sensor_info("signalState"), state)
            self.assertEqual(self.get_sensor_info("speed"), speed)
            self.assertEqual(self.get_sensor_info("oilTemp"), oil_temp)
            # send msg to change speed, then check other sensor values should keep original
            self.canbox_helper.cansend("00E#00") # select Speed mode
            self.canbox_helper.cansend("011#00") # set the initial value
            time.sleep(0.5)
            self.canbox_helper.cansend("011#01") # raise speed
            time.sleep(0.5)
            speed = self.get_sensor_info("speed")
            self.assertEqual(self.get_sensor_info("night"), night_mode)
            self.assertEqual(self.get_sensor_info("fuelLevel"), fuel_level)
            self.assertEqual(self.get_sensor_info("signalState"), state)
            self.assertEqual(self.get_sensor_info("rpm"), eng_rpm)
            self.assertEqual(self.get_sensor_info("oilTemp"), oil_temp)
            # send msg to change oil temperature, then check other sensor values should keep original
            self.canbox_helper.cansend("00F#00") # select Oil Temperature mode
            self.canbox_helper.cansend("011#00") # set the initial value
            time.sleep(0.5)
            self.canbox_helper.cansend("011#01") # raise oil temperature
            time.sleep(0.5)
            oil_temp = self.get_sensor_info("oilTemp")
            self.assertEqual(self.get_sensor_info("night"), night_mode)
            self.assertEqual(self.get_sensor_info("fuelLevel"), fuel_level)
            self.assertEqual(self.get_sensor_info("signalState"), state)
            self.assertEqual(self.get_sensor_info("rpm"), eng_rpm)
            self.assertEqual(self.get_sensor_info("speed"), speed)


class VehicleHalSendMsgHVACTest(VehicleTestBase):
    """
    @summary: this test used to test vehicle hal HVAC
    """

    __hvac_resource_ids = {
        "AC": "com.google.android.car.kitchensink:id/tbAc",
        "Auto": "com.google.android.car.kitchensink:id/tbAuto",
        "DefrostFront": "com.google.android.car.kitchensink:id/tbDefrostFront",
        "DefrostRear": "com.google.android.car.kitchensink:id/tbDefrostRear",
        "FanSpeed": "com.google.android.car.kitchensink:id/tvFanSpeed",
        "Recirc": "com.google.android.car.kitchensink:id/tbRecirc",
        "TempDriver": "com.google.android.car.kitchensink:id/tvDTemp",
        "TempPassenger": "com.google.android.car.kitchensink:id/tvPTemp",
    }

    __hvac_fan_direction = ['None', 'Face', 'Floor', 'Face+Floor', 'Defrost', 'Def+Floor']

    @classmethod
    def setUpClass(cls):
        super(VehicleHalSendMsgHVACTest, cls).setUpClass()
        cls.canbox_helper = MultiMediaCanboxHelper()

    @classmethod
    def tearDownClass(cls):
        super(VehicleHalSendMsgHVACTest, cls).tearDownClass()

    def setUp(self):
        super(VehicleHalSendMsgHVACTest, self).setUp()
        self.launch_kitchensink_hvac()
        time.sleep(2)

    def tearDown(self):
        self.close_kitchensink_app()
        super(VehicleHalSendMsgHVACTest, self).tearDown()

    def check_recirc_mode(self):
        self.canbox_helper.cansend("001#01") # turn on RECIRC
        time.sleep(0.5)
        self.assertTrue(self.d(resourceId=self.__hvac_resource_ids["Recirc"]).checked)
        self.canbox_helper.cansend("001#00") # turn off RECIRC
        time.sleep(0.5)
        self.assertFalse(self.d(resourceId=self.__hvac_resource_ids["Recirc"]).checked)

    def check_ac_mode(self):
        self.canbox_helper.cansend("003#01") # turn on AC
        time.sleep(0.5)
        self.assertTrue(self.d(resourceId=self.__hvac_resource_ids["AC"]).checked)
        self.canbox_helper.cansend("003#00") # turn off AC
        time.sleep(0.5)
        self.assertFalse(self.d(resourceId=self.__hvac_resource_ids["AC"]).checked)

    def check_auto_mode(self):
        self.canbox_helper.cansend("005#01") # turn on Auto mode
        time.sleep(0.5)
        self.assertTrue(self.d(resourceId=self.__hvac_resource_ids["Auto"]).checked)
        self.canbox_helper.cansend("005#00") # turn off Auto mode
        time.sleep(0.5)
        self.assertFalse(self.d(resourceId=self.__hvac_resource_ids["Auto"]).checked)

    def check_front_defrost(self):
        self.canbox_helper.cansend("004#01") # turn on front defrost
        time.sleep(0.5)
        self.assertTrue(self.d(resourceId=self.__hvac_resource_ids["DefrostFront"]).checked)
        self.canbox_helper.cansend("004#00") # turn off front defrost
        time.sleep(0.5)
        self.assertFalse(self.d(resourceId=self.__hvac_resource_ids["DefrostFront"]).checked)

    def check_rear_defrost(self):
        self.canbox_helper.cansend("006#01") # turn on rear defrost
        time.sleep(0.5)
        self.assertTrue(self.d(resourceId=self.__hvac_resource_ids["DefrostRear"]).checked)
        self.canbox_helper.cansend("006#00") # turn off rear defrost
        time.sleep(0.5)
        self.assertFalse(self.d(resourceId=self.__hvac_resource_ids["DefrostRear"]).checked)

    def check_fan_speed(self):
        min_level = 1
        max_level = 7
        self.canbox_helper.cansend("008#00") # select Fan Speed mode
        self.canbox_helper.cansend("011#00") # set the initial value
        speed_level0 = int(self.d(resourceId=self.__hvac_resource_ids["FanSpeed"]).text)

        self.canbox_helper.cansend("011#01") # raise speed, like #01, #02, #03, ...
        time.sleep(0.5)
        speed_level1 = int(self.d(resourceId=self.__hvac_resource_ids["FanSpeed"]).text)
        if speed_level0 == max_level:
            self.assertEqual(speed_level1, max_level)
        else:
            self.assertGreater(speed_level1, speed_level0)

        self.canbox_helper.cansend("011#02") # raise speed, like #01, #02, #03, ...
        time.sleep(0.5)
        speed_level2 = int(self.d(resourceId=self.__hvac_resource_ids["FanSpeed"]).text)
        if speed_level1 == max_level:
            self.assertEqual(speed_level2, max_level)
        else:
            self.assertGreater(speed_level2, speed_level1)

        self.canbox_helper.cansend("011#01") # drop speed, like #03, #02, #01 ...
        time.sleep(0.5)
        speed_level3 = int(self.d(resourceId=self.__hvac_resource_ids["FanSpeed"]).text)
        if speed_level2 == min_level:
            self.assertEqual(speed_level3, min_level)
        else:
            self.assertLess(speed_level3, speed_level2)

    def check_driver_temperature(self):
        min_level = 16.0
        max_level = 32.0
        self.canbox_helper.cansend("007#00") # select Driver Temperature mode
        self.canbox_helper.cansend("011#00") # set the initial value
        temp_level0 = float(self.d(resourceId=self.__hvac_resource_ids["TempDriver"]).text)

        self.canbox_helper.cansend("011#01") # raise speed, like #01, #02, #03, ...
        time.sleep(0.5)
        temp_level1 = float(self.d(resourceId=self.__hvac_resource_ids["TempDriver"]).text)
        if temp_level0 == max_level:
            self.assertEqual(temp_level1, max_level)
        else:
            self.assertGreater(temp_level1, temp_level0)

        self.canbox_helper.cansend("011#02") # raise speed, like #01, #02, #03, ...
        time.sleep(0.5)
        temp_level2 = float(self.d(resourceId=self.__hvac_resource_ids["TempDriver"]).text)
        if temp_level1 == max_level:
            self.assertEqual(temp_level2, max_level)
        else:
            self.assertGreater(temp_level2, temp_level1)

        self.canbox_helper.cansend("011#01") # drop speed, like #03, #02, #01 ...
        time.sleep(0.5)
        temp_level3 = float(self.d(resourceId=self.__hvac_resource_ids["TempDriver"]).text)
        if temp_level2 == min_level:
            self.assertEqual(temp_level3, min_level)
        else:
            self.assertLess(temp_level3, temp_level2)

    def check_passenger_temperature(self):
        min_level = 16.0
        max_level = 32.0
        self.canbox_helper.cansend("009#00") # select Passenger Temperature mode
        self.canbox_helper.cansend("011#00") # set the initial value
        temp_level0 = float(self.d(resourceId=self.__hvac_resource_ids["TempPassenger"]).text)

        self.canbox_helper.cansend("011#01") # raise speed, like #01, #02, #03, ...
        time.sleep(0.5)
        temp_level1 = float(self.d(resourceId=self.__hvac_resource_ids["TempPassenger"]).text)
        if temp_level0 == max_level:
            self.assertEqual(temp_level1, max_level)
        else:
            self.assertGreater(temp_level1, temp_level0)

        self.canbox_helper.cansend("011#02") # raise speed, like #01, #02, #03, ...
        time.sleep(0.5)
        temp_level2 = float(self.d(resourceId=self.__hvac_resource_ids["TempPassenger"]).text)
        if temp_level1 == max_level:
            self.assertEqual(temp_level2, max_level)
        else:
            self.assertGreater(temp_level2, temp_level1)

        self.canbox_helper.cansend("011#01") # drop speed, like #03, #02, #01 ...
        time.sleep(0.5)
        temp_level3 = float(self.d(resourceId=self.__hvac_resource_ids["TempPassenger"]).text)
        if temp_level2 == min_level:
            self.assertEqual(temp_level3, min_level)
        else:
            self.assertLess(temp_level3, temp_level2)

    def check_fan_direction(self):
        for i in range(1, len(self.__hvac_fan_direction)):
            msg = "002#0" + str(i)
            self.canbox_helper.cansend(msg) # select Fan Direction
            time.sleep(1)
            self.assertTrue(self.d(text=self.__hvac_fan_direction[i]).checked)

    def test_VehicalHal_SendMsgToCanBox_HVACAC(self):
        self.check_ac_mode()

    def test_VehicalHal_SendMsgToCanBox_HVACAuto(self):
        self.check_auto_mode()

    def test_VehicalHal_SendMsgToCanBox_HVACDefrosterFront(self):
        self.check_front_defrost()

    def test_VehicalHal_SendMsgToCanBox_HVACDefrosterRear(self):
        self.check_rear_defrost()

    def test_VehicalHal_SendMsgToCanBox_HVACRecirc(self):
        self.check_recirc_mode()

    def test_VehicalHal_SendMsgToCanBox_HVACFanSpeed(self):
        self.check_fan_speed()

    def test_VehicalHal_SendMsgToCanBox_HVACTemperatureDriver(self):
        self.check_driver_temperature()

    def test_VehicalHal_SendMsgToCanBox_HVACTemperaturePassenger(self):
        self.check_passenger_temperature()

    def test_VehicalHal_SendMsgToCanBox_HVACFanDirection(self):
        self.check_fan_direction()

    def test_VehicalHal_SendMsgToCanBox_HVAC(self):
        self.check_ac_mode()
        self.check_auto_mode()
        self.check_recirc_mode()
        self.check_front_defrost()
        self.check_rear_defrost()
        self.check_driver_temperature()
        self.check_passenger_temperature()
        self.check_fan_speed()
        self.check_fan_direction()

    def confirm_opened_fan_direction(self, direc_code):
        for i in range(1, len(self.__hvac_fan_direction)):
            if i == direc_code:
                self.assertTrue(self.d(text=self.__hvac_fan_direction[i]).checked)
            else:
                self.assertFalse(self.d(text=self.__hvac_fan_direction[i]).checked)

    def test_VehicalHal_SendMsgToCanBox_HVACIterative(self):
        import random

        ac_mode = self.d(resourceId=self.__hvac_resource_ids["AC"]).checked
        auto_mode = self.d(resourceId=self.__hvac_resource_ids["Auto"]).checked
        recirc_mode = self.d(resourceId=self.__hvac_resource_ids["Recirc"]).checked
        front_def = self.d(resourceId=self.__hvac_resource_ids["DefrostFront"]).checked
        rear_def = self.d(resourceId=self.__hvac_resource_ids["DefrostRear"]).checked
        fan_speed = int(self.d(resourceId=self.__hvac_resource_ids["FanSpeed"]).text)
        driver_temp = float(self.d(resourceId=self.__hvac_resource_ids["TempDriver"]).text)
        passenger_temp = float(self.d(resourceId=self.__hvac_resource_ids["TempPassenger"]).text)
        fan_direc = 0

        for i in range(3): # repeat below tests for 3 cycles
            # send msg to change AC mode, then check other HVAC values should keep original
            if False == ac_mode:
                ac_mode = True
                self.canbox_helper.cansend("003#01") # turn on AC
            else:
                ac_mode = False
                self.canbox_helper.cansend("003#00") # turn off AC
            time.sleep(1)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["Auto"]).checked, auto_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["Recirc"]).checked, recirc_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["DefrostFront"]).checked, front_def)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["DefrostRear"]).checked, rear_def)
            self.assertEqual(int(self.d(resourceId=self.__hvac_resource_ids["FanSpeed"]).text), fan_speed)
            self.assertEqual(float(self.d(resourceId=self.__hvac_resource_ids["TempDriver"]).text), driver_temp)
            self.assertEqual(float(self.d(resourceId=self.__hvac_resource_ids["TempPassenger"]).text), passenger_temp)
            self.confirm_opened_fan_direction(fan_direc)

            # send msg to change Auto mode, then check other HVAC values should keep original
            if False == auto_mode:
                auto_mode = True
                self.canbox_helper.cansend("005#01") # turn on Auto
            else:
                auto_mode = False
                self.canbox_helper.cansend("005#00") # turn off Auto
            time.sleep(1)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["AC"]).checked, ac_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["Recirc"]).checked, recirc_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["DefrostFront"]).checked, front_def)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["DefrostRear"]).checked, rear_def)
            self.assertEqual(int(self.d(resourceId=self.__hvac_resource_ids["FanSpeed"]).text), fan_speed)
            self.assertEqual(float(self.d(resourceId=self.__hvac_resource_ids["TempDriver"]).text), driver_temp)
            self.assertEqual(float(self.d(resourceId=self.__hvac_resource_ids["TempPassenger"]).text), passenger_temp)
            self.confirm_opened_fan_direction(fan_direc)

            # send msg to change Recirc mode, then check other HVAC values should keep original
            if False == recirc_mode:
                recirc_mode = True
                self.canbox_helper.cansend("001#01") # turn on Recirc
            else:
                recirc_mode = False
                self.canbox_helper.cansend("001#00") # turn off Recirc
            time.sleep(1)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["AC"]).checked, ac_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["Auto"]).checked, auto_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["DefrostFront"]).checked, front_def)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["DefrostRear"]).checked, rear_def)
            self.assertEqual(int(self.d(resourceId=self.__hvac_resource_ids["FanSpeed"]).text), fan_speed)
            self.assertEqual(float(self.d(resourceId=self.__hvac_resource_ids["TempDriver"]).text), driver_temp)
            self.assertEqual(float(self.d(resourceId=self.__hvac_resource_ids["TempPassenger"]).text), passenger_temp)
            self.confirm_opened_fan_direction(fan_direc)

            # send msg to change Front Defrost mode, then check other HVAC values should keep original
            if False == front_def:
                front_def = True
                self.canbox_helper.cansend("004#01") # turn on Front Defrost
            else:
                front_def = False
                self.canbox_helper.cansend("004#00") # turn off Front Defrost
            time.sleep(1)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["AC"]).checked, ac_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["Recirc"]).checked, recirc_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["Auto"]).checked, front_def)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["DefrostRear"]).checked, rear_def)
            self.assertEqual(int(self.d(resourceId=self.__hvac_resource_ids["FanSpeed"]).text), fan_speed)
            self.assertEqual(float(self.d(resourceId=self.__hvac_resource_ids["TempDriver"]).text), driver_temp)
            self.assertEqual(float(self.d(resourceId=self.__hvac_resource_ids["TempPassenger"]).text), passenger_temp)
            self.confirm_opened_fan_direction(fan_direc)

            # send msg to change Rear Defrost mode, then check other HVAC values should keep original
            if False == rear_def:
                rear_def = True
                self.canbox_helper.cansend("006#01") # turn on Rear Defrost
            else:
                rear_def = False
                self.canbox_helper.cansend("006#00") # turn off Rear Defrost
            time.sleep(1)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["AC"]).checked, ac_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["Recirc"]).checked, recirc_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["Auto"]).checked, front_def)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["DefrostFront"]).checked, rear_def)
            self.assertEqual(int(self.d(resourceId=self.__hvac_resource_ids["FanSpeed"]).text), fan_speed)
            self.assertEqual(float(self.d(resourceId=self.__hvac_resource_ids["TempDriver"]).text), driver_temp)
            self.assertEqual(float(self.d(resourceId=self.__hvac_resource_ids["TempPassenger"]).text), passenger_temp)
            self.confirm_opened_fan_direction(fan_direc)

            # send msg to change Fan Speed, then check other HVAC values should keep original
            self.canbox_helper.cansend("008#00") # select Fan Speed mode
            self.canbox_helper.cansend("011#01")
            time.sleep(1)
            fan_speed = int(self.d(resourceId=self.__hvac_resource_ids["FanSpeed"]).text)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["AC"]).checked, ac_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["Recirc"]).checked, recirc_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["Auto"]).checked, front_def)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["DefrostFront"]).checked, rear_def)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["DefrostRear"]).checked, rear_def)
            self.assertEqual(float(self.d(resourceId=self.__hvac_resource_ids["TempDriver"]).text), driver_temp)
            self.assertEqual(float(self.d(resourceId=self.__hvac_resource_ids["TempPassenger"]).text), passenger_temp)
            self.confirm_opened_fan_direction(fan_direc)

            # send msg to change Driver Temperature, then check other HVAC values should keep original
            self.canbox_helper.cansend("007#00") # select Driver Temperature mode
            self.canbox_helper.cansend("011#01")
            time.sleep(1)
            driver_temp = float(self.d(resourceId=self.__hvac_resource_ids["TempDriver"]).text)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["AC"]).checked, ac_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["Recirc"]).checked, recirc_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["Auto"]).checked, front_def)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["DefrostFront"]).checked, rear_def)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["DefrostRear"]).checked, rear_def)
            self.assertEqual(int(self.d(resourceId=self.__hvac_resource_ids["FanSpeed"]).text), fan_speed)
            self.assertEqual(float(self.d(resourceId=self.__hvac_resource_ids["TempPassenger"]).text), passenger_temp)
            self.confirm_opened_fan_direction(fan_direc)

            # send msg to change Passenger Temperature, then check other HVAC values should keep original
            self.canbox_helper.cansend("009#00") # select Passenger Temperature mode
            self.canbox_helper.cansend("011#01")
            time.sleep(1)
            passenger_temp = float(self.d(resourceId=self.__hvac_resource_ids["TempPassenger"]).text)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["AC"]).checked, ac_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["Recirc"]).checked, recirc_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["Auto"]).checked, front_def)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["DefrostFront"]).checked, rear_def)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["DefrostRear"]).checked, rear_def)
            self.assertEqual(int(self.d(resourceId=self.__hvac_resource_ids["FanSpeed"]).text), fan_speed)
            self.assertEqual(float(self.d(resourceId=self.__hvac_resource_ids["TempDriver"]).text), driver_temp)
            self.confirm_opened_fan_direction(fan_direc)

            # send msg to change Fan Direction, then check other HVAC values should keep original
            fan_direc = random.randint(1,5)
            msg = "002#0" + str(fan_direc)
            self.canbox_helper.cansend(msg)
            time.sleep(1)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["AC"]).checked, ac_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["Recirc"]).checked, recirc_mode)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["Auto"]).checked, front_def)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["DefrostFront"]).checked, rear_def)
            self.assertEqual(self.d(resourceId=self.__hvac_resource_ids["DefrostRear"]).checked, rear_def)
            self.assertEqual(int(self.d(resourceId=self.__hvac_resource_ids["FanSpeed"]).text), fan_speed)
            self.assertEqual(float(self.d(resourceId=self.__hvac_resource_ids["TempDriver"]).text), driver_temp)
            self.assertEqual(float(self.d(resourceId=self.__hvac_resource_ids["TempPassenger"]).text), passenger_temp)
