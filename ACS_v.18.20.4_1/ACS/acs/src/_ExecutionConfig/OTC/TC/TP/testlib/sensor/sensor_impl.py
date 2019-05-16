#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import time
import re
import fileinput
import logging
import ConfigParser
import math
import csv

from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.log import Logger
from sensor_common import SensorCommon

base_path = os.path.split(os.path.realpath(__file__))[0].split(os.sep)

class SensorImpl(SensorCommon):
    """
    Sensor Library Resource
    """
    def __init__(self):
        SensorCommon.__init__(self)

    def write_parameter(self, file_name, catagory, sensorName, result, para1, para2):
        self.logger.info(file_name + ' ' + catagory + ' ' + sensorName + ' ' + result + ' ' + para1 + ' ' + para2 + ': device is: '+self.dsn+'\n')

    def check_result(self, catagory, sensorName, result, para1, para2, type=None, sensor=None, testtype=None):
        # execute the am command
        t_result = 'Failed'
        gyroscope_range = 17.45
        accelerometer_range = 39.14
        linearaccelerometer_range = 19.6
        gravity_range = 19.6
        rotationvector_range = 1.0
        orientation_range = 360.0
        compass_range = 1800
        more_range = 5000
        product = self.check_product()

        t_testfun = {
            'list': 'testListGet',
            'frequency': 'testFrequencyGet',
            'range': 'testRangeGet',
            'delay': 'testDelayGet',
            'concurrent': 'testConcurrent'
            }.get(catagory, 'null')
        if t_testfun == 'null':
            self.logger.error('Invalid test catagory in arg1.\n')
            assert False, "Test Result : Failed"
            return False

        cmd_str = 'am instrument -e class android.intel.umg.sensor.test.TSensorTestTest#'+t_testfun+\
            ' -e testitem '+sensorName+' -e testresult '+result+' -e testresultpara1 '\
            +para1+' -e testresultpara2 '+para2+\
            ' -w android.intel.umg.sensor.test/android.intel.umg.sensor.test.STIOInstrumentationTestRunner'
        t_strdev = g_common_obj.adb_cmd_capture_msg(cmd_str).splitlines()
        #print t_strdev

        p=re.compile('.+for\s+STIOInstrumentationTestRunner=\.$')
        q=re.compile('.+error. Freq:')
        r=re.compile('.+error. Delay:')
        s=re.compile('.+error. Range:')
        t=re.compile('.+error.Res:')
        u=re.compile('.+error. Num:')

        self.conf.read(self.conf_path)
        freq_fastest = self.conf.get("freq_num", "freq_fastest")
        freq_game = self.conf.get("freq_num", "freq_game")
        freq_ui = self.conf.get("freq_num", "freq_ui")
        self.logger.info("freq_fastest: %s,freq_game: %s,freq_ui: %s"%(freq_fastest,freq_game,freq_ui))
        num = 0
        num1 = 0
        num2 = 0
        num3 = 0
        num4 = -1

        for i in t_strdev:
            self.logger.info('  --'+i)
            m=p.match(i)
            n=q.match(i)
            o=r.match(i)
            l=s.match(i)
            e=t.match(i)
            v=u.match(i)

            if m:
                t_result = 'Passed'
                break
            if n:
                num = i.split('error. Freq:')[1].strip()
                self.logger.info('Error Freq:%s'%num)
            if o:
                num1 = i.split('error. Delay:')[1].strip()
                self.logger.info('Error Delay:%s'%num1)
            if l:
                num2 = i.split('error. Range:')[1].strip()
                self.logger.info('Error Range:%s'%num2)
            if e:
                num3 = i.split('error.Res:')[1].strip()
                self.logger.info('Error Res:%s'%num3)
            if v:
                num4 = i.split('error. Num:')[1].split(' ')[0].strip()
                self.logger.info('Error Sensor Number:%s'%num4)

        if product in ["surftab"]: # Trekstor M
            if num != 0:
                if sensor == "gyroscope":
                    if type.find("fastest") == 0:
                        if int(num) >= 198:
                            t_result = 'Passed'
                    elif type.find("game") == 0:
                        if int(num) >= 98:
                            t_result = 'Passed'
                    elif type.find("ui") == 0:
                        if int(num) >= 98:
                            t_result = 'Passed'
                    else:
                        if int(num) >= 98:
                            t_result = 'Passed'
                else:
                    if type.find("fastest") == 0:
                        if int(num) >= 95:
                            t_result = 'Passed'
                        # self.set_freq_num(num, freq_game, freq_ui)
                    elif type.find("game") == 0:
                        if (int(num) >= 49 and int(num) < int(freq_fastest)):
                            t_result = 'Passed'
                        # self.set_freq_num(freq_fastest, num, freq_ui)
                    elif type.find("ui") == 0:
                        if (int(num) >= 9 and int(num) < int(freq_game)):
                            t_result = 'Passed'
                        # self.set_freq_num(freq_fastest, freq_game, num)
                    else:
                        if (int(num) >= 4 and int(num) <= int(freq_game)):
                            t_result = 'Passed'
        else:
            if num != 0:
                if sensor == "compass":
                    # compass
                    if type.find("fastest") == 0:
                        if int(num) >= 20:
                            t_result = 'Passed'
                    elif type.find("game") == 0:
                        if int(num) >= 20:
                            t_result = 'Passed'
                    elif type.find("ui") == 0:
                        if int(num) >= 4:
                            t_result = 'Passed'
                    else:
                        if int(num) >= 4:
                            t_result = 'Passed'
                else:
                    if type.find("fastest") == 0:
                        if int(num) >= 95:
                            t_result = 'Passed'
                        # self.set_freq_num(num, freq_game, freq_ui)
                    elif type.find("game") == 0:
                        if (int(num) >= 49 and int(num) < int(freq_fastest)):
                            t_result = 'Passed'
                        # self.set_freq_num(freq_fastest, num, freq_ui)
                    elif type.find("ui") == 0:
                        if (int(num) >= 9 and int(num) < int(freq_game)):
                            t_result = 'Passed'
                        # self.set_freq_num(freq_fastest, freq_game, num)
                    else:
                        if (int(num) >= 4 and int(num) <= int(freq_game)):
                            t_result = 'Passed'
        if num1 != 0:
            if int(num1) >=0 and int(num1) <= 10:
                t_result = 'Passed'

        if num2 != 0:
            if sensor == "gyroscope":
                if float(num2) >= gyroscope_range:
                    t_result = 'Passed'
            elif sensor == "accelerometer":
                if float(num2) >= accelerometer_range:
                    t_result = 'Passed'
            elif sensor == "linearaccelerometer":
                if float(num2) >= linearaccelerometer_range:
                    t_result = 'Passed'
            elif sensor == "gravity":
                if float(num2) >= gravity_range:
                    t_result = 'Passed'
            elif sensor == "rotationvector":
                if float(num2) == rotationvector_range:
                    t_result = 'Passed'
            elif sensor == "orientation":
                if float(num2) == orientation_range:
                    t_result = 'Passed'
            elif sensor == "compass":
                if float(num2) >= compass_range:
                    t_result = 'Passed'
            else:
                if float(num2) >= more_range:
                    t_result = 'Passed'
        if num3 != 0:
            if sensor == "gyroscope":
                if float(num3) >= 0 and float(num3) <= 0.001:
                    t_result = 'Passed'
            elif sensor == "accelerometer":
                if float(num3) >= 0 and float(num3) <= 0.01:
                    t_result = 'Passed'
            elif sensor == "linearaccelerometer":
                if float(num3) >= 0 and float(num3) <= 0.01:
                    t_result = 'Passed'
            elif sensor == "gravity":
                if float(num3) >= 0 and float(num3) <= 0.01:
                    t_result = 'Passed'
            elif sensor == "orientation":
                if float(num3) >= 0 and float(num3) <= 0.01:
                    t_result = 'Passed'
            elif sensor == "rotationvector":
                if float(num3) >= 0 and float(num3) <= 0.6:
                    t_result = 'Passed'
            elif sensor == "compass":
                if float(num3) >= 0 and float(num3) <= 0.6:
                    t_result = 'Passed'
            else:
                if float(num3) >= 0 and float(num3) <= 1:
                    t_result = 'Passed'

        if int(num4) != -1:
            if int(num4) >= 1:
                t_result = 'Passed'
            else:
                self.logger.info("Test Result : %s"%t_result)
                assert False, "Error List Sensor Number: %s"%num4

        self.logger.info("Test Result : %s"%t_result)

        if cmp(t_result, 'Passed') == 0:
            return True
        else:
            return False

    def download_artifactory_content(self, option_file):
        from testlib.util.repo import Artifactory
        cfg = TestConfig().read(self.conf_file, "artifactory")
        arti_obj = Artifactory(cfg.get("location_apks"))
        ret_file = arti_obj.get(cfg.get(option_file))
        return ret_file

    def install_artifactory_app(self, option_file, package):
        cmd = "pm list package %s" % package
        message = g_common_obj.adb_cmd_capture_msg(cmd).splitlines()
        if 'package:' + package in message:
            self.logger.info("App %s already installed." % package)
            return
        self.logger.info("Install %s." % package)
        ret_file = self.download_artifactory_content(option_file)
        assert os.path.isfile(ret_file)
        install_cmd = "install -r %s" % ret_file
        g_common_obj.adb_cmd_common(install_cmd)
        time.sleep(2)
        message = g_common_obj.adb_cmd_capture_msg(cmd).splitlines()
        print message
        assert 'package:' + package in message, "Install app failed!"


    def download_content(self, cfg, option_location, option_file):
        from testlib.util.repo import Artifactory
        arti_obj = Artifactory(cfg.get(option_location))
        ret_file = arti_obj.get(cfg.get(option_file))
        return ret_file

    def install_app(self, cfg, option_location, option_file, package):
        message = g_common_obj.adb_cmd_capture_msg("pm list package %s" % package)
        if 'package:' + package in message.splitlines():
            self.logger.info("App %s already installed." % package)
            return
        self.logger.info("Install %s." % package)
        ret_file = self.download_content(cfg, option_location, option_file)
        assert os.path.isfile(ret_file)
        install_cmd = "install -r %s" % ret_file
        g_common_obj.adb_cmd_common(install_cmd)
        time.sleep(2)
        message = g_common_obj.adb_cmd_capture_msg("pm list package %s" % package)
        print message.splitlines()
        assert 'package:' + package in message.splitlines(), "Install app failed!"

    def sensor_execute_get_para_common(self, file_name, freq_type):
        cfg = TestConfig().read(self.conf_file, file_name)
        catagory = cfg.get("catagory")
        sensorName = cfg.get("sensorname")
        compare_result = cfg.get("compare_result")
        para1 = cfg.get("para1")
        para2 = cfg.get("para2")
        if self.excute(file_name, catagory, sensorName, compare_result, para1, para2, freq_type) is False:
            return False
        else:
            return True

    def excute(self, file_name, catagory, sensorName, result, para1, para2, type_=None, sensor=None, testtype=None):
        self.write_parameter(file_name, catagory, sensorName, result, para1, para2)
        if self.check_result(catagory, sensorName, result, para1, para2, type_, sensor, testtype) is False:
            return False
        else:
            return True

    def match_configuration(self):
        product = self.check_product()
        self.d.orientaton = "n"
        if "ecs28a" in product:
            return "tests.tablet.sensor.8a.conf"
        elif "ecs27b" in product:
            return "tests.tablet.sensor.7b.conf"
        elif "chiphd8" in product:
            return "tests.tablet.sensor.chiphd.conf"
        elif "malata8" in product:
            return "tests.tablet.sensor.malata.conf"
        elif "ecs210a" in product:
            return "tests.tablet.sensor.10a.conf"
        else:
            return "tests.tablet.sensor.ecs.conf"

    def set_freq_num(self, para1, para2, para3):
        self.conf.read(self.conf_path)
        self.logger.info("freq_fastest: %s,freq_game: %s,freq_ui: %s"%(para1,para2,para3))
        self.conf.set("freq_num", "freq_fastest", para1)
        self.conf.set("freq_num", "freq_game", para2)
        self.conf.set("freq_num", "freq_ui", para3)
        self.conf.write(open(self.conf_path, "w"))

    def sensor_log(self, cfg, delay_type, sensor_type, test_time = 98):
        sensor_package = "com.intel.android.sensorlog"
        sensor_activity = "com.intel.android.sensorlog.SensorRecordFromDeviceActivity"
        #self.install_app(cfg, "location_apks", "sensorlog", sensor_package)
        self.install_artifactory_app("sensorlog", sensor_package)
        self.d.screen.on()
        self.d.press("menu")
        self.setSleepMode("30 minutes")
        g_common_obj.launch_app_am(sensor_package, sensor_activity)
        time.sleep(2)

        if self.d(resourceId="com.intel.android.sensorlog:id/sensors_list", scrollable=True).exists:
            self.d(resourceId="com.intel.android.sensorlog:id/sensors_list", scrollable=True).scroll.vert.to(text="Type: %s (streaming sensor)"%sensor_type)
        self.d(text = "Type: %s (streaming sensor)"%sensor_type).click.wait()
        time.sleep(2)
        self.d(resourceId = "com.intel.android.sensorlog:id/sensor_delay_spinner").click.wait()
        time.sleep(2)
        self.d(text = "Delay %s"%delay_type).click.wait()
        time.sleep(2)
        g_common_obj.adb_cmd_common("logcat -c")
        self.d(text = "Start").click.wait()
        time.sleep(test_time)
        if not "longlasting" in cfg:
            self.d(text = "Stop").click.wait()
        else:
            self.logger.info("streaming running--loop")

    def long_lasting_check_log(self, num = 20):
        product = self.check_product()
        info = g_common_obj.adb_cmd_capture_msg("logcat -v time -d |grep 'Sensor event interval'").splitlines()
        list_persent = []
        num_percent = 0
        a = re.compile('.+%')
        b = re.compile('.+average')
        for i in info:
            c = a.match(i)
            d = b.match(i)
            if c:
                percent = i.split("Sensor event interval jitter (% of avg):")[1].split("%")[0].strip()
                list_persent.append(percent)
        print "[[INFO]-[DATA]]check percent %s times: %s" % (num, list_persent)
        for i in list_persent:
            if "cht" in product:
                if float(i) > 6:
                    num_percent += 1
            else:
                if float(i) > 3:
                    num_percent += 1

        if num_percent > num:
            print "Failed, jitter too high "
            return False

    def longlasting_20min_get(self,):
        g_common_obj.adb_cmd_common("logcat -c")
        get_logtime = 20 * 60
        time.sleep(get_logtime)
        print "-----20min--get-log--"

    def check_log(self, num = 20):
        product = self.check_product()
        info = g_common_obj.adb_cmd_capture_msg("logcat -v time -d |grep 'Sensor event interval'").splitlines()
        # print info
        list_persent = []
        list_time = []
        num_percent = 0
        num_time = 0

        a = re.compile('.+%')
        b = re.compile('.+average')
        for i in info:
            c = a.match(i)
            d = b.match(i)
            if c:
                percent = i.split("Sensor event interval jitter (% of avg):")[1].split("%")[0].strip()
                list_persent.append(percent)
            # if d:
            #     time = i.split("Sensor event interval (average)")[1].split(":")[1].split("ms")[0].strip()
            #     list_time.append(time)
        # self.logger.info("check percent 10 times: %s"%list_persent)
        print "[[INFO]-[DATA]]check percent %s times: %s"%(num, list_persent)
        # self.logger.info("check time 10 times: %s"%list_time)

        for i in list_persent:
            if "cht" in product:
                if float(i) > 10:
                    num_percent +=1
            else:
                if float(i) > 3:
                    num_percent +=1
        # for i in list_time:
        #     if float(i) > 10:
        #         num_time +=1
        # if num_time > 10:
        #     assert False, "delay time error"
        if num_percent > num:
            assert False, "jitter too high: %s"%list_persent

    def check_log_average(self):
        product = self.check_product()
        info = g_common_obj.adb_cmd_capture_msg("logcat -v time -d |grep 'Timestamp synchronization error'").splitlines()
        list_data= []

        for i in info:
            data = i.split('Timestamp synchronization error (average):')[1].split('ms')[0].strip()
            list_data.append(float(data))

        print "Get the Data of Timestamp Sync Error: %s"%list_data

        for i in list_data:
            if i > 100:
                assert False, "Timestamp Sync Error Exception for %s times!"%i 


    def sensor_compass(self, cfg, app_package, app_activity):
        # app_: com.gyz.sensorbackground/com.gyz.sensorbackground.SensorBackgroundActivity
        self.install_app(cfg, "location_apks", "sensorbackground", app_package)
        self.d.screen.on()
        self.d.press("menu")
        self.setSleepMode("30 minutes")
        g_common_obj.launch_app_am(app_package, app_activity)
        time.sleep(5)

        self.d(text = "Setting").click.wait()
        time.sleep(2)
        self.d.press.back()
        # if self.d(scrollable=True).exists:
        #     self.d(scrollable=True).scroll.vert.to(text="0BMC150 Magnetometer")
        self.d(text = "0BMC150 Magnetometer").click.wait()
        time.sleep(2)
        self.d(text = "OK").click.wait()
        time.sleep(5)
        self.d(text = "Resume").click.wait()
        text = self.d(resourceId="com.gyz.sensorbackground:id/TextViewData").text
        log_file = text.split(':')[1].split('\n')[0].strip()

        return log_file

    def aver_index(self, array):

        sum = 0
        for i in range(len(array)/2,len(array)/2+100):
            sum += float(array[i])

        return sum/100.0

    def parse_csv(self, log_file):

        csv_file = log_file.split('/')[4]
        self.root()
        self.pull(log_file)

        csvReader = csv.reader(open(csv_file, 'rb'))
        index_x = []
        index_y = []
        index_z = []

        for row in csvReader:
            parameterStr = ','.join(row)
            parameters = parameterStr.split(',')

            if len(parameters) < 7:
                parameters = [0,0,0,0,0,0,0]

            index_x.append(parameters[4])
            index_y.append(parameters[5])
            index_z.append(parameters[6])

        aver_x = float('%.6f'%self.aver_index(index_x))
        aver_y = float('%.6f'%self.aver_index(index_y))
        aver_z = float('%.6f'%self.aver_index(index_z))

        result = 90 - math.atan2(aver_y, aver_x)*180/math.pi
        return float('%.1f'%result)

    def pull(self, log_file):
        log = log_file.split('/')[4]
        g_common_obj.adb_cmd_common("pull %s ."%log_file)
        if os.path.exists(r'./' + log):
            return
        else:
            assert False, "log file with the format of csv failed to export!"

    def launch_SensorTest_App(self, option_test = None):
        self.unlock_screen()
        g_common_obj.launch_app_am("android.intel.umg.sensor", ".SensorTest")
        for i in range(5):
            if self.d(textStartsWith=option_test).exists:
                break
            else:
                g_common_obj.launch_app_am("android.intel.umg.sensor", ".SensorTest")
        if option_test:
            for i in range(5):
                if self.d(textStartsWith=option_test).exists:
                    self.d(textStartsWith=option_test).click()
                    break
                time.sleep(2)

    def basicTest_SensorTest_App(self, sub_sensor = None, test_time = 50):
        g_common_obj.adb_cmd_common("logcat -c")
        self.launch_SensorTest_App("Basic Test")
        if sub_sensor:
            for i in range(5):
                if self.d(scrollable=True).exists:
                    self.d(scrollable=True).scroll.vert.to(textStartsWith=sub_sensor)
                if self.d(textStartsWith=sub_sensor).exists:
                    self.d(textStartsWith=sub_sensor).click()
                    time.sleep(test_time)
                    break
                time.sleep(2)

    def light_check_reuslt(self, sensor_name, num = 5):
        cmd_str = "logcat -v threadtime -d |grep 'SensorTestV5.0.4: X:'"
        info = g_common_obj.adb_cmd_common(cmd_str).splitlines()

        list_data = []
        num_fail_data = 1
        for i in info:
            if sensor_name == "Light":
                x_data = i.split("SensorTestV5.0.4: X:")[1].split("Y:")[0].strip()
                list_data.append(float(x_data))
            #elif:
             #   sensor_name == ""
        print "[INFO]-[DATA]" + sensor_name + (" sensor check many times SensorTestV5.0.4: X: %s ") % list_data

        if len(list_data) > 0:
            for i in list_data:
                # print "xvlhao: %s , num: %s " %(list_data.index(i)+1, float(i))
                if float(i) < 0 or float(i) == 0:
                    num_fail_data +=1
            print num_fail_data
            if num_fail_data > len(list_data):
                assert False, sensor_name + " Sensor doesn't work: %s" % list_data
        else:
            assert False, sensor_name + " Sensor doesn't work: %s" % list_data

    def launch_My_Sensor_App(self):
        product = self.check_product()
        my_sensor_package = "com.kfodor.MySensors"
        self.install_artifactory_app("mysensor", my_sensor_package)
        g_common_obj.launch_app_am(my_sensor_package, ".MySensors")
        if "cht" in product:
            if self.d(textContains="Temperature Sensor").exists:
                self.d(textContains="Temperature Sensor").click.wait()
        elif "gordon_peak" in product:
            if self.d(textContains="Ambient temperature").exists:
                self.d(textContains="Ambient temperature").click.wait()
        else:
            if self.d(textContains="ambient_temp").exists:
                self.d(textContains="ambient_temp").click.wait()

    def check_temperature_sensor_data(self):
        #compare var 25 on lab, expected var: +-10C[15, 35]
        self.launch_My_Sensor_App()
        time.sleep(8)
        list_data = []
        fail_temp = 1
        for i in range(10):
            time.sleep(4)
            temp_data = self.d(resourceId= "com.kfodor.MySensors:id/data_value").text
            if temp_data == '' or temp_data == None:
                raise ValueError("[Info]: the temp data is null string")
            assert float(temp_data), "[Info]: the temp data must be a num"
            list_data.append(float(temp_data))
        print "[INFO]-[DATA]temperature sensor data: %s " % list_data
        for i in list_data:
            if float(i) < 17 or float(i) > 35:
                fail_temp += 1
        print fail_temp
        if fail_temp > 5:
            assert False, "temperature sensor ambient data not Accurately: %s " % list_data

    def check_temperature_sensor_accuracy(self):
        # 1.temperature sensor accuracy, compare var 28 on lab, expected var: +-5C[21, 33]
        # 2. get Temp var by input shell commands
        #    get Temp var not support thermal_zone2/temp
        #self.check_temperature_sensor_accuracy_by_shell_commands()
        self.launch_My_Sensor_App()
        time.sleep(3)
        self.check_temperature_sensor_info("data")
        list_data = []
        fail_temp = 1
        for i in range(10):
            time.sleep(4)
            temp_data = self.d(resourceId="com.kfodor.MySensors:id/data_value").text
            list_data.append(float(temp_data))
        print "[INFO]-[DATA]temperature sensor data by MySensor apk: %s " % list_data
        for i in list_data:
            if float(i) < 20 or float(i) > 33:
                fail_temp += 1
        print fail_temp
        if fail_temp > 6:
            assert False, "temperature sensor ambient data not Accurately: %s " % list_data

    def check_temperature_sensor_accuracy_by_shell_commands(self):
        # get Temp var by input shell commands, compare var 28 on lab, expected var: +-4C[21, 33]
        self.root()
        ambient_temp_cmd = "cat /sys/class/thermal/thermal_zone2/temp"
        data = g_common_obj.adb_cmd_capture_msg(ambient_temp_cmd)
        if data == '' or data == None:
            raise ValueError("[Info]: the temp data is null string")
        temp_data = int(data) / 1000
        print "[INFO]-[DATA]temperature sensor data by Commands: %s " % temp_data
        if temp_data < 21 or temp_data > 33:
            assert False, "temperature sensor ambient data not Accurately: %s " % temp_data

    def check_temperature_sensor_info(self, target):
        t_resourceid_func = {
            "Name" : "com.kfodor.MySensors:id/sensor_name",
            "Vendor" : "com.kfodor.MySensors:id/sensor_vendor",
            "Type" : "com.kfodor.MySensors:id/sensor_type",
            "Range" : "com.kfodor.MySensors:id/sensor_range",
            "Version" : "com.kfodor.MySensors:id/sensor_version",
            "Resolution" : "com.kfodor.MySensors:id/sensor_resolution",
            "Accuracy" : "com.kfodor.MySensors:id/sensor_accuracy",
            "Delay" : "com.kfodor.MySensors:id/sensor_delay",
            "data" : "com.kfodor.MySensors:id/data_value"
        }.get(target, 'null')
        if "Range" in target:
            range_temp = self.d(resourceId=t_resourceid_func).text
            print "[INFO]-[DATA]temperature sensor Range: %s " % range_temp
            if float(range_temp) < -40 or  float(range_temp) > 85:
                assert False, "temperature sensor Range Error"
        if "Accuracy" in target:
            comm_temp = self.d(resourceId=t_resourceid_func).text
            if 'High' in comm_temp:
                print "[INFO]-[DATA]temperature sensor Accuracy: %s " % comm_temp
            else:
                assert False, comm_temp
        if "Name" in target:
            comm_temp = self.d(resourceId=t_resourceid_func).text
            if 'ambient_temp' in comm_temp or 'Temperature' in comm_temp or "Ambient temperature" in comm_temp:
                print "[INFO]-[DATA]temperature sensor Name: %s " % comm_temp
            else:
                assert False, comm_temp
        if "Type" in target:
            comm_temp = self.d(resourceId=t_resourceid_func).text
            if 'Ambient Temperature (13)' in comm_temp:
                print "[INFO]-[DATA]temperature sensor Type: %s " % comm_temp
            else:
                assert False, comm_temp
        if "Version" in target:
            comm_temp = self.d(resourceId=t_resourceid_func).text
            if '1' in comm_temp:
                print "[INFO]-[DATA]temperature sensor Version: %s " % comm_temp
            else:
                assert False, comm_temp
        if "Delay" in target:
            comm_temp = self.d(resourceId=t_resourceid_func).text
            if 'Normal' in comm_temp:
                print "[INFO]-[DATA]temperature sensor Event Delay: %s " % comm_temp
            else:
                assert False, comm_temp
        if "data" in target:
            time.sleep(5)
            data_value = self.d(resourceId=t_resourceid_func).text
            if data_value == '' or data_value == None:
                raise ValueError("[Info]: the temp data is null string")
            assert float(data_value), "[Info]: the temp data must be a num"
            print "[INFO]-[DATA]temperature sensor data: %s " % data_value

    def not_bxt_check_all_sensor(self):
        # eg: cht, 3gr
        testprjsenortest_package = "android.intel.umg.sensor.test"
        sensortest_package = "android.intel.umg.sensor"
        self.install_artifactory_app("testprjsenortest", testprjsenortest_package)
        self.install_artifactory_app("sensortest", sensortest_package)
        freq_type = ['accelerometer','gyroscope', 'gravity', 'compass', 'linear_accelerometer', 'rotation_vector', 'orientation']
        for i in range(len(freq_type)):
            file_Name = 'frequency_%s_fastest' % freq_type[i]
            self.logger.info("")
            self.logger.info("Loop all sensor %d/7 " % (i + 1) + file_Name)
            if self.sensor_execute_get_para_common(file_Name, "fastest") is False:
                assert False, "%s Sensor support is Failed" % file_Name
            time.sleep(5)
            continue

    def check_all_sensor_support(self):
        product_name = self.check_product()
        if "bxt" in product_name or "gordon_peak" in product_name:
            self.launch_My_Sensor_App()
            time.sleep(5)
            self.check_temperature_sensor_info("data")
        else:
            self.not_bxt_check_all_sensor()
