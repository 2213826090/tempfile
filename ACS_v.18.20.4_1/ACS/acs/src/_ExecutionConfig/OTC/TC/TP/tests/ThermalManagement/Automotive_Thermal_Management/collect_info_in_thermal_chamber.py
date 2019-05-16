#! /usr/bin/env python
# coding:utf-8

import time
import os
import csv
import unittest
from testlib.em.thermal import Thermal
from testlib.em.power import get_power_obj
#from testlib.em.tools import gen_tmp_file_path_by_time

class CollectData(unittest.TestCase):
    def setUp(self):
        self._test_name = __name__
        print
        print "[Setup]: %s" % self._test_name
        self.thermal = Thermal()
        self.power = get_power_obj()
        #self.csv_path = gen_tmp_file_path_by_time("temps.csv")
        self.csv_name = ("Thermal_data_%s.csv" % time.strftime("%y-%m-%d-%H-%M-%S"))
        self.csv_file = open(self.csv_name, "wb")
        self.csv_writer = csv.writer(self.csv_file)
        #self.reboot_temps = [0, 20, 30, 40, 55, 75]
        self.reboot_temps = [0, 10, 20, 30, 40, 50, 55, 70, 75, 80]
        self.thermal.adb_root()
        self.csv_writer.writerow(["tcpu", "tskn", "tamb", "fan", "power", "reboot"])

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        self.csv_file.close()

    def test_collect_data(self):
        print "[RunTest]: %s" % self.__str__()
        for temp in self.reboot_temps:
            print "[Info]------------------- %s" %temp
            while True:
                tskn, tamb, fan = self.thermal.get_fan_status()
                tcpu = self.thermal.ivi_get_cpu_temp() / 1000
                power = self.thermal.ivi_get_power()
                power = power / float(10 ** 6)
                line = [tcpu, tskn, tamb, fan, power, "No"]
                if abs(temp - tamb) < 3:
                    if temp in (20, 30):
                        self.power.adb_reboot()
                        self.thermal.adb_root()
                        line[-1] = "Yes"
                        print "tcpu: %s, tskn: %s, tamb: %s, fan: %s, power: %.2f, boot: %s" % tuple(line)
                        self.csv_writer.writerow(line)
                        time.sleep(10)
                    if temp in (0, 20, 40, 55, 75):
                        self.power.power_off_on_os_by_ignition()
                        time.sleep(10)
                        self.thermal.adb_root()
                        line[-1] = "boot_up_completed"
                        print "tcpu: %s, tskn: %s, tamb: %s, fan: %s, power: %.2f, boot: %s" % tuple(line)
                        self.csv_writer.writerow(line)
                    break
                else:
                    print "tcpu: %s, tskn: %s, tamb: %s, fan: %s, power: %.2f, boot: %s" % tuple(line)
                    self.csv_writer.writerow(line)
                    #time.sleep(30)
                self.csv_file.flush()

