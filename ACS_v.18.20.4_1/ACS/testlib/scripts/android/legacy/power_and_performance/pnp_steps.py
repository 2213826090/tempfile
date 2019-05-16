#!/usr/bin/env python

########################################################################
#
# @filename:    pnp_steps.py
# @description: PnP test steps
# @author:      emilianx.c.ioana@intel.com
#
########################################################################

from pnp_step import step as pnp_step
from testlib.base import base_utils
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.power_and_performance import pnp_utils

import time
import os

class pnp_clear_cache_and_logs(adb_step):
    def __init__(self,**kwargs):
        adb_step.__init__(self, **kwargs)
    def do(self):
        adb_steps.command(command = "logcat -c")()
        benchmark_packs = {
            "Quadrant" : "com.aurorasoftworks.quadrant.ui.professional",
            "Antutu" : "com.antutu.ABenchMark",
            "3D Mark" : "com.futuremark.dmandroid.application",
            "GL Benchmark" : "com.glbenchmark.glbenchmark27"}
        adb_steps.command(command = "pm clear {0}".format(benchmark_packs[kwargs["benchmark"]]))

class antutu(pnp_step):

    """ description:
            Runs an AnTuTu test. It can be an overall score test or component
            test. Returns a dictionary with the result value(s).

        usage:
            antutu(serial = <serial>,
                    port = <adb_port>,
                    benchmark = Antutu,
                    test_case = {"IO" | "CPU" | "GPU" | "UX" | "RAM" | "all"})

        tags:
            antutu
    """
    def __init__(self,**kwargs):
        pnp_step.__init__(self,**kwargs)

        self.possible_tests = pnp_utils.get_antutu_possible_tests()
        self.sleeps = pnp_utils.get_antutu_sleep("t100")

    def do(self, **kwargs):
        pnp_step.do(self)

        #adb_steps.install_apk("./apks/AnTuTu-4.0.3.apk")()
        pnp_clear_cache_and_logs(**kwargs)

        ui_steps.open_app_from_allapps(view_to_find = {"textContains": "AnTuTu"}, debug=True)()
        ui_steps.click_button(view_to_find = {"resourceId":"com.antutu.ABenchMark:id/test_btn"})()

        if self.test_case != "all":
            for key in self.possible_tests:
                if key != self.test_case:
                    ui_steps.click_button(view_to_find = {"textContains":key})()
        ui_steps.click_button(view_to_find = {"resourceId":"com.antutu.ABenchMark:id/test_all_btn"})()
        time.sleep(self.sleeps[self.test_case])

        self.step_data = pnp_utils.get_antutu_result(self.test_case,self.possible_tests)


    def check_condition(self):
        for key,value in self.step_data.iteritems():
            if value == 0 or value == "0" or value == "" or value is None:
                return False
        return True

class quadrant(pnp_step):

    """ description:
            Runs a Quadrant test. Always runs all sub-tests

        usage:
            threedmark(serial = <serial>,
                    port = <adb_port>,
                    benchmark = "Quadrant",
                    test_case = "all")

        tags:
            Quadrant
    """
    def __init__(self,**kwargs):
        pnp_step.__init__(self,**kwargs)
        self.benchmark = self.test_case

        self.sleeps = pnp_utils.get_quadrant_sleep("t100")

    def do(self, **kwargs):
        pnp_step.do(self)
        '''
        adb_steps.install_apk("./apks/quadrant.ui.professional-2.1.1.apk")
        '''
        adb_steps.command(command = "pm clear com.aurorasoftworks.quadrant.ui.professional")()


        ui_steps.open_app_from_allapps(view_to_find = {"textContains": "Quadrant"})()
        ui_steps.click_button(view_to_find = {"textContains":"OK"})()
        ui_steps.click_button(view_to_find = {"textContains":"full benchmark"})()
        adb_steps.command(command = "logcat -c")()
        time.sleep(self.sleeps[self.test_case])
        self.step_data = pnp_utils.get_quadrant_result(self.adb_connection)

    def check_condition(self):
        for key,value in self.step_data.iteritems():
            if value == 0 or value == "0" or value == "" or value is None:
                return False
        return True

class threedmark(pnp_step):

    """ description:
            Runs an 3DMark test. It can be either IceStorm or IceStorm Extreme

        usage:
            threedmark(serial = <serial>,
                    port = <adb_port>,
                    benchmark = {"3D Mark"},
                    test_case = {"Ice Storm" | "Ice Storm Extreme})

        tags:
            3DMark, threedmark
    """
    def __init__(self,**kwargs):
        pnp_step.__init__(self,**kwargs)
        self.sleeps = pnp_utils.get_threedmark_sleep("t100")

    def do(self, **kwargs):
        pnp_step.do(self)
        '''
        adb_steps.install_apk("./apks/3DMark_v100_installer.apk")
        adb_steps.command(command = "mkdir /mnt/shell/emulated/0/Android/obb")
        adb_steps.command(command = "mkdir /mnt/shell/emulated/0/Android/obb/com.futuremark.dmandroid.application/")

        adb_steps.push_file(local = "./apks/main.7.com.futuremark.dmandroid.application.obb", \
                            remote = "/mnt/shell/emulated/0/Android/obb/com.futuremark.dmandroid.application/", \
                            timeout = 500)
        adb_steps.push_file(local = "./apks/patch.7.com.futuremark.dmandroid.application.obb", \
                            remote = "/mnt/shell/emulated/0/Android/obb/com.futuremark.dmandroid.application/", \
                            timeout = 500)

        adb_steps.command(command = "pm clear com.futuremark.dmandroid.application")()
        '''
        adb_steps.command(command = "logcat -c")()

        ui_steps.open_app_from_allapps(view_to_find = {"textContains": "3DMark"})()
        ui_steps.click_button(view_to_find = {"textContains":" MY DEVICE"})()
        ui_steps.click_button(view_to_find = {"textContains":" 3DMARK"})()
        ui_steps.click_button(view_to_find = {"descriptionContains":self.test_case})()

        time.sleep(self.sleeps)

        self.step_data = pnp_utils.get_threedmark_result(self.adb_connection,self.test_case)

    def check_condition(self):
        for key,value in self.step_data.iteritems():
            if value == 0 or value == "0" or value == "" or value is None:
                return False
        return True

class glbenchmark(pnp_step,ui_step):

    """ description:
            Runs a GLBenchmark test.

        usage:
            threedmark(serial = <serial>,
                    port = <adb_port>,
                    benchmark = "GL Benchmark",
                    test_case = "all")

        tags:
            GLBenchmark
    """
    def __init__(self,**kwargs):
        ########################################################################
        # Self-explanatory.
        ########################################################################
        pnp_step.__init__(self,**kwargs)
        ui_step.__init__(self, **kwargs)

        ########################################################################
        # get the benchmark / platform specific sleep time.
        self.sleeps = pnp_utils.get_glbenchmark_sleep(platform = "t100")

    def do(self, **kwargs):
        pnp_step.do(self)
        ########################################################################
        # install apk and prereqs.
        ########################################################################
        '''
        adb_steps.install_apk(\
        "./apks/GLBenchmark_2_7_0_Release_a68901_Android_Corporate_Package.apk")
        '''
        ########################################################################
        # clear logcat and app cache prior to running the benchmark. Logcat must
        # be cleared for result collection, cache for ensuring similar test env.
        ########################################################################
        pnp_clear_cache_and_logs(**kwargs)

        ########################################################################
        # Start benchmark and select desired test cases. As the list is quite
        # long, scroll through the page and always select the first two of each
        # T-Rex and Egypt test cases.
        ########################################################################
        ui_steps.open_app_from_allapps(view_to_find = {"textContains": "GLBenchmark"})()
        ui_steps.click_button(view_to_find = {"textContains":"Performance Tests"}, view_to_check = {"textContains":"Start"})()
        for text in ['T-Rex', 'Egypt']:
            self.uidevice(scrollable=True).scroll.to(textContains=text)
            for i in [0, 1]:
                self.uidevice(textContains=text)[i].click()

        ########################################################################
        # Start test and wait for a predefined period of time. Not waiting would
        # interfere with result values.
        ########################################################################
        ui_steps.click_button(view_to_find = {"textContains":"Start"})()
        time.sleep(self.sleeps[self.test_case])

        ########################################################################
        # Get the result. get_glbenchmark_results returns a dictionary with all
        # relevan {key:value} items.
        ########################################################################
        self.step_data = pnp_utils.get_glbenchmark_result(self.adb_connection)

    def check_condition(self):
        ########################################################################
        # very basic result validation ("must have some value")
        ########################################################################
        for key,value in self.step_data.iteritems():
            if value == 0 or value == "0" or value == "" or value is None:
                return False
        return True
