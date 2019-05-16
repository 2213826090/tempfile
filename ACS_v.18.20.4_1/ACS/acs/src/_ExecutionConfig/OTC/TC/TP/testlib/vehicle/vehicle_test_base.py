# coding: UTF-8
"""
@summary: Vehicle test base class
@since: 11/30/2017
@author: Jinliang Wang (jinliang.wang@intel.com)
"""
import time
from testlib.util.common import g_common_obj
from testlib.util.uiatestbase import UIATestBase

class VehicleTestBase(UIATestBase):

    @classmethod
    def setUpClass(cls):
        super(VehicleTestBase, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        g_common_obj.root_on_device()
        g_common_obj.remount_device()
        cls.kitchensink_pkg_name = "com.google.android.car.kitchensink"
        cls.kitchensink_activity_name = ".KitchenSinkActivity"

    @classmethod
    def tearDownClass(cls):
        super(VehicleTestBase, cls).tearDownClass()

    def setUp(self):
        super(VehicleTestBase, self).setUp()

    def tearDown(self):
        super(VehicleTestBase, self).tearDown()

    def launch_kitchensink_app(self):
        g_common_obj.launch_app_am(self.kitchensink_pkg_name, self.kitchensink_activity_name)
        self.d(description="Open drawer").click()

    def launch_kitchensink_sensors(self):  
        self.launch_kitchensink_app()
        while True:
            if self.d(text="sensors").exists:
                self.d(text="sensors").click()
                break
            self.d.press.down()
            time.sleep(0.5)

    def launch_kitchensink_hvac(self):
        self.launch_kitchensink_app()
        while True:
            if self.d(text="hvac").exists:
                self.d(text="hvac").click()
                break
            self.d.press.down()
            time.sleep(0.5)

    def close_kitchensink_app(self):
        g_common_obj.stop_app_am(self.kitchensink_pkg_name)

