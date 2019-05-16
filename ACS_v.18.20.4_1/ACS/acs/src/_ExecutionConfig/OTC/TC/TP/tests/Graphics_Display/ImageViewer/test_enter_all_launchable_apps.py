# -*- coding: utf-8 -*-
"""
Created on Jan 23, 2015

@author: yusux
"""
from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.launcher import LaunchFromDrawer
from testlib.graphics.aosp_apps_init import AospAppsInit


class LaunchAllApps(UIATestBase):
    @classmethod
    def setUpClass(self):
        super(LaunchAllApps, self).setUpClass()

    @classmethod
    def tearDownClass(self):
        super(LaunchAllApps, self).tearDownClass()

    def setUp(self):
        super(LaunchAllApps, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(LaunchAllApps, self).tearDown()

    def test_launch_all_apps(self):
        """
            launch all installed apps in DUT\
            will require gmail account loggable and logged in\
        """
        instanc = LaunchFromDrawer()
        # instanc.getlistoflauncheableapps()
        AospAppsInit().init_dut_squeence()
        instanc.launch_app_from_drawer()
