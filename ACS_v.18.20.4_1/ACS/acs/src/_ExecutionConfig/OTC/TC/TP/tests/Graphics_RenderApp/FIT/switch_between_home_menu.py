# -*- coding: utf-8 -*-
'''
Created on Dec 19, 2014

@author: yusux
'''
from testlib.graphics.launcher import LaunchFromDrawer
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase


class TestLauncherMenu(RenderAppTestBase):

    '''
    launcher Home MenuSwitcher
    '''

    def setUp(self):
        super(TestLauncherMenu, self).setUp()
        self._test_name = __name__
        self.instance = LaunchFromDrawer()

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(TestLauncherMenu, self).tearDown()
        self.instance = None

    def test_Home_Menu_Switch(self):
        self.instance.swichbetweenhomemenus()
