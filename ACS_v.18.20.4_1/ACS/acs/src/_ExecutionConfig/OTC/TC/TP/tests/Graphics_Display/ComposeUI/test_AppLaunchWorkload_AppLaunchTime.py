# -*- coding: utf-8 -*-
'''
@summary: This case will not throw Exception,
need framework support check the crash issue.
@since: 04/01/2015
@author: Yingjun Jin
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.graphics.tools import ConfigHandle
from testlib.graphics.composeui_impl import ComposeUiImpl

class ComposeUI(UIATestBase):

    def setUp(self):
        super(ComposeUI, self).setUp()
        self._test_name = __name__
        print "[Setup]: %s" % self._test_name
        self._composeui = ComposeUiImpl()
        config_handle = ConfigHandle()
        result = config_handle.check_apps("com.intel.aws.workload.applaunch")
        if result == 0:
            self._composeui.install_apk('LaunchWorkload')

    def tearDown(self):
        print "[Teardown]: %s" % self._test_name
        super(ComposeUI, self).tearDown()
        self._composeui.uninstall_workload()

    def test_applaunchworkload_applaunchtime(self):
        ''' refer TC test_AppLaunchWorkload_AppLaunchTime
        '''
        print "[RunTest]: %s" % self.__str__()
        self._composeui.launch_workload_am()
        self._composeui.test_config_apps()
        self._composeui.stop_workload_am()

