# -*- coding:utf-8 -*-

'''
@summary: Test DUT display with 20 recents.
@since: 07/07/2016
@author: Lijin Xiong
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.androidframework.common import AdbUtils,UiAutomatorUtils
from testlib.util.log import Logger
from testlib.androidframework.fetch_resources import resource
from testlib.util.common import g_common_obj

LOG = Logger.getlogger(__name__)

class Recents_20(UIATestBase):

    def setUp(self):
        super(Recents_20, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self.d = g_common_obj.get_device()
        UiAutomatorUtils.unlock_screen()
        resource.disable_app_verification()
        for i in range(1,21):
            _apk_path = resource.get_resource_from_atifactory\
            ("tests.tablet.artifactory.conf", "20recents", str(i))
            g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def test_verify_20_recent_apps(self):
        activity_string_template = "com.example.test$NR$/.MainActivity"
        app_string_template = "com.example.test$NR$"
        UiAutomatorUtils.close_all_tasks()
        for i in range(1, 21):
            package_name = app_string_template.replace("$NR$", str(i))
            activity_string = activity_string_template.replace("$NR$", str(i))
            AdbUtils.start_activity_from_shell(activity_string)
            self.d(packageName=package_name).wait.exists(timeout=3000)
            AdbUtils.force_stop_app(package_name)
        # wait a little for the tasks to be registered with the visual manager component
        nr_of_dismissed_tasks = 0
        # hack to count a large number of dismissed tasks
        while True:
            task_dismissed_at_current_step = UiAutomatorUtils.close_all_tasks()
            nr_of_dismissed_tasks += task_dismissed_at_current_step
            if task_dismissed_at_current_step == 0:
                break
        print nr_of_dismissed_tasks
        LOG.info("dismissed " + str(nr_of_dismissed_tasks) + " tasks")
        self.assertTrue(nr_of_dismissed_tasks >= 20, "test was not able to dismiss 20 recent tasks")