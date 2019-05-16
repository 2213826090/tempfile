# -*- coding:utf-8 -*-

'''
@summary: Reminder granurality test.
@since: 07/01/2016
@author: Lijin Xiong
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.androidframework.common import SystemApiTestsInterface,Calendar,SystemUtils,UiAutomatorUtils
from testlib.util.log import Logger
from testlib.androidframework.fetch_resources import resource


LOG = Logger.getlogger(__name__)


class Calendar_Reminder_Granurality(UIATestBase):

    def setUp(self):
        super(Calendar_Reminder_Granurality, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self.c = Calendar()
        self.s = SystemUtils()
        self.d = g_common_obj.get_device()
        UiAutomatorUtils.unlock_screen()
        resource.disable_app_verification()
        for i in ["api_test", "system_api"]:
            _apk_path = resource.get_resource_from_atifactory\
            ("tests.tablet.artifactory.conf", "SDK_API", i)
            g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def test_check_reminder_granularity(self):
        result = SystemApiTestsInterface.run_instrumentation(class_name="CommonTestsDriver",
                                                             method_name="testDisableAutoTime",
                                                             instrumentation_args=None,
                                                             runner_name="GenericArgumentPassingTestRunner")
        self.assertTrue(SystemApiTestsInterface.was_instrumentation_test_successful(result),
                        "Could not disable auto time sync")

        time_slided_on_dut = None
        try:
            # set a reminder 5 minutes (300 seconds) in the future
            event_title = "GranularityTestMinutes"
            event_created = Calendar.create_calendar_event(event_title, "GranularityTestMinutesDesc", 300, 100)
            self.assertTrue(event_created, "5 minute granularity event was not created")
            SystemUtils.set_system_time_slide_seconds(250)
            self.assertTrue(SystemUtils.wait_for_reminder_notification(event_title),
                            "5 minute granularity event did not trigger a notification")
            time_slided_on_dut = 250

            # set a reminder 1 hour (3600 seconds) in the future
            event_title = "GranularityTestHours"
            event_created = Calendar.create_calendar_event(event_title, "GranularityTestHoursDesc", 3600, 100)
            self.assertTrue(event_created, "1 hour granularity event was not created")
            SystemUtils.set_system_time_slide_seconds(3600)
            self.assertTrue(SystemUtils.wait_for_reminder_notification(event_title),
                            "1 hour granularity event did not trigger a notification")
            time_slided_on_dut += 3600

            # set a reminder 1 day (86400 seconds) in the future
            event_title = "GranularityTestDays"
            event_created = Calendar.create_calendar_event(event_title, "GranularityTestDaysDesc", 86400, 100)
            self.assertTrue(event_created, "1 hour granularity event was not created")
            SystemUtils.set_system_time_slide_seconds(86400)
            self.assertTrue(SystemUtils.wait_for_reminder_notification(event_title),
                            "1 day granularity event did not trigger a notification")
            time_slided_on_dut += 86400
        finally:
            if time_slided_on_dut is not None:
                SystemUtils.set_system_time_slide_seconds(-time_slided_on_dut)