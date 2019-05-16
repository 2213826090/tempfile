# -*- coding:utf-8 -*-

'''
@summary: Multiple sequential reminders test.
@since: 07/01/2016
@author: Lijin Xiong
'''

import time
from testlib.util.uiatestbase import UIATestBase
from testlib.util.common import g_common_obj
from testlib.androidframework.common import Calendar,UiAutomatorUtils
from testlib.util.log import Logger
from testlib.androidframework.fetch_resources import resource


LOG = Logger.getlogger(__name__)


class Calendar_Reminder(UIATestBase):

    def setUp(self):
        super(Calendar_Reminder, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name
        self.c = Calendar()
        self.d = g_common_obj.get_device()
        UiAutomatorUtils.unlock_screen()
        resource.disable_app_verification()
        _apk_path = resource.get_resource_from_atifactory\
        ("tests.tablet.artifactory.conf", "SDK_API", "api_test")
        g_common_obj.adb_cmd_common("install -r %s" % _apk_path)

    def test_check_multiple_sequential_reminders(self):
        events_added = []
        for i in range(6):
            title = "ApiTestsEvent" + str(i+1)
            description = "ApiTestsDescription"
            start_point_sec = 100 + 40 * i  # add reminder every 40 seconds
            duration_sec = 60
            event_added = Calendar.create_calendar_event(title, description, start_point_sec, duration_sec)
            LOG.info(str(title) + " event add was: " + str(event_added))
            if event_added:
                events_added.append(title)
        self.assertTrue(len(events_added) > 4, " there were not enough events added")
        try:
            notifications_elapsed = False
            for i in range(35):
                LOG.info("waiting for event notification, time elapsed: " + str(i*20) + " seconds")
                time.sleep(20)
#                 StatusBar.open_notifications()
                self.d.open.notification()
                for event_title in events_added:
                    if self.d(textContains=event_title).wait.exists(timeout=1000):
                        LOG.info("notification for event %s appeared" % event_title)
                        events_added.remove(event_title)
                        LOG.info("notifications left: " + str(events_added))
                    if len(events_added) == 0:  # all events notifications appeared
                        notifications_elapsed = True
                self.d.press.home()
                if notifications_elapsed:
                    break
            self.assertTrue(len(events_added) == 0, "some added reminders did not yield notifications")
        finally:
            self.d.open.notification()
            if self.d(resourceId="com.android.systemui:id/dismiss_text").exists:
                self.d(resourceId="com.android.systemui:id/dismiss_text").click()