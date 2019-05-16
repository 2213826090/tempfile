from PyUiApi.common.shell_utils import *
from PyUiApi.adb_helper.instrumentation_utils import *
import datetime


class Calendar(object):
    @staticmethod
    def launch():
        if d(packageName=CALENDAR_PACKAGE_NAME).exists:
            pass
        else:
            UiAutomatorUtils.launch_app_from_apps_menu(CALENDAR_SHORTCUT_NAME)
        Calendar.wait_for_app_launch()

    @staticmethod
    def wait_for_app_launch():
        if d(resourceId=CALENDAR_ADD_ITEM_RESID).wait.exists(timeout=3000):
            return
        while d(resourceId=CALENDAR_RIGHT_ARROW_RESID).wait.exists(timeout=5000):
            d(resourceId=CALENDAR_RIGHT_ARROW_RESID).click()
            time.sleep(2)
        if d(resourceId=CALENDAR_DONE_BUTTON_RESID).wait.exists(timeout=5000):
            d(resourceId=CALENDAR_DONE_BUTTON_RESID).click()
        d(resourceId=CALENDAR_ADD_ITEM_RESID).wait.exists(timeout=5000)

    @staticmethod
    def create_event_in_immediate_future():
        hh, mm, ss = ShellUtils.get_current_dut_time()
        hour = int(hh)
        minute = int(mm)
        alarm_time = datetime.datetime(1, 1, 1, hour, minute, 0) + datetime.timedelta(seconds=60)
        alarm_minute = alarm_time.minute
        delta_to_five_dividable_minute_value = (5 - alarm_minute % 5) % 5
        alarm_time += datetime.timedelta(seconds=60 * delta_to_five_dividable_minute_value)
        hour = alarm_time.hour
        minute = alarm_time.minute
        Calendar.create_event_at_specific_time_multiple_of_5_minutes(hour, minute)
        return alarm_time

    @staticmethod
    def create_event_at_specific_time_multiple_of_5_minutes(hour, minute):
        if d(resourceId=CALENDAR_ADD_ITEM_RESID).wait.exists(timeout=3000):
            d(resourceId=CALENDAR_ADD_ITEM_RESID).click()
        if d(textContains=CALENDAR_ADD_EVENT_OPTION_TXT).wait.exists(timeout=3000):
            d(textContains=CALENDAR_ADD_EVENT_OPTION_TXT).click()
        if d(resourceId=CALENDAR_START_TIME_OPTION_RESID).wait.exists(timeout=3000):
            d(resourceId=CALENDAR_START_TIME_OPTION_RESID).click()
        if hour >= 12 and d(resourceId=CALENDAR_PM_LABEL_RESID).wait.exists(timeout=3000):
            d(resourceId=CALENDAR_PM_LABEL_RESID).click()
            hour -= 12
        else:
            d(resourceId=CALENDAR_AM_LABEL_RESID).click()
        if d(description=str(hour)).wait.exists(timeout=2000):
            d(description=str(hour)).click()
        if d(description=str(minute)).wait.exists(timeout=2000):
            d(description=str(minute)).click()
        if d(text=CALENDAR_OK_OPTION_TXT).wait.exists(timeout=3000):
            d(text=CALENDAR_OK_OPTION_TXT).click()

    @staticmethod
    def save_event():
        if d(resourceId=CALENDAR_SAVE_EVENT_RESID).wait.exists(timeout=3000):
            d(resourceId=CALENDAR_SAVE_EVENT_RESID).click()
        time.sleep(2)

    @staticmethod
    def create_calendar_event(title, description, start_point_seconds, duration_seconds, reminder_minutes=1):
        LOG.info("adding event: %s" % title)
        instrumentation_args = ApiTestsGenericExtraArgs()
        args = instrumentation_args\
            .get_args_string(title=title,
                             description=description,
                             startPointSeconds=start_point_seconds,
                             durationSeconds=duration_seconds,
                             reminderValueMinutes=reminder_minutes)
        result = ApiTestsInterface.run_instrumentation(class_name="CalendarTestsDriver",
                                                       method_name="testInsertEventInCalendar",
                                                       instrumentation_args=args,
                                                       runner_name="GenericArgumentPassingTestRunner")
        return InstrumentationInterface.instrumentation_one_test_pass_output in result
