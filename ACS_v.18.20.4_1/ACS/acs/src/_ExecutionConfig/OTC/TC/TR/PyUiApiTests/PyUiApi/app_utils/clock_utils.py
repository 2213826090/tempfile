from PyUiApi.common.uiautomator_utils import *
import datetime


class ClockM(object):
    @staticmethod
    def launch():
        if d(packageName=CLOCK_PACKAGE_NAME).exists:
            pass
        else:
            UiAutomatorUtils.launch_app_from_apps_menu(CLOCK_SHORTCUT_NAME)

    @staticmethod
    def go_to_alarms_activity():
        Clock.launch()
        if d(description=CLOCK_ALARM_DESC).wait.exists(timeout=3000):
            d(description=CLOCK_ALARM_DESC).click()

    @staticmethod
    def delete_all_alarms():
        Clock.launch()
        Clock.go_to_alarms_activity()
        while d(description=CLOCK_EXPAND_ALARM_DESC).wait.exists(timeout=3000):
            try:
                d(description=CLOCK_EXPAND_ALARM_DESC)[0].click()
                d(description=CLOCK_DELETE_ALARM_DESC).wait.exists(timeout=3000)
                d(description=CLOCK_DELETE_ALARM_DESC)[0].click()
            except:
                LOG.info("some error occured at alarm cleanup")
            time.sleep(2)

    @staticmethod
    def determine_alarm_time(hour, minute, offset=0):
        hour = int(hour)
        minute = int(minute)
        alarm_time = datetime.datetime(1, 1, 1, hour, minute, 0) + datetime.timedelta(seconds=120 + offset)
        alarm_minute = alarm_time.minute
        delta_to_five_dividable_minute_value = (5 - alarm_minute % 5) % 5
        alarm_time += datetime.timedelta(seconds=60 * delta_to_five_dividable_minute_value)
        return alarm_time

    @staticmethod
    def set_new_alarm(hour, minute, offset=0):
        alarm_time = ClockM.determine_alarm_time(hour, minute, offset)
        ClockM.launch()
        hour = alarm_time.hour
        minute = alarm_time.minute
        if d(description=CLOCK_ALARM_DESC).wait.exists(timeout=3000):
            d(description=CLOCK_ALARM_DESC).click()
        if d(description=CLOCK_ADD_ALARM_BUTTON_DESC).wait.exists(timeout=3000):
            d(description=CLOCK_ADD_ALARM_BUTTON_DESC).click()
        if hour >= 12 and d(text=CLOCK_PM_MARKER_TXT).wait.exists(timeout=2000):
            d(text=CLOCK_PM_MARKER_TXT).click()
            hour -= 12
        else:
            d(text=CLOCK_AM_MARKER_TXT).click()
        if d(description=str(hour)).wait.exists(timeout=2000):
            d(description=str(hour)).click()
        if d(description=str(minute)).wait.exists(timeout=2000):
            d(description=str(minute)).click()
        if d(text="OK").wait.exists(timeout=3000):
            d(text="OK").click()
        return alarm_time


class ClockN(ClockM):
    @staticmethod
    def set_new_alarm(hour, minute, offset=0):
        alarm_time = ClockN.determine_alarm_time(hour, minute, offset)
        ClockN.launch()
        hour = alarm_time.hour
        minute = alarm_time.minute
        if d(description=CLOCK_ALARM_DESC).wait.exists(timeout=3000):
            d(description=CLOCK_ALARM_DESC).click()
        if d(description=CLOCK_ADD_ALARM_BUTTON_DESC).wait.exists(timeout=3000):
            d(description=CLOCK_ADD_ALARM_BUTTON_DESC).click()
        if d(description=str(hour)).wait.exists(timeout=2000):
            d(description=str(hour)).click()
        if d(description=str(minute)).wait.exists(timeout=2000):
            d(description=str(minute)).click()
        if d(text="OK").wait.exists(timeout=3000):
            d(text="OK").click()
        return alarm_time

Clock = None
if ANDROID_VERSION is "M":
    Clock = ClockM
elif ANDROID_VERSION is "L":
    Clock = ClockM
elif ANDROID_VERSION is "N":
    Clock = ClockN
else:
    Clock = ClockN