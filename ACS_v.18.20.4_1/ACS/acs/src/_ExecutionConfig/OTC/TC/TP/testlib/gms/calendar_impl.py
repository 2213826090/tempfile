import time
import os
from nose.tools import assert_equals
from testlib.util.common import g_common_obj



class CalendarImpl:
    """
    Implements Mail app actions.
    """

    packagename = "com.google.android.calendar"
    activityname = "com.android.calendar.AllInOneActivity"

    def __init__ (self, cfg={}):
        self.d = g_common_obj.get_device()
        self.cfg=cfg

    # launch contact app by am
    def launch_by_am(self):
        g_common_obj.launch_app_am(CalendarImpl.packagename, \
            CalendarImpl.activityname)
        print "[INFO] launch calendar app successfully"
    def stop_from_am(self):
        """
            Stop Calendar from am
        """
        print "[INFO] Stop Calendar from am"
        g_common_obj.stop_app_am(CalendarImpl.packagename)

    @staticmethod
    def startApp():
        """
        Skip Calendar first launch screen
        """
        d = g_common_obj.get_device()
        g_common_obj.launch_app_from_home_sc("Calendar")
        if d(resourceId="com.android.calendar:id/add_event_button").exists:
            return
        time.sleep(10)
        if d(resourceId="com.android.calendar:id/add_event_button").exists:
            return
        while d(resourceId="com.android.calendar:id/right_arrow").wait.exists(timeout=3000):
            d(resourceId="com.android.calendar:id/right_arrow").click()
            d.wait.update()
        if d(
            resourceId="com.android.calendar:id/done_button").\
        wait.exists(timeout=60000):
            d(resourceId="com.android.calendar:id/done_button").click()
        g_common_obj.back_home()

    def event_add(self, title, waittime):
        """
        @summary: add calendar event
        @parameter:
            waittime : the waittime between current and event start time
        @return: None
        """
        print("[INFO] Add event %s" % title)
        os.system("adb logcat -c")
        title = "-e title " + "\"" + title + "\""
        event_stime = self.__get_date()*1000 + int(waittime)*1000
        event_add = self.cfg.get("cmd_addevent") + " " + title + \
        " --el beginTime " + str(event_stime) + ";echo $?"
        mesg = g_common_obj.adb_cmd_capture_msg(event_add)
        assert mesg.find("Error:") == -1, \
        "ERROR:Event added command fail!"
        self.d(text = "Save").click()

    def event_getnum(self, title):
        """
        @summary: get event number from logcat
        """
        key_mesg = "act=android.intent.action.VIEW dat=content://com.android.calendar/events/"
        os.system("adb logcat -c")
        self.d.open.notification()
        self.d.open.notification()
        if self.d(text=title, \
            packageName="com.android.systemui").wait.exists(timeout=60000):
            self.d(text=title, instance="0", \
                packageName="com.android.systemui").click()
        time.sleep(5)
        cmd = "adb logcat -d|grep \"" + key_mesg + "\""
        mesg = os.popen(cmd).read().split("\n")
        #print "DEBUG:", mesg
        assert len(mesg) >= 2, "ERROR: grep return %s" % mesg
        mesg = mesg[0]
        numlist = mesg.split()[5]
        num = numlist.split("/")[-1]
        print("[INFO] the event number is %d" % int(num))
        return int(num)

    def event_show(self, num):
        """
        @summary: show the event
        """
        print("[INFO] Show event")
        event_show = self.cfg.get("cmd_showevent") + str(num) + ";echo $?"
        mesg = g_common_obj.adb_cmd_capture_msg(event_show)
        assert mesg.find("Error:") == -1, \
        "ERROR:Event show command fail!"

    def event_edit(self):
        """
        @summary: edit the event and save
        """
        print("[INFO] Edit event")
        self.d(resourceId="com.android.calendar:id/edit", className="android.widget.ImageButton").long_click()
        #self._event.btn_color.click()
        #self._event.btn_color_icon.click()
        self.d(resourceId="com.android.calendar:id/title").clear_text()
        self.d(resourceId="com.android.calendar:id/title").set_text(self.cfg.get("c_title"))
        self.d(resourceId="com.android.calendar:id/location").clear_text()
        self.d(resourceId="com.android.calendar:id/location").set_text(self.cfg.get("c_location"))
        self.d(resourceId="com.android.calendar:id/start_date").click()
        self.__back_key()
        self.d(resourceId="com.android.calendar:id/start_time").click()
        self.__back_key()
        self.d(resourceId="com.android.calendar:id/end_date").click()
        self.__back_key()
        self.d(resourceId="com.android.calendar:id/end_time").click()
        self.__back_key()
        self.d(resourceId="com.android.calendar:id/is_all_day").click()
        self.d(resourceId="com.android.calendar:id/is_all_day").click()
        self.d(resourceId="com.android.calendar:id/timezone_button").click()
        time.sleep(2)
        self.__back_key()
        time.sleep(2)
        self.__back_key()
        self.d(resourceId="com.android.calendar:id/attendees").clear_text()
        self.d(resourceId="com.android.calendar:id/attendees").set_text(self.cfg.get("c_guests"))
        self.d(resourceId="com.android.calendar:id/description").clear_text()
        self.d(resourceId="com.android.calendar:id/description").set_text(self.cfg.get("c_description"))
        self.d(resourceId="com.android.calendar:id/rrule").click()
        self.__back_key()
        self.d(resourceId="com.android.calendar:id/reminder_minutes_value").click()
        self.__back_key()
        self.d(resourceId="com.android.calendar:id/reminder_method_value").click()
        self.__back_key()
        self.d(resourceId="com.android.calendar:id/availability").click()
        self.__back_key()
        self.d(resourceId="com.android.calendar:id/visibility").click()
        self.__back_key()
        self.d(resourceId="com.android.calendar:id/action_done").click()

    def event_delete(self, num):
        """
        @summary: delete the event
        """
        print("[INFO] Delete event!")
        self.d(resourceId="com.android.calendar:id/info_action_edit").click()
        time.sleep(1)
        self.d(resourceId= "com.android.calendar:id/delete").click()
        self.d(text = "OK").click()
        self.event_show(num)
        time.sleep(2)
        assert not self.d(
            resourceId="com.android.calendar:id/info_action_edit").exists,\
        "ERROR:Delete event error!"

    def check_reminder(self, title, waittime):
        """
        @summary: check the reminder
        """

        if waittime <= 60:
            time_last = waittime
        else:
            count = waittime / 60
            time_last = waittime % 60
            if time_last == 0:
                time_last = 60
                count -= 1
            for _ in range (0, count):
                print("[INFO] Waiting 60s")
                time.sleep(60)
        self.d.open.notification()
        reminder = False
        while time_last >= 0:
            if self.d(text=title).exists:
                reminder = True
                break
            print("[INFO] Check Reminder!")
            time.sleep(5)
            time_last -= 5
        assert reminder, "ERROR: Reminder does not come out"

    @staticmethod
    def __back_key():
        """
        press back key
        """
        g_common_obj.adb_cmd("input keyevent 4")

    @staticmethod
    def __get_date():
        """
        @summary: get device current time
        @return: million seconds
        """
        cmd = "date +'%'s"
        pipe = g_common_obj.adb_cmd_capture_msg(cmd)
        #print "DEBUG", pipe, type(pipe), int(pipe)
        return int(pipe)

    def account_cancel(self, account):
        """
        @summary: change calendar account
        @parameter:
            account : account to account_changed
        @return: None
        """
        print("[INFO] Account cancel %s" % account)
        self.launch_from_am()
        self._locator.btn_more.click()
        self._locator.btn_settings.click()
        assert self.d(text=account).exists, \
        "Please make sure the account in conf has been signed up in device"
        self.d(text=account).click()
        if self._locator.btn_synccheck.exists:
            self._locator.btn_synccheck.click()
        else:
            print("[WARNING] The account[%s] is not sync.Skip cancel the account" % account)
        self.d.press.back()

    def account_sync(self, account):
        """
        @summary: change calendar account
        @parameter:
            account : account to account_changed
        @return: None
        """
        print("[INFO] Account sync %s" % account)
        self.launch_from_am()
        self._locator.btn_more.click()
        self._locator.btn_settings.click()
        assert self.d(text=account).exists, \
        "Please make sure the account in conf has been signed up in device"
        self.d(text=account).click()
        if self._locator.btn_unsynccheck.exists:
            self._locator.btn_unsynccheck.click()
        else:
            print("[WARNING] The account[%s] is sync.Skip sync the account" % account)
        self.d.press.back()