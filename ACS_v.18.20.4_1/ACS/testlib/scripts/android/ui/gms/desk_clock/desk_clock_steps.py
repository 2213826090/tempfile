#!/usr/bin/env python

from testlib.scripts.android.ui import ui_steps
from testlib.base import base_utils
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui.browser import browser_utils
from testlib.scripts.android.adb import adb_utils

import time

class open_desk_clock_alarm(ui_step):
    """
        description:
            opens alarm tab under clock

        usage:
            open_desk_clock_alarm()()

        tags:
            android, desk_clock, clock, alarm
    """
    def do(self):
        ui_steps.press_home(serial = self.serial)()
        ui_steps.open_app_from_allapps(serial = self.serial,
                                    view_to_find = {"textContains": "Clock"},
                                    view_to_check = {"description": "Alarm"})()
        ui_steps.click_button(serial = self.serial,
                                view_to_find = {"description": "Alarm"})()

    def check_condition(self):
        return self.uidevice(description="Add alarm").exists

class open_desk_clock_stopwatch(ui_step):
    """
        description:
            opens stopwatch tab under clock

        usage:
            open_desk_clock_stopwatch()()

        tags:
            android, desk_clock, clock, alarm
    """
    def do(self):
        ui_steps.press_home(serial = self.serial)()
        ui_steps.open_app_from_allapps(serial = self.serial,
                                    view_to_find = {"textContains": "Clock"},
                                    view_to_check = {"description": "Stopwatch"})()
        ui_steps.click_button(serial = self.serial,
                                view_to_find = {"description": "Stopwatch"})()

    def check_condition(self):
        return self.uidevice(resourceId="com.android.deskclock:id/stopwatch_time_text").exists

class open_desk_clock_countdown(ui_step):
    """
        description:
            opens countdown timer tab under clock

        usage:
            open_desk_clock_countdown(serial = serial)()

        tags:
            android, desk_clock, clock, countdown
    """
    def do(self):
        ui_steps.press_home(serial = self.serial)()
        ui_steps.open_app_from_allapps(serial = self.serial,
                                    view_to_find = {"textContains": "Clock"},
                                    view_to_check = {"description": "Timer"})()
        ui_steps.click_button(serial = self.serial,
                                view_to_find = {"description": "Timer"})()

    def check_condition(self):
        return self.uidevice(resourceId="com.android.deskclock:id/desk_clock_pager").exists

class open_desk_clock_world_clock(ui_step):
    """
        description:
            opens world clock tab under clock

        usage:
            open_desk_clock_world_clock(serial = serial)()

        tags:
            android, desk_clock, clock, world clock
    """
    def do(self):
        ui_steps.press_home(serial = self.serial)()
        ui_steps.open_app_from_allapps(serial = self.serial,
                                    view_to_find = {"textContains": "Clock"},
                                    view_to_check = {"description": "Alarm"})()
        ui_steps.click_button(serial = self.serial,
                                view_to_find = {"description": "Clock"})()

    def check_condition(self):
        return self.uidevice(description="Cities").exists

class add_an_alarm(ui_step):
    """
        description:
            Adds a new alarm with specified parameters.
            By default, it will set the alarm for midnight.
            Time input is 24h format.

        usage:
            add_an_alarm(alarm_label="new_alarm", time_period="AM", hours=1,
                        minutes=42)()

        tags:
            android, desk_clock, clock, alarm
    """
    def __init__(self, hours=1, minutes = 1, alarm_label = "New Alarm Label",
            table = "alarm_templates",
            db = "/data/data/com.google.android.deskclock/databases/alarms.db",
            **kwargs):
            self.hours = hours
            self.minutes = minutes - minutes%5
            self.alarm_label = alarm_label
            self.table = table
            self.db = db
            self.errorm = "Alarm count did not increment as expected."
            ui_step.__init__(self, **kwargs)
    def do(self):
        ########################################################################
        # Must check for 12h/24h formats and set alarm accordingly.
        # Set new alarm.
        ########################################################################
        open_desk_clock_alarm(serial = self.serial)()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description": "Add alarm"})()
        if ui_utils.view_exists(serial = self.serial,view_to_find = {"text":"AM"}):
            if self.hours > 12:
                ui_steps.click_button(serial = self.serial,
                                    view_to_find = {"text":"PM"})()
                ui_steps.click_button(serial = self.serial,
                                    view_to_find = {"description":self.hours-12})()
            else:
                ui_steps.click_button(serial = self.serial,
                                    view_to_find = {"text":"AM"})()
                if self.hours == 0:
                    ui_steps.click_button(serial = self.serial,
                                    view_to_find = {"description":12})()
                else:
                    ui_steps.click_button(serial = self.serial,
                                    view_to_find = {"description":self.hours})()

        else:
            ui_steps.click_button(serial = self.serial,
                                view_to_find = {"description":self.hours})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description":self.minutes})()

        self.where = "label=\"{0}\" and hour={1} and minutes={2}".format(
                                                                    self.alarm_label,
                                                                    self.hours,
                                                                    self.minutes)
        self.initial_alarm_count = adb_utils.sqlite_count_query(db=self.db,
                                    table = self.table,
                                    where = self.where)

        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"textContains" : "OK"},
                            view_to_check = {"description": "Add alarm"})()
        if not self.uidevice(text="Label").exists:
            ui_steps.scroll_up_to_view(serial = self.serial,
                            view_to_check = {"text": "Label"},
                                    ey=250 )()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text": "Label"},
                            view_to_check = {"className":"android.widget.EditText"})()
        ui_steps.edit_text(serial = self.serial,
                            view_to_find = {"className":"android.widget.EditText"},
                            value = self.alarm_label)()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"textContains":"OK"},
                            view_to_check = {"description": "Add alarm"})()
        ui_steps.press_home(serial = self.serial)()

    def check_condition(self):
        return self.initial_alarm_count+1 == adb_utils.sqlite_count_query(
                                    db=self.db,
                                    table = self.table,
                                    where = self.where)

class delete_an_alarm(ui_step):
    """
        description:
            Deletes an alarm. If alarm label is provided, the first alarm found
            with that label will be removed. Otherwise, a random alarm will be
            chosen.


        usage:
            delete_an_alarm(alarm_label="new_alarm")()

        tags:
            android, desk_clock, clock, delete alarm
    """
    def __init__(self, alarm_label = "New Alarm Label",
            table = "alarm_templates",
            db = "/data/data/com.google.android.deskclock/databases/alarms.db",
            **kwargs):
            self.alarm_label = alarm_label
            self.table = table
            self.db = db
            self.errorm = "Alarm was not deleted."
            ui_step.__init__(self, **kwargs)
    def do(self):
        open_desk_clock_alarm(serial = self.serial)()
        self.initial_alarm_count = adb_utils.sqlite_count_query(db=self.db,
                                                            table = self.table)
        ########################################################################
        # try to find the alarm by label. If it doesn't exist, use a random one
        ########################################################################
        alarm_exists = ui_steps.scroll_up_to_view(serial = self.serial,
                            view_to_check = {"textContains": self.alarm_label},
                            ey=150, blocking=False, critical=False)
        alarm_exists()
        if alarm_exists.resolution == "[FAILED]":
            ui_steps.click_button(serial = self.serial,
                view_to_find = {"resourceId": "com.android.deskclock:id/arrow"})()
        else:
            if not ui_utils.view_exists(
                            {"resourceId":"com.android.deskclock:id/edit_label",
                             "textContains": self.alarm_label}, serial = self.serial):
                ui_steps.click_button(serial = self.serial,
                    view_to_find = {"textContains": self.alarm_label})()

        ui_steps.scroll_up_to_view(serial = self.serial,
                    view_to_check = {"resourceId": "com.android.deskclock:id/delete"},
                    ey=150 )()
        ui_steps.click_button(serial = self.serial,
                    view_to_find = {"resourceId": "com.android.deskclock:id/delete"},
                    view_to_check = {"text": "Alarm deleted."})()

    def check_condition(self):
        return self.initial_alarm_count-1 == adb_utils.sqlite_count_query(
                                    db=self.db,
                                    table = self.table)
class change_alarm_state(ui_step):
    """
        description:
            Changes an alarm's state to the one specified.
            Alarm is identified by label. If multiple alarms have the same label
            one of them will be selected randomy.
            If alarm already has the desired state, do nothing.

            alarm_state values:
             - 1 == "enabled"
             - 0 == "disabled"


        usage:
            change_alarm_state(serial = serial,
                            alarm_label= "new_alarm",
                            alarm_state = 1)()

        tags:
            android, desk_clock, clock, delete alarm
    """
    def __init__(self, alarm_label = "New Alarm Label",
            table = "alarm_templates",
            db = "/data/data/com.google.android.deskclock/databases/alarms.db",
            alarm_state = 0,
            **kwargs):
            self.alarm_label = alarm_label
            self.table = table
            self.db = db
            self.alarm_state = alarm_state
            self.errorm = "Alarm state not changed."
            self.possible_states = {1:"ON",
                            0: "OFF"}
            ui_step.__init__(self, **kwargs)


    def do(self):
        open_desk_clock_alarm(serial = self.serial)()

        ########################################################################
        # Check if alarm is already expanded or not (repeat should appear as
        # sibling of the desired label
        ########################################################################
        ui_steps.scroll_up_to_view(serial = self.serial,
                    view_to_check = {"textContains": self.alarm_label},
                    ey=150 )()
        ########################################################################
        # Hack for build 380: the label is printed with two trailing whitespaces
        # Checking for its presence with or without them
        ########################################################################

        if self.uidevice(text=self.alarm_label+"  "):
            repeat_exists = self.uidevice(\
                resourceId="com.android.deskclock:id/alarms_list").\
                child_by_text(self.alarm_label+"  ",resourceId="com.android.deskclock:id/alarm_item").\
                child(text="Repeat").exists
        else:
            repeat_exists = self.uidevice(\
                resourceId="com.android.deskclock:id/alarms_list").\
                child_by_text(self.alarm_label,resourceId="com.android.deskclock:id/alarm_item").\
                child(text="Repeat").exists
        ########################################################################
        # end of hack. To be written properly once the app no longer does this
        ########################################################################

        if not repeat_exists:
            ui_steps.click_button(
                view_to_find = {"textContains":self.alarm_label},
                view_to_check = {"text":"Repeat"})()

        ui_steps.scroll_up_to_view(serial = self.serial,
                    view_to_check = {"textContains": "Repeat"},
                    ey=150 )()
        is_state_correct = self.uidevice(resourceId="com.android.deskclock:id/alarms_list").\
                            child_by_text(self.alarm_label,resourceId="com.android.deskclock:id/alarm_item").\
                            child(textContains=self.possible_states[self.alarm_state]).exists
        if not is_state_correct:
            self.uidevice(resourceId="com.android.deskclock:id/alarms_list").\
                child_by_text(self.alarm_label,resourceId="com.android.deskclock:id/alarm_item").\
                child(resourceId = "com.android.deskclock:id/onoff").\
                click()


    def check_condition(self):
        stat = self.uidevice(resourceId="com.android.deskclock:id/alarms_list").\
                child_by_text(self.alarm_label,resourceId="com.android.deskclock:id/alarm_item").\
                child(textContains=self.possible_states[self.alarm_state]).exists
        return stat

class change_alarm_label(ui_step):
    """
        description:
            Changes an alarm's label to the one specified.
            Alarm is identified by label. If multiple alarms have the same label
            one of them will be selected randomy.


        usage:
            change_alarm_state(serial = serial,
                            alarm_label= "new_alarm",
                            new_label = "new_label")()

        tags:
            android, desk_clock, clock, delete alarm
    """
    def __init__(self, alarm_label = "Alarm Label",
            new_alarm_label = "Updated Alarm Label",
            **kwargs):
            self.alarm_label = alarm_label
            self.new_alarm_label = new_alarm_label
            self.errorm = "Alarm not edited."
            ui_step.__init__(self, **kwargs)


    def do(self):
        open_desk_clock_alarm(serial = self.serial)()

        ########################################################################
        # Check if alarm is already expanded or not (repeat should appear as
        # sibling of the desired label
        ########################################################################
        ui_steps.scroll_up_to_view(serial = self.serial,
                    view_to_check = {"textContains": self.alarm_label},
                    ey=150 )()
        if not self.uidevice(textContains = self.alarm_label).sibling(text="Repeat").exists:
            ui_steps.click_button(
                view_to_find = {"resourceId":"com.android.deskclock:id/arrow"},
                view_to_check = {"text":"Repeat"})()

        ui_steps.scroll_up_to_view(serial = self.serial,
                    view_to_check = {"resourceId": "com.android.deskclock:id/delete"},
                    ey=150 )()

        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"textContains": self.alarm_label},
                            view_to_check = {"className":"android.widget.EditText"})()
        ui_steps.edit_text(serial = self.serial,
                            view_to_find = {"className":"android.widget.EditText"},
                            value = self.new_alarm_label)()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"textContains":"OK"},
                            view_to_check = {"description": "Add alarm"})()


    def check_condition(self):
        return self.uidevice(resourceId = "com.android.deskclock:id/repeat_onoff")\
                .sibling(textContains = self.new_alarm_label).exists

class stopwatch_action(ui_step):
    """
        description:
            Performs an action on the stopwatch.
            Actions can be:
            - start
            - stop
            - reset

        usage:
            stopwatch_action(serial = serial,
                            action = "reset")()

        tags:
            android, desk_clock, clock, stopwatch
    """
    def __init__(self, action = "start",
            **kwargs):
            self.action = action
            self.action_dictionary = {
                        "start":
                            {'view_to_find':{"descriptionContains":"Start"},
                            'view_to_check':{"descriptionContains":"Stop"}},
                        "stop":
                            {'view_to_find':{"descriptionContains":"Stop"},
                            'view_to_check':{"descriptionContains":"Reset"}},
                        "reset":
                            {'view_to_find':{"descriptionContains":"Reset"},
                            'view_to_check':{"descriptionContains":"0 minutes 0 seconds"}}
                        }
            ui_step.__init__(self, **kwargs)


    def do(self):
        print self.action_dictionary[self.action]['view_to_find']
        open_desk_clock_stopwatch(serial = self.serial)()
        ui_steps.click_button(
            view_to_find = self.action_dictionary[self.action]['view_to_find'],
            view_to_check = self.action_dictionary[self.action]['view_to_check'])()

    def check_condition(self):
        check_resource = self.action_dictionary[self.action]['view_to_check']['descriptionContains']
        return self.uidevice(descriptionContains=check_resource).exists

class add_countdown_timer(ui_step):
    """
        description:
            Sets a countdown timer. If "label" is present, set it.

        usage:
            add_countdown_timer(hours="1", minutes="07", seconds="33")()

        tags:
            android, desk_clock, clock, countdown
    """
    def __init__(self, hours="1", minutes = "00", seconds="00", label = False,
            **kwargs):
            self.hours = hours
            self.minutes = minutes
            self.seconds = seconds
            self.label = label
            self.errorm = "Could not set countdown timer."
            ui_step.__init__(self, **kwargs)
    def do(self):
        open_desk_clock_countdown(serial = self.serial)()
        if ui_utils.view_exists(view_to_find = {"description":"Add Timer"}, serial = self.serial):
            ui_steps.click_button(view_to_find = {"description":"Add Timer"},
                    view_to_check = {"resourceId":"com.android.deskclock:id/hours_ones"})()

        to_be_clicked = list(self.hours+self.minutes + self.seconds)
        for item in to_be_clicked:
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":item,
                                        "resourceIdMatches":"com.android.deskclock:id/key_.*"})()

        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description": "Start"},
                            view_to_check = {"description":"Delete"})()
        if self.label:
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"resourceId":"com.android.deskclock:id/timer_label"},
                            view_to_check = {"resourceId":"com.android.deskclock:id/labelBox"})()
            ui_steps.edit_text(serial = self.serial,
                            view_to_find = {"resourceId":"com.android.deskclock:id/labelBox"},
                            value = self.label)()
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"textContains":"OK"},
                            view_to_check = {"description":"Delete"})()

    def check_condition(self):
        return self.uidevice(description="Delete").exists

class modify_timer(ui_step):
    """
        description:
            Modify a specific timer. Possible actions:
             - start OR stop OR reset OR delete
             AND
             - add minutes
             AND
             - change label (unless it's deleted)

        usage:
            modify_timer(extra_minutes=7,
                        new_label = False,
                        restart = True)()

        tags:
            android, desk_clock, clock, countdown
    """
    def __init__(self, extra_minutes = 0, new_label = False, delete = False,
            stop = False, start = False, reset = False,
            **kwargs):
            self.extra_minutes = extra_minutes
            self.label = kwargs["label"]
            self.new_label = new_label
            self.delete = delete
            self.start = start
            self.stop = stop
            self.reset = reset
            self.errorm = "Could not edit timer."
            ui_step.__init__(self, **kwargs)
    def do(self):
        open_desk_clock_countdown(serial = self.serial)()
        if ui_utils.view_exists(view_to_find = {"description":"Cancel"}, serial = self.serial):
            ui_steps.click_button(view_to_find = {"description":"Cancel"},
                    view_to_check = {"description":"Add Timer"})()

        ui_steps.scroll_up_to_view(serial = self.serial,
                view_to_check = {"resourceId": "com.android.deskclock:id/timer_label_text",
                                "text":self.label},
                ey=150 )()

        if self.start and self.uidevice(description="Start").exists:
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description": "Start"},
                            view_to_check = {"description":"Stop"})()
        elif self.stop and self.uidevice(description="Stop").exists:
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description": "Stop"},
                            view_to_check = {"description":"Start"})()
        elif self.reset and self.uidevice(description="Reset").exists:
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description": "Reset"})()
        elif self.delete:
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description": "Delete"})()
        if self.extra_minutes > 0:
            for _ in xrange(0,self.extra_minutes):
                ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description": "Add 1 Minute"})()
        if self.new_label and not self.delete:
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"resourceId":"com.android.deskclock:id/timer_label"},
                            view_to_check = {"resourceId":"com.android.deskclock:id/labelBox"})()
            ui_steps.edit_text(serial = self.serial,
                            view_to_find = {"resourceId":"com.android.deskclock:id/labelBox"},
                            value = self.new_label)()
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"textContains":"OK"},
                            view_to_check = {"description":"Delete"})()


    def check_condition(self):
        return self.uidevice(description="Delete").exists

class modify_world_clock(ui_step):
    """
        description:
            Add one or more world clock(s).
            Cities to be selected / deselected is provided as dictionary.
        usage:
            modify_world_clock(serial = serial,
                        cities = {"Bucharest":"off", "London":"on"})()

        tags:
            android, desk_clock, clock, world clock
    """
    def __init__(self, **kwargs):
            self.cities = kwargs["cities"]
            self.errorm = "Could not add cities."
            ui_step.__init__(self, **kwargs)
    def do(self):
        open_desk_clock_world_clock(serial = self.serial)()
        for city_name, desired_status in self.cities.iteritems():
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description": "Cities"},
                            view_to_check = {"description":"Navigate up"})()
            ui_steps.scroll_up_to_view(serial = self.serial,
                view_to_check = {"text": city_name},
                ey=50,
                iterations = 200)()
            city_checked = self.uidevice(text=city_name).\
                    sibling(resourceId="com.android.deskclock:id/city_onoff").\
                    info["checked"]
            if (desired_status == "off" and city_checked) or \
                (desired_status == "on" and not city_checked):
                ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text": city_name})()
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description":"Navigate up"},
                            view_to_check = {"description": "Cities"})()

    def check_condition(self):
        check_result = True
        for city_name,desired_status in self.cities.iteritems():
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description": "Cities"},
                            view_to_check = {"text":"Selected Cities"})()
            ui_steps.scroll_up_to_view(serial = self.serial,
                view_to_check = {"text": city_name},
                ey=50,
                iterations = 200)()
            city_checked = self.uidevice(text=city_name).\
                    sibling(resourceId="com.android.deskclock:id/city_onoff").\
                    info["checked"]
            if (desired_status == "on" and not city_checked) or \
                (desired_status == "off") and city_checked:
                check_result = False
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description":"Navigate up"},
                            view_to_check = {"description": "Cities"})()
        return check_result

class set_clock_type(ui_step):
    """
        description:
            Sets clock type to a specific value
        usage:
            set_clock_type(serial = serial,
                        clock_type = "digital")()
            clock_type can be:
            - "Digital"
            - "Analog"

        tags:
            android, desk_clock, clock, clock type
    """
    def __init__(self, clock_type = "Digital", **kwargs):
            self.clock_type = clock_type
            ui_step.__init__(self, **kwargs)
    def do(self):
        open_desk_clock_world_clock(serial = self.serial)()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description": "More options"},
                            view_to_check = {"text":"Settings"})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Settings"},
                            view_to_check = {"text":"Style"})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Style"},
                            view_to_check = {"text":self.clock_type})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":self.clock_type},
                            view_to_check = {"description":"Navigate up"})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description":"Navigate up"})()


    def check_condition(self):
        return self.uidevice(resourceId="com.android.deskclock:id/"+self.clock_type.lower()+"_clock").exists

class set_auto_home_clock(ui_step):
    """
        description:
            Sets automatic home clock

        usage:
            set_auto_home_clock(serial = serial,
                        ahc = "on")()
            clock_type can be:
            - "off"
            - "on"

        tags:
            android, desk_clock, clock, automatic home clock
    """
    def __init__(self, ahc = "on", **kwargs):
            self.ahc = ahc
            self.settings = {"on":"true",
                            "off":"false"}
            ui_step.__init__(self, **kwargs)
    def do(self):
        open_desk_clock_world_clock(serial = self.serial)()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description": "More options"},
                            view_to_check = {"text":"Settings"})()

        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Settings"},
                            view_to_check = {"text":"Style"})()
        if not self.uidevice(text="Automatic home clock").\
            sibling(resourceId="android:id/checkbox",checked = self.settings[self.ahc]).exists:
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Automatic home clock"})()

    def check_condition(self):
        return self.uidevice(text="Automatic home clock").\
            sibling(resourceId="android:id/checkbox",checked = self.settings[self.ahc]).exists

class set_automatic_home_clock(ui_step):
    """
        description:
            Sets home time zone

        usage:
            set_auto_home_clock(serial = serial,
                        ahc = "on")()
            clock_type can be:
            - "off"
            - "on"

        tags:
            android, desk_clock, clock, automatic home clock
    """
    def __init__(self, ahc = "on", **kwargs):
            self.ahc = ahc
            self.settings = {"on":"true",
                            "off":"false"}
            ui_step.__init__(self, **kwargs)
    def do(self):
        open_desk_clock_world_clock(serial = self.serial)()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description": "More options"},
                            view_to_check = {"text":"Settings"})()

        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Settings"},
                            view_to_check = {"text":"Style"})()
        if not self.uidevice(text="Home time zone", enabled = self.settings[self.ahc]).exists:
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Automatic home clock"})()

    def check_condition(self):
        return self.uidevice(text="Home time zone", enabled = self.settings[self.ahc]).exists

class set_home_timezone(ui_step):
    """
        description:
            Sets home time zone

        usage:
            set_home_timezone(serial = serial,
                        timezone = "London")()
            clock_type can be:
            - "off"
            - "on"

        tags:
            android, desk_clock, clock, automatic home clock
    """
    def __init__(self, timezone = "London", **kwargs):
            self.timezone = timezone
            ui_step.__init__(self, **kwargs)
    def do(self):
        open_desk_clock_world_clock(serial = self.serial)()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"description": "More options"},
                            view_to_check = {"text":"Settings"})()

        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Settings"},
                            view_to_check = {"text":"Style"})()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Home time zone"},
                            view_to_check = {"text" : "Cancel"})()
        if not self.uidevice(textContains = self.timezone).exists:
            self.uidevice(scrollable=True).scroll.vert.to(textContains = self.timezone)
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"textContains":self.timezone},
                            view_to_check = {"text" : "Home time zone"})()
    def check_condition(self):
        return self.uidevice(text="Home time zone").\
            sibling(resourceId="android:id/summary",textContains = self.timezone).exists

class check_home_clock_exists(ui_step):
    """
        description:
            checks home timezone exists and compares the result with an expected
            outcome.

        usage:
            check_home_clock_exists(serial = serial,
                                    expected = True )()

        tags:
            android, desk_clock, clock, automatic home clock
    """
    def __init__(self, expected = True, **kwargs):
            self.expected = expected
            ui_step.__init__(self, **kwargs)
    def do(self):
        open_desk_clock_world_clock(serial = self.serial)()

    def check_condition(self):
        return self.uidevice(resourceId="com.android.deskclock:id/city_name").\
            sibling(text="Home").exists == self.expected
