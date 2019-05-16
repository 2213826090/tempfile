#!/usr/bin/env python

from testlib.scripts.android.ui import ui_steps
from testlib.base import base_utils
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.browser import browser_utils
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.gms.google_search import google_search_utils

import time

class open_search(ui_step):
    """
        description:
            Opens search app.

        usage:
            open_search(serial = serial,
                            from_location = "homescreen")()

        tags:
            android, search web
    """
    def __init__(self, wait_time = 20000,
                from_location = "homescreen",
                **kwargs):
        self.from_location = from_location
        self.wait_time = wait_time
        ui_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.press_home(serial = self.serial)()
        ui_steps.click_button(serial = self.serial,
                    view_to_find = {"resourceIdMatches":".*launcher_search_button.*"})()
        if self.uidevice(text="Skip").wait.exists(timeout = 20000):
            self.uidevice(text="Skip").click()
        else:
            self.uidevice(textContains = "Search, or say ").wait.exists(timeout = 20000)
    def check_condition(self):
        return self.uidevice(textContains="Search, or say").\
                wait.exists(timeout = self.wait_time)

class search_web_from_home(ui_step):
    """
        description:
            Searches a keyword from homescreen.

        usage:
            search_web_from_home(serial = serial,
                            keyword = "somekeyword")()

        tags:
            android, search web
    """
    def __init__(self, keyword = "test", wait_time = 20000, **kwargs):
        self.keyword = keyword
        self.wait_time = wait_time
        ui_step.__init__(self, **kwargs)
    def do(self):
        open_search(serial = self.serial,
                    wait_time = self.wait_time)()
        ui_steps.edit_text(serial = self.serial,
                        view_to_find = {"textContains":"Search, or say "},
                        value = self.keyword)()
        self.uidevice.press("enter")

    def check_condition(self):
        self.uidevice(text="Web").wait.exists(timeout = self.wait_time)
        return self.uidevice(text="Web").exists and\
                self.uidevice(text="Images").exists and\
                self.uidevice(text="Maps").exists and\
                self.uidevice(text="Videos").exists and\
                self.uidevice(text="MORE").exists

class search_apps_from_home(search_web_from_home):
    """
        description:
            Searches apps for a keyword.

        usage:
            search_apps_from_home(serial = serial,
                            keyword = "somekeyword")()

        tags:
            android, search app
    """
    def __init__(self, keyword = "test", **kwargs):
        search_web_from_home.__init__(self, keyword = keyword, **kwargs)
    def do(self):
        max_steps = 10
        search_web_from_home.do(self)
        ui_steps.click_button(serial = self.serial,
                    view_to_find = {"text":"MORE"},
                    view_to_check = {"text":"News"})()
        while not self.uidevice(text="Apps").wait.exists(timeout = 1000) and max_steps > 0:
            self.uidevice(text="News").swipe.left()
            max_steps += -1
        ui_steps.click_button(serial = self.serial,
                    view_to_find = {"text":"Apps"})()
    def check_condition(self):
        return self.uidevice(text="Apps").info["selected"] == True

class search_contact(search_web_from_home):
    """
        description:
            Searches apps for a keyword.

        usage:
            search_apps_from_home(serial = serial,
                            keyword = "somekeyword")()

        tags:
            android, search app
    """
    def __init__(self, keyword = "test", **kwargs):
        search_web_from_home.__init__(self, keyword = keyword, **kwargs)
    def do(self):
        max_steps = 10
        open_search(serial = self.serial,
                    wait_time = self.wait_time)()
        ui_steps.edit_text(serial = self.serial,
                        view_to_find = {"textContains":"Search, or say "},
                        value = self.keyword)()
    def check_condition(self):
        return self.uidevice(resourceIdMatches=".*search_suggestions_summons").\
                    child(textContains=self.keyword).wait.\
                    exists(timeout = self.wait_time)

class search_tablet_from_home(search_web_from_home):
    """
        description:
            Searches a keyword from homescreen.

        usage:
            search_web_from_home(serial = serial,
                            keyword = "somekeyword")()

        tags:
            android, search web
    """
    def __init__(self, keyword = "test", **kwargs):
        search_web_from_home.__init__(self, keyword = keyword, **kwargs)
    def do(self):
        max_steps = 10
        search_web_from_home.do(self)
        if self.uidevice(text="MORE").wait.exists(timeout = self.wait_time):
            ui_steps.click_button(serial = self.serial,
                        view_to_find = {"text":"MORE"},
                        view_to_check = {"text":"News"})()
            while not self.uidevice(text="Tablet").wait.exists(timeout = 1000) and max_steps > 0:
                self.uidevice(text="News").swipe.left()
                max_steps += -1
            ui_steps.click_button(serial = self.serial,
                        view_to_find = {"text":"Tablet"})()
        else:
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":self.keyword},
                            view_to_check = {"text":"Search Tablet"})()
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Search Tablet"})()

    def check_condition(self):
        if self.uidevice(text="No results found\non your tablet.").\
                    wait.exists(timeout = self.wait_time):
            return True
        elif self.uidevice(resourceId="com.google.android.googlequicksearchbox:id/main_content_back").\
                child_by_text("Apps", resourceId="com.google.android.googlequicksearchbox:id/cards_view").\
                child(text=self.keyword).wait.exists(timeout = self.wait_time):
            return True
        else:
            return False
