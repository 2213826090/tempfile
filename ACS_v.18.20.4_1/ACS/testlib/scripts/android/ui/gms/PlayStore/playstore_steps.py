#!/usr/bin/env python

##############################################################################
#
# @filename:    playstore_steps.py
# @description: UI Playstore test steps
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################

from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.gms.PlayStore import playstore_utils
from testlib.scripts.android.ui.ui_step import step as ui_step
import random


class install_playstore_app(ui_step):

    """ description:
            opens Play store application and installs <app_name>
            application. it will install it in the given timeout:
            <app_install_time>.
            if the app is already installed it will fail

        usage:
            gms_steps.install_playstore_app(app_name = "File Manager",
                                            app_description =\
                                            "File Manager (Explorer)",
                                            app_install_time = 10000)()

        tags:
            ui, android, playstore, app, application, install
    """

    def __init__(self, app_name, app_description, app_install_time, **kwargs):
        self.app_name = app_name
        self.app_description = app_description
        self.app_install_time = app_install_time * 1000
        self.set_passm(str(app_name))
        self.set_errorm("", str(app_name))
        ui_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.open_playstore(serial = self.serial)()

        ui_steps.edit_text(serial = self.serial,
                           view_to_find = {"description":
                                           "Search"},
                           value = self.app_name)()
        self.uidevice.press("enter")

        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"textContains": self.app_description},
                              view_to_check = {"text":"INSTALL"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text":"INSTALL"},
                              view_to_check = {"text":"ACCEPT"},
                              wait_time = 30000)()

        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text":"ACCEPT"})()

        self.uidevice(text = "OPEN").wait.exists(timeout = self.app_install_time)

    def check_condition(self):
        return self.uidevice(text = "UNINSTALL").exists


class uninstall_playstore_app(ui_step):

    """ description:
            opens Play store application and uninstalls <app_name>
            application. it will uninstall it in the given timeout:
            <app_install_time>.
            the app needs to be installed first

        usage:
            gms_steps.uninstall_playstore_app(app_name = "File Manager",
                                              app_description =\
                                              "File Manager (Explorer)",
                                              app_uninstall_time = 10000)()

        tags:
            ui, android, playstore, app, application, unistall
    """

    def __init__(self, app_name, app_description, app_uninstall_time, **kwargs):
        self.app_name = app_name
        self.app_description = app_description
        self.app_uninstall_time = app_uninstall_time * 1000
        self.set_passm(str(app_name))
        self.set_errorm("", str(app_name))
        ui_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.open_playstore(serial = self.serial)()

        ui_steps.edit_text(serial = self.serial,
                           view_to_find = {"description":
                                           "Search"},
                           value = self.app_name)()
        self.uidevice.press("enter")
        self.uidevice.wait.idle()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"textContains":self.app_description},
                              view_to_check = {"text":"UNINSTALL"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text":"UNINSTALL"},
                              view_to_check = {"text":"OK"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text":"OK"})()
        self.uidevice(text = "INSTALL").wait.exists(timeout = self.app_uninstall_time)

    def check_condition(self):
        ui_steps.press_all_apps(serial = self.serial)()
        return not ui_utils.is_text_visible_scroll_left(serial = self.serial,
                                                        text_to_find = self.app_name)


class playstore_add_to_wishlist(ui_step):

    """ description:
            adds the given <app_name> to wishlist.
            if <app_name> is not provided and <random> is True, it will addd
            a random app (from the top row) to wishlist.

        usage:
            gms_steps.change_playstore_category(category_name = "Apps",
                                                view_to_check = {"text":
                                                                 "Smth"})()

        tags:
            ui, android, playstore, app, application, category
    """

    def __init__(self, random_app = False, app_name = None, **kwargs):
        if not random_app:
            if app_name:
                self.app_name = app_name
            else:
                self.set_errorm("", "App name is not provided")
            self.app_no = None
        else:
            self.app_name = None
            self.app_no = random.randint(0,5)
            self.set_errorm("", "App number " + str(self.app_no + 1))
            self.set_passm("App number " + str(self.app_no + 1))
        ui_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "MORE"},
                              view_to_check = {"text": "MORE"},
                              view_presence = False)()
        if self.app_no <> None:
            while True:
                self.uidevice(resourceId = "com.android.vending:id/li_subtitle").wait.exists(timeout = 10000)
                item_no = self.uidevice(resourceId = "com.android.vending:id/li_subtitle").count
                if self.app_no > item_no:
                    self.app_no = item_no
                app_view = self.uidevice(resourceId = "com.android.vending:id/li_subtitle", instance = self.app_no)
                self.step_data = app_view.info["text"]
                ui_steps.click_view(serial = self.serial,
                                    view = app_view)()
                if self.uidevice(description = "Include in wishlist").exists:
                    break
                else:
                    self.app_no = random.randint(0, item_no - 1)
                    ui_steps.press_back(serial = self.serial)()

        else:
            self.step_data = self.app_name
            app_view = self.uidevice(text = self.app_name)
            ui_steps.wait_for_view(view_to_find = {"resourceId":
                "com.android.vending:id/search_button"}, timeout = 30)()

            ui_steps.edit_text(serial = self.serial,
                           view_to_find = {"resourceId":
                                           "com.android.vending:id/search_button"},
                           value = self.app_name)()
            self.uidevice.press("enter")

            ui_steps.click_button(serial = self.serial,
                              view_to_find = {"textContains": self.app_name},
                              view_to_check = {"text": "INSTALL"})()

        if self.uidevice(descriptionContains=\
                    "Remove from wishlist").wait.exists(timeout=3000):
                    ui_steps.click_button(serial = self.serial,
                        view_to_find = {"descriptionContains": "Remove from wishlist"},
                        view_to_check ={"descriptionContains": "Include in wishlist"})()

        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"description": "Include in wishlist"},
                              view_to_check = {"description": "Remove from wishlist"})()

    def check_condition(self):
        while True:
            ui_steps.press_back(serial = self.serial)()
            if self.uidevice(description = "Show navigation drawer").exists:
                break
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"description": "Show navigation drawer"},
                              view_to_check = {"text": "My wishlist"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "My wishlist"},
                              view_to_check = {"text": self.step_data})()
        return self.uidevice(text = self.step_data).exists


class playstore_remove_from_wishlist(ui_step):

    """ description:
            adds the given <app_name> to wishlist.
            if <app_name> is not provided and <random> is True, it will addd
            a random app (from the top row) to wishlist.

        usage:
            gms_steps.change_playstore_category(category_name = "Apps",
                                                view_to_check = {"text":
                                                                 "Smth"})()

        tags:
            ui, android, playstore, app, application, category
    """

    def __init__(self, random_app = False, app_name = None, **kwargs):
        if not random_app:
            if app_name:
                self.app_name = app_name
            else:
                self.set_errorm("", "App name is not provided")
            self.app_no = None
        else:
            self.app_name = None
            self.app_no = random.randint(0, 5)
            self.set_errorm("", "App number " + str(self.app_no + 1))
            self.set_passm("App number " + str(self.app_no + 1))
        ui_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"description": "Show navigation drawer"},
                              view_to_check = {"text": "My wishlist"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "My wishlist"},
                              view_to_check = {"resourceId": "com.android.vending:id/play_card"})()

        if self.app_name:
            self.step_data = self.app_name
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text": self.app_name},
                                  view_to_check = {"descriptionContains": "wishlist"})()
        else:
            item_no = self.uidevice(resourceId = "com.android.vending:id/play_card").count
            if item_no > 9:
                item_no = 5
            else:
                item_no -= 1
            self.app_no = random.randint(0, item_no)
            app_view = self.uidevice(resourceId = "com.android.vending:id/play_card", instance = self.app_no)
            self.step_data = app_view.info["text"]
            ui_steps.click_view(serial = self.serial,
                                view = app_view,
                                view_to_check = {"descriptionContains": "wishlist"})()
        ui_steps.click_button(serial = self.serial,
                          view_to_find = {"description": "Remove from wishlist"},
                          view_to_check = {"description": "Include in wishlist"})()
        ui_steps.press_back(serial = self.serial)()

    def check_condition(self):
        self.uidevice.wait.idle()
        return not self.uidevice(text = self.step_data).exists
