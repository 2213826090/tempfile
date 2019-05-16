#!/usr/bin/env python

from testlib.scripts.android.ui import ui_steps
from testlib.base import base_utils
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui.browser import browser_utils
from testlib.utils.defaults import wifi_defaults
import time


class open_new_tab(ui_step):
    """
        description:
            opens an empty new tab in browser (chrome)

        usage:
            open_new_tab()()

        tags:
            android, browser, chrome, new, tab
    """
    def do(self):
        empty_view = {"resourceId":
                      "com.android.chrome:id/empty_new_tab_button"}
        if self.uidevice(**empty_view).exists:
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = empty_view)()
        else:
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"resourceId":
                                                  "com.android.chrome:id/menu_button"},
                                  wait_time = 3000,
                                  view_to_check = {"textContains":"New incognito tab"})()
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text": "New tab"})()

    def check_condition(self):
        self.uidevice(text = "Search or type URL").exists


class close_all_tabs(ui_step):
    """
        description:
            close all browser tabs

        usage:
            close_all_tabs()()

        tags:
            android, browser, chrome, all, tab, close
    """
    def do(self):
        while not self.uidevice(resourceId = "com.android.chrome:"
                                             "id/empty_new_tab_button"):
            close_tab()()
        self.uidevice.wait.update()
        time.sleep(1)

    def check_condition(self):
        return self.uidevice(resourceId = "com.android.chrome:"
                                          "id/empty_new_tab_button").exists


class close_tab(ui_step):
    """
        description:
            close the active browser tab

        usage:
            close_tab()()

        tags:
            android, browser, chrome, tab, close
    """
    def do(self):
        self.step_data = browser_utils.get_open_tabs_no(serial = self.serial)
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"description": "Close tab"})()

    def check_condition(self):
        open_tabs_no = browser_utils.get_open_tabs_no(serial = self.serial)
        return self.step_data - open_tabs_no == 1


class go_back(ui_step):
    """
        description:
            presses <go back> button in browser
            it checks if <go forward> button gets activated

        usage:
            go_back()()

        tags:
            android, browser, chrome, back
    """
    def __init__(self, wait_time = 2000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time/1000

    def do(self):
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"description": "Go back"})()
        time.sleep(self.wait_time)

    def check_condition(self):
        forward_button = self.uidevice(description = "Go forward")
        return forward_button.info["enabled"]


class press_print_button(ui_step):
    """
        description:
            presses print button from browser settings

        usage:
            press_print_button()()

        tags:
            android, browser, chrome, print, press, settings
    """
    def do(self):
        open_browser_settings(serial = self.serial)()
        self.uidevice(textContains = "Print").wait.exists(timeout = 2000)
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"textContains": "Print"})()

    def check_condition(self):
        first_page = self.uidevice(textContains = "Save as PDF").exists
        self.uidevice(text = "Print jobs").wait.exists(timeout = 5000)
        second_page = self.uidevice(text = "Print jobs").exists
        return first_page and second_page


class go_forward(ui_step):
    """
        description:
            presses <go forward> button in browser
            it first presse <go_back> and then <go forward> button
            and checks if the latest url title is displayed

        usage:
            go_forward(url_title = "Google Translate")()

        tags:
            android, browser, chrome, back, forward
    """
    def __init__(self, url_title, wait_time = 2000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.url_title = url_title
        self.wait_time = wait_time/1000

    def do(self):
        go_back()()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"description": "Go forward"})()
        time.sleep(self.wait_time)

    def check_condition(self):
        return self.uidevice(textContains= self.url_title).exists


class refresh_button(ui_step):
    """
        description:
            checks the refresh button works

        usage:
            refresh_button()()

        tags:
            android, browser, chrome, url, refresh
    """
    def do(self):
        self.uidevice(description = "Search").wait.exists()
        ui_steps.edit_text(serial = self.serial,
                           view_to_find = {"description": "Search"},
                           value = "bogus",
                           is_password = True)() #in order not to check the text
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"description": "Refresh page"})()

    def check_condition(self):
        return self.uidevice(description = "Search").info["text"] == ""


class open_browser(ui_step, adb_step):
    """
        description:
            opens the browser on the device (default Chrome)

        usage:
            open_browser()()

        tags:
            android, browser, chrome, open
    """
    def __init__(self, browser = "Chrome", browser_process = "chrome",
                 view_to_check = {"packageName": "com.android.chrome"},**kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.browser = browser
        self.browser_process = browser_process
        self.view_to_check = view_to_check

    def do(self):
        browser_pid_list = self.adb_connection.pgrep(self.browser_process)
        self.adb_connection.kill_all(browser_pid_list)
        ui_steps.open_app_from_allapps(serial = self.serial,
                                       view_to_find = {"text": self.browser},
                                       wait_time = 10000,
                                       view_to_check = self.view_to_check)()

    def check_condition(self):
        return self.adb_connection.get_pid(self.browser_process) != None


class open_chrome_first_time(ui_step):

    def __init__(self, intent = False, url_to_open = None, **kwargs):
        self.intent = intent
        self.url_to_open = url_to_open
        ui_step.__init__(self, **kwargs)

    def do(self):
        if self.intent:
            adb_steps.am_start_command(serial = self.serial,
                                       component = "com.android.chrome/com.google.android.apps.chrome.Main")()
        else:
            open_browser(serial = self.serial)()
        self.uidevice.wait.idle()

        # Check if we have the Chrome first time pop-up.
        if self.uidevice(**self.device_info.chrome_accept_welcome_btn_id).wait.exists(timeout = 10000):
            ui_steps.click_button(serial = self.serial,
                                view_to_find = self.device_info.chrome_accept_welcome_btn_id)()

            if self.uidevice(**self.device_info.chrome_welcome_sign_in_no_thanks_btn_id).wait.exists(timeout = 10000):
                pass
            elif self.uidevice(text = "Next").wait.exists(timeout = 10000):
                ui_steps.click_button(serial = self.serial,
                                view_to_find = {"text": "Next"},
                                view_to_check = {"text": "Set up Chrome"})()
            ui_steps.click_button(serial = self.serial,
                                view_to_find = self.device_info.chrome_welcome_sign_in_no_thanks_btn_id)()

        # Opening an invalid page as the first page opened has issues with UI automator.
        # TODO: rework with param the line below
        if self.url_to_open:
            open_specific_page(serial = self.serial,
                               url = self.url_to_open)()
    def check_condition(self):
        #TODO
        return True


class open_chrome(ui_step):

    def do(self):
        open_browser(serial = self.serial)()
        if self.uidevice(textContains = "Accept").wait.exists(timeout = 10000):
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text": "Accept & continue"})()
            if self.uidevice(textContains = "Find your tabs here").wait.exists(timeout = 10000):
                ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text": "Next"},
                                  view_to_check = {"text": "Set up Chrome"})()
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text": "No thanks"})()
        if self.uidevice(textContains = "No thanks").wait.exists(timeout = 10000):
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"textContains": "No thanks"})()
        if self.uidevice(textContains = "No Thanks").wait.exists(timeout = 10000):
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"textContains": "No Thanks"})()

    def check_condition(self):
        return True


class open_browser_settings(ui_step):
    """
        description:
            opens the browser settings menu

        usage:
            open_browser_settings()()

        tags:
            android, browser, chrome, settings
    """
    def do(self):
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"description": "More options"})()
        self.uidevice.wait.update()

    def check_condition(self):
        return self.uidevice(textContains= "New incognito tab").exists


class open_browser_history(ui_step):
    """
        description:
            opens browser history from <options> menu

        usage:
            open_browser_history()()

        tags:
            android, browser, chrome, history
    """
    def do(self):
        open_browser_settings(serial = self.serial)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "History"} )()
        self.uidevice.wait.idle()
        self.uidevice.wait.update()

    def check_condition(self):
        return self.uidevice(textContains = "chrome://history").exists


class open_browser_bookmarks(ui_step):
    """
        description:
            opens the bookmarks page in the browser

        usage:
            open_browser_bookmarks()()

        tags:
            android, browser, chrome, bookmarks
    """
    def do(self):
        open_browser_settings(serial = self.serial)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "Bookmarks"})()

    def check_condition(self):
        self.uidevice.wait.idle()
        return self.uidevice(text = "Mobile bookmarks").exists or\
               self.uidevice(text = "Mobile Bookmarks").exists


class open_incognito_tab(ui_step):
    """
        description:
            opens a new incognito tab in browser

        usage:
            open_incognito_tab()()

        tags:
            android, browser, chrome, new_tab, incognito
    """
    def do(self):
        open_browser_settings(serial = self.serial)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "New incognito tab"})()

    def check_condition(self):
        return self.uidevice(textContains = "You've gone incognito.").exists


class add_bookmark(ui_step):
    """
        description:
            adds current displayed page to bookmarks.
            it checks it presence in the bookmark page.

        usage:
            add_bookmark(name = "Google"
                         url_to_check = "http://www.google.com")()

        tags:
            android, browser, chrome, bookmarks, add
    """
    def __init__(self, name, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.name = name

    def do(self):
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId":
                                              "com.android.chrome:"
                                              "id/bookmark_button"},
                              view_to_check = {"text":
                                               "Add bookmark"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "Save"})()
        open_browser_bookmarks(serial = self.serial)()

    def check_condition(self):
        self.uidevice.wait.idle()
        return self.uidevice(textContains = self.name).exists


class delete_all_bookmarks(ui_step):
    """
        description:
            delete all browser bookmarks

        usage:
            delete_all_bookmarks()()

        tags:
            android, browser, chrome, all, bookmarks, delete
    """
    def do(self):
        open_browser_bookmarks(serial = self.serial)()
        while not self.uidevice(text = "No bookmarks here"):
            view_to_delete = self.uidevice(resourceId =\
                                           "com.android.chrome:"
                                           "id/bookmarks_list_view").\
                             child(className = "android.widget.TextView")
            view_to_delete.long_click()
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text": "Delete bookmark"},
                                  view_to_check = {"text": "Mobile bookmarks"})()


    def check_condition(self):
        return self.uidevice(text = "No bookmarks here").exists


class clear_browser_history(ui_step):
    """
        description:
            clear the browser history

        usage:
            clear_browser_history()()

        tags:
            android, browser, chrome, history, clear
    """
    def do(self):
        open_browser_history()()
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - Could not open browser options",
            blocking = True,
            view_to_find = {"descriptionContains": "Clear browsing"})()
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - Could not open browser options",
            blocking = True,
            view_to_find = {"textContains": "Clear saved"})()
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - Could not open browser options",
            blocking = True,
            view_to_find = {"textContains": "Clear autofill"})()
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - Could not open browser options",
            blocking = True,
            view_to_find = {"text": "Clear"})()
        time.sleep(1)
        ui_steps.click_button(
            serial = self.serial,
            print_error = "Error - Could not open browser options",
            blocking = True,
            view_to_find = {"text": "Settings"})()
        self.uidevice.wait.update()

    def check_condition(self):
        return self.uidevice(text = "No history entries found.").exists or\
               self.uidevice(description = "No history entries found.").exists


class open_specific_page(ui_step):
    """
        description:
            opens the given url in the new open tab of the browser.
            it can be used to check the browser when a bad url is
            given.

        usage:
            open_specific_page(url = "http://www.wikipedia.org",
                               url_title = "Wikipedia")()

        tags:
            android, browser, chrome, open, url
    """
    def __init__(self, url, text_in_page = None, url_title = None, is_bad_url = False,
                 wait_time = 10000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.url = url
        self.text_in_page = text_in_page
        self.url_title = url_title
        self.is_bad_url = is_bad_url
        self.wait_time = wait_time
        self.set_passm("Openning " + url + " checking " + str(url_title))
        self.set_errorm("", "Openning " + url + " checking " + str(url_title))

    def do(self):
        adb_steps.command(command = "am start -n com.android.chrome/com.google.android.apps.chrome.Main -d "
                          +self.url, serial=self.serial)()
        time.sleep(3)

    def check_condition(self):
        resolution = True
        refresh_button = {"resourceId":"com.android.chrome:id/refresh_button"}
        if(self.is_bad_url):
            return self.uidevice(descriptionContains = "This webpage is not available").wait.exists(timeout = self.wait_time)
        if self.text_in_page:
            #On some devices, the page may be loaded slowly, so try 5 times
            for i in range(5):
                if not self.uidevice(descriptionContains = self.text_in_page).wait.exists(timeout = self.wait_time):
                    resolution = False
                    self.uidevice(**refresh_button).click.wait()
                    time.sleep(2)
                else:
                    resolution = True
                    break
        if self.url_title:
            for i in range(5):
                if not self.uidevice(descriptionContains = self.url_title).wait.exists(timeout = self.wait_time):
                    resolution = False
                    self.uidevice(**refresh_button).click.wait()
                    time.sleep(2)
                else:
                    resolution = True
                    break
        return resolution


class open_image_in_new_tab(ui_step):
    """
        descriptions:
            opens a image present in a tab in a new tab by
            long-clicking on it (x,y) and checks the new tab for
            <text_to_check> text in view

        usage:
            open_image_in_new_tab("new_tab_image_title")()

        tags:
            android, browser, chrome, image, new_tab
    """
    def __init__(self, text_to_check, x = 300, y = 600, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.text_to_check = text_to_check
        self.x = x
        self.y = y

    def do(self):
        ui_steps.swipe(serial = self.serial,
                       print_error = "Swipe error",
                       sx = self.x,
                       sy = self.y,
                       ex = self.x,
                       ey = self.y,
                       steps = 100,
                       blocking = True)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text":
                                              "Open image in new tab"})()
        self.uidevice.wait.update()

    def check_condition(self):
        return check_page_in_history(serial = self.serial,
                                     page_to_check = self.text_to_check)()


class save_image(ui_step):
    """
        description:
            saves image displayed in browser with the given
            <file_name>.
            it checks the presence of the file in the download path.
            (x,y) is a point inside the image displayed.

        usage:
            save_image(file_name = "some_image_file_name",
                       x = 300,
                       y = 600)()

        tags:
            android, browser, chrome, image, save, download
    """
    def __init__(self, file_name, x = 300, y = 600, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.file_name = file_name
        self.x = x
        self.y = y

    def do(self):
        adb_steps.delete_folder_content(serial = self.serial,
                                        folder = "/storage/sdcard0/Download")()
        ui_steps.swipe(
            serial = self.serial,
            print_error = "Swipe error",
            sx = self.x,
            sy = self.y,
            ex = self.x,
            ey = self.y,
            steps = 100,
            blocking = True)()
        ui_steps.click_button(serial = self.serial,
                              wait_time = 2000,
                              view_to_find = {"text": "Save image"})()

    def check_condition(self):
        time.sleep(3)
        return browser_utils.check_for_download_file(serial = self.serial,
                                                    value_str = self.file_name)


class check_page_in_history(ui_step):
    """
        description:
            checks the <page_to_check> is present in history page.
            it can be used to check the absence of a page with <exists>
            paramter.

        usage:
            check_page_from_history(page_to_check = "Wikipedia")()

        tags:
            android, browser, history, page
    """
    def __init__(self, page_to_check, exists = True, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.page_to_check = page_to_check
        self.exists = exists
        if exists:
            self.set_passm("page " + page_to_check + " is present")
            self.set_errorm("", "page " + page_to_check + " is present")
        else:
            self.set_passm("page \"" + page_to_check + "\" not is present")
            self.set_errorm("", "page \"" + page_to_check + "\" not is present")

    def do(self):
        open_browser_history(serial = self.serial)()

    def check_condition(self):
        self.uidevice.wait.update()
        self.step_date = self.uidevice(textContains = self.page_to_check).exists
        if self.exists:
            return self.uidevice(textContains = self.page_to_check).exists
        else:
            return not self.uidevice(textContains = self.page_to_check).exists


class check_multiple_page_number(ui_step):
    """
        description:
            checks if the number of open tabs is <tab_no>

        usage:
            check_multiple_page_number(tab_no = 3)()

        tags:
            android, browser, chrome, tab, number
    """
    def __init__(self, tab_no, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.tab_no = tab_no
        self.set_passm("Number of tabs open is " + str(tab_no))
        self.set_errorm("", "Number of tabs open is " + str(tab_no))

    def do(self):
        self.step_data = browser_utils.get_open_tabs_no(serial = self.serial)

    def check_condition(self):
        return self.step_data == self.tab_no

class open_chrome_proxy(ui_step):
    """
        description:
            opens the Chrome browser and verify the HTTP test page via HTTP Proxy Server

        usage:
            open_chrome_proxy (url_to_open, valid_server = None, serial)()

        tags:
            browser, chrome, proxy, open
    """
    def __init__(self, url_to_open, valid_server = None, **kwargs):
        self.url_to_open = url_to_open
        self.valid_server = valid_server
        ui_step.__init__(self, **kwargs)

    def do(self):
        adb_steps.command(command = "pm clear com.android.chrome", serial=self.serial)()
        # Open the HTTP test page
        open_chrome_first_time(intent = False,
                                url_to_open = self.url_to_open,
                                serial = self.serial)()

    def check_condition(self):
        if self.valid_server == str(False):
            return ui_steps.wait_for_view(view_to_find = {"descriptionContains":"proxy"}, serial=self.serial)()
        return ui_steps.wait_for_view(view_to_find = {"descriptionContains":"internal"}, serial=self.serial)()
