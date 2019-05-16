from PyUiApi.common.uiautomator_utils import *
import urllib
import platform
import os
import subprocess
import signal
import shutil


class Chrome(object):
    files_acces_allowed = False

    @staticmethod
    def launch():
        if d(packageName=CHROME_PACKAGE_NAME, className=CHROME_FRAME_LAYOUT_CLASS).exists:
            pass
        else:
            UiAutomatorUtils.launch_app_from_apps_menu(CHROME_SHORTCUT_NAME)
        if d(resourceId=CHROME_ACCEPT_TERMS_RESID).wait.exists(timeout=3000):
            d(resourceId=CHROME_ACCEPT_TERMS_RESID).click()
        if d(resourceId=CHROME_NEGATIVE_BUTTON_RESID).wait.exists(timeout=3000):
            d(resourceId=CHROME_NEGATIVE_BUTTON_RESID).click()

    @staticmethod
    def open_chrome_menu():
        if d(description=CHROME_OPTIONS_DESC).wait.exists(timeout=3000):
            d.press(KEYCODE_MENU)
        else:
            ScreenSwiper.swipe_down()
            if d(description=CHROME_OPTIONS_DESC).wait.exists(timeout=3000):
                d.press(KEYCODE_MENU)
        time.sleep(2)
        # Chrome.click_options_menu_view()

    @staticmethod
    def is_option_menu_displayed():
        return d(description=CHROME_OPTIONS_DESC).wait.exists(timeout=3000) or\
               d(resourceId=CHROME_OPTION_RESID).exists or\
               d(resourceId=CHROME_EMPTY_OPTION_RESID).exists

    @staticmethod
    def click_options_menu_view():
        if d(description=CHROME_OPTIONS_DESC).wait.exists(timeout=3000):
            d(description=CHROME_OPTIONS_DESC).click()
        elif d(resourceId=CHROME_OPTION_RESID).exists:
            d(resourceId=CHROME_OPTION_RESID).click()
        elif d(resourceId=CHROME_EMPTY_OPTION_RESID).exists:
            d(resourceId=CHROME_EMPTY_OPTION_RESID).click()
        else:
            ScreenSwiper.swipe_down()
            if d(description=CHROME_OPTIONS_DESC).wait.exists(timeout=3000):
                d(description=CHROME_OPTIONS_DESC).click()
            elif d(resourceId=CHROME_OPTION_RESID).exists:
                d(resourceId=CHROME_OPTION_RESID).click()
        d(text=CHROME_NEW_TAB_OPTION_NAME).wait.exists(timeout=2000)

    @staticmethod
    def find_text_in_current_page(text):
        Chrome.open_chrome_menu()
        if d(text=CHROME_FIND_IN_PAGE_MENU_OPTION).wait.exists(timeout=5000):
            d(text=CHROME_FIND_IN_PAGE_MENU_OPTION).click()
        if d(resourceId=CHROME_FIND_QUERY_RESID).wait.exists(timeout=5000):
            d(resourceId=CHROME_FIND_QUERY_RESID).set_text(text)
        if d(resourceId=CHROME_FIND_STATUS_RESID).wait.exists(timeout=5000):
            return d(resourceId=CHROME_FIND_STATUS_RESID).info["text"]
        return None

    @staticmethod
    def navigation_back():
        if d(resourceId=CHROME_BACK_BUTTON_RESID).wait.exists(timeout=2000):
            d(resourceId=CHROME_BACK_BUTTON_RESID).click()
        else:
            d.press.back()
        d.wait.idle()

    @staticmethod
    def navigation_fwd():
        if d(resourceId=CHROME_FORWARD_BUTTON_RESID).wait.exists(timeout=2000):
            d(resourceId=CHROME_FORWARD_BUTTON_RESID).click()
        else:
            Chrome.open_chrome_menu()
            d(resourceId=CHROME_FORWARD_BUTTON_RESID).wait.exists(timeout=2000)
            d(resourceId=CHROME_FORWARD_BUTTON_RESID).click()
        d.wait.idle()

    @staticmethod
    def refresh_page():
        if d(resourceId=CHROME_REFRESH_BUTTON_RESID).wait.exists(timeout=2000):
            d(resourceId=CHROME_REFRESH_BUTTON_RESID).click()
        else:
            Chrome.open_chrome_menu()
            d(resourceId=CHROME_REFRESH_BUTTON_RESID).wait.exists(timeout=2000)
            d(resourceId=CHROME_REFRESH_BUTTON_RESID).click()
        d.wait.idle()

    @staticmethod
    def close_all_chrome_tabs_with_switcher():
        d(resourceId=CHROME_TAB_SWITCHER_RESID).click()
        Chrome.open_chrome_menu()
        d(text=CHROME_CLOSE_TABS_OPTION_NAME).click()

    @staticmethod
    def scroll_up_to_reveal_toolbar():
        if d(className=CHROME_WEBKIT_CLASS, scrollable=True).exists:
            d(className=CHROME_WEBKIT_CLASS, scrollable=True).scroll.vert.toBeginning()
        else:
            ScreenSwiper.swipe_down()

    @staticmethod
    def verify_open_tab_or_make_new_tab():
        Chrome.hide_context_menu_click()
        if d(resourceId=CHROME_ADDRESS_BAR_RESID).exists:
            return

        Chrome.scroll_up_to_reveal_toolbar()

        if d(resourceId=CHROME_ADDRESS_BAR_RESID).exists:
            return
        else:
            Chrome.open_chrome_menu()
            d(text=CHROME_NEW_TAB_OPTION_NAME).click()

    @staticmethod
    def clear_browsing_data():
        Chrome.open_chrome_menu()
        vn = ViewNavigator()
        vn.nagivate_text(["Settings", "Privacy"])
        if d(text=CHROME_CLEAR_BROWSING_DATA_BUTTON_TXT).wait.exists(timeout=3000):
            d(text=CHROME_CLEAR_BROWSING_DATA_BUTTON_TXT).click()
        if d(text=CHROME_CLEAR_TXT).wait.exists(timeout=3000):
            d(text=CHROME_CLEAR_TXT).click()

    @staticmethod
    def toggle_force_zoom():
        Chrome.open_chrome_menu()
        vn = ViewNavigator()
        vn.nagivate_text(["Settings", "Accessibility"])
        if d(text="Force enable zoom").wait.exists(timeout=3000):
            d(text="Force enable zoom").click()
            time.sleep(2)

    @staticmethod
    def set_text_scaling(value):
        SEEK_BAR_NR_OF_VALUES = 30
        Chrome.open_chrome_menu()
        vn = ViewNavigator()
        vn.nagivate_text(["Settings", "Accessibility"])
        if d(resourceId=CHROME_TEXT_SCALING_SEEKBAR_RESID).wait.exists(timeout=3000):
            seek_bounds = Bounds(d(resourceId=CHROME_TEXT_SCALING_SEEKBAR_RESID).info)
            seek_y = seek_bounds.top + (seek_bounds.bottom - seek_bounds.top) / 2
            seek_x = seek_bounds.left + (seek_bounds.right - seek_bounds.left) * value / SEEK_BAR_NR_OF_VALUES
            d.click(seek_x, seek_y)
            time.sleep(2)

    @staticmethod
    def open_new_tab():
        Chrome.open_chrome_menu()
        d(text=CHROME_NEW_TAB_OPTION_NAME).wait.exists(timeout=2000)
        d(text=CHROME_NEW_TAB_OPTION_NAME).click()
        time.sleep(2)

    @staticmethod
    def open_new_incognito_tab():
        Chrome.open_chrome_menu()
        d(text=CHROME_NEW_INCOGNITO_TAB_OPTION_NAME).wait.exists(timeout=2000)
        d(text=CHROME_NEW_INCOGNITO_TAB_OPTION_NAME).click()
        d(text=CHROME_INCOGNITO_TAB_TXT).wait.exists(timeout=5000)

    @staticmethod
    def is_tab_open():
        if not d(resourceId=CHROME_OPTION_RESID).exists:
            ScreenSwiper.swipe_down()
            d(resourceId=CHROME_OPTION_RESID).wait.exists(timeout=3000)
        return d(resourceId=CHROME_ADDRESS_BAR_RESID).exists

    @staticmethod
    def close_all_tabs_and_count(webdriver):
        nr_of_open_tabs = len(webdriver.window_handles)
        count = nr_of_open_tabs
        for i in range(nr_of_open_tabs - 1):  # leave one last tab opened or browser will exit
            Chrome.close_current_tab()
        return count

    @staticmethod
    def go_to_url(url_address, timeout=None, wait_for_page_to_load=True):
        if not d(resourceId=CHROME_ADDRESS_BAR_RESID).wait.exists(timeout=1000):
            if d(resourceId=CHROME_GOOGLE_SEARCH_TEXTBOX_RESID).wait.exists(timeout=3000):
                d(resourceId=CHROME_GOOGLE_SEARCH_TEXTBOX_RESID).click()
            else:
                ScreenSwiper.swipe_down()
        time.sleep(2)
        UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
        d(resourceId=CHROME_ADDRESS_BAR_RESID).wait.exists(timeout=4000)
        d(resourceId=CHROME_ADDRESS_BAR_RESID).set_text(url_address)
        d.press("enter")
        d.wait.idle()
        if wait_for_page_to_load:
            d(resourceId=CHROME_REFRESH_BUTTON_RESID).wait.exists(timeout=30000)
        if timeout is not None and wait_for_page_to_load:
            time.sleep(timeout)

    @staticmethod
    def get_current_tab_url():
        if d(resourceId=CHROME_ADDRESS_BAR_RESID).exists:
            return d(resourceId=CHROME_ADDRESS_BAR_RESID).info["text"]
        else:
            ScreenSwiper.swipe_down()
            if d(resourceId=CHROME_ADDRESS_BAR_RESID).wait.exists(timeout=3000):
                return d(resourceId=CHROME_ADDRESS_BAR_RESID).info["text"]

    @staticmethod
    def close_current_tab():
        d.press(KEYCODE_W, META_CTRL)
        time.sleep(2)

    @staticmethod
    # when clicking with webdriver on a input text box, there is an element that appears obscuring the page
    def clear_obscuring_input_element():
        w, h = UiAutomatorUtils.get_screen_dims()
        d.click(w - 2, h / 3)  # click in an inconspicous area to get rid of the element
        time.sleep(2)

    @staticmethod
    def hide_context_menu_click():
        width, height = UiAutomatorUtils.get_screen_dims()
        d.click(0, height/2)

    @staticmethod
    def try_to_switch_normal_incognito():
        if d(resourceId=CHROME_OPTION_RESID).exists:
            options_info = d(resourceId=CHROME_OPTION_RESID).info
        else:
            options_info = d(resourceId=CHROME_EMPTY_OPTION_RESID).info
        control_info = d(resourceId=CHROME_CONTROL_CONTAINER_RESID).info
        options_bounds = Bounds(options_info)
        control_bounds = Bounds(control_info)
        switcher_x = (options_bounds.left + options_bounds.right) / 2
        switcher_y = (control_bounds.top + options_bounds.top) / 2
        # print switcher_x, switcher_y
        d.click(switcher_x, switcher_y)
        time.sleep(2)

    @staticmethod
    def clear_chrome_allow_access_files():
        if not Chrome.files_acces_allowed and d(text="Allow").wait.exists(timeout=3000):
            d(text="Allow").click()
            Chrome.files_acces_allowed = True

    @staticmethod
    def clear_replace_files():
        if d(textContains=CHROME_REPLACE_FILE_TXT).wait.exists(timeout=3000):
            d(textContains=CHROME_REPLACE_FILE_TXT).click()

    @staticmethod
    def clear_chrome_popups():
        Chrome.clear_chrome_allow_access_files()
        Chrome.clear_replace_files()

    @staticmethod
    def toggle_javascript(enable=True):
        Chrome.open_chrome_menu()
        vn = ViewNavigator()
        vn.nagivate_text(["Settings", "Site settings"])
        if d(textContains="JavaScript").wait.exists(timeout=3000):
            d(textContains="JavaScript").click()
        javascript_status = None
        # needed because uiautomator seems to hang in here
        UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
        if d(textContains="Allowed").wait.exists(timeout=3000):
            javascript_status = True
        elif d(textContains="Blocked").wait.exists(timeout=3000):
            javascript_status = False
        if enable:
            if javascript_status:
                return
            else:
                d(textContains="Blocked").click()
        else:
            if not javascript_status:
                return
            else:
                d(textContains="Allowed").click()

    @staticmethod
    def toggle_popups(enable=True):
        Chrome.open_chrome_menu()
        vn = ViewNavigator()
        vn.nagivate_text(["Settings", "Site settings", "Pop-ups"])
        if d(textStartsWith="Block").wait.exists(timeout=3000):
            if enable:
                if d(resourceId=CHROME_POPUPS_SWITCH_RESID).wait.exists(timeout=3000):
                    d(textStartsWith="Block").click()
                elif d(text="OFF").wait.exists(timeout=3000):
                    d(text="OFF").click()
            else:
                return
        else:
            if enable:
                return
            else:
                if d(resourceId=CHROME_POPUPS_SWITCH_RESID).wait.exists(timeout=3000):
                    d(textStartsWith="Allowed").click()
                elif d(text="ON").wait.exists(timeout=3000):
                    d(text="ON").click()

    @staticmethod
    def toggle_merge_tabs_with_apps(enable=False):
        Chrome.open_chrome_menu()
        vn = ViewNavigator()
        vn.nagivate_text(["Settings", "Merge tabs and apps"])
        option_is_on = False
        if d(text="ON").wait.exists(timeout=1000):
            option_is_on = True
        if option_is_on and not enable:
            d(text="ON").click()
        elif not option_is_on and enable:
            d(text="OFF").click()
        vn.nagivate_text(["OK"])

    @staticmethod
    def toggle_cookies(enable=True):
        Chrome.open_chrome_menu()
        vn = ViewNavigator()
        vn.nagivate_text(["Settings", "Site settings", "Cookies"])
        UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
        cookies_blocked = d(textStartsWith="Block").wait.exists(timeout=3000)
        if cookies_blocked and not enable:
            return
        if cookies_blocked and enable:
            d(textStartsWith="Block").click()
            return
        if not cookies_blocked and enable:
            return
        if not cookies_blocked and not enable:
            d(textStartsWith="Allow").click()
            return

    @staticmethod
    def is_context_menu_showing():
        return d(textStartsWith="Open in new tab").wait.exists(timeout=3000)

    @staticmethod
    def is_image_context_menu_showing():
        return d(textStartsWith="Save image").wait.exists(timeout=3000)

    @staticmethod
    def return_to_main_screen():
        max_back_steps = 10
        count = 0
        while not Chrome.chrome_mainscreen_in_focus():
            count += 1
            if d(description=APPS_BUTTON_DESC).exists or count >= max_back_steps:
                break
            d.press.back()
        time.sleep(2)

    @staticmethod
    def chrome_mainscreen_in_focus():
        return d(resourceId=CHROME_OPTION_RESID).wait.exists(timeout=2000) or\
            d(resourceId=CHROME_ADDRESS_BAR_RESID).exists

    @staticmethod
    def open_bookmarks():
        Chrome.open_chrome_menu()
        if d(text=CHROME_BOOKMARKS_OPTION_MENU_TXT).wait.exists(timeout=3000):
            d(text=CHROME_BOOKMARKS_OPTION_MENU_TXT).click()
        time.sleep(2)

    @staticmethod
    def open_mobile_bookmarks():
        Chrome.return_to_main_screen()
        Chrome.open_bookmarks()
        if d(description=CHROME_OPEN_NAVIGATION_DRAWER_DESC).wait.exists(timeout=2000):
            d(description=CHROME_OPEN_NAVIGATION_DRAWER_DESC).click()
        time.sleep(2)
        if d(text=CHROME_MOBILE_BOOKMARKS_FOLDER_TXT).wait.exists(timeout=2000):
            d(text=CHROME_MOBILE_BOOKMARKS_FOLDER_TXT).click()
        elif d(description=CHROME_MOBILE_BOOKMARKS_FOLDER_DESC).wait.exists(timeout=300):
            d(description=CHROME_MOBILE_BOOKMARKS_FOLDER_DESC).click()
        time.sleep(2)

    @staticmethod
    def get_first_bookmark():
        if d(resourceId=CHROME_BOOKMARK_LIST_RESID).child(className="android.widget.TextView")\
                .wait.exists(timeout=2000):
            first_bookmark = d(resourceId=CHROME_BOOKMARK_LIST_RESID)\
                .child(className="android.widget.TextView")
            return first_bookmark
        return None

    @staticmethod
    def delete_current_bookmarks():
        Chrome.open_bookmarks()
        if d(resourceId=CHROME_BOOKMARK_LIST_RESID).wait.exists(timeout=3000):
            first_bookmark = Chrome.get_first_bookmark()
            if first_bookmark is not None:
                while first_bookmark.sibling(className="android.widget.TextView").count > 0:
                    UiAutomatorUtils.long_click(first_bookmark.sibling(className="android.widget.TextView"), 2)
                    if d(textContains=CHROME_DELETE_BOOKMARK_TXT).wait.exists(timeout=2000):
                        d(textContains=CHROME_DELETE_BOOKMARK_TXT).click()
                    time.sleep(2)
                    UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
                    first_bookmark = Chrome.get_first_bookmark()
                first_bookmark = Chrome.get_first_bookmark()
                UiAutomatorUtils.long_click(first_bookmark, 2)
                if d(textContains=CHROME_DELETE_BOOKMARK_TXT).wait.exists(timeout=2000):
                    d(textContains=CHROME_DELETE_BOOKMARK_TXT).click()
                time.sleep(2)

    @staticmethod
    def delete_bookmarks():
        Chrome.open_bookmarks()
        for i in range(20):
            if d(resourceId=CHROME_MORE_RESID).wait.exists(timeout=3000):
                d(resourceId=CHROME_MORE_RESID).click()
                if d(text=CHROME_DELETE_TXT).wait.exists(timeout=3000):
                    d(text=CHROME_DELETE_TXT).click()
            else:
                Chrome.navigation_back()
                break

    @staticmethod
    def click_bookmark_star():
        # Checks have been doubled for Sofia-like devices, with different IDs
        if d(resourceId=CHROME_BOOKMARK_BUTTON_RESID).wait.exists(timeout=2000):
            d(resourceId=CHROME_BOOKMARK_BUTTON_RESID).click()
        elif d(resourceId=CHROME_BOOKMARK_BUTTON_RESID_SOFIA).wait.exists(timeout=100):
            d(resourceId=CHROME_BOOKMARK_BUTTON_RESID_SOFIA).click()
        else:
            ScreenSwiper.swipe_down()
            if d(resourceId=CHROME_BOOKMARK_BUTTON_RESID).wait.exists(timeout=3000):
                d(resourceId=CHROME_BOOKMARK_BUTTON_RESID).click()
            elif d(resourceId=CHROME_BOOKMARK_BUTTON_RESID_SOFIA).wait.exists(timeout=100):
                d(resourceId=CHROME_BOOKMARK_BUTTON_RESID_SOFIA).click()
            else:
                Chrome.open_chrome_menu()
                if d(resourceId=CHROME_BOOKMARK_BUTTON_RESID).wait.exists(timeout=3000):
                    d(resourceId=CHROME_BOOKMARK_BUTTON_RESID).click()
                elif d(resourceId=CHROME_BOOKMARK_BUTTON_RESID_SOFIA).wait.exists(timeout=100):
                    d(resourceId=CHROME_BOOKMARK_BUTTON_RESID_SOFIA).click()


    @staticmethod
    def enter_autofill_forms_info(name="test", organization="test", address="test", city="test", state="test",
                                  zip_code="111111", phone="2222222", email="test@test.com"):
        Chrome.open_chrome_menu()
        vn = ViewNavigator()
        vn.nagivate_text(["Settings", "Autofill forms"])
        if d(text="On").exists:
            pass
        elif d(text="Off").wait.exists(timeout=2000):
            d(text="Off").click()
        if d(description=CHROME_ADD_ADDRESS_DESC).wait.exists(timeout=2000):
            d(description=CHROME_ADD_ADDRESS_DESC).click()

        def enter_form_info(form_info_dict):
            for info_view_description in form_info_dict:
                if not d(description=info_view_description).wait.exists(timeout=2000):
                    # hide poped up keyboard
                    d.press.back()
                    ScreenSwiper.swipe_up()
                d(description=info_view_description).set_text(form_info_dict[info_view_description])
                time.sleep(1)

        enter_form_info({"Name": name, "Organization": organization, "Street address": address,
                         "City": city, "State": state, "ZIP code": zip_code})
        if not d(resourceId="com.android.chrome:id/phone_number_edit").wait.exists(timeout=2000):
                d.press.back()
                ScreenSwiper.swipe_up()
        d(resourceId="com.android.chrome:id/phone_number_edit").set_text(phone)
        time.sleep(1)
        if not d(resourceId="com.android.chrome:id/email_address_edit").wait.exists(timeout=2000):
                d.press.back()
                ScreenSwiper.swipe_up()
        d(resourceId="com.android.chrome:id/email_address_edit").set_text(email)
        time.sleep(1)
        if not d(text="Save").wait.exists(timeout=2000):
            # keyboard is obscuring the save button
            d.press.back()
        d(text="Save").click()
        # go back to Settings main activity
        Chrome.return_to_main_screen()

    @staticmethod
    def add_bookmark_handle_name(name):
        for res_id in CHROME_BOOKMARK_TITLE_INPUT_RESIDS:
            if name is not None and d(resourceId=res_id).wait.exists(timeout=3000):
                bookmark_title = d(resourceId=res_id).info["text"]
                d(resourceId=res_id).click()
                for char in bookmark_title:
                    d.press.delete()  # delete
                d(resourceId=res_id).set_text(name)
                d.press.back()
                break

    @staticmethod
    def edit_bookmark_folder_name(name):
        for res_id in CHROME_BOOKMARK_FOLDER_EDIT_RESIDS:
            if name is not None and d(resourceId=res_id).wait.exists(timeout=3000):
                bookmark_title = d(resourceId=res_id).info["text"]
                d(resourceId=res_id).click()
                for char in bookmark_title:
                    d.press.delete()  # delete
                d(resourceId=res_id).set_text(name)
                d.press.back()
                break

    @staticmethod
    def add_bookmark_handle_folder(folder, cancel_folder_creation):
        for res_id in CHROME_BOOKMARK_FOLDER_SELECT_RESIDS:
            if folder is not None:
                # select the bookmark folder option
                if d(resourceId=res_id).wait.exists(timeout=3000):
                    d(resourceId=res_id).click()
                # select the "mobile bookmarks" folder
                if d(text=CHROME_MOBILE_BOOKMARKS_FOLDER_TXT).wait.exists(timeout=3000):
                    d(text=CHROME_MOBILE_BOOKMARKS_FOLDER_TXT).click()
                # enter pe bookmarks folder options again
                if d(resourceId=res_id).wait.exists(timeout=3000):
                    d(resourceId=res_id).click()
                # select the new folder option
                if d(textStartsWith=CHROME_NEW_FOLDER_BUTTON_TXT).wait.exists(timeout=3000):
                    d(textStartsWith=CHROME_NEW_FOLDER_BUTTON_TXT).click()
                elif d(resourceId=CHROME_NEW_FOLDER_BUTTON_RESID).wait.exists(timeout=300):
                    d(resourceId=CHROME_NEW_FOLDER_BUTTON_RESID).click()
                # enter the new folder name
                if d(resourceId=CHROME_NEW_FOLDER_NAME_RESID).wait.exists(timeout=3000):
                    d(resourceId=CHROME_NEW_FOLDER_NAME_RESID).set_text(folder)
                if d(description=CHROME_NEW_FOLDER_NAME_DESC).wait.exists(timeout=300):
                    d(description=CHROME_NEW_FOLDER_NAME_DESC).set_text(folder)
                # hide the keyboard
                d.press.back()
                # used in cancel folder creation test
                if cancel_folder_creation:
                    if d(description=CHROME_ADD_BOOKMARK_NAVIGATE_UP_DESC).wait.exists(timeout=3000):
                        d(description=CHROME_ADD_BOOKMARK_NAVIGATE_UP_DESC).click()
                    elif d(text=CHROME_ADD_BOOKMARK_CANCEL_TXT).wait.exists(timeout=300):
                        d(text=CHROME_ADD_BOOKMARK_CANCEL_TXT).click()
                    d.press.back()
                # save the new folder
                elif d(description=CHROME_ADD_BOOKMARK_SAVE_DESC).wait.exists(timeout=3000):
                    d(description=CHROME_ADD_BOOKMARK_SAVE_DESC).click()
                elif d(text=CHROME_ADD_BOOKMARK_SAVE_TXT).wait.exists(timeout=300):
                    d(text=CHROME_ADD_BOOKMARK_SAVE_TXT).click()
                break

    @staticmethod
    def add_bookmark_handle_url(url):
        for res_id in CHROME_BOOKMARK_URL_INPUT_RESIDS:
            if url is not None and d(resourceId=res_id).wait.exists(timeout=3000):
                bookmark_url = d(resourceId=res_id).info["text"]
                d(resourceId=res_id).click()
                for char in bookmark_url:
                    d.press.delete()  # delete
                d(resourceId=res_id).set_text(url)
                d.press.back()
                break

    @staticmethod
    def add_bookmark(name=None, url=None, folder=None, cancel_bookmark_creation=False,
                     cancel_folder_creation=False):
        Chrome.click_bookmark_star()
        # this is a feature introduced in April 2016, the edit bookmark screen comes up only after second click
        # on bookmark star
        if not d(text=CHROME_EDIT_BOOKMARK_OPTION_TXT).wait.exists(timeout=2000):
            Chrome.click_bookmark_star()
        Chrome.add_bookmark_handle_folder(folder, cancel_folder_creation)
        Chrome.add_bookmark_handle_name(name)
        Chrome.add_bookmark_handle_url(url)
        # used for the cancel bookmark creation tests
        if cancel_bookmark_creation and d(text=CHROME_ADD_BOOKMARK_CANCEL_TXT).wait.exists(timeout=3000):
            d(text=CHROME_ADD_BOOKMARK_CANCEL_TXT).click()
        # save the bookmark altogether
        elif d(description=CHROME_ADD_BOOKMARK_NAVIGATE_UP_DESC).wait.exists(timeout=3000):
            d(description=CHROME_ADD_BOOKMARK_NAVIGATE_UP_DESC).click()
        elif d(text=CHROME_ADD_BOOKMARK_SAVE_TXT).wait.exists(timeout=300):
            d(text=CHROME_ADD_BOOKMARK_SAVE_TXT).click()
        time.sleep(2)

    @staticmethod
    def edit_bookmark(bookmark_name, new_name, folder_name=None, bookmark_index=0, cancel_edit=False):
        if folder_name is not None:
            if d(textContains=folder_name).wait.exists(timeout=3000):
                d(textContains=folder_name).click()
        if d(textContains=bookmark_name).wait.exists(timeout=3000):
            d(resourceId=CHROME_BOOKMARK_OPTIONS_MORE_RESID, instance=bookmark_index).click()
        if d(text=CHROME_BOOKMARK_OPTIONS_EDIT_TXT).wait.exists(timeout=3000):
            d(text=CHROME_BOOKMARK_OPTIONS_EDIT_TXT).click()
        Chrome.add_bookmark_handle_name(new_name)
        if cancel_edit:
            d.press.back()
        else:
            Chrome.save_bookmark()

    @staticmethod
    def edit_bookmark_folder(folder_name, new_name, parent_name=None, bookmark_index=0, cancel_edit=False):
        if parent_name is not None:
            if d(textContains=parent_name).wait.exists(timeout=3000):
                d(textContains=parent_name).click()
        if d(textContains=folder_name).wait.exists(timeout=3000):
            d(resourceId=CHROME_BOOKMARK_OPTIONS_MORE_RESID, instance=bookmark_index).click()
        if d(text=CHROME_BOOKMARK_OPTIONS_EDIT_TXT).wait.exists(timeout=3000):
            d(text=CHROME_BOOKMARK_OPTIONS_EDIT_TXT).click()
        Chrome.edit_bookmark_folder_name(new_name)
        if cancel_edit:
            d.press.back()
        else:
            Chrome.save_bookmark()

    @staticmethod
    def delete_bookmark(bookmark_name, folder_name=None, bookmark_index=0):
        if folder_name is not None:
            if d(textContains=folder_name).wait.exists(timeout=3000):
                d(textContains=folder_name).click()
        if d(textContains=bookmark_name).wait.exists(timeout=3000):
            d(resourceId=CHROME_BOOKMARK_OPTIONS_MORE_RESID, instance=bookmark_index).click()
        if d(text=CHROME_BOOKMARK_OPTIONS_DELETE_TXT).wait.exists(timeout=3000):
            d(text=CHROME_BOOKMARK_OPTIONS_DELETE_TXT).click()

    @staticmethod
    def save_bookmark():
        if d(description=CHROME_ADD_BOOKMARK_NAVIGATE_UP_DESC).wait.exists(timeout=3000):
            d(description=CHROME_ADD_BOOKMARK_NAVIGATE_UP_DESC).click()

    @staticmethod
    def bookmark_current_page():
        if d(resourceId=CHROME_BOOKMARK_BUTTON_RESID).wait.exists(timeout=3000):
            d(resourceId=CHROME_BOOKMARK_BUTTON_RESID).click()
        if d(text="Save").wait.exists(timeout=3000):
            d(text="Save").click()
        time.sleep(1)

    @staticmethod
    def is_incognito_tab_in_focus(screenshot_util):
        screenshot_util.take_screenshot()
        current_tab_image_stat = screenshot_util.get_current_screenshot_stat()
        r, g, b = current_tab_image_stat.median
        # the median pixel for the incognito tab is very dark
        return r < 60 and g < 60 and b < 60

    @staticmethod
    def tap_incognito_mode_switcher():
        menu_button_bounds = Bounds(d(resourceId=CHROME_OPTION_RESID).info)
        # try to compute a point from the new tab button
        y = menu_button_bounds.top - (menu_button_bounds.bottom - menu_button_bounds.top) / 3
        x = menu_button_bounds.left + (menu_button_bounds.right - menu_button_bounds.left) * 2 / 3
        d.click(x, y)
        time.sleep(2)

    @staticmethod
    def get_html_page_top_left():
        if d(resourceId=CHROME_TOOLBAR_RESID).wait.exists(timeout=3000):
            toolbar_bounds = Bounds(d(resourceId=CHROME_TOOLBAR_RESID).info)
            return toolbar_bounds.bottom, toolbar_bounds.left
        else:
            if d(description=CHROME_OPTIONS_DESC).exists:
                options_bounds = Bounds(d(description=CHROME_OPTIONS_DESC).info)
                return 0, options_bounds.bottom
        return None, None

CHROMEDRIVER_LINUX = "http://mcg-depot.intel.com/artifactory/simple/acs_test_artifacts/APPLICATIONS/CHROME/CHROMEDRIVER/chromedriver_linux64"


class ChromeDriverUtils(object):
    chrome_driver_file_name = "chromedriver"
    server = None
    chromedriver_local_download_folder = os.path.expanduser("~/tmp/")
    local_chromedriver_download = os.path.join(chromedriver_local_download_folder, chrome_driver_file_name)

    def __init__(self):
        self.os_name = platform.system()
        self.downloader = urllib.URLopener()
        chromedriver_dir = os.path.dirname(os.path.realpath(__file__))
        self.chrome_driver_file_name = os.path.join(chromedriver_dir, self.chrome_driver_file_name)

    def resolve_dependencies(self):
        self.mask_env_var("http_proxy")  # this needs to be unset, otherwise chromedrive is not able to connect
        if os.path.isfile(self.chrome_driver_file_name):
            print "file exists, no more downloading"
        elif os.path.isfile(ChromeDriverUtils.local_chromedriver_download):
            print "trying to get file from temporary download folder"
            shutil.copyfile(ChromeDriverUtils.local_chromedriver_download, self.chrome_driver_file_name)
        elif "Linux" in self.os_name:
            print "downloading " + CHROMEDRIVER_LINUX
            self.downloader.retrieve(CHROMEDRIVER_LINUX, self.chrome_driver_file_name)
        elif "Windows" in self.os_name:
            pass  # TO DO
        subprocess.call("chmod a+x " + self.chrome_driver_file_name, shell=True)

    def start_chromedriver(self):
        self.server = subprocess.Popen(self.chrome_driver_file_name,
                                       shell=True, preexec_fn=os.setsid, stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT)
        time.sleep(5)

    def stop_chromedriver(self):
        os.killpg(self.server.pid, signal.SIGTERM)
        time.sleep(3)

    def is_chromedriver_running(self):
        chromedriver_procs = subprocess.check_output("ps -ef | grep chromedriver", shell=True).splitlines()
        return len(chromedriver_procs) > 2

    def kill_chromedriver(self):
        if self.is_chromedriver_running():
            chromedriver_procs = subprocess.check_output("ps -ef | grep chromedriver", shell=True)
            chromedriver_procs = chromedriver_procs.splitlines()
            for proc_line in chromedriver_procs:
                infos = proc_line.split()
                pid = infos[1]
                subprocess.call("kill -9 " + pid, shell=True)
        time.sleep(5)

    def restart_chromedriver(self):
        self.stop_chromedriver()
        if self.is_chromedriver_running():
            self.kill_chromedriver()
        self.start_chromedriver()

    def mask_env_var(self, var_mask):
        for key in os.environ.keys():
            if key.lower().startswith(var_mask):
                del os.environ[key]


class WaitForPageLoad(object):

    def __init__(self, browser):
        self.browser = browser

    def __enter__(self):
        self.old_page = self.browser.find_element_by_tag_name('html')

    def page_has_loaded(self):
        new_page = self.browser.find_element_by_tag_name('html')
        return new_page.id != self.old_page.id

    def __exit__(self, *_):
        wait_for(self.page_has_loaded)


def wait_for(condition_function, timeout=20):
    start_time = time.time()
    while time.time() < start_time + timeout:
        if condition_function():
            print "page has loaded"
            return True
        else:
            time.sleep(1)
    raise Exception(
        'Timeout waiting for {}'.format(condition_function.__name__)
    )

if __name__ == "__main__":
    #cdu = ChromeDriverUtils()
    #cdu.resolve_dependencies()
    #cdu.start_chromedriver()
    #time.sleep(20)
    #cdu.stop_chromedriver()
    Chrome.return_to_main_screen()
