from uiautomator import device as d
from adb_helper.adb_utils import *
import traceback
import time

MAX_SWIPES_TO_LIMIT = 15
META_ALT = 0x02
META_CTRL = 0x1000
META_SHIFT = 0x80
KEYCODE_W = 0x33

android_version = AdbUtils.run_adb_cmd("getprop ro.build.version.release")
if android_version.split(".")[0] == 5:
    # android lollipop detected
    from chrome_ui_strings_lollipop import *
elif android_version.split(".")[0] == 6:
    # TO DO if the UI will change in the future, create a new chrome_ui_string_for_android_6
    pass
else:
    # default chrome ui strings for android lollipop
    from chrome_ui_strings_lollipop import *


def print_exception():
    print traceback.format_exc()


class UiAutomatorUtils:
    @staticmethod
    def launch_app_from_apps_menu(app_name):
        UiAutomatorUtils.clear_popups()
        d.screen.on()
        d.press.home()
        d(description=APPS_BUTTON_DESC).click()
        if d(text=app_name).wait.exists(timeout=3000):
            d(text=app_name).click()
            return
        d(scrollable=True).scroll.horiz.backward()
        d(scrollable=True).scroll.horiz.toBeginning(max_swipes=MAX_SWIPES_TO_LIMIT)
        chrome = d(text=app_name)
        i = 1
        while not chrome.exists:
            if i == MAX_SWIPES_TO_LIMIT:
                break
            d(scrollable=True).scroll.horiz.forward()
            chrome = d(text=app_name)
            i += 1
        chrome.click()

    @staticmethod
    def get_screen_dims():
        return d.info["displayWidth"], d.info["displayHeight"]

    @staticmethod
    def go_to_apps_beginning():
        UiAutomatorUtils.clear_popups()
        d.screen.on()
        d.press.home()
        d(description=APPS_BUTTON_DESC).click()
        if d(scrollable=True).wait.exists(timeout=3000):
            d(scrollable=True).scroll.horiz.backward()
            d(scrollable=True).scroll.horiz.toBeginning(max_swipes=MAX_SWIPES_TO_LIMIT)

    @staticmethod
    def clear_popups():
        if d(text=POPUP_OK).exists:
            d(text=POPUP_OK).click()

    @staticmethod
    def close_all_tasks():
        d.press.home()
        d.press("recent")
        while d(resourceId=DISMISS_TASK_RESID).wait.exists(timeout=5000):
            d(resourceId=DISMISS_TASK_RESID).click()


class Chrome:
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
        if d(resourceId=CHROME_OPTION_RESID).exists:
            d(resourceId=CHROME_OPTION_RESID).click()
        elif d(resourceId=CHROME_EMPTY_OPTION_RESID).exists:
            d(resourceId=CHROME_EMPTY_OPTION_RESID).click()
        d(text=CHROME_NEW_TAB_OPTION_NAME).wait.exists(timeout=5000)

    @staticmethod
    def navigation_back():
        d(resourceId=CHROME_BACK_BUTTON_RESID).click()
        d.wait.idle()

    @staticmethod
    def navigation_fwd():
        d(resourceId=CHROME_FORWARD_BUTTON_RESID).click()
        d.wait.idle()

    @staticmethod
    def refresh_page():
        d(resourceId=CHROME_REFRESH_BUTTON_RESID).click()
        d.wait.idle()

    @staticmethod
    def close_all_chrome_tabs_with_switcher():
        d(resourceId=CHROME_TAB_SWITCHER_RESID).click()
        Chrome.open_chrome_menu()
        d(text=CHROME_CLOSE_TABS_OPTION_NAME).click()

    @staticmethod
    def verify_open_tab_or_make_new_tab():
        Chrome.hide_context_menu_click()
        if d(resourceId=CHROME_ADDRESS_BAR_RESID).exists:
            return

        if d(className=CHROME_WEBKIT_CLASS, scrollable=True).exists:
            d(className=CHROME_WEBKIT_CLASS, scrollable=True).scroll.vert.toBeginning()

        if d(resourceId=CHROME_ADDRESS_BAR_RESID).exists:
            return
        else:
            Chrome.open_chrome_menu()
            d(text=CHROME_NEW_TAB_OPTION_NAME).click()

    @staticmethod
    def open_new_tab():
        Chrome.open_chrome_menu()
        d(text=CHROME_NEW_TAB_OPTION_NAME).click()

    @staticmethod
    def open_new_incognito_tab():
        Chrome.open_chrome_menu()
        d(text=CHROME_NEW_INCOGNITO_TAB_OPTION_NAME).click()

    @staticmethod
    def is_tab_open():
        return d(resourceId=CHROME_ADDRESS_BAR_RESID).exists

    @staticmethod
    def close_all_tabs_and_count():
        count = 0
        while Chrome.is_tab_open():
            Chrome.close_current_tab()
            count += 1
        return count

    @staticmethod
    def go_to_url(url_address, timeout=None):
        refresh_in_layout = d(resourceId=CHROME_REFRESH_BUTTON_RESID).exists
        d(resourceId=CHROME_ADDRESS_BAR_RESID).set_text(url_address)
        d.press("enter")
        d.wait.idle()
        if refresh_in_layout:
            d(resourceId=CHROME_REFRESH_BUTTON_RESID).wait.exists(timeout=60000)
        if timeout is not None:
            time.sleep(timeout)

    @staticmethod
    def get_current_tab_url():
        return d(resourceId=CHROME_ADDRESS_BAR_RESID).info["text"]

    @staticmethod
    def close_current_tab():
        d.press(KEYCODE_W, META_CTRL)
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


class Bounds:
    def __init__(self, view_info):
        vis_bounds = view_info["visibleBounds"]
        self.top = vis_bounds["top"]
        self.bottom = vis_bounds["bottom"]
        self.left = vis_bounds["left"]
        self.right = vis_bounds["right"]


class TabSwitcher:
    @staticmethod
    def swipe_to_left_tab():
        address_bar_info = d(resourceId=CHROME_ADDRESS_BAR_RESID).info
        bounds = Bounds(address_bar_info)
        sx = bounds.left
        ex = bounds.right
        sy = (bounds.top + bounds.bottom) / 2
        ey = sy
        d.swipe(sx, sy, ex, ey)
        time.sleep(2)
    @staticmethod
    def swipe_to_right_tab():
        address_bar_info = d(resourceId=CHROME_ADDRESS_BAR_RESID).info
        bounds = Bounds(address_bar_info)
        sx = bounds.right
        ex = bounds.left
        sy = (bounds.top + bounds.bottom) / 2
        ey = sy
        d.swipe(sx, sy, ex, ey)
        time.sleep(2)


class ScreenSwiper:
    @staticmethod
    def swipe_up(horizontal_offset=0.5):
        width, height = UiAutomatorUtils.get_screen_dims()
        sx = width * horizontal_offset
        sy = height * 0.7
        ex = width * horizontal_offset
        ey = height * 0.3
        d.swipe(sx, sy, ex, ey)
        time.sleep(2)
    @staticmethod
    def swipe_down(horizontal_offset=0.5):
        width, height = UiAutomatorUtils.get_screen_dims()
        sx = width * horizontal_offset
        sy = height * 0.3
        ex = width * horizontal_offset
        ey = height * 0.7
        d.swipe(sx, sy, ex, ey)
        time.sleep(2)
    @staticmethod
    def swipe_left(horizontal_offset=0.5):
        width, height = UiAutomatorUtils.get_screen_dims()
        sx = width * 0.7
        sy = height * horizontal_offset
        ex = width * 0.3
        ey = height * horizontal_offset
        d.swipe(sx, sy, ex, ey)
        time.sleep(2)
    @staticmethod
    def swipe_right(horizontal_offset=0.5):
        width, height = UiAutomatorUtils.get_screen_dims()
        sx = width * 0.3
        sy = height * horizontal_offset
        ex = width * 0.7
        ey = height * horizontal_offset
        d.swipe(sx, sy, ex, ey)
        time.sleep(2)