from PyUiApi.common.screenshot_utils import *
from PyUiApi.adb_helper.adb_utils import *
import traceback
import time
import datetime
from PyUiApi.multi_dut_support.dut_parameters import *
import xml.etree.ElementTree as ET
from PyUiApi.multi_dut_support.version_manager import AndroidVersion

android_version = AndroidVersion()
if '5' in android_version.major_version_nr:
    # android lollipop detected
    LOG.info("Android Lollipop detected")
    from PyUiApi.common.ui_strings.ui_strings_lollipop import *
elif '6' in android_version.major_version_nr or "M" in android_version.version_name_initial:
    LOG.info("Android Marshmallow detected")
    from PyUiApi.common.ui_strings.ui_strings_marshmallow import *
elif '7' in android_version.major_version_nr or "N" in android_version.version_name_initial:
    LOG.info("Android Neyyappam detected")
    from PyUiApi.common.ui_strings.ui_strings_neyyappam import *
else:
    # default chrome ui strings for android marshmallow
    LOG.info("Android version not detected, trying Marshmallow strings")
    from PyUiApi.common.ui_strings.ui_strings_marshmallow import *

MAX_SWIPES_TO_LIMIT = 15
META_ALT = 0x02
META_CTRL = 0x1000
META_SHIFT = 0x80
# codes for android keyevents
KEYCODE_A = 0x1d
KEYCODE_C = 0x1f
KEYCODE_V = 0x32
KEYCODE_W = 0x33
KEYCODE_H = 0x24
KEYCODE_E = 0x21
KEYCODE_L = 0x28
KEYCODE_O = 0x2b
KEYCODE_W = 0x33
KEYCODE_I = 0x25
KEYCODE_K = 0x27
KEYCODE_SPACE = 0x3e
KEYCODE_DEL = 0x43
KEYCODE_ENTER = 0x42
POLLING_TIMEOUT = 15
KEYCODE_MENU = 0x52
KEYCODE_POWER = 0x1a
screen_unlock_cmd = "input keyevent 82"


def print_exception():
    print traceback.format_exc()


class BasicScroller(object):

    def __init__(self, scroll_type):
        self.scroll_type = scroll_type

    def scroll_to_begining(self):
        if d(scrollable=True).exists:
            if self.scroll_type == "horizontal":
                d(scrollable=True).scroll.horiz.backward()
                d(scrollable=True).scroll.horiz.toBeginning(max_swipes=MAX_SWIPES_TO_LIMIT)
            elif self.scroll_type == "vertical":
                d(scrollable=True).scroll.vert.backward()
                d(scrollable=True).scroll.vert.toBeginning(max_swipes=MAX_SWIPES_TO_LIMIT)
        time.sleep(2)

    def scroll_forward(self):
        if d(scrollable=True).exists:
            if self.scroll_type == "horizontal":
                d(scrollable=True).scroll.horiz.forward()
            elif self.scroll_type == "vertical":
                d(scrollable=True).scroll.vert.forward()
        time.sleep(2)

    def scroll_backward(self):
        if d(scrollable=True).exists:
            if self.scroll_type == "horizontal":
                d(scrollable=True).scroll.horiz.backward()
            elif self.scroll_type == "vertical":
                d(scrollable=True).scroll.vert.backward()
        time.sleep(2)


class AppsScroller(object):

    @staticmethod
    def scroll_to_begining():
        if d(scrollable=True).exists:
            if LAUNCHER_SCROLL == "horizontal":
                d(scrollable=True).scroll.horiz.backward()
                d(scrollable=True).scroll.horiz.toBeginning(max_swipes=MAX_SWIPES_TO_LIMIT)
            elif LAUNCHER_SCROLL == "vertical":
                d(scrollable=True).scroll.vert.backward()
                d(scrollable=True).scroll.vert.toBeginning(max_swipes=MAX_SWIPES_TO_LIMIT)
        time.sleep(2)

    @staticmethod
    def scroll_forward():
        if d(scrollable=True).exists:
            if LAUNCHER_SCROLL == "horizontal":
                d(scrollable=True).scroll.horiz.forward()
            elif LAUNCHER_SCROLL == "vertical":
                d(scrollable=True).scroll.vert.forward()
        time.sleep(2)


class UiAutomatorUtils(object):

    @staticmethod
    def launch_app_from_apps_menu(app_name):
        UiAutomatorUtils.open_apps_menu()
        if UiAutomatorUtils.click_app_name(app_name):
            return True
        AppsScroller.scroll_to_begining()
        app = d(text=app_name)
        if app.exists:
            if UiAutomatorUtils.click_app_name(app_name):
                return True
        i = 1
        while not app.exists:
            if i == MAX_SWIPES_TO_LIMIT:
                break
            AppsScroller.scroll_forward()
            if UiAutomatorUtils.click_app_name(app_name):
                return True
            i += 1
        return False

    @staticmethod
    def open_apps_menu():
        UiAutomatorUtils.clear_popups()
        d.press.home()
        time.sleep(2)
        d(description=APPS_BUTTON_DESC).click()
        time.sleep(1)

    @staticmethod
    def click_app_name(app_name):
        if d(text=app_name).wait.exists(timeout=2000):
            # usual case with only one shortcut
            if d(text=app_name).count == 1:
                d(text=app_name).click()
                return True
            # if there are more shortcuts with the same name
            elif d(text=app_name).count > 1:
                d(text=app_name)[0].click()
                return True
        return False

    @staticmethod
    def refresh_uiautomator_view_hierarchy():
        d.dump()

    @staticmethod
    def is_app_in_apps_menu(app_name):
        UiAutomatorUtils.clear_popups()
        d.screen.on()
        d.press.home()
        d(description=APPS_BUTTON_DESC).click()
        if d(text=app_name).wait.exists(timeout=3000):
            return True
        AppsScroller.scroll_to_begining()
        app = d(text=app_name)
        i = 1
        while not app.exists:
            if i == MAX_SWIPES_TO_LIMIT:
                break
            AppsScroller.scroll_forward()
            if d(text=app_name).wait.exists(timeout=2000):
                return True
            i += 1
        return False

    @staticmethod
    def wait_for_compound_resource_id(compound_resource):
        wait_val = 3000
        for res_id in compound_resource:
            if d(resourceId=res_id).wait.exists(timeout=wait_val):
                return True
            # only first wait should be a little longer
            wait_val = 300
        return False

    @staticmethod
    def unlock_screen(force_unlock_cmd=False):
        lock_status_cmd = "dumpsys power | grep 'mHolding'"
        ''' We expect an output like this, if the screen is unlocked:
               mHoldingWakeLockSuspendBlocker=true
               mHoldingDisplaySuspendBlocker=true
        '''
        lock_status = AdbUtils.run_adb_cmd(lock_status_cmd, add_ticks=False)
        blockers_active = re.findall('true', lock_status)
        if force_unlock_cmd:
            AdbUtils.run_adb_cmd(screen_unlock_cmd)
        if len(blockers_active) == 2:
            # screen already unlocked
            LOG.info("screen already unlocked")
            AdbUtils.run_adb_cmd(screen_unlock_cmd)  # in case there is a lock in place (like Swipe)
        elif len(blockers_active) == 1:
            AdbUtils.run_adb_cmd(screen_unlock_cmd)
        elif len(blockers_active) == 0:
            AdbUtils.run_adb_cmd(screen_unlock_cmd)
            AdbUtils.run_adb_cmd(screen_unlock_cmd)
        time.sleep(1)

    @staticmethod
    def get_screen_dims():
        return d.info["displayWidth"], d.info["displayHeight"]

    @staticmethod
    def clear_obscuring_element():
        w, h = UiAutomatorUtils.get_screen_dims()
        d.click(w - 2, h / 3)  # click in an inconspicous area to get rid of the element
        time.sleep(2)

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
    def close_all_tasks(sleep_after_close=1):
        d.press.home()
        d.press("recent")
        nr_of_dismissed_tasks = 0
        while d(resourceId=DISMISS_TASK_RESID).wait.exists(timeout=5000):
            d(resourceId=DISMISS_TASK_RESID).click()
            time.sleep(sleep_after_close)
            nr_of_dismissed_tasks += 1
        return nr_of_dismissed_tasks

    @staticmethod
    def close_tasks_when_there_are_many_open(sleep_after_close=1):
        nr_of_tries = 20
        total_nr_of_dismissed_tasks = 0
        while nr_of_tries > 0:
            d.press.home()
            d.press("recent")
            nr_of_dismissed_tasks = 0
            if not d(resourceId=DISMISS_TASK_RESID).wait.exists(timeout=5000):
                # there are no more opened tasks, we can break
                break
            while d(resourceId=DISMISS_TASK_RESID).exists:
                d(resourceId=DISMISS_TASK_RESID).click()
                time.sleep(sleep_after_close)
                nr_of_dismissed_tasks += 1
            total_nr_of_dismissed_tasks += nr_of_dismissed_tasks
            nr_of_tries -= 1
        return total_nr_of_dismissed_tasks

    @staticmethod
    def clear_app_first_screens(view_hints):
        for hint in view_hints:
            if d(text=hint).wait.exists(timeout=50000):
                d(text=hint).click()

    @staticmethod
    def click_view_with_text(text, view_timeout=3000):
        view_found = d(text=text).wait.exists(timeout=view_timeout)
        LOG.info("view with text '" + str(text) + "' exists: " + str(view_found))
        if view_found:
            d(text=text).click()
        return view_found

    @staticmethod
    def launch_activity_with_data_uri(uri):
        cmd = 'am start -a android.intent.action.VIEW -d "$URI$"'
        cmd = cmd.replace("$URI$", uri)
        AdbUtils.run_adb_cmd(cmd)
        time.sleep(2)

    nr_of_swipe_steps_that_take_one_second = 200

    @staticmethod
    # simulate a long_click with a swipe over a number of steps
    def long_click(view, press_time_in_seconds=2):
        swipe_steps = press_time_in_seconds * UiAutomatorUtils.nr_of_swipe_steps_that_take_one_second
        view_bounds = Bounds(view.info)
        view_center_x, view_center_y = view_bounds.get_center_coordinates()
        d.swipe(view_center_x, view_center_y, view_center_x, view_center_y, swipe_steps)
        time.sleep(2)

    @staticmethod
    def adb_view_click(view):
        view_bounds = Bounds(view.info)
        view_center_x, view_center_y = view_bounds.get_center_coordinates()
        AdbUtils.tap(view_center_x, view_center_y)
        time.sleep(2)

    @staticmethod
    def open_homescreen_shortcut(shortcut_name):
        d.press.home()
        scroller = BasicScroller(LAUNCHER_WORKSPACE_SCROLL_TYPE)
        scroller.scroll_to_begining()
        scroll_steps = 0
        max_scroll_steps = 4
        while scroll_steps < max_scroll_steps:
            time.sleep(2)
            if d(text=shortcut_name).wait.exists(timeout=3000):
                d(text=shortcut_name).click()
                break
            scroll_steps += 1
            scroller.scroll_forward()

    @staticmethod
    def is_view_clickable(ui_object):
        return Info.get_clickable(ui_object) and Info.get_enabled(ui_object)

    @staticmethod
    def click_center_of_ui_object(ui_object):
        bounds = Bounds(ui_object.info)
        center_x, center_y = bounds.get_center_coordinates()
        print center_x, center_y
        LOG.info("clicking coordinates: %s, %s" % (center_x, center_y))
        d.click(center_x, center_y)

    @staticmethod
    def clear_text_from_edit_text(edit_resit):
        if d(className=ANDROID_WIDGET_EDIT_TEXT, resourceId=edit_resit).wait.exists(timeout=5000):
            d.press(KEYCODE_A, META_ALT)
            d.press.delete()

    @staticmethod
    def reboot_device():
        initial_time = datetime.datetime.now()
        AdbUtils.kill_python_uiautomator_rpc_server_on_dut()
        time.sleep(5)
        AdbUtils.reboot_device()
        time.sleep(5)
        AdbUtils.wait_for_main_ui_services()
        time.sleep(5)
        LOG.info("trying to refresh uiautomator device")
        AdbUtils.run_adb_cmd("kill-server", adb_shell=False, add_ticks=False)
        for i in xrange(50):
            try:
                AdbUtils.run_adb_cmd("root", adb_shell=False, add_ticks=False)
                dut_manager.refresh_active_device()
            except:
                LOG.info("UI not yet available")
                time.sleep(5)
        final_time = datetime.datetime.now()
        time_delta = final_time - initial_time
        time.sleep(5)
        return time_delta.seconds

    @staticmethod
    def get_attr_content_matching_text(text):
        xml_dump = d.dump()
        root = ET.fromstring(xml_dump)

        def recursive_search_for_text(node, text):
            try:
                node_text = node.attrib["text"]
            except KeyError:
                node_text = ""

            if text in node_text:
                return node_text
            else:
                for subnode in list(node):
                    return recursive_search_for_text(subnode, text)

        return recursive_search_for_text(root, text)

    @staticmethod
    def wait_for_ui_object_to_be_enabled(ui_kwprops, max_wait=10):
        for i in range(max_wait):
            if Info.get_enabled(d(**ui_kwprops)):
                return True
            time.sleep(1)
        return False


class DeviceInfo(object):

    @staticmethod
    def get_device_screen_dimensions():
        device_info = d.info
        return device_info['displayWidth'], device_info['displayHeight']


class Info(object):

    @staticmethod
    def get_text(ui_object):
        return str(ui_object.info["text"])

    @staticmethod
    def get_description(ui_object):
        return str(ui_object.info["contentDescription"])

    @staticmethod
    def get_clickable(ui_object):
        return ui_object.info["clickable"]

    @staticmethod
    def get_enabled(ui_object):
        return ui_object.info["enabled"]


class Bounds:

    def __init__(self, view_info):
        vis_bounds = view_info["visibleBounds"]
        self.top = vis_bounds["top"]
        self.bottom = vis_bounds["bottom"]
        self.left = vis_bounds["left"]
        self.right = vis_bounds["right"]

    def get_center_coordinates(self):
        x = self.left + (self.right - self.left) / 2
        y = self.top + (self.bottom - self.top) / 2
        return x, y


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
    def swipe_up(horizontal_offset=0.5, steps=None):
        width, height = UiAutomatorUtils.get_screen_dims()
        sx = width * horizontal_offset
        sy = height * 0.7
        ex = width * horizontal_offset
        ey = height * 0.3
        if steps is None:
            d.swipe(sx, sy, ex, ey)
        else:
            d.swipe(sx, sy, ex, ey, steps)
        time.sleep(2)

    @staticmethod
    def swipe_down(horizontal_offset=0.5, steps=None):
        width, height = UiAutomatorUtils.get_screen_dims()
        sx = width * horizontal_offset
        sy = height * 0.3
        ex = width * horizontal_offset
        ey = height * 0.7
        if steps is None:
            d.swipe(sx, sy, ex, ey)
        else:
            d.swipe(sx, sy, ex, ey, steps)
        time.sleep(2)

    @staticmethod
    def swipe_left(vertical_offset=0.5, steps=None, x_begin=0.7, x_end=0.3):
        width, height = UiAutomatorUtils.get_screen_dims()
        sx = width * x_begin
        sy = height * vertical_offset
        ex = width * x_end
        ey = height * vertical_offset
        if steps is None:
            d.swipe(sx, sy, ex, ey)
        else:
            d.swipe(sx, sy, ex, ey, steps)
        time.sleep(2)

    @staticmethod
    def swipe_right(vertical_offset=0.5, steps=None):
        width, height = UiAutomatorUtils.get_screen_dims()
        sx = width * 0.3
        sy = height * vertical_offset
        ex = width * 0.7
        ey = height * vertical_offset
        if steps is None:
            d.swipe(sx, sy, ex, ey)
        else:
            d.swipe(sx, sy, ex, ey, steps)
        time.sleep(2)


class ViewUtils():

    @staticmethod
    def view_generator_to_view_list(view_generator):
        view_list = []
        for view in view_generator:
            view_list.append(view)
        return view_list


class OrientationChanger():

    @staticmethod
    def change_orientation(new_orientation):
        if d.orientation == new_orientation:
            return
        d.orientation = new_orientation
        for i in range(POLLING_TIMEOUT):
            time.sleep(1)
            if d.orientation == new_orientation:
                break


class ViewNavigator(object):

    resource_id_marker = ':id/'
    current_view_found = False

    def __init__(self, vertical_scroll_direction=True, scroll_times=10):
        self.vert_scroll = vertical_scroll_direction
        self.scroll_times = scroll_times

    def navigate_views(self, views_info, timeout=5000, exit_if_view_not_found=False):
        for info in views_info:
            previous_current_view_found = ViewNavigator.current_view_found
            ViewNavigator.current_view_found = False
            LOG.info("searching for view: " + str(info))
            info_type = type(info)
            if info_type == str:
                self.handle_string_info(info, timeout)
            elif info_type == tuple:
                self.handle_tuple_info(info, timeout)
            elif info_type == int:
                wait_timeout = info / 1000 + info % 1000
                LOG.info("sleeping for %s seconds" % wait_timeout)
                time.sleep(wait_timeout)
                ViewNavigator.current_view_found = previous_current_view_found
                continue
            if ViewNavigator.current_view_found is False:
                LOG.info("View not found: " + str(info))
            if ViewNavigator.current_view_found is False and exit_if_view_not_found:
                break

    def handle_tuple_info(self, info, timeout):
        LOG.info("handling view: " + str(info))
        selector_type = info[0]
        selector_string = info[1]
        set_text_arg = None
        extra_timeout = None
        if len(info) > 2:
            set_text_arg = info[2]
        if len(info) > 3:
            extra_timeout = int(info[3]) / 1000 + int(info[3]) % 1000
        if selector_type == "resourceId":
            if d(resourceId=selector_string).wait.exists(timeout=timeout):
                self.interact_with_view(d(resourceId=selector_string), set_text_arg)
        elif selector_type == "text":
            if d(text=selector_string).wait.exists(timeout=timeout):
                self.interact_with_view(d(text=selector_string), set_text_arg)
        elif selector_type == "description":
            if d(description=selector_string).wait.exists(timeout=timeout):
                self.interact_with_view(d(description=selector_string), set_text_arg)
        elif selector_type == "textContains":
            if d(textContains=selector_string).wait.exists(timeout=timeout):
                self.interact_with_view(d(textContains=selector_string), set_text_arg)
        elif selector_type == "descriptionContains":
            if d(descriptionContains=selector_string).wait.exists(timeout=timeout):
                self.interact_with_view(d(descriptionContains=selector_string), set_text_arg)
        if extra_timeout is not None:
            LOG.info("sleeping for an additional %s seconds" % (str(extra_timeout)))
            time.sleep(extra_timeout)

    def interact_with_view(self, view, set_text_arg):
        LOG.info("View found")
        ViewNavigator.current_view_found = True
        if set_text_arg is None:
            view.click()
        else:
            view.set_text(set_text_arg)

    def handle_string_info(self, info, timeout):
        if ViewNavigator.resource_id_marker in info:
            if d(resourceId=info).wait.exists(timeout=timeout):
                self.interact_with_view(d(resourceId=info), None)
        elif d(textContains=info).wait.exists(timeout=timeout):
            self.interact_with_view(d(textContains=info), None)
        elif d(descriptionContains=info).wait.exists(timeout=timeout):
            self.interact_with_view(d(descriptionContains=info), None)

    def nagivate_text(self, text_labels):
        for label in text_labels:
            LOG.info("searching for text: " + label)
            if d(text=label).wait.exists(timeout=3000):
                d(text=label).click()
            elif d(textStartsWith=label).wait.exists(timeout=1000):
                d(textStartsWith=label).click()
            elif d(textContains=label).wait.exists(timeout=1000):
                d(textContains=label).click()
            else:
                self.search_text_in_scroll_view(label)
            time.sleep(1)
        time.sleep(2)

    def scroll_to_beginning(self):
        if self.vert_scroll:
            d(scrollable=True).scroll.vert.toBeginning()
        else:
            d(scrollable=True).scroll.horiz.toBeginning()

    def scroll_forward(self):
        if self.vert_scroll:
            d(scrollable=True).scroll.vert.forward()
        else:
            d(scrollable=True).scroll.horiz.backward()

    def search_text_in_scroll_view(self, text):
        if d(scrollable=True).exists:
            self.scroll_to_beginning()
            for i in range(self.scroll_times):
                self.scroll_forward()
                d.wait.idle()
                UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
                if d(text=text).wait.exists(timeout=3000):
                    d(text=text).click()
                    return True
                elif d(textStartsWith=text).wait.exists(timeout=1000):
                    d(textStartsWith=text).click()
                    return True
                elif d(textContains=text).wait.exists(timeout=1000):
                    d(textContains=text).click()
                    return True
        return False


class ActionMenuModel(object):

    def __init__(self, options_list, options_lengths):
        self.logical_units_length = reduce(lambda x, y: x + y, options_lengths)
        self.options = options_list
        self.options_unit_length = options_lengths

    def select_option(self, option, menu_left, menu_top, menu_right, menu_bottom):
        if option not in self.options:
            LOG.info("action menu does not have option: " + option)
            return
        option_index = self.options.index(option)
        logical_position = reduce(lambda x, y: x + y, self.options_unit_length[:option_index]) + \
            self.options_unit_length[option_index] / 2
        LOG.info("option logical position is: " + str(logical_position))
        option_y_coord = menu_top + (menu_bottom - menu_top) / 2
        option_x_coord = menu_left + (menu_right - menu_left) * logical_position / self.logical_units_length
        LOG.info("center of desired option found at: %s %s" % (str(option_x_coord), str(option_y_coord)))
        d.click(option_x_coord, option_y_coord)


class ActionMenu(object):
    screenshot_utils = ScreenshotUtils()
    android_M_model = ActionMenuModel(["", "Copy", "Share", "Select all", "Web Search", "Assist"],
                                      [0, 2, 2, 4, 4, 2])

    @staticmethod
    def copy(left=None, top=None, right=None, bottom=None):
        if ANDROID_VERSION == "L":
            if d(resourceId=ACTION_MENU_COPY_RESID).wait.exists(timeout=3000):
                d(resourceId=ACTION_MENU_COPY_RESID).click()
        elif ANDROID_VERSION in ["M", "N"]:
            if left is not None and top is not None and right is not None and bottom is not None:
                ActionMenu.android_M_model.select_option("Copy", left, top, right, bottom)

    @staticmethod
    def delete():
        ActionMenu.screenshot_utils.take_screenshot()
        width, height = UiAutomatorUtils.get_screen_dims()
        del_bar_top, del_bar_right = ActionMenu.screenshot_utils \
            .search_for_first_pixel_of_color_in_column(DELETE_BAR_COLOR, width - 1)
        # get a pixel from the delete button that appeared
        direction = DELETE_BUTTON_DIRECTION
        ref_start_pixel = (del_bar_top - 1, del_bar_right)
        x, y = ActionMenu.screenshot_utils.get_first_pixel_of_color_from_ref_with_direction(DELETE_BAR_ITEMS_COLOR,
                                                                                            ref_start_pixel, direction)
        LOG.info("Action Menu delete click: " + str(x) + " " + str(y))
        if x is not None:
            d.click(x, y)
        ActionMenu.screenshot_utils.remove_all_screenshots()
        time.sleep(2)

    @staticmethod
    def click_paste(paste_element_bounds):
        if ANDROID_VERSION == "L":
            d.click(paste_element_bounds.left + 20, paste_element_bounds.top - 20)
        elif ANDROID_VERSION in ["M", "N"]:
            ActionMenu.screenshot_utils.take_screenshot()
            ref_start_pixel = (paste_element_bounds.left, paste_element_bounds.bottom)
            search_direction = (1, 1)
            x, y = ActionMenu.screenshot_utils.get_first_pixel_of_color_from_ref_with_direction \
                (COLOR_WHITE, ref_start_pixel, search_direction)
            LOG.info("Action Menu paste click: " + str(x) + " " + str(y))
            if x is not None:
                d.click(x, y)
            ActionMenu.screenshot_utils.remove_all_screenshots()
            time.sleep(2)

    @staticmethod
    def push_long_click_share():
        ActionMenu.screenshot_utils.take_screenshot()
        width, height = UiAutomatorUtils.get_screen_dims()
        del_bar_right, del_bar_top = ActionMenu.screenshot_utils \
            .search_for_first_pixel_of_color_in_column(DELETE_BAR_COLOR, width - 1)
        del_bar_right, del_bar_bot = ActionMenu.screenshot_utils \
            .search_for_first_pixel_of_color_in_column(DELETE_BAR_BOTTOM_COLOR, width - 1, 10)
        del_bar_vert_center = del_bar_top + (del_bar_bot - del_bar_top) / 2
        direction = (1, 0)
        ref_start_pixel = (width / 2, del_bar_vert_center)
        x, y = ActionMenu.screenshot_utils.get_first_pixel_of_color_from_ref_with_direction(COLOR_WHITE,
                                                                                            ref_start_pixel, direction)
        LOG.info("Action Menu share click: " + str(x) + " " + str(y))
        if x is not None:
            d.click(x, y)
        ActionMenu.screenshot_utils.remove_all_screenshots()
        time.sleep(2)
