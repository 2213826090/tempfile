from PyUiApi.common.system_utils import *


class WidgetsUtilsM(object):
    @staticmethod
    def launch_widget_chooser():
        SystemUtils.open_homescreen_settings()
        UiAutomatorUtils.refresh_uiautomator_view_hierarchy()
        # if d(resourceId=WIDGET_LAUNCHER_BUTTON_RESID).wait.exists(timeout=10000):
        if d(text="Widgets").wait.exists(timeout=3000):
            LOG.info("found widget button ... ")
            d(text="Widgets").click()

    @staticmethod
    def scroll_main_list():
        if WIDGET_LIST_SCROLL_TYPE == "vertical":
            ScreenSwiper.swipe_up()
        if WIDGET_LIST_SCROLL_TYPE == "horizontal":
            ScreenSwiper.swipe_left(x_begin=0.9, x_end=0.1)

    @staticmethod
    def scroll_to_widget(widget_name):
        max_scrolls = 10
        scrolls_so_far = 0
        while not d(textContains=widget_name).wait.exists(timeout=3000) and scrolls_so_far < max_scrolls:
            WidgetsUtils.scroll_main_list()
            scrolls_so_far += 1
            time.sleep(1)


class WidgetsUtilsN(WidgetsUtilsM):
    @staticmethod
    def launch_widget_chooser():
        UiAutomatorUtils.open_apps_menu()
        if d(text=WIDGETS_MENU_TXT).wait.exists(timeout=3000):
            LOG.info("found widget button ... ")
            d(text=WIDGETS_MENU_TXT).click()
            return True
        return False


WidgetsUtils = None
if ANDROID_VERSION is "M":
    WidgetsUtils = WidgetsUtilsM
elif ANDROID_VERSION is "N":
    WidgetsUtils = WidgetsUtilsN
else:
    WidgetsUtils = WidgetsUtilsM
