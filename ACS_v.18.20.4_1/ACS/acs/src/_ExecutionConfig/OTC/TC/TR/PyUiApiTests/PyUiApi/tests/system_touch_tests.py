from PyUiApi.adb_helper.adb_utils import *
from PyUiApi.tests.chrome_tests import *


class SystemTouchTests(unittest.TestCase):
    TAG = "SystemTouchTests"
    vertical_swype_activity_name = "VerticalScrollActivity"
    horizontal_swype_activity_name = "HorizontalScrollActivity"
    text_tab_element_prefix = "Text_Tab"

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation
        self.screenshooter = ScreenshotUtils()

    def tearDown(self):
        self.log_before_cleanup()
        UiAutomatorUtils.close_all_tasks()
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        self.screenshooter.remove_all_screenshots()

    def log_before_cleanup(self):
        result = self._resultForDoCleanups
        if not result.wasSuccessful():
            TestUtils.log_screenshot_and_dump()

    def test_touchscreen_gestures_swipe(self):
        UiAutomatorUtils.close_all_tasks()
        UiAutomatorUtils.launch_app_from_apps_menu(SystemTouchTests.horizontal_swype_activity_name)
        d(textStartsWith=SystemTouchTests.text_tab_element_prefix).wait.exists(timeout=3000)
        nr_of_text_tabs = d(textStartsWith=SystemTouchTests.text_tab_element_prefix).count
        last_text_tab = d(textStartsWith=SystemTouchTests.text_tab_element_prefix)[nr_of_text_tabs - 1]
        initial_text = last_text_tab.info["text"]
        LOG.info("initial text: %s" % initial_text)
        final_text = ""
        for _ in range(10):
            ScreenSwiper.swipe_left()
            ScreenSwiper.swipe_left()
            if d(textStartsWith=SystemTouchTests.text_tab_element_prefix).wait.exists(timeout=3000):
                nr_of_text_tabs = d(textStartsWith=SystemTouchTests.text_tab_element_prefix).count
                final_text_tab = d(textStartsWith=SystemTouchTests.text_tab_element_prefix)[nr_of_text_tabs - 1]
                final_text = final_text_tab.info["text"]
                LOG.info("final text: %s" % final_text)
                if initial_text != final_text:
                    break
        else:
            assert False

        UiAutomatorUtils.close_all_tasks()
        UiAutomatorUtils.launch_app_from_apps_menu(SystemTouchTests.vertical_swype_activity_name)
        d(textStartsWith=SystemTouchTests.text_tab_element_prefix).wait.exists(timeout=3000)
        nr_of_text_tabs = d(textStartsWith=SystemTouchTests.text_tab_element_prefix).count
        last_text_tab = d(textStartsWith=SystemTouchTests.text_tab_element_prefix)[nr_of_text_tabs - 1]
        initial_text = last_text_tab.info["text"]
        LOG.info("initial text: %s" % initial_text)
        final_text = ""
        for _ in range(10):
            ScreenSwiper.swipe_up()
            ScreenSwiper.swipe_up()
            if d(textStartsWith=SystemTouchTests.text_tab_element_prefix).wait.exists(timeout=3000):
                nr_of_text_tabs = d(textStartsWith=SystemTouchTests.text_tab_element_prefix).count
                final_text_tab = d(textStartsWith=SystemTouchTests.text_tab_element_prefix)[nr_of_text_tabs - 1]
                final_text = final_text_tab.info["text"]
            LOG.info("final text: %s" % final_text)
            if initial_text != final_text:
                break
        self.assertTrue(initial_text != final_text)

    def test_touchscreen_gestures_zoom(self):
        chrome_test_result = SingleMethodRunner.run_single_test(ChromeTests, "test_system_touch_zoom")
        self.assertTrue(chrome_test_result.wasSuccessful())
