# -*- coding: utf-8 -*-
import unittest
from PyUiApi.app_utils.keyboard_utils import *
from PyUiApi.common.test_utils import *
from PyUiApi.app_utils.settings_utils import *
from PyUiApi.adb_helper.instrumentation_utils import *
from PyUiApi.app_utils.chrome_utils import *
import sys

reload(sys)
sys.setdefaultencoding("utf-8")

api_tests_logcat = '''logcat -d | grep API'''
dumpsys_power_cmd = '''dumpsys power | grep "Display Power"'''
geometry_string = "keyboard geometry: "
test_app_name = "InputMethodTests"
edit_text_resid = "com.intel.test.apitests:id/edit_text"


class GenericKeyboardTests(unittest.TestCase):
    def log_before_cleanup(self):
        result = self._resultForDoCleanups
        if not result.wasSuccessful():
            TestUtils.log_screenshot_and_dump()

    @staticmethod
    def click_edit_text():
        if d(resourceId=edit_text_resid).wait.exists(timeout=20000):
            d(resourceId=edit_text_resid).click()
        time.sleep(2)

    @staticmethod
    def hide_keyboard():
        if not d(resourceId=edit_text_resid).wait.exists(timeout=1000):
            d.press.back()

    @staticmethod
    def get_edit_text():
        if d(resourceId=edit_text_resid).wait.exists(timeout=20000):
            text = d(resourceId=edit_text_resid).info["text"]
            LOG.info("got following text from editbox: " + text)
            return text
        return None

    def get_keyboard_geometry_information_from_logcat(self, logcat_string):
        last_detected_geometry = None
        for line in logcat_string.split("\n"):
            if geometry_string in line:
                last_detected_geometry = line
        if last_detected_geometry is not None:
            geometry = last_detected_geometry.split(":")[-1].split(" ")
            geometry = [x for x in geometry if x != '']
            return geometry[0], geometry[1], geometry[2], geometry[3]
        return None, None, None, None

    def get_keyboard_geometry(self):
        ApiTestsInterface\
            .run_instrumentation(class_name="InputMethodTestsDriver",
                                 method_name="testGetKeyboardLocation",
                                 instrumentation_args=None,
                                 runner_name="GenericArgumentPassingTestRunner")
        api_tests_logcat_output = AdbUtils.run_adb_cmd(api_tests_logcat, adb_shell=False)
        return self.get_keyboard_geometry_information_from_logcat(api_tests_logcat_output)

    def press_key(self, key_string, keyboard=None):
        if keyboard is None:
            x, y = self.keyboard.get_key_coordinates(key_string)
        else:
            x, y = keyboard.get_key_coordinates(key_string)
        self.assertIsNotNone(x, "could not identify keyboard coordinates on screen")
        LOG.info("pressing key: " + key_string + " coordinates: %s %s " % (x, y))
        d.click(x, y)
        time.sleep(1)

    def long_press_key(self, key_string, keyboard=None, press_time_sec=2):
        if keyboard is None:
            x, y = self.keyboard.get_key_coordinates(key_string)
        else:
            x, y = keyboard.get_key_coordinates(key_string)
        self.assertIsNotNone(x, "could not identify keyboard coordinates on screen")
        swipe_steps = UiAutomatorUtils.nr_of_swipe_steps_that_take_one_second * press_time_sec
        d.swipe(x, y, x, y, swipe_steps)
        time.sleep(1)


class LatinKeyboardTests(GenericKeyboardTests):
    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation
        ktopx, ktopy, kwidth, kheight = self.get_keyboard_geometry()
        self.assertIsNotNone(ktopx, "could not get keyboard geometry")
        self.keyboard = GoogleKeyboard("Latin_IM", int(ktopx), int(ktopy), int(kwidth), int(kheight))
        LOG.info("got keyboard geometry: " + ktopx + " " + ktopy + " " + kwidth + " " + kheight)
        app_found = UiAutomatorUtils.is_app_in_apps_menu(test_app_name)
        self.assertTrue(app_found, "Input Test application not found")
        UiAutomatorUtils.launch_app_from_apps_menu(test_app_name)

    def test_type_numbers_from_letters(self):
        self.click_edit_text()
        time.sleep(2)
        for letter in "qwertyuiop":
            ax, ay = self.keyboard.get_key_coordinates(letter)
            self.assertIsNotNone(ax, "keyboard does not have letter" + str(letter))
            d.long_click(ax, ay)
            time.sleep(2)
        inputed_text = self.get_edit_text()
        self.assertTrue(inputed_text.isdigit(), "input text does not contain only digits")

    def test_shift_behaviour(self):
        self.click_edit_text()
        self.press_key("lshift")
        self.press_key("a")
        self.press_key("a")
        self.press_key("lshift")
        self.press_key("l")
        self.press_key("l")
        inputed_text = self.get_edit_text()
        # check to see if inputed text is mixed case
        self.assertTrue(not inputed_text.islower() and not inputed_text.isupper() and len(inputed_text) == 4,
                        "shift key behaviour is not as expected")

    def test_caps_lock_behaviour(self):
        self.click_edit_text()
        ax, ay = self.keyboard.get_key_coordinates("lshift")
        self.assertIsNotNone(ax, "could not find lshift coordinates on keyboard")
        # double click
        d.click(ax, ay)
        d.click(ax, ay)
        for letter in "testcaps":
            self.press_key(letter)
        inputed_text = self.get_edit_text()
        self.assertTrue(inputed_text.isupper() and len(inputed_text) == 8,
                        "all input characters should be upper case and at least 8")

    def test_special_chars(self):
        self.click_edit_text()
        for letter in "test":
            self.press_key(letter)
        self.press_key("symbols")
        for letter in "task":
            self.press_key(letter)
        inputed_text = self.get_edit_text()
        self.assertTrue(len(inputed_text) == 8 and inputed_text[:4].isalpha and not inputed_text[5:].isalnum(),
                        "input text should contain only special chars")

    def test_gms_lock_screen(self):
        Settings.enable_screen_lock("Swipe")
        UiAutomatorUtils.launch_app_from_apps_menu(test_app_name)
        self.click_edit_text()
        d.press.power()
        time.sleep(10)
        power_output = AdbUtils.run_adb_cmd(dumpsys_power_cmd)
        self.assertTrue('OFF' in power_output, "screen is not yet OFF")
        d.press.power()
        time.sleep(10)

        UiAutomatorUtils.unlock_screen()
        power_output = AdbUtils.run_adb_cmd(dumpsys_power_cmd)
        self.assertTrue('ON' in power_output, "screen is not yet ON")
        for letter in "abc":
            self.press_key(letter)
        inputed_text = self.get_edit_text()
        self.assertTrue(len(inputed_text) == 3 and inputed_text.isalpha(),
                        "input text should contain some characters")

    def test_input_by_sliding(self):
        self.click_edit_text()
        wx, wy = self.keyboard.get_key_coordinates("w")
        self.assertIsNotNone(wx, "could not find w key coordinates")
        lx, ly = self.keyboard.get_key_coordinates("l")
        self.assertIsNotNone(lx, "could not find l key coordinates")
        d.swipe(wx, wy, lx, ly)
        time.sleep(2)
        ox, oy = self.keyboard.get_key_coordinates("o")
        self.assertIsNotNone(ox, "could not find o key coordinates")
        zx, zy = self.keyboard.get_key_coordinates("z")
        self.assertIsNotNone(zx,  "could not find z key coordinates")
        d.swipe(ox, oy, zx, zy)
        time.sleep(2)
        inputed_text = self.get_edit_text()
        alpha_chars_in_inputed_text = [c for c in inputed_text if c.isalpha()]
        self.assertTrue(len(alpha_chars_in_inputed_text) > 2, "sliding should generate input word")

    def determine_backspace_position(self):
        self.click_edit_text()
        self.press_key("a")
        self.press_key("backspace")
        inputed_text = self.get_edit_text()
        if "a" in inputed_text:
            # on sofia screens, the backspace key is in the position of the right shift on the big layout
            return "rshift"
        else:
            return "backspace"

    def test_delete_all_chars(self):
        backspace = self.determine_backspace_position()
        self.click_edit_text()
        for letter in "abc":
            self.press_key(letter)
        inputed_text = self.get_edit_text()
        self.assertTrue(len(inputed_text) >= 3, "input text should contain at least 3 letters")
        ax, ay = self.keyboard.get_key_coordinates(backspace)
        self.assertIsNotNone(ax, "could not find backspace key coordinates")
        d.long_click(ax, ay)
        time.sleep(2)
        inputed_text = self.get_edit_text()
        self.assertTrue(len(inputed_text) < 3 or inputed_text == "Test_edit", "input text should be empty")

    def test_rotate_screen(self):
        OrientationChanger.change_orientation("n")
        ktopx, ktopy, kwidth, kheight = self.get_keyboard_geometry()
        self.assertIsNotNone(ktopx, "could not get keyboard geometry")
        natural_orientation_keyboard = GoogleKeyboard("Latin_IM", int(ktopx), int(ktopy), int(kwidth), int(kheight))
        print "got natural orientation keyboard geometry: " + ktopx + " " + ktopy + " " + kwidth + " " + kheight

        OrientationChanger.change_orientation("l")
        ktopx, ktopy, kwidth, kheight = self.get_keyboard_geometry()
        self.assertIsNotNone(ktopx, "could not get keyboard geometry")
        left_orientation_keyboard = GoogleKeyboard("Latin_IM", int(ktopx), int(ktopy), int(kwidth), int(kheight))
        print "got left orientation keyboard geometry: " + ktopx + " " + ktopy + " " + kwidth + " " + kheight

        UiAutomatorUtils.launch_app_from_apps_menu(test_app_name)
        self.click_edit_text()
        # we are in "l" orientation
        for letter in "left":
            self.press_key(letter, left_orientation_keyboard)
        OrientationChanger.change_orientation("n")
        for letter in "nat":
            self.press_key(letter, natural_orientation_keyboard)
        inputed_text = self.get_edit_text()
        self.assertTrue(len(inputed_text) == 7, "input text should contain 7 characters")

    def tearDown(self):
        self.log_before_cleanup()
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        UiAutomatorUtils.close_all_tasks()


class PinyinKeyboardTests(GenericKeyboardTests):
    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation
        self.kc = KeyboardChanger()
        self.kc.change_keyboard(PINYIN_KEYBOARD_NAME)
        ktopx, ktopy, kwidth, kheight = self.get_keyboard_geometry()
        self.assertIsNotNone(ktopx, "could not get keyboard geometry")
        self.keyboard = PinyinKeyboard("Pinyin_IM", int(ktopx), int(ktopy), int(kwidth), int(kheight))
        self.english_keyboard = GoogleKeyboard("Latin_IM", int(ktopx), int(ktopy), int(kwidth), int(kheight))
        print "got keyboard geometry: " + ktopx + " " + ktopy + " " + kwidth + " " + kheight
        app_found = UiAutomatorUtils.is_app_in_apps_menu(test_app_name)
        self.assertTrue(app_found, "could not find test app")
        UiAutomatorUtils.launch_app_from_apps_menu(test_app_name)
        self.screenshooter = ScreenshotUtils()

    def change_keyboard(self, keyboard_name):
            kc = KeyboardChanger()
            kc.change_keyboard(keyboard_name)

    def select_pinyin_keyboard_type(self, keyboard_number=1):
        if keyboard_number == 1:
            self.press_key("pred1")
            self.press_key("q")

    @staticmethod
    def is_chinese_string(string):
        return all(u'\u4e00' <= c <= u'\u9fff' for c in string)

    def tearDown(self):
        try:
            self.kc.change_keyboard(GOOGLE_KEYBOARD_NAME)
        except Exception as e:
            LOG.info("error while setting keyboard back to latin")
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        UiAutomatorUtils.close_all_tasks()
        self.screenshooter.remove_all_screenshots()
        self.log_before_cleanup()

    def test_type_numbers_from_letters(self):
        self.click_edit_text()
        self.select_pinyin_keyboard_type()
        for letter in "qwertyuiop":
            ax, ay = self.keyboard.get_key_coordinates(letter)
            self.assertIsNotNone(ax, "could not get letter coordinates for: " + str(letter))
            d.long_click(ax, ay)
            time.sleep(2)
        inputed_text = self.get_edit_text()
        for digit in "1234567890":
            self.assertTrue(digit in inputed_text, "digit " + str(digit) + "should pe present in input text box")

    def test_special_chars(self):
        self.click_edit_text()
        self.select_pinyin_keyboard_type()
        for letter in "test":
            self.press_key(letter)
        self.press_key("symbols")
        for letter in "txm":
            self.press_key(letter)
        inputed_text = self.get_edit_text()
        self.assertTrue('test5=' in inputed_text.lower().encode('ascii', 'ignore'),
                        "input text box should contain all the following chars: 'test5='")

    def test_switch_between_chinese_and_english(self):
        self.click_edit_text()
        self.select_pinyin_keyboard_type()
        for i in range(3):
            self.press_key("change")
            for letter in "test":
                self.press_key(letter)
            self.press_key("change", self.english_keyboard)
            for letter in "shin":
                self.press_key(letter)
            self.press_key("pred1")
        inputed_text = u'' + self.get_edit_text()
        self.assertTrue(u'test' in inputed_text.lower() and
                        self.is_chinese_string(inputed_text.replace("test", "")),
                        "input text box should contain chinese characters")

    def test_lock_screen(self):
        self.click_edit_text()
        self.select_pinyin_keyboard_type()
        d.press.power()
        time.sleep(2)
        power_output = AdbUtils.run_adb_cmd(dumpsys_power_cmd)
        self.assertTrue('OFF' in power_output, "screen is not yet OFF")
        d.press.power()
        time.sleep(2)
        UiAutomatorUtils.unlock_screen()
        power_output = AdbUtils.run_adb_cmd(dumpsys_power_cmd)
        self.assertTrue('ON' in power_output, "screen is not yet ON")
        for letter in "shin":
            self.press_key(letter)
        self.press_key("pred1")
        inputed_text = u'' + self.get_edit_text()
        self.assertTrue(self.is_chinese_string(inputed_text), "input text box should contain chinese characters")

    def test_input_chinese_characters(self):
        self.click_edit_text()
        self.select_pinyin_keyboard_type()
        for letter in "shin":
            self.press_key(letter)
        self.press_key("pred1")
        inputed_text = u'' + self.get_edit_text()
        self.assertTrue(self.is_chinese_string(inputed_text), "input text box should contain chinese characters")

    def test_input_by_sliding(self):
        self.click_edit_text()
        self.select_pinyin_keyboard_type()
        wx, wy = self.keyboard.get_key_coordinates("w")
        self.assertIsNotNone(wx, "could not find coordinates of w key")
        lx, ly = self.keyboard.get_key_coordinates("l")
        self.assertIsNotNone(lx, "could not find coordinates of l key")
        d.swipe(wx, wy, lx, ly)
        time.sleep(2)
        self.press_key("pred1")
        ox, oy = self.keyboard.get_key_coordinates("o")
        self.assertIsNotNone(ox, "could not find coordinates of o key")
        zx, zy = self.keyboard.get_key_coordinates("z")
        self.assertIsNotNone(zx, "could not find coordinates of z key")
        d.swipe(ox, oy, zx, zy)
        time.sleep(2)
        self.press_key("pred1")
        inputed_text = u'' + self.get_edit_text()
        print inputed_text
        self.assertTrue(self.is_chinese_string(inputed_text), "input text box should contain chinese characters")

    def test_enable_disable_pinyin(self):
        self.assertTrue(self.kc.is_keyboard_enabled(PINYIN_KEYBOARD_NAME), "PinYin keyboard is not enabled")
        UiAutomatorUtils.close_all_tasks()
        self.kc.disable_keyboard(PINYIN_KEYBOARD_NAME)
        self.assertFalse(self.kc.is_keyboard_enabled(PINYIN_KEYBOARD_NAME), "PinYin keyboard is still enabled")
        UiAutomatorUtils.close_all_tasks()

    def test_delete_all_chars(self):
        self.click_edit_text()
        self.select_pinyin_keyboard_type()
        for letter in "abc":
            self.press_key(letter)
            self.press_key("pred1")
        inputed_text = u'' + self.get_edit_text()
        ref_string = inputed_text
        self.assertTrue(self.is_chinese_string(inputed_text), "input text box should contain chinese characters")
        ax, ay = self.keyboard.get_key_coordinates("backspace")
        self.assertIsNotNone(ax, "could not find backspace key coordinates")
        d.long_click(ax, ay)
        time.sleep(2)
        inputed_text = self.get_edit_text()
        self.assertTrue(ref_string not in inputed_text, "backspace did not delete any chars")

    def test_rotate_screen(self):
        OrientationChanger.change_orientation("n")
        ktopx, ktopy, kwidth, kheight = self.get_keyboard_geometry()
        self.assertIsNotNone(ktopx, "could not get keyboard geometry")
        natural_orientation_keyboard = PinyinKeyboard("Pinyin_IM", int(ktopx), int(ktopy), int(kwidth), int(kheight))
        print "got natural orientation keyboard geometry: " + ktopx + " " + ktopy + " " + kwidth + " " + kheight

        OrientationChanger.change_orientation("l")
        ktopx, ktopy, kwidth, kheight = self.get_keyboard_geometry()
        self.assertIsNotNone(ktopx, "could not get keyboard geometry")
        left_orientation_keyboard = PinyinKeyboard("Pinyin_IM", int(ktopx), int(ktopy), int(kwidth), int(kheight))
        print "got left orientation keyboard geometry: " + ktopx + " " + ktopy + " " + kwidth + " " + kheight

        UiAutomatorUtils.launch_app_from_apps_menu(test_app_name)
        self.click_edit_text()
        self.select_pinyin_keyboard_type()
        # we are in "l" orientation
        for letter in "shin":
            self.press_key(letter, left_orientation_keyboard)
        self.press_key("pred1", left_orientation_keyboard)
        OrientationChanger.change_orientation("n")
        for letter in "nat":
            self.press_key(letter, natural_orientation_keyboard)
        self.press_key("pred1", natural_orientation_keyboard)
        inputed_text = self.get_edit_text()
        self.assertTrue(self.is_chinese_string(inputed_text), "input text box should contain chinese chars")

    def test_change_keyboard_theme(self):
        Settings.select_pinyin_material_dark_theme()
        UiAutomatorUtils.launch_app_from_apps_menu(test_app_name)
        self.click_edit_text()
        self.screenshooter.take_screenshot()
        dark_theme_screenshot_data = self.screenshooter.crop(self.keyboard.topx, self.keyboard.topy,
                                                             self.keyboard.topx + self.keyboard.width,
                                                             self.keyboard.topy + self.keyboard.height)
        Settings.select_pinyin_material_light_theme()
        UiAutomatorUtils.launch_app_from_apps_menu(test_app_name)
        self.click_edit_text()
        self.screenshooter.take_screenshot()
        light_theme_screenshot_data = self.screenshooter.crop(self.keyboard.topx, self.keyboard.topy,
                                                              self.keyboard.topx + self.keyboard.width,
                                                              self.keyboard.topy + self.keyboard.height)
        self.assertTrue(dark_theme_screenshot_data != light_theme_screenshot_data,
                        "PinYin keyboard theme did not change during test")


class JapaneseKeyboardTests(GenericKeyboardTests):
    japanese_input_settings_activity = "com.google.android.inputmethod.japanese/.preference.MozcFragmentPreferenceActivity"

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        self.initial_orientation = d.orientation
        self.kc = KeyboardChanger()
        self.kc.change_keyboard(JAPANESE_KEYBOARD_NAME)
        ktopx, ktopy, kwidth, kheight = self.get_keyboard_geometry()
        self.assertIsNotNone(ktopx, "could not get keyboard geometry")
        self.keyboard = JapaneseKeyboard("Japanese_IM", int(ktopx), int(ktopy), int(kwidth), int(kheight))
        self.english_keyboard = GoogleKeyboard("Latin_IM", int(ktopx), int(ktopy), int(kwidth), int(kheight))
        print "got keyboard geometry: " + ktopx + " " + ktopy + " " + kwidth + " " + kheight
        app_found = UiAutomatorUtils.is_app_in_apps_menu(test_app_name)
        self.assertTrue(app_found, "could not find test app")
        UiAutomatorUtils.launch_app_from_apps_menu(test_app_name)
        self.screenshooter = ScreenshotUtils()

    def change_keyboard(self, keyboard_name):
            kc = KeyboardChanger()
            kc.change_keyboard(keyboard_name)

    @staticmethod
    def is_japanese_string(string):
        for char in string:
            if not u'\u4e00' <= char <= u'\u9fff' and\
                not u'\u3041' <= char <= u'\u3094' and\
                    not u'\u30A1' <= char <= u'\u30FB':
                return False
        return True

    def tearDown(self):
        try:
            self.kc.change_keyboard(GOOGLE_KEYBOARD_NAME)
        except Exception as e:
            LOG.info("error while setting keyboard back to latin")
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        UiAutomatorUtils.close_all_tasks()
        self.screenshooter.remove_all_screenshots()
        self.log_before_cleanup()

    def test_input_japanese_chars(self):
        self.click_edit_text()
        for kana in ["ya", "ma"]:
            self.press_key(kana)
        self.press_key("sugestion_1")
        inputed_text = u'' + self.get_edit_text()
        self.assertTrue(self.is_japanese_string(inputed_text), "input text box should contain japanese chars")

    def test_switch_between_kana_and_symbols_chars(self):
        self.click_edit_text()
        for kana in ["ya", "ma"]:
            self.press_key(kana)
        self.press_key("numbers")
        self.press_key("a")
        time.sleep(2)  # wait for yama suggestion to pop up
        self.press_key("sugestion_1")
        for kana in ["ka", "sa"]:
            self.press_key(kana)
        self.press_key("latin")
        for kana in ["ya", "ma"]:
            self.press_key(kana)
        inputed_text = u'' + self.get_edit_text()
        self.assertTrue(u'やま' in inputed_text, "input text box should contain やま hiragana")
        self.assertTrue(u'123' in inputed_text, "input text box should contain 123 number")
        self.assertTrue(u'山' in inputed_text, "input text box should contain 山 kanji")

    def test_enable_disable_japanese_ime(self):
        kc = KeyboardChanger()
        kc.disable_keyboard(JAPANESE_KEYBOARD_NAME)
        d.press.back()
        Settings.go_to_main_screen()
        self.assertFalse(kc.is_keyboard_enabled(JAPANESE_KEYBOARD_NAME), "could not DISABLE Japanese keyboard")
        kc.enable_keyboard(JAPANESE_KEYBOARD_NAME)
        d.press.back()
        Settings.go_to_main_screen()
        self.assertTrue(kc.is_keyboard_enabled(JAPANESE_KEYBOARD_NAME), "could not ENABLE Japanese keyboard")

    def select_other_keyboard(self, keyboard_name):
        """ legacy code - not working on Sophia
        self.screenshooter.take_screenshot()
        keyboard_icon_x, keyboard_icon_y = self.screenshooter.\
                get_first_pixel_of_color_from_ref_with_direction(CHROME_KEYBOARD_SWITCH_ICON_LOWER_BAR_COLOR,
                                                                 (self.screenshooter.screen_width - 3,
                                                                  self.screenshooter.screen_height - 3),
                                                                 (-1, -1), color_error=30)
        LOG.info("found keyboard icon at: %s %s" % (keyboard_icon_x, keyboard_icon_y))
        AdbUtils.tap(keyboard_icon_x, keyboard_icon_y)
        self.assertTrue(d(textContains=keyboard_name).wait.exists(timeout=4000),
                        "could not find english keyboard in popup selection menu")
        d(textContains=keyboard_name).click()
        time.sleep(2)
        """
        self.long_press_key("latin")
        self.assertTrue(d(textContains="Select").wait.exists(timeout=3000), "select keyboard option not available")
        d(textContains="Select").click()
        self.assertTrue(d(textContains=keyboard_name).wait.exists(timeout=4000),
                        "could not find english keyboard in popup selection menu")
        d(textContains=keyboard_name).click()
        time.sleep(2)

    def test_switch_between_japanese_english(self):
        self.click_edit_text()
        couple_times = 2
        for times in range(couple_times):
            time.sleep(1)
            for kana in ["ma", "a"]:
                self.press_key(kana)
            time.sleep(2)  # wait for suggestions to show up
            self.press_key("sugestion_1")
            time.sleep(2)
            self.press_key("change_keyboard")
            for key in "la":
                self.english_keyboard.press_key(key)
            # little hack ... should have used english keyboard press_key("change")
            self.press_key("change_keyboard")
        inputed_text = u'' + self.get_edit_text()
        self.assertTrue(len([i for i in inputed_text if i == "a"]) == couple_times,
                        "could not find a's in input text")
        self.assertTrue(len([i for i in inputed_text if i == "l" or i == "k"]) == couple_times,
                        "could not find l's in input text")
        self.assertTrue(self.is_japanese_string(inputed_text.replace("l", "").replace("k", "").replace("a", "")),
                        "japanese sub-strings could not be found")

    def test_take_effect_with_browser(self):
        # Open Chrome
        Chrome.launch()

        # Select the address bar
        d(resourceId=CHROME_ADDRESS_BAR_RESID).click()
        time.sleep(3)

        # Input some text
        for kana in ["ma", "a"]:
            self.press_key(kana)
            time.sleep(2)  # wait for suggestions to show up
            self.press_key("sugestion_1")
            time.sleep(2)

        # Get the inputed text
        inputed_text = u'' + d(resourceId=CHROME_ADDRESS_BAR_RESID).info["text"]
        self.assertTrue(self.is_japanese_string(inputed_text), "Japanese sub-strings not found")

    def test_lock_rotate_japanese_ime(self):
        OrientationChanger.change_orientation("n")
        ktopx, ktopy, kwidth, kheight = self.get_keyboard_geometry()
        self.assertIsNotNone(ktopx, "could not get keyboard geometry")
        natural_orientation_keyboard = JapaneseKeyboard("JP_IM", int(ktopx), int(ktopy), int(kwidth), int(kheight))
        LOG.info("got natural orientation keyboard geometry: " + ktopx + " " + ktopy + " " + kwidth + " " + kheight)

        OrientationChanger.change_orientation("l")
        ktopx, ktopy, kwidth, kheight = self.get_keyboard_geometry()
        self.assertIsNotNone(ktopx, "could not get keyboard geometry")
        left_orientation_keyboard = JapaneseKeyboard("JP_IM", int(ktopx), int(ktopy), int(kwidth), int(kheight))
        LOG.info("got left orientation keyboard geometry: " + ktopx + " " + ktopy + " " + kwidth + " " + kheight)

        UiAutomatorUtils.launch_app_from_apps_menu(test_app_name)
        self.click_edit_text()
        # lock screen
        time.sleep(2)
        d.press.power()
        time.sleep(10)
        UiAutomatorUtils.unlock_screen()
        # we are in "l" orientation
        for kana in ["wa"]:
            self.press_key(kana, left_orientation_keyboard)
        d.click(d.info['displayHeight'] / 2 , d.info['displayHeight'] * 4 / 5)
        time.sleep(1)
        OrientationChanger.change_orientation("n")
        for kana in ["ma"]:
            self.press_key(kana, natural_orientation_keyboard)
        inputed_text = self.get_edit_text()
        self.assertTrue(len(inputed_text) > 1 and self.is_japanese_string(inputed_text),
                        "input text should contain japanese substrings")

    def test_change_japanese_ime_theme(self):
        AdbUtils.start_activity_from_shell(JapaneseKeyboardTests.japanese_input_settings_activity)
        time.sleep(4)
        vn = ViewNavigator()
        vn.nagivate_text(["theme", "Light"])
        UiAutomatorUtils.launch_app_from_apps_menu(test_app_name)
        self.click_edit_text()
        time.sleep(2)
        self.screenshooter.take_screenshot()
        validation_crop_square_side = min(self.screenshooter.screen_height, self.screenshooter.screen_width)
        initial_crop = self.screenshooter.crop_center_square_pixels(validation_crop_square_side)
        AdbUtils.start_activity_from_shell(JapaneseKeyboardTests.japanese_input_settings_activity)
        time.sleep(4)
        vn = ViewNavigator()
        vn.nagivate_text(["theme", "Dark"])

        UiAutomatorUtils.launch_app_from_apps_menu(test_app_name)
        self.click_edit_text()
        self.screenshooter.take_screenshot()
        validation_crop_square_side = min(self.screenshooter.screen_height, self.screenshooter.screen_width)
        final_crop = self.screenshooter.crop_center_square_pixels(validation_crop_square_side)

        self.assertTrue(initial_crop != final_crop, "keyboard theme seems not to be properly applied")

        for kana in ["ma", "na"]:
            self.press_key(kana)
        inputed_text = self.get_edit_text()
        self.assertTrue(len(inputed_text) > 0 and self.is_japanese_string(inputed_text),
                        "could not input japanese substring with theme keyboard")

if __name__ == "__main__":
    test_result = SingleMethodRunner.run_single_test(LatinKeyboardTests, "test_type_numbers_from_letters")
