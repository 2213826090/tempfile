from PyUiApi.app_utils.settings_utils import *
from PyUiApi.adb_helper.instrumentation_utils import *


class Key:
    def __init__(self, name, length_units):
        self.name = name
        self.length_units = length_units


class KeyRow:
    def __init__(self, name, height_units):
        self.name = name
        self.height_units = height_units
        self.keys = []

    def add_key(self, key):
        self.keys.append(key)
        return self


class Keyboard(object):
    def __init__(self, name, topx, topy, width, height):
        self.name = name
        self.topx = topx
        self.topy = topy
        self.width = width
        self.height = height
        self.rows = []
        self.pixels_in_height_unit = self.height / 9
        self.pixels_in_width_unit = self.width / 22

    def get_key_position(self, key_string):
        height_units = 0
        width_units = 0
        for row in self.rows:
            height_units += row.height_units
            for key in row.keys:
                width_units += key.length_units
                if key_string == key.name:
                    # substract 1 to find the center of the key
                    return width_units - key.length_units / 2.0, height_units - row.height_units / 2.0
            width_units = 0
        return None, None

    def get_key_coordinates(self, key_string):
        width_units, height_units = self.get_key_position(key_string)
        if width_units is None:
            return None, None
        return int(self.topx + width_units * self.pixels_in_width_unit),\
            int(self.topy + height_units * self.pixels_in_height_unit)

    def press_key(self, key_string, keyboard=None):
        if keyboard is None:
            x, y = self.get_key_coordinates(key_string)
        else:
            x, y = keyboard.get_key_coordinates(key_string)
        d.click(x, y)
        time.sleep(1)


class GoogleKeyboard(Keyboard):
    def __init__(self, name, topx, topy, width, height):
        super(GoogleKeyboard, self).__init__(name, topx, topy, width, height)
        row_0 = KeyRow("row_0", 1)
        row_1 = KeyRow("row_1", 2)
        row_2 = KeyRow("row_2", 2)
        row_3 = KeyRow("row_3", 2)
        row_4 = KeyRow("row_4", 2)
        row_1.add_key(Key("q", 2)).add_key(Key("w", 2)).add_key(Key("e", 2)).add_key(Key("r", 2)).add_key(Key("t", 2))\
            .add_key(Key("y", 2)).add_key(Key("u", 2)).add_key(Key("i", 2)).add_key(Key("o", 2)).add_key(Key("p", 2))\
            .add_key(Key("backspace", 2))
        row_2.add_key(Key(None, 1)).add_key(Key("a", 2)).add_key(Key("s", 2)).add_key(Key("d", 2)).add_key(Key("f", 2))\
            .add_key(Key("g", 2)).add_key(Key("h", 2)).add_key(Key("j", 2)).add_key(Key("k", 2)).add_key(Key("l", 2))\
            .add_key(Key("enter", 3))
        row_3.add_key(Key("lshift", 2)).add_key(Key("z", 2)).add_key(Key("x", 2)).add_key(Key("c", 2)).add_key(Key("v", 2))\
            .add_key(Key("b", 2)).add_key(Key("n", 2)).add_key(Key("m", 2)).add_key(Key("!", 2)).add_key(Key("?", 2))\
            .add_key(Key("rshift", 2))
        row_4.add_key(Key("symbols", 2)).add_key(Key(",", 2)).add_key(Key("change", 2)).add_key(Key("space", 12))\
            .add_key(Key(".", 2)).add_key(Key("smileys", 2))
        self.rows.append(row_0)
        self.rows.append(row_1)
        self.rows.append(row_2)
        self.rows.append(row_3)
        self.rows.append(row_4)
        self.pixels_in_height_unit = self.height / 9
        self.pixels_in_width_unit = self.width / 22


class PinyinKeyboard(Keyboard):
    def __init__(self, name, topx, topy, width, height):
        super(PinyinKeyboard, self).__init__(name, topx, topy, width, height)
        row_0 = KeyRow("row_0", 1)
        row_1 = KeyRow("row_1", 2)
        row_2 = KeyRow("row_2", 2)
        row_3 = KeyRow("row_3", 2)
        row_4 = KeyRow("row_4", 2)
        row_0.add_key(Key("pred1", 3)).add_key(Key("pred2", 3)).add_key(Key("pred3", 3)).add_key(Key("pred4", 3))\
             .add_key(Key("pred5", 3)).add_key(Key("pred6", 3)).add_key(Key("pred7", 3)).add_key(Key("pred8", 3))\
             .add_key(Key("pred_more", 1))
        row_1.add_key(Key("q", 2)).add_key(Key("w", 2)).add_key(Key("e", 2)).add_key(Key("r", 2)).add_key(Key("t", 2))\
            .add_key(Key("y", 2)).add_key(Key("u", 2)).add_key(Key("i", 2)).add_key(Key("o", 2)).add_key(Key("p", 2))\
            .add_key(Key("backspace", 2))
        row_2.add_key(Key(None, 1)).add_key(Key("a", 2)).add_key(Key("s", 2)).add_key(Key("d", 2)).add_key(Key("f", 2))\
            .add_key(Key("g", 2)).add_key(Key("h", 2)).add_key(Key("j", 2)).add_key(Key("k", 2)).add_key(Key("l", 2))\
            .add_key(Key("enter", 3))
        row_3.add_key(Key("lshift", 2)).add_key(Key("z", 2)).add_key(Key("x", 2)).add_key(Key("c", 2)).add_key(Key("v", 2))\
            .add_key(Key("b", 2)).add_key(Key("n", 2)).add_key(Key("m", 2)).add_key(Key("!", 2)).add_key(Key("?", 2))\
            .add_key(Key("rshift", 2))
        row_4.add_key(Key("symbols", 2)).add_key(Key("DPAD", 2)).add_key(Key("/", 2)).add_key(Key("change", 2))\
            .add_key(Key("space", 8)).add_key(Key(",", 2)).add_key(Key("txt_smileys", 2)).add_key(Key("smileys", 2))
        self.rows.append(row_0)
        self.rows.append(row_1)
        self.rows.append(row_2)
        self.rows.append(row_3)
        self.rows.append(row_4)
        self.pixels_in_height_unit = self.height / 9
        self.pixels_in_width_unit = self.width / 22


class JapaneseKeyboard(Keyboard):
    def __init__(self, name, topx, topy, width, height):
        super(JapaneseKeyboard, self).__init__(name, topx, topy, width, height)
        row_0 = KeyRow("row_0", 1)
        row_1 = KeyRow("row_1", 2)
        row_2 = KeyRow("row_2", 2)
        row_3 = KeyRow("row_3", 2)
        row_4 = KeyRow("row_4", 2)
        row_0.add_key(Key("sugestion_1", 1)).add_key(Key("sugestion_2", 1)).add_key(Key("sugestion_3", 1))\
             .add_key(Key("sugestion_4", 1))
        row_1.add_key(Key("cycle_kanji", 1)).add_key(Key("a", 1)).add_key(Key("ka", 1))\
            .add_key(Key("sa", 1)).add_key(Key("back", 1))
        row_2.add_key(Key("left", 1)).add_key(Key("ta", 1)).add_key(Key("na", 1))\
            .add_key(Key("ha", 1)).add_key(Key("right", 1))
        row_3.add_key(Key("numbers", 1)).add_key(Key("ma", 1)).add_key(Key("ya", 1))\
            .add_key(Key("ra", 1)).add_key(Key("space", 1))
        row_4.add_key(Key("latin", 1)).add_key(Key("change_keyboard", 1)).add_key(Key("wa", 1))\
            .add_key(Key("symbols", 1)).add_key(Key("enter", 1))
        self.rows.append(row_0)
        self.rows.append(row_1)
        self.rows.append(row_2)
        self.rows.append(row_3)
        self.rows.append(row_4)
        self.pixels_in_height_unit = self.height / 9
        self.pixels_in_width_unit = self.width / 5

    def select_suggestion_shadow_row(self, sugestion_nr):
        shadow_x = self.topx + (sugestion_nr - 0.5) * self.pixels_in_width_unit
        shadow_y = self.topy - 0.5 * self.pixels_in_height_unit
        d.click(shadow_x, shadow_y)
        time.sleep(1)


class KeyboardGeometry(object):
    geometry_string = "keyboard geometry: "

    @staticmethod
    def get_keyboard_geometry_information_from_logcat(logcat_string):
        last_detected_geometry = None
        for line in logcat_string.split("\n"):
            if KeyboardGeometry.geometry_string in line:
                last_detected_geometry = line
        if last_detected_geometry is not None:
            geometry = last_detected_geometry.split(":")[-1].split(" ")
            geometry = [x for x in geometry if x != '']
            return geometry[0], geometry[1], geometry[2], geometry[3]
        return None, None, None, None

    @staticmethod
    def get_keyboard_geometry():
        ApiTestsInterface\
            .run_instrumentation(class_name="InputMethodTestsDriver",
                                 method_name="testGetKeyboardLocation",
                                 instrumentation_args=None,
                                 runner_name="GenericArgumentPassingTestRunner")
        api_tests_logcat_output = AdbUtils.run_adb_cmd(api_tests_logcat, adb_shell=False)
        return KeyboardGeometry.get_keyboard_geometry_information_from_logcat(api_tests_logcat_output)


class KeyboardChanger(object):
    def __init__(self):
        pass

    def change_keyboard(self, keyboard_name):
        Settings.go_to_main_screen()
        Settings.select_current_keyboard()
        if d(text=keyboard_name).wait.exists(timeout=2000):
            d(text=keyboard_name).click()
        else:
            vnav = ViewNavigator()
            vnav.nagivate_text([SETTINGS_CHOOSE_KEYBOARDS_OPTION_TXT, keyboard_name])
            d.press.back()
            vnav.nagivate_text([SETTINGS_CURRENT_KEYBOARD_OPTION_TXT, keyboard_name])
        time.sleep(1)

    def enable_keyboard(self, keyboard_name):
        Settings.select_current_keyboard()
        if d(text=keyboard_name).wait.exists(timeout=2000):
            d.press.back()
            return  # keyboard already enabled
        else:
            vnav = ViewNavigator()
            vnav.nagivate_text([SETTINGS_CHOOSE_KEYBOARDS_OPTION_TXT, keyboard_name])

    def disable_keyboard(self, keyboard_name):
        Settings.select_current_keyboard()
        if d(text=keyboard_name).wait.exists(timeout=2000):
            vnav = ViewNavigator()
            vnav.nagivate_text([SETTINGS_CHOOSE_KEYBOARDS_OPTION_TXT, keyboard_name])
        else:
            d.press.back()
            return  # keyboard already disabled

    def is_keyboard_enabled(self, keyboard_name):
        Settings.select_current_keyboard()
        keyboard_enabled = False
        if d(text=keyboard_name).wait.exists(timeout=2000):
            keyboard_enabled = True
        d.press.back()
        return keyboard_enabled

if __name__ == "__main__":
    # google_keyboard = GoogleKeyboard("Google_Keyboard", 0, 0, 400, 100)
    # print google_keyboard.get_key_position("m")
    kc = KeyboardChanger()
    # kc.change_keyboard("Chinese Pinyin")
    # kc.change_keyboard("Google Keyboard")
    kc.enable_keyboard("Chinese Pinyin")
    UiAutomatorUtils.close_all_tasks()
    kc.disable_keyboard("Chinese Pinyin")
    UiAutomatorUtils.close_all_tasks()
    kc.enable_keyboard("Chinese Pinyin")
