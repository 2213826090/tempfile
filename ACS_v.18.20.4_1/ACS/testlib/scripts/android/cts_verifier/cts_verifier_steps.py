from testlib.base.base_step import step as base_step
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils


class set_screen_rotation(ui_step):

    """ description:
            sets device screen rotation to false
        usage:
            cts_verifier.set_screen_rotation()()

        tags:
            ui, android, cts_verifier, rotation
    """

    def __init__(self, intent = True, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.intent = intent

    def do(self):
        if self.intent:
            adb_steps.am_start_command(serial = self.serial,
                                       component = "com.android.settings/.DisplaySettings")()
        else:
            ui_steps.open_settings_app(serial = self.serial,
                                       view_to_find = {"text": "Display"},
                                       view_to_check = {"text": "Sleep"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "When device is rotated"},
                              view_to_check = {"textContains": "Stay in"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"textContains": "Stay in"})()

    def check_condition(self):
        return self.uidevice(textContains = "Stay in").wait.exists(timeout = 5000)


class set_display(ui_step):

    """ description:
            sets Adaptive Brightess to OFF

        usage:
            cts_verifier_steps.set_display()()

        tags:
            ui, android, cts_verifier, display, brightness
    """
    def __init__(self, intent = True, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.intent = intent
        self.set_passm("Set Adaptive Brightness to false")
        self.set_errorm("", "Set Adaptive Brightness to false")

    def do(self):
        if self.intent:
            adb_steps.am_start_command(serial = self.serial,
                                       component = "com.android.settings/.DisplaySettings")()
        else:
            ui_steps.open_settings_app(serial = self.serial,
                                       view_to_find = {"text": "Display"},
                                       view_to_check = {"text": "Sleep"})()
        if self.uidevice(resourceId = "com.android.settings:id/switch_widget").exists:
            ui_steps.click_switch(serial = self.serial,
                                  view_to_find = {"resourceId":
                                                  "com.android.settings:"
                                                  "id/switch_widget"},
                                  state = "OFF")()


    def check_condition(self):
        return True


class set_location(ui_step):

    """ description:
            sets location to Off for CTS verifier run

        usage:
            cts_steps.set_location()()

        tags:
            ui, android, cts, location
    """
    def __init__(self, intent = True, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.intent = intent
        self.set_passm("Location is OFF")
        self.set_errorm("", "Location is OFF")

    def do(self):
        if self.intent:
            adb_steps.am_start_command(serial = self.serial,
                                       component = "com.android.settings/.Settings")()
            self.uidevice(text = "Location").wait.exists(timeout = 10000)
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"text": "Location"},
                                              view_to_check = {"text": "Mode"})()
        else:
            ui_steps.open_settings_app(serial = self.serial,
                                       view_to_find = {"text": "Location"},
                                       view_to_check = {"text": "Mode"})()
        ui_steps.click_switch(serial = self.serial,
                              view_to_find = {"resourceId":
                                              "com.android.settings:"
                                              "id/switch_widget"},
                              state = "OFF")()

    def check_condition(self):
        self.uidevice.wait.update()
        return self.uidevice(text = "Off").exists



class run_instrumentation(adb_step):

    """ description:
        runs an instrumentation test:
        - creates "am instrument" command with <argument_list>
        (-e arg1 val1 ... -e arg2 val2) waiting for script to finish
        (-w script)
        - runs the test
        - checks the output for pass message ("." default)

        usage:
        InstrumentTest(serial = "10.237.100.212:5555",
                       port = 17002,
                       argument_list = "-e class
com.android.music.tests.functional.TestPlaylist#testDeletePlaylist",
                       script =
"com.android.music.tests/android.test.InstrumentationTestRunner",
                       timeout = 100)()

        tags:
            android, instrument, instrumentation, runner
    """

    def __init__(self, argument_list = None, script = None, pass_message = ".",
            **kwargs):
        adb_step.__init__(self, **kwargs)
        self.am_command = "am instrument "
        if argument_list:
            for arg in argument_list:
                self.am_command += arg + " "
        if script:
            self.am_command += " -w " + script
        self.script = script
        self.pass_message = pass_message
        self.set_passm("Running instrumentation test " + self.am_command)

    def do(self):
        self.step_data =\
            self.adb_connection.parse_cmd_output(cmd = self.am_command,
                                                 timeout = 2400,
                                                 left_separator = "Time:")

    def check_condition(self):
        return None

