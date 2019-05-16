from testlib.base.base_step import step as base_step
from testlib.base.base_step import FailedError
from testlib.base import base_utils
from testlib.scripts.connections.local.local_step import step as local_step
from testlib.scripts.connections.local import local_steps
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.wireless.wifi_generic import wifi_generic_steps
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui.browser import browser_steps
from testlib.scripts.android.flash import flash_steps
from testlib.scripts.android.cts import suite_utils
from testlib.scripts.file  import file_utils
from testlib.scripts.mail  import mail_steps
from testlib.scripts.connections.local import local_utils
from testlib.utils.relay  import Relayed_device
from testlib.utils.statics.android import statics

import os
import time
from datetime import timedelta
import traceback
from multiprocessing import *

PLATFORM_DICT = {
    # ECS_E7
    "TREKSTOR" :"ECS_E7",
    "CLOUDFONE" :"ECS_E7",
    # ECS27B
    "ONE695": "ECS27B",
    "VSI7Q" : "ECS27B",
    # ECS28A
    "ONE8_0": "ECS28A",
    "VSI8Q": "ECS28A",
    "TC80RA3": "ECS28A",
    # ECS210A
    "TC10RA3": "ECS210A",
    "T10A2IG": "ECS210A",
    # MALATA8LOW
    "A82I": "MALATA8LOW",
    # MALATA10
    "A105I" : "MALATA10",
    # T15
    "IRA101": "T15",
    # Emdoor I8170
    "VTA0705": "Emdoor i8170",
    # Emdoor I8880
    "VTA0803": "Emdoor i8880",
}

class set_storage(ui_step):
    """ description:
            sets storage to internal for M

        usage:
            cts_steps.set_storage()()

        tags:
            ui, android, cts, storage, internal
    """

    def __init__(self, intent, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.set_passm("SD card type is set to internal type")
        self.set_errorm("", "SD card type is set to internal type")
        self.intent = intent
        self.wait_time = wait_time

    def do(self):
        if self.intent:
            adb_steps.am_start_command(serial = self.serial,
                                       component = "com.android.settings/.Settings")()
            self.uidevice(textContains = "Storage").wait.exists(timeout = 3 * self.wait_time)
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"textContains": "Storage"})()
        else:
            ui_steps.open_settings_app(serial = self.serial,
                                       view_to_find = {"textContains": "Storage"})()
        if not self.uidevice(textContains = "SD card").wait.exists(timeout = self.wait_time):
            print "[ {0} ]: No SIM card was inserted or recognized!!!".format(self.serial)
            return
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"textContains": "SD card"})()
        if self.uidevice(textContains = "Setup").wait.exists(timeout = self.wait_time):
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"textContains": "Setup"},
                                  view_to_check = {"textContains": "Use as"})()
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text": "Use as internal storage"})()
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text": "Next"},
                                  view_to_check = {"textContains": "Erase"})()
        elif self.uidevice(textContains = "Set up").wait.exists(timeout = self.wait_time):
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"textContains": "Set up"},
                                  view_to_check = {"textContains": "Use as"})()
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text": "Use as internal storage"})()
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text": "Next"},
                                  view_to_check = {"textContains": "Erase"})()
        elif self.uidevice(description = "More options").exists:
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"description": "More options"},
                                  view_to_check = {"text": "Settings"})()
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text": "Settings"},
                                  view_to_check = {"text": "Format"})()
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"textContains": "Format"},
                                  view_to_check = {"textContains": "Erase"})()

        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"textContains": "Erase & format"},
                              view_to_check = {"textContains": "Formatting"})()
        self.uidevice(textContains = " remove the SD card while ").wait.gone(timeout = 120 * self.wait_time)
        if self.uidevice(textContains = "Move").exists:
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text": "Move now"},
                                  view_to_check = {"text": "Next"})()
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text": "Next"},
                                  view_to_check = {"textContains": "During"})()
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text": "Move"})()
            self.uidevice(textContains = "Moving data").wait.gone(timeout = 120 * self.wait_time)
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "Done"})()
        ui_steps.press_home(serial = self.serial)()

    def check_condition(self):
        return True


class set_display(ui_step):

    """ description:
            sets sleep from display to 30 minutes for CTS run

        usage:
            cts_steps.set_display()()

        tags:
            ui, android, cts, display, sleep
    """
    def __init__(self, intent = True, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.intent = intent
        self.wait_time = wait_time
        self.set_passm("Sleep from display is set to 30 minutes")
        self.set_errorm("", "Sleep from display is set to 30 minutes")

    def do(self):
        if self.intent:
            adb_steps.am_start_command(serial = self.serial,
                                       component = "com.android.settings/.DisplaySettings")()
        else:
            ui_steps.open_settings_app(serial = self.serial,
                                       view_to_find = {"text": "Display"},
                                       view_to_check = {"text": "Sleep"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "Sleep"},
                              view_to_check = {"text": "30 minutes"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "30 minutes"})()

    def check_condition(self):
        return self.uidevice(text = "After 30 minutes of inactivity").wait.exists(timeout = self.wait_time)


class set_language(ui_step):

    """ description:
            sets language to English (United States) for CTS run

        usage:
            cts_steps.set_language()()

        tags:
            ui, android, cts, language
    """
    def __init__(self, intent = True, wait_time = 2000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.intent = intent
        self.wait_time = wait_time
        self.set_passm("Language is set to English (United State)")
        self.set_errorm("", "Language is set to English (United State)")

    def do(self):
        if self.intent:
            adb_steps.am_start_command(serial = self.serial,
                                       component = "com.android.settings/.LanguageSettings")()
        else:
            ui_steps.open_settings_app(serial = self.serial,
                                       view_to_find = {"text": "Language & input"},
                                       view_to_check = {"text": "Spell checker"})()

        if adb_utils.get_android_version(serial = self.serial) >= 'N':
            label = "Languages"
            view = "Add a language"
        else:
            label = "Language"
            view = "Afrikaans"
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": label},
                              view_to_check = {"text": view})()
        if adb_utils.get_android_version(serial = self.serial) >= 'N':
            if not self.uidevice(text = "English (United States)").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"text": "Add a language"},
                                      view_to_check = {"text": "Suggested"})()
                ui_steps.click_button_with_scroll(serial = self.serial,
                                                  view_to_find = {"text":
                                                                  "English"})()
                ui_steps.click_button_with_scroll(serial = self.serial,
                                                  view_to_find = {"text":
                                                                  "United States"})()
        else:
            ui_steps.click_button_with_scroll(serial = self.serial,
                                              view_to_find = {"text":
                                                              "English "
                                                              "(United States)"})()
            if self.uidevice(text = "OK").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"text": "OK"},
                                      view_to_check = {"text": "Spell checker"})()


    def check_condition(self):
        return self.uidevice(text = "English (United States)").wait.exists(timeout = self.wait_time)


class set_location(ui_step):

    """ description:
            sets location to On for CTS run

        usage:
            cts_steps.set_location()()

        tags:
            ui, android, cts, location
    """
    def __init__(self, intent = True, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.intent = intent
        self.set_passm("Location is ON")
        self.set_errorm("", "Location is ON")

    def do(self):
        if self.intent:
            adb_steps.am_start_command(serial = self.serial,
                                       component = "com.android.settings/.Settings")()
            self.uidevice(text = "Location").wait.exists(timeout = 20000)
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
                              state = "ON")()

    def check_condition(self):
        self.uidevice.wait.update()
        return self.uidevice(text = "On").exists


class set_screen_lock(ui_step):

    """ description:
            sets screen lock to None for CTS run

        usage:
            cts_steps.set_screen_lock()()

        tags:
            ui, android, cts, screen lock
    """
    def __init__(self, intent = True, **kwargs):
        self.intent = intent
        ui_step.__init__(self, **kwargs)
        self.set_passm("Screen lock is None")
        self.set_errorm("", "Screen lock is None")

    def do(self):
        if self.intent:
            adb_steps.am_start_command(serial = self.serial,
                                       component = "com.android.settings/.SecuritySettings")()
        else:
            ui_steps.open_settings_app(serial = self.serial,
                                       view_to_find = {"text": "Security"},
                                       view_to_check = {"text": "Screen lock"},
                                       wait_time = 20000)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "Screen lock"},
                              view_to_check = {"textContains": "Current screen lock"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "None"},
                              view_to_check = {"text": "Security"})()

    def check_condition(self):
        self.uidevice.wait.update()
        return self.uidevice(text = "None").exists


class enable_cts_device_admins(ui_step):

    """ description:
            authorizes device admins to CTSDeviceAdminReceivers for CTS run

        usage:
            cts_steps.enable_cts_device_admins()()

        tags:
            ui, android, cts, device admin
    """


    def __init__(self, intent = True, **kwargs):
        self.intent = intent
        ui_step.__init__(self, **kwargs)
        self.set_passm("Set device admins")
        self.set_errorm("", "Set device admins")

    def do(self):
        if self.intent:
            adb_steps.am_start_command(serial = self.serial,
                                       component = "com.android.settings/.SecuritySettings")()
        else:
            ui_steps.open_settings_app(serial = self.serial,
                                       view_to_find = {"text": "Security"},
                                       view_to_check = {"textContains":
                                                    "Device admin"})()
        ui_steps.click_button_with_scroll(serial = self.serial,
                              view_to_find = {"text": "Device administrators"},
                              view_to_check = {"text": "Personal"})()
        ui_steps.click_checkbox_button(serial = self.serial,
                                       view_to_find = {"text":
                        "android.deviceadmin.cts.CtsDeviceAdminReceiver"},
                                       confirm_view = {"text": "Activate"},
                                       view_to_check_after_confirm = {"text":
                                                                      "Personal"},
                                       relationship = "right")()
        ui_steps.click_checkbox_button(serial = self.serial,
                                       view_to_find = {"text":
                        "android.deviceadmin.cts.CtsDeviceAdminReceiver2"},
                                       confirm_view = {"text": "Activate"},
                                       view_to_check_after_confirm = {"text":
                                                                      "Personal"},
                                       relationship = "right")()

        ui_steps.click_checkbox_button(serial = self.serial,
                                       state = "OFF",
                                       view_to_find = {"text":
                                                        "Android Device Manager"},
                                       confirm_view = {"text": "Deactivate"},
                                       view_to_check_after_confirm = {"text":
                                                                      "Personal"},
                                       relationship = "right")()

        ui_steps.click_checkbox_button(serial = self.serial,
                                       state = "OFF",
                                       view_to_find = {"text":
                        "android.deviceadmin.cts.CtsDeviceAdminDeactivatedReceiver"},
                                       confirm_view = {"text": "Deactivate"},
                                       view_to_check_after_confirm = {"text":
                                                                      "Personal"},
                                       relationship = "right")()

    def check_condition(self):
        return True


class run_cts_command(local_step):

    """ description:
            runs a CTS command inside the <screen_name> screen. It checks
            "XML test result file generated" to validate the correct
            execution of the command. "add derivedplan" is also covered
            in this step
            it returns the results of the run

        usage:
            cts_steps.run_cts_command(cts_command = cts_command,
                                      screen_name = self.screen_name,
                                      cts_base_path = self.cts_base_path,
                                      timeout = self.timeout,
                                      list_results_command = self.list_results_command,
                                      cts_plans_dir = self.cts_plans_dir)()

        tags:
            android, cts, run, screen, results
    """
    def __init__(self,
                 cts_command,
                 cts_base_path,
                 screen_name = "irda_cts",
                 screen_log = "screenlog.0",
                 runner_type = None,
                 timeout = 288000,
                 list_results_command = "list results",
                 cts_plans_dir = "repository/plans/",
                 **kwargs):
        self.cts_command = cts_command
        self.last_session = suite_utils.get_session(screen_name = screen_name,
                                                         suite = "cts",
                                                         screen_log = screen_log)
        self.screen_name = screen_name
        self.screen_log = screen_log
        self.runner_type = runner_type
        self.timeout = timeout
        self.list_results_command = list_results_command
        self.cts_base_path = cts_base_path
        self.cts_plans_dir = cts_plans_dir
        local_step.__init__(self, **kwargs)
        print "Last session before run: {0}".format(self.last_session)
        if "continue-session" in self.cts_command:
            self.set_passm("CTS - continue_session: {0}".format(self.cts_command))
            self.set_errorm("", "CTS - continue_session: {0}".format(self.cts_command))
            self.last_session -= 1
        elif "derivedplan" in self.cts_command:
            self.set_passm("CTS - derivedplan: {0}".format(self.cts_command))
            self.set_errorm("", "CTS - derivedplan: {0}".format(self.cts_command))
        else:
            self.set_passm("CTS - run plan: {0}".format(self.cts_command))
            self.set_errorm("", "CTS - run plan: {0}".format(self.cts_command))

    def do(self):
        if "derivedplan" in self.cts_command:
            local_steps.screen_command(screen_name = self.screen_name,
                                       screen_log = self.screen_log,
                                       screen_command = self.cts_command)()
        elif self.cts_command == "bogus":
            pass
        else:
            grep_for = ""
            if self.runner_type == "plan_based":
                grep_for = "XML test result file generated"
            elif self.runner_type == "module_based":
                grep_for = "Result saved at:"
            print  "I am using grep_for:" + grep_for
            local_steps.screen_command(screen_name = self.screen_name,
                                       screen_command = self.cts_command,
                                       with_log = True,
                                       grep_for = grep_for,
                                       screen_log = self.screen_log,
                                       timeout = self.timeout)()

    def check_condition(self):
        if "derivedplan" in self.cts_command:
            time.sleep(5)
            plan = self.cts_command.split(" ")[3]
            return suite_utils.is_plan_created(plan = plan,
                                             suite_base_path = self.cts_base_path,
                                             suite_plans_dir = self.cts_plans_dir)

        original_run_timestamp = suite_utils.get_run_timestamp(screen_log = self.screen_log,
                                                      runner_type = self.runner_type)
        run_timestamp = suite_utils.format_timestamp(run_timestamp = original_run_timestamp,
                                                      runner_type = self.runner_type)
        print ("timestamp after formatting")
        print run_timestamp
        run_results = suite_utils.get_results_by_timestamp(run_timestamp,
                                                           suite = "cts",
                                                           list_results_command = self.list_results_command,
                                                           screen_name = self.screen_name,
                                                           screen_log = self.screen_log)

        # run_results = cts_utils.get_results_by_timestamp(screen_name = self.screen_name,
        #                                                      screen_log = self.screen_log,
        #                                                      list_results_command = self.list_results_command,
        #                                                      session_id = self.last_session + 1)


        self.step_data = run_timestamp, run_results, original_run_timestamp

        if self.step_data:
            return True
        else:
            return False


class cts_first_run(base_step):

    """ description:

        usage:

        tags:
    """

    def __init__(self,
                 devices,
                 serials,
                 cts_plan_name,
                 cts_module_name,
                 runner_type,
                 screen_name,
                 cts_base_path,
                 cts_binary_dir,
                 cts_binary,
                 list_results_command,
                 dessert = "M",
                 return_failing_tests = False,
                 cts_disable_reboot = True,
                 cts_skip_preconditions = True,
                 **kwargs):
        base_step.__init__(self, **kwargs)
        self.devices = devices
        self.serials = serials
        self.cts_plan_name = cts_plan_name
        self.cts_module_name = cts_module_name
        self.runner_type = runner_type
        self.screen_name = screen_name
        self.cts_base_path = cts_base_path
        self.cts_binary_dir= cts_binary_dir
        self.cts_binary = cts_binary
        self.list_results_command = list_results_command
        self.dessert = dessert
        self.cts_disable_reboot = cts_disable_reboot
        self.cts_skip_preconditions = cts_skip_preconditions
        self.return_failing_tests = return_failing_tests

    def do(self):
        first_cts_run = {
            "cts_plan": self.cts_plan_name,
            "cts_class": None,
            "cts_method": None,
            "cts_module": self.cts_module_name,
            "cts_test": None,
            "cts_dessert": self.dessert,
            "cts_session_id": None,
            "cts_result_type": None,
            "cts_continue_session": False,
            "cts_retry_session": '',
            "cts_derived_plan": False,
            "cts_run_timestamp": None,
            "cts_disable_reboot": self.cts_disable_reboot,
            "cts_skip_preconditions": self.cts_skip_preconditions,
            "loop_no": 0
        }
        start_time = base_utils.get_time_string()
        start_counter = time.time()

        screen_log = local_steps.create_screen(screen_name = self.screen_name,
                                               with_log = True)()
        local_steps.screen_command(screen_name = self.screen_name,
                                   screen_command = os.path.join(self.cts_base_path,
                                                    self.cts_binary_dir,
                                                    self.cts_binary),
                                   screen_log = screen_log)()
        self.serials = [device["serial"] for device in self.devices]
        cts_command = suite_utils.create_suite_run_command(serials = self.serials,
                                                           suite = "cts",
                                                           suite_run = first_cts_run,
                                                           runner_type = self.runner_type)
        print cts_command
        if "Unable to create the run command!" in cts_command:
            send_report(debug = True,
                        recipients = self.recipients,
                        critical = False,
                        email_sender = self.email_sender,
                        host = self.email_machine,
                        user = self.email_machine_user,
                        passwd = self.email_machine_password,
                        subject = "{0} - {1}: Error runnig CTS for {2} with {3}".format(self.cts_version,
                                                                                        self.cts_abi,
                                                                                        self.platform,
                                                                                        self.build_no),
                        objective = "There are no devices available when creating the cts run command for the first run")()
            import sys
            sys.exit(1)
        first_cts_run["cts_run_timestamp"], first_results, original_run_timestamp = run_cts_command(cts_command = cts_command,
                                        cts_base_path = self.cts_base_path,
                                        screen_name = self.screen_name,
                                        screen_log = screen_log,
                                        runner_type = self.runner_type)()
        ############
        # Continue sessions while notExecuted tests exist
        ############
        cts_run = first_cts_run
        results = first_results
        print "Done first loop: {0} on session {1}".format(results, first_cts_run["cts_run_timestamp"])
        if self.runner_type == "plan_based":
            if int(results["notExecuted"])  > 0:
                print "Continue session for not executed tests"
                target_not_executed_tests = 0
                not_executed_tests = 0
                not_executed_loop = 0
                while (int(results["notExecuted"]) != not_executed_tests) and (int(results["notExecuted"]) != target_not_executed_tests):

                    self.devices = reboot_devices(devices = self.devices,
                                                  blocking = True)()
                    self.serials = [device["serial"] for device in self.devices]
                    not_executed_tests = int(results["notExecuted"])
                    cts_run = suite_utils.create_continue_session(suite_run = cts_run,
                                                                  suite = "cts",
                                                                  list_results_command = self.list_results_command,
                                                                  screen_name = self.screen_name,
                                                                  screen_log = screen_log)
                    cts_command = suite_utils.create_suite_run_command(serials = self.serials,
                                                                   suite = "cts",
                                                                   suite_run = cts_run,
                                                                   runner_type = self.runner_type)
                    if "Unable to create the run command!" in cts_command:
                        send_report(debug = True,
                                    recipients = self.recipients,
                                    critical = False,
                                    email_sender = self.email_sender,
                                    host = self.email_machine,
                                    user = self.email_machine_user,
                                    passwd = self.email_machine_password,
                                    subject = "{0} - {1}: Error runnig CTS for {2} with {3}".format(self.cts_version,
                                                                                                    self.cts_abi,
                                                                                                    self.platform,
                                                                                                    self.build_no),
                                    objective = "There are no devices available when creating the cts run command for the continue session on the first run")()
                        import sys
                        sys.exit(1)
                    run_timestamp, results = run_cts_command(cts_command = cts_command,
                                              cts_base_path = self.cts_base_path,
                                              screen_name = self.screen_name,
                                              screen_log = screen_log)()
                    not_executed_loop += 1
                    print "Results continue session {0}: {1}".format(not_executed_loop, results)

                cts_run = first_cts_run
                cts_run["cts_run_timestamp"] = run_timestamp

        result = {}
        result["ww"] = "WW{0}".format(base_utils.get_ww())
        result["session"] = self.cts_plan_name
        result["tests_no"] = int(results["pass"]) + int(results["fail"]) + int(results["notExecuted"])
        result["pass"] = results["pass"]
        result["fail"] = results["fail"]
        result["notExecuted"] = results["notExecuted"]
        result["start_time"] = start_time
        result["end_time"] = base_utils.get_time_string()
        end_counter = time.time()
        result["duration"] = str(timedelta(seconds=int(end_counter-start_counter))).replace(':', '-')

        failing_tests = None
        if self.return_failing_tests:
            if self.runner_type == "plan_based":
                result_xml_path = os.path.join(self.cts_base_path,
                                               "repository/results/",
                                               first_cts_run["cts_run_timestamp"],
                                               "testResult.xml")
                failing_tests = suite_utils.get_fails_from_xml_file(result_xml_path)
                # first_cts_run["cts_run_timestamp"]
            elif self.runner_type == "module_based":
                result_xml_path = os.path.join(self.cts_base_path,
                                               "results/",
                                               original_run_timestamp,
                                               "test_result.xml")
                failing_tests = suite_utils.get_fails_from_xml_file_module_based(result_xml_path)

        self.step_data = result, first_cts_run, screen_log, failing_tests
        print "Done first session"

    def check_condition(self):
        if self.step_data:
            return True
        return False


class cts_retry_session(base_step):

    """ description

        usage:

        tags:
    """

    def __init__(self,
                 devices,
                 serials,
                 screen_name,
                 screen_log,
                 cts_base_path,
                 cts_plans_dir,
                 list_results_command,
                 last_cts_run,
                 runner_type,
                 return_failing_tests = False,
                 **kwargs):
        base_step.__init__(self, **kwargs)
        self.devices = devices
        self.serials = serials
        self.screen_name = screen_name
        self.screen_log = screen_log
        self.cts_base_path = cts_base_path
        self.cts_plans_dir = cts_plans_dir
        self.list_results_command = list_results_command
        self.last_cts_run = last_cts_run
        self.return_failing_tests = return_failing_tests
        self.runner_type = runner_type

    def do(self):
        start_time = base_utils.get_time_string()
        start_counter = time.time()

        cts_run = self.last_cts_run
        suite = "cts"
        cts_run["cts_module"] = ""
        cts_run["cts_retry_session"] = suite_utils.get_session_id_by_timestamp(cts_run["{0}_run_timestamp".format(suite)],
                                                                                suite,
                                                                                self.list_results_command,
                                                                                self.screen_name,
                                                                                self.screen_log)["session_id"]
        cts_command = suite_utils.create_suite_run_command(serials = self.serials,
                                                       suite = "cts",
                                                       suite_run = cts_run,
                                                       runner_type = self.runner_type)
        if "Unable to create the run command!" in cts_command:
            send_report(debug = True,
                        recipients = self.recipients,
                        critical = False,
                        email_sender = self.email_sender,
                        host = self.email_machine,
                        user = self.email_machine_user,
                        passwd = self.email_machine_password,
                        subject = "{0} - {1}: Error runnig CTS for {2} with {3}".format(self.cts_version,
                                                                                        self.cts_abi,
                                                                                        self.platform,
                                                                                        self.build_no),
                        objective = "There are no devices available when creating the cts run command for a retry session")()
        print "Running command: {0}".format(cts_command)
        cts_run["cts_run_timestamp"], results, original_run_timestamp = run_cts_command(cts_command = cts_command,
                                  cts_base_path = self.cts_base_path,
                                  screen_name = self.screen_name,
                                  screen_log = self.screen_log,
                                  runner_type = self.runner_type)()
        first_cts_run = cts_run

        result = {}
        result["ww"] = "WW{0}".format(base_utils.get_ww())
        result["session"] = new_plan
        result["tests_no"] = int(results["pass"]) + int(results["fail"]) + int(results["notExecuted"])
        result["pass"] = results["pass"]
        result["fail"] = results["fail"]
        result["notExecuted"] = results["notExecuted"]
        result["start_time"] = start_time
        result["end_time"] = base_utils.get_time_string()
        end_counter = time.time()
        result["duration"] = str(timedelta(seconds=int(end_counter-start_counter))).replace(':', '-')

        failing_tests = None
        if self.return_failing_tests:
            if self.runner_type == "plan_based":
                result_xml_path = os.path.join(self.cts_base_path,
                                               "repository/results/",
                                               first_cts_run["cts_run_timestamp"],
                                               "testResult.xml")
                failing_tests = suite_utils.get_fails_from_xml_file(result_xml_path)
            elif self.runner_type == "module_based":
                result_xml_path = os.path.join(self.cts_base_path,
                                               "results/",
                                               original_run_timestamp,
                                               "test_result.xml")
                failing_tests = suite_utils.get_fails_from_xml_file_module_based(result_xml_path)

        self.step_data = result, cts_run, failing_tests

    def check_condition(self):
        if self.step_data:
            return True
        return False


class cts_derived_plan_run(base_step):

    """ description

        usage:

        tags:
    """

    def __init__(self,
                 devices,
                 serials,
                 screen_name,
                 screen_log,
                 cts_base_path,
                 cts_plans_dir,
                 list_results_command,
                 last_cts_run,
                 runner_type,
                 return_failing_tests = False,
                 **kwargs):
        base_step.__init__(self, **kwargs)
        self.devices = devices
        self.serials = serials
        self.screen_name = screen_name
        self.screen_log = screen_log
        self.cts_base_path = cts_base_path
        self.cts_plans_dir = cts_plans_dir
        self.list_results_command = list_results_command
        self.last_cts_run = last_cts_run
        self.return_failing_tests = return_failing_tests
        self.runner_type = runner_type

    def do(self):
        start_time = base_utils.get_time_string()
        start_counter = time.time()
        cts_run = suite_utils.create_derived_plan(suite_run = self.last_cts_run,
                                                  suite = "cts",
                                                  suite_base_path = self.cts_base_path,
                                                  suite_plans_dir = self.cts_plans_dir,
                                                  plan_suffix = self.screen_name,
                                                  list_results_command = self.list_results_command,
                                                  screen_name = self.screen_name,
                                                  screen_log = self.screen_log)
        cts_command = suite_utils.create_suite_run_command(serials = self.serials,
                                                       suite = "cts",
                                                       suite_run = cts_run,
                                                       runner_type = self.runner_type)
        print "Create derived plan: {0}".format(cts_command)
        run_cts_command(cts_command = cts_command,
                        cts_base_path = self.cts_base_path,
                        screen_name = self.screen_name,
                        screen_log = self.screen_log)()
        new_plan = cts_run["cts_plan"]

        cts_run = self.last_cts_run
        cts_run["cts_plan"] = new_plan
        cts_command = suite_utils.create_suite_run_command(serials = self.serials,
                                                       suite = "cts",
                                                       suite_run = cts_run,
                                                       runner_type = self.runner_type)
        if "Unable to create the run command!" in cts_command:
            send_report(debug = True,
                        recipients = self.recipients,
                        critical = False,
                        email_sender = self.email_sender,
                        host = self.email_machine,
                        user = self.email_machine_user,
                        passwd = self.email_machine_password,
                        subject = "{0} - {1}: Error runnig CTS for {2} with {3}".format(self.cts_version,
                                                                                        self.cts_abi,
                                                                                        self.platform,
                                                                                        self.build_no),
                        objective = "There are no devices available when creating the cts run command for a derived plan")()
        print "Running plan: {0}".format(cts_command)
        cts_run["cts_run_timestamp"], results, original_run_timestamp = run_cts_command(cts_command = cts_command,
                                  cts_base_path = self.cts_base_path,
                                  screen_name = self.screen_name,
                                  screen_log = self.screen_log)()
        first_cts_run = cts_run
        if int(results["notExecuted"])  > 0:
            print "Continue session for not executed tests for derived plan {0}".format(cts_run["cts_plan"])
            not_executed_tests = 0
            not_executed_loop = 0
            while int(results["notExecuted"]) != not_executed_tests:
                self.devices = reboot_devices(devices = self.devices,
                                          blocking = True)()
                self.serials = [device["serial"] for device in self.devices]
                not_executed_tests = int(results["notExecuted"])
                cts_run = suite_utils.create_continue_session(suite_run = cts_run,
                                                              suite = "cts",
                                                              list_results_command = self.list_results_command,
                                                              screen_name = self.screen_name,
                                                              screen_log = screen_log)
                cts_command = suite_utils.create_suite_run_command(serials = self.serials,
                                                               suite = "cts",
                                                               suite_run = cts_run,
                                                               runner_type = self.runner_type)
                if "Unable to create the run command!" in cts_command:
                    send_report(debug = True,
                                recipients = self.recipients,
                                critical = False,
                                email_sender = self.email_sender,
                                host = self.email_machine,
                                user = self.email_machine_user,
                                passwd = self.email_machine_password,
                                subject = "{0} - {1}: Error runnig CTS for {2} with {3}".format(self.cts_version,
                                                                                                self.cts_abi,
                                                                                                self.platform,
                                                                                                self.build_no),
                                objective = "There are no devices available when creating the cts run command for a continue session on a derived plan")()
                run_timestamp, results = run_cts_command(cts_command = cts_command,
                                          cts_base_path = self.cts_base_path,
                                          screen_name = self.screen_name,
                                          screen_log = self.screen_log)()
                not_executed_loop += 1
                print "Results continue session {0}: {1}".format(not_executed_loop, results)
            cts_run = first_cts_run
            cts_run["run_timestamp"] = run_timestamp
        result = {}
        result["ww"] = "WW{0}".format(base_utils.get_ww())
        result["session"] = new_plan
        result["tests_no"] = int(results["pass"]) + int(results["fail"]) + int(results["notExecuted"])
        result["pass"] = results["pass"]
        result["fail"] = results["fail"]
        result["notExecuted"] = results["notExecuted"]
        result["start_time"] = start_time
        result["end_time"] = base_utils.get_time_string()
        end_counter = time.time()
        result["duration"] = str(timedelta(seconds=int(end_counter-start_counter))).replace(':', '-')

        failing_tests = None
        if self.return_failing_tests:
            if self.runner_type == "plan_based":
                result_xml_path = os.path.join(self.cts_base_path,
                                               "repository/results/",
                                               first_cts_run["cts_run_timestamp"],
                                               "testResult.xml")
                failing_tests = suite_utils.get_fails_from_xml_file(result_xml_path)
                # first_cts_run["cts_run_timestamp"]
            elif self.runner_type == "module_based":
                result_xml_path = os.path.join(self.cts_base_path,
                                               "results/",
                                               original_run_timestamp,
                                               "test_result.xml")
                failing_tests = suite_utils.get_fails_from_xml_file_module_based(result_xml_path)

        self.step_data = result, cts_run, failing_tests

    def check_condition(self):
        if self.step_data:
            return True
        return False



class run_cts_plan(base_step):

    """ description:
            runs the CTS <cts_plan> plan on the devices described by
            <devices> serial list and follows the Google procedure:
            - executes first loop
            - calls continue_session until all packages are executed or
            the non executed packages remain the same (max 5 iteration)
            - executes derivedplan loops 2 times
            - sends report via e-mail after each loop to <recipients>
            list
            plan execution is done in the <screen_name> screen

        usage:
            cts_steps.run_cts_plan(cts_plan_name = "CTS",
                       cts_base_path = cts_base_path,
                       cts_binary_dir = cts_binary_dir,
                       cts_binary = cts_binary,
                       devices = device_serials,
                       recipients = recipients,
                       debug_recipients = debug_recipients,
                       screen_name = "irda",
                       list_results_command = list_results_command,
                       cts_plans_dir = cts_plans_dir,
                       timeout = 28800,
                       build_no = build_no,
                       image = project_id + build_no + target_image,
                       bios = bios,
                       cts_version = cts_version,
                       platform = platform,
                       cts_mail_report_path = "/home/oane/Work/automation/testlib/results/",
                       cts_mail_report_template = "cts_mail_report_template.html",
                       email_sender = "SSH",
                       host = "10.237.112.149",
                       user = "regression",
                       password = "regressioncts",
                       remote_path = "/opt/regression/mail"
                       )()

        tags:
            android, cts, run, plan, screen, results, email, report
    """
    def __init__ (self,
                  screen_name,
                  devices,
                  serials,
                  cts_abi,
                  cts_plan_name,
                  cts_module_name,
                  cts_base_path,
                  cts_binary_dir,
                  cts_tcs_dir,
                  cts_plans_dir,
                  cts_results_dir,
                  cts_logs_dir,
                  cts_binary,
                  cts_version,
                  loops_no,
                  build_no,
                  bios,
                  platform,
                  target,
                  recipients,
                  debug_recipients,
                  cts_disable_reboot = True,
                  cts_skip_preconditions = True,
                  cts_mail_report_failing_tests = False,
                  user_build_variant = 'user',
                  list_results_command = "list results",
                  exit_command = "exit",
                  cts_mail_report_template = "cts_mail_report_template.html",
                  email_sender = "RELAY",
                  email_machine = "10.237.112.149",
                  email_machine_user = "regression",
                  email_machine_password = "regressioncts",
                  dessert = "M",
                  remote_path = "/opt/regression/mail",
                  timeout = 28800,
                  **kwargs):
        self.screen_name = screen_name
        self.serials = serials
        self.devices = devices
        self.cts_abi = cts_abi
        self.cts_plan_name = cts_plan_name
        self.cts_module_name = cts_module_name
        self.cts_base_path = cts_base_path
        self.cts_binary_dir = cts_binary_dir
        self.cts_tcs_dir = cts_tcs_dir
        self.cts_plans_dir = cts_plans_dir
        self.cts_results_dir = cts_results_dir
        self.cts_logs_dir = cts_logs_dir
        self.cts_binary = cts_binary
        self.cts_version = cts_version
        self.cts_disable_reboot = cts_disable_reboot
        self.cts_skip_preconditions = cts_skip_preconditions
        self.cts_mail_report_failing_tests = cts_mail_report_failing_tests
        self.loops_no = loops_no
        self.build_no = build_no
        self.bios = bios
        self.platform = platform
        self.target = target
        self.user_build_variant = user_build_variant
        self.recipients = recipients
        self.debug_recipients = debug_recipients
        self.list_results_command = list_results_command
        self.exit_command = exit_command
        if "RESOURCES_FOLDER" in os.environ:
            self.cts_mail_report_path = os.environ["RESOURCES_FOLDER"]
        else:
            self.cts_mail_report_path = os.path.join(os.path.realpath(__file__).split("/scripts")[0], "results")
        self.cts_mail_report_template = cts_mail_report_template
        self.email_sender = email_sender
        self.email_machine = email_machine
        self.email_machine_user = email_machine_user
        self.email_machine_password = email_machine_password
        self.dessert = dessert
        self.remote_path = remote_path
        self.timeout = timeout
        base_step.__init__(self, **kwargs)

    def do(self):
        ############
        # Run CTS plan
        ############
        platform = self.platform
        send_report(debug = True,
                    recipients = self.recipients,
                    build_no = self.build_no,
                    critical = False,
                    email_sender = self.email_sender,
                    host = self.email_machine,
                    user = self.email_machine_user,
                    passwd = self.email_machine_password,
                    subject = "{0} - {1}: on {2} with {3} image".format(self.cts_version,
                                                                        self.cts_abi,
                                                                        self.platform,
                                                                        self.build_no),
                    objective = "Android CTS {0} - {1} on {2} with {3} {4} image was started on {5} shards.".format(self.cts_version,
                                                                                                                    self.cts_abi,
                                                                                                                    self.platform,
                                                                                                                    self.build_no,
                                                                                                                    self.user_build_variant,
                                                                                                                    len(self.serials)))()
        res = []

        runner_type = statics.Device(dessert = self.dessert).cts_runner_type

        first_results, first_cts_run, screen_log, failing_tests = cts_first_run(devices = self.devices,
                                                     serials =self.serials,
                                                     cts_plan_name = self.cts_plan_name,
                                                     cts_module_name = self.cts_module_name,
                                                     runner_type = runner_type,
                                                     screen_name = self.screen_name,
                                                     cts_base_path = self.cts_base_path,
                                                     cts_binary_dir = self.cts_binary_dir,
                                                     cts_binary = self.cts_binary,
                                                     cts_disable_reboot = self.cts_disable_reboot,
                                                     cts_skip_preconditions = self.cts_skip_preconditions,
                                                     dessert = self.dessert,
                                                     list_results_command = self.list_results_command,
                                                     return_failing_tests = self.cts_mail_report_failing_tests)()
        res.append(first_results)

        send_report(cts_mail_report_path = self.cts_mail_report_path,
                    recipients = self.recipients,
                    result_status = " first run ",
                    build_no = self.build_no,
                    critical = False,
                    email_sender = self.email_sender,
                    host = self.email_machine,
                    user = self.email_machine_user,
                    passwd = self.email_machine_password,
                    subject = "{0} - {1}: First run results for {2} with {3}".format(self.cts_version,
                                                                                     self.cts_abi,
                                                                                     self.platform,
                                                                                     self.build_no),
                    objective = "Android CTS {0} - {1} on {2} with {3} {4} image.".format(self.cts_version,
                                                                                          self.cts_abi,
                                                                                          self.platform,
                                                                                          self.build_no,
                                                                                          self.user_build_variant),
                    image = "{0} {1} {2} image".format(self.target,
                                                       self.build_no,
                                                       self.user_build_variant),
                    bios = self.bios,
                    cts_version = self.cts_version,
                    shards_info = "{0}: {1}".format(len(self.serials), self.serials),
                    results = res,
                    failing_test = failing_tests)()
        ############
        # Run failed tests as derived plan
        ############
        print "Check serials for availability after first run"
        for serial in self.serials:
            if not local_utils.has_adb_serial(serial = serial):
                self.serials.remove(serial)
                for device in self.devices:
                    if device["serial"] == serial:
                        self.devices.remove(device)
        print "Serials still available after first run: {0}".format(self.serials)
        results = first_results
        current_loop = 0
        cts_run = first_cts_run
        while current_loop < self.loops_no:
            if runner_type == "plan_based":
                if int(results["fail"]) > 0:
                    print "Creating derived plans for fail tests"
                    self.devices = reboot_devices(devices = self.devices,
                                                  blocking = True)()
                    self.serials = [device["serial"] for device in self.devices]
                    if len(self.serials) == 0:
                        send_report(debug = True,
                                    recipients = self.recipients,
                                    critical = False,
                                    email_sender = self.email_sender,
                                    host = self.email_machine,
                                    user = self.email_machine_user,
                                    passwd = self.email_machine_password,
                                    subject = "{0} - {1}: Error runnig CTS for {2} with {3}".format(self.cts_version,
                                                                                                    self.cts_abi,
                                                                                                    self.platform,
                                                                                                    self.build_no),
                                    objective = "There are no devices available for running derived plan on {0} loop".format(current_loop + 1))()
                        import sys
                        sys.exit(1)
                    last_cts_run = cts_run
                    last_cts_run["loop_no"] = current_loop
                    results, cts_run, failing_tests = cts_derived_plan_run(devices = self.devices,
                                                            serials = self.serials,
                                                            screen_name = self.screen_name,
                                                            screen_log = screen_log,
                                                            cts_base_path = self.cts_base_path,
                                                            cts_plans_dir = self.cts_plans_dir,
                                                            list_results_command = self.list_results_command,
                                                            last_cts_run = last_cts_run,
                                                            runner_type = runner_type,
                                                            return_failing_tests = self.cts_mail_report_failing_tests
                                                            )()
                    res.append(results)
                    report_failing_tests = (current_loop == self.loops_no-1) and self.cts_mail_report_failing_tests
                    print "{0} - {1}: Loop {2} results for {3} with {4}".format(self.cts_version,
                                                                                self.cts_abi,
                                                                                current_loop + 1,
                                                                                self.platform,
                                                                                self.build_no)
                    print "Android CTS {0} - {1} on {2} with {3} {4} image.".format(self.cts_version,
                                                                                    self.cts_abi,
                                                                                    self.platform,
                                                                                    self.build_no,
                                                                                    self.user_build_variant)
                    send_report(cts_mail_report_path = self.cts_mail_report_path,
                                recipients = self.recipients,
                                result_status = " loop {0}".format(current_loop + 1),
                                build_no = self.build_no,
                                critical = False,
                                email_sender = self.email_sender,
                                host = self.email_machine,
                                user = self.email_machine_user,
                                passwd = self.email_machine_password,
                                subject = "{0} - {1}: Loop {2} results for {3} with {4}".format(self.cts_version,
                                                                                                self.cts_abi,
                                                                                                current_loop + 1,
                                                                                                self.platform,
                                                                                                self.build_no),
                                objective = "Android CTS {0} - {1} on {2} with {3} {4} image.".format(self.cts_version,
                                                                                                      self.cts_abi,
                                                                                                      self.platform,
                                                                                                      self.build_no,
                                                                                                      self.user_build_variant),
                                image = "{0} {1} {2} image".format(self.target, self.build_no, self.user_build_variant),
                                bios = self.bios,
                                cts_version = self.cts_version,
                                shards_info = "{0}: {1}".format(len(self.serials), self.serials),
                                results = res,
                                report_failing_tests = report_failing_tests,
                                failing_tests = failing_tests)()
                    print "Done loop {0}: {1}".format(current_loop + 1, results)
                    print "Check serials for availability after loop {0} run".format(current_loop + 1)
                    for serial in self.serials:
                        if not local_utils.has_adb_serial(serial = serial):
                            self.serials.remove(s)
                            for device in self.devices:
                                if device["serial"] == serial:
                                    self.devices.remove(device)
                    print "Serials still available after loop {0} run: {1}".format(current_loop + 1, self.serials)
                current_loop += 1

            elif runner_type == "module_based":
                if (int(results["fail"]) > 0) or (int(results["notExecuted"]) > 0):
                    print "Performing a retry on the previously run session"
                    self.devices = reboot_devices(devices = self.devices,
                                                   blocking = True)()
                    self.serials = [device["serial"] for device in self.devices]
                    if len(self.serials) == 0:
                        send_report(debug = True,
                                    recipients = self.recipients,
                                    critical = False,
                                    email_sender = self.email_sender,
                                    host = self.email_machine,
                                    user = self.email_machine_user,
                                    passwd = self.email_machine_password,
                                    subject = "{0} - {1}: Error runnig CTS for {2} with {3}".format(self.cts_version,
                                                                                                    self.cts_abi,
                                                                                                    self.platform,
                                                                                                    self.build_no),
                                    objective = "There are no devices available for performing the retry on {0} loop".format(current_loop + 1))()
                        import sys
                        sys.exit(1)

                    last_cts_run = cts_run
                    last_cts_run["loop_no"] = current_loop
                    ###########
                    ###########
                    #continue to develop cts_retry_session to do the retry in the same manner that the derived plan branch did it
                    results, cts_run, failing_tests = cts_retry_session(devices = self.devices,
                                                            serials = self.serials,
                                                            screen_name = self.screen_name,
                                                            screen_log = screen_log,
                                                            cts_base_path = self.cts_base_path,
                                                            cts_plans_dir = self.cts_plans_dir,
                                                            list_results_command = self.list_results_command,
                                                            last_cts_run = last_cts_run,
                                                            runner_type = runner_type,
                                                            return_failing_tests = self.cts_mail_report_failing_tests
                                                            )()

                current_loop += 1

        local_steps.screen_command(screen_name = self.screen_name,
                                   screen_command = self.exit_command,
                                   screen_log = screen_log)()


class prepare_devices_for_flash(base_step):

    """ description:
            prepares the device for flashing (enable OEM unlock)

        usage:
            cts_steps.prepare_devices_for_flash(serials)()

        tags:
            ui, android, cts, before, flash
    """

    def __init__(self, devices, platform, intent = True, parallel = True,**kwargs):
        self.devices = devices
        self.platform = platform
        self.intent = intent
        self.parallel = parallel
        base_step.__init__(self, **kwargs)

    def do(self):
        if self.parallel:
            pool = base_utils.LoggingPool(processes = len(self.devices))
            flash_processes = []
            manager = Manager()
            self.step_data = manager.dict()

        for device in self.devices:
            if self.parallel:
                new_process = pool.apply_async(device_oem_unlock,
                                               [device,
                                                self.intent, self.step_data])
                flash_processes.append(new_process)
            else:
                device_oem_unlock(device, self.intent)

        if self.parallel:
            pool.close()
            pool.join()
            for device in self.devices:
                if self.step_data[device["serial"]] is False:
                    self.devices.remove(device)

    def check_condition(self):
        return True


class after_flash_operations(ui_step):

    """ description:
            prepares the device for CTS run

        usage:
            cts_steps.after_flash_operations()()

        tags:
            ui, android, cts, prepare
    """
    def __init__(self,
                 steps_to_perform,
                 device,
                 platform,
                 recipients,
                 email_sender,
                 email_machine,
                 email_machine_user,
                 email_machine_password,
                 cts_media_path = "",
                 cts_base_path = "",
                 cts_tcs_dir = "",
                 cts_admin_apk = "",
                 copy_media_files = True,
                 hidden_ap = False,
                 ap_name = "Android Core QA",
                 ap_pass = "AndroidQA",
                 ap_encryption = "WPA2",
                 ap_802_1x_EAP_method = None,
                 ap_802_1x_phase_2_auth = None,
                 ap_802_1x_user_certificate = None,
                 ap_802_1x_identity = None,
                 ap_802_1x_anonymous_identity = None,
                 intent = True,
                 browser_init = True,
                 **kwargs):
        self.device = device
        self.copy_media_files = copy_media_files
        self.serial = device["serial"]
        self.intent = intent
        self.cts_media_path = cts_media_path
        self.cts_base_path = cts_base_path
        self.cts_tcs_dir = cts_tcs_dir
        self.cts_admin_apk = cts_admin_apk
        self.platform = platform
        self.hidden_ap = hidden_ap
        self.ap_name = ap_name
        self.ap_pass = ap_pass
        self.ap_encryption = ap_encryption
        self.ap_802_1x_EAP_method = ap_802_1x_EAP_method
        self.ap_802_1x_phase_2_auth = ap_802_1x_phase_2_auth
        self.ap_802_1x_user_certificate = ap_802_1x_user_certificate
        self.ap_802_1x_identity = ap_802_1x_identity
        self.ap_802_1x_anonymous_identity = ap_802_1x_anonymous_identity
        self.browser_init = browser_init
        self.recipients = recipients
        self.email_sender = email_sender
        self.email_machine = email_machine
        self.email_machine_user = email_machine_user
        self.email_machine_password = email_machine_password
        self.steps_to_perform = steps_to_perform
        ui_step.__init__(self, **kwargs)

    def do(self):
        try:
            self.step_data = True
            if "wait_for_ui" in self.steps_to_perform and self.steps_to_perform["wait_for_ui"]:
                print "[ {0} ]: waiting for UI".format(self.serial)
                adb_steps.wait_for_ui(serial = self.serial,
                                      timeout = 1800)()
        except:
            print "[ {0} ]: UI not ready - {1}".format(self.serial, traceback.format_exc())
            try:
                reboot_device(device = self.device,
                              result = {},
                              disable_uiautomator = False)
            except:
                print "[ {0} ]: Relay reboot error - {1}".format(self.serial, traceback.format_exc())
                self.step_data = False
                return
        if "screen_lock" in self.steps_to_perform and self.steps_to_perform["screen_lock"]:
            ############
            # set screen lock to None
            ############
            print "[ {0} ]: Set screen lock to None".format(self.serial)
            set_screen_lock(serial = self.serial,
                            intent = self.intent)()
            ui_steps.press_home(serial = self.serial)()
            print "[ {0} ]: Screen lock - Done!".format(self.serial)
            self.steps_to_perform["screen_lock"] = False
        if "developer_options" in self.steps_to_perform and self.steps_to_perform["developer_options"]:
            ############
            # set options from Developer options
            ############
            print "[ {0} ]: Set options from Developer options".format(self.serial)
            developer_enabled = False
            if self.intent:
                adb_steps.enable_developer_options(serial = self.serial)()
                developer_enabled = True
            developer_options = ["Stay awake", "USB debugging", "Allow mock locations"]
            if adb_utils.get_android_version(serial = self.serial) >= 'M':
                developer_options.remove("Allow mock locations")
            ui_steps.enable_options_from_developer_options(serial = self.serial,
                                                           enabled = developer_enabled,
                                                           developer_options = developer_options)()
            ui_steps.press_home(serial = self.serial)()
            print "[ {0} ]: Developer options - Done!".format(self.serial)
            self.steps_to_perform["developer_options"] = False
        if "language" in self.steps_to_perform and self.steps_to_perform["language"]:
            ############
            # set language to English (United States)
            ############
            print "[ {0} ]: Set language to English (United States)".format(self.serial)

            set_language(serial = self.serial,
                         intent = self.intent)()
            ui_steps.press_home(serial = self.serial)()
            print "[ {0} ]: Language - Done!".format(self.serial)
            self.steps_to_perform["language"] = False
        if "sleep" in self.steps_to_perform and self.steps_to_perform["sleep"]:
            ############
            # set sleep to 30 minutes
            ############
            print "[ {0} ]: Set sleep to 30 minutes".format(self.serial)
            set_display(serial = self.serial,
                        intent = self.intent)()
            ui_steps.press_home(serial = self.serial)()
            print "[ {0} ]: Sleep - Done!".format(self.serial)
            self.steps_to_perform["sleep"] = False
        if "location" in self.steps_to_perform and self.steps_to_perform["location"]:
            ############
            # set location On
            ############
            print "[ {0} ]: Set location On".format(self.serial)
            set_location(serial = self.serial,
                         intent = self.intent)()
            ui_steps.press_home(serial = self.serial)()
            print "[ {0} ]: Location -- Done!".format(self.serial)
            self.steps_to_perform["location"] = False
        if "wifi" in self.steps_to_perform and self.steps_to_perform["wifi"]:
            ############
            # connect to WiFi
            ############
            try:
                print "[ {0} ]: Connect to WiFi".format(self.serial)
                #wifi_steps.add_network(serial = self.serial,
                wifi_generic_steps.add_network(serial = self.serial,
                                       ssid = self.ap_name,
                                       security = self.ap_encryption,
                                       password = self.ap_pass,
                                       EAP_method = self.ap_802_1x_EAP_method,
                                       phase_2_auth = self.ap_802_1x_phase_2_auth,
                                       user_certificate = self.ap_802_1x_user_certificate,
                                       identity = self.ap_802_1x_identity,
                                       anonymous_identity = self.ap_802_1x_anonymous_identity,
                                       valid_config=True,
                                       apply_config=True)()
                #wifi_steps.wait_until_connected(serial = self.serial,
                wifi_generic_steps.wait_until_connected(serial = self.serial,
                                                timeout = 45)()
                #wifi_steps.check_connection_info(serial = self.serial,
                wifi_generic_steps.check_connection_info(serial = self.serial,
                                                 SSID = self.ap_name,
                                                 state='CONNECTED/CONNECTED')()
                ui_steps.press_home(serial = self.serial)()
                print "[ {0} ]: WiFi - Done!".format(self.serial)
                self.steps_to_perform["wifi"] = False
            except:
                print "[ {0} ]: Error connecting to WiFi - Try connecting manually: {1}".format(self.serial, traceback.format_exc())
        if "browser" in self.steps_to_perform and self.browser_init and self.steps_to_perform["browser"]:
            ##########
            # perform browser init
            ############
            print "[ {0} ]: Perform browser init".format(self.serial)
            browser_steps.open_chrome_first_time(serial = self.serial,
                                                 intent = self.intent)()
            ui_steps.press_home(serial = self.serial)()
            print "[ {0} ]: Browser init - Done!".format(self.serial)
            self.steps_to_perform["browser"] = False
        if "device_admins" in self.steps_to_perform and self.steps_to_perform["device_admins"]:
            ############
            # set CTS Device Admins
            ############
            if adb_utils.get_android_version(serial = self.serial) < 'N':
                print "[ {0} ]: Set CTS Device Admins".format(self.serial)
                developer_enabled = False
                if self.intent:
                    adb_steps.enable_developer_options(serial = self.serial)()
                    developer_enabled = True
                ui_steps.disable_options_from_developer_options(serial = self.serial,
                                                                enabled = developer_enabled,
                                                                developer_options =
                                                                ["Verify apps over USB"])()
                adb_steps.install_apk(serial = self.serial,
                                      apk_path = os.path.join(self.cts_base_path,
                                      self.cts_tcs_dir,
                                      self.cts_admin_apk))()
                ui_steps.press_home(serial = self.serial)()
                enable_cts_device_admins(serial = self.serial,
                                         intent = self.intent)()
                ui_steps.press_home(serial = self.serial)()
                print "[ {0} ]: CTS Device Admins - Done!".format(self.serial)
                self.steps_to_perform["device_admins"] = False
        if "storage" in self.steps_to_perform  and self.steps_to_perform["storage"]:
            ############
            # set storage to to internal
            ############
            try:
                print "[ {0} ]: Set SD card storage to internal".format(self.serial)
                set_storage(serial = self.serial,
                            intent = self.intent)()
                ui_steps.press_home(serial = self.serial)()
                print "[ {0} ]: SD card - Done!".format(self.serial)
                self.steps_to_perform["storage"] = False
            except:
                print "[ {0} ]: Error performing SD Card wizard: {1}".format(self.serial, traceback.format_exc())
        if "media_files" in self.steps_to_perform and self.copy_media_files and self.steps_to_perform["media_files"]:
            ############
            # copy CTS media files
            ############
            print "[ {0} ]: Copy CTS media files".format(self.serial)
            current_dir = os.getcwd()
            local_steps.change_dir(new_folder = self.cts_media_path)()
            local_steps.command(command = "bash copy_media.sh all -s {0}".format(self.serial),
                                timeout = 60000)()
            local_steps.change_dir(new_folder = current_dir)()
            copy_ok = adb_steps.check_folders_exist(serial = self.serial,
                          folder_list = ["/mnt/sdcard/test",
                                         "/mnt/sdcard/test/bbb_short",
                                         "/mnt/sdcard/test/bbb_full",],
                          critical = False,
                          blocking = False)()
            if not copy_ok:
                send_report(debug = True,
                            recipients = self.recipients,
                            critical = False,
                            email_sender = self.email_sender,
                            host = self.email_machine,
                            user = self.email_machine_user,
                            passwd = self.email_machine_password,
                            subject = "Error copying media files for {0}".format(self.platform),
                            objective = "Serial: {0}".format(self.serial))()
            print "[ {0} ]: CTS media files - Done!".format(self.serial)
            self.steps_to_perform["media_files"] = False

    def check_condition(self):
        return self.step_data


class reboot_devices(base_step):

    """ description:
            reboots the devices described by <devices> list using relays
            if no relays use adb

        usage:
            cts_steps.reboot_devices(devices = list_of_devices)()

        tags:
            adb, android, reboot, devices, relay
    """
    def __init__(self, devices, parallel = True, **kwargs):
        self.devices = devices
        self.parallel = parallel
        base_step.__init__(self, **kwargs)

    def do(self):

        if self.parallel:
            pool = base_utils.LoggingPool(processes = len(self.devices))
            flash_processes = []
            manager = Manager()
            self.step_data = manager.dict()
        else:
            self.step_data = {}
        for device in self.devices:
            if self.parallel:
                new_process = pool.apply_async(reboot_device,
                                               [device,
                                                self.step_data])
                flash_processes.append(new_process)
            else:
                reboot_device(device,
                              self.step_data)
        if self.parallel:
            pool.close()
            pool.join()

        time.sleep(5)
        for device in self.devices:
            serial = device["serial"]
            if self.step_data[serial] is False:
                print "[ {0} ]: Removed, could not be rebooted!!!".format(serial)
                self.devices.remove(device)


    def check_condition(self):
        self.step_data = self.devices
        return True


class flash_devices_for_cts(base_step):

    """ description:
            calls flash (PFT) for every device in <serials>. Flash is
            performed calling a thread for each device

        usage:
            cts_steps.flash_devices_for_cts(serials = list_of_serials,
                                            flash_xml_path = path_to_flash_xml,
                                            build_no = build_no_to_check_flash)()

        tags:
            android, cts, flash, PFT, build, threads
    """
    def __init__(self,
                 devices,
                 flash_xml_path,
                 build_no,
                 timeout,
                 usb_debugging = True,
                 user_build = True,
                 version = 'L',
                 platform = 'ECS-Trekstor',
                 update = False,
                 user_signed = False,
                 parallel = None,
                 fastboot = None,
                 **kwargs):
        self.devices = devices
        self.flash_xml_path = flash_xml_path
        self.build_no = build_no
        self.user_build = user_build
        self.usb_debugging = usb_debugging
        self.version = version
        self.platform = platform
        self.timeout = timeout
        self.user_signed = user_signed
        self.update = update
        if parallel is None:
            self.parallel = statics.Device(serial = self.devices[0]["serial"],
                                           platform = self.platform,
                                           dessert = self.version).parallel_flash
        else:
            self.parallel = parallel
        print "Flashing will be done parallel: {0}".format(self.parallel)
        if fastboot is None:
            self.fastboot = statics.Device(serial = self.devices[0]["serial"],
                                           platform = self.platform,
                                           dessert = self.version).fastboot_to_flash
        else:
            self.fastboot = fastboot
        base_step.__init__(self, **kwargs)

    def do(self):
        if self.parallel:
            pool = base_utils.LoggingPool(processes = len(self.devices))
            flash_processes = []
            manager = Manager()
            self.step_data = manager.dict()
        else:
            self.step_data = {}
        for d in self.devices:
            if self.parallel:
                new_process = pool.apply_async(flash_device,
                                               [d,
                                                self.flash_xml_path,
                                                self.build_no,
                                                self.user_build,
                                                self.user_signed,
                                                self.usb_debugging,
                                                self.platform,
                                                self.version,
                                                self.timeout,
                                                self.update,
                                                self.fastboot,
                                                self.step_data])
                flash_processes.append(new_process)
            else:
                flash_device(d,
                        self.flash_xml_path,
                        self.build_no,
                        self.user_build,
                        self.user_signed,
                        self.usb_debugging,
                        self.platform,
                        self.version,
                        self.timeout,
                        self.update,
                        self.fastboot,
                        self.step_data)
        if self.parallel:
            pool.close()
            pool.join()

    def check_condition(self):
        self.set_passm("Image flashed on: {0}".format(self.devices))

        ret = True
        message = ""
        for device in self.devices:
            if self.step_data[device["serial"]] == False:
                message = "{0}{1} ".format(message, device["serial"])
                self.devices.remove(device)
                ret = False
        self.set_errorm("", "Image on {0} could not be flashed!!".format(message))
        return ret


class prepare_devices_for_aft(base_step):

    """ description:
            calls prepare_aft for every device in <serials>. AFT details
            should be present.
            Each device has a thread for performing these actions.

        usage:
            cts_steps.prepare_devices_for_aft(devices = devices,)()

        tags:
            android, cts, prepare, threads
    """
    def __init__(self,
                 devices,
                 recipients,
                 email_sender,
                 email_machine,
                 email_machine_user,
                 email_machine_password,
                 platform,
                 intent = True,
                 parallel = True,
                 browser_init = True,
                 **kwargs):
        self.devices = devices
        self.serials = [device["serial"] for device in devices]
        self.email_sender = email_sender
        self.email_machine = email_sender
        self.email_machine_user = email_machine_user
        self.email_machine_password = email_machine_password
        self.platform = platform
        self.intent = intent
        self.parallel = parallel
        self.browser_init = browser_init
        self.recipients = recipients
        base_step.__init__(self, **kwargs)


    def do(self):
        if self.parallel:
            pool = base_utils.LoggingPool(processes = len(self.serials))
            manager = Manager()
            self.step_data = manager.dict()
            after_flash_processes = []
        else:
            self.step_data = {}
        for device in self.devices:
            if self.parallel:
                new_process = pool.apply_async(prepare_aft,
                                               [device,
                                                self.recipients,
                                                self.email_sender,
                                                self.email_machine,
                                                self.email_machine_user,
                                                self.email_machine_password,
                                                self.platform,
                                                self.intent,
                                                self.browser_init,
                                                self.step_data
                                                ])
                after_flash_processes.append(new_process)
            else:
                prepare_aft(device,
                            self.recipients,
                            self.email_sender,
                            self.email_machine,
                            self.email_machine_user,
                            self.email_machine_password,
                            self.platform,
                            self.dessert,
                            self.intent,
                            self.browser_init,
                            self.step_data)
        if self.parallel:
            pool.close()
            pool.join()

    def check_condition(self):
        self.set_passm("Prerequisits performed on: {0}".format(self.devices))
        ret = True
        message = ""
        for serial in [device["serial"] for device in self.devices]:
            if self.step_data[serial] == False:
                message = "{0}{1} ".format(message, serial)
                ret = False
        self.set_errorm("", "Prerequisits on {0} were not perform correctly!!".format(message))
        return ret


class prepare_devices_for_cts(base_step):

    """ description:
            calls prepare_cts for every device in <serials>. CTS details
            should be present. Details for WiFi connection should be
            provided: <hidden_ap>, <ap_name>, <ap_pass>.
            Each device has a thread for performing these actions.

        usage:
            cts_steps.prepare_devices_for_cts(serials = serials,
                                              cts_media_path = cts_media_path,
                                              cts_base_path = cts_base_path,
                                              cts_tcs_dir = cts_tcs_dir,
                                              cts_admin_apk = cts_admin_apk,
                                              hidden_ap = False,
                                              ap_name = "Android Core QA",
                                              ap_pass = "AndroidQA",
                                              ap_encryption = "WPA2")()

        tags:
            android, cts, prepare, threads
    """
    def __init__(self,
                 devices,
                 cts_media_path,
                 cts_base_path,
                 cts_tcs_dir,
                 cts_admin_apk,
                 recipients,
                 email_sender,
                 email_machine,
                 email_machine_user,
                 email_machine_password,
                 platform,
                 dessert,
                 hidden_ap = False,
                 ap_name = "Android Core QA",
                 ap_pass = "AndroidQA",
                 ap_encryption = "WPA2",
                 ap_802_1x_EAP_method = None,
                 ap_802_1x_phase_2_auth = None,
                 ap_802_1x_user_certificate = None,
                 ap_802_1x_identity = None,
                 ap_802_1x_anonymous_identity = None,
                 intent = True,
                 parallel = True,
                 browser_init = True,
                 copy_media_files = True,
                 **kwargs):
        self.devices = devices
        self.serials = [device["serial"] for device in devices]
        self.cts_media_path = cts_media_path
        self.cts_base_path = cts_base_path
        self.cts_tcs_dir = cts_tcs_dir
        self.cts_admin_apk = cts_admin_apk
        self.copy_media_files = copy_media_files
        self.platform = platform
        self.dessert = dessert
        self.recipients = recipients
        self.email_sender = email_sender
        self.email_machine = email_sender
        self.email_machine_user = email_machine_user
        self.email_machine_password = email_machine_password
        self.hidden_ap = hidden_ap
        self.ap_name = ap_name
        self.ap_pass = ap_pass
        self.ap_encryption = ap_encryption
        self.ap_802_1x_EAP_method = ap_802_1x_EAP_method
        self.ap_802_1x_phase_2_auth = ap_802_1x_phase_2_auth
        self.ap_802_1x_user_certificate = ap_802_1x_user_certificate
        self.ap_802_1x_identity = ap_802_1x_identity
        self.ap_802_1x_anonymous_identity = ap_802_1x_anonymous_identity
        self.intent = intent
        self.parallel = parallel
        self.browser_init = browser_init
        self.recipients = recipients
        base_step.__init__(self, **kwargs)


    def do(self):
        if self.parallel:
            pool = base_utils.LoggingPool(processes = len(self.serials))
            manager = Manager()
            self.step_data = manager.dict()
            after_flash_processes = []
        else:
            self.step_data = {}
        for device in self.devices:
            if self.parallel:
                new_process = pool.apply_async(prepare_cts,
                                               [device,
                                                self.cts_media_path,
                                                self.cts_base_path,
                                                self.cts_tcs_dir,
                                                self.cts_admin_apk,
                                                self.copy_media_files,
                                                self.recipients,
                                                self.email_sender,
                                                self.email_machine,
                                                self.email_machine_user,
                                                self.email_machine_password,
                                                self.platform,
                                                self.dessert,
                                                self.hidden_ap,
                                                self.ap_name,
                                                self.ap_pass,
                                                self.ap_encryption,
                                                self.ap_802_1x_EAP_method,
                                                self.ap_802_1x_phase_2_auth,
                                                self.ap_802_1x_user_certificate,
                                                self.ap_802_1x_identity,
                                                self.ap_802_1x_anonymous_identity,
                                                self.intent,
                                                self.browser_init,
                                                self.step_data
                                                ])
                after_flash_processes.append(new_process)
            else:
                prepare_cts(device,
                            self.cts_media_path,
                            self.cts_base_path,
                            self.cts_tcs_dir,
                            self.cts_admin_apk,
                            self.copy_media_files,
                            self.recipients,
                            self.email_sender,
                            self.email_machine,
                            self.email_machine_user,
                            self.email_machine_password,
                            self.platform,
                            self.dessert,
                            self.hidden_ap,
                            self.ap_name,
                            self.ap_pass,
                            self.ap_encryption,
                            self.ap_802_1x_EAP_method,
                            self.ap_802_1x_phase_2_auth,
                            self.ap_802_1x_user_certificate,
                            self.ap_802_1x_identity,
                            self.ap_802_1x_anonymous_identity,
                            self.intent,
                            self.browser_init,
                            self.step_data
                            )
        if self.parallel:
            pool.close()
            pool.join()

    def check_condition(self):
        self.set_passm("Prerequisits performed on: {0}".format(self.devices))
        ret = True
        message = ""
        for serial in [device["serial"] for device in self.devices]:
            if self.step_data[serial] == False:
                message = "{0}{1} ".format(message, serial)
                ret = False
        self.set_errorm("", "Prerequisits on {0} were not perform correctly!!".format(message))
        return ret


class send_report(local_step):

    """ description:
            sends cts report to <recipients> list. The report is
            described by <cts_mail_report_template> html files. The
            results are inserted inside the template

        usage:
            send_report(recipients = self.debug_recipients,
                    result_status = self.result_status,
                    build_no = self.build_no,
                    objective = "Android CTS " + self.cts_version +\
                                " on " + self.platform + " with " +\
                                self.image,
                    image = self.image,
                    bios = self.bios,
                    cts_version = self.cts_version,
                    shards_info = len(self.devices),
                    results = res)()

        tags:
            android, cts, report, email
    """
    def __init__(self,
                 recipients,
                 result_status = None,
                 build_no = None,
                 subject = None,
                 objective = None,
                 image = None,
                 bios = None,
                 cts_version = None,
                 shards_info = None,
                 results = None,
                 cts_mail_report_path = "/home/alex/localgit/gitSP/testlib/results/",
                 cts_mail_report_template = "cts_mail_report_template.html",
                 email_sender = "RELAY",
                 email_relay = "linux.intel.com",
                 email_from = "compliance@compass.rb.intel.com",
                 host = "10.237.112.149",
                 user = "regression",
                 passwd = "regressioncts",
                 remote_path = "/opt/regression/mail",
                 debug = False,
                 report_failing_tests = False,
                 failing_tests = None,
                 **kwargs):
        self.set_passm("Report sent")
        self.set_errorm("", "Error! Report was not senf")
        self.recipients = recipients
        self.result_status = result_status
        self.build_no = build_no
        self.subject = subject
        self.objective = objective
        self.image = image
        self.bios = bios
        self.cts_version = cts_version
        self.shards_info = shards_info
        self.results = results
        self.cts_mail_report_path = cts_mail_report_path
        self.cts_mail_report_template = cts_mail_report_template
        self.email_sender = email_sender
        self.critical = False
        self.blocking = False
        self.report_failing_tests = report_failing_tests
        self.failing_tests = failing_tests
        if self.email_sender == "SSH":
            self.host = host
            self.user = user
            self.passwd = passwd
            self.remote_path = remote_path
        elif self.email_sender == "RELAY":
            self.email_relay = email_relay
            self.email_from = email_from

        self.debug = debug
        local_step.__init__(self, **kwargs)

    def _replace_platform_names(self, text):

        for key, val in PLATFORM_DICT.iteritems():
            text = text.replace(key, "{0} - {1}".format(val, key))
        return text

    def do(self):
        self.step_data = True
        if self.debug:
            for to in self.recipients:
                if self.email_sender == "SSH":
                    try:
                        command = "echo {0} | mailx -s '{1}' ".format(self.objective, self.subject)
                        local_steps.ssh_command(command = "{0} {1}".format(command, to),
                                                ssh_host = self.host,
                                                ssh_user = self.user,
                                                ssh_pass = self.passwd)()
                    except Exception as e:
                        print "Could not send email to {0} via SSH - {1}".format(to, e.message)
                elif self.email_sender == "RELAY":
                    try:
                        mail_steps.send_mail(sender = self.email_from,
                                             recipient = to,
                                             subject=self.subject,
                                             body_text = self.objective,
                                             body_html = None,
                                             smtp_server = self.email_relay)()
                    except Exception as e:
                        print "Could not send email to {0} via Relay - {1}".format(to, e.message)
                else:
                    self.set_errorm("", "Report could not be send becuause e-mail sender details are not correct.".format(report_path))
                    self.step_data = False
        else:
            if "RESOURCES_FOLDER" in os.environ:
                self.cts_mail_report_path = os.path.join(os.environ["RESOURCES_FOLDER"], "results")
            else:
                self.cts_mail_report_path = os.path.join(os.path.realpath(__file__).split("/scripts")[0], "results")
            now = base_utils.get_time_string()
            report_name = "{0}.{1}.html".format(self.cts_mail_report_template.split(".")[0], now)
            report_path = os.path.join(self.cts_mail_report_path, report_name)
            self.objective = self._replace_platform_names(self.objective)

            local_steps.copy_file(file = os.path.join(self.cts_mail_report_path, self.cts_mail_report_template),
                                  destination = report_path,
                                  with_rename = True,
                                  critical = False)()
            if not os.path.isfile(report_path):
                self.set_errorm("", "Report file could not be found at {0}.".format(report_path))
                self.step_data = False
                return
            file_utils.replace_string(old = "$result_status",
                                      new = self.result_status,
                                      file_to_search = report_path)
            file_utils.replace_string(old = "$build_no",
                                      new = self.build_no,
                                      file_to_search = report_path)
            file_utils.replace_string(old = "$objective",
                                      new = self.objective,
                                      file_to_search = report_path)
            file_utils.replace_string(old = "$image",
                                      new = self.image,
                                      file_to_search = report_path)
            file_utils.replace_string(old = "$bios_id",
                                      new = self.bios,
                                      file_to_search = report_path)
            file_utils.replace_string(old = "$cts_version",
                                      new = self.cts_version,
                                      file_to_search = report_path)
            file_utils.replace_string(old = "$shards_info",
                                      new = self.shards_info,
                                      file_to_search = report_path)
            results_table = ""
            for result in self.results:
                results_table += "<tr>"
                results_table += "<td>{0}</td>".format(result["ww"])
                results_table += "<td>{0}</td>".format(self.build_no)
                results_table += "<td>{0}</td>".format(result["session"])
                results_table += "<td>{0}</td>".format(result["tests_no"])
                results_table += "<td>{0}</td>".format(result["pass"])
                results_table += "<td><font color = 'red'>{0}</font></td>".format(result["fail"])
                results_table += "<td>{0}</td>".format(result["notExecuted"])
                results_table += "<td>{0}</td>".format(result["start_time"])
                results_table += "<td>{0}</td>".format(result["end_time"])
                results_table += "<td>{0}</td>".format(result["duration"])
                results_table += "</tr>"
            file_utils.replace_string(old = "$results_table",
                                      new = results_table,
                                      file_to_search = report_path)
            failing_table = ""
            if self.report_failing_tests and int(result["fail"]) > 0:
                failing_table += "<p><b>Failing tests</b></p>"
                failing_table += "<table border = 1>"
                failing_table += "<tr>"
                failing_table += "<th bgcolor='green'>Package</th>"
                failing_table += "<th bgcolor='green'>Test</th>"
                failing_table += "</tr>"
                for package, fails in self.failing_tests.iteritems():
                    for fail in fails:
                        failing_table += "<tr>"
                        failing_table += "<td>{0}</td>".format(package)
                        failing_table += "<td><font color = 'red'>{0}</font></td>".format(fail['test'])
                        failing_table += "</tr>"
                failing_table += "</table>"
            file_utils.replace_string(old = "$failed_tests",
                                      new = str(failing_table),
                                      file_to_search = report_path)
            if self.email_sender == "SSH":
                command = "cat {0} | mailx -a 'Content-Type: text/html;' -s '{1}' ".format(os.path.join(self.remote_path, report_name),
                                                                                       self.subject)
            for to in self.recipients:
                if self.email_sender == "SSH":
                    try:
                        local_steps.scp_command(ssh_host = self.host,
                                                ssh_user = self.user,
                                                ssh_pass = self.passwd,
                                                local = report_path,
                                                remote = self.remote_path)()
                        print command
                        local_steps.ssh_command(command = "{0} {1}".format(command, to),
                                                ssh_host = self.host,
                                                ssh_user = self.user,
                                                ssh_pass = self.passwd)()
                    except Exception as e:
                        print "Could not send email to {0} via SSH - {1}".format(to, e.message)
                elif self.email_sender == "RELAY":
                    try:
                        with open(os.path.join(self.cts_mail_report_path, report_name), 'r') as content_file:
                            html = content_file.read()
                        mail_steps.send_mail(sender = self.email_from,
                                             recipient = to,
                                             subject=self.subject,
                                             body_text = None,
                                             body_html = html,
                                             smtp_server = self.email_relay)()
                    except Exception as e:
                        print "Could not send email to {0} via Relay - {1}".format(to, e.message)
                else:
                    self.step_data = False
    def check_condition(self):
        return self.step_data



def prepare_aft(device,
                recipients,
                email_sender,
                email_machine,
                email_machine_user,
                email_machine_password,
                platform,
                intent,
                browser_init,
                ret_values):

    """ description:
            function called by the prepare aft multithreading pool.
            [Ion] TODO: find a solution to make it member of the
            prepare_devices_for_cts step
    """
    serial = device["serial"]
    iteration = 5
    steps_to_perform = {
        "wait_for_ui": True,
        "screen_lock": True,
        "developer_options": True,
        "language": True,
        "sleep": True,
        "location": True,
        "browser": True
    }
    while iteration > 0:
        print "[ {0} ]: Step to be performed: {1}".format(serial, steps_to_perform)
        try:
            ret_values[serial] = False
            after_flash_operations(device = device,
                                   steps_to_perform = steps_to_perform,
                                   serial = serial,
                                   recipients = recipients,
                                   email_sender = email_sender,
                                   email_machine = email_machine,
                                   email_machine_user = email_machine_user,
                                   email_machine_password = email_machine_password,
                                   platform = platform,
                                   intent = intent,
                                   browser_init = browser_init)()
            ret_values[serial] = True
            return
        except:
            print "[ {0} ]: Prepare error on {1} iteration - {2}".format(serial, iteration, traceback.format_exc())
            if iteration > 1:
                print "[ {0} ]: Rebooting ".format(serial)
                try:
                    adb_steps.reboot(serial = serial,
                                     disable_uiautomator = False)()
                    steps_to_perform["wait_for_ui"] = False
                except:
                    print "[ {0} ]: Error rebooting via adb- {1}".format(serial, traceback.format_exc())
                    try:
                        print "[ {0} ]: Rebooting device using relay".format(serial)
                        relay_reboot_device(device = device,
                                            result = {},
                                            disable_uiautomator = False)
                        if result[serial] is False:
                            print "[ {0} ]: Could not reboot device using relay".format(serial)
                            ret_values[serial] = False
                            return
                        steps_to_perform["wait_for_ui"] = False
                    except:
                        print "[ {0} ]: No relay!!! Device will be removed!!! - {1}".format(serial, traceback.format_exc())
                        ret_values[serial] = False
                        return
            iteration -= 1
    ret_values[serial] = False



def prepare_cts(device,
                cts_media_path,
                cts_base_path,
                cts_tcs_dir,
                cts_admin_apk,
                copy_media_files,
                recipients,
                email_sender,
                email_machine,
                email_machine_user,
                email_machine_password,
                platform,
                dessert,
                hidden_ap,
                ap_name,
                ap_pass,
                ap_encryption,
                ap_802_1x_EAP_method,
                ap_802_1x_phase_2_auth,
                ap_802_1x_user_certificate,
                ap_802_1x_identity,
                ap_802_1x_anonymous_identity,
                intent,
                browser_init,
                ret_values):

    """ description:
            function called by the prepare cts multithreading pool.
            [Ion] TODO: find a solution to make it member of the
            prepare_devices_for_cts step
    """
    serial = device["serial"]
    iteration = 5
    steps_to_perform = {
        "wait_for_ui": True,
        "screen_lock": True,
        "developer_options": True,
        "language": True,
        "sleep": True,
        "location": True,
        "wifi": True,
        "browser": True,
        "device_admins": True,
        "media_files": True,
    }
    steps_to_perform["storage"] = (statics.Device(serial = serial,
                                           platform = platform,
                                           dessert = dessert) == "M")
    while iteration > 0:
        print "[ {0} ]: Step to be performed: {1}".format(serial, steps_to_perform)
        try:
            ret_values[serial] = False
            after_flash_operations(device = device,
                                   steps_to_perform = steps_to_perform,
                                   cts_media_path = cts_media_path,
                                   cts_base_path = cts_base_path,
                                   cts_tcs_dir = cts_tcs_dir,
                                   cts_admin_apk = cts_admin_apk,
                                   copy_media_files = copy_media_files,
                                   serial = serial,
                                   recipients = recipients,
                                   email_sender = email_sender,
                                   email_machine = email_machine,
                                   email_machine_user = email_machine_user,
                                   email_machine_password = email_machine_password,
                                   platform = platform,
                                   hidden_ap = hidden_ap,
                                   ap_name = ap_name,
                                   ap_pass = ap_pass,
                                   ap_encryption = ap_encryption,
                                   ap_802_1x_EAP_method = ap_802_1x_EAP_method,
                                   ap_802_1x_phase_2_auth = ap_802_1x_phase_2_auth,
                                   ap_802_1x_user_certificate = ap_802_1x_user_certificate,
                                   ap_802_1x_identity = ap_802_1x_identity,
                                   ap_802_1x_anonymous_identity = ap_802_1x_anonymous_identity,
                                   intent = intent,
                                   browser_init = browser_init)()
            ret_values[serial] = True
            return
        except:
            print "[ {0} ]: Prepare error on {1} iteration - {2}".format(serial, iteration, traceback.format_exc())
            if iteration > 1:
                print "[ {0} ]: Rebooting ".format(serial)
                try:
                    adb_steps.reboot(serial = serial,
                                     disable_uiautomator = False)()
                    steps_to_perform["wait_for_ui"] = False
                except:
                    print "[ {0} ]: Error rebooting via adb- {1}".format(serial, traceback.format_exc())
                    try:
                        print "[ {0} ]: Rebooting device using relay".format(serial)
                        relay_reboot_device(device = device,
                                            result = {},
                                            disable_uiautomator = False)
                        if result[serial] is False:
                            print "[ {0} ]: Could not reboot device using relay".format(serial)
                            ret_values[serial] = False
                            return
                        steps_to_perform["wait_for_ui"] = False
                    except:
                        print "[ {0} ]: No relay!!! Device will be removed!!! - {1}".format(serial, traceback.format_exc())
                        ret_values[serial] = False
                        return
            iteration -= 1
    ret_values[serial] = False


def flash_device(device, flash_xml_path, build_no, user_build, user_signed,
                 usb_debugging, platform, version, timeout, update, fastboot, ret_values):

    """ description:
            function called by the flash devices multithreading pool.
            [Ion] TODO: find a solution to make it member of the
            flash_devices_for_cts step
    """
    use_relay = False
    print "[ {0} ]: Flashing".format(device["serial"])
    if fastboot:
        """if local_utils.has_adb_serial(serial = device["serial"]):
            try:
                adb_steps.reboot(command = "fastboot",
                                 serial = device["serial"],
                                 reboot_timeout = 180)()
            except:
                print "[ {0} ]: Did not boot into fastboot - {1}".format(device["serial"], traceback.format_exc())
                use_relay = True
        else:
        """
        use_relay = not local_utils.has_fastboot_serial(serial = device["serial"]) and \
                    not local_utils.has_adb_serial(serial = device["serial"])

        if use_relay:
            try:
                print "[ {0} ]: Use relay to start in fastboot".format(device["serial"])
                # Shut down device
                my_relay = Relayed_device(relay_port = device["relay"]["tty"],
                                          power_port = device["relay"]["power_port"],
                                          v_up_port = device["relay"]["v_up_port"],
                                          v_down_port = device["relay"]["v_down_port"])
                my_relay.power_on()
                my_relay.power_off()
                # Start device in fastboot
                my_relay.enter_fastboot()
                my_relay.close()
                if not local_utils.has_fastboot_serial(serial = device["serial"]):
                    print "[ {0} ]: Could not boot in fastboot using relay".format(device["serial"])
                    ret_values[device["serial"]] = False
                    return
            except:
                # No relay
                print "[ {0} ]: No relay - {1}".format(device["serial"],  traceback.format_exc())
                ret_values[device["serial"]] = False
                return
    try:
        ret_values[device["serial"]] = False
        flash_steps.pft_flash(device = device,
                              user_build = user_build,
                              user_signed = user_signed,
                              serial = device["serial"],
                              flash_xml_path = flash_xml_path,
                              build_no = build_no,
                              usb_debugging = usb_debugging,
                              version = version,
                              platform = platform,
                              timeout = timeout,
                              update = update)()
        print "[ {0} ]: Waiting for UI".format(device["serial"])
        adb_steps.wait_for_ui_processes(serial = device["serial"],
                            timeout = 900)()
        adb_steps.enable_uiautomator_service(serial = device["serial"], timeout = 300)()
        ret_values[device["serial"]] = True
        return
    except:
        print  "[ {0} ]: Flash error - {1}".format(device["serial"], traceback.format_exc())
    ret_values[device["serial"]] = False


def device_oem_unlock(device, intent, results):

    """ description:
            function called by the prepare for flash multithreading pool.
            [Ion] TODO: find a solution to make it member of the
            prepae_devices_for_flash step
    """
    serial = device["serial"]
    try:
        time.sleep(3)
        if not local_utils.has_adb_serial(serial = serial):
            print "[ {0} ]: No adb connectivity!!!".format(serial)
            try:
                print "[ {0} ]: Rebooting device using relay".format(serial)
                relay_reboot_device(device = device,
                                    result = results)
                if results[serial] is False:
                    print "[ {0} ]: Could not reboot. Device will be removed!!!".format(serial)
                    return
            except:
                # NO RELAY
                print "[ {0} ]: No relay!!! Device will be removed!!! - {1}".format(serial, traceback.format_exc())
                results[serial] = False
                return
        print "[ {0} ]: Prepare device for flash.".format(serial)
        ############
        # enable OEM unlock
        ############
        print "[ {0} ]: Enable OEM unlock".format(serial)
        #adb_steps.enable_uiautomator_service(serial = serial,
        #                                     timeout = 120)()
        # wake and unlock if necessary
        print "[ {0} ]: wake device if necessary".format(serial)
        adb_steps.wake_up_device(serial = serial)()
        time.sleep(1)
        print "[ {0} ]: unlock device if necessary".format(serial)
        adb_steps.menu_to_unlock(serial = serial)()
        time.sleep(1)
        # perform starup wizard if necessary
        print "[ {0} ]: perform starup wizard if necessary".format(serial)
        ui_steps.click_button_if_exists(serial = serial,
                                        view_to_find = {"text": "OK"},
                                        wait_time = 5000)()
        if ui_utils.is_view_displayed(serial = serial,
                                      view_to_find = {"resourceId": "com.google.android.setupwizard:id/start"}):
            ui_steps.perform_startup_wizard(serial = serial)()
        # accept telemetry if necessary
        print "[ {0} ]: accept telemetry if necessary".format(serial)
        ui_steps.click_button_if_exists(serial = serial,
                                        view_to_find = {"text": "Allow"},
                                        view_to_check = {"text": "GOT IT"},
                                        wait_time = 5000)()
        # accept welcome if necessary
        print "[ {0} ]: accept welcome if necessary".format(serial)
        ui_steps.click_button_if_exists(serial = serial,
                                        view_to_find = {"text": "GOT IT"},
                                        wait_time = 5000)()
        ui_steps.press_home(serial = serial)()
        developer_enabled = False
        if intent:
            adb_steps.enable_developer_options(serial = serial)()
            developer_enabled = True
        ui_steps.enable_oem_unlock(serial = serial,
                                   enabled = developer_enabled)()
        ui_steps.press_home(serial = serial)()
        print "[ {0} ]: OEM unlock - Done!".format(serial)
        results[device["serial"]] = True

    except:
        print "[ {0} ]: OEM unlock error - {1}".format(serial, traceback.format_exc())
        results[device["serial"]] = False


def relay_reboot_device(device, result, disable_uiautomator = True):

    """ description:
            reboot devices using relay
            can be used in parallel
    """
    serial = device["serial"]
    my_relay = Relayed_device(relay_port = device["relay"]["tty"],
                              power_port = device["relay"]["power_port"],
                              v_up_port = device["relay"]["v_up_port"],
                              v_down_port = device["relay"]["v_down_port"])
    my_relay.relay_reboot()
    result[serial] = True
    try:
        adb_steps.wait_for_ui(serial = serial,
                              disable_uiautomator = disable_uiautomator)()
    except:
        print "[ {0} ]:Relay reboot error - {1}".format(serial, traceback.format_exc())
        # try to reboot again
        try:
            my_relay.relay_reboot()
            adb_steps.wait_for_ui(serial = serial,
                                  disable_uiautomator = disable_uiautomator)()
        except:
            print "[ {0} ]: Relay reboot error - {1}".format(serial, traceback.format_exc())
            result[serial] = False
    finally:
        my_relay.close()


def reboot_device(device, result, disable_uiautomator = True):

    """ description:
            reboot devices using relay or adb
            can be used in parallel
    """
    serial = device["serial"]
    result[serial] = True
    try:
        adb_steps.reboot(serial = serial,
                         disable_uiautomator = disable_uiautomator)()
    except Exception as e:
        print "[ {0} ]: Reboot error - {1}".format(serial, e.message, traceback.format_exc())
        print "[ {0} ]: Trying relay reboot".format(serial)
        try:
            relay_reboot_device(device = device,
                                result = result,
                                disable_uiautomator = disable_uiautomator)
        except:
            print "[ {0} ]: Relay reboot error - {1}".format(serial, traceback.format_exc())
            result[serial] = False
            return
