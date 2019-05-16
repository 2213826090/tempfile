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
from testlib.scripts.android.cts import cts_steps
from testlib.scripts.file import file_utils
from testlib.scripts.connections.local import local_utils

import os
import time
from datetime import timedelta
import traceback
from multiprocessing import *


class set_brightness(ui_step):

    """ description:
            sets brightness to minimum

        usage:
            gts_steps.set_brightness()()

        tags:
            ui, android, gts, brightness
    """
    def __init__(self, intent = True, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.intent = intent
        self.set_passm("Brightness is set to minimum")
        self.set_errorm("", "Brightness is set to minimum")

    def do(self):
        if self.intent:
            adb_steps.am_start_command(serial = self.serial,
                                       component = "com.android.settings/.DisplaySettings")()
        else:
            ui_steps.open_settings_app(serial = self.serial,
                                       view_to_find = {"text": "Display"},
                                       view_to_check = {"text": "Sleep"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "Brightness level"},
                              view_to_check = {"resourceId": "com.android.systemui:id/slider"})()
        top = self.uidevice(resourceId = "com.android.systemui:id/slider").info["bounds"]['top']
        left = self.uidevice(resourceId = "com.android.systemui:id/slider").info["bounds"]['left']
        right = self.uidevice(resourceId = "com.android.systemui:id/slider").info["bounds"]['right']
        bottom = self.uidevice(resourceId = "com.android.systemui:id/slider").info["bounds"]['bottom']
        middle_x = (left + right) / 2
        middle_y = (top + bottom) / 2
        ui_steps.click_xy(serial = self.serial,
                          x = left,
                          y = (top + bottom) / 2)()

    def check_condition(self):
        return True


class run_gts_command(local_step):

    """ description:
            runs a GTS command inside the <screen_name> screen. It checks
            "XML test result file generated" to validate the correct
            execution of the command. "add derivedplan" is also covered
            in this step
            it returns the results of the run

        usage:
            gts_steps.run_gts_command(gts_command = gts_command,
                                      screen_name = self.screen_name,
                                      gts_base_path = self.gts_base_path,
                                      timeout = self.timeout,
                                      list_results_command = self.list_results_command,
                                      gts_plans_dir = self.gts_plans_dir)()

        tags:
            android, gts, run, screen, results
    """
    def __init__(self,
                 gts_command,
                 gts_base_path,
                 screen_name = "irda_gts",
                 screen_log = "screenlog.0",
                 timeout = 288000,
                 list_results_command = "list results",
                 gts_plans_dir = "repository/plans/",
                 **kwargs):
        self.gts_command = gts_command
        self.last_session = suite_utils.get_session(screen_name = screen_name,
                                                         suite = "gts",
                                                         screen_log = screen_log)
        self.screen_name = screen_name
        self.screen_log = screen_log
        self.timeout = timeout
        self.list_results_command = list_results_command
        self.gts_base_path = gts_base_path
        self.gts_plans_dir = gts_plans_dir
        local_step.__init__(self, **kwargs)
        print "Last session before run: {0}".format(self.last_session)
        if "continue-session" in self.gts_command:
            self.set_passm("GTS - continue_session: {0}".format(self.gts_command))
            self.set_errorm("", "GTS - continue_session: {0}".format(self.gts_command))
            self.last_session -= 1
        elif "derivedplan" in self.gts_command:
            self.set_passm("GTS - derivedplan: {0}".format(self.gts_command))
            self.set_errorm("", "GTS - derivedplan: {0}".format(self.gts_command))
        else:
            self.set_passm("GTS - run plan: {0}".format(self.gts_command))
            self.set_errorm("", "GTS - run plan: {0}".format(self.gts_command))

    def do(self):
        if "derivedplan" in self.gts_command:
            local_steps.screen_command(screen_name = self.screen_name,
                                       screen_log = self.screen_log,
                                       screen_command = self.gts_command)()
        elif self.gts_command == "bogus":
            # Debug
            pass
        else:
            local_steps.screen_command(screen_name = self.screen_name,
                                       screen_command = self.gts_command,
                                       with_log = True,
                                       grep_for = "XML test result file generated",
                                       screen_log = self.screen_log,
                                       timeout = self.timeout)()

    def check_condition(self):
        if "derivedplan" in self.gts_command:
            time.sleep(5)
            plan = self.gts_command.split(" ")[3]
            return suite_utils.is_plan_created(plan = plan,
                                             suite_base_path = self.gts_base_path,
                                             suite_plans_dir = self.gts_plans_dir)
        run_timestamp = suite_utils.get_run_timestamp(screen_log = self.screen_log)
        run_results = suite_utils.get_results_by_timestamp(run_timestamp,
                                                           suite = "gts",
                                                           list_results_command = self.list_results_command,
                                                           screen_name = self.screen_name,
                                                           screen_log = self.screen_log)
        self.step_data = run_timestamp, run_results

        if self.step_data:
            return True
        else:
            return False


class gts_first_run(base_step):

    """ description:

        usage:

        tags:
    """

    def __init__(self,
                 devices,
                 gts_plan_name,
                 screen_name,
                 gts_base_path,
                 gts_binary_dir,
                 gts_binary,
                 list_results_command,
                 return_failing_tests = False,
                 **kwargs):
        base_step.__init__(self, **kwargs)
        self.devices = devices
        self.serials = [d["serial"] for d in self.devices]
        self.gts_plan_name = gts_plan_name
        self.screen_name = screen_name
        self.gts_base_path = gts_base_path
        self.gts_binary_dir= gts_binary_dir
        self.gts_binary = gts_binary
        self.list_results_command = list_results_command
        self.return_failing_tests = return_failing_tests

    def do(self):
        first_gts_run = {
            "gts_plan": self.gts_plan_name,
            "gts_class": None,
            "gts_method": None,
            "gts_session_id": None,
            "gts_result_type": None,
            "gts_continue_session": False,
            "gts_derived_plan": False,
            "gts_run_timestamp": None,
            "loop_no": 0
        }
        start_time = base_utils.get_time_string()
        start_counter = time.time()

        screen_log = local_steps.create_screen(screen_name = self.screen_name,
                                               with_log = True)()
        local_steps.screen_command(screen_name = self.screen_name,
                                   screen_command = os.path.join(self.gts_base_path,
                                                    self.gts_binary_dir,
                                                    self.gts_binary),
                                   screen_log = screen_log)()

        gts_command = suite_utils.create_suite_run_command(serials = self.serials,
                                                       suite = "gts",
                                                       suite_run = first_gts_run)
        print gts_command
        if "Unable to create the run command!" in gts_command:
            cts_steps.send_report(debug = True,
                        recipients = self.recipients,
                        critical = False,
                        subject = "Error runnig GTS for {0} with {1} - {2}".format(self.platform,
                                                                                   self.build_no,
                                                                                   self.gts_version),
                        objective = "There are no devices available for when creating the gts run command for the first run")()
            import sys
            sys.exit(1)
        first_gts_run["gts_run_timestamp"], first_results = run_gts_command(gts_command = gts_command,
                                        gts_base_path = self.gts_base_path,
                                        screen_name = self.screen_name,
                                        screen_log = screen_log)()


        ############
        # Continue sessions while notExecuted tests exist
        ############
        gts_run = first_gts_run
        results = first_results
        print "Done first loop: {0} on session {1}".format(results , first_gts_run["gts_run_timestamp"])
        if int(results["notExecuted"])  > 0:
            print "Continue session for not executed tests"
            target_not_executed_tests = 0
            not_executed_tests = 0
            not_executed_loop = 0
            while (int(results["notExecuted"]) != not_executed_tests) and (int(results["notExecuted"]) != target_not_executed_tests):
                self.devices = cts_steps.reboot_devices(devices = self.devices,
                                                    blocking = True)()
                self.serials = [device["serial"] for device in self.devices]
                not_executed_tests = int(results["notExecuted"])
                gts_run = suite_utils.create_continue_session(suite_run = gts_run,
                                                              suite = "gts",
                                                              list_results_command = self.list_results_command,
                                                              screen_name = self.screen_name,
                                                              screen_log = screen_log)
                gts_command = suite_utils.create_suite_run_command(serials = self.serials,
                                                               suite = "gts",
                                                               suite_run = gts_run)
                if "Unable to create the run command!" in gts_command:
                    cts_steps.send_report(debug = True,
                                recipients = self.recipients,
                                critical = False,
                                subject = "Error runnig GTS for {0} with {1} - {2}".format(self.platform,
                                                                                           self.build_no,
                                                                                           self.gts_version),
                                objective = "There are no devices available for when creating the gts run command for a continue session on the first run")()
                    import sys
                    sys.exit(1)
                run_timestamp, results = run_gts_command(gts_command = gts_command,
                                          gts_base_path = self.gts_base_path,
                                          screen_name = self.screen_name,
                                          screen_log = screen_log)()
                not_executed_loop += 1
                print "Results continue session {0}: {1}".format(not_executed_loop, results)
            gts_run = first_gts_run
            gts_run["gts_run_timestamp"] = run_timestamp

        res = []
        result = {}
        result["ww"] = "WW{0}".format(base_utils.get_ww())
        result["session"] = self.gts_plan_name
        result["tests_no"] = int(results["pass"]) + int(results["fail"]) + int(results["notExecuted"])
        result["pass"] = results["pass"]
        result["fail"] = results["fail"]
        result["notExecuted"] = results["notExecuted"]
        result["start_time"] = start_time
        result["end_time"] = base_utils.get_time_string()
        end_counter = time.time()
        result["duration"] = str(timedelta(seconds=int(end_counter-start_counter))).replace(':', '-')
        res.append(result)
        failing_tests = None
        if self.return_failing_tests:
            # result_xml_folder = suite_utils.get_latest_result_folder(list_results_command = self.list_results_command,
            #                                                          suite = "gts",
            #                                                          screen_name = self.screen_name,
            #                                                          screen_log = screen_log)
            result_xml_path = os.path.join(self.gts_base_path,
                                           "repository/results/",
                                           first_gts_run["gts_run_timestamp"],
                                           "xtsTestResult.xml")
            failing_tests = suite_utils.get_fails_from_xml_file(result_xml_path)

        self.step_data = result, first_gts_run, screen_log, failing_tests

    def check_condition(self):
        if self.step_data:
            return True
        return False


class gts_derived_plan_run(base_step):

    """ description

        usage:

        tags:
    """

    def __init__(self,
                 devices,
                 screen_name,
                 screen_log,
                 gts_base_path,
                 gts_plans_dir,
                 list_results_command,
                 last_gts_run,
                 return_failing_tests = False,
                 **kwargs):
        base_step.__init__(self, **kwargs)
        self.devices = devices
        self.serials = [d["serial"] for d in self.devices]
        self.screen_name = screen_name
        self.screen_log = screen_log
        self.gts_base_path = gts_base_path
        self.gts_plans_dir = gts_plans_dir
        self.list_results_command = list_results_command
        self.last_gts_run = last_gts_run
        self.return_failing_tests = return_failing_tests

    def do(self):
        start_time = base_utils.get_time_string()
        start_counter = time.time()
        gts_run = suite_utils.create_derived_plan(suite_run = self.last_gts_run,
                                                  suite = "gts",
                                                  suite_base_path = self.gts_base_path,
                                                  suite_plans_dir = self.gts_plans_dir,
                                                  plan_suffix = self.screen_name,
                                                  list_results_command = self.list_results_command,
                                                  screen_name = self.screen_name,
                                                  screen_log = self.screen_log)
        gts_command = suite_utils.create_suite_run_command(serials = self.serials,
                                                           suite = "gts",
                                                           suite_run = gts_run)

        print "Create derived plan: {0}".format(gts_command)
        if "Unable to create the run command!" in gts_command:
            cts_steps.send_report(debug = True,
                        recipients = self.recipients,
                        critical = False,
                        subject = "Error runnig GTS for {0} with {1} - {2}".format(self.platform,
                                                                                   self.build_no,
                                                                                   self.gts_version),
                        objective = "There are no devices available for when creating the gts run command for the first run")()
            import sys
            sys.exit(1)
        run_gts_command(gts_command = gts_command,
                        gts_base_path = self.gts_base_path,
                        screen_name = self.screen_name,
                        screen_log = self.screen_log)()
        new_plan = gts_run["gts_plan"]

        gts_run = self.last_gts_run
        gts_run["gts_plan"] = new_plan
        gts_command = suite_utils.create_suite_run_command(serials = self.serials,
                                                           suite = "gts",
                                                           suite_run = gts_run)
        print "Running plan: {0}".format(gts_command)
        if "Unable to create the run command!" in gts_command:
            cts_steps.send_report(debug = True,
                        recipients = self.recipients,
                        critical = False,
                        subject = "Error runnig GTS for {0} with {1} - {2}".format(self.platform,
                                                                                   self.build_no,
                                                                                   self.gts_version),
                        objective = "There are no devices available for when creating the gts run command for a derived plan")()
            import sys
            sys.exit(1)
        gts_run["gts_run_timestamp"], results = run_gts_command(gts_command = gts_command,
                                  gts_base_path = self.gts_base_path,
                                  screen_name = self.screen_name,
                                  screen_log = self.screen_log)()
        first_gts_run = gts_run
        if int(results["notExecuted"])  > 0:
            print "Continue session for not executed tests for derived plan {0}".format(gts_run["gts_plan"])
            not_executed_tests = 0
            not_executed_loop = 0
            while int(results["notExecuted"]) != not_executed_tests:
                self.devices = cts_steps.reboot_devices(devices = self.devices,
                                                        blocking = True)()
                self.serials = [device["serial"] for device in self.devices]
                not_executed_tests = int(results["notExecuted"])
                gts_run = suite_utils.create_continue_session(suite_run = gts_run,
                                                              suite = "gts",
                                                              list_results_command = self.list_results_command,
                                                              screen_name = self.screen_name,
                                                              screen_log = self.screen_log)
                gts_command = suite_utils.create_suite_run_command(serials = self.serials,
                                                                   suite = "gts",
                                                                   suite_run = gts_run)
                if "Unable to create the run command!" in gts_command:
                    cts_steps.send_report(debug = True,
                                recipients = self.recipients,
                                critical = False,
                                subject = "Error runnig GTS for {0} with {1} - {2}".format(self.platform,
                                                                                           self.build_no,
                                                                                           self.gts_version),
                                objective = "There are no devices available for when creating the gts run command for a derived plan")()
                    import sys
                    sys.exit(1)
                run_timestamp, results = run_gts_command(gts_command = gts_command,
                                          gts_base_path = self.gts_base_path,
                                          screen_name = self.screen_name,
                                          screen_log = self.screen_log)()
                not_executed_loop += 1
                print "Results continue session {0}: {1}".format(not_executed_loop, results)
            gts_run = first_gts_run
            gts_run["run_timestamp"] = run_timestamp
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
            result_xml_path = os.path.join(self.gts_base_path,
                                           "repository/results/",
                                           gts_run["gts_run_timestamp"],
                                           "xtsTestResult.xml")
            failing_tests = suite_utils.get_fails_from_xml_file(result_xml_path)

        self.step_data = result, gts_run, failing_tests

    def check_condition(self):
        if self.step_data:
            return True
        return False



class run_gts_plan(base_step):

    """ description:
            runs the GTS <gts_plan> plan on the device described by
            <device> serial list and follows the Google procedure:
            - executes first loop
            - calls continue_session until all packages are executed or
            the non executed packages remain the same (max 5 iteration)
            - executes derivedplan loops 2 times
            - sends report via e-mail after each loop to <recipients>
            list
            plan execution is done in the <screen_name> screen

        usage:
            gts_steps.run_gts_plan(gts_plan_name = "GTS",
                       gts_base_path = gts_base_path,
                       gts_binary_dir = gts_binary_dir,
                       gts_binary = gts_binary,
                       device = device_serials,
                       recipients = recipients,
                       debug_recipients = debug_recipients,
                       screen_name = "irda",
                       list_results_command = list_results_command,
                       gts_plans_dir = gts_plans_dir,
                       timeout = 28800,
                       build_no = build_no,
                       image = project_id + build_no + target_image,
                       bios = bios,
                       gts_version = gts_version,
                       platform = platform,
                       gts_mail_report_path = "/home/oane/Work/automation/testlib/results/",
                       gts_mail_report_template = "gts_mail_report_template.html",
                       email_sender = "SSH",
                       host = "10.237.112.149",
                       user = "regression",
                       password = "regressiongts",
                       remote_path = "/opt/regression/mail"
                       )()

        tags:
            android, gts, run, plan, screen, results, email, report
    """
    def __init__ (self,
                  screen_name,
                  devices,
                  gts_plan_name,
                  gts_base_path,
                  gts_binary_dir,
                  gts_tcs_dir,
                  gts_plans_dir,
                  gts_results_dir,
                  gts_logs_dir,
                  gts_binary,
                  gts_version,
                  loops_no,
                  build_no,
                  bios,
                  platform,
                  target,
                  recipients,
                  debug_recipients,
                  gts_mail_report_failing_tests = False,
                  user_build_variant = 'user',
                  list_results_command = "list results",
                  exit_command = "exit",
                  gts_mail_report_template = "gts_mail_report_template.html",
                  email_sender = "SSH",
                  host = "10.237.112.149",
                  user = "regression",
                  password = "regressiongts",
                  remote_path = "/opt/regression/mail",
                  timeout = 28800,
                  **kwargs):
        self.screen_name = screen_name
        self.devices = devices
        self.gts_plan_name = gts_plan_name
        self.gts_base_path = gts_base_path
        self.gts_binary_dir = gts_binary_dir
        self.gts_tcs_dir = gts_tcs_dir
        self.gts_plans_dir = gts_plans_dir
        self.gts_results_dir = gts_results_dir
        self.gts_logs_dir = gts_logs_dir
        self.gts_binary = gts_binary
        self.gts_version = gts_version
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
        self.gts_mail_report_template = gts_mail_report_template
        self.gts_mail_report_failing_tests = gts_mail_report_failing_tests
        self.email_sender = email_sender
        self.host = host
        self.user = user
        self.password = password
        self.remote_path = remote_path
        self.timeout = timeout
        base_step.__init__(self, **kwargs)

    def do(self):
        ############
        # Run GTS plan
        ############
        res = []
        first_results, first_gts_run, screen_log, failing_tests = gts_first_run(devices = self.devices,
                                                     gts_plan_name = self.gts_plan_name,
                                                     screen_name = self.screen_name,
                                                     gts_base_path = self.gts_base_path,
                                                     gts_binary_dir = self.gts_binary_dir,
                                                     gts_binary = self.gts_binary,
                                                     list_results_command = self.list_results_command,
                                                     return_failing_tests = self.gts_mail_report_failing_tests)()
        if first_results == "No devices":
            cts_steps.send_report(debug = True,
                        recipients = self.recipients,
                        critical = False,
                        subject = "Error runnig GTS for {0} with {1} - {2}".format(self.platform,
                                                                                   self.build_no,
                                                                                   self.gts_version),
                        objective = "There are no devices available for running continue-session after first run")()
            import sys
            sys.exit(1)
        res.append(first_results)

        self.serials = [device["serial"] for device in self.devices]
        cts_steps.send_report(subject = "First run results for {0} with {1} - {2}".format(self.platform,
                                                                                self.build_no,
                                                                                self.gts_version),
                                cts_mail_report_template = self.gts_mail_report_template,
                                recipients = self.recipients,
                                result_status = " first run ",
                                build_no = self.build_no,
                                critical = False,
                                objective = "Android GTS {0}  on {1} with {2} {3} image.".format(self.gts_version,
                                                                                                 self.platform,
                                                                                                 self.build_no,
                                                                                                 self.user_build_variant),
                                image = "{0} {1} image".format(self.build_no, self.user_build_variant),
                                bios = self.bios,
                                cts_version = self.gts_version,
                                serials = self.serials,
                                results = res,
                                shards_info = "{0}: {1}".format(len(self.serials), self.serials),
                                failing_test = failing_tests)()
        ############
        # Run failed tests as derived plan
        ############
        print "Creating derived plans for fail tests"
        results = first_results
        current_loop = 0
        gts_run = first_gts_run
        while current_loop < self.loops_no:
            if int(results["fail"]) > 0:
                self.devices = cts_steps.reboot_devices(devices = self.devices,
                                          blocking = True)()
                last_gts_run = gts_run
                last_gts_run["loop_no"] = current_loop
                results, gts_run, failing_tests = gts_derived_plan_run(devices = self.devices,
                                                        screen_name = self.screen_name,
                                                        screen_log = screen_log,
                                                        gts_base_path = self.gts_base_path,
                                                        gts_plans_dir = self.gts_plans_dir,
                                                        list_results_command = self.list_results_command,
                                                        last_gts_run = last_gts_run,
                                                        return_failing_tests = self.gts_mail_report_failing_tests)()
                if results == "No devices":
                    cts_steps.send_report(debug = True,
                                recipients = self.recipients,
                                critical = False,
                                subject = "Error runnig GTS for {0} with {1} - {2}".format(self.platform,
                                                                                           self.build_no,
                                                                                           self.cts_version),
                                objective = "There are no devices available for running continue-session for derived plan on {0} loop".format(current_loop + 1))()
                    import sys
                    sys.exit(1)
                res.append(results)
                cts_steps.send_report(subject = "Loop {0} results for {1} with {2} - {3}".format(current_loop + 1,
                                                                                       self.platform,
                                                                                       self.build_no,
                                                                                       self.gts_version),
                            cts_mail_report_template = self.gts_mail_report_template,
                            recipients = self.recipients,
                            result_status = " loop {0}".format(current_loop + 1),
                            build_no = self.build_no,
                            critical = False,
                            objective = "Android GTS {0} on {1} with {2} {3} image.".format(self.gts_version,
                                                                                            self.platform,
                                                                                            self.build_no,
                                                                                            self.user_build_variant),
                            image = "{0} {1} image".format(self.build_no, self.user_build_variant),
                            bios = self.bios,
                            cts_version = self.gts_version,
                            serials = self.serials,
                            results = res,
                            report_failing_tests = self.gts_mail_report_failing_tests,
                            shards_info = "{0}: {1}".format(len(self.serials), self.serials),
                            failing_tests = failing_tests)()
                print "Done loop {0}:  {1}".format(current_loop + 1, results)
                print "Check serials for availability after {0} run".format(current_loop + 1)
            current_loop += 1

        local_steps.screen_command(screen_name = self.screen_name,
                                   screen_command = self.exit_command)()


class after_flash_operations(ui_step):

    """ description:
            prepares the device for GTS run

        usage:
            gts_steps.after_flash_operations()()

        tags:
            ui, android, gts, prepare
    """
    def __init__(self,
                 device,
                 steps_to_perform,
                 gts_base_path,
                 gts_tcs_dir,
                 platform,
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
                 **kwargs):
        self.device = device
        self.intent = intent
        self.gts_base_path = gts_base_path
        self.gts_tcs_dir = gts_tcs_dir
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
        self.steps_to_perform = steps_to_perform
        ui_step.__init__(self, **kwargs)

    def do(self):
        try:
            self.step_data = True
            if self.steps_to_perform["wait_for_ui"]:
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
        if self.steps_to_perform["screen_lock"]:
            ############
            # set screen lock to None
            ############
            print "[ {0} ]: Set screen lock to None".format(self.serial)
            cts_steps.set_screen_lock(serial = self.serial,
                                      intent = self.intent)()
            ui_steps.press_home(serial = self.serial)()
            print "[ {0} ]: Screen lock - Done!".format(self.serial)
        if self.steps_to_perform["developer_options"]:
            ############
            # set options from Developer options
            ############
            print "[ {0} ]: Set options from Developer options".format(self.serial)
            developer_enabled = False
            if self.intent:
                adb_steps.enable_developer_options(serial = self.serial)()
                developer_enabled = True
            ui_steps.enable_options_from_developer_options(serial = self.serial,
                                                           enabled = developer_enabled,
                                                           developer_options =
                                                           ["Stay awake",
                                                            "USB debugging"])()
            ui_steps.press_home(serial = self.serial)()
            if self.intent:
                adb_steps.enable_developer_options(serial = self.serial)()
            ui_steps.disable_options_from_developer_options(serial = self.serial,
                                                            enabled = developer_enabled,
                                                            developer_options =
                                                            ["Verify apps over USB"])()
            ui_steps.press_home(serial = self.serial)()
            print "[ {0} ]: Developer options - Done!".format(self.serial)
        if self.steps_to_perform["language"]:
            ############
            # set language to English (United States)
            ############
            print "[ {0} ]: Set language to English (United States)".format(self.serial)
            cts_steps.set_language(serial = self.serial,
                             intent = self.intent)()
            ui_steps.press_home(serial = self.serial)()
            print "[ {0} ]: Language - Done!".format(self.serial)
        if self.steps_to_perform["sleep"]:
            ############
            # set sleep to 30 minutes
            ############
            print "[ {0} ]: Set sleep to 30 minutes".format(self.serial)
            cts_steps.set_display(serial = self.serial,
                                  intent = self.intent)()
            ui_steps.press_home(serial = self.serial)()
            print "[ {0} ]: Sleep - Done!".format(self.serial)
        if self.steps_to_perform["location"]:
            ############
            # set location On
            ############
            print "[ {0} ]: Set location On".format(self.serial)
            cts_steps.set_location(serial = self.serial,
                                   intent = self.intent)()
            ui_steps.press_home(serial = self.serial)()
            print "[ {0} ]: Location -- Done!".format(self.serial)
        if self.steps_to_perform["brightness"]:
            ############
            # set brightness to minimum
            ############
            print "[ {0} ]: Set brigthness to minimum".format(self.serial)
            set_brightness(serial = self.serial,
                                     intent = self.intent)()
            ui_steps.press_home(serial = self.serial)()
            print "[ {0} ]: Brightness -- Done!".format(self.serial)
        if self.steps_to_perform["wifi"]:
            ############
            # connect to WiFi
            ############
            try:
                print  self.ap_802_1x_EAP_method
                print "[ {0} ]: Connect to WiFi".format(self.serial)
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
                wifi_generic_steps.wait_until_connected(serial = self.serial,
                                                        timeout = 180)()
                wifi_generic_steps.check_connection_info(serial = self.serial,
                                                         SSID = self.ap_name,
                                                         state='CONNECTED/CONNECTED')()
                ui_steps.press_home(serial = self.serial)()
                print "[ {0} ]: WiFi - Done!".format(self.serial)
            except:
                print "[ {0} ]: Error connecting to WiFi - Try connecting manually: {1}".format(self.serial, traceback.format_exc())

    def check_condition(self):
        return self.step_data


class prepare_devices_for_gts(base_step):

    """ description:
            calls prepare_gts for every device in <serials>. GTS details
            should be present. Details for WiFi connection should be
            provided: <hidden_ap>, <ap_name>, <ap_pass>.
            Each device has a thread for performing these actions.

        usage:
            gts_steps.prepare_device_for_gts(serials = serials,
                                              gts_media_path = gts_media_path,
                                              gts_base_path = gts_base_path,
                                              gts_tcs_dir = gts_tcs_dir,
                                              gts_admin_apk = gts_admin_apk,
                                              hidden_ap = False,
                                              ap_name = "Android Core QA",
                                              ap_pass = "AndroidQA")()

        tags:
            android, gts, prepare, threads
    """
    def __init__(self,
                 devices,
                 gts_base_path,
                 gts_tcs_dir,
                 platform,
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
                 **kwargs):
        self.devices = devices
        self.serials = [d["serial"] for d in devices]
        self.gts_base_path = gts_base_path
        self.gts_tcs_dir = gts_tcs_dir
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
        self.intent = intent
        self.parallel = parallel
        base_step.__init__(self, **kwargs)

    def do(self):
        if self.parallel:
            pool = base_utils.LoggingPool(processes = len(self.serials))
            manager = Manager()
            self.step_data = manager.dict()
            after_flash_processes = []
        else:
            self.step_data = {}
        for d in self.devices:
            if self.parallel:
                new_process = pool.apply_async(prepare_gts,
                                               [d,
                                                self.gts_base_path,
                                                self.gts_tcs_dir,
                                                self.platform,
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
                                                self.step_data
                                                ])
                after_flash_processes.append(new_process)
            else:
                prepare_gts(d,
                            self.gts_base_path,
                            self.gts_tcs_dir,
                            self.platform,
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
                            self.step_data
                            )
        if self.parallel:
            pool.close()
            pool.join()

    def check_condition(self):
        self.set_passm("Prerequisits on {0} performed!!!".format(self.serials))
        ret = True
        message = ""
        for serial in [device["serial"] for device in self.devices]:
            if self.step_data[serial] == False:
                message += "{0} ".format(serial)
                ret = False
        if ret:
            self.set_passm("Prerequisits on {0} done!!".format(self.serials))
            return True
        else:
            self.set_errorm("", "Prerequisits on {0} were not perform correctly!!".format(message))
            return False


def prepare_gts(device,
                gts_base_path,
                gts_tcs_dir,
                platform,
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
                ret_values):

    """ description:
            function called by the prepare gts multithreading pool.
            [Ion] TODO: find a solution to make it member of the
            prepare_devices_for_gts step
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
        "brightness": True,
        "wifi": True,
    }
    while iteration > 0:
        print steps_to_perform
        try:
            ret_values[serial] = False
            after_flash_operations(device = device,
                                   steps_to_perform = steps_to_perform,
                                   gts_base_path = gts_base_path,
                                   gts_tcs_dir = gts_tcs_dir,
                                   serial = serial,
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
                                   intent = intent)()
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
                    print "{0} Error rebooting via adb- {1}".format(serial, traceback.format_exc())
                    try:
                        print "[ {0} ]: Rebooting device using relay".format(serial)
                        cts_steps.relay_reboot_device(device = device,
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
