#!/usr/bin/env python
#coding:utf-8

import os
import re
import time
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.fastboot.fastboot_step import step as fastboot_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.connections.local import local_steps
from testlib.scripts.android.fastboot import fastboot_utils
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.connections.local import local_utils


class command(fastboot_step):

    """ description:
            runs the given command to fastboot. to check the correct
                execution of the command, the stdout or stderr can be
                grepped for given string (if it is present or not)

        usage:
            command(command = "command_to_be_executed",
                    <stdout_grep = "text_to_exist_in_stdout>,
                    <stderr_grep = "text_to_exist_in_stderr>)()

        tags:
            fastboot, command, grep, stdout, stderr
    """
    command = None

    def __init__(self, command, stdout_grep = None, stderr_grep = None,
                 timeout = 5, mode = "sync", **kwargs):
        self.command = command
        self.stdout_grep = stdout_grep
        self.stderr_grep = stderr_grep
        self.mode = mode
        self.timeout = timeout
        fastboot_step.__init__(self, **kwargs)
        self.set_errorm("", "Executing fastboot command {0}".format(self.command))
        self.set_passm("Executing fastboot command {0}".format(self.command))

    def do(self):
        self.step_data = local_steps.command(command = "fastboot -s "
                                                        + self.serial + " "
                                                        + self.command,
                                             stdout_grep = self.stdout_grep,
                                             stderr_grep = self.stderr_grep,
                                             mode = self.mode,
                                             timeout = self.timeout)()

    def check_condition(self):
        if self.verbose:
            stds = "\n\tSTDOUT = \n\t {0} \tSTDERR =\n\t{1}\n".format(self.step_data[0], self.step_data[1])
        if self.stdout_grep:
            if self.verbose:
                error_mess = "\'{0}\' not in stdout - {1}".format(self.stdout_grep, stds)
                self.set_errorm("Executing {0}: {1}".format(self.command, error_mess))
                self.set_passm("Executing {0}: {1}".format(self.command, stds))
            return self.stdout_grep in str(self.step_data)
        elif self.stderr_grep:
            if self.verbose:
                error_mess = "\'{0}\' not in stderr - {1}".format(self.stderr_grep, stds)
                self.set_errorm("Executing {0}: {1}".format(self.command, error_mess))
                self.set_passm("Executing {0}: {1}".format(self.command, stds))
            return self.stderr_grep in str(self.step_data)
        return True


class change_state(fastboot_step):

    """ description:
            Changes bootloader state.

        usage:
            fastboot_steps.change_state(state = "verified")()

        tags:
            fastboot, android, bootloader, unlock
    """

    def __init__(self, unlock_bootloader, dessert, **kwargs):
        fastboot_step.__init__(self, **kwargs)
        self.unlock_bootloader = unlock_bootloader
        self.dessert = dessert
        self.set_errorm("", "Could not change state to {0}".format(self.unlock_bootloader))
        self.set_passm("State Changed to {}".format(self.unlock_bootloader))
        if self.dessert == "L":
            self.unlock_cmd = "oem"
        elif self.dessert == "M":
            self.unlock_cmd = "flashing"

        if self.unlock_bootloader == "yes":
            self.state_cmd = "unlock"
        elif self.unlock_bootloader == "no":
            self.state_cmd = "lock"


    def do(self):
        current_state = fastboot_utils.get_var(serial=self.serial,
                                               var="unlocked")

        if self.unlock_bootloader not in str(current_state):
            command(serial=self.serial,
                    command=self.unlock_cmd + " " + self.state_cmd,
                    stderr_grep="finished. total time:",
                    timeout=120000)()

    def check_condition(self):
        return self.unlock_bootloader == fastboot_utils.get_var(serial=self.serial,
                                                                var="unlocked")


class change_color_state(fastboot_step):
    """ description:
            Changes color boot state.

        usage:
            fastboot_steps.change_color_state(serial = serial,
                                              state = "red",
                                              boot_img = "/tmp/<username>/img/user_signed_corrupted_boot.img",
                                              dessert=dut_dessert)()

        tags:
            fastboot, android, bootloader, unlock
    """

    def __init__(self, state, boot_img, dessert, **kwargs):
        fastboot_step.__init__(self, **kwargs)
        self.state = state
        self.boot_img = boot_img
        self.dessert = dessert
        self.set_errorm("", "Could not change color to {0}".format(self.state))
        self.set_passm("Color Changed to {0}".format(self.state))


    def do(self):
        # reboot to fastboot
        adb_steps.reboot(serial = self.serial,
                         command = "fastboot",
                         ip_enabled = False,
                         blocking = True)()

        # unlock
        change_state(serial = self.serial,
                     unlock_bootloader = "yes",
                     dessert = self.dessert)()


        # erase boot
        command(serial = self.serial,
                command = "erase boot")()

        # flash boot user_signed_boots
        command(serial = self.serial,
               command = "flash boot {0}".format(self.boot_img))()

        # lock
        change_state(serial = self.serial,
                     unlock_bootloader = "no",
                     dessert = self.dessert)()

        # check for fastboot variables
        continue_to_adb(serial = self.serial)()
        adb_steps.wait_for_ui_processes(serial = self.serial)()

    def check_condition(self):
        return adb_steps.check_gvb_state(serial = self.serial,
                                         color = self.state)()


class continue_to_adb(fastboot_step):

    """ description:
            Continues to adb.

        usage:
            fastboot_steps.continue_to_adb()()

        tags:
            fastboot, android, bootloader, unlock
    """

    def __init__(self, timeout = 600, **kwargs):
        self.timeout = timeout
        fastboot_step.__init__(self, **kwargs)

    def do(self):
        command(serial = self.serial,
                command = "continue",
                timeout = self.timeout)()

    def check_condition(self):
        return local_steps.wait_for_adb(serial = self.serial,
                                        timeout = self.timeout)()


class reboot_fastboot(fastboot_step):

    """ description:
            Reboots to fastboot.

        usage:
            fastboot_steps.reboot_fastboot()()

        tags:
            fastboot, android, bootloader, reboot
    """

    def __init__(self, timeout = 120, **kwargs):
        self.timeout = timeout
        fastboot_step.__init__(self, **kwargs)

    def do(self):
        command(serial = self.serial,
                command = "reboot-bootloader",
                timeout = self.timeout)()

    def check_condition(self):
        return local_steps.wait_for_fastboot(serial = self.serial, timeout = self.timeout)()


class reboot_bootloader(fastboot_step):

    """ description:
            Reboots to fastboot.

        usage:
            fastboot_steps.reboot_bootloader()()

        tags:
            fastboot, android, bootloader, reboot
    """

    def __init__(self, timeout = 120, **kwargs):
        self.timeout = timeout
        fastboot_step.__init__(self, **kwargs)

    def do(self):
        command(serial = self.serial,
                command = "reboot bootloader",
                timeout = self.timeout)()

    def check_condition(self):
        return local_steps.wait_for_fastboot(serial = self.serial, timeout = self.timeout)()


class reboot(fastboot_step):

    """ description:
            Reboots to android.

        usage:
            fastboot_steps.reboot()()

        tags:
            fastboot, android, reboot
    """

    def __init__(self, timeout = 120, **kwargs):
        self.timeout = timeout
        fastboot_step.__init__(self, **kwargs)

    def do(self):
        command(serial = self.serial,
                command = "reboot",
                timeout = self.timeout)()

    def check_condition(self):
        return local_steps.wait_for_adb(serial = self.serial, timeout = self.timeout)()


class reboot_crashmode(fastboot_step):

    """ description:
            Reboots to crashmode.

        usage:
            fastboot_steps.reboot_crashmode()()

        tags:
            fastboot, android, bootloader, reboot, crashmode
    """

    def __init__(self, timeout = 120, **kwargs):
        self.timeout = timeout
        fastboot_step.__init__(self, **kwargs)

    def do(self):
        command(serial = self.serial,
                command = "oem reboot crashmode",
                timeout = self.timeout)()

    def check_condition(self):
        return local_steps.wait_for_crashmode(serial = self.serial, timeout = self.timeout)()


class check_partition_size_bounds(fastboot_step):

    """ description:
            Flashes a partition with a dummy file (size bigger than the partition's).
            Checks if the bootloader handles this by rejecting the file.

        usage:
            fastboot_steps.check_partition_size_bounds(partition="misc", test_file="./test_dummy.tmp")()

        tags:
            fastboot, android, bootloader, check, bounds
    """

    def __init__(self, test_file, **kwargs):
        fastboot_step.__init__(self, **kwargs)
        self.partition = self.device_info.partition_for_bounds_test
        self.test_file = test_file
        self.set_errorm("", "The {0} partition should not accept the file: {1}".format(self.partition, self.test_file))
        self.set_passm("The {0} partition was not flashed with bigger file: {1}".format(self.partition, self.test_file))

    def do(self):
        self.step_data = command(serial=self.serial,
                                 command="flash {0} {1} 2>&1".format(self.partition, self.test_file),
                                 stdout_grep=self.device_info.partition_bounds_check_string)()

    def check_condition(self):
        return self.step_data


class check_all_vars(fastboot_step):

    """ description:
            Runs the "fastboot getvar all" command and checks that some vars are present and have valid values

        usage:
            fastboot_steps.check_all_vars()()

        tags:
            fastboot, android, bootloader, check, var
    """

    def __init__(self, device_info = None, **kwargs):
        fastboot_step.__init__(self, **kwargs)
        self.set_passm("The vars were successfully checked")
        self.out = None
        self.err = None
        self.regex_patterns = self.device_info.getvar_test_regex_patterns
        self.device_info = device_info

    def do(self):
        self.out, self.err = command(serial=self.serial,
                                     command="getvar all")()

    def check_condition(self):
        for var in self.regex_patterns:
            if var == "battery_regex" and self.device_info == "broxtonp":
                continue
            if not re.findall(self.regex_patterns[var], self.err):
                self.set_errorm("", "The {0} var is not found".format(var))
                return False

        # If all the vars are OK, the test should pass
        return True


class setup_the_oobe(ui_step):

    def __init__(self, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time

    def do(self):
        fastboot_utils.push_uiautomator_jar(serial = self.serial)
        if self.uidevice(resourceId = "com.google.android.setupwizard:id/welcome_title", text = "Welcome").wait.exists(timeout = self.wait_time):
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.google.android.setupwizard:id/start"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/suw_navbar_next", "text": "Skip"})()
            if self.uidevice(resourceId = "android:id/message").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/button1", "text": "Skip anyway"})()
            if self.uidevice(resourceId = "com.google.android.setupwizard:id/suw_layout_title", text = "Date & time").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.google.android.setupwizard:id/suw_navbar_next", "text": "Next"})()
            if self.uidevice(resourceId = "com.google.android.setupwizard:id/suw_layout_title", text = "Name").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.google.android.setupwizard:id/suw_navbar_next", "text": "Next"})()
            return_result = ui_utils.is_checkbox_checked(serial = self.serial,
                                                                                       view_to_find = {"resourceId": "com.google.android.setupwizard:id/lock_screen_intro_check_box",
                                                                                                                "text": "Protect this device and require a PIN, pattern, or password to unlock the screen"})
            if return_result:
                ui_steps.click_button(serial = self.serial,
                                                    view_to_find = {"resourceId": "com.google.android.setupwizard:id/lock_screen_intro_check_box",
                                                                              "text": "Protect this device and require a PIN, pattern, or password to unlock the screen"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.google.android.setupwizard:id/suw_navbar_next", "text": "Skip"})()
            if self.uidevice(resourceId = "android:id/message").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/button1", "text": "Skip anyway"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.google.android.gms:id/suw_navbar_next", "text": "Next"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.intel.crashreport:id/button_accept_disclaimer", "text": "OK"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.launcher3:id/cling_dismiss_longpress_info", "text": "GOT IT"})()

    def check_condition(self):
        return True


class config_first_boot_wizard(ui_step):

    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.x = self.uidevice.info["displayWidth"]
        self.y = self.uidevice.info["displayHeight"]
        self.succeed = False

    def do(self):
        # time.sleep(15)
        self.uidevice.wakeup()
        self.uidevice.press("home")
        time.sleep(2)
        if self.uidevice(textContains="Drive safely").exists:
            self.uidevice(text="Owner").click.wait()
            self.succeed = True
            return
        self.uidevice.wakeup()
        if self.uidevice(resourceId="com.android.systemui:id/lock_icon"):
            self.close_lock_screen()
            self.succeed = True
            return
        for i in range(10):
            try:
                if self.uidevice(text="OK").exists:
                    self.uidevice(text="OK").click.wait()
                if self.uidevice(text="Welcome") or \
                        self.uidevice(resourceId="com.google.android.setupwizard:id/welcome_title") or \
                        self.uidevice(resourceId="com.google.android.setupwizard:id/start"):
                    self.uidevice.click(100, 100)
                    self.uidevice.click(self.x-100, 100)
                    self.uidevice.click(self.x-100, self.y-100)
                    self.uidevice.click(100, self.y-100)
                    time.sleep(2)
                    if self.uidevice(text="OK").exists:
                        self.uidevice(text="OK").click.wait()
                        self.uidevice(text="OK").wait.gone(timeout=3000)
                    if self.uidevice(text="GOT IT").exists:
                        self.uidevice(text="GOT IT").click.wait()
                        self.uidevice(text="GOT IT").wait.gone(timeout=3000)
                    self.close_lock_screen()
                    self.succeed = True
                    return
                break
            except Exception as e:
                print e
        self.succeed = True
        time.sleep(30)

    def check_condition(self):
        return self.succeed

    def close_lock_screen(self):
        self.uidevice.wakeup()
        if self.uidevice(resourceId="com.android.systemui:id/lock_icon"):
            self.uidevice.swipe(self.x/2, self.y, self.x/2, 0)
            self.uidevice.press("menu")
            self.uidevice.press("home")
        os.system(" adb -s %s shell am start -S com.android.settings/.Settings" % self.serial)
        time.sleep(2)
        if not self.uidevice(textContains="Security").exists:
            try:
                self.uidevice(scrollable=True).scroll.vert.to(textContains="Security")
            except:
                print "Can't scroll"
        self.uidevice(textContains="Security").click()
        self.uidevice(text="Screen lock").click()
        self.uidevice(text="None").click()
        assert self.uidevice(text="Screen lock").down(text="None") != None
        self.uidevice.press("home")


class factory_data_reset(ui_step):

    def __init__(self, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time
        self.platform_name = fastboot_utils.get_platform_name(serial=self.serial)

    def do(self):
        if self.platform_name == "bxtp_abl":
            ui_steps.open_settings(serial = self.serial)()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/title", "text": "Backup & reset"})()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Factory data reset"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/initiate_master_clear", "text": "Reset phone"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/execute_master_clear", "text": "Erase everything"})()
        if self.platform_name == "gordon_peak":
            # ui_steps.open_settings(serial = self.serial)()
            # ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "System"})()
            # ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Reset"})()
            # ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Factory data reset"})()
            # ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/initiate_master_clear", "text": "RESET PHONE"})()
            # ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/execute_master_clear", "text": "ERASE EVERYTHING"})()
            ui_steps.am_start_command(component = "com.android.settings/.Settings", serial = self.serial)()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"text": "System"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"text": "Reset options"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"textContains": "Erase all data"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"text": "RESET PHONE"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"text": "ERASE EVERYTHING"})()
        if self.platform_name == "androidia_64":
            ui_steps.am_start_command(component = "com.android.settings/.Settings", serial = self.serial)()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"text": "System"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"text": "Reset options"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"textContains": "Erase all data"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"text": "RESET TABLET"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"text": "ERASE EVERYTHING"})()
        time.sleep(120)
        local_steps.wait_for_adb(serial = self.serial, timeout = 300)()
        time.sleep(10)

    def check_condition(self):
        return True


class corrupt_esp_partition(fastboot_step):

    def __init__(self, unlock_dut = True, lock_dut = False, reboot = True, **kwargs):
        fastboot_step.__init__(self, **kwargs)
        self.unlock_dut = unlock_dut
        self.lock_dut = lock_dut
        self.reboot = reboot

    def do(self):
        if self.serial in local_utils.get_connected_android_devices()['android']:
            adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = self.serial)()
        if self.unlock_dut:
            unlock_device(serial = self.serial)()
        command(command = "oem garbage-disk  > ./temp/files/oem_garbagedisk_result.txt 2>&1", serial = self.serial)()
        return_result = fastboot_utils.fastboot_command_result(file_name = "./temp/files/oem_garbagedisk_result.txt")
        if not return_result: raise Exception("The test result did not achieve the desired results")
        if self.lock_dut:
            lock_device(serial = self.serial)()
        if self.reboot:
            os.system("fastboot reboot > /dev/null 2>&1")
            time.sleep(10)

    def check_condition(self):
        if not self.reboot: return True
        if self.serial in local_utils.get_connected_android_devices()['fastboot']: return True
        return False


class flash_wrong_file(fastboot_step):

    def __init__(self, partition_name = None, file_name = "./temp/image/n/flashfiles/flash.json", unlock_dut = True, lock_dut = False, **kwargs):
        fastboot_step.__init__(self, **kwargs)
        self.partition_name = partition_name
        self.file_name = file_name
        self.unlock_dut = unlock_dut
        self.lock_dut = lock_dut

    def do(self):
        if self.serial in local_utils.get_connected_android_devices()['android']:
            adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = self.serial)()
        if self.unlock_dut:
            unlock_device(serial = self.serial)()
        command(command = "flash {0} {1} > ./temp/files/flash_{0}_result.txt 2>&1".format(self.partition_name, self.file_name), serial = self.serial)()
        self.flash_result = fastboot_utils.fastboot_command_result(file_name = "./temp/files/flash_{0}_result.txt".format(self.partition_name))
        if self.flash_result == False: raise Exception("The test result did not achieve the desired results")
        if self.lock_dut:
            lock_device(serial = self.serial)()
        os.system("fastboot reboot > /dev/null 2>&1")
        time.sleep(60)

    def check_condition(self):
        if self.serial not in local_utils.get_connected_android_devices()['android']:
            fastboot_utils.to_fastboot_by_script(serial=self.serial)
            return True
        return False


class fastboot_erase_partition(fastboot_step):

    def __init__(self, partition_name = None, unlock_dut = True, lock_dut = False, reboot = True, **kwargs):
        fastboot_step.__init__(self, **kwargs)
        self.partition_name = partition_name
        self.unlock_dut = unlock_dut
        self.lock_dut = lock_dut
        self.reboot = reboot

    def do(self):
        if self.serial in local_utils.get_connected_android_devices()['android']:
            adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = self.serial)()
        if self.unlock_dut:
            unlock_device(serial = self.serial)()
        command(command = "erase {0} > ./temp/files/erase_{0}_result.txt 2>&1".format(self.partition_name), serial = self.serial)()
        self.erase_result = fastboot_utils.fastboot_command_result(file_name = "./temp/files/erase_{0}_result.txt".format(self.partition_name))
        if self.erase_result == False: raise Exception("The test result did not achieve the desired results")
        if self.lock_dut:
            lock_device(serial = self.serial)()
        if self.reboot:
            os.system("fastboot reboot > /dev/null 2>&1")
            time.sleep(60)

    def check_condition(self):
        if not self.reboot: return True
        if self.serial not in local_utils.get_connected_android_devices()['android']:
            fastboot_utils.to_fastboot_by_script(serial=self.serial)
            return True
        return False


class flash_right_file(fastboot_step):

    def __init__(self, partition_name = None, file_name = None, unlock_dut = True, lock_dut = False, **kwargs):
        fastboot_step.__init__(self, **kwargs)
        self.partition_name = partition_name
        self.file_name = file_name
        self.unlock_dut = unlock_dut
        self.lock_dut = lock_dut

    def do(self):
        if self.serial in local_utils.get_connected_android_devices()['android']:
            adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = self.serial)()
        if self.unlock_dut:
            unlock_device(serial = self.serial)()
        command(command = "flash {0} ./temp/image/n/flashfiles/{1} > ./temp/files/flash_{0}_result.txt 2>&1".format(self.partition_name, self.file_name), serial = self.serial)()
        self.flash_result = fastboot_utils.fastboot_command_result(file_name = "./temp/files/flash_{0}_result.txt".format(self.partition_name))
        if self.flash_result == False: raise Exception("The test result did not achieve the desired results")
        if self.lock_dut:
            lock_device(serial = self.serial)()
        continue_to_adb(serial = self.serial)()
        time.sleep(60)
        local_steps.wait_for_adb(timeout = 300, serial = self.serial)()

    def check_condition(self):
        return fastboot_utils.fastboot_command_result(file_name = "./temp/files/flash_{0}_result.txt".format(self.partition_name))


class flash_image(fastboot_step):

    def __init__(self, partition_name = None, file_name = None, unlock_dut = True, lock_dut = False, **kwargs):
        fastboot_step.__init__(self, **kwargs)
        self.partition_name = partition_name
        self.file_name = file_name
        self.unlock_dut = unlock_dut
        self.lock_dut = lock_dut

    def do(self):
        if self.unlock_dut:
            unlock_device(serial = self.serial)()
        command(command = "flash {0} {1} > ./temp/files/flash_{0}_result.txt 2>&1".format(self.partition_name, self.file_name), serial = self.serial)()
        self.flash_result = fastboot_utils.fastboot_command_result(file_name = "./temp/files/flash_{0}_result.txt".format(self.partition_name))
        if self.flash_result == False: raise Exception("The test result did not achieve the desired results")
        if self.lock_dut:
            lock_device(serial = self.serial)()

    def check_condition(self):
        return True


class format_partition(fastboot_step):

    def __init__(self, partition_name=None, unlock_dut=False, lock_dut=False, **kwargs):
        fastboot_step.__init__(self, **kwargs)
        self.partition_name = partition_name
        self.unlock_dut = unlock_dut
        self.lock_dut = lock_dut
        self.format_result = None

    def do(self):
        if self.unlock_dut:
            unlock_device(serial=self.serial)()
        command(command="format {0} > ./temp/files/format_{0}_result.txt 2>&1".format(self.partition_name), serial=self.serial)()
        self.format_result = fastboot_utils.fastboot_command_result(file_name="./temp/files/format_{0}_result.txt".format(self.partition_name))
        if self.lock_dut:
            lock_device(serial=self.serial)()

    def check_condition(self):
        return self.format_result


class unlock_device(fastboot_step):

    def __init__(self, **kwargs):
        fastboot_step.__init__(self, **kwargs)
        self.device_state = None
        self.platform_name = fastboot_utils.get_platform_name(serial=self.serial)

    def do(self):
        command(command = "getvar device-state > ./temp/files/device_state.txt 2>&1", serial = self.serial)()
        self.device_state = fastboot_utils.get_device_state(file_path = "./temp/files/device_state.txt")
        if self.device_state == "locked":
            command(command = "flashing unlock > ./temp/files/unlock_result.txt 2>&1", serial = self.serial)()
            self.change_result = fastboot_utils.fastboot_command_result(file_name = "./temp/files/unlock_result.txt")
            if self.change_result == False: raise Exception("The test result did not achieve the desired results")
            if self.platform_name == "gordon_peak":
                command(command = "format data > ./temp/files/format_data_result.txt 2>&1", serial = self.serial)()
                self.format_result = fastboot_utils.fastboot_command_result(file_name = "./temp/files/format_data_result.txt")
                if self.format_result == False: raise Exception("The test result did not achieve the desired results")
            command(command = "getvar device-state > ./temp/files/device_state.txt 2>&1", serial = self.serial)()
            self.device_state = fastboot_utils.get_device_state(file_path = "./temp/files/device_state.txt")

    def check_condition(self):
        if self.device_state == "unlocked": return True
        return False


class lock_device(fastboot_step):

    def __init__(self, **kwargs):
        fastboot_step.__init__(self, **kwargs)
        self.device_state = None
        self.platform_name = fastboot_utils.get_platform_name(serial=self.serial)

    def do(self):
        command(command = "getvar device-state > ./temp/files/device_state.txt 2>&1", serial = self.serial)()
        self.device_state = fastboot_utils.get_device_state(file_path = "./temp/files/device_state.txt")
        if self.device_state == "unlocked":
            if self.platform_name == "gordon_peak":
                command(command = "format data > ./temp/files/format_data_result.txt 2>&1", serial = self.serial)()
                self.format_result = fastboot_utils.fastboot_command_result(file_name = "./temp/files/format_data_result.txt")
                if self.format_result == False: raise Exception("The test result did not achieve the desired results")
            command(command = "flashing lock > ./temp/files/lock_result.txt 2>&1", serial = self.serial)()
            self.change_result = fastboot_utils.fastboot_command_result(file_name = "./temp/files/lock_result.txt")
            if self.change_result == False: raise Exception("The test result did not achieve the desired results")
            command(command = "getvar device-state > ./temp/files/device_state.txt 2>&1", serial = self.serial)()
            self.device_state = fastboot_utils.get_device_state(file_path = "./temp/files/device_state.txt")

    def check_condition(self):
        if self.device_state == "locked": return True
        return False


class create_some_contacts(ui_step):

    def __init__(self, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time
        self.platform_name = fastboot_utils.get_platform_name(serial=self.serial)

    def do(self):
        if self.platform_name == "bxtp_abl":
            ui_steps.am_start_command(component = "com.android.contacts/.activities.PeopleActivity", serial = self.serial)()
            if self.uidevice(resourceId = "com.google.android.gms:id/suw_layout_title", text = "Couldn't sign in").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.google.android.gms:id/suw_navbar_next", "text": "Next"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.contacts:id/floating_action_button"})()
            if self.uidevice(resourceId = "com.android.contacts:id/text", text = "Your new contact won't be backed up. Add an account that backs up contacts online?").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.contacts:id/left_button", "text": "Keep local"})()
            if self.uidevice(text = "Add new contact").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial, view_to_find = {"text": "Name"})()
                local_steps.command("adb -s {} shell input text test".format(self.serial))()
                ui_steps.click_button(serial = self.serial, view_to_find = {"text": "Phone"})()
                local_steps.command("adb -s {} shell input text 1234567890".format(self.serial))()
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.contacts:id/menu_save"})()
            self.uidevice.press.back()
        if self.platform_name == "gordon_peak":
            ui_steps.am_start_command(component = "com.android.contacts/.activities.PeopleActivity", serial = self.serial)()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.contacts:id/floating_action_button"})()
            if self.uidevice(resourceId = "com.android.contacts:id/text", text = "Take a minute to add an account that will back up your contacts to Google.").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.contacts:id/left_button", "text": "CANCEL"})()
            if self.uidevice(text = "Create new contact").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial, view_to_find = {"text": "First name"})()
                local_steps.command("adb -s {} shell input text test".format(self.serial))()
                ui_steps.click_button(serial = self.serial, view_to_find = {"text": "Phone"})()
                local_steps.command("adb -s {} shell input text 1234567890".format(self.serial))()
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.contacts:id/editor_menu_save_button", "text": "SAVE"})()
            self.uidevice.press.back()
        if self.platform_name == "androidia_64":
            ui_steps.am_start_command(component = "com.android.contacts/.activities.PeopleActivity", serial = self.serial)()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.contacts:id/floating_action_button"})()
            if self.uidevice(resourceId = "com.android.contacts:id/text", text = "Take a minute to add an account that will back up your contacts to Google.").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.contacts:id/left_button", "text": "CANCEL"})()
            if self.uidevice(text = "Create new contact").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial, view_to_find = {"text": "First name"})()
                local_steps.command("adb -s {} shell input text test".format(self.serial))()
                ui_steps.click_button(serial = self.serial, view_to_find = {"text": "Phone"})()
                local_steps.command("adb -s {} shell input text 1234567890".format(self.serial))()
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.contacts:id/editor_menu_save_button", "text": "SAVE"})()
            self.uidevice.press.back()

    def check_condition(self):
        if self.platform_name == "bxtp_abl":
            return not self.uidevice(resourceId = "com.android.contacts:id/message", text = "No contacts.").wait.exists(timeout = self.wait_time)
        if self.platform_name == "gordon_peak":
            return not self.uidevice(resourceId = "com.android.contacts:id/message", text = "Your contacts list is empty").wait.exists(timeout = self.wait_time)
        if self.platform_name == "androidia_64":
            return not self.uidevice(resourceId = "com.android.contacts:id/message", text = "Your contacts list is empty").wait.exists(timeout = self.wait_time)
        return False


class contact_is_empty(ui_step):

    def __init__(self, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time
        self.platform_name = fastboot_utils.get_platform_name(serial=self.serial)

    def do(self):
        if self.platform_name == "bxtp_abl":
            ui_steps.am_start_command(component = "com.android.contacts/.activities.PeopleActivity", serial = self.serial)()
            if self.uidevice(resourceId = "com.google.android.gms:id/suw_layout_title", text = "Couldn't sign in").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.google.android.gms:id/suw_navbar_next", "text": "Next"})()
        if self.platform_name == "gordon_peak":
            ui_steps.am_start_command(component = "com.android.contacts/.activities.PeopleActivity", serial = self.serial)()
        if self.platform_name == "androidia_64":
            ui_steps.am_start_command(component = "com.android.contacts/.activities.PeopleActivity", serial = self.serial)()

    def check_condition(self):
        if self.platform_name == "bxtp_abl":
            return self.uidevice(resourceId = "com.android.contacts:id/message", text = "No contacts.").wait.exists(timeout = self.wait_time)
        if self.platform_name == "gordon_peak":
            return self.uidevice(resourceId = "com.android.contacts:id/message", text = "Your contacts list is empty").wait.exists(timeout = self.wait_time)
        if self.platform_name == "androidia_64":
            return self.uidevice(resourceId = "com.android.contacts:id/message", text = "Your contacts list is empty").wait.exists(timeout = self.wait_time)
        return False


class connect_to_internet(ui_step):

    def __init__(self, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time
        self.platform_name = fastboot_utils.get_platform_name(serial=self.serial)

    def do(self):
        if self.platform_name == "bxtp_abl":
            ui_steps.open_settings(serial = self.serial)()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/title", "text": "Wi‑Fi"})()
            if self.uidevice(resourceId = "com.android.settings:id/switch_text", text = "Off").wait.exists(timeout = self.wait_time):
                self.uidevice(text = "Off").right(className = "android.widget.Switch").click.wait()
            ui_steps.click_button(serial = self.serial, view_to_find = {"className": "android.widget.ImageButton", "description": "More options"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Add network"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/security"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/text1", "text": "WPA/WPA2 PSK"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/ssid", "text": "Enter the SSID"})()
            local_steps.command("adb -s {} shell input text shz_asvl_ng_6d95_2.4".format(self.serial))()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/password"})()
            local_steps.command("adb -s {} shell input text 16220BB9CC12316220BB9CC123".format(self.serial))()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/button1", "text": "Save"})()
        if self.platform_name == "gordon_peak":
            ui_steps.am_start_command(component = "com.android.settings/.Settings", serial = self.serial)()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Network & Internet"})()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Wi‑Fi"})()
            if self.uidevice(resourceId = "com.android.settings:id/switch_widget", text = "OFF").wait.exists(timeout = self.wait_time):
                self.uidevice(text = "Off").right(className = "android.widget.Switch").click.wait()
            ui_steps.click_button(serial = self.serial, view_to_find = {"text": "Add network"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/security"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/text1", "text": "WPA/WPA2 PSK"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/ssid", "text": "Enter the SSID"})()
            local_steps.command("adb -s {} shell input text shz_asvl_ng_6d95_2.4".format(self.serial))()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/password"})()
            local_steps.command("adb -s {} shell input text 16220BB9CC12316220BB9CC123".format(self.serial))()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/button1", "text": "SAVE"})()
        if self.platform_name == "androidia_64":
            ui_steps.am_start_command(component = "com.android.settings/.Settings", serial = self.serial)()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Network & Internet"})()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Wi‑Fi"})()
            if self.uidevice(resourceId = "com.android.settings:id/switch_text", text = "Off").wait.exists(timeout = self.wait_time):
                self.uidevice(text = "Off").right(className = "android.widget.Switch").click.wait()
            ui_steps.click_button(serial = self.serial, view_to_find = {"text": "Add network"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/security"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/text1", "text": "WPA/WPA2 PSK"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/ssid", "text": "Enter the SSID"})()
            local_steps.command("adb -s {} shell input text shz_asvl_ng_6d95_2.4".format(self.serial))()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/password"})()
            local_steps.command("adb -s {} shell input text 16220BB9CC12316220BB9CC123".format(self.serial))()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/button1", "text": "SAVE"})()

    def check_condition(self):
        if self.platform_name == "bxtp_abl":
            return self.uidevice(resourceId = "android:id/summary", text = "Connected").wait.exists(timeout = self.wait_time*12)
        if self.platform_name == "gordon_peak":
            return self.uidevice(resourceId = "android:id/summary", text = "Connected").wait.exists(timeout = self.wait_time*12)
        if self.platform_name == "androidia_64":
            return self.uidevice(resourceId = "android:id/summary", text = "Connected").wait.exists(timeout = self.wait_time*12)
        return False


class find_bt_devices(ui_step):

    def __init__(self, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time
        self.platform_name = fastboot_utils.get_platform_name(serial=self.serial)

    def do(self):
        if self.platform_name == "bxtp_abl":
            ui_steps.open_settings(serial=self.serial)()
            ui_steps.click_button_with_scroll(serial=self.serial, view_to_find={"resourceId": "com.android.settings:id/title", "text": "Bluetooth"})()
            if self.uidevice(resourceId="com.android.settings:id/switch_text", text="Off").wait.exists(timeout=self.wait_time):
                self.uidevice(text="Off").right(className="android.widget.Switch").click.wait()
        if self.platform_name == "gordon_peak":
            ui_steps.am_start_command(component = "com.android.settings/.Settings", serial = self.serial)()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Connected devices"})()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Bluetooth"})()
            if self.uidevice(resourceId = "com.android.settings:id/switch_text", text = "Off").wait.exists(timeout = self.wait_time):
                self.uidevice(text = "Off").right(className = "android.widget.Switch").click.wait()
            ui_steps.click_button(serial = self.serial, view_to_find = {"text": "Pair new device"})()
        if self.platform_name == "androidia_64":
            ui_steps.am_start_command(component = "com.android.settings/.Settings", serial = self.serial)()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Connected devices"})()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Bluetooth"})()
            if self.uidevice(resourceId = "com.android.settings:id/switch_text", text = "Off").wait.exists(timeout = self.wait_time):
                self.uidevice(text = "Off").right(className = "android.widget.Switch").click.wait()
            ui_steps.click_button(serial = self.serial, view_to_find = {"text": "Pair new device"})()

    def check_condition(self):
        if self.platform_name == "bxtp_abl":
            return self.uidevice(resourceId = "com.android.settings:id/scanning_progress").wait.exists(timeout = self.wait_time*2)
        if self.platform_name == "gordon_peak":
            return self.uidevice(resourceId = "com.android.settings:id/scanning_progress").wait.exists(timeout = self.wait_time*2)
        if self.platform_name == "androidia_64":
            return self.uidevice(resourceId = "com.android.settings:id/scanning_progress").wait.exists(timeout = self.wait_time*2)
        return False


class visit_web_page_by_browser(ui_step):

    def __init__(self, wait_time = 30000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time

    def do(self):
        self.uidevice.press.home()
        ui_steps.am_start_command(component = "com.android.chrome/com.google.android.apps.chrome.Main", serial = self.serial)()
        if self.uidevice(resourceId = "com.android.chrome:id/title", text = "Welcome to Chrome").wait.exists(timeout = self.wait_time):
            ui_steps.click_button(serial = self.serial,
                                                view_to_find = {"resourceId": "com.android.chrome:id/terms_accept", "text": "Accept & continue"},
                                                view_to_check = {"resourceId": "com.android.chrome:id/signin_title", "text": "Sign in to Chrome"})()
            ui_steps.click_button(serial = self.serial,
                                                view_to_find = {"resourceId": "com.android.chrome:id/negative_button", "text": "No Thanks"})()
        ui_steps.click_button(serial = self.serial,
                                            view_to_find = {"resourceId": "com.android.chrome:id/url_bar", "text": "Search or type URL"})()
        local_steps.command("adb -s {} shell input text www.baidu.com".format(self.serial))()
        self.uidevice.press.enter()
        if self.uidevice(className = "android.widget.Image", description = "百度一下,你就知道").wait.exists(timeout = self.wait_time):
            ui_steps.click_button(serial = self.serial,
                                                view_to_find = {"resourceId": "com.android.chrome:id/menu_button", "description": "More options"},
                                                view_to_check = {"resourceId": "com.android.chrome:id/menu_item_text", "text": "New tab"})()
            ui_steps.click_button(serial = self.serial,
                                                view_to_find = {"resourceId": "com.android.chrome:id/menu_item_text", "text": "History"},
                                                view_to_check = {"description": "History"})()

    def check_condition(self):
        return not self.uidevice(resourceId = "results-header", description = "No history entries found").wait.exists(timeout = self.wait_time)


class browser_history_is_empty(ui_step):

    def __init__(self, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time

    def do(self):
        ui_steps.am_start_command(component = "com.android.chrome/com.google.android.apps.chrome.Main", serial = self.serial)()
        if self.uidevice(resourceId = "com.android.chrome:id/title", text = "Welcome to Chrome").wait.exists(timeout = self.wait_time):
            ui_steps.click_button(serial = self.serial,
                                                view_to_find = {"resourceId": "com.android.chrome:id/terms_accept", "text": "Accept & continue"},
                                                view_to_check = {"resourceId": "com.android.chrome:id/signin_title", "text": "Sign in to Chrome"})()
            ui_steps.click_button(serial = self.serial,
                                                view_to_find = {"resourceId": "com.android.chrome:id/negative_button", "text": "No Thanks"})()
        ui_steps.click_button(serial = self.serial,
                                            view_to_find = {"resourceId": "com.android.chrome:id/menu_button", "description": "More options"},
                                            view_to_check = {"resourceId": "com.android.chrome:id/menu_item_text", "text": "New tab"})()
        ui_steps.click_button(serial = self.serial,
                                            view_to_find = {"resourceId": "com.android.chrome:id/menu_item_text", "text": "History"},
                                            view_to_check = {"description": "History"})()

    def check_condition(self):
        return self.uidevice(resourceId = "results-header", description = "No history entries found").wait.exists(timeout = self.wait_time)


class login_google_account(ui_step):

    def __init__(self, wait_time = 30000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time
        self.platform_name = fastboot_utils.get_platform_name(serial=self.serial)

    def do(self):
        if self.platform_name == "bxtp_abl":
            self.uidevice.press.home()
            ui_steps.open_settings(serial = self.serial)()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/title", "text": "Accounts"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Add account"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Google"})()
            if self.uidevice(resourceId = "headingText", description = "Add your account").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "identifierId", "text": "Email or phone"})()
                local_steps.command("adb -s {} shell input text otcqatestbackup@gmail.com".format(self.serial))()
                self.uidevice.press.enter()
            if self.uidevice(className = "android.view.View", description = "Sign in").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "password", "text": "Password"})()
                local_steps.command("adb -s {} shell input text otcqa123456".format(self.serial))()
                self.uidevice.press.enter()
            if self.uidevice(resourceId = "tosText").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "next", "description": "ACCEPT"})()
            if self.uidevice(resourceId = "com.google.android.gms:id/suw_layout_title", text = "Google services").wait.exists(timeout = self.wait_time):
                if self.uidevice(textContains = "Automatically back up device data").right(className = "android.widget.Switch").info["text"] == "ON":
                    self.uidevice(textContains = "Automatically back up device data").right(className = "android.widget.Switch").click.wait()
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.google.android.gms:id/suw_navbar_next", "text": "Next"})()
            if self.uidevice(resourceId = "com.android.vending:id/title", text = "Set up payment info").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.vending:id/title", "text": "No thanks"})()
                ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "com.android.vending:id/positive_button", "text": "Continue"})()
        if self.platform_name == "gordon_peak":
            self.uidevice.press.home()
            ui_steps.am_start_command(component = "com.android.settings/.Settings", serial = self.serial)()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Users & accounts"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Add account"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Google"})()
            if self.uidevice(text = "Email or phone").wait.exists(timeout = self.wait_time):
                time.sleep(5)
                ui_steps.click_button(serial = self.serial, view_to_find = {"text": "Email or phone"})()
                local_steps.command("adb -s {} shell input text otcqatestbackup@gmail.com".format(self.serial))()
                self.uidevice.press.enter()
                # ui_steps.click_button(serial = self.serial, view_to_find = {"text": "NEXT"})()
            if self.uidevice(text = "Enter your password").wait.exists(timeout = self.wait_time):
                time.sleep(5)
                ui_steps.click_button(serial = self.serial, view_to_find = {"text": "Enter your password"})()
                local_steps.command("adb -s {} shell input text otcqa123456".format(self.serial))()
                self.uidevice.press.enter()
                # ui_steps.click_button(serial = self.serial, view_to_find = {"text": "NEXT"})()
                time.sleep(5)
                ui_steps.click_button(serial = self.serial, view_to_find = {"description": "I AGREE"})()
            if self.uidevice(resourceId = "com.google.android.gms:id/suw_layout_title", text = "Google services").wait.exists(timeout = self.wait_time):
                if self.uidevice(textContains = "Back up to Google Drive").right(className = "android.widget.Switch").info["text"] == "ON":
                    self.uidevice(textContains = "Back up to Google Drive").right(className = "android.widget.Switch").click.wait()
                ui_steps.click_button(serial = self.serial, view_to_find = {"text": "AGREE"})()
        if self.platform_name == "androidia_64":
            self.uidevice.press.home()
            ui_steps.am_start_command(component = "com.android.settings/.Settings", serial = self.serial)()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Users & accounts"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Add account"})()
            ui_steps.click_button(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Google"})()
            if self.uidevice(text = "Email or phone").wait.exists(timeout = self.wait_time):
                time.sleep(5)
                ui_steps.click_button(serial = self.serial, view_to_find = {"text": "Email or phone"})()
                local_steps.command("adb -s {} shell input text otcqatestbackup@gmail.com".format(self.serial))()
                self.uidevice.press.enter()
                # ui_steps.click_button(serial = self.serial, view_to_find = {"text": "NEXT"})()
            if self.uidevice(text = "Enter your password").wait.exists(timeout = self.wait_time):
                time.sleep(5)
                ui_steps.click_button(serial = self.serial, view_to_find = {"text": "Enter your password"})()
                local_steps.command("adb -s {} shell input text otcqa123456".format(self.serial))()
                self.uidevice.press.enter()
                # ui_steps.click_button(serial = self.serial, view_to_find = {"text": "NEXT"})()
                time.sleep(5)
                ui_steps.click_button(serial = self.serial, view_to_find = {"description": "I AGREE"})()
            if self.uidevice(resourceId = "com.google.android.gms:id/suw_layout_title", text = "Google services").wait.exists(timeout = self.wait_time):
                if self.uidevice(textContains = "Back up to Google Drive").right(className = "android.widget.Switch").info["text"] == "ON":
                    self.uidevice(textContains = "Back up to Google Drive").right(className = "android.widget.Switch").click.wait()
                ui_steps.click_button(serial = self.serial, view_to_find = {"text": "AGREE"})()

    def check_condition(self):
        if self.platform_name == "bxtp_abl":
            return self.uidevice(resourceId = "android:id/title", text = "Google").wait.exists(timeout = self.wait_time)
        if self.platform_name == "gordon_peak":
            return self.uidevice(resourceId = "android:id/summary", text = "Google").wait.exists(timeout = self.wait_time)
        if self.platform_name == "androidia_64":
            return self.uidevice(resourceId = "android:id/summary", text = "Google").wait.exists(timeout = self.wait_time)
        return False


class google_account_is_empty(ui_step):

    def __init__(self, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time
        self.platform_name = fastboot_utils.get_platform_name(serial=self.serial)

    def do(self):
        if self.platform_name == "bxtp_abl":
            ui_steps.open_settings(serial = self.serial)()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "com.android.settings:id/title", "text": "Accounts"})()
        if self.platform_name == "gordon_peak":
            ui_steps.am_start_command(component = "com.android.settings/.Settings", serial = self.serial)()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Users & accounts"})()
        if self.platform_name == "androidia_64":
            ui_steps.am_start_command(component = "com.android.settings/.Settings", serial = self.serial)()
            ui_steps.click_button_with_scroll(serial = self.serial, view_to_find = {"resourceId": "android:id/title", "text": "Users & accounts"})()

    def check_condition(self):
        if self.platform_name == "bxtp_abl":
            return not self.uidevice(resourceId = "android:id/title", text = "Google").wait.exists(timeout = self.wait_time)
        if self.platform_name == "gordon_peak":
            return not self.uidevice(resourceId = "android:id/summary", text = "Google").wait.exists(timeout = self.wait_time)
        if self.platform_name == "androidia_64":
            return not self.uidevice(resourceId = "android:id/summary", text = "Google").wait.exists(timeout = self.wait_time)
        return False


class time_management_with_device_off(ui_step):

    def __init__(self, **kwargs):
        ui_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.am_start_command(component="com.android.settings/.Settings", serial=self.serial)()
        ui_steps.click_button_with_scroll(serial=self.serial, view_to_find={"text":"System"})()
        ui_steps.click_button(serial=self.serial, view_to_find={"text":"Date & time"})()
        if self.uidevice(text="Automatic date & time").right(className="android.widget.Switch").info["text"] == "ON":
            self.uidevice(text="Automatic date & time").right(className="android.widget.Switch").click.wait()
        # ui_steps.click_button(serial=self.serial, view_to_find={"text":"Set date"})()
        # ui_steps.click_button(serial=self.serial, view_to_find={"text":"1"})()
        # ui_steps.click_button(serial=self.serial, view_to_find={"text":"OK"})()
        ui_steps.click_button(serial=self.serial, view_to_find={"text":"Set time"})()
        ui_steps.click_button(serial=self.serial, view_to_find={"className":"android.widget.RadialTimePickerView$RadialPickerTouchHelper", "instance":"11"})()
        ui_steps.click_button(serial=self.serial, view_to_find={"className":"android.widget.RadialTimePickerView$RadialPickerTouchHelper", "instance":"11"})()
        ui_steps.click_button(serial=self.serial, view_to_find={"text":"OK"})()
        if self.uidevice(text="Use 24-hour format").right(className="android.widget.Switch").info["text"] == "OFF":
            self.uidevice(text="Use 24-hour format").right(className="android.widget.Switch").click.wait()

    def check_condition(self):
        return True


class time_set(ui_step):

    def __init__(self, wait_time=5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time

    def do(self):
        time_management_with_device_off(serial=self.serial)()
        ui_steps.click_button(serial=self.serial, view_to_find={"text":"Set date"})()
        ui_steps.click_button(serial=self.serial, view_to_find={"resourceId":"android:id/date_picker_header_year"})()
        for _ in range(10):
            self.uidevice(className="android.widget.ListView").scroll.vert.toBeginning()
        self.minimum_available = self.uidevice(className="android.widget.ListView").child(instance="0").info["text"]
        self.uidevice(className="android.widget.ListView").child(instance="0").click.wait()
        while self.uidevice(resourceId="android:id/prev").wait.exists(timeout=self.wait_time):
            self.uidevice(resourceId="android:id/prev").click.wait()
        ui_steps.click_button(serial=self.serial, view_to_find={"text":"OK"})()
        ui_steps.click_button(serial=self.serial, view_to_find={"text":"Set date"})()
        ui_steps.click_button(serial=self.serial, view_to_find={"resourceId":"android:id/date_picker_header_year"})()
        for _ in range(10):
            self.uidevice(className="android.widget.ListView").scroll.vert.toEnd()
        self.maximum_available = self.uidevice(className="android.widget.ListView").child(instance="5").info["text"]
        self.uidevice(className="android.widget.ListView").child(instance="5").click.wait()
        while self.uidevice(resourceId="android:id/next").wait.exists(timeout=self.wait_time):
            self.uidevice(resourceId="android:id/next").click.wait()
        ui_steps.click_button(serial=self.serial, view_to_find={"text":"OK"})()

    def check_condition(self):
        if int(self.maximum_available) - int(self.minimum_available) == 30:
            return True
        return False


class fastboot_unlock_device(ui_step):

    def __init__(self, wait_time = 30000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time
        self.platform_name = fastboot_utils.get_platform_name(serial=self.serial)
        self.device_state = None
        self.change_result = False

    def do(self):
        adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = self.serial)()
        command(command = "getvar device-state > ./temp/files/device_state.txt 2>&1", serial = self.serial)()
        self.device_state = fastboot_utils.get_device_state(file_path = "./temp/files/device_state.txt")
        if self.device_state == "locked":
            if self.platform_name == "bxtp_abl": pass
            if self.platform_name == "gordon_peak":
                command(command = "flashing unlock > ./temp/files/unlock_result.txt 2>&1", serial = self.serial)()
                self.change_result = fastboot_utils.fastboot_command_result(file_name = "./temp/files/unlock_result.txt")
                if self.change_result == False: raise Exception("The test result did not achieve the desired results")
                reboot(serial = self.serial)()
                local_steps.wait_for_adb(serial = self.serial, timeout = 300)()
                time.sleep(10)
                fastboot_utils.push_uiautomator_jar(serial = self.serial)
                if self.uidevice(text = "To start Android, enter your password").wait.exists(timeout = self.wait_time):
                    adb_steps.command(command = "input text 11111", serial = self.serial)()
                    adb_steps.command(command = "input keyevent 66", serial = self.serial)()
                if self.uidevice(text = "RESET PHONE").wait.exists(timeout = self.wait_time):
                    ui_steps.click_button(serial = self.serial, view_to_find = {"text": "RESET PHONE"})()
                    time.sleep(120)
                local_steps.wait_for_adb(serial = self.serial, timeout = 300)()
                time.sleep(10)
                adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = self.serial)()
                command(command = "getvar device-state > ./temp/files/device_state.txt 2>&1", serial = self.serial)()
                self.device_state = fastboot_utils.get_device_state(file_path = "./temp/files/device_state.txt")
        reboot(serial = self.serial)()
        time.sleep(120)
        local_steps.wait_for_adb(serial = self.serial, timeout = 300)()
        time.sleep(10)

    def check_condition(self):
        if self.device_state == "unlocked": return True
        return False


class fastboot_lock_device(ui_step):

    def __init__(self, wait_time = 30000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time
        self.platform_name = fastboot_utils.get_platform_name(serial=self.serial)
        self.device_state = None
        self.change_result = False

    def do(self):
        adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = self.serial)()
        command(command = "getvar device-state > ./temp/files/device_state.txt 2>&1", serial = self.serial)()
        self.device_state = fastboot_utils.get_device_state(file_path = "./temp/files/device_state.txt")
        if self.device_state == "unlocked":
            if self.platform_name == "bxtp_abl": pass
            if self.platform_name == "gordon_peak":
                command(command = "flashing lock > ./temp/files/lock_result.txt 2>&1", serial = self.serial)()
                self.change_result = fastboot_utils.fastboot_command_result(file_name = "./temp/files/lock_result.txt")
                if self.change_result == False: raise Exception("The test result did not achieve the desired results")
                reboot(serial = self.serial)()
                local_steps.wait_for_adb(serial = self.serial, timeout = 300)()
                time.sleep(10)
                fastboot_utils.push_uiautomator_jar(serial = self.serial)
                if self.uidevice(text = "To start Android, enter your password").wait.exists(timeout = self.wait_time):
                    adb_steps.command(command = "input text 11111", serial = self.serial)()
                    adb_steps.command(command = "input keyevent 66", serial = self.serial)()
                if self.uidevice(text = "RESET PHONE").wait.exists(timeout = self.wait_time):
                    ui_steps.click_button(serial = self.serial, view_to_find = {"text": "RESET PHONE"})()
                    time.sleep(120)
                local_steps.wait_for_adb(serial = self.serial, timeout = 300)()
                time.sleep(10)
                adb_steps.reboot(command = "fastboot", reboot_timeout = 300, serial = self.serial)()
                command(command = "getvar device-state > ./temp/files/device_state.txt 2>&1", serial = self.serial)()
                self.device_state = fastboot_utils.get_device_state(file_path = "./temp/files/device_state.txt")
        reboot(serial = self.serial)()
        time.sleep(120)
        local_steps.wait_for_adb(serial = self.serial, timeout = 300)()
        time.sleep(10)

    def check_condition(self):
        if self.device_state == "locked": return True
        return False