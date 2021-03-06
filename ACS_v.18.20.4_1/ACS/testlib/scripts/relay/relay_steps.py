
#!/usr/bin/env python

########################################################################
#
# @filename:    relay_step.py
# @description: relay test step
# @author:      aurel.constantin@intel.com
#
########################################################################

from testlib.scripts.relay.relay_step import relay_step
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.android_step import step as android_step
from testlib.scripts.relay import relay_utils
from testlib.scripts.connections.local import local_utils
from testlib.scripts.connections.local import local_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_steps
from testlib.base.ParallelSteps import ParallelSteps
from testlib.scripts.android.fastboot import fastboot_utils
from testlib.scripts.android.fastboot import fastboot_steps
from testlib.utils.statics.android import statics
import datetime
import re
import time


class power_off_device(relay_step):

    """ description:
            Shuts down the device.

        usage:
            relay_steps.power_off_device(serial=serial,
                                         relay_type = relay_type,
                                         relay_port = relay_port,
                                         power_port = power_port)()

        tags:
            shutdown, relay
    """

    def __init__(self, serial, except_charging=False, timeout = 120, device_info = "", **kwargs):
        relay_step.__init__(self, **kwargs)
        self.serial = serial
        self.timeout = timeout
        self.except_charging = except_charging
        self.device_info = device_info

    def do(self):
        if self.device_info != "broxtonp": self.relay.power_on()
        self.relay.power_off()

    def check_condition(self):
        wait_time = 0
        while local_utils.is_device_connected(self.serial, self.except_charging) and\
              wait_time < self.timeout:
                    time.sleep(2)
                    wait_time += 2

        return not local_utils.is_device_connected(self.serial, self.except_charging)


class long_press_power_shutdown(relay_step):

    """ description:
            Shuts down the device.

        usage:
            relay_steps.power_off_device(serial=serial,
                                         relay_type = relay_type,
                                         relay_port = relay_port,
                                         power_port = power_port)()

        tags:
            shutdown, relay
    """

    def __init__(self, serial, except_charging=False, timeout = 120, wait_ui = True, long_press_time = 15, device_info = "", **kwargs):
        relay_step.__init__(self, **kwargs)
        self.serial = serial
        self.except_charging = except_charging
        self.timeout = timeout
        self.wait_ui = wait_ui
        self.long_press_time = long_press_time
        self.device_info = device_info

    def do(self):
        self.relay.long_press_power_shutdown(long_press_time = self.long_press_time)

    def check_condition(self):
        if self.device_info == "broxtonp":
            wait_time = 0
            while self.serial not in local_utils.get_connected_android_devices()['android'] and\
                  wait_time < self.timeout:
                        time.sleep(2)
                        wait_time += 2
            if self.wait_ui:
                return adb_steps.wait_for_ui_processes(serial = self.serial)()
            return self.serial in local_utils.get_connected_android_devices()['android']


class gracefully_power_off_device(relay_step):

    """ description:
            Shuts down the device.

        usage:
            relay_steps.power_off_device(serial=serial,
                                         relay_type = relay_type,
                                         relay_port = relay_port,
                                         power_port = power_port)()

        tags:
            shutdown, relay
    """

    def __init__(self, serial, timeout = 120, except_charging = False, **kwargs):
        relay_step.__init__(self, **kwargs)
        self.serial = serial
        self.timeout = timeout
        self.except_charging = except_charging

    def do(self):
        self.relay.long_press_power()
        ui_steps.click_button(serial=self.serial, view_to_find={"text": "Power off"}, wait_for_event_occurs=False)()

    def check_condition(self):
        wait_time = 0
        while local_utils.is_device_connected(self.serial, self.except_charging) and\
              wait_time < self.timeout:
                    time.sleep(2)
                    wait_time += 2
        return not local_utils.is_device_connected(self.serial, self.except_charging)


class power_on_device(relay_step):

    """ description:
            Powers up the device.

        usage:
            relay_steps.power_on_device(serial=serial,
                                         relay_type = relay_type,
                                         relay_port = relay_port,
                                         power_port = power_port)()

        tags:
            startup, relay
    """

    def __init__(self, serial, timeout = 120, except_charging = False, **kwargs):
        relay_step.__init__(self, **kwargs)
        self.serial = serial
        self.timeout = timeout
        self.except_charging = except_charging

    def do(self):
        self.relay.power_on()

    def check_condition(self):
        wait_time = 0
        while not local_utils.is_device_connected(self.serial, self.except_charging) and\
              wait_time < self.timeout:
                    time.sleep(2)
                    wait_time += 2

        time.sleep(10)
        return local_utils.is_device_connected(self.serial, self.except_charging)


class reboot_fastboot(relay_step):

    """ description:
            Reboots to fastboot.

        usage:
            relay_steps.reboot_fastboot(serial = serial,
                                                relay_type = relay_type,
                                                relay_port=relay_port,
                                                power_port=power_port,
                                                v_up_port=v_up_port,
                                                v_down_port=v_down_port,
                                                USB_VC_cut_port=USB_VC_cut_port)()

        tags:
            fastboot, android, reboot, relay
    """

    def __init__(self, serial, timeout = 10, **kwargs):
        relay_step.__init__(self, **kwargs)
        self.serial = serial
        self.timeout = timeout

    def do(self):
        # hard boot to fastboot
        self.relay.power_off()
        self.relay.enter_fastboot()

    def check_condition(self):
        wait_time = 0
        while self.serial not in local_utils.get_fastboot_devices() and\
              wait_time < self.timeout:
                    time.sleep(2)
                    wait_time += 2

        return self.serial in local_utils.get_fastboot_devices()


class reboot_main_os(relay_step):

    """ description:
            Reboots to main OS.

        usage:
            relay_steps.reboot_main_os(serial = serial,
                                            relay_type = relay_type,
                                            relay_port=relay_port,
                                            power_port=power_port,
                                            force_reboot = True)()
            - force_reboot: if equals True then the device will be rebooted
                            even if it is already into main OS

        tags:
            reboot, android, relay
    """

    def __init__(self, serial, timeout = 30, force_reboot = False, wait_ui = True, delay_power_on = 0, device_info = "", **kwargs):
        relay_step.__init__(self, **kwargs)
        self.serial = serial
        self.kwargs = kwargs
        self.timeout = timeout
        self.force_reboot = force_reboot
        self.wait_ui = wait_ui
        self.delay_power_on = delay_power_on
        self.device_info = device_info

    def do(self):
        # hard boot to main OS
        if self.serial not in local_utils.get_connected_android_devices()['android'] or self.force_reboot:
            power_off_device(serial = self.serial, except_charging = True, device_info = self.device_info, **self.kwargs)()
            if self.delay_power_on > 0: time.sleep(self.delay_power_on)
            power_on_device(serial = self.serial, except_charging = True, **self.kwargs)()

    def check_condition(self):
        wait_time = 0
        while self.serial not in local_utils.get_connected_android_devices()['android'] and\
              wait_time < self.timeout:
                    time.sleep(2)
                    wait_time += 2
        if self.wait_ui:
            return adb_steps.wait_for_ui_processes(serial = self.serial)()
        return self.serial in local_utils.get_connected_android_devices()['android']


class recovery_reboot(relay_step):

    """ description:
            Reboots to main OS.

        usage:
            relay_steps.recovery_reboot(serial = serial,
                                            mode = "fastboot",
                                            relay_type = relay_type,
                                            relay_port=relay_port,
                                            power_port=power_port,
                                            v_up_port = v_up_port,
                                            v_down_port = v_down_port)()

            - mode: "fastboot" or "android"
        tags:
            reboot, recovery, android, relay
    """

    def __init__(self, serial, mode = "android", menu_position=0,  timeout = 30, **kwargs):
        relay_step.__init__(self, **kwargs)
        self.serial = serial
        self.kwargs = kwargs
        self.timeout = timeout
        self.mode = mode
        self.menu_position = menu_position

    def do(self):
        # hard boot from recovery  to main OS
        relay_utils.select_ros_menu_item(self.relay, mode=self.menu_position)
        self.relay.press_power()

    def check_condition(self):
        wait_time = 0
        while self.serial not in local_utils.get_connected_android_devices()[self.mode] and\
              wait_time < self.timeout:
                    time.sleep(2)
                    wait_time += 2

        return self.serial in local_utils.get_connected_android_devices()[self.mode]


class recovery_factory_reset(relay_step):

    """ description:
            Reset to factory defaults from recovery OS. At the end the system
            remains in ROS.

        usage:
            relay_steps.recovery_factory_reset(serial = serial,
                                            relay_type = relay_type,
                                            relay_port=relay_port,
                                            power_port=power_port)()

        tags:
            reboot, recovery, android, relay
    """

    def __init__(self, serial, timeout = 600, **kwargs):
        relay_step.__init__(self, **kwargs)
        self.serial = serial
        self.kwargs = kwargs
        self.timeout = timeout
        self.mode = "factory_reset"

    def do(self):
        # hard boot from recovery  to main OS
        relay_utils.select_ros_menu_item(self.relay,
                    mode = statics.Device(serial=serial).ros_menu_entry[option])
        self.relay.press_power()
        time.sleep(1)
        # accept the factory reset
        self.relay.press_volume_down()
        self.relay.press_power()

    def check_condition(self):
        wait_time = 0
        while self.serial not in local_utils.get_connected_android_devices()["recovery"] and\
              wait_time < self.timeout:
                    time.sleep(2)
                    wait_time += 2

        return self.serial in local_utils.get_connected_android_devices()["recovery"]


class reboot_safe_mode(relay_step):

    """ description:
            Reboots into safe mode.

        usage:
            relay_steps.reboot_safe_mode(serial = serial,
                                            mode = "android",
                                            relay_type = relay_type,
                                            relay_port=relay_port,
                                            power_port=power_port)()

            - mode: "fastboot" or "android"
        tags:
            reboot, recovery, android, relay
    """

    def __init__(self, serial, app_to_find, mode="android", timeout=30, **kwargs):
        relay_step.__init__(self, **kwargs)
        self.serial = serial
        self.mode = mode
        self.timeout = timeout
        self.app_to_find = app_to_find
        self.kwargs = kwargs
        self.set_errorm("", "Cannot reboot in safe mode - device {0}".format(self.serial))
        self.set_passm("Rebooted in safe mode - device {0}".format(self.serial))

    def do(self):
        # Long press on power button
        self.relay.long_press_power()

        # Long press on power to enable reboot safe mode prompt and select OK
        ui_steps.long_click(serial=self.serial,
                            view_to_find={"resourceId": "android:id/contentPanel"},
                            view_to_check={"text": "Reboot to safe mode"})()
        ui_steps.click_button(serial=self.serial,
                              view_to_find={"text": "OK"})()

        # Wait for the device to reboot
        local_steps.wait_until_disconnected(serial=self.serial)()
        local_steps.wait_for_adb(serial=self.serial)()
        adb_steps.wait_for_ui(serial=self.serial)()

    def check_condition(self):
        self.step_data = ui_steps.find_app_from_allapps(serial=self.serial,
                                                        view_to_find={"text": self.app_to_find},
                                                        presence=False)()
        return self.step_data


class reboot_safe_mode_magic_key(relay_step):

    """ description:
            Enters safe mode when booting by pressing volume down button.

        usage:
            relay_steps.reboot_safe_mode_magic_key(serial = serial,
                                            relay_type = relay_type,
                                            relay_port=relay_port,
                                            power_port=power_port,
                                            v_up_port=v_up_port
                                            v_down_port=v_down_port)()

        tags:
            reboot, android, relay, safe_mode, power_off, power_on
    """

    def __init__(self, serial, app_to_find, **kwargs):
        relay_step.__init__(self, **kwargs)
        self.serial = serial
        self.app_to_find = app_to_find
        self.kwargs = kwargs
        self.set_errorm("", "Cannot reboot in safe mode - device {0}".format(self.serial))
        self.set_passm("Rebooted in safe mode - device {0}".format(self.serial))

    def do(self):
        # Turn off the device
        power_off_device(serial=self.serial,
                         except_charging=True,
                         **self.kwargs)()

        # Power on the device
        power_on_device(serial=self.serial,
                        **self.kwargs)()

        # Press volume down
        self.relay.relay.on(self.relay.v_down_port)

        # Wait for UI processes
        adb_steps.wait_for_ui_processes(serial=self.serial)()

        # Release volume down button
        self.relay.relay.off(self.relay.v_down_port)

    def check_condition(self):
        # Unlock device
        ui_steps.wake_up_device(serial=self.serial)()
        ui_steps.unlock_device(serial=self.serial)()

        # Check if the app is NOT displayed in normal mode
        ui_steps.find_app_from_allapps(serial=self.serial,
                                       view_to_find={"text": self.app_to_find},
                                       presence=False)()
        return True


class connect_disconnect_usb(relay_step):

    """ description:


        usage:
            relay_steps.disconnect_usb(serial=serial,
                                       connect=True,
                                       USB_VC_cut_port = USB_VC_cut_port)()

        tags:
            usb, relay
    """

    def __init__(self, serial, connect, timeout = 30, **kwargs):
        relay_step.__init__(self, **kwargs)
        self.serial = serial
        self.timeout = timeout
        self.connect = connect

    def do(self):
        if self.connect:
            self.relay.uncut_usb_vc()
        else:
            self.relay.cut_usb_vc()

    def check_condition(self):
        wait_time = 0
        result = False
        while wait_time < self.timeout:
            if self.connect:
                time.sleep(10)
                if local_utils.is_device_connected(self.serial):
                    result = True
                    break
            else:
                if not local_utils.is_device_connected(self.serial):
                    result = True
                    break
            time.sleep(2)
            wait_time += 2

        return result


class take_screenshot(relay_step, adb_step):
    """ description:
            Takes screenshot using power button and volume button.

        usage:
            relay_steps.take_screenshot(serial = serial,
                                            relay_type = relay_type,
                                            relay_port=relay_port,
                                            power_port=power_port,
                                            v_up_port=v_up_port
                                            v_down_port=v_down_port)()

        tags:
            screenshot, android, relay
    """

    def __init__(self, serial, screenshots_folder, timeout=30, **kwargs):
        adb_step.__init__(self, serial=serial, **kwargs)
        relay_step.__init__(self, **kwargs)
        self.serial = serial
        self.screenshots_folder = screenshots_folder
        self.timeout = timeout
        self.kwargs = kwargs
        self.result = True
        self.step_data = None
        self.set_errorm("", "Cannot take screenshot - device {0}".format(self.serial))
        self.set_passm("Screenshot taken successfully - device {0}".format(self.serial))

    def do(self):
        # Delete the contents of the "/sdcard/Pictures/Screenshots" folder on the DUT
        if adb_utils.folder_exists(serial=self.serial, folder=self.screenshots_folder):
            adb_steps.delete_folder_content(serial=self.serial, folder=self.screenshots_folder)()

        # Take screeenshot
        self.relay.take_screenshot()

        # Check if the screenshot exists
        screenshot_date = datetime.datetime.strftime(datetime.datetime.now(), "%Y%m%d")
        # List the contents of the screenshot folder
        output = adb_utils.wait_for_file_with_text(text_contained=screenshot_date,
                                                      dir_to_search=self.screenshots_folder,
                                                      serial=self.serial)
        if output is None:
            self.result = False
            self.set_errorm("", "Failed when waiting for file with text {0} - device {1}".format(screenshot_date,
                                                                                                 self.serial))
            return

        # Check if the file has the correct name (screenshot_date in name)
        if not re.search("\w+{0}".format(screenshot_date), output):
            self.result = False

        # Get the file size
        self.step_data = int(re.findall("\s(\d+)\s", output)[0])

    def check_condition(self):
        # The correct file should exist and the size should not be 0
        return self.result and self.step_data > 0


class choose_fastboot_menu(relay_step):

    """ description:
            Selects the provided menu option from fastboot.

        usage:
            relay_steps.choose_fastboot_menu(serial = serial,
                                            option = "normal_boot",
                                            relay_type = relay_type,
                                            relay_port=relay_port,
                                            power_port=power_port,
                                            v_up_port = v_up_port,
                                            v_down_port = v_down_port)()

            - option:
                 - normal_boot
                 - power_off
                 - bootloader
                 - recovery
                 - reboot
        tags:
            reboot, fastboot, android, relay
    """

    def __init__(self, serial, option=None, menu_position=None, timeout = 60, **kwargs):
        relay_step.__init__(self, **kwargs)
        self.serial = serial
        self.kwargs = kwargs
        self.timeout = timeout
        self.option = option
        if not menu_position:
            self.menu_position = statics.Device(serial=self.serial).fastboot_menu_entry[self.option]
        else:
            self.menu_position = menu_position

    def do(self):
        # choose the option from bootloader
        time.sleep(1) # sleep to give time the getvar from statics.Device to finish
                      # the animation
        relay_utils.select_fastboot_menu_item(self.relay, self.menu_position)
        time.sleep(0.5)
        self.relay.press_power()

    def check_condition(self):
        wait_time = 0
        if self.option in ["normal_boot", "reboot"]:
            device_state = "android"
        elif self.option == "power_off":
            device_state = "charge_os"
            return True
        elif self.option == "bootloader":
            device_state = "fastboot"
            time.sleep(3)
        elif self.option == "recovery":
            device_state = "recovery"
        else:
            self.set_errorm("", "Invalid menu option {0}".format(self.option))
            return False

        while self.serial not in local_utils.get_connected_android_devices()[device_state] and\
              wait_time < self.timeout:
                    time.sleep(2)
                    wait_time += 2

        return self.serial in local_utils.get_connected_android_devices()[device_state]

class choose_crashmode_menu(choose_fastboot_menu):

    """ description:
            Selects the provided menu option from fastboot.

        usage:
            relay_steps.choose_crashmode_menu(serial = serial,
                                            menu_position=3,
                                            relay_type = relay_type,
                                            relay_port=relay_port,
                                            power_port=power_port,
                                            v_up_port = v_up_port,
                                            v_down_port = v_down_port)()

            - option:
                 - normal_boot
                 - power_off
                 - bootloader
                 - recovery
                 - reboot
        tags:
            reboot, crashmode, android, relay
    """

    def __init__(self, serial, option=None, menu_position=None, timeout = 60, **kwargs):
        choose_fastboot_menu.__init__(self, serial, option, timeout = 60, **kwargs)

    def do(self):
        # overwrite the do method
        relay_utils.select_crashmode_menu_item(self.relay, self.menu_position)
        self.relay.press_power()

    # check method is the one from inherited class
    #def check_condition(self):



class change_state(relay_step, android_step):

    """ description:
            Changes bootloader state if the oem_unlock_enabled is set to yes.
            Else it will attempt to unlock and check for message in output.

        usage:
            relay_steps.change_state(serial=serial,
                             dessert=dessert,
                             unlock_bootloader=lock_state,
                             relay_type=relay_type,
                             relay_port=relay_port,
                             power_port=power_port,
                             v_down_port=v_down_port,
                             v_up_port=v_up_port)()

        tags:
            fastboot, android, bootloader, unlock, relay
    """

    def __init__(self, unlock_bootloader, dessert, oem_unlock_enabled="yes", **kwargs):
        android_step.__init__(self, **kwargs)
        relay_step.__init__(self, **kwargs)
        self.unlock_bootloader = unlock_bootloader
        self.dessert = dessert
        self.oem_unlock_enabled = oem_unlock_enabled

        # Define the unlock command depending on Android version
        if self.dessert == "L":
            self.unlock_cmd = "oem"
        elif self.dessert >= "M":
            self.unlock_cmd = "flashing"

        # Define the state command
        if self.unlock_bootloader == "yes":
            self.state_cmd = "unlock"
        elif self.unlock_bootloader == "no":
            self.state_cmd = "lock"

        if self.oem_unlock_enabled == "yes":
            self.string_to_check = "finished. total time:"
            self.set_errorm("", "Could not change state to \"{0}\"".format(self.unlock_bootloader))
            self.set_passm("State Changed to \"{0}\"".format(self.unlock_bootloader))
        else:
            self.string_to_check = "Unlock is not allowed"
            self.set_errorm("", "State changed to \"{0}\", even if OEM unlock is disabled".format(self.unlock_bootloader))
            self.set_passm("State not changed to \"{0}\". OEM unlock is disabled".format(self.unlock_bootloader))


    def do(self):

        #use_control_process=False
        psteps = ParallelSteps(use_control_process=False)

        # Accept lock/ unlock action using volume and power keys
        def accept_lock_unlock():
            time.sleep(1)
            self.relay.press_volume_down()
            time.sleep(1)
            self.relay.press_power()

        current_state = fastboot_utils.get_var(serial=self.serial,
                                               var="unlocked")

        if self.unlock_bootloader not in str(current_state):
            # Run the lock/ unlock command in a parallel step
            step_id_lock = psteps.add_step(fastboot_steps.command,
                                           serial=self.serial,
                                           command=self.unlock_cmd + " " + self.state_cmd,
                                           stderr_grep=self.string_to_check,
                                           timeout=120000)
            time.sleep(2)
            # Run accept lock unlock if the operations is allowed
            if self.oem_unlock_enabled == "yes":
                accept_lock_unlock()

            # Interpret the parallel step result
            psteps.interpret_step(step_id_lock)

    def check_condition(self):
        # If the OEM unlocking is disabled, pass if the state is unchanged
        if self.oem_unlock_enabled != "yes":
            return self.unlock_bootloader != fastboot_utils.get_var(serial=self.serial,
                                                                    var="unlocked")
        # If the OEM unlocking is enabled, pass if tha state
        return self.unlock_bootloader == fastboot_utils.get_var(serial=self.serial,
                                                                var="unlocked")


class create_panic_and_check_state(relay_step, android_step):

    """ description:
            Creates kernel panics (or stops the watchdog daemon) watchdog_counter_max times.
            It then checks the device boot state (MOS, crash mode or fastboot), depending on wait_for_state value

        usage:
            create_panic_and_check_state(serial=serial,
                                         create_crash_mode=panic/watchdog,
                                         wait_for_state=state)()

        state = [android, crashmode, fastboot]

        tags:
            adb, panic, watchdog, fastboot, crashmode, android
    """
    def __init__(self, create_crash_mode, wait_for_state, use_combo_button, **kwargs):
        self.create_crash_mode = create_crash_mode
        self.wait_for_state = wait_for_state
        self.panic_command = "\"echo e > /proc/sysrq-trigger\""
        self.watchdog_process_name = "/sbin/watchdogd"
        self.wait_for_fastboot_timeout = 70
        self.use_combo_button=use_combo_button
        relay_step.__init__(self, **kwargs)
        android_step.__init__(self, **kwargs)

    def do(self):
        # Create panic
        adb_steps.root_connect_device(serial=self.serial)()
        if self.create_crash_mode == "panic":
            # Using local_steps to run the command because adb_steps.command() does not create kernel panic
            local_steps.command("adb -s {0} shell {1}".format(self.serial, self.panic_command))()

            # Wait for the device to disconnect
            local_steps.wait_until_disconnected(serial=self.serial)()
            if self.use_combo_button==False:
                print "======================wait 120s======================="
                time.sleep(120)
        else:
            adb_steps.kill_process(serial=self.serial,
                                   process_name=self.watchdog_process_name,
                                   with_reboot=True,
                                   reboot_timeout=60)()

        # Wait for a device state
        if self.wait_for_state == "fastboot":
            # Set the pass and error messages
            self.set_errorm("", "The device with serial {0} is not in fastboot".format(self.serial))
            self.set_passm("The device with serial {0} is in fastboot".format(self.serial))

            # If the expected state is fastboot, we need to press the volume down button after the panic is created
            # This will boot the device in fastboot
            # Create a parallel step
            # use_control_process=False
            psteps = ParallelSteps(use_control_process=False)

            step_id_combo = psteps.add_step(local_steps.wait_for_fastboot,
                                            serial=self.serial,
                                            timeout=self.wait_for_fastboot_timeout)

            self.relay.relay.on(self.relay.v_down_port)
            time.sleep(self.wait_for_fastboot_timeout-10)
            self.relay.relay.off(self.relay.v_down_port)

            # Interpret the parallel step result
            psteps.interpret_step(step_id_combo)

        elif self.wait_for_state == "android":
            # Set the pass and error messages
            self.set_errorm("", "The device with serial {0} is not in MOS".format(self.serial))
            self.set_passm("The device with serial {0} is in MOS".format(self.serial))

            # Wait for adb connection
            local_steps.wait_for_adb(serial=self.serial)()

            # Wait for the UI processes
            adb_steps.wait_for_ui_processes(serial=self.serial)()
        elif self.wait_for_state == "crashmode":
            # Set the pass and  error messages
            self.set_errorm("", "The device with serial {0} is not in crash mode".format(self.serial))
            self.set_passm("The device with serial {0} is in crash mode".format(self.serial))

            # Wait for crashmode
            local_steps.wait_for_crashmode(serial=self.serial,
                                           timeout=60)()

    def check_condition(self):
        # Check performed in do()
        return True
