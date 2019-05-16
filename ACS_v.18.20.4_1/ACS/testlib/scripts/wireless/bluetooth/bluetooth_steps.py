#!/usr/bin/env python

#######################################################################
#
# @description: Bluetooth test steps
# @author:      adrian.palko@intel.com
# @author:      lucia.huru@intel.com
# @author:      mihaela.maracine@intel.com
#
#######################################################################

import re
import subprocess
import time
import os.path
import traceback

from testlib.base.base_step import step as base_step
from testlib.scripts.wireless.bluetooth.bt_step import Step as BtStep
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.wireless.bluetooth import bluetooth_utils


class GetAndroidVersion(BtStep):
    """ Description:
            Gets Android version via adb command (float type)
        Usage:
            bluetooth_steps.GetAndroidVersion(serial=serial)()
    """

    def __init__(self, **kwargs):
        """
        :param kwargs: serial, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.step_data = None
        self.set_errorm("Get version", "Could not obtain Android version")

    def do(self):
        try:
            self.step_data = self.adb_connection.cmd('shell getprop ro.build.version.release').communicate()[
                0].decode("utf-8").strip()
        except Exception, e:
            self.set_errorm("Get version", e.message)

    def check_condition(self):
        """
        :return: True if android version is successfully obtained, False if not. Version is saved in step_data as string
        """
        if self.step_data:
            self.set_passm("Android version " + str(self.step_data))
            return True
        else:
            return False


class ClickBluetoothSwitch(BtStep):
    """ Description:
            Only makes sure that the Bluetooth switch has the required state.
            Furthermore, if you call this function with check_if_already=True,
            if BT switch already has the required state, it returns failure.
            Call this from the Bluetooth Settings activity
        Usage:
            bluetooth_steps.ClickBluetoothSwitch(
                        state = "ON", check_if_already=False)()
    """

    def __init__(self, state="ON", check_if_already=False, **kwargs):
        """
        :param state: "ON" for on state required, OFF for off state required
        :param check_if_already: True to fail if already has the required state, False otherwise
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.state = state
        # In some platform, state text is 'On' & 'Off', so covert to 'checked'
        self.checked = True if state == "ON" else False
        self.check_if_already = check_if_already
        self.switch = self.uidevice(className="android.widget.Switch", enabled=True)
        self.step_data = True
        self.set_passm("BT set to " + self.state)

    def do(self):
        try:
            # check if switch is present
            if not self.switch.wait.exists(timeout=self.timeout):
                raise Exception("No BT switch found")
            if not self.switch.info["checked"] == self.checked:
                self.switch.click()
            else:
                # check if already has required state
                if self.check_if_already:
                    raise Exception("BT already has " + self.state + " state")
                self.set_passm("BT already set to " + self.state)
        except Exception, e:
            self.set_errorm("Set BT to " + self.state, e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if required state was set, False if not
        """
        if self.step_data:
            # wait for switch transition
            if not self.switch.wait.exists(timeout=self.timeout):
                self.set_errorm("Set BT to " + self.state, "BT state not set to " + self.state + " correctly")
                self.step_data = False
            else:
                # check if it has required state
                if not self.uidevice(className="android.widget.Switch",
                                     enabled=True,
                                     checked=self.checked).wait.exists(
                                         timeout=self.timeout):
                    self.set_errorm("Set BT to " + self.state, "BT state not set to " + self.state)
                    self.step_data = False
        return self.step_data


class OpenBluetoothSettings(BtStep):
    """ Description:
            Opens the Bluetooth activity from settings, either from all
            apps menu, or by sending an intent. Call this from the Home
            screen if use_intent=False
        Usage:
            bluetooth_steps.OpenBluetoothSettings(serial=serial, use_intent=False, version=version)()
    """

    def __init__(self, use_intent=False, **kwargs):
        """
        :param use_intent: True to open from the home screen, False to use BT settings launch intent
        :param kwargs: serial, version, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.use_intent = use_intent
        self.step_data = True
        # part of logging message
        if self.use_intent:
            self.message_str = "with intent"
        else:
            self.message_str = "from menu"
        self.set_passm("BT settings opened " + self.message_str)

    def do(self):
        try:
            if self.use_intent:
                # execute start BT command if use_intent=True
                cmd_launch_bt_settings = "shell am start -a android.settings.BLUETOOTH_SETTINGS -p com.android.settings"
                self.adb_connection.cmd(cmd_launch_bt_settings).wait()
            else:
                # open settings from all apps menu
                #apps_button = self.uidevice(description="Apps")
                #if not apps_button.wait.exists(timeout=self.timeout):
                #    raise Exception("All apps button not found")
                #apps_button.click()
                #apps_menu = self.uidevice(
                #        resourceId="com.google.android.googlequicksearchbox
                # :id/apps_list_view")
                #if not apps_menu.wait.exists(timeout=self.timeout):
                #    raise Exception("All apps menu was not opened")
                # click on Settings icon
                #if not apps_menu.scroll.vert.to(text="Settings"):
                #    raise Exception("Settings app not found in All apps menu")
                #self.uidevice(text="Settings").click()
                #if not self.uidevice(
                # packageName="com.android.settings").wait.exists(timeout=self.timeout):
                #    raise Exception("Settings app not opened")
                # press Bluetooth option after scrolling to it in the menu
                #settings_list = self.uidevice(
                #    resourceId="com.android.settings:id/dashboard")
                #if not settings_list.wait.exists(timeout=self.timeout):
                #    raise Exception("Settings list was not found")
                #if not settings_list.scroll.to(text="Bluetooth"):
                #    raise Exception("Bluetooth option not found")
                #self.uidevice(text="Bluetooth").click()
                ui_steps.open_settings(serial=self.serial)()
                ui_steps.click_button_if_exists(serial=self.serial,
                   wait_time=5000,view_to_find={"text": "Connected devices"})()
                ui_steps.click_button_with_scroll(serial=self.serial,
                                  view_to_find={"text": "Bluetooth"})()
        except Exception, e:
            self.set_errorm("Open " + self.message_str, e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if BT settings list was launched, False if not
        """
        if self.step_data:
            self.set_errorm("Open " + self.message_str, "BT settings was not opened")
            # wait for the BT activity to open
            self.step_data = self.uidevice(text="Bluetooth").wait.exists(timeout=self.timeout)
        return self.step_data


class CheckBtVisibility(BtStep):
    """ Description:
            Checks if the device is visible. Call this from the BT settings list,
            with BT ON
        Usage:
            bluetooth_steps.CheckBtVisibility(serial=serial, version=version)()
    """

    def __init__(self, **kwargs):
        """
        :param kwargs: serial, version, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.step_data = True
        self.set_passm("DUT is visible")

    def do(self):
        """"       try:
            if self.version.startswith("5.") or self.version.startswith("6.0"):
                # LLP, M versions
                bt_list = self.uidevice(resourceId="android:id/list")
                if not bt_list.wait.exists(timeout=self.timeout):
                    raise Exception("BT devices list was not found")
                bt_list.scroll.to(textContains=" is visible")
            else:
                # N version
                bt_list = self.uidevice(resourceId="com.android.settings:id/list")
                if not bt_list.wait.exists(timeout=self.timeout):
                    raise Exception("BT devices list was not found")
                bt_list.scroll.to(textContains=" is visible")
        except Exception, e:
            self.set_errorm("Check if visible", e.message)
            self.step_data = False
        """
        if not ui_steps.wait_for_view_common(serial=self.serial,
                                         view_to_find={"textContains": "is visible"}, optional=True)():
            self.step_data = ui_steps.wait_for_view_common(serial=self.serial,
                                                       view_to_find={"textMatches": ".*?(v|V)isible.*?"})()

    def check_condition(self):
        """
        :return: True if BT is visible message was found on the screen, False if not
        """
        if not self.step_data:
            self.set_errorm("Check if visible",
                            "Check condition for Visibility has failed, 'visible' text can not" +
                            " be found on the screen")
            #self.step_data = self.uidevice(textContains=" is visible").wait.exists(timeout=self.timeout)
        return self.step_data


class WaitBtScanning(BtStep):
    """ Description:
            Makes sure that the BT scanning progress is finished, by waiting
            for progress bar to be gone. Call this from BT settings list,
            with BT on
        Usage:
            bluetooth_steps.WaitBtScanning(serial=serial,
                                timeout_appear=5000, time_to_wait=60000, version=version)()
    """

    def __init__(self, timeout_appear=5000, time_to_wait=60000, **kwargs):
        """
        :param timeout_appear: time to wait till the scanning progress bar appears
        :param time_to_wait: time to wait till the scanning progress bar is gone
        :param kwargs: serial, version, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.timeout_appear = timeout_appear
        self.time_to_wait = time_to_wait
        if self.version.startswith("5.") or self.version.startswith("6.0"):
            # LLP, M versions
            self.bt_list = self.uidevice(resourceId="android:id/list")
        else:
            # N version
            self.bt_list = self.uidevice(resourceId="com.android.settings:id/list")
        self.step_data = True

    def do(self):
        if self.device_info.dessert < "O":
            try:
                if not self.bt_list.wait.exists(timeout=self.timeout_appear):
                    raise Exception("BT devices list was not found")
                # scroll here to reveal scanning progressbar
                if not self.bt_list.scroll.to(text="Available devices"):
                    raise Exception("Available devices title was not found in BT list")
            except Exception, e:
                self.set_errorm("Wait scan finish", e.message)
                self.step_data = False
        else:
            # Fixme This needs to updated once Indefinite scanning is fixed
            # in Android IVI O
            # Below 15 second is given based on Bt search timeout
            time.sleep(15)

    def check_condition(self):
        """
        :return: True if BT scanning progress was finished after timeout reached, False if not
        """
        if self.device_info.dessert < "O" and self.step_data:
            progress_bar = self.uidevice(resourceId="com.android.settings:id/scanning_progress")
            if progress_bar.wait.exists(timeout=self.timeout_appear):
                self.set_passm("Scanning progress finished")
                self.set_errorm("Wait scan finish", "Timeout reached, still scanning")
                self.step_data = progress_bar.wait.gone(timeout=self.time_to_wait)
            else:
                self.set_passm("Scanning progress already finished")
                self.step_data = True
            return self.step_data


class GetBtMac(BtStep):
    """ Description:
            Get BT Address Mac via adb command
        Usage:
            bluetooth_steps.GetBtMac(serial=serial)()
    """

    def __init__(self, **kwargs):
        """
        :param kwargs: serial, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.step_data = None
        self.set_errorm("Obtain BT MAC", "Could not obtain BT MAC address")

    def do(self):
        try:
            self.step_data = self.adb_connection.cmd('shell settings get secure bluetooth_address').communicate()[
                0].decode("utf-8").strip()
        except Exception, e:
            self.set_errorm("Obtain BT MAC", e.message)

    def check_condition(self):
        """
        :return: True if bt mac was found, False if not. Note that the mac is saved in step_data
        """
        if self.step_data:
            self.set_passm("BT MAC address " + str(self.step_data))
            return True
        else:
            return False


class BtChangeDeviceName(BtStep):
    """ Description:
            Replaces the name of the devices with the given name, if not
            already named with the given name. If there is not any character
            given in the name, it validates that Rename button from the
            pop-up is disabled. Call this from the BT settings list, with
            BT ON
        Usage:
            bluetooth_steps.BtChangeDeviceName(serial=serial, name = "", version=version)()
    """

    def __init__(self, name="", **kwargs):
        """
        :param name: name to be set; if empty, it checks if the Rename button is disabled
        :param kwargs: serial, version, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.name = name
        self.step_data = True
        if self.version.startswith("5.") or self.version.startswith("6.0"):
            # LLP, M versions
            self.bt_list = self.uidevice(resourceId="android:id/list")
        else:
            # N version
            self.bt_list = self.uidevice(resourceId="com.android.settings:id/list")

    def do(self):
        try:
            if self.version.startswith("5.") or self.version.startswith("6.0"):
                # LLP, M versions
                if not self.bt_list.wait.exists(timeout=self.timeout):
                    raise Exception("BT devices list was not found")
                if not self.name == '':
                    # check if BT does not already have the given name
                    if not self.bt_list.scroll.to(textContains="is visible to nearby devices"):
                        raise Exception("BT name was not found down of the list")
                    bt_check_object = self.uidevice(textContains="is visible to nearby devices")
                    condition = bt_check_object.info["text"].startswith(self.name + " is visible to nearby devices")
                else:
                    # if empty name is given, do not need to check if not already
                    condition = False
                # if BT does not already have given name, rename it
                if not condition:
                    # open Rename pop-up
                    menu_button = self.uidevice(description="More options")
                    if not menu_button.wait.exists(timeout=self.timeout):
                        raise Exception("More options button in BT settings not found")
                    menu_button.click()
                    rename_button = self.uidevice(textContains="Rename ")
                    if not rename_button.wait.exists(timeout=self.timeout):
                        raise Exception("Rename option from the menu not found")
                    rename_button.click()
                    if not self.uidevice(resourceId="android:id/alertTitle", text="Rename this device").wait.exists(
                            timeout=self.timeout):
                        raise Exception("Rename DUT alert title not opened")
                    # replace name
                    rename_edit_text = self.uidevice(className="android.widget.EditText")
                    '''if not rename_edit_text.wait.exists(timeout=self.timeout):
                        raise Exception("Rename Edit text not found")
                    rename_edit_text.set_text(self.name)
                    # force a small delay due to window transition
                    time.sleep(1)
                    rename_button = self.uidevice(text="Rename")
                    if not rename_button.wait.exists(timeout=self.timeout):
                        raise Exception("Rename button from pop-up not found")
                        '''
                    ui_steps.edit_text(view_to_find={"className":"android.widget.EditText"}, value=self.name,
                                       serial=self.serial)()
                    rename_button = self.uidevice(text="Rename")
                    # if given name is empty, check the status of Rename button and return to the BT list
                    if self.name == '':
                        if rename_edit_text.text:
                            raise Exception("Error when clearing old BT name, not empty")
                        if rename_button.enabled:
                            raise Exception("Rename button in popup not disabled when empty name")
                        cancel_button = self.uidevice(text="Cancel")
                        if not cancel_button.wait.exists(timeout=self.timeout):
                            raise Exception("Cancel button not found in Rename popup when empty name")
                        cancel_button.click()
                        if not self.bt_list.wait.exists(timeout=self.timeout):
                            raise Exception("BT devices list not reached after cancel rename BT with empty name")
                        self.set_passm("Rename button disabled when empty name")
                    # if given name is not empty, rename it
                    else:
                        rename_button.click()
                        if not self.bt_list.wait.exists(timeout=self.timeout):
                            raise Exception("BT devices list not reached after renaming BT")
                        if not self.bt_list.scroll.to(textContains="is visible to nearby devices"):
                            raise Exception("BT name was not found down of the list after rename")
                        bt_check_object = self.uidevice(textContains="is visible to nearby devices")
                        if not bt_check_object.info["text"].startswith(self.name + " is visible to nearby devices"):
                            raise Exception("Found: " + bt_check_object.info["text"] + " instead of " + self.name)
                        self.set_passm("Device renamed: " + self.name)
                # else pass, and write in the logs that device is already renamed
                else:
                    self.set_passm("Device already named: " + self.name)

            elif self.version.startswith("7."):
                # N version
                if not self.bt_list.wait.exists(timeout=self.timeout):
                    raise Exception("BT devices list was not found")
                if not self.name == '':
                    # check if BT does not already have the given name
                    if not self.bt_list.scroll.to(textContains="is visible to nearby devices"):
                        raise Exception("BT name was not found down of the list")
                    bt_check_object = self.uidevice(textContains="is visible to nearby devices")
                    condition = bt_check_object.info["text"].startswith(self.name + " is visible to nearby devices")
                else:
                    # if empty name is given, do not need to check if not already
                    condition = False
                # if BT does not already have given name, rename it
                if not condition:
                    # open Rename pop-up
                    menu_button = self.uidevice(description="More options")
                    if not menu_button.wait.exists(timeout=self.timeout):
                        raise Exception("More options button in BT settings not found")
                    menu_button.click()
                    rename_button = self.uidevice(textContains="Rename ")
                    if not rename_button.wait.exists(timeout=self.timeout):
                        raise Exception("Rename option from the menu not found")
                    rename_button.click()
                    if not self.uidevice(resourceId="android:id/alertTitle", text="Rename this device").wait.exists(
                            timeout=self.timeout):
                        raise Exception("Rename DUT alert title not opened")
                    # force a small delay due to window transition and close keyboard
                    time.sleep(1)
                    if "mInputShown=true" in self.adb_connection.cmd("shell dumpsys input_method").communicate()[
                        0].decode("utf-8"):
                        self.uidevice.press.back()
                        time.sleep(1)
                    # replace name
                    rename_edit_text = self.uidevice(className="android.widget.EditText")
                    '''if not rename_edit_text.wait.exists(timeout=self.timeout):
                        raise Exception("Rename Edit text not found")
                    rename_edit_text.set_text(self.name)
                    # force a small delay due to window transition and close keyboard
                    time.sleep(1)
                    if "mInputShown=true" in self.adb_connection.cmd("shell dumpsys input_method").communicate()[
                        0].decode("utf-8"):
                        self.uidevice.press.back()
                        time.sleep(1)
                        '''
                    ui_steps.edit_text(view_to_find={"className":"android.widget.EditText"}, value=self.name,
                                       serial=self.serial)()
                    rename_button = self.uidevice(text="RENAME")
                    if not rename_button.wait.exists(timeout=self.timeout):
                        raise Exception("Rename button from pop-up not found")
                    # if given name is empty, check the status of Rename button and return to the BT list
                    if self.name == '':
                        if rename_edit_text.text:
                            raise Exception("Error when clearing old BT name, not empty")
                        if rename_button.enabled:
                            raise Exception("Rename button in popup not disabled when empty name")
                        cancel_button = self.uidevice(text="CANCEL")
                        if not cancel_button.wait.exists(timeout=self.timeout):
                            raise Exception("Cancel button not found in Rename popup when empty name")
                        cancel_button.click()
                        if not self.bt_list.wait.exists(timeout=self.timeout):
                            raise Exception("BT devices list not reached after cancel rename BT with empty name")
                        self.set_passm("Rename button disabled when empty name")
                    # if given name is not empty, rename it
                    else:
                        rename_button.click()
                        if not self.bt_list.wait.exists(timeout=self.timeout):
                            raise Exception("BT devices list not reached after renaming BT")
                        if not self.bt_list.scroll.to(textContains="is visible to nearby devices"):
                            raise Exception("BT name was not found down of the list after rename")
                        bt_check_object = self.uidevice(textContains="is visible to nearby devices")
                        if not bt_check_object.info["text"].startswith(self.name + " is visible to nearby devices"):
                            raise Exception("Found: " + bt_check_object.info["text"] + " instead of " + self.name)
                        self.set_passm("Device renamed: " + self.name)
                # else pass, and write in the logs that device is already renamed
                else:
                    self.set_passm("Device already named: " + self.name)
            else:
                # O-dessert version
                if not self.name == '':
                    # check if BT does not already have the given name
                    if not self.bt_list.scroll.to(textContains="visible"):
                        raise Exception("BT name was not found down of the list")
                    bt_check_object = self.uidevice(textContains="Visible as")
                    condition = self.name in bt_check_object.info["text"]
                else:
                    # if empty name is given, do not need to check if not already
                    condition = False
                # if BT does not already have given name, rename it
                if not condition:
                    ui_steps.click_button_common(serial=self.serial,
                                                 view_to_find={"textContains": "Device name"})()
                    if not self.uidevice(resourceId="android:id/alertTitle", text="Rename this device").wait.exists(
                            timeout=self.timeout):
                        raise Exception("Rename DUT alert title not opened")
                    # force a small delay due to window transition and close keyboard
                    time.sleep(1)
                    if "mInputShown=true" in self.adb_connection.cmd("shell dumpsys input_method").communicate()[
                        0].decode("utf-8"):
                        self.uidevice.press.back()
                        time.sleep(1)
                    # replace name
                    rename_edit_text = self.uidevice(className="android.widget.EditText")
                    ui_steps.edit_text(view_to_find={"className": "android.widget.EditText"}, value=self.name,
                                       serial=self.serial)()
                    rename_button = self.uidevice(text="RENAME")
                    if not rename_button.wait.exists(timeout=self.timeout):
                        raise Exception("Rename button from pop-up not found")
                    # if given name is empty, check the status of Rename button and return to the BT list
                    if self.name == '':
                        if rename_edit_text.text:
                            raise Exception("Error when clearing old BT name, not empty")
                        if rename_button.enabled:
                            raise Exception("Rename button in popup not disabled when empty name")
                        cancel_button = self.uidevice(text="CANCEL")
                        if not cancel_button.wait.exists(timeout=self.timeout):
                            raise Exception("Cancel button not found in Rename popup when empty name")
                        cancel_button.click()
                        if not self.bt_list.wait.exists(timeout=self.timeout):
                            raise Exception("BT devices list not reached after cancel rename BT with empty name")
                        self.set_passm("Rename button disabled when empty name")
                    # if given name is not empty, rename it
                    else:
                        rename_button.click()
                        if not self.bt_list.wait.exists(timeout=self.timeout):
                            raise Exception("BT devices list not reached after renaming BT")
                        if not self.bt_list.scroll.to(textContains="visible"):
                            raise Exception("BT name was not found down of the list after rename")
                        self.set_passm("Device renamed: " + self.name)
                # else pass, and write in the logs that device is already renamed
                else:
                    self.set_passm("Device already named: " + self.name)
                #self.set_passm("Device already named: " + self.name)
        except Exception, e:
            message = e.message
            if message==None or message == "":
                message = traceback.print_exc()
            self.set_errorm("Rename BT to " + self.name, message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if device was renamed(or Rename button is grayed out when empty name), False if not.
        """
        return self.step_data


class BtSearchDevices(BtStep):
    """ Description:
            Refreshes the BT available list until a certain device has
            appeared(for a max_attempt tries). Note that this let the BT
            list scrolled to the required device in the list. Call this in
            BT settings list, with BT ON and not any scanning in progress
        Usage:
            bluetooth_steps.BtSearchDevices(serial=serial,
                                dev_to_find="BT_test", scan_timeout=60000,
                                max_attempts=1, version=version)()
    """

    def __init__(self, dev_to_find, scan_timeout=60000, max_attempts=1, **kwargs):
        """
        :param dev_to_find: name of the device to be found
        :param scan_timeout: maximum timeout for scanning progress
        :param max_attempts: maximum no. of tries
        :param kwargs: serial, version, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.dev_to_find = dev_to_find
        self.scan_timeout = scan_timeout
        self.max_attempts = max_attempts
        self.step_data = True
        #if self.version.startswith("5.") or self.version.startswith("6.0"):
            # LLP, M versions
        #    self.bt_list = self.uidevice(resourceId="android:id/list")
        #else:
            # N version
        #    self.bt_list = self.uidevice(resourceId="com.android.settings:id/list")

    def do(self):
        try:
            #if not self.uidevice(text="Available devices").wait.exists(timeout=self.timeout):
            #    raise Exception("BT devices list was not found")
            counter = 1
            # condition = True means that the device was found
            condition = False
            while not condition:
                # break if max_attempts reached
                if counter > self.max_attempts:
                    break
                if self.device_info.dessert < 'O':
                    # open More options menu and click Refresh
                    menu_button = self.uidevice(description="More options")
                    if not menu_button.wait.exists(timeout=self.timeout):
                        raise Exception("Try " + str(counter) + ": More options button in BT settings not found")
                    menu_button.click()
                    refresh_button = self.uidevice(text="Refresh")
                    if not refresh_button.wait.exists(timeout=self.scan_timeout):
                        raise Exception("Try " + str(counter) + ": Refresh button was not found")
                    refresh_button.click()
                else:
                    if counter != 1:
                        self.uidevice.press.back()
                    ui_steps.click_button_common(serial=self.serial,
                                                 view_to_find={"text": "Pair new device"}, optional=True)()
                #if not self.bt_list.wait.exists(timeout=self.timeout):
                #    raise Exception("Try " + str(counter) + ": BT devices list was not found")
                # wait until scanning process is finished
                if not WaitBtScanning(serial=self.serial,
                        time_to_wait=self.scan_timeout, critical=False,
                        version=self.version)():
                    raise Exception("Wait for scanning to finish failed")
                counter += 1
                # check if device was found, if not, perform again the while loop
                condition = ui_steps.wait_for_view_common(serial=self.serial,
                            view_to_find={"text":self.dev_to_find}, optional=True)()
            self.set_passm("Device " + self.dev_to_find + " found after " + str(
                    counter - 1) + " attempt(s)")
        except Exception, e:
            self.set_errorm("Scan after " + self.dev_to_find, e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if device was found, False if not.
        """
        if self.step_data:
            # returns if the device is on the screen, or not
            self.set_errorm("Scan after " + self.dev_to_find,
                            "Search failed, device " + self.dev_to_find + " was not found in Available devices" +
                            " list after " + str(self.max_attempts) + " attempt(s)")
            self.step_data = self.uidevice(text=self.dev_to_find).exists
        return self.step_data


class GetPasskey(BtStep):
    """ Description:
            Get the pairing code from the pair request window. Call this in
            the Pairing request window
        Usage:
            bluetooth_steps.GetPasskey(serial=serial)()
    """

    def __init__(self, **kwargs):
        """
        :param kwargs: serial, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.step_data = None
        self.set_errorm("Get passkey from Pair request", "Could not obtain passkey")

    def do(self):
        try:
            passkey_object = self.uidevice(resourceId="com.android.settings:id/pairing_subhead")
            if not passkey_object.exists:
                raise Exception("Pairing code not displayed")
            # save the passkey in step_data
            self.step_data = passkey_object.text
        except Exception, e:
            self.set_errorm("Get passkey from Pair request", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if passkey was found, False if not. Note that the passkey is saved as str in step_data
        """
        if self.step_data:
            self.set_passm("Passkey " + str(self.step_data))
            return True
        else:
            return False


class PasskeyCheck(BtStep):
    """ Description:
            This method checks if the pairing request passkeys are both
            on the initiator and on the receiver
        Usage:
            bluetooth_steps.PasskeyCheck(serial=serial, passkey_initiator=passkey1,
                                            passkey_receiver=passkey2)()
     """

    def __init__(self, passkey_initiator, passkey_receiver, **kwargs):
        """
        :param passkey_initiator: passkey of the initiator device
        :param passkey_receiver: passkey of the receiver device
        :param kwargs: standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.passkey_initiator = passkey_initiator
        self.passkey_receiver = passkey_receiver
        self.set_passm("Pairing code matches; " + str(self.passkey_initiator) + " = " + str(self.passkey_receiver))
        self.set_errorm("Pairing code does not match",
                        "Initiator: " + str(self.passkey_initiator) + " Receiver: " + str(self.passkey_initiator))

    def do(self):
        # do nothing, only check
        pass

    def check_condition(self):
        """
        :return: True if passkeys match, False if not.
        """
        return self.passkey_initiator == self.passkey_receiver


class CheckIfPaired(BtStep):
    """ Description:
            Checks if the device is paired, or not(depending on the paired parameter)
            with another device. Call this with the BT list opened
        Usage:
            bluetooth_steps.CheckIfPaired(serial=serial,
                            dev_paired_with = DEVNAME, paired=True, version=version)()
    """

    def __init__(self, dev_paired_with, paired=True, **kwargs):
        """
        :param dev_paired_with: name of the device to check if DUT is(not) paired with
        :param paired: True, to check if DUT is paired with, False, to check if DUT is not paired with
        :param kwargs: serial, version, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.dev_paired_with = dev_paired_with
        self.paired = paired
        self.step_data = True
        if self.version.startswith("5.") or self.version.startswith("6.0"):
            # LLP, M versions
            self.bt_list = self.uidevice(resourceId="android:id/list")
        else:
            # N version
            self.bt_list = self.uidevice(resourceId="com.android.settings:id/list")
        if self.paired:
            self.message_str = "Check if paired"
            self.set_passm("Paired with " + str(dev_paired_with))
            self.set_errorm(self.message_str, "Not paired with " + str(dev_paired_with))
        else:
            self.message_str = "Check if not paired"
            self.set_passm("Not paired with " + str(dev_paired_with))
            self.set_errorm(self.message_str, "Paired with " + str(dev_paired_with))

    def do(self):
        try:
            if self.device_info.dessert < "O":
                if not self.bt_list.wait.exists(timeout=self.timeout):
                    raise Exception("BT list not found")
                if self.bt_list.scroll.to(text=self.dev_paired_with):
                    device_layout = self.bt_list.child_by_text(self.dev_paired_with, allow_scroll_search=False,
                                                               className="android.widget.LinearLayout")
                    if self.paired:
                        if not device_layout.child(resourceId="com.android.settings:id/deviceDetails").wait.exists(
                                timeout=self.timeout):
                            self.step_data = False
                    else:
                        if not device_layout.child(resourceId="com.android.settings:id/deviceDetails").wait.gone(
                                timeout=self.timeout):
                            self.step_data = False
                else:
                    if self.paired:
                        self.step_data = False
            else:
                condition = False
                condition = ui_steps.wait_for_view_common(serial=self.serial,
                                                          view_to_find={"textContains": self.dev_paired_with},
                                                          second_view_to_find={
                                                              "resourceId": "com.android.settings:id/settings_button"},
                                                          position='right', optional=True)()
                if self.paired:
                    if condition:
                        self.step_data = True
                    else:
                        self.step_data = False
                else:
                    if condition:
                        self.step_data = False
                    else:
                        self.step_data = True
        except Exception, e:
            self.set_errorm(self.message_str, e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if DUT is paired (or not, depending on Paired=True/False) with required device, False otherwise
        """
        return self.step_data


class WaitPairRequest(BtStep):
    """ Description:
            Waits for the pair request alert to appear or to be gone,
            as defined by parameter appear=True/False.
        Usage:
            bluetooth_steps.WaitPairRequest(serial=serial,
                                    appear=True, time_to_wait=10000, version=version)()
    """

    def __init__(self, appear=True, time_to_wait=10000, **kwargs):
        """
        :param appear: True, to check if appears, False, to check if gone
        :param time_to_wait: maximum time to wait for pairing request window
        :param kwargs: serial, version, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.appear = appear
        self.time_to_wait = time_to_wait
        if self.appear:
            self.set_passm("Pair request appeared")
            self.set_errorm("Wait pair request window",
                            "Pair request not appeared after " + str(self.time_to_wait) + " milliseconds")
        else:
            self.set_passm("Pair request gone")
            self.set_errorm("Wait pair request window gone",
                            "Pair request not gone after " + str(self.time_to_wait) + " milliseconds")

    def do(self):
        # nothing to do here, only to check
        pass

    def check_condition(self):
        """
        :return: True if pairing request window appears(or is gone), False otherwise
        """
        if self.version.startswith("5."):
            # LLP version
            if self.appear:
                # wait for appear of pair request dialog
                self.step_data = self.uidevice(resourceId="android:id/alertTitle",
                                               text="Bluetooth pairing request").wait.exists(timeout=self.time_to_wait)
            else:
                # wait until pair request dialog disappears
                self.step_data = self.uidevice(resourceId="android:id/alertTitle",
                                               text="Bluetooth pairing request").wait.gone(timeout=self.time_to_wait)
        else:
            # M version
            if self.appear:
                # wait for appear of pair request dialog
                self.step_data = self.uidevice(resourceId="android:id/alertTitle",
                                               textContains="Pair with").wait.exists(timeout=self.time_to_wait)
            else:
                # wait until pair request dialog disappears
                self.step_data = self.uidevice(resourceId="android:id/alertTitle",
                                               textContains="Pair with").wait.gone(timeout=self.time_to_wait)
        return self.step_data


class InitiatePairRequest(BtStep):
    """ Description:
            Initiate a pair request. It searches for the device name, clicks on it and assures
            that the initiator device is in the pairing request window (i.e. if pair request window
            is not displayed on the screen, it checks if the "Cannot communicate" message is displayed,
            and if not, it searches the request in the notifications menu)
        Usage:
            bluetooth_steps.InitiatePairRequest(serial=serial, dev_to_pair_name="Name",
                        scan_timeout=60000, scan_max_attempts=1, version=version)()
    """

    def __init__(self, dev_to_pair_name, scan_timeout=60000, scan_max_attempts=1, **kwargs):
        """
        :param dev_to_pair_name: name of device to pair with
        :param scan_timeout: maximum timeout for scanning progress
        :param scan_max_attempts: maximum no. of scan tries till the device is found
        :param kwargs: serial, version, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.dev_to_pair_name = dev_to_pair_name
        self.scan_timeout = scan_timeout
        self.scan_max_attempts = scan_max_attempts
        self.step_data = True
        self.set_passm("Pair request initiated to " + str(dev_to_pair_name))

    def do(self):
        try:
            # search for required device
            if not BtSearchDevices(serial=self.serial, dev_to_find=self.dev_to_pair_name,
                                   scan_timeout=self.scan_timeout,
                                   timeout=self.timeout, max_attempts=self.scan_max_attempts, version=self.version,
                                    critical=False)():
                raise Exception("Search for device failed")
            # click on the device name (already scrolled in the view)
            self.uidevice(text=self.dev_to_pair_name).click()

            if self.version.startswith("5."):
                # LLP version
                # if pair request window not appear on the device, open notification and check
                # if there is not even there the pairing request
                if not self.uidevice(resourceId="android:id/alertTitle",
                                     text="Bluetooth pairing request").wait.exists(timeout=5000):
                    if self.uidevice(textContains="Can't communicate with").exists:
                        raise Exception(
                                "Pair request not initiated from DUT because can't communicate with other one device")
                    if not SearchPairRequestNotification(serial=self.serial, timeout=self.timeout, version=self.version,
                                                         critical=False, no_log=True)():
                        raise Exception(
                                "Pair request not appeared on the screen, also failed" +
                                " searching it in notifications menu")
                    if not WaitPairRequest(serial=self.serial, appear=True, time_to_wait=self.timeout,
                                           version=self.version, critical=False, no_log=True)():
                        raise Exception("Pair request not initiated")
                if not self.uidevice(resourceId="com.android.settings:id/message_subhead",
                                     text=self.dev_to_pair_name).wait.exists(timeout=self.timeout):
                    raise Exception("Pair request not initiated to the expected device")
            else:
                # M, N version
                # if pair request window not appear on the device, open notification and check
                # if there is not even there the pairing request
                pair_request_title_obj = self.uidevice(resourceId="android:id/alertTitle", textContains="Pair with")
                if not pair_request_title_obj.wait.exists(timeout=5000):
                    if self.uidevice(textContains="Can't communicate with").exists:
                        raise Exception(
                                "Pair request not initiated from DUT because can't communicate with other one device")
                    if self.device_info.dessert < "O":
                        if not SearchPairRequestNotification(serial=self.serial, timeout=self.timeout, version=self.version,
                                                             critical=False, no_log=True)():
                            raise Exception(
                                    "Pair request not appeared on the screen, also failed" +
                                    " searching it in notifications menu")
                        if not WaitPairRequest(serial=self.serial, appear=True, time_to_wait=self.timeout,
                                               version=self.version, critical=False, no_log=True)():
                            raise Exception("Pair request not initiated")
                pair_request_title_str = pair_request_title_obj.text
                if not pair_request_title_str == "Pair with " + str(self.dev_to_pair_name) + "?":
                    raise Exception(
                            "Pair request not initiated to the expected device, found " + str(pair_request_title_str))
        except Exception, e:
            self.set_errorm("Pair request to " + str(self.dev_to_pair_name), e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if Both devices are in the pair request window, False otherwise
        """
        return self.step_data


class PairDevice(BtStep):
    """ Description:
            Initiate a pair request. It searches for the device name, clicks on it and assures
            that the initiator device is in the pairing request window (i.e. if pair request window
            is not displayed on the screen, it checks if the "Cannot communicate" message is displayed,
            and checks device name paired or not to DUT, If paired returns true)
        Usage:
            bluetooth_steps.PairDevice(serial=serial, dev_to_pair_name="Name",
                        scan_timeout=60000, scan_max_attempts=1, version=version)()
    """

    def __init__(self, dev_to_pair_name, scan_timeout=60000, scan_max_attempts=1, **kwargs):
        """
        :param dev_to_pair_name: name of device to pair with
        :param scan_timeout: maximum timeout for scanning progress
        :param scan_max_attempts: maximum no. of scan tries till the device is found
        :param kwargs: serial, version, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.dev_to_pair_name = dev_to_pair_name
        self.scan_timeout = scan_timeout
        self.scan_max_attempts = scan_max_attempts
        self.step_data = True
        self.set_passm("Paired with " + str(dev_to_pair_name))

    def do(self):
        try:
            # search for required device
            if not BtSearchDevices(serial=self.serial, dev_to_find=self.dev_to_pair_name,
                                   scan_timeout=self.scan_timeout,
                                   timeout=self.timeout, max_attempts=self.scan_max_attempts, version=self.version,
                                   critical=False)():
                raise Exception("Search for device failed")
            # click on the device name (already scrolled in the view)
            self.uidevice(text=self.dev_to_pair_name).click()
            if self.version.startswith("5."):
                # LLP version
                # if pair request window not appear on the device, open notification and check
                # if there is not even there the pairing request
                if not self.uidevice(resourceId="android:id/alertTitle",
                                     text="Bluetooth pairing request").wait.exists(timeout=5000):
                    if self.uidevice(textContains="Can't communicate with").exists:
                        raise Exception(
                                "Pair request not initiated from DUT because can't communicate with other one device")
            else:
                # M, N version
                # if pair request window not appear on the device, open notification and check
                # if there is not even there the pairing request
                pair_request_title_obj = self.uidevice(resourceId="android:id/alertTitle", textContains="Pair with")
                if not pair_request_title_obj.wait.exists(timeout=5000):
                    if self.uidevice(textContains="Can't communicate with").exists:
                        raise Exception(
                                "Pair request not initiated from DUT because can't communicate with other one device")
        except Exception, e:
            self.set_errorm("Pair request to " + str(self.dev_to_pair_name), e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if Device was paired, False if not
        """
        if self.step_data:
            # check if is  paired with required device
            self.step_data = CheckIfPaired(serial=self.serial, dev_paired_with=self.dev_to_pair_name, paired=True,
                                           timeout=self.timeout, version=self.version, critical=False)()
        return self.step_data


class ReceivePairRequest(BtStep):
    """ Description:
            Receives a pair request. It assures that device is
            in the pairing request window (i.e. if pair request window
            is not received on the screen, it searches it in the
            notifications menu)
        Usage:
            bluetooth_steps.ReceivePairRequest(serial=serial,
                        dev_receiving_from_name="Name", version=version)()
    """

    def __init__(self, dev_receiving_from_name, **kwargs):
        """
        :param dev_receiving_from_name: name of the device receiving pair request from
        :param kwargs: serial, version, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.dev_receiving_from_name = dev_receiving_from_name
        self.step_data = True
        self.set_passm("Pair request received from " + str(self.dev_receiving_from_name))

    def do(self):
        try:
            if self.version.startswith("5."):
                # LLP version
                # if pair request window not appear on the receiver device, open notification and check if
                # there is not even there the pairing request
                if not self.uidevice(resourceId="android:id/alertTitle", text="Bluetooth pairing request").wait.exists(
                        timeout=5000):
                    if not SearchPairRequestNotification(serial=self.serial, timeout=self.timeout, version=self.version,
                                                         critical=False)():
                        raise Exception(
                                "Pair request not received on the screen, also failed" +
                                " searching it in notifications menu")
                    if not WaitPairRequest(serial=self.serial, appear=True, time_to_wait=self.timeout,
                                           version=self.version, critical=False)():
                        raise Exception("Pair request not received")
                if not self.uidevice(resourceId="com.android.settings:id/message_subhead",
                                     text=self.dev_receiving_from_name).wait.exists(timeout=self.timeout):
                    raise Exception("Pair request not received from the expected device")
            else:
                # M, N version
                # if pair request window not appear on the receiver device, open notification and check if
                # there is not even there the pairing request
                pair_request_title_obj = self.uidevice(resourceId="android:id/alertTitle", textContains="Pair with")
                if not pair_request_title_obj.wait.exists(timeout=5000):
                    if not SearchPairRequestNotification(serial=self.serial, timeout=self.timeout, version=self.version,
                                                         critical=False, no_log=True)():
                        raise Exception(
                                "Pair request not received on the screen, also failed" +
                                " searching it in notifications menu")
                    if not WaitPairRequest(serial=self.serial, appear=True, time_to_wait=self.timeout,
                                           verion=self.version, critical=False, no_log=True)():
                        raise Exception("Pair request not received on device")
                pair_request_title_str = pair_request_title_obj.text
                if not pair_request_title_str == "Pair with " + str(self.dev_receiving_from_name) + "?":
                    raise Exception(
                            "Pair request not received from the expected device, found " + str(pair_request_title_str))
        except Exception, e:
            self.set_errorm("Pair request from " + str(self.dev_receiving_from_name), e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if Both devices are in the pair request window, False otherwise
        """
        return self.step_data


class SearchPairRequestNotification(BtStep):
    """ Description:
            Opens a Pairing request from the notification menu. Note that
            this does not check if, indeed the pairing request dialog appears,
            it only clicks the notification. Call this only if the request
            dialog is not displayed and it should be
        Usage:
            bluetooth_steps.SearchPairRequestNotification(serial=serial)()
    """

    def __init__(self, **kwargs):
        """
        :param kwargs: serial, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.step_data = True
        self.set_passm("Pairing request notification clicked")

    def do(self):
        try:
            # open notification menu
            if not OpenNotificationsMenu(serial=self.serial, timeout=self.timeout, version=self.version, critical=False,
                                         no_log=True)():
                raise Exception("Notification menu not opened when searching for pairing request")
            # click on the pairing request notification
            if not BtCheckNotificationAppear(serial=self.serial, text_contains="Pairing request",
                                             click_on_notification=True, time_to_appear=self.timeout,
                                             version=self.version, critical=False, no_log=True)():
                raise Exception("Check Pair request notification not successful")
        except Exception, e:
            self.set_errorm("Search pair request in notifications ", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if Pair request notification was found and clicked, False otherwise
        """
        return self.step_data


class OpenNotificationsMenu(BtStep):
    """ Description:
            Opens the notifications menu in order to operate with Bluetooth notifications
        Usage:
            bluetooth_steps.OpenNotificationsMenu(serial=serial)()
    """

    def __init__(self, **kwargs):
        """
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.set_passm("Notifications menu opened")
        self.set_errorm("Open notifications", "Notifications menu not opened")

    def do(self):
        self.uidevice.open.notification()
        # sleep here for transition to be finished
        time.sleep(2)

    def check_condition(self):
        """
        :return: True if Notifications menu was opened, False otherwise
        """
        self.step_data = self.uidevice(resourceId="com.android.systemui:id/notification_stack_scroller").wait.exists(
                timeout=self.timeout)
        #self.step_data = True
        return self.step_data


class CloseNotificationsMenu(BtStep):
    """ Description:
            Closes the notifications menu
        Usage:
            bluetooth_steps.CloseNotificationsMenu(serial=serial)()
    """

    def __init__(self, **kwargs):
        """
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.step_data = True
        self.notifications_menu = self.uidevice(resourceId="com.android.systemui:id/notification_stack_scroller")
        self.set_passm("Notifications menu closed")
        self.set_errorm("Close notifications", "Notifications menu not gone")

    def do(self):
        try:
            if not self.notifications_menu.exists:
                raise Exception("Notifications menu is not already opened")
            self.uidevice.press.back()
        except Exception, e:
            self.set_errorm("Close notifications", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if Notifications menu was closed, False otherwise
        """
        if self.step_data:
            self.step_data = self.notifications_menu.wait.gone(timeout=self.timeout)
        return self.step_data


class PerformActionPairRequest(BtStep):
    """ Description:
            Performs a click on the button with label exact text as defined by
            action parameter and checks if the pair request window is gone. If
            the action is 'Timeout', it only waits for pair request window to be
            gone, the amount of time as defined by timeout parameter. Call this
            only when Pair request window is already shown
        Usage:
            bluetooth_steps.PerformActionPairRequest(serial=serial,
                                                        action="Pair", version=version)()
    """

    def __init__(self, action="Pair", **kwargs):
        """
        :param action: "Pair"/"Cancel"/"Timeout" action to be performed
        :param kwargs: serial, timeout, version, no_log,  and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        if action not in ["Cancel", "Pair", "Timeout"]:
            raise Exception("Config error: not any expected value for action")

        if self.version.startswith("5.") or self.version.startswith("6.0"):
            # LLP, M versions
            self.action = action
        else:
            # N version
            self.action = action.upper()
        self.step_data = True
        self.set_passm("Action " + str(self.action) + " successful")
        self.set_errorm("Action " + str(self.action), "Pair request window not gone after action performed")

    def do(self):
        try:
            # if action is not Timeout, perform click on the button
            if self.action.upper() != "TIMEOUT":
                action_button = self.uidevice(text=self.action)
                if not action_button.wait.exists(timeout=self.timeout+30000):
                    raise Exception("Button " + str(self.action) + " not found")
                action_button.click()
            if self.uidevice(text="YES").wait.exists(timeout=1000):
                self.uidevice(text="YES").click()
        except Exception, e:
            self.set_errorm("Action " + str(self.action), e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if pair request window is gone, False if not
        """
        if self.step_data:
            # check if the pair request window is gone
            self.step_data = WaitPairRequest(serial=self.serial, appear=False, time_to_wait=self.timeout,
                                             version=self.version, critical=False)()
        return self.step_data


class CouldNotPairDialogCheck(BtStep):
    """ Description:
            Checks if the "Couldn't pair" dialog is displayed
            (by waiting for it) and clicks on it's OK button.
        Usage:
            bluetooth_steps.CouldNotPairDialogCheck(serial=serial)()
    """

    def __init__(self, **kwargs):
        """
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.step_data = True
        self.set_passm("Dialog appeared, canceled successful")
        self.set_errorm("Could not pair dialog", "Not canceled successfully")
        self.dialog_window = self.uidevice(resourceId="android:id/message", textContains="incorrect PIN or passkey")

    def do(self):
        try:
            if self.device_info.dessert < "O":
                # wait for dialog to appear
                if not self.dialog_window.wait.exists(timeout=self.timeout+30000):
                    raise Exception("Dialog not appeared")
                # click on it's OK button
                ok_button = self.uidevice(text="OK")
                if not ok_button.wait.exists(timeout=self.timeout+30000):
                    raise Exception("OK not found in the dialog")
                ok_button.click()
            else:
                pass
                # in O dialog box disappears automatically, we are not checking dialog box for greater than O dessert
        except Exception, e:
            self.set_errorm("Could not pair dialog", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if Could not pair dialog is gone after press on OK, False otherwise
        """
        if self.step_data:
            # check if dialog is gone
            self.step_data = self.dialog_window.wait.gone(timeout=self.timeout)
        return self.step_data


class BtRemoveAllPairedDevices(BtStep):
    """ Description:
            All pair devices will be removed from the list. Call this in BT
            devices list, with no scanning in progress
        Usage:
            bluetooth_steps.BtRemoveAllPairedDevices(serial = serial,
                                            max_attempts=20, version=version)()
    """
    def __init__(self, max_attempts=20, **kwargs):
        """
        :param max_attempts: maximum no. of tries
        :param kwargs: serial, version, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.max_attempts = max_attempts
        self.paired_title = self.uidevice(text="Paired devices")
        if self.version.startswith("5.") or self.version.startswith("6.0"):
            # LLP, M versions
            self.bt_list = self.uidevice(resourceId="android:id/list")
        else:
            # N version
            self.bt_list = self.uidevice(resourceId="com.android.settings:id/list")
        self.step_data = True
        self.set_passm("Nothing to unpair")
        self.set_errorm(str(max_attempts) + "attempts", "Not removed all paired devices")

    def do(self):
        try:
            if self.version.startswith("5.") or self.version.startswith("6.") or self.version.startswith("7."):
                if not self.bt_list.wait.exists(timeout=self.timeout):
                    raise Exception("BT devices list was not found")
                # execute only if Paired devices title is found
                if self.bt_list.scroll.to(text="Paired devices"):
                    counter = 1
                    # for each existing paired button, click on it and FORGET
                    while self.paired_title.exists:
                        if counter > self.max_attempts:
                            break
                        paired_button = self.uidevice(description="Device settings")
                        paired_button.click.wait()
                        time.sleep(1)
                        if not self.uidevice(resourceId="android:id/alertTitle").wait.exists(timeout=self.timeout):
                            raise Exception(
                                    "Alert title not opened when removing " + " (device no. " + str(
                                            counter) + ")")
                        if self.version.startswith("5."):
                            # LLP version
                            forget_button = self.uidevice(resourceId="android:id/button2", text="FORGET")
                        elif self.version.startswith("6.0"):
                            # M version
                            forget_button = self.uidevice(resourceId="android:id/button2", text="Forget")
                            # force a small delay due to window transition
                            time.sleep(1)
                        else:
                            # N version
                            # force a small delay due to window transition and close keyboard
                            time.sleep(1)
                            if "mInputShown=true" in self.adb_connection.cmd("shell dumpsys input_method").communicate()[
                                0].decode("utf-8"):
                                self.uidevice.press.back()
                                time.sleep(1)
                            forget_button = self.uidevice(text="FORGET")
                        if not forget_button.wait.exists(timeout=self.timeout):
                            raise Exception(
                                    "Forget button not found when unpair " + " (device no. " + str(
                                            counter) + ")")
                        forget_button.click()
                        if not self.bt_list.wait.exists(timeout=self.timeout):
                            raise Exception(
                                    "Not returned to BT list after unpair " + " (device no. " + str(
                                            counter) + ")")
                        counter += 1
                    self.set_passm(str(counter - 1) + " device(s) unpaired")
            else:
                counter = 1
                # for each existing paired button, click on it and FORGET
                while self.paired_title.exists and self.uidevice(description="Settings"):
                    if counter > self.max_attempts:
                        break
                    ui_steps.click_button_common(serial=self.serial,
                                                view_to_find={"description": "Settings"},
                                                view_to_check={"resourceId": "android:id/alertTitle"})()
                    time.sleep(1)
                    if not ui_steps.click_button_common(serial=self.serial,
                                                        view_to_find={"text": "FORGET"})():
                        raise Exception("Forget button not found when unpair " + " (device no. " + str(counter) + ")")
                    counter += 1
                    self.set_passm(str(counter - 1) + " device(s) unpaired")

        except Exception, e:
            self.set_errorm("Unpair devices", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if all paired devices were unpaired, False otherwise
        """
        if self.step_data and self.device_info.dessert < "O":
            # check if "Paired devices" title is gone
            self.step_data = self.uidevice(text="Paired devices").wait.gone(timeout=self.timeout)
        else:
            self.step_data = self.uidevice(description="Settings").wait.gone(timeout=self.timeout)
        return self.step_data


class OpenPairedDeviceSettings(BtStep):
    """ Description:
            Open the device settings alert title for a certain paired device.
            Call this in BT settings list for a device a device already
            paired
        Usage:
            bluetooth_steps.OpenPairedDeviceSettings(serial = serial,
                                        device_name="DEV_name", version=version)()
    """

    def __init__(self, device_name, **kwargs):
        """
        :param device_name: name of device in the list for which Settings should be opened
        :param kwargs: serial, version, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.device_name = device_name
        self.step_data = True
        if self.version.startswith("5.") or self.version.startswith("6.0"):
            # LLP, M versions
            self.bt_list = self.uidevice(resourceId="android:id/list")
        else:
            # N version
            self.bt_list = self.uidevice(resourceId="com.android.settings:id/list")
        self.set_passm("Device settings opened for device " + str(self.device_name))
        self.set_errorm("Paired device settings for " + str(self.device_name), "Device settings not opened")

    def do(self):
        try:
            if self.device_info.dessert < "O":
                if not self.bt_list.wait.exists(timeout=self.timeout):
                    raise Exception("BT list not found")
                if not self.bt_list.scroll.to(text=self.device_name):
                    raise Exception("Device " + str(self.device_name) + " not found in BT list")
                # get linear layout corresponding to required device
                device_layout = self.bt_list.child_by_text(self.device_name, allow_scroll_search=False,
                                                           className="android.widget.LinearLayout")
                # get the device settings button corresponding to required device by searching the child of the linear
                # layout
                device_settings_button = device_layout.child(resourceId="com.android.settings:id/deviceDetails")
                if not device_settings_button.wait.exists(timeout=self.timeout):
                    raise Exception("Device settings button not found")
                # click on device settings
                device_settings_button.click()
                if self.version.startswith("5.") or self.version.startswith("6.0"):
                    # LLP, M versions
                    # do nothing, no workaround needed
                    pass
                else:
                    # N version workaround
                    if not self.uidevice(resourceId="android:id/alertTitle", text="Paired devices").wait.exists(
                            timeout=self.timeout+30000):
                        raise Exception("Device settings not opened")
                    # force a small delay due to window transition and close keyboard
                    time.sleep(1)
                    if "mInputShown=true" in self.adb_connection.cmd("shell dumpsys input_method").communicate()[0].decode(
                            "utf-8"):
                        self.uidevice.press.back()
                        time.sleep(1)
            else:
                paired_button = self.uidevice(description="Settings")
                paired_button.click.wait()
                time.sleep(1)
                if not self.uidevice(resourceId="android:id/alertTitle").wait.exists(timeout=self.timeout):
                    raise Exception("Alert title not opened when removing")
                    # O version
                    # force a small delay due to window transition and close keyboard
                time.sleep(1)
                if "mInputShown=true" in self.adb_connection.cmd("shell dumpsys input_method").communicate()[0].decode(
                        "utf-8"):
                    self.uidevice.press.back()
        except Exception, e:
            self.set_errorm("Paired device settings for " + str(self.device_name), e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if Device settings was opened, False otherwise
        """
        if self.step_data:
            # check if Device settings window is opened
            self.step_data = self.uidevice(resourceId="android:id/alertTitle", text="Paired devices").wait.exists(
                    timeout=self.timeout)
        return self.step_data


class UnpairDevice(BtStep):
    """ Description:
            Unpair a certain device from the list.Call this in BT settings
            list for a device a device already paired
        Usage:
            bluetooth_steps.UnpairDevice(serial = serial,
                                    device_name="DEV_name", version=version)()
    """

    def __init__(self, device_name, **kwargs):
        """
        :param device_name: name of device from the list to be unpaired
        :param kwargs: serial, timeout, version, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.device_name = device_name
        self.step_data = True
        if self.version.startswith("5.") or self.version.startswith("6.0"):
            # LLP, M versions
            self.bt_list = self.uidevice(resourceId="android:id/list")
        else:
            # N version
            self.bt_list = self.uidevice(resourceId="com.android.settings:id/list")
        self.set_passm("Device " + str(self.device_name) + " unpaired")
        self.set_errorm("Unpair device " + str(self.device_name), "Device is still paired")

    def do(self):
        try:
            if not OpenPairedDeviceSettings(serial=self.serial, device_name=self.device_name, timeout=self.timeout,
                                            version=self.version, critical=False)():
                raise Exception("Open paired device settings failed")
            if not self.uidevice(text=self.device_name):
                raise Exception("Name of the device not found in the unpair alert window")
            # click on forget
            if self.version.startswith("5."):
                # LLP version
                forget_button = self.uidevice(resourceId="android:id/button2", text="FORGET")
            elif self.version.startswith("6.0"):
                # M version
                forget_button = self.uidevice(resourceId="android:id/button2", text="Forget")
                # force a small delay due to window transition
                time.sleep(1)
            else:
                # N version
                forget_button = self.uidevice(text="FORGET")
            if not forget_button.wait.exists(timeout=self.timeout):
                raise Exception("Forget button not found when unpairing device")
            forget_button.click()
            if not self.bt_list.wait.exists(timeout=self.timeout):
                raise Exception("Not returned to BT list after unpairing device")
        except Exception, e:
            self.set_errorm("Unpair device " + str(self.device_name), e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if Device was unpaired, False if not
        """
        if self.step_data:
            # check if is not paired with required device
            self.step_data = CheckIfPaired(serial=self.serial, dev_paired_with=self.device_name, paired=False,
                                           timeout=self.timeout, version=self.version, critical=False)()
        return self.step_data


class DisconnectDevice(BtStep):
    """ Description:
            disconnect a certain device from the list and still it will be paired
        Usage:
            bluetooth_steps.DisconnectDevice(serial = serial,
                                    device_name="DEV_name", version=version)()
    """

    def __init__(self, device_name, **kwargs):
        """
        :param device_name: name of device from the list to be disconnected
        :param kwargs: serial, timeout, version, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.device_name = device_name
        self.step_data = True
        self.set_passm("Device " + str(self.device_name) + " Disconnected and still will be paired")
        self.set_errorm("DisconnectDevice" + str(self.device_name), "Device is still connected")

    def do(self):
        try:
            if ui_steps.click_button_common(serial=self.serial, view_to_find={"textContains": self.device_name},
                                         view_to_check = {"resourceId": "android:id/alertTitle"})():
                if not ui_steps.click_button_common(serial=self.serial, view_to_find={"textContains": "ok"})():
                    self.step_data=False
            else:
                self.step_data=False

        except Exception, e:
            self.set_errorm("Disconnect device " + str(self.device_name), e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if Device was paired, False if not
        """
        if self.step_data:
            # check if is not paired with required device
            self.step_data = CheckIfPaired(serial=self.serial, dev_paired_with=self.device_name, paired=True,
                                           timeout=self.timeout, version=self.version, critical=False)()
        return self.step_data


class BtCheckProfiles(BtStep):
    """ Description:
            Checks that DUT supports a BT profile, with 'sdptool browse'
            command. Make sure that you have a BT adapter on the workstation
        Usage:
            bluetooth_steps.BtCheckProfiles(serial=serial, bt_mac_add="MAC_ADDR_OF_DUT",
                                            bl_profile="BT_PROFILE_TO_CHECK",
                                            bl_version_value="EXPECTED_VERSION_VALUE")()
    """

    def __init__(self, bt_mac_add, bl_profile, bl_version_value, **kwargs):
        """
        :param bt_mac_add: mac address of the DUT
        :param bl_profile: profile to be checked
        :param bl_version_value: expected version value for the profile
        """
        BtStep.__init__(self, **kwargs)
        self.bt_mac_add = bt_mac_add
        self.bl_profile = bl_profile
        self.bl_version_value = bl_version_value
        self.step_data = True
        self.actual_version = None
        self.stdout, self.stderr = None, None
        self.cmd = 'sdptool browse ' + str(self.bt_mac_add)
        self.set_passm(self.bl_profile + " has the value: " + self.bl_version_value)

    def do(self):
        try:
            self.stdout, self.stderr = subprocess.Popen(self.cmd, shell=True, stdout=subprocess.PIPE,
                                                        stderr=subprocess.PIPE).communicate()
            profiles = self.stdout.split("\n\n")
            for profile in profiles:
                if self.bl_profile in profile:
                    self.actual_version = profile.split("Profile Descriptor List:")[-1].split("Version: ")[-1]
                    break
        except Exception, e:
            self.set_errorm("Exception encountered when running " + self.cmd + " command", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if profile with expected version was found, False if not
        """
        if self.step_data:
            self.step_data = self.actual_version == self.bl_version_value
            if not self.step_data:
                if self.actual_version:
                    self.set_errorm("Profile " + self.bl_profile,
                                    "Profile has the value " + self.actual_version + " instead of " +
                                    self.bl_version_value)
                elif self.stdout:
                    self.set_errorm("Profile " + self.bl_profile, "Output '" + str(self.cmd) + ": " + str(self.stdout))
                elif self.stderr:
                    self.set_errorm("Profile " + self.bl_profile, "Error '" + str(self.cmd) + ": " + str(self.stderr))
                else:
                    self.set_errorm("Profile " + self.bl_profile, "Empty stdout, stderr for '" + str(self.cmd) + "'")
        return self.step_data


class BtSetTethering(BtStep):
    """ Description:
            Sets the required state to BT tethering, if not already.
            Use tethering_settings_opened=False to open BT tethering settings.
            Use check_if_already=True to fail if already has required state
        Usage:
            bluetooth_steps.BtSetTethering(serial=serial, state="ON",
                        tethering_settings_opened=False, check_if_already=False, version=version)()
    """

    def __init__(self, state="ON", tethering_settings_opened=False, check_if_already=False, **kwargs):
        """
        :param state: "ON"/"OFF", state of tethering switch to be set
        :param tethering_settings_opened: True if tethering settings is already opened, False to use intent to open
        :param check_if_already: True to fail if tethering  already has the required state, False to not fail
        :param kwargs: serial, version, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.state = state
        if self.state not in ["ON", "OFF"]:
            raise Exception("Config error, incorrect value for state")
        self.tethering_settings_opened = tethering_settings_opened
        self.check_if_already = check_if_already
        if self.version.startswith("5.") or self.version.startswith("6.0"):
            # LLP, M versions
            self.teth_list = self.uidevice(resourceId="android:id/list")
        else:
            # N version
            self.teth_list = self.uidevice(resourceId="com.android.settings:id/list")
        self.summary_field = self.uidevice(text="Bluetooth tethering").sibling(resourceId="android:id/summary")
        self.step_data = True
        self.set_passm("Tethering turned " + self.state)
        self.set_errorm("Set tethering " + self.state, "Could not turn Tethering " + self.state)

    def do(self):
        try:
            # open Bluetooth tethering settings with intent
            if not self.tethering_settings_opened:
                intent_cmd = "shell am start -n com.android.settings/.TetherSettings"
                self.adb_connection.cmd(intent_cmd).wait()
            if self.version.startswith("5.") or self.version.startswith("6.0"):
                # LLP, M versions
                action_bar_id = "android:id/action_bar"
            else:
                # N version
                action_bar_id = "com.android.settings:id/action_bar"
            if not self.uidevice(resourceId=action_bar_id).child(textContains="tethering").wait.exists(
                    timeout=self.timeout):
                raise Exception("BT tethering settings not opened")
            if not self.teth_list.wait.exists(timeout=self.timeout):
                raise Exception("Tethering settings list not found")
            # scroll to Tethering option and get the corresponding switch
            if not self.teth_list.scroll.to(text="Bluetooth tethering"):
                raise Exception("Bluetooth tethering option not exists")
            teth_switch = self.teth_list.child_by_text("Bluetooth tethering", allow_scroll_search=False,
                                                       className="android.widget.LinearLayout").child(
                    className="android.widget.Switch", enabled=True)
            if not teth_switch.wait.exists(timeout=self.timeout):
                raise Exception("BT tethering switch not found")
            # click on switch if does not already have the required state
            if not teth_switch.text == self.state:
                teth_switch.click()
                if not teth_switch.wait.exists(timeout=self.timeout):
                    raise Exception("BT tethering switch not set to " + self.state + " correctly")
            else:
                # fail if already has the state
                if self.check_if_already:
                    raise Exception("Tethering already " + self.state)
                else:
                    self.set_passm("Tethering already " + self.state)
            if not teth_switch.text == self.state:
                raise Exception("BT tethering switch not set to " + self.state)
            if not self.summary_field.wait.exists(timeout=self.timeout):
                raise Exception("Status of BT tethering not found")
        except Exception, e:
            self.set_errorm("Set tethering " + self.state, e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if required state was set, False if not
        """
        if self.step_data:
            if self.state == "ON":
                self.step_data = self.summary_field.text.startswith("Sharing this")
            else:
                self.step_data = self.summary_field.text.startswith("Not sharing this")
        return self.step_data


class BtCheckTetheringState(BtStep):
    """ dDescription:
            Checks if BT tethering has the required state.
            Use tethering_settings_opened=False to open BT tethering settings
        Usage:
            bluetooth_steps.BtCheckTetheringState(serial=serial, state="ON",
                            tethering_settings_opened=False, version=version)()
    """

    def __init__(self, state="ON", tethering_settings_opened=False, **kwargs):
        """
        :param state: "ON"/"OFF", expected state of tethering switch
        :param tethering_settings_opened: True if tethering settings is already opened, False to use intent to open
        :param kwargs: serial, version, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.state = state
        if self.state not in ["ON", "OFF"]:
            raise Exception("Config error, incorrect value for state")
        self.tethering_settings_opened = tethering_settings_opened
        if self.version.startswith("5.") or self.version.startswith("6.0"):
            # LLP, M versions
            self.teth_list = self.uidevice(resourceId="android:id/list")
        else:
            # N version
            self.teth_list = self.uidevice(resourceId="com.android.settings:id/list")
        self.summary_field = self.uidevice(text="Bluetooth tethering").sibling(resourceId="android:id/summary")
        self.step_data = True
        self.set_errorm("Check tethering to be " + self.state, "Tethering is not in the desire state")
        self.set_passm("Tethering is in the desire state : " + self.state)

    def do(self):
        try:
            # open Bluetooth tethering settings with intent
            if not self.tethering_settings_opened:
                intent_cmd = "shell am start -n com.android.settings/.TetherSettings"
                self.adb_connection.cmd(intent_cmd).wait()
            if self.version.startswith("5.") or self.version.startswith("6.0"):
                # LLP, M versions
                action_bar_id = "android:id/action_bar"
            else:
                # N version
                action_bar_id = "com.android.settings:id/action_bar"
            if not self.uidevice(resourceId=action_bar_id).child(textContains="tethering").wait.exists(
                    timeout=self.timeout):
                raise Exception("BT tethering settings not opened")
            if not self.teth_list.wait.exists(timeout=self.timeout):
                raise Exception("Tethering settings list not found")
            # scroll to Tethering option and get the status
            if not self.teth_list.scroll.to(text="Bluetooth tethering"):
                raise Exception("Bluetooth tethering option not exists")
            if not self.summary_field.wait.exists(timeout=self.timeout):
                raise Exception("Status of BT tethering not found")
        except Exception, e:
            self.set_errorm("Check tethering to be " + self.state, e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if tethering has required state, False if not
        """
        if self.step_data:
            if self.state == "ON":
                self.step_data = self.summary_field.text.startswith("Sharing this")
            else:
                self.step_data = self.summary_field.text.startswith("Not sharing this")
        return self.step_data


class BtSetService(BtStep):
    """ Description:
            Enable or disable a service for a paired device based on state
            parameter (checks if not already only if check_if_already parameter
            is True). Call this from the BT paired device list for a device already
            paired. Set disable_profile_alert to False for profiles with no disable
            service confirmation needed
        Usage:
            bluetooth_steps.BtSetService(serial=serial,
                                    paired_device_name="DEV_name", state=True,
                                    service="Internet access", check_if_already=False,
                                    disable_profile_confirm=True, version=version)()
    """

    def __init__(self, paired_device_name, state=True, service="Internet access", check_if_already=False,
                 disable_profile_confirm=True, **kwargs):
        """
        :param paired_device_name: name of the paired device
        :param state: True for check, False for uncheck
        :param service: name of the service(exact text as it appears)
        :param check_if_already: True to fail if service  already has the required state, False to not fail
        :param disable_profile_confirm: set to False for profiles with no disable service prompt confirmation needed
        :param kwargs: serial, version, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.paired_device_name = paired_device_name
        self.state = state
        self.service = service
        self.check_if_already = check_if_already
        self.disable_profile_confirm = disable_profile_confirm
        self.step_data = True
        self.set_passm("Service " + str(self.service) + " set to checked: " + str(self.state))

    def do(self):
        try:
            if not OpenPairedDeviceSettings(serial=self.serial, device_name=self.paired_device_name,
                                            timeout=self.timeout, version=self.version)():
                raise Exception("Open paired device settings failed")
            if self.version.startswith("5."):
                # LLP version
                bt_list = self.uidevice(resourceId="android:id/list")
                if not bt_list.wait.exists(timeout=self.timeout):
                    raise Exception("Paired device settings list not found")
                # get linear layout of the profile in order to get it's checkbox
                try:
                    profile_linear_layout = bt_list.child_by_text(self.service, allow_scroll_search=True,
                                                                  className="android.widget.LinearLayout")
                except:
                    profile_linear_layout = None
                if not profile_linear_layout:
                    raise Exception("Provided service was not found")
                # get the checkbox
                checkbox = profile_linear_layout.child(className="android.widget.CheckBox", enabled=True)
            elif self.version.startswith("6.0"):
                # M version
                if "mInputShown=true" in self.adb_connection.cmd("shell dumpsys input_method").communicate()[0].decode(
                        "utf-8"):
                    self.uidevice.press.back()
                    # no check, not always makes problem the keyboard here
                # get the checkbox
                checkbox = self.uidevice(className="android.widget.CheckBox", enabled=True, text=self.service)
            else:
                # N version
                # get the checkbox
                checkbox = self.uidevice(className="android.widget.CheckBox", enabled=True, text=self.service)
            if not checkbox.wait.exists(timeout=60000):
                raise Exception("Checkbox of service not found")
            ok_button = self.uidevice(text="OK")
            # if checkbox does not have the required state, click on it and verify
            if checkbox.checked != self.state:
                checkbox.click()
                # confirm the disabling of the profile(when required state is False)
                if not self.state and self.disable_profile_confirm:
                    disable_profile_alert = self.uidevice(resourceId="android:id/alertTitle",
                                                          textContains="Disable")
                    if not disable_profile_alert.wait.exists(timeout=self.timeout):
                        raise Exception("Disable Profile alert not shown when disabling service")
                    if not ok_button.wait.exists(timeout=self.timeout):
                        raise Exception("OK button not found when confirming Disable profile")
                    ok_button.click()
                    if not disable_profile_alert.wait.gone(timeout=self.timeout):
                        raise Exception(
                                "Disable Profile alert not gone after confirming disable of service")
                if not checkbox.wait.exists(timeout=self.timeout):
                    raise Exception(
                            "Checkbox not set correctly to checked: " + str(self.state))
            else:
                if self.check_if_already:
                    raise Exception("Service already has the required state")
                else:
                    self.set_passm("Service " + str(self.service) + " already set to checked: " + str(self.state))
            # verify that the checkbox now have the required state
            if checkbox.checked != self.state:
                raise Exception("Checkbox not set to checked: " + str(self.state))
            # close the device settings window by pressing OK
            if not ok_button.wait.exists(timeout=self.timeout):
                raise Exception("OK button not found after setting service")
            ok_button.click()
            if not self.uidevice(resourceId="android:id/alertTitle").wait.gone(timeout=self.timeout):
                raise Exception("Device settings alert window not gone after OK")
        except Exception, e:
            self.set_errorm(
                    "Set service " + self.service + " for device " + self.paired_device_name + " to checked: " + str(
                            self.state), e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if desired state for service was set, False if not
        """
        return self.step_data


class WaitForState(BtStep):
    """ Description:
            Wait for "Connected" state under a certain device from BT list to appear,
            or disappear(depending on "connected=True/False" parameter). To be used
            immediately after a service was checked/unchecked.
        Usage:
            bluetooth_steps.WaitForState(serial=serial, device_name="DEV_name",
                                                    connected=True, version=version)()
    """

    def __init__(self, device_name, connected=True, wait_time=10000, **kwargs):
        """
        :param device_name: name of the paired device
        :param connected: True to wait for Connected state to appear, False to wait to be gone
        :param wait_time: max time to wait for state to appear/disappear
        :param kwargs: serial, version, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.device_name = device_name
        self.connected = connected
        self.wait_time = wait_time
        self.step_data = True
        if self.version.startswith("5.") or self.version.startswith("6.0"):
            # LLP, M versions
            self.bt_list = self.uidevice(resourceId="android:id/list")
        else:
            # N version
            self.bt_list = self.uidevice(resourceId="com.android.settings:id/list")
        self.connected_state = self.uidevice(text=self.device_name).sibling(text="Connected")
        if self.connected:
            self.message_str = "connected"
            self.set_passm("State for " + str(self.device_name) + " is connected")
            self.set_errorm("Wait " + self.message_str + " state for " + str(self.device_name),
                            "State is not connected")

        else:
            self.message_str = "not connected"
            self.set_passm("State for " + str(self.device_name) + " is not connected")
            self.set_errorm("Wait " + self.message_str + " state for " + str(self.device_name), "State is connected")

    def do(self):
        try:
            # scroll to desired device
            if not self.bt_list.wait.exists(timeout=self.timeout):
                raise Exception("BT list not found")
            if not self.bt_list.scroll.to(text=self.device_name):
                raise Exception("Device " + str(self.device_name) + " not found in BT list")
        except Exception, e:
            self.set_errorm("Wait " + self.message_str + " state for " + str(self.device_name), e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if Connected state was appeared/has gone(as required), False if not
        """
        if self.step_data:
            if self.connected:
                # check if "Connected" state appears
                self.step_data = self.connected_state.wait.exists(timeout=self.wait_time)
            else:
                # check if "Connected" state disappears
                self.step_data = self.connected_state.wait.gone(timeout=self.wait_time)
        return self.step_data


class CheckInternetAccessServiceState(BtStep):
    """ Description:
            Validates the Internet Access profile using netcfg and ping commands.
            If required state is OFF, it checks that the bt-pan interface is not
            present, if required state is ON, it checks that the DUT has internet
            connection. Call this with not any other sources of Internet available
            on the device(i.e. before calling this, Disable Wi-Fi etc)
            If max_checks>1, it will wait till the internet connection through
            PAN interface reaches required state (this is recommended to be used
            immediately after Internet access service was checked/unchecked).
        Usage:
            bluetooth_steps.CheckInternetAccessServiceState(serial=serial,
                            state="ON", test_page = "www.google.com", max_checks=1, version=version)()
    """

    def __init__(self, state="ON", test_page="www.google.com", max_checks=1, **kwargs):
        """
        :param state: required state to be checked
        :param test_page: web page to be ping-ed as a test
        :param max_checks: maximum number of checks, to be used for scenarios when you must wait for required state
        :param kwargs: serial, version, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.state = state
        self.test_page = test_page
        self.max_checks = max_checks
        self.step_data = False
        self.counter = 0
        if self.state not in ['ON', 'OFF']:
            raise Exception("Config error: not expected value for state parameter")

    def do(self):
        if self.version.startswith("5."):
            # LLP version
            # if required state is ON
            if self.state == "ON":
                # perform check while required state not reached
                while not self.step_data and self.counter < self.max_checks:
                    try:
                        self.counter += 1
                        # check bt-pan interface present in netcfg
                        outp = self.adb_connection.cmd("shell netcfg").communicate()[0].decode("utf-8")
                        netcfg = [line for line in outp.splitlines() if "bt" in line]
                        if not netcfg:
                            raise Exception("bt-pan interface is not in netcfg output")
                        if not len(netcfg) == 1:
                            raise Exception("More than 1 bt-pan interface was found: " + str(netcfg))
                        netcfg = " ".join(netcfg[0].split())

                        # check the state of bt-pan interface
                        bt_pan_state = None
                        try:
                            bt_pan_state = netcfg.split(' ')[1]
                        finally:
                            if not bt_pan_state:
                                raise Exception("Unexpected output for cmd 'netcfg | grep bt' :" + str(netcfg))
                        if not bt_pan_state == "UP":
                            raise Exception("bt-pan interface is " + str(bt_pan_state))

                        # get the ip of bt-pan interface
                        ip = None
                        try:
                            ip = netcfg.split(' ')[2].split('/')[0]
                        finally:
                            if not ip:
                                raise Exception("Unexpected output for cmd 'netcfg | grep bt' :" + str(netcfg))

                        # check ip in routing table
                        routing_table = self.adb_connection.cmd("shell ip r").communicate()[0].decode("utf-8").strip()
                        if not routing_table:
                            raise Exception("Empty output for cmd 'ip r'")
                        if ip not in routing_table:
                            raise Exception(str(ip) + " ip not in routing table: " + str(routing_table))

                        # check internet connection by pinging test page
                        ping_output = self.adb_connection.cmd("shell ping -w2 " + str(self.test_page)).communicate()[
                            0].decode("utf-8")
                        if "2 packets transmitted, 2 received" not in ping_output:
                            raise Exception("Empty output for cmd 'ping -w2 " + str(
                                    self.test_page) + " | grep '2 packets transmitted, 2 received''")
                        # if not any exception was raised, set result to true, means that connected state was reached
                        self.step_data = True
                    except Exception, e:
                        self.set_errorm("After " + str(self.counter) + " check(s)", e.message)
                        if self.counter < self.max_checks:
                            time.sleep(3)

            # if required state is off
            else:
                # perform check while required state not reached
                while not self.step_data and self.counter < self.max_checks:
                    try:
                        self.counter += 1
                        # check bt-pan interface not present in netcfg
                        netcfg = self.adb_connection.cmd("shell netcfg").communicate()[0].decode("utf-8")
                        if "bt" in netcfg:
                            raise Exception("bt-pan interface is in netcfg output: " + str(netcfg))
                        # if not any exception was raised, set result to true, means that not connected state was
                        # reached
                        self.step_data = True
                    except Exception, e:
                        self.set_errorm("After " + str(self.counter) + " check(s)", e.message)
                        if self.counter < self.max_checks:
                            time.sleep(3)
        else:
            # M, N versions
            # if required state is ON
            if self.state == "ON":
                # perform check while required state not reached
                while not self.step_data and self.counter < self.max_checks:
                    try:
                        self.counter += 1
                        # check bt-pan interface present in ifconfig
                        outp = self.adb_connection.cmd("shell ifconfig bt-pan").communicate()[0].decode("utf-8")
                        if not outp or "No such device" in outp:
                            raise Exception("bt-pan interface is not present")

                        # get the ip of bt-pan interface
                        ip_identif = "inet addr:"
                        if ip_identif not in outp:
                            raise Exception("No IP assigned for bt-pan,output for cmd ifconfig bt-pan: " + outp)
                        try:
                            ip = outp[(outp.index(ip_identif) + len(ip_identif)):]
                            ip = ip[:ip.index("  ")]
                        except Exception:
                            raise Exception("Unexpected output for cmd ifconfig bt-pan: " + outp)
                        if not ip or not re.match("\d{1,3}.\d{1,3}.\d{1,3}.\d{1,3}", ip):
                            raise Exception("IP not correctly computed, output for cmd ifconfig bt-pan: " + outp)

                        # check ip in routing table
                        routing_table = self.adb_connection.cmd("shell ip r").communicate()[0].decode("utf-8").strip()
                        if not routing_table:
                            raise Exception("Empty output for cmd 'ip r'")
                        if ip not in routing_table:
                            raise Exception(str(ip) + " ip not in routing table: " + str(routing_table))

                        # check internet connection by pinging test page
                        ping_output = self.adb_connection.cmd("shell ping -c 2 " + str(self.test_page)).communicate()[
                            0].decode("utf-8")
                        if "2 packets transmitted, 2 received" not in ping_output:
                            raise Exception("Empty output for cmd 'ping -w2 " + str(
                                    self.test_page) + " | grep '2 packets transmitted, 2 received''")
                        # if not any exception was raised, set result to true, means that connected state was reached
                        self.step_data = True
                    except Exception, e:
                        self.set_errorm("After " + str(self.counter) + " check(s)", e.message)
                        if self.counter < self.max_checks:
                            time.sleep(3)

            # if required state is off
            else:
                # perform check while required state not reached
                while not self.step_data and self.counter < self.max_checks:
                    try:
                        self.counter += 1
                        # check bt-pan interface not present in ifconfig
                        outp = self.adb_connection.cmd("shell ifconfig bt-pan").communicate()[0].decode("utf-8")
                        if self.version.startswith("6.0"):
                            if "No such device" not in outp:
                                raise Exception("bt-pan interface is not down, ifconfig bt-pan output: " + outp)
                        else:
                            if len(outp) != 0:
                                raise Exception("bt-pan interface is not down, ifconfig bt-pan output: " + outp)
                        # if not any exception was raised, set result to true, means that not connected state was
                        # reached
                        self.step_data = True
                    except Exception, e:
                        self.set_errorm("After " + str(self.counter) + " check(s)", e.message)
                        if self.counter < self.max_checks:
                            time.sleep(3)

    def check_condition(self):
        """
        :return: True if bt pan interface has reached required state, False if not
        """
        if self.step_data:
            if self.state == "ON":
                self.set_passm(
                        "Internet access profile connected, BT-Pan interface present, ping output correct after " + str(
                                self.counter) + " check(s)")
            else:
                self.set_passm(
                        "Internet access profile disconnected, BT-Pan interface not present after " + str(
                                self.counter) + " check(s)")
        return self.step_data


class CheckInternetConnection(BtStep):
    """ Description:
            Validates that the device has a stable internet connection(by default,
            it pings www.google.com). Use for device that shares internet
            connection to other device through BT tethering
        Usage:
            bluetooth_steps.CheckInternetConnection(serial=serial,
                                                    test_page = "www.google.com")()
    """

    def __init__(self, test_page="www.google.com", **kwargs):
        """
        :param test_page: web page to be ping-ed as a test
        :param kwargs: serial, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.test_page = test_page
        self.ping_output = None
        self.step_data = True
        self.set_passm("Device has internet access, " + self.test_page + " pinged successfully")
        self.set_errorm("Ping " + self.test_page, "No Internet connection")

    def do(self):
        try:
            # check internet connection by pinging test_page
            self.ping_output = self.adb_connection.cmd("shell ping -c 2 " + str(self.test_page)).communicate()[
                0].decode("utf-8")
        except Exception, e:
            self.set_errorm("Ping " + self.test_page, e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if device has internet connection, False if not
        """
        if not self.ping_output:
            self.step_data = False
        else:
            if "2 packets transmitted, 2 received" not in self.ping_output:
                self.step_data = False
        return self.step_data


class BtDisconnectService(BtStep):
    """ Description:
            Disconnect device when connected to profiles (by tapping on device
            name in the bt list). Call this in the bt list, for a device connected
            to services
        Usage:
            bluetooth_steps.BtDisconnectService(serial=serial,
                                        device_name="DEV_name")()
    """

    def __init__(self, device_name, **kwargs):
        """
        :param device_name: name of the paired device to be disconnected
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.device_name = device_name
        self.step_data = True
        self.set_passm(str(self.device_name) + " disconnected")
        self.set_errorm("Disconnect device " + str(self.device_name), "Not disconnected")

    def do(self):
        try:
            # check if already connected (this also scrolls for the device)
            if not WaitForState(serial=self.serial, device_name=self.device_name, connected=True,
                                timeout=self.timeout, wait_time=0, version=self.version, critical=False, no_log=True)():
                raise Exception("Device " + str(self.device_name) + " is not already connected")
            # click on it
            self.uidevice(text=self.device_name).click()
            alert_dialog = self.uidevice(resourceId="android:id/alertTitle")
            if not alert_dialog.wait.exists(timeout=self.timeout):
                raise Exception("Confirmation alert dialog was not opened")
            # tap OK
            ok_button = self.uidevice(text="OK")
            if not ok_button.wait.exists(timeout=self.timeout):
                raise Exception("OK confirmation button not found")
            ok_button.click()
            if not alert_dialog.wait.gone(timeout=self.timeout):
                raise Exception("Confirmation alert dialog not gone after click on OK")
            # check if device remains paired after disconnect
            if not CheckIfPaired(serial=self.serial, dev_paired_with=self.device_name, paired=True,
                                 timeout=self.timeout, version=self.version, critical=False)():
                raise Exception("Not paired anymore  after disconnecting")
        except Exception, e:
            self.set_errorm("Disconnect device " + str(self.device_name), e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if devices are disconnected, False if not
        """
        if self.step_data:
            # check if devices are disconnected
            self.step_data = WaitForState(serial=self.serial, device_name=self.device_name, connected=False,
                                          timeout=self.timeout, wait_time=self.timeout, version=self.version,
                                          critical=False, no_log=True)()
        return self.step_data


class BtOppSharePhotosFile(BtStep):
    """ Description:
            Share picture/video via bluetooth from the Photos app. Call this in
            the required photo's screen/video's screen already opened.
            Note that this TS lets your device in the Home screen
        Usage:
            bluetooth_steps.BtOppSharePhotosFile(serial=serial,
                            server_dut="device_name_to_send_to",
                            bt_already_opened=True, scan_timeout=60000, version=version)()
    """

    def __init__(self, server_dut, bt_already_opened=True, scan_timeout=60000, **kwargs):
        """
        :param server_dut: name of the device to share picture/video with
        :param bt_already_opened: True means that BT is opened. False it also checks the appearance of 'Turning BT on'
        :param scan_timeout: max time to wait for BT scanning progress
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.server_dut = server_dut
        self.bt_already_opened = bt_already_opened
        self.scan_timeout = scan_timeout
        self.step_data = True
        self.set_passm("Bluetooth share sending file initiated")

    def do(self):
        try:
            share_button = self.uidevice(description="Share")

            # check if in a photo view
            if not self.uidevice(resourceId="com.google.android.apps.photos:id/photo_view_pager").wait.exists(
                    timeout=self.timeout):
                raise Exception("Not in a photo screen")
            # click on share button
            if not share_button.wait.exists(timeout=self.timeout):
                raise Exception("Share button not found")
            share_button.click()
            if not self.uidevice(resourceId="com.google.android.apps.photos:id/application_grid").wait.exists(
                    timeout=self.timeout):
                if not self.uidevice(resourceId="com.google.android.apps.photos:id/share_sheet").wait.exists(
                    timeout=self.timeout):
                    raise Exception("Share chooser option not shown after pressing Share")

            # click on Bluetooth option
            bluetooth_option = self.uidevice(text="Bluetooth")
            if not bluetooth_option.wait.exists(timeout=self.timeout):
                raise Exception("Bluetooth as Share option not exists")
            bluetooth_option.click()

            if self.version.startswith("5.") or self.version.startswith("6.0"):
                # LLP, M versions
                #  enable bluetooth if not already
                if not self.bt_already_opened:
                    if not self.uidevice(packageName="com.android.bluetooth").wait.exists(timeout=self.timeout):
                        raise Exception("Turn on bluetooth now? window not shown")
                    turn_on_button = self.uidevice(text="Turn on")
                    if not turn_on_button.wait.exists(timeout=self.timeout):
                        raise Exception("Turn on bluetooth button not shown")
                    turn_on_button.click()
                # wait for bt list to appear, and for scanning progress to finish
                window_title_obj = self.uidevice(text="Choose Bluetooth device")
                if not window_title_obj.wait.exists(timeout=self.timeout):
                    raise Exception("Choose Bluetooth devices list not displayed")
                # wait for scanning progress to finish
                scan_progress_obj = self.uidevice(resourceId="com.android.settings:id/scanning_progress")
                if scan_progress_obj.wait.exists(timeout=5000):
                    if not scan_progress_obj.wait.gone(timeout=self.scan_timeout):
                        raise Exception("Timeout reached, still scanning after " + str(self.scan_timeout))
                bt_list = self.uidevice(resourceId="android:id/list")
                if not bt_list.wait.exists(timeout=self.timeout):
                    raise Exception("BT devices list not shown")
                # click on required device name to share with
                if not bt_list.scroll.to(text=self.server_dut):
                    raise Exception(str(self.server_dut) + " not found in bluetooth devices list")
                self.uidevice(text=self.server_dut).click()
                if not window_title_obj.wait.gone(timeout=self.timeout):
                    raise Exception("Bluetooth devices list not closed after choosing device to send to")
            else:
                # N version
                # enable bluetooth if not already
                if not self.bt_already_opened:
                    if not self.uidevice(packageName="com.android.bluetooth").wait.exists(timeout=self.timeout):
                        raise Exception("Turn on bluetooth now? window not shown")
                    turn_on_button = self.uidevice(text="TURN ON")
                    if not turn_on_button.wait.exists(timeout=self.timeout):
                        raise Exception("TURN ON bluetooth button not shown")
                    turn_on_button.click()
                # wait for bt list to appear, and for scanning progress to finish
                window_title_obj = self.uidevice(text="Choose Bluetooth device")
                if not window_title_obj.wait.exists(timeout=self.timeout):
                    raise Exception("Choose Bluetooth devices list not displayed")
                # wait for scanning progress to finish
                scan_progress_obj = self.uidevice(resourceId="com.android.settings:id/scanning_progress")
                if scan_progress_obj.wait.exists(timeout=5000):
                    if not scan_progress_obj.wait.gone(timeout=self.scan_timeout):
                        raise Exception("Timeout reached, still scanning after " + str(self.scan_timeout))
                bt_list = self.uidevice(resourceId="com.android.settings:id/list")
                if not bt_list.wait.exists(timeout=self.timeout):
                    raise Exception("BT devices list not shown")
                # click on required device name to share with
                if not bt_list.scroll.to(text=self.server_dut):
                    raise Exception(str(self.server_dut) + " not found in bluetooth devices list")
                self.uidevice(text=self.server_dut).click()
                if not window_title_obj.wait.gone(timeout=self.timeout):
                    raise Exception("Bluetooth devices list not closed after choosing device to send to")

            # press home
            if not PressHome(serial=self.serial, timeout=self.timeout, version=self.version, critical=False,
                             no_log=True)():
                raise Exception("Press home failed")
            # check the bluetooth sharing notification
            if not CheckTransferInitiated(serial=self.serial, timeout=self.timeout, version=self.version,
                                          critical=False, no_log=True)():
                raise Exception("Check Bluetooth sharing initiated failed")

        except Exception, e:
            self.set_errorm("Share file", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if sharing was initiated, False if not
        """
        return self.step_data


class BtCheckNotificationAppear(BtStep):
    """ Description:
            Checks if a Bluetooth notification appeared (searching for a
            textContains selector). You have two options: click on
            notification (and validates that the notification menu is gone),
            or only check if appeared. Call this with notification menu
            already opened
        Usage:
            bluetooth_steps.BtCheckNotificationAppear(serial=serial,
                        text_contains="text_contained_into_notification_title",
                        click_on_notification=False, time_to_appear=60000)()
    """

    def __init__(self, text_contains, click_on_notification=False, time_to_appear=60000, **kwargs):
        """
        :param text_contains: text contained in the notification to check
        :param click_on_notification: True-click on notification. False-only check
        :param time_to_appear: max time to wait till notification appears
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.text_contains = text_contains
        self.click_on_notification = click_on_notification
        self.time_to_appear = time_to_appear
        self.step_data = True
        self.notification = self.uidevice(resourceId="android:id/notification_main_column").child(
                textContains=self.text_contains)
        if self.click_on_notification:
            self.set_passm("Notification '" + str(
                    self.text_contains) + "' found, clicked on it successful")
        else:
            self.set_passm("Notification '" + str(self.text_contains) + "' found")

    def do(self):
        try:
            # check if notification appeared
            if not self.notification.wait.exists(timeout=self.time_to_appear):
                raise Exception("Notification not appeared")
            # click on the notification if required and validate that the notifications menu is gone
            if self.click_on_notification:
                self.notification.click()
                if not self.uidevice(resourceId="com.android.systemui:id/notification_stack_scroller").wait.gone(
                        timeout=self.timeout):
                    raise Exception("Notification menu not gone after click")
        except Exception, e:
            self.set_errorm("Notification " + str(self.text_contains), e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if notification was found (and was clicked if requested), False if not
        """
        return self.step_data


class BtCheckNotificationGone(BtStep):
    """ Description:
            Waits for a Bluetooth notification to be gone (searching for a
            textContains selector). Call this with notification menu
            already opened, with the required notification already displayed
        Usage:
            bluetooth_steps.BtCheckNotificationGone(serial=serial,
                        text_contains="text_contained_into_notification_title",
                        time_to_wait=60000)()
    """

    def __init__(self, text_contains, time_to_wait=60000, **kwargs):
        """
        :param text_contains: text contained in the desired notification
        :param time_to_wait: max time to wait till notification is gone
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.text_contains = text_contains
        self.time_to_wait = time_to_wait
        self.step_data = True
        self.notification = self.uidevice(resourceId="android:id/notification_main_column").child(
                textContains=self.text_contains)
        self.set_passm("Notification '" + str(self.text_contains) + "' gone")

    def do(self):
        try:
            # wait for the notification to be gone
            if not self.notification.wait.gone(timeout=self.time_to_wait):
                raise Exception("Notification not gone after " + str(self.time_to_wait))
        except Exception, e:
            self.set_errorm("Notification " + str(self.text_contains), e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if notification was gone, False if not
        """
        return self.step_data


class BtOppReceiveFile(BtStep):
    """ Description:
            Open the notification bar, clicks on the Bluetooth receiving
            file notification, accepts/declines it and verifies the storage
            for the file. Optionally, you can give as a parameter a starting
            string of the name of expected file to be received (and
            it will fail if there is some other file received). Also, if the
            'Accept incoming file?' pop-up is already on the screen, you can
            set is_confirm_window_displayed=True. Note that it is recommended
            to call this TS in the Home screen, some other screens(eg. Full
            screens) prevents opening of Notification menu
        Usage:
            bluetooth_steps.BtOppReceiveFile(serial=serial, action='Accept',
                                time_to_receive=30000, time_to_wait_finish=60000,
                                path_to_clear="/sdcard/bluetooth/",
                                filename_starting_string="",
                                is_confirm_window_displayed=False, version=version)()
    """

    def __init__(self, action='Accept', time_to_receive=30000, time_to_wait_finish=60000,
                 bluetooth_save_path="/sdcard/bluetooth/", filename_starting_string="",
                 is_confirm_window_displayed=False, check_file_received=True, **kwargs):
        """
        :param action: "Decline"/"Accept"- action to be performed when receiving file
        :param time_to_receive: max time to wait till confirm window appears
        :param time_to_wait_finish: max time to wait till transfer is finished
        :param bluetooth_save_path: default path where transferred files are saved
        :param filename_starting_string: starting string in the filename(if empty, it does not check the filename)
        :param is_confirm_window_displayed: True if the pop-up is already on the screen. False if not
        :param kwargs: serial, version, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.action = action
        self.time_to_receive = time_to_receive
        self.time_to_wait_finish = time_to_wait_finish
        self.bluetooth_save_path = bluetooth_save_path
        self.filename_starting_string = filename_starting_string
        self.is_confirm_window_displayed = is_confirm_window_displayed
        self.file_to_accept = ''
        if self.action not in ['Decline', 'Accept']:
            raise Exception("Config error: not an expected value for action")
        self.cmd_check_file = 'shell ls ' + self.bluetooth_save_path
        self.check_file_received = check_file_received
        self.step_data = True

    def do(self):
        try:
            if self.version.startswith("5.") or self.version.startswith("6.0"):
                # LLP, M versions
                if not self.is_confirm_window_displayed:
                    # open notification menu and tap on Incoming file notification
                    if not OpenNotificationsMenu(serial=self.serial, timeout=self.timeout, version=self.version,
                                                 critical=False, no_log=True)():
                        raise Exception("Notifications menu not opened")
                    if not BtCheckNotificationAppear(serial=self.serial, text_contains="Bluetooth share: Incoming file",
                                                     click_on_notification=True, time_to_appear=self.time_to_receive,
                                                     version=self.version,
                                                     critical=False, no_log=True)():
                        raise Exception("Incoming file notification click failed")
                # check if confirmation dialog is displayed
                accept_title_window = self.uidevice(text="Accept incoming file?")
                if not accept_title_window.wait.exists(timeout=self.timeout):
                    raise Exception("Confirmation dialog not appeared")
                # get received file name
                file_name = self.uidevice(resourceId="com.android.bluetooth:id/filename_content")
                if not file_name.wait.exists(timeout=self.timeout):
                    raise Exception("Receiving file name not found in the confirmation dialog")
                self.file_to_accept = str(file_name.text)
                if not self.file_to_accept:
                    raise Exception("Empty file name was found in the confirmation dialog")
                # optionally check if it was received proper filename
                if self.filename_starting_string != "":
                    if not self.file_to_accept.startswith(self.filename_starting_string):
                        raise Exception(
                                "Not the proper file received: " + self.file_to_accept + ", Exp. to start with " + str(
                                        self.filename_starting_string))
                # Accept or Decline file
                button = self.uidevice(text=self.action)
                if not button.wait.exists(timeout=self.timeout):
                    raise Exception(str(self.action) + " button not found")
                button.click()
                time.sleep(5)
                if not accept_title_window.wait.gone(timeout=self.timeout):
                    raise Exception("Confirmation dialog not disappeared after " + str(self.action) + " incoming file")
                # if declined, check that receiving file progress notification is not displayed
                if self.action == "Decline":
                    if not OpenNotificationsMenu(serial=self.serial, timeout=self.timeout, critical=False,
                                                 no_log=True)():
                        raise Exception("Notifications menu not opened")

                    if self.uidevice(resourceId="android:id/notification_main_column").child(
                            textContains="Bluetooth share: Receiving").wait.exists(timeout=2000):
                        raise Exception("Receiving file progress started, even Decline button was pressed")
                    self.uidevice.press.back()
                    if not self.uidevice(resourceId="com.android.systemui:id/notification_stack_scroller").wait.gone(
                            timeout=self.timeout):
                        raise Exception("Notification menu not gone after checking Decline action")
                # if accepted, wait for the transfer progress to finish
                elif self.action == "Accept":
                    if self.check_file_received and not \
                        BtOppCheckFileReceivedPanel(serial=self.serial,
                                                time_to_wait_finish=self.time_to_wait_finish,
                                                filename_part=self.filename_starting_string,
                                                timeout=self.timeout, version=self.version, critical=False,
                                                no_log=True)():
                        raise Exception("File transfer not finished properly")
            else:
                # N version
                if not self.is_confirm_window_displayed:
                    # open notification menu and tap on Incoming file notification
                    if not OpenNotificationsMenu(serial=self.serial, timeout=self.timeout, version=self.version,
                                                 critical=False, no_log=True)():
                        raise Exception("Notifications menu not opened")
                    if not BtCheckNotificationAppear(serial=self.serial, text_contains="Incoming file",
                                                     click_on_notification=True, time_to_appear=self.time_to_receive,
                                                     version=self.version,
                                                     critical=False, no_log=True)():
                        raise Exception("Incoming file notification click failed")
                # check if confirmation dialog is displayed
                accept_title_window = self.uidevice(text="Accept incoming file?")
                if not accept_title_window.wait.exists(timeout=self.timeout):
                    raise Exception("Confirmation dialog not appeared")
                # get received file name
                file_name = self.uidevice(resourceId="com.android.bluetooth:id/filename_content")
                if not file_name.wait.exists(timeout=self.timeout):
                    raise Exception("Receiving file name not found in the confirmation dialog")
                self.file_to_accept = str(file_name.text)
                if not self.file_to_accept:
                    raise Exception("Empty file name was found in the confirmation dialog")
                # optionally check if it was received proper filename
                if self.filename_starting_string != "":
                    if not self.file_to_accept.startswith(self.filename_starting_string):
                        raise Exception(
                                "Not the proper file received: " + self.file_to_accept + ", Exp. to start with " + str(
                                        self.filename_starting_string))
                # Accept or Decline file
                button = self.uidevice(text=self.action.upper())
                if not button.wait.exists(timeout=self.timeout):
                    raise Exception(str(self.action) + " button not found")
                button.click()
                if not accept_title_window.wait.gone(timeout=self.timeout):
                    raise Exception("Confirmation dialog not disappeared after " + str(self.action) + " incoming file")
                # if declined, check that receiving file progress notification is not displayed
                if self.action == "Decline":
                    if not OpenNotificationsMenu(serial=self.serial, timeout=self.timeout, critical=False,
                                                 no_log=True)():
                        raise Exception("Notifications menu not opened")

                    if self.uidevice(resourceId="android:id/notification_main_column").child(
                            textContains="Bluetooth share: Receiving").wait.exists(timeout=2000):
                        raise Exception("Receiving file progress started, even Decline button was pressed")
                    self.uidevice.press.back()
                    if not self.uidevice(resourceId="com.android.systemui:id/notification_stack_scroller").wait.gone(
                            timeout=self.timeout):
                        raise Exception("Notification menu not gone after checking Decline action")
                # if accepted, wait for the transfer progress to finish
                elif self.action == "Accept":
                    if self.check_file_received and not \
                        BtOppCheckFileReceivedPanel(serial=self.serial,
                                                time_to_wait_finish=self.time_to_wait_finish,
                                                filename_part=self.filename_starting_string,
                                                timeout=self.timeout, version=self.version, critical=False,
                                                no_log=True)():
                        raise Exception("File transfer not finished properly")
        except Exception, e:
            self.set_errorm(self.action + " incoming file", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if file was received (or not, depending on performed action), False otherwise
        """
        if self.step_data:
            cmd_output = self.adb_connection.cmd(self.cmd_check_file).communicate()[0].decode("utf-8").strip()
            # depending on performed action, check if file is found or not on the bluetooth save path
            # (with a 'ls' command)
            if self.action == "Accept":
                msg = "File " + self.file_to_accept + " received correctly" \
                    if self.check_file_received else "File " + \
                    self.file_to_accept + " accepted successfully"
                self.set_passm(msg)
                self.set_errorm(self.action + " incoming file", "File " + self.file_to_accept + " not received")
                self.step_data = self.file_to_accept in cmd_output
            else:
                self.set_passm("File " + self.file_to_accept + " not received")
                self.set_errorm(self.action + " incoming file", "File " + self.file_to_accept + " received")
                self.step_data = self.file_to_accept not in cmd_output
        return self.step_data


class BtOppCheckFileReceivedPanel(BtStep):
    """ Description:
            Opens the notification bar, clicks on the Bluetooth receiving file
            notification and it checks that in the file transfer pop-up the
            message 'File received' is prompted at the end of the successful
            transfer. Call this immediately after an incoming file sharing was accepted.
            Note that it is recommended to call this TS in the Home screen, some other
            screens (eg. Full screens) prevents opening of Notification menu
        Usage:
            bluetooth_steps.BtOppCheckFileReceivedPanel(serial=device_serial,
                                                time_to_wait_finish=60000, filename_part="")()
    """

    def __init__(self, time_to_wait_finish=60000, filename_part="", **kwargs):
        """
        :param time_to_wait_finish: max time to wait till transfer is finished
        :param filename_part: string, part of the filename(if empty, it does not check the filename)
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.time_to_wait_finish = time_to_wait_finish
        self.filename_part = filename_part
        self.step_data = True
        self.set_passm("File transfer process finished")

    def do(self):
        try:
            # open notification menu and click on incoming file notification
            if not OpenNotificationsMenu(serial=self.serial, timeout=self.timeout, version=self.version, critical=False,
                                         no_log=True)():
                raise Exception("Notifications menu not opened")
            if not BtCheckNotificationAppear(serial=self.serial, text_contains="Bluetooth share: Receiving",
                                             click_on_notification=True, time_to_appear=self.timeout,
                                             version=self.version, critical=False, no_log=True)():
                raise Exception("Receiving file notification click failed")
            if not self.uidevice(resourceId="android:id/alertTitle", text="File transfer").wait.exists(
                    timeout=self.timeout):
                raise Exception("File transfer progress window not shown after click on receiving file notification")
            # check the filename in the window (if not empty)
            if self.filename_part != "":
                filename_obj = self.uidevice(textContains="File:")
                if not filename_obj.wait.exists(timeout=self.timeout):
                    raise Exception("Receiving File Name not found in the file transfer window")
                filename_text = str(filename_obj.text)
                if self.filename_part not in filename_text:
                    raise Exception("Not expected string contained in the file transfer window, exp " + str(
                            self.filename_part) + " but found " + filename_text)
            # wait till the file is received
            if not self.uidevice(text="File received").wait.exists(timeout=self.time_to_wait_finish):
                raise Exception("Timeout reached, file transfer not finished")
            # return to the previous window
            self.uidevice.press.back()
            if not self.uidevice(resourceId="android:id/alertTitle", text="File transfer").wait.gone(
                    timeout=self.timeout):
                self.uidevice.press.back()
                raise Exception("File transfer progress window not closed")
        except Exception, e:
            self.set_errorm("File transfer progress", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if file transfer progress window was closed after transfer was finished, False otherwise
        """
        return self.step_data


class BtOppResendFile(BtStep):
    """ Description:
            Open the notification bar, clicks on the Bluetooth sent files
            notification, and retries to send one rejected file.
            If file_to_send string parameter is not empty, file will be
            identified using it's (part of) name, otherwise, first rejected file
            from the Outbounds transfers list will be chosen. Note that it is
            recommended to call this TS in the Home screen, some other screens
            (eg. Full screens) prevents opening of Notification menu.
            This TS let the DUT in the Outbounds folder
        Usage:
            bluetooth_steps.BtOppResendFile(serial=serial, file_to_resend="")()
    """

    def __init__(self, file_to_resend="", **kwargs):
        """
        :param file_to_resend: starting string in the filename to be resend (if empty, it chooses the first in the list)
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.file_to_resend = str(file_to_resend)
        self.step_data = True
        if self.file_to_resend != "":
            self.message_str = self.file_to_resend + "[...]"
        else:
            self.message_str = "first rejected"
        self.set_passm("Bluetooth share retry sending " + self.message_str + " file initiated")

    def do(self):
        try:
            # open notifications menu and click on Sent files notification
            if not OpenNotificationsMenu(serial=self.serial, timeout=self.timeout, critical=False, no_log=True)():
                raise Exception("Notifications menu not opened")
            if not BtCheckNotificationAppear(serial=self.serial, text_contains="Bluetooth share: Sent files",
                                             click_on_notification=True, time_to_appear=self.timeout,
                                             version=self.version, critical=False, no_log=True)():
                raise Exception("Click on Sent files notifications failed")
            if not self.uidevice(resourceId="android:id/action_bar").child(text="Outbound transfers").wait.exists(
                    timeout=self.timeout):
                raise Exception("Outbounds transfer activity not opened after click on Sent files notification")
            outbounds_list = self.uidevice(resourceId="com.android.bluetooth:id/list")
            if not outbounds_list.wait.exists(timeout=self.timeout):
                raise Exception("List in Outbounds transfer activity not found")
            if self.file_to_resend != "":
                # identify the file by it's (part of) name
                if not outbounds_list.scroll.to(textContains=self.file_to_resend):
                    raise Exception("File '" + self.message_str + "' not found in Outbounds transfers")
                element_to_click = self.uidevice(textContains=self.file_to_resend).sibling(
                        resourceId="com.android.bluetooth:id/complete_text")
                if not element_to_click.wait.exists(timeout=self.timeout):
                    raise Exception("Status for file '" + self.message_str + "' not found in Outbounds transfers")
                file_status_text = element_to_click.text
                if not file_status_text == "Transfer forbidden by target device.":
                    raise Exception(
                            "File '" + self.file_to_resend + "' does not have rejected status in Outbounds list: " +
                            file_status_text)
            else:
                # choose first rejected file from the list
                if not outbounds_list.scroll.to(text="Transfer forbidden by target device."):
                    raise Exception("Not any file with rejected status was found in Outbounds transfer")
                element_to_click = self.uidevice(text="Transfer forbidden by target device.")
            # click on the file
            element_to_click.click()
            # wait for file transfer alert and choose to Retry
            alert_title = self.uidevice(resourceId="android:id/alertTitle", text="File transfer")
            if not alert_title.wait.exists(timeout=self.timeout):
                raise Exception("File transfer window not shown, retry cannot be performed")
            try_again_button = self.uidevice(textMatches="(?i)(Try again)")
            if not try_again_button.wait.exists(timeout=self.timeout):
                raise Exception("Try again option not shown")
            try_again_button.click()
            if not alert_title.wait.gone(timeout=self.timeout):
                self.uidevice.press.back()
                raise Exception("File transfer window not closed after click on Try again")
            # search in the notification menu the sending progress notification
            if not CheckTransferInitiated(serial=self.serial, timeout=self.timeout, version=self.version,
                                          critical=False, no_log=True)():
                raise Exception("Check transfer initiated failed")

        except Exception, e:
            self.set_errorm("Resend " + self.message_str + " file", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if BT sharing was initiated, False if not
        """
        return self.step_data


class BtOppCheckStatusInTransfersList(BtStep):
    """ Description:
            Search for a file in the transfers list and check if the transfer
            status displayed is the expected one. Call this in the inbound/outbound
            transfers list
        Usage:
            bluetooth_steps.BtOppCheckStatusInTransfersList(serial=serial,
                            file_to_check=<file_name_starting_string>,
                            exp_status=<exact_text_status>, full_status=True, transfer_type="Inbound")()
    """

    def __init__(self, file_to_check, exp_status, full_status=True, transfer_type="Inbound", **kwargs):
        """
        :param file_to_check: starting string in the filename to be checked
        :param exp_status: expected status of the file
        :param full_status: set to False if exp_status is only a part of the status, set to True for full status
        :param transfer_type: "Inbound"/"Outbound" - type of the transfer list
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.file_to_check = str(file_to_check)
        self.exp_status = exp_status
        self.full_status = full_status
        self.transfer_type = transfer_type
        if self.transfer_type not in ['Inbound', 'Outbound']:
            raise Exception("Config error: not an expected value for transfer_type")
        self.step_data = True
        self.filename_message_str = self.file_to_check + "[...]"
        if self.full_status:
            self.status_message_str = self.exp_status
        else:
            self.status_message_str = "[...]" + self.exp_status + "[...]"
        self.file_status_text = ""
        self.set_passm(
                "File '" + self.filename_message_str + "' with expected status '" + self.status_message_str +
                "' found in " + self.transfer_type + " transfers")

    def do(self):
        try:
            if not self.uidevice(resourceId="android:id/action_bar").child(
                    text=self.transfer_type + " transfers").wait.exists(timeout=self.timeout):
                raise Exception(self.transfer_type + " transfers activity not found")
            transfers_list = self.uidevice(resourceId="com.android.bluetooth:id/list")
            if not transfers_list.wait.exists(timeout=self.timeout):
                raise Exception("List in " + self.transfer_type + " transfers activity not found")
            # identify the file by it's (part of) name
            if not transfers_list.scroll.to(textContains=self.file_to_check):
                raise Exception(
                        "File '" + self.filename_message_str + "' not found in " + self.transfer_type + " transfers")
            status_obj = self.uidevice(textContains=self.file_to_check).sibling(
                    resourceId="com.android.bluetooth:id/complete_text")
            if not status_obj.wait.exists(timeout=self.timeout):
                raise Exception(
                        "Status for file '" + self.filename_message_str + "' not found in " + self.transfer_type +
                        " transfers")
            self.file_status_text = status_obj.text

        except Exception, e:
            self.set_errorm(
                    "Exp '" + self.status_message_str + "' for " + self.transfer_type + " file '" +
                    self.filename_message_str + "'", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if the status is the expected one, False if not
        """
        if self.step_data:
            if self.full_status:
                self.step_data = self.exp_status == self.file_status_text
            else:
                self.step_data = self.exp_status in self.file_status_text
            self.set_errorm(
                    "Exp '" + self.exp_status + "' for " + self.transfer_type + " file '" + self.filename_message_str +
                    "'", "Not the expected status, found: " + self.file_status_text)
        return self.step_data


class CheckTransferInitiated(BtStep):
    """ Description:
            Checks in the notifications menu if the file transfer progress was initiated.
            Call this right after starting a file transfer process, on the initiator device
            Use this TS in non-full screens activities
        Usage:
            bluetooth_steps.CheckTransferInitiated(serial=serial)()
    """

    def __init__(self, **kwargs):
        """
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.step_data = True
        self.set_passm("File transfer initiated")

    def do(self):
        try:
            # search in the notification menu the sending progress notification
            if not OpenNotificationsMenu(serial=self.serial, timeout=self.timeout, version=self.version, critical=False,
                                         no_log=True)():
                raise Exception("Notifications menu not opened")
            if not BtCheckNotificationAppear(serial=self.serial, text_contains='Bluetooth share: Sending ',
                                             click_on_notification=False, time_to_appear=self.timeout,
                                             version=self.version, critical=False, no_log=True)():
                raise Exception("Bluetooth sharing not initiated")
            time.sleep(1)
            self.uidevice.press.back()
            if not self.uidevice(resourceId="com.android.systemui:id/notification_stack_scroller").wait.gone(
                    timeout=self.timeout):
                raise Exception("Notifications panel not closed")

        except Exception, e:
            self.set_errorm("File transfer", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if BT sharing was initiated, False if not
        """
        return self.step_data


class BtOppGetNotificationNumberUpdate(BtStep):
    """ Description:
            Swipes down to open the notification bar and counts the current successful or
            unsuccessful OPP file transfers.
            If count_successful=True, then it returns the value of the successful transfers
            If count_unsuccessful=True, then it returns the value of the successful transfers
            If the notification menu is already opened, set notif_menu_already_opened to True
        Usage:
            count_value = bluetooth_steps.BtOppGetNotificationNumberUpdate(serial=device_serial,
                                    share_type=Received/Sent, count_successful=bool,
                                    count_unsuccessful=bool, notif_menu_already_opened=False)()
    """

    def __init__(self, share_type="Sent", count_successful=True, count_unsuccessful=False,
                 notif_menu_already_opened=False, **kwargs):
        """
        :param share_type: "Sent"/"Received" share type to count in notification menu
        :param count_successful: True to count successful transfers
        :param count_unsuccessful: True to count unsuccessful transfers
        :param notif_menu_already_opened: True, it assumes that the notification menu is already opened
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.share_type = share_type
        self.count_successful = count_successful
        self.count_unsuccessful = count_unsuccessful
        self.notif_menu_already_opened = notif_menu_already_opened
        if self.share_type is "Sent":
            self.notification_title = "Bluetooth share: Sent files"
        elif self.share_type is "Received":
            self.notification_title = "Bluetooth share: Received files"
        else:
            raise Exception("Config error, not any of expected value for share_type")
        if self.count_successful == self.count_unsuccessful or True not in [self.count_successful,
                                                                            self.count_unsuccessful]:
            raise Exception("Config error,(only)one of count_successful/count_unsuccessful must be True")
        if self.count_successful:
            self.string_to_search = " successful"
        else:
            self.string_to_search = " unsuccessful"
        self.step_data = -1
        self.global_verdict = True

    def do(self):
        try:
            # open notification menu if not already
            if not self.notif_menu_already_opened:
                if not OpenNotificationsMenu(serial=self.serial, timeout=self.timeout, version=self.version,
                                             critical=False, no_log=True)():
                    raise Exception("Notifications menu not opened")
            # check if required(Sent/Received) notification is available
            if not BtCheckNotificationAppear(serial=self.serial, text_contains=self.notification_title,
                                             click_on_notification=False, time_to_appear=self.timeout,
                                             version=self.version, critical=False, no_log=True)():
                raise Exception("Find notification " + self.notification_title + " failed")
            # get notification in order to obtain it's status
            try:
                notification_container = self.uidevice(
                        resourceId="com.android.systemui:id/notification_stack_scroller").child_by_text(
                        self.notification_title, resourceId="android:id/status_bar_latest_event_content")
            except:
                notification_container = None
            if not notification_container:
                raise Exception("Notification " + self.notification_title + " container not found")
            status_obj = notification_container.child(resourceId="android:id/text")
            if not status_obj.wait.exists(timeout=self.timeout):
                raise Exception("Status counting files not found in " + self.share_type + " notification")
            # get text of the status and parse it in order to obtain required number
            number_update_string = status_obj.text
            if not number_update_string:
                raise Exception("Empty text was found in status of " + self.share_type + " notification")
            if self.string_to_search not in number_update_string:
                raise Exception(
                        "Counter for " + self.string_to_search + " not found in " + self.share_type + " notification")
            status_string = number_update_string.strip('.').split(', ')
            check_string = None
            for obj in status_string:
                if self.string_to_search in obj:
                    check_string = obj
            if not check_string:
                raise Exception("Something went wrong when parsing status: " + number_update_string)
            self.step_data = int(check_string.split(' ')[0])
            # close notification menu if wasn't already opened
            if not self.notif_menu_already_opened:
                time.sleep(1)
                self.uidevice.press.back()
                if not self.uidevice(resourceId="com.android.systemui:id/notification_stack_scroller").wait.gone(
                        timeout=self.timeout):
                    raise Exception("Notifications panel not closed")

        except Exception, e:
            self.set_errorm("Get " + self.share_type + " files" + self.string_to_search, e.message)
            self.global_verdict = False

    def check_condition(self):
        """
        :return: True if number was found, False if not. Note that the number of files is saved in step_data
        """
        self.set_errorm("Get " + self.share_type + " files" + self.string_to_search,
                        "Something went wrong, files  count: " + str(self.step_data))
        if self.global_verdict:
            if self.step_data >= 0:
                self.set_passm("Found on notification - " + self.share_type + " files" + self.string_to_search +
                               " " + str(self.step_data))
                return True
            else:
                return False
        else:
            return False


class BtOppNotificationUpdateCompare(BtStep):
    """ Description:
            This method compares the before (reference) and after (obtained) values of OPP status count values.
            the result is pass if the obtained value is higher that the initial one by the value of
            the 'increment' parameter
        Usage:
            bluetooth_steps.BtOppNotificationUpdateCompare(serial=device_serial,
                                            before_value=<int>, after_value=<int>, increment=<int>)()
     """

    def __init__(self, before_value, after_value, increment, **kwargs):
        """
        :param before_value: value before file transfer(s)
        :param after_value: value after file transfer(s)
        :param increment: expected increment
        :param kwargs: serial, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.before_value = before_value
        self.after_value = after_value
        self.increment = increment
        self.set_passm("Notifications counter updated correctly, " + str(
                self.before_value) + " was increased with " + str(self.increment))
        self.set_errorm("Expected: " + str(self.before_value + self.increment),
                        "Notifications counter not updated correctly, found: " + str(self.after_value))

    def do(self):
        # do nothing, only check
        pass

    def check_condition(self):
        """
        :return: True if values were incremented correctly, False if not.
        """
        return self.after_value == self.before_value + self.increment


class ClearAllNotifications(BtStep):
    """ Description:
            Opens the notification bar and clears all the list of notifications.
            After that, it opens again the notification list and checks that
            not any Sent/Received files notifications are available.
        Usage:
            bluetooth_steps.ClearAllNotifications(serial=serial,
                                                max_flings=20)()
    """

    def __init__(self, max_flings=20, **kwargs):
        """
        :param max_flings: max number of flings till end of notifications menu is reached
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.max_flings = max_flings
        self.step_data = True
        self.set_passm("All notifications cleared")

    def do(self):
        try:
            # open notifications menu
            if not OpenNotificationsMenu(serial=self.serial, timeout=self.timeout, version=self.version, critical=False,
                                         no_log=True)():
                raise Exception("Notifications menu not opened")
            notifications_scroller = self.uidevice(resourceId="com.android.systemui:id/notification_stack_scroller")
            clear_button = self.uidevice(description="Clear all notifications.")
            counter = 0
            # force a small delay till the clear button appears
            time.sleep(1)
            # scroll until the clear all button is displayed
            while notifications_scroller.exists and not clear_button.exists and counter < self.max_flings:
                notifications_scroller.scroll.vertical.forward()
                counter += 1

            # if the clear button does not exists(i.e. Notifications menu does not contain any notification) the fling
            # forward methods previously called will close the Notification menu(the last performed fling closes the
            # menu), so the next methods will be called only if the Notification menu is still displayed on the screen
            if self.uidevice(packageName="com.android.systemui").exists:
                if clear_button.wait.exists(timeout=1000):
                    clear_button.click()
                    if not clear_button.wait.gone(timeout=self.timeout):
                        raise Exception("Notifications could not be cleared")
                else:
                    self.uidevice.press.back()
            else:
                self.set_passm("No notifications to clear")
            if not notifications_scroller.wait.gone(timeout=self.timeout):
                raise Exception("Notifications menu not closed after clearing all")
        except Exception, e:
            self.set_errorm("Notifications clear", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: False if something went wrong, True otherwise
        """
        return self.step_data


class BtOppDismissEventualIncoming(BtStep):
    """ description:
            Opens the notification bar and clears all the Incoming file notifications
            if exists by click on it and on Dismiss button. To be used for cleanup
        Usage:
            bluetooth_steps.BtOppDismissEventualIncoming(serial=serial,
                                                        max_retries=5, version=version)()
    """

    def __init__(self, max_retries=5, **kwargs):
        """
        :param max_retries: max number of retries
        :param kwargs: serial, version, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.max_retries = max_retries
        self.step_data = True
        self.set_passm("Not any incoming transfer found")

    def do(self):
        try:
            # open notifications menu
            if self.version.startswith("5.") or self.version.startswith("6.0"):
                # LLP, M versions
                notification = self.uidevice(resourceId="android:id/notification_main_column").child(
                        textContains="Bluetooth share: Incoming file")
            else:
                # N version
                notification = self.uidevice(resourceId="android:id/notification_main_column").child(
                        textContains="Incoming file")

            counter = 0
            # while notification exists, click on it and dismiss
            while counter < self.max_retries:
                message_str = "(try " + str(counter + 1) + ")"
                if not OpenNotificationsMenu(serial=self.serial, timeout=self.timeout, version=self.version,
                                             critical=False, no_log=True)():
                    raise Exception("Notifications menu not opened " + message_str)
                # if incoming notification not exists, close the menu and break the loop
                if not notification.wait.exists(timeout=1000):
                    # close the notifications menu
                    self.uidevice.press.back()
                    if not self.uidevice(resourceId="com.android.systemui:id/notification_stack_scroller").wait.gone(
                            timeout=self.timeout):
                        raise Exception("Notification menu not closed after " + str(counter + 1) + " tries")
                    break
                self.set_passm(str(counter + 1) + " Incoming transfer(s) dismissed")
                notification.click()
                alert_title_obj = self.uidevice(text="Accept incoming file?")
                if not alert_title_obj.wait.exists(timeout=self.timeout):
                    raise Exception("Confirmation dialog not appeared " + message_str)
                # Decline the file
                button = self.uidevice(textMatches="(?i)(Decline)")
                if not button.wait.exists(timeout=self.timeout):
                    raise Exception("Decline button not found " + message_str)
                button.click()
                if not alert_title_obj.wait.gone(timeout=self.timeout):
                    raise Exception(
                            "Confirmation dialog not disappeared after Dismiss incoming file " + message_str)
                counter += 1

        except Exception, e:
            self.set_errorm("Cleanup incoming", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if incoming notifications were dismissed, False if not.
        """
        return self.step_data


class ClearPath(BtStep):
    """ Description:
            It removes the files at the provided path. Default path
            is "/sdcard/bluetooth/". If filename parameter is empty,
            all the files in the path_to_clear will be removed,
            otherwise only the provided filename will be removed
        Usage:
            bluetooth_steps.ClearPath(serial=serial,
                    path_to_clear="/sdcard/bluetooth/")()
    """

    def __init__(self, path_to_clear="/sdcard/bluetooth/", filename="", **kwargs):
        """
        :param path_to_clear: path to clear; default is bluetooth default path where received files are stored
        :param filename: if provided, only the provided file name will be removed in the path_to_clear
        :param kwargs: serial, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.path_to_clear = str(path_to_clear)
        self.filename = str(filename)
        self.output = None
        self.step_data = True
        self.set_passm(self.path_to_clear + self.filename + " cleared successfully")

    def do(self):
        try:
            if self.filename:
                remove_command = "shell rm " + self.path_to_clear + self.filename
                check_command = "shell ls " + self.path_to_clear + self.filename
            else:
                remove_command = "shell rm " + self.path_to_clear + "*"
                check_command = "shell ls " + self.path_to_clear
            self.adb_connection.cmd(remove_command).wait()
            self.output = self.adb_connection.cmd(check_command).communicate()[0].decode("utf-8").strip()
        except Exception, e:
            self.set_errorm("Exception encountered when clearing " + self.path_to_clear + self.filename,
                            e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if path was cleared, False if not.
        """
        if self.step_data:
            self.set_errorm("Path " + self.path_to_clear + self.filename,
                            "Not cleared successfully, output: " + self.output)
            self.step_data = self.output == "" or "No such file or directory" in self.output
        return self.step_data


class BtOpenFromRecent(BtStep):
    """ Description:
            Opens a certain activity/app from the recent apps list, by searching
            for a text contained in it's title. Call this for recent app/activity
            just opened (it does not scroll the list). Better practice: check
            outside this TS that the expected activity was indeed opened.
        Usage:
            bluetooth_steps.BtOpenFromRecent(serial=serial,
                                        text_contains="tethering")()
    """

    def __init__(self, text_contains="Tethering", **kwargs):
        """
        :param text_contains: text contained into recent view title
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.text_contains = text_contains
        self.step_data = True
        self.set_passm("Title containing " + str(self.text_contains) + " found, clicked successful")

    def do(self):
        try:
            if self.version.startswith("5.") or self.version.startswith("6.0"):
                # LLP, M versions
                self.uidevice.press.recent()
                if not self.uidevice(resourceId="com.android.systemui:id/recents_view").wait.exists(
                        timeout=self.timeout):
                    raise Exception("Recent apps list not opened")
                title_to_click = self.uidevice(resourceId="com.android.systemui:id/activity_description",
                                               textContains=self.text_contains)
                if not title_to_click.wait.exists(timeout=self.timeout):
                    raise Exception("Title not found in recent apps")
                title_to_click.click()
                if not self.uidevice(resourceId="com.android.systemui:id/recents_view").wait.gone(timeout=self.timeout):
                    raise Exception("Recent apps list not gone after click on the title")
            else:
                # TODO N version
                raise Exception("Version not implemented")
        except Exception, e:
            self.set_errorm("Click title containing " + str(self.text_contains), e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if recent apps list was gone after click, False if not.
        """
        return self.step_data


class OpenFileInPhotos(BtStep):
    """ Description:
            Open in Photos app a photo/video using intent. You should stop Photos app first
            (to avoid failures)
        Usage:
            bluetooth_steps.OpenFileInPhotos(serial=dut_serial,
                                        local_folder_name=<full_path>, open_video=False)()
    """

    def __init__(self, file_full_path, open_video=False, **kwargs):
        """
        :param file_full_path: complete path to the file to open (eg: "/sdcard/Bluetooth/img.jpg")
        :param open_video: True means that provided file is a video, False means that provided file is photo
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.photo_full_path = str(file_full_path)
        self.open_video = open_video
        self.step_data = True
        # part of logging message
        if self.open_video:
            self.message_str = "Video " + self.photo_full_path
        else:
            self.message_str = "Photo " + self.photo_full_path
        self.set_passm(self.message_str + " opened")

    def do(self):
        try:
            if self.open_video:
                type_cmd = "video/*"
            else:
                type_cmd = "image/*"

            launch_command = "shell am start -a android.intent.action.VIEW -d file://" + self.photo_full_path + \
                             " -t " + type_cmd + " -p com.google.android.apps.photos"

            # launch the Photos app to the required photo/video
            self.adb_connection.cmd(launch_command).wait()
            time.sleep(3)
            backup_btn = self.uidevice(className="android.widget.Button", text="SIGN IN TO BACK UP")
            if backup_btn.exists:
                self.uidevice.press.back()
                if self.uidevice(text="Keep backup off?").wait.exists(timeout=self.timeout):
                    if self.uidevice(text="KEEP OFF").wait.exists(timeout=self.timeout):
                        self.uidevice(text="KEEP OFF").click.wait()
                    else:
                        raise Exception("Photos KEEP OFF not show")
                else:
                    raise Exception("Photos Keep backup off not show")
            allow_btn = self.uidevice(className="android.widget.Button", text="Allow")
            if allow_btn.exists:  # click out 'ALLOW' botton
                allow_btn.click.wait()

            if not self.uidevice(packageName="com.google.android.apps.photos").wait.exists(timeout=self.timeout):
                raise Exception("Photos app not opened")
            # check if photo viewer is opened
            photo_view_pager_obj = self.uidevice(resourceId="com.google.android.apps.photos:id/photo_view_pager")
            if not photo_view_pager_obj.wait.exists(timeout=self.timeout):
                raise Exception("Photo view pager not found")
            photo_action_bar_obj = self.uidevice(resourceId="com.google.android.apps.photos:id/photo_action_bar")
            # video starts playing, we must reveal the action bar
            if self.open_video:
                # force a small delay, wait the video to start playing
                time.sleep(1)
                if photo_action_bar_obj.wait.gone(timeout=1000):
                    photo_view_pager_obj.click()
            # check if action bar is not hidden
            if not photo_action_bar_obj.wait.exists(timeout=self.timeout):
                raise Exception("File not opened correctly, action bar not found")
            # check can't load file error
            if self.uidevice(resourceId="com.google.android.apps.photos:id/error_text").exists or self.uidevice(
                    resourceId="com.google.android.apps.photos:id/photos_videoplayer_list_empty_text").exists:
                raise Exception("File cannot be loaded in Photos app")

        except Exception, e:
            self.set_errorm(self.message_str, e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if photo viewer was opened successfully, False if not.
        """
        return self.step_data


class NavigateToFileInPhotos(BtStep):
    """ Description:
            Open in Photos app a photo/video by navigating through the menus.
            You should stop Photos app first (to avoid failures) and with
            no Google account added. Use this with a single file in an unique local
            folder (Ui limitation- Photos app does not have unique identifiers)
        Usage:
            bluetooth_steps.NavigateToFileInPhotos(serial=dut_serial,
                            local_folder_name=<folder_name>, open_video=False, bypass_tutorial=True)()
    """

    def __init__(self, local_folder_name, open_video=False, bypass_tutorial=True, **kwargs):
        """
        :param local_folder_name: name of the unique name folder containing a single photo/video to open
        :param open_video: True means that provided file is a video, False means that provided file is photo
        :param bypass_tutorial: set to True to bypass Photos firstrun tutorial, set to false otherwise
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.local_folder_name = str(local_folder_name)
        self.open_video = open_video
        self.bypass_tutorial = bypass_tutorial
        self.step_data = True
        # part of logging message and file frame identifier in Photos
        if self.open_video:
            self.message_str = "Video from folder " + self.local_folder_name
            self.file_identifier = "Video"
        else:
            self.message_str = "Photo from folder " + self.local_folder_name
            self.file_identifier = "Photo"
        self.set_passm(self.message_str + " opened")

    def do(self):
        try:

            launch_command = "shell am start -c android.intent.category.LAUNCHER -p com.google.android.apps.photos"

            # launch the Photos app main activity with intent
            self.adb_connection.cmd(launch_command).wait()
            time.sleep(3)
            backup_btn = self.uidevice(className="android.widget.Button", text="SIGN IN TO BACK UP")
            if backup_btn.exists:
                self.uidevice.press.back()
                if self.uidevice(text="Keep backup off?").wait.exists(timeout=self.timeout):
                    if self.uidevice(text="KEEP OFF").wait.exists(timeout=self.timeout):
                        self.uidevice(text="KEEP OFF").click.wait()
                    else:
                        raise Exception("Photos KEEP OFF not show")
                else:
                    raise Exception("Photos Keep backup off not show")
            allow_btn = self.uidevice(className="android.widget.Button", text="Allow")
            if allow_btn.exists:  # click out 'ALLOW' botton
                allow_btn.click.wait()

            if not self.uidevice(packageName="com.google.android.apps.photos").wait.exists(timeout=self.timeout):
                raise Exception("Photos app not opened")
            if self.bypass_tutorial:
                # skip the tutorial here if exists: get started button, skip sign in and promo button
                if not self.uidevice(
                        resourceId="com.google.android.apps.photos:id/intro_fragment_container").wait.exists(
                        timeout=self.timeout):
                    # workaround, force a layout refresh, sometimes tutorial is not seen, even if appeared
                    self.uidevice.dump(compressed=False)

                if self.uidevice(resourceId="com.google.android.apps.photos:id/intro_fragment_container").wait.exists(
                        timeout=self.timeout):
                    get_started_btn = self.uidevice(resourceId="com.google.android.apps.photos:id/get_started")
                    if not get_started_btn.wait.exists(timeout=self.timeout):
                        # workaround, force a layout refresh, sometimes get started button is not seen, even if appeared
                        self.uidevice.dump(compressed=False)
                        #refer nexus 5x not get start button
                        #if not get_started_btn.exists:
                            #raise Exception("Get started button not found in Photos tutorial")
                    if get_started_btn.wait.exists(timeout=self.timeout):
                        get_started_btn.click()
                skip_sign_in_btn = self.uidevice(resourceId="com.google.android.apps.photos:id/dont_sign_in_button")
                #refer nexus 5x not skip_sign_in button
                if skip_sign_in_btn.wait.exists(timeout=self.timeout):
                    skip_sign_in_btn.click()
                close_promo_btn = self.uidevice(resourceId="com.google.android.apps.photos:id/promo_close_button")
                if close_promo_btn.wait.exists(timeout=self.timeout):
                    close_promo_btn.click()
                    if not close_promo_btn.wait.gone(timeout=self.timeout):
                        raise Exception("First run promo not closed successfully")

            # check if main page is displayed
            if not self.uidevice(resourceId="com.google.android.apps.photos:id/main_container").wait.exists(
                    timeout=self.timeout):
                raise Exception("Main activity of Photos app not opened")

            # scrollable list
            scrollable_list = self.uidevice(resourceId="com.google.android.apps.photos:id/recycler_view")
            if not scrollable_list.wait.exists(timeout=self.timeout):
                raise Exception("Photos recycler view not found")
            # wait for progress bar to be gone
            progress_bar = self.uidevice(className="android.widget.ProgressBar")
            if progress_bar.exists:
                if not progress_bar.wait.gone(timeout=self.timeout):
                    raise Exception("Timeout reached, Main activity of Photos app still loading")

            # open the folder
            if not scrollable_list.scroll.vert.to(text=self.local_folder_name):
                raise Exception("Folder name not found in main list")
            self.uidevice(text=self.local_folder_name).click()
            if not self.uidevice(resourceId="com.google.android.apps.photos:id/toolbar").child(
                    text=self.local_folder_name).wait.exists(timeout=self.timeout):
                raise Exception("Folder not opened in Photos app")

            # open the file
            photo_frame = self.uidevice(descriptionStartsWith=self.file_identifier)
            if not photo_frame.wait.exists(timeout=self.timeout):
                raise Exception("No file was found in the folder")
            if photo_frame.count > 1:
                raise Exception("More than one file in the folder")
            photo_frame.click()

            # check if photo viewer is opened
            photo_view_pager_obj = self.uidevice(resourceId="com.google.android.apps.photos:id/photo_view_pager")
            if not photo_view_pager_obj.wait.exists(timeout=self.timeout):
                raise Exception("Photo view pager not found")
            photo_action_bar_obj = self.uidevice(resourceId="com.google.android.apps.photos:id/photo_action_bar")
            # video starts playing, we must reveal the action bar
            if self.open_video:
                # force a small delay, wait the video to start playing
                time.sleep(1)
                if photo_action_bar_obj.wait.gone(timeout=1000):
                    photo_view_pager_obj.click()
            # check if action bar is not hidden
            if not photo_action_bar_obj.wait.exists(timeout=self.timeout):
                raise Exception("File not opened correctly, action bar not found")
            # check can't load file error
            if self.uidevice(resourceId="com.google.android.apps.photos:id/error_text").exists or self.uidevice(
                    resourceId="com.google.android.apps.photos:id/photos_videoplayer_list_empty_text").exists:
                raise Exception("File cannot be loaded in Photos app")

        except Exception, e:
            self.set_errorm(self.message_str, e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if photo viewer was opened successfully, False if not.
        """
        return self.step_data


class RecordVideo(BtStep):
    """ Description:
            Record a video file with a specified duration using screenrecord
        Usage:
            bluetooth_steps.record_video_and_open(serial=dut_serial,
                                    recording_folder=<path>,recording_name=<name>, record_time=5)()
    """

    def __init__(self, recording_folder, recording_name, record_time=5, **kwargs):
        """
        :param recording_folder: folder on the /sdcard of the file to be recorded-can be empty (e.g. "bt_test")
        :param recording_name: name of the recording (e.g. "video.mp4")
        :param record_time: recording length (in secs)
        :param timeout: default timeout for each wait for exists
        """
        BtStep.__init__(self, **kwargs)
        self.recording_folder = recording_folder
        self.recording_name = recording_name
        self.record_time = record_time
        self.outp_check = None
        self.step_data = True
        self.set_passm("Video " + self.recording_name + " recorded")
        self.set_errorm("Video " + self.recording_name, "Not recorded")

    def do(self):
        try:
            if not self.recording_folder == "":
                # create the folder
                record_path = "/sdcard/" + self.recording_folder + "/"
                full_path_record = record_path + self.recording_name
                if "No such file" in self.adb_connection.cmd("shell ls " + record_path).communicate()[0].decode(
                        "utf-8"):
                    self.adb_connection.cmd("shell mkdir " + record_path).wait()
                    if "No such file" in self.adb_connection.cmd("shell ls " + record_path).communicate()[0].decode(
                            "utf-8"):
                        raise Exception("Cannot create " + record_path)
            else:
                full_path_record = "/sdcard/" + self.recording_name
            record_cmd = "shell screenrecord " + full_path_record + " --time-limit " + str(self.record_time)
            self.adb_connection.cmd(record_cmd).wait()
            check_cmd = "shell ls " + full_path_record
            self.outp_check = self.adb_connection.cmd(check_cmd).communicate()[0].decode("utf-8").strip()

        except Exception, e:
            self.set_errorm("Video " + self.recording_name, e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if video file is found after recording, False if not.
        """
        if self.step_data:
            self.step_data = "No such file" not in self.outp_check
        return self.step_data


class DismissTransferList(BtStep):
    """ Description:
            Dismiss the apparition of inbounds/outbounds transfers window.
            Use fail_if_not_found=True when the transfers window is expected to be on the screen.
            Call this TS with fail_if_not_found=False as an workaround for cases when the incoming
            file pops-up automatically opens the list in background (this case appears
            if a previous test has opened the list by clicking on the Bluetooth
            share: Sent/Received files notification - due to cleanup not well performed)
        Usage:
            bluetooth_steps.DismissEventualTransferList(serial=dut_serial,
                                            time_to_wait_appear=0, fail_if_not_found=False)()
    """

    def __init__(self, time_to_wait_appear=0, fail_if_not_found=False, **kwargs):
        """
        :param time_to_wait_appear: timeout to wait till the transfers window is possible to appear
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.time_to_wait_appear = time_to_wait_appear
        self.fail_if_not_found = fail_if_not_found
        self.step_data = True
        self.transfer_activity_title = self.uidevice(packageName="com.android.bluetooth",
                                                     resourceId="android:id/action_bar").child(textContains="transfers")
        self.set_passm("Transfers window found, dismissed")

    def do(self):
        try:
            if self.transfer_activity_title.wait.exists(timeout=self.time_to_wait_appear):
                # press back in order to dismiss the window
                self.uidevice.press.back()
                if not self.transfer_activity_title.wait.gone(timeout=self.timeout):
                    raise Exception("Failed to dismiss it")
            else:
                if self.fail_if_not_found:
                    raise Exception("Transfers window not found, cannot dismiss it")
                else:
                    self.set_passm("Nothing to dismiss")
        except Exception, e:
            self.set_errorm("Transfers window", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if transfer window is gone, False if not.
        """
        return self.step_data


class PressHome(BtStep):
    """ Description:
            Press the home button as a setup for tests.
        Usage:
            bluetooth_steps.PressHome(serial=serial)()
    """

    def __init__(self, **kwargs):
        """
        :param kwargs: serial, timeout and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.step_data = True
        self.set_passm("Home pressed")

    def do(self):
        try:
            self.uidevice.press.home()
            time.sleep(1)
        except Exception, e:
            self.step_data = False
            self.set_errorm("Press home exception", e.message)

    def check_condition(self):
        """
        :return: True if home pressed, False if not.
        """
        return self.step_data


class CheckPackageInstalled(BtStep):
    """ Description:
            Checks if an app is installed.
        Usage:
            bluetooth_steps.CheckPackageInstalled(serial=serial, package_name="com.google.android.GoogleCamera")()
    """

    def __init__(self, package_name="com.google.android.GoogleCamera", **kwargs):
        """
        :param package_name: package name of the app to be checked
        :param kwargs: serial and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.step_data = True
        self.package_name = package_name
        self.set_passm(str(package_name) + " installed")
        self.set_errorm("App " + str(self.package_name), "Not installed")

    def do(self):
        try:
            self.step_data = str(self.package_name) in self.adb_connection.cmd("shell pm list packages").communicate()[
                0].decode("utf-8").strip()
        except Exception, e:
            self.step_data = False
            self.set_errorm("Exception when checking " + str(self.package_name), e.message)

    def check_condition(self):
        """
        :return: True if package name is found on the device, False if not.
        """
        return self.step_data


class ClearDataPackage(BtStep):
    """ Description:
            Executes command 'adb shell pm clear [package_name]'. By default,
            it clears the Settings app, but you can also clear other apps by
            passing their package name to package_name parameter. This does
            not check anything, to be used for setup/teardown of tests
        Usage:
            bluetooth_steps.ClearDataPackage(serial=serial,
                                        package_name="com.android.settings")()
    """

    def __init__(self, package_name="com.android.settings", **kwargs):
        """
        :param package_name: package name of the app to be cleared
        :param kwargs: serial and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.package_name = package_name

    def do(self):
        try:
            self.adb_connection.cmd("shell pm clear " + str(self.package_name)).wait()
        except Exception, e:
            info_message = "Exception encountered when clear data for " + str(self.package_name) + ": " + e.message
            if self.serial:
                info_message = "[ " + str(self.serial) + " ] " + info_message
            self.logger.info(info_message)

    def check(self):
        # prevent test step to display info, not relevant for BT tests
        pass


class StopPackage(BtStep):
    """ Description:
            Executes command 'adb shell am force-stop [package_name]'. By default,
            it stops the Settings app, but you can also clear other apps by passing
            their package name to package_name parameter. This does not check
            anything, to be used for setup/teardown of tests
        Usage:
            bluetooth_steps.StopPackage(serial=serial,
                                        package_name="com.android.settings")()
    """

    def __init__(self, package_name="com.android.settings", **kwargs):
        """
        :param package_name: package name of the app to be stopped
        :param kwargs: serial and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.package_name = package_name

    def do(self):
        try:
            self.adb_connection.cmd("shell am force-stop " + str(self.package_name)).wait()
        except Exception, e:
            info_message = "Exception encountered when stop " + str(self.package_name) + ": " + e.message
            if self.serial:
                info_message = "[ " + str(self.serial) + " ] " + info_message
            self.logger.info(info_message)

    def check(self):
        # prevent test step to display info, not relevant for BT tests
        pass


class LogInfo(base_step):
    """ Description:
            Logs an info message
        Usage:
            bluetooth_steps.LogInfo(message=<your_message>)()
    """

    def __init__(self, info_message):
        """
        :param info_message: info message to be logged
        """
        base_step.__init__(self)
        self._info_message = info_message

    def do(self):
        self.logger.info(self._info_message)

    def check(self):
        # prevent test step to display info, not relevant for BT tests
        pass


class ConnectPairedDevices(BtStep):
    """ Description:
            Do not use in BT tests!
            Connects device with the already paired <dev_to_connect_name>
        Usage:
            bluetooth_steps.ConnectPairedDevices(dev_to_connect_name=<device name>)()
        Tags:
            ui, android, bluetooth
    """

    def __init__(self, dev_to_connect_name, **kwargs):
        BtStep.__init__(self, **kwargs)
        self.dev_to_connect_name = dev_to_connect_name
        self.connected = True
        self.set_passm("Connected to device " + str(dev_to_connect_name))

    def do(self):
        try:
            ui_steps.click_button_if_exists(serial=self.serial,
                                            view_to_find={"text":
                                                              self.dev_to_connect_name})()
        except Exception, e:
            self.connected = False
            self.set_errorm("Connect to device " +
                            str(self.dev_to_connect_name), e.message)

    def check_condition(self):
        if ui_utils.is_text_visible(text_to_find=self.dev_to_connect_name,
                                    serial=self.serial):
            self.connected = True
        else:
            self.connected = False
        return self.connected


class SetBT(BtStep):
    """Api to turn ON or OFF bluetooth.
    Description:
        Helps in turing on or off bluetooth through Graphical User Interface
        (use_gui=True) or Command Line(use_gui=False)
    Usage:
        bluetooth_steps.SetBT(state='ON', use_gui=True)

        Possible values:
            state = ON/OFF (default=ON)
            interface = CL/UI (default=CL)(deprecated)
            use_gui = True/False (default=True)



    """
    def __init__(self, state="ON", use_gui=True, **kwargs):
        self.state = state.upper()
        self.use_gui = use_gui
        BtStep.__init__(self, **kwargs)
        self.step_data = True
        self.set_passm("[" + self.serial + "]" + " Succesfully turned " +
                       state + " BT")
        self.set_errorm("[" + self.serial + "]", " Failed to turn " + state
         + " BT")

    def do(self):
        try:
            if self.use_gui:
                DEVICE_VERSION = GetAndroidVersion(serial=self.serial,
                                                   blocking=True)()
                StopPackage(serial=self.serial, blocking=True)()
                PressHome(serial=self.serial, blocking=True)()
                OpenBluetoothSettings(serial=self.serial,
                                      version=DEVICE_VERSION, blocking=True)()
                ClickBluetoothSwitch(serial=self.serial, state=self.state,
                                     version=DEVICE_VERSION, blocking=True)()
            else:
                bt_current_state = bluetooth_utils.check_bluetooth_state_on(
                    serial=self.serial)
                if bt_current_state and self.state=='ON' or not \
                        bt_current_state and self.state=='OFF':
                    self.set_passm("BT already set to " + self.state)
                else:
                    if self.state == "ON":
                        cmd = "shell service call bluetooth_manager 6"
                    elif self.state == 'OFF':
                        cmd = "shell service call bluetooth_manager 8"

                    output = self.adb_connection.cmd(cmd).communicate()[
                     0].decode("utf-8").strip()
                    if "00000000 00000001   '........'" not in output:
                        self.step_data = False
                        raise Exception()
        except Exception as e:
            self.step_data = False

    def check_condition(self):
        if self.step_data:
            # below sleep time is added to let the bluetooth to switch on,
            # otherwise status will come as bluetooth is off
            time.sleep(1)
            cmd = "shell service call bluetooth_manager 5"
            if self.state == "ON":
                expected_output = "00000000 00000001   '........'"
            else:
                expected_output = "00000000 00000000   '........'"
            try:
                output = self.adb_connection.cmd(cmd).communicate()[
                0].decode("utf-8").strip()
                if expected_output not in output:
                    raise Exception()
            except Exception:
                self.step_data = False

        return self.step_data


class BrowseFileInSettingsStorage(BtStep):
    """ Description:
               Browse files in the "Settings/Storage & USB or Storage"
               (Device or Portable Storage.)
           Usage:
               bluetooth_steps.BrowseFileInSettingsStorage(serial=dut_serial,
                                           file_full_path=<file full path>)()
    """

    def __init__(self, file_full_path=None, **kwargs):
        """
        :param file_full_path: complete path to the file to open (eg: "/sdcard/Bluetooth/img.jpg")
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.step_data = True
        if file_full_path != None:
            self.file_full_path = str(file_full_path).replace('\\', '/')
            # part of logging message
            self.message_str = "File " + self.file_full_path
            self.set_passm(self.message_str + " is located")
        else:
            self.file_full_path = file_full_path
            self.message_str = ""
            self.set_passm("Opened Settings/Storage browser")

    def do(self):
        try:
            artifact_name = ""
            navigation_flow = ""
            is_device_storage = ""

            # Todo, Need to check whether below paths for Device and Portable storage are correct
            if self.file_full_path != None:
                if self.file_full_path.startswith("/storage/emulated/0/"):
                    is_device_storage = True
                    self.file_full_path = self.file_full_path.replace(
                        "/storage/emulated/0/", "")
                elif self.file_full_path.startswith("/sdcard/"):
                    is_device_storage = True
                    self.file_full_path = self.file_full_path.replace("/sdcard/", "")
                elif self.file_full_path.startswith("/storage/"):
                    is_device_storage = False
                else:
                    raise Exception("Path: {0} is not valid and cannot be "
                        "browsed through Settings/Storage".format(self.file_full_path))

            launch_command = "shell am start -a " \
                             "android.settings.INTERNAL_STORAGE_SETTINGS"

            # launch the Settings/Storage activity
            self.adb_connection.cmd(launch_command).wait()
            if self.file_full_path != None:
                if is_device_storage == False:
                   if self.uidevice(textContains="Portable "
                        "storage").wait.exists(timeout=self.timeout):
                       self.uidevice(textContains="SD card").click.wait()
                   else:
                       raise Exception("Path given is portable storage "
                                       "path, but it is not mounted")
                else:
                    if self.uidevice(textContains="Portable "
                        "storage").wait.exists(timeout=self.timeout):
                        # below regular expression is to make it compatible to M
                        #  and other latest dessert
                        if self.uidevice(textMatches="Internal("
                              ".*)storage").wait.exists(timeout=self.timeout):
                            self.uidevice(textMatches="Internal(.*)storage").click.wait()

                artifact_name = os.path.basename(self.file_full_path)
                navigation_flow = os.path.dirname(self.file_full_path).split("/")

                # clearing all null values from navigation path
                while True:
                    try:
                       navigation_flow.pop(navigation_flow.index(""))
                    except:
                        break

                # Below folder 'Explore' is added since generally we
                # should check in this folder for browse in device or
                # protable storage
                if navigation_flow:
                    navigation_flow[:0] = ["Explore"]
                else:
                    navigation_flow = ["Explore"]

                folder_list = self.uidevice(scrollable=True)
                for folder in navigation_flow:
                    if not self.uidevice(text=folder).wait.exists(
                            timeout=self.timeout):
                        if not folder_list.scroll.to(text=folder):
                            raise Exception(folder + " not found in "
                                                 "Settings/Storage")
                    self.uidevice(text=folder).click.wait()

                if self.uidevice(text=artifact_name).wait.exists(
                        timeout=self.timeout):
                    self.uidevice(text=artifact_name).long_click()
                else:
                    raise Exception("Artifact {0} not found in view".format(
                        artifact_name))

        except Exception, e:
            self.set_errorm(self.message_str, e.message)
            self.step_data = False

    def check_condition(self):
        if self.step_data == True:
            if not self.uidevice(resourceId="com.android.documentsui:id/menu_search"
                                ).wait.exists(timeout=self.timeout):
                self.step_data = False

        return self.step_data


class BtOppShareFile(BtStep):
    """ Description:
            Share any file via bluetooth from Settings/Storage & USB or
            Storage. Call this after the required artifact is long pressed
            which shows share option.
            Note that this TS lets your device in the Home screen
        Usage:
            bluetooth_steps.BtOppShareFile(serial=serial,
                            server_dut="device_name_to_send_to",
                            bt_already_opened=True, scan_timeout=60000, version=version)()
    """

    def __init__(self, server_dut, bt_already_opened=True, scan_timeout=60000, **kwargs):
        """
        :param server_dut: name of the device to share picture/video with
        :param bt_already_opened: True means that BT is opened. False it also checks the appearance of 'Turning BT on'
        :param scan_timeout: max time to wait for BT scanning progress
        :param kwargs: serial, timeout, no_log and standard kwargs for base_step
        """
        BtStep.__init__(self, **kwargs)
        self.server_dut = server_dut
        self.bt_already_opened = bt_already_opened
        self.scan_timeout = scan_timeout
        self.step_data = True
        self.set_passm("Bluetooth share sending file initiated")

    def do(self):
        try:
            # TODO
            # below if else is added to mitigate the bug in Android M
            # which not dumping share toolbar when long pressed artifact in
            # Settings/Strorage browser. Because of this uiautomator
            # couldn't pick and click 'Share' option.
            # Once bug is resolved, 'else' part body alone sufficient to handle
            # and 'if' part should be removed
            if self.version.startswith("6."):
                self.uidevice.click(1760, 100)
            else:
                share_button = self.uidevice(description="Share")

                # click on share button
                if not share_button.wait.exists(timeout=self.timeout):
                    raise Exception("Share button not found")
                share_button.click()

            if not self.uidevice(resourceId="android:id/resolver_list").wait.exists(
                    timeout=self.timeout):
                raise Exception("Share chooser option not shown after pressing Share")

            # click on Bluetooth option
            bluetooth_option = self.uidevice(text="Bluetooth")
            if not bluetooth_option.wait.exists(timeout=self.timeout):
                raise Exception("Bluetooth as Share option not exists")
            bluetooth_option.click()

            if self.version.startswith("5.") or self.version.startswith("6.0"):
                # LLP, M versions
                #  enable bluetooth if not already
                if not self.bt_already_opened:
                    if not self.uidevice(packageName="com.android.bluetooth").wait.exists(timeout=self.timeout):
                        raise Exception("Turn on bluetooth now? window not shown")
                    turn_on_button = self.uidevice(text="Turn on")
                    if not turn_on_button.wait.exists(timeout=self.timeout):
                        raise Exception("Turn on bluetooth button not shown")
                    turn_on_button.click()
                # wait for bt list to appear, and for scanning progress to finish
                window_title_obj = self.uidevice(text="Choose Bluetooth device")
                if not window_title_obj.wait.exists(timeout=self.timeout):
                    raise Exception("Choose Bluetooth devices list not displayed")
                # wait for scanning progress to finish
                scan_progress_obj = self.uidevice(resourceId="com.android.settings:id/scanning_progress")
                if scan_progress_obj.wait.exists(timeout=5000):
                    if not scan_progress_obj.wait.gone(timeout=self.scan_timeout):
                        raise Exception("Timeout reached, still scanning after " + str(self.scan_timeout))
                bt_list = self.uidevice(resourceId="android:id/list")
                if not bt_list.wait.exists(timeout=self.timeout):
                    raise Exception("BT devices list not shown")
                # click on required device name to share with
                if not bt_list.scroll.to(text=self.server_dut):
                    raise Exception(str(self.server_dut) + " not found in bluetooth devices list")
                self.uidevice(text=self.server_dut).click()
                if not window_title_obj.wait.gone(timeout=self.timeout):
                    raise Exception("Bluetooth devices list not closed after choosing device to send to")
            else:
                # N version
                # enable bluetooth if not already
                if not self.bt_already_opened:
                    if not self.uidevice(packageName="com.android.bluetooth").wait.exists(timeout=self.timeout):
                        raise Exception("Turn on bluetooth now? window not shown")
                    turn_on_button = self.uidevice(text="TURN ON")
                    if not turn_on_button.wait.exists(timeout=self.timeout):
                        raise Exception("TURN ON bluetooth button not shown")
                    turn_on_button.click()
                # wait for bt list to appear, and for scanning progress to finish
                window_title_obj = self.uidevice(text="Choose Bluetooth device")
                if not window_title_obj.wait.exists(timeout=self.timeout):
                    raise Exception("Choose Bluetooth devices list not displayed")
                # wait for scanning progress to finish
                scan_progress_obj = self.uidevice(resourceId="com.android.settings:id/scanning_progress")
                if scan_progress_obj.wait.exists(timeout=5000):
                    if not scan_progress_obj.wait.gone(timeout=self.scan_timeout):
                        raise Exception("Timeout reached, still scanning after " + str(self.scan_timeout))
                bt_list = self.uidevice(resourceId="com.android.settings:id/list")
                if not bt_list.wait.exists(timeout=self.timeout):
                    raise Exception("BT devices list not shown")
                # click on required device name to share with
                if not bt_list.scroll.to(text=self.server_dut):
                    raise Exception(str(self.server_dut) + " not found in bluetooth devices list")
                self.uidevice(text=self.server_dut).click()
                if not window_title_obj.wait.gone(timeout=self.timeout):
                    raise Exception("Bluetooth devices list not closed after choosing device to send to")

            # press home
            if not PressHome(serial=self.serial, timeout=self.timeout, version=self.version, critical=False,
                             no_log=True)():
                raise Exception("Press home failed")
            # check the bluetooth sharing notification
            if not CheckTransferInitiated(serial=self.serial, timeout=self.timeout, version=self.version,
                                          critical=False, no_log=True)():
                raise Exception("Check Bluetooth sharing initiated failed")

        except Exception, e:
            self.set_errorm("Share file", e.message)
            self.step_data = False

    def check_condition(self):
        """
        :return: True if sharing was initiated, False if not
        """
        return self.step_data


class ClearRecentApps(BtStep):
    """ Description:
            Clear recent apps in recent apps windows
        Usage:
            bluetooth_steps.ClearRecentApps(serial=serial)
    """

    def __init__(self, app=None, **kwargs):
        BtStep.__init__(self, **kwargs)
        self.app = app
        self.step_data = True
        self.set_passm("Cleared all recent apps")

    def do(self):
        try:
            self.uidevice.press("home")
            self.uidevice.wait.update(timeout=2000)
            self.uidevice.press("recent")
            self.uidevice.wait.update(timeout=2000)
            if not self.uidevice(
                    resourceId="com.android.systemui:id/dismiss_task").wait.exists(
                timeout=self.timeout):
                self.set_passm("No recent apps to clear")
            else:
                if self.version.startswith("6.0"):
                    # wait for ~10 seconds and will quit if still doesn't have any recent apps
                    while self.uidevice(
                            resourceId="com.android.systemui:id/dismiss_task").exists:
                        self.uidevice(
                            resourceId="com.android.systemui:id/dismiss_task").click()
                        # just quit if back to home page
                        if self.uidevice(textContains="No recent items"):
                            break
                else:
                    if self.uidevice(textContains="CLEAR ALL").exists:
                        print "indis direct click"
                        self.uidevice(textContains="CLEAR ALL").click()
                    else:
                        for i in range(0, 5):
                            self.uidevice(scrollable=True).scroll.toBeginning()
                            if self.uidevice(textContains="CLEAR ALL").exists:
                                self.uidevice(textContains="CLEAR ALL").click()
                                break
            #self.uidevice.press.home()
        except:
            self.set_errorm("ClearRecentApps", "Failed to clear recent apps")
            self.step_data = False

    def check_condition(self):
        time.sleep(1)
        if self.step_data and self.uidevice(
                    resourceId="com.android.systemui:id/dismiss_task").exists:
            self.step_data = False
            self.set_errorm("ClearRecentApps", "Still some apps are "
                                               "available in recent apps window")
        self.uidevice.press("home")
        return self.step_data