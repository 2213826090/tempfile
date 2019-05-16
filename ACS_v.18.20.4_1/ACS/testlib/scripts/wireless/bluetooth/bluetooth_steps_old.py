#!/usr/bin/env python

#######################################################################
#
# @filename:    bluetooth_steps.py
# @description: Bluetooth test steps
# @author:      nicolas.paccou@intel.com
#
#######################################################################

from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.wireless.bluetooth import bluetooth_utils
from testlib.utils.ui.uiandroid import UIDevice as ui_device

import os
import subprocess
import sys
import time


class set_airplane_mode(ui_step, adb_step):
    """ description:
            sets the airplane mode to the state issued by the test.
            Possible states:
               ON -> state="1"
               OFF - > state="0"
        usage:
            bluetooth_steps.set_airplane_mode(state="1")()
        tags:
            bluetooth, android, airplane mode
    """
    state = None
    def __init__(self, state = "0", **kwargs):
        adb_step.__init__(self, **kwargs)
        ui_step.__init__(self, **kwargs)
        self.state = state

    def do(self):

        ui_steps.open_quick_settings(
            print_error = "Error - Quick settings page as not displayed")()

        if self.verbose:
            print bluetooth_utils.check_airplane_mode_on()

        if bluetooth_utils.check_airplane_mode_on() == self.state :
            if self.state == "0":
                ui_steps.press_home()()
                ui_steps.open_settings()()
            ui_steps.open_app_from_settings(view_to_find =
                    {"text": "Bluetooth"}, view_to_check = {"description": "Bluetooth, Navigate up"})()
            Bluetooth_state = bluetooth_utils.get_switch_state(self)

        if Bluetooth_state == "OFF":
            if self.verbose:
                print ">>> Bluetooth is already OFF, enabling it first"
                bluetooth_set_from_status_bar(state = "ON")()

            if self.verbose:
                print ">>> Airplane mode is already in the required state, inverting it first..."
        ui_steps.open_quick_settings(
                print_error = "Error - Quick settings page as not displayed")()
        ui_steps.click_button(
        print_error = "Error - Airplane Mode not found",
        view_to_find = {"text": "Airplane mode"})()
        time.sleep(10)
        adb_steps.connect_device(serial = sys.argv[1] + ":5555")()

        if self.state == "1":
            ui_steps.press_home()()
            ui_steps.open_settings()()
        ui_steps.open_app_from_settings(view_to_find =
                    {"text": "Bluetooth"}, view_to_check = {"description": "Bluetooth, Navigate up"})()
        Bluetooth_state = bluetooth_utils.get_switch_state(self)

        if Bluetooth_state == "OFF":
            if self.verbose:
                print ">>> Bluetooth is already OFF, enabling it first"
                bluetooth_set_from_status_bar(state = "ON")()

        if self.state == "0":
            ui_steps.press_home()()
            ui_steps.open_settings()()
            ui_steps.open_app_from_settings(view_to_find =
                {"text": "Bluetooth"}, view_to_check = {"description": "Bluetooth, Navigate up"})()
        Bluetooth_state = bluetooth_utils.get_switch_state(self)

        if Bluetooth_state == "ON":
            if self.verbose:
                print ">>> Bluetooth radio is enabled whereas Flight Mode is active, disabling then enabling again Flight Mode to reset Bluetooth radio state..."
                ui_steps.open_quick_settings(
                   print_error = "Error - Quick settings page as not displayed")()
            ui_steps.click_button(
                   print_error = "Error - Airplane Mode not found",
                   view_to_find = {"text": "Airplane mode"})()
            ui_steps.open_quick_settings(
                   print_error = "Error - Quick settings page as not displayed")()
            ui_steps.click_button(
                   print_error = "Error - Airplane Mode not found",
                   view_to_find = {"text": "Airplane mode"})()

        ui_steps.open_quick_settings(
            print_error = "Error - Quick settings page as not displayed")()
        ui_steps.click_button(
        print_error = "Error - Airplane Mode not found",
        view_to_find = {"text": "Airplane mode"})()
        time.sleep(10)
        adb_steps.connect_device(serial = sys.argv[1] + ":5555")()

        if self.state == "0":
            ui_steps.press_home()()
            ui_steps.open_settings()()
            ui_steps.open_app_from_settings(view_to_find =
                {"text": "Bluetooth"}, view_to_check = {"description": "Bluetooth, Navigate up"})()
            Bluetooth_state = bluetooth_utils.get_switch_state(self)

        if Bluetooth_state == "OFF":
            if self.verbose:
                print ">>> Bluetooth radio stays OFF whereas Flight Mode has been deactivated, checking if not a special case..."
                bluetooth_set_from_status_bar(state = "ON")()
            ui_steps.open_quick_settings(
                print_error = "Error - Quick settings page as not displayed")()
            ui_steps.click_button(
                print_error = "Error - Airplane Mode not found",
                view_to_find = {"text": "Airplane mode"})()
            time.sleep(10)
        adb_steps.connect_device(serial = sys.argv[1] + ":5555")()
        ui_steps.open_quick_settings(
            print_error = "Error - Quick settings page as not displayed")()
        ui_steps.click_button(
            print_error = "Error - Airplane Mode not found",
            view_to_find = {"text": "Airplane mode"})()
        time.sleep(10)

    def check_condition(self):

        if self.state == "1":
            expected_state = "Bluetooth Off"
        elif self.state == "0":
            expected_state = "Bluetooth"

        ui_steps.open_quick_settings(
            print_error = "Error - Quick settings page as not displayed")()
        return self.uidevice(text = expected_state).exists


class set_from_settings(ui_step):
    """ description:
            resets the bluetooth_module. If it is in the same state as the
              user specified, then it issues a restart. If it is in the
              opposite state, it turns it to the desired state.
        usage:
            set_from_settings(state = "ON")()
        tags:
            bluetooth, android, fresh_start, set bluetooth
    """
    state = None
    def __init__(self, state = "ON", **kwargs):
        ui_step.__init__(self, **kwargs)
        self.state = state

    def do(self):

        if self.state == "ON":
            reverse_state = "OFF"
        elif self.state == "OFF":
            reverse_state = "ON"

        ui_steps.press_home()()
        ui_steps.open_settings()()
        ui_steps.open_app_from_settings(view_to_find =
                    {"text": "Bluetooth"}, view_to_check = {"description": "Bluetooth, Navigate up"})()
        print ">>> Bluetooth radio is %s" % bluetooth_utils.get_switch_state(self)
        if self.state == bluetooth_utils.get_switch_state(self):
            if self.verbose:
                print ">>> Bluetooth radio is already in the required state, inverting it first..."
        print ui_steps.click_switch(
                view_to_find = {"className": "android.widget.Switch",
                                "instance": "0"},
                state = reverse_state)()
        time.sleep(10)
    time.sleep(10)

    def check_condition(self):
        return ui_steps.click_switch(
                view_to_find = {"className": "android.widget.Switch",
                                "instance": "0"},
                state = self.state)()


class open_bluetooth_settings(ui_step):
    """ description:
            opens the bluetooth module from settings
        usage:
            bluetooth_steps.open_bluetooth_settings()()
        tags:
            bluetooth, android, open
    """
    def do(self):
        ui_steps.press_home()()
        ui_steps.open_settings()()
        ui_steps.click_button(
            print_error = "Error - bluetooth page was not displayed",
            view_to_find = {"textContains": "Bluetooth"})()
    def check_condition(self):
        return self.uidevice(textContains = "Bluetooth").exists


class bluetooth_set_from_status_bar(ui_step):
    """ description:
            resets the bluetooth_module from the Status Bar. If it is in the
              same state as the user specified, then it issues a restart. If
              it is in the opposite state, it turns it to the desired state.
        usage:
            bluetooth_steps.bluetooth_set_from_status_bar(state = "ON")()
        tags:
            bluetooth, android, fresh_start, set bluetooth, status bar
    """
    state = None
    def __init__(self, state = "ON", **kwargs):
        ui_step.__init__(self, **kwargs)
        self.state = state

    def do(self):
        ui_steps.press_home()()
        ui_steps.open_settings()()
        ui_steps.open_app_from_settings(view_to_find =
                    {"text": "Bluetooth"}, view_to_check = {"description": "Bluetooth, Navigate up"})()
        print ">>> Bluetooth radio is %s" % bluetooth_utils.get_switch_state(self)
        if self.state == bluetooth_utils.get_switch_state(self):
            if self.verbose:
                print ">>> Bluetooth radio is already in the required state, inverting it first..."
            ui_steps.open_quick_settings(
                print_error = "Error - Quick settings page was not displayed")()
        ui_steps.long_click(
            print_error = "Error - bluetooth page was not displayed",
            view_to_find = {"textContains": "Bluetooth"})()
        time.sleep(10)
        ui_steps.open_quick_settings(
            print_error = "Error - Quick settings page was not displayed")()
        ui_steps.long_click(
            print_error = "Error - bluetooth page was not displayed",
            view_to_find = {"textContains": "Bluetooth"})()
    time.sleep(10)

    def check_condition(self):

        if self.state == "ON":
            expected_state = "Bluetooth"
        elif self.state == "OFF":
            expected_state = "Bluetooth Off"

        return self.uidevice(text = expected_state).exists


class bt_open_settings(ui_step):
    """ description:
            Open Allappps, Settings and turn ON or OFF bluetooth
        usage:
            bt_open_settings(print_error = "Error - Settings page was
                                            not displayed OR bt status
                                            incorrect",
                             view_to_find =\
                         {"resourceId":"com.android.settings:id/switchWidget"},
                             state = "ON")()
        tags:
            ui, android, click, switch, bluetooth, settings
    """
    def __init__(self, state = "ON", **kwargs):
        ui_step.__init__(self, **kwargs)
        self.state = state

    def do(self):
        ui_steps.open_settings(print_error = "Error - Settings page was not "
                                             "displayed")()
        ui_steps.click_switch(print_error = "Error - Settings page was not "
                                            "displayed",
                              view_to_find = {"resourceId":
                                          "com.android.settings:id/switchWidget",
                                              "instance":"1"},
                              state = self.state)()

    def check_condition(self):
        is_switch_on = ui_utils.is_switch_on(view_to_find =\
                    {"resourceId":"com.android.settings:id/switchWidget",
                    "instance":"1"})
        if self.state == "ON" and is_switch_on:
            return True
        elif self.state == "OFF" and not is_switch_on:
            return True
        else:
            return False

class bt_list_displayed(ui_step):
    """ description:
            Toggles bluetooth ON/OFF and checks whether no. of
            displayed devices is 0/>0
        usage:
            bt_list_displayed(print_error = "Error - counting bt devices",
                              state = "ON")()
        tags:
            ui, android, click, switch, bluetooth, settings
    """

    def __init__(self, state = "ON", **kwargs):
        ui_step.__init__(self, **kwargs)
        self.state = state

    def do(self):
        is_switch_on = ui_utils.is_switch_on(view_to_find =\
                           {"className":"android.widget.Switch"})

        if self.state == "ON":
            if is_switch_on:
                pass
            if not is_switch_on:
                ui_steps.click_switch(print_error = "Error - switching bt on",
                                  view_to_find = {"className":
                                                  "android.widget.Switch"},
                                  state = self.state,
                                  index = 0)()
        elif self.state == "OFF":
            if is_switch_on:
                ui_steps.click_switch(print_error = "Error - switching bt on",
                                  view_to_find = {"className":
                                                  "android.widget.Switch"},
                                  state = self.state,
                                  index = 0)()
            if not is_switch_on:
                pass

        self.bt_list = self.uidevice(resourceId = "android:id/title").count

    def check_condition(self):
        if self.state == "ON" and self.bt_list > 1:
            return True
        elif self.state == "OFF" and self.bt_list == 0:
            return True
        else:
            return False


class bt_search_devices(ui_step):
    """ description:
            Open the bluetooth settings, turns ON the bluetooth and clicks
              on the button "Search for devices
        usage:
            bluetooth_steps.bt_search_devices()()
        tags:
            bluetooth, android, bluetooth, search
    """
    def __init__(self, dev_to_find, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.dev_to_find = dev_to_find

    def do(self):
        open_bluetooth_settings()()
        bt_remove_all_pair_devices()()

        ui_steps.click_switch(print_error =
                                    "Error - Settings page was not displayed",
                              blocking = True,
                              view_to_find = {
                                    "className": "android.widget.Switch"
                                             },
                              state = "OFF"
                             )()
        ui_steps.click_switch(print_error =
                                    "Error - Settings page was not displayed",
                              blocking = True,
                              view_to_find = {
                                    "className":"android.widget.Switch"
                                             },
                              state = "ON"
                             )()

    def check_condition(self):
        time.sleep(15)
        return self.uidevice(text = self.dev_to_find).exists


class bt_pair_devices(ui_step):
    """ description:
            Given two devices by the IP address, they will pair
        usage:
            bluetooth_steps.bt_pair_devices(dut = IP1,
                                            dev = IP2,
                                            dev_name = DEV_NAME
                                           )()
        tags:
            bluetooth, android, bluetooth, pair
    """
    def __init__(self, dut, dev, dev_name, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.dut = dut
        self.dev = dev
        self.dev_name = dev_name

    def do(self):
        uidevice = ui_device(serial = self.dut)

        bt_remove_all_pair_devices(serial = self.dut)()
        bt_remove_all_pair_devices(serial = self.dev)()

        bt_search_devices(serial = self.dut,
                          dev_to_find = self.dev_name
                         )()
        uidevice(text = self.dev_name).wait.exists(timeout = 20000)

        ui_steps.click_button(serial = self.dut,
                              view_to_find = {'text': self.dev_name}
                             )()
        ui_steps.click_button(serial = self.dev,
                              view_to_find = {'textMatches': 'Pair'}
                             )()
        ui_steps.click_button(serial = self.dut,
                              view_to_find = {'textMatches': 'Pair'}
                             )()

    def check_condition(self):
        time.sleep(5)
        return ui_device(serial = self.dut)(descriptionContains = 'Device settings').exists


class bt_remove_all_pair_devices(ui_step):
    """ description:
            Given two devices by the IP address, they will pair
        usage:
            bluetooth_steps.bt_pair_devices(dut = IP1,
                                            dev = IP2,
                                            dev_name = DEV_NAME)()
        tags:
            bluetooth, android, bluetooth, pair
    """
    def do(self):
        open_bluetooth_settings()()
        while self.uidevice(description = 'Device settings').exists:
            if self.uidevice(descriptionContains = 'Device settings').exists:
                ui_steps.click_button(view_to_find = {
                                        'descriptionContains': 'Device settings'
                                                     })()
                ui_steps.click_button(view_to_find = {
                                        'textMatches': 'Unpair'
                                                     })()

    def check_condition(self):
        return not self.uidevice(text = 'Paired devices').exists


class bt_change_device_name(ui_step):
    """ description:
            Open the bluetooth settings and replaces the name
                of the devices with the given name
        usage:
            bluetooth_steps.bt_replace_name(name = self.name)()
        tags:
            bluetooth, android, bluetooth, name
    """
    def __init__(self, name, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.name = name

    def do(self):
        open_bluetooth_settings()()
        ui_steps.click_switch(print_error =
                                "Error - Settings page was not displayed",
                              blocking = True,
                              view_to_find = {
                                'className': "android.widget.Switch"
                                             },
                              state = "ON"
                             )()

        if self.uidevice(resourceId = "android:id/title").text != self.name:
            ui_steps.click_button(view_to_find = {
                                    'descriptionContains': 'More options'
                                                 },
                                  view_to_check = {
                                    'textContains': 'Rename tablet'
                                                 }
                                 )()
            ui_steps.click_button(view_to_find = {
                                    'textContains': 'Rename tablet'
                                                 }
                                 )()
            #replace name
            ui_steps.click_button(view_to_find = {
                                     'className' : 'android.widget.EditText'
                                                 }
                                 )()
            while self.uidevice(
                                    className = 'android.widget.EditText'
                                        ).text:
                self.uidevice.press("del")
            ui_steps.edit_text(view_to_find = {
                                'className' : 'android.widget.EditText'
                                              },
                               value = self.name
                              )()
            ui_steps.click_button(view_to_find = {'textMatches' : 'Rename'})()
        bt_make_discoverable(name = self.name)()



    def check_condition(self):
        self.uidevice(text=self.name)


class bt_make_discoverable(ui_step):
    """ description:
            Open the bluetooth settings and replaces the name
                of the devices with the given name
        usage:
            bluetooth_steps.bt_replace_name(name = self.name)()
        tags:
            bluetooth, android, bluetooth, name
    """
    def __init__(self, name, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.name = name

    def do(self):
        if not (self.uidevice(text = 'Bluetooth').exists and
                self.uidevice(text = self.name).exists):
            open_bluetooth_settings()()
        if self.uidevice(textContains = 'Not visible').exists or \
           self.uidevice(textContains = 'Only visible').exists:
            ui_steps.click_button(view_to_find = {
                                    'resourceId': 'android:id/title'
                                                 }
                                 )()
        else:
            ui_steps.click_button(view_to_find = {
                                    'resourceId': 'android:id/title'
                                                 }
                                 )()
            ui_steps.click_button(view_to_find = {
                                    'resourceId': 'android:id/title'
                                                 }
                                 )()

    def check_condition(self):
        self.uidevice(text='Visible to all')

class bt_share_picture(ui_step):
    """ description:
            Share picture via bluetooth
        usage:
            bluetooth_steps.bt_share_picture()()

        tags:
            bluetooth, android, bluetooth, picture, share
    """
    def do(self):
        ui_steps.click_button(view_to_find = {
                                'descriptionContains': 'Share with'
                                             }
                             )()
        ui_steps.click_button(view_to_find = {'text': 'See all'})()

        if self.uidevice(text = 'Bluetooth').exists:
            ui_steps.click_button(view_to_find = {'text': 'Bluetooth'})()
        else:
            ui_steps.press_back()()
            ui_steps.click_button(view_to_find = {'instance': 13})()

        if self.uidevice(text = 'Turn on').exists:
           ui_steps.click_button(view_to_find = {'text': 'Turn on'})()

    def check_condition(self):
        return self.uidevice(text = 'Bluetooth device chooser').exists


class bt_accept_picture(ui_step, adb_step):
    """ description:
            Swipes down to open the notification bar, clicks on the Bluetooth
                receiving file notification, accepts it and verifies the
                storage for the file.
        usage:
            bluetooth_steps.bt_accept_file(path = '/where/to/store/the/file')()
        tags:
            bluetooth, android, bluetooth, tranfer, file
    """

    def __init__(self, path, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.path = path
        self.bt_files = ''
        self.cmd_check_file = 'ls ' + self.path
        self.file_to_accept = ''

    def do(self):
        ui_steps.swipe(sx = 100, sy = 10, ex = 100, ey = 500, steps = 10)()
        time.sleep(1)
        ui_steps.click_button(view_to_find = {
                                'text': 'Bluetooth share: Incoming file'
                                             }
                             )()
        accept_text = self.uidevice(textContains='wants to send').text
        self.file_to_accept = accept_text.split('"')[2].split(' ')[5]
        ui_steps.click_button(view_to_find = {'text': 'Accept'})()
        self.bt_files = self.adb_connection \
                                    .parse_cmd_output(self.cmd_check_file)

    def check_condition(self):
        time.sleep(1)
        if self.file_to_accept in self.bt_files:
            return True
        else:
            return False


class bt_share_contact(ui_step):
    """ description:
            Share via bluetooth
        usage:
            bluetooth_steps.bt_share()()

        tags:
            bluetooth, android, bluetooth, share
    """
    def __init__(self, contact_name, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.contact_name = contact_name

    def do(self):
        ui_steps.click_button(view_to_find = {'text': self.contact_name})()
        ui_steps.click_button(view_to_find = {
                                'descriptionContains': 'More options'
                                             }
                             )()
        ui_steps.click_button(view_to_find = {'text': 'Share'})()
        ui_steps.click_button(view_to_find = {'text': 'Bluetooth'})()


    def check_condition(self):
        return self.uidevice(text = 'Bluetooth device chooser').exists


class bt_accept_contact(ui_step, adb_step):
    """ description:
            Swipes down to open the notification bar, clicks on the Bluetooth
                receiving file notification, accepts it and verifies the
                storage for the file.
        usage:
            bluetooth_steps.bt_accept_file(path = '/where/to/store/the/file')()
        tags:
            bluetooth, android, bluetooth, tranfer, file
    """

    def __init__(self, contact_name, **kwargs):
        ui_step.__init__(self, **kwargs)
        adb_step.__init__(self, **kwargs)
        self.contact_name = contact_name

    def do(self):
        ui_steps.swipe(sx = 100, sy = 10, ex = 100, ey = 500, steps = 10)()
        time.sleep(1)
        ui_steps.click_button(view_to_find = {
                                'text': 'Bluetooth share: Incoming file'
                                             }
                             )()
        accept_text = self.uidevice(textContains='wants to send').text
        self.file_to_accept = accept_text.split('"')[2].split(' ')[5]
        ui_steps.click_button(view_to_find = {'text': 'Accept'})()

    def check_condition(self):
        ui_steps.swipe(sx = 100, sy = 10, ex = 100, ey = 500, steps = 10)()
        time.sleep(1)
        ui_steps.click_button(view_to_find = {
                                'text': 'Bluetooth share: Received files'
                                             }
                             )()
        ui_steps.click_button(view_to_find = {
                                'textContains': self.contact_name
                                             }
                             )()
        return self.uidevice(text = self.contact_name).exists
