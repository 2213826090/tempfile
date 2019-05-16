from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.android.android_step import step as android_step
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.adb import adb_utils
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.connections.local import local_steps
from testlib.scripts.common import common_utils
from testlib.scripts.android.logcat import logcat_steps
from testlib.utils.defaults import telephony_defaults
import time
import datetime
import re


class check_carrier(ui_step):

    def __init__(self, carrier_name, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.carrier_name = carrier_name
        self.wait_time = wait_time

    def do(self):
        ui_steps.open_quick_settings(serial = self.serial)()

    def check_condition(self):
        return self.uidevice(textContains = self.carrier_name).wait.exists(timeout = self.wait_time)


class open_phone(android_step):

    def __init__(self, wait_time = 5000, **kwargs):
        android_step.__init__(self, **kwargs)
        self.wait_time = wait_time

    def do(self):
        ui_steps.open_app_from_allapps(serial = self.serial,
                               view_to_find = {"text": "Phone"},
                               view_to_check = {"resourceId": "com.android.dialer:id/floating_action_button"},
                               wait_time = self.wait_time)()

    def check_condition(self):
        # Check performed in do()
        return True

class open_messenger(android_step):

    def __init__(self, wait_time = 5000, **kwargs):
        android_step.__init__(self, **kwargs)
        self.wait_time = wait_time

    def do(self):
        ui_steps.open_app_from_allapps(serial = self.serial,
                               view_to_find = {"text": "Messenger"},
                               view_to_check = {"resourceId": "com.google.android.apps.messaging:id/start_new_conversation_button"},
                               wait_time = self.wait_time)()

    def check_condition(self):
        # Check performed in do()
        return True

class open_new_conversation(ui_step):

    def __init__(self, number, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time
        self.number = number

    def do(self):
        ui_steps.click_button(serial = self.serial,
                               view_to_find = {"resourceId": "com.google.android.apps.messaging:id/start_new_conversation_button"},
                               view_to_check = {"resourceId": "com.google.android.apps.messaging:id/recipient_text_view"},
                               wait_time = self.wait_time)()
        ui_steps.edit_text(serial = self.serial,
                           view_to_find = {"resourceId": "com.google.android.apps.messaging:id/recipient_text_view"},
                           value = self.number)()
        adb_utils.input_keyevent(serial = self.serial,
                                 key_number = 66)

    def check_condition(self):
        return self.uidevice(resourceId = "com.google.android.apps.messaging:id/compose_message_text").wait.exists(timeout = self.wait_time)




class open_phone_dialer(android_step):

    def __init__(self, wait_time = 1000, **kwargs):
        android_step.__init__(self, **kwargs)
        self.wait_time = wait_time

    def do(self):
        open_phone(serial = self.serial)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"descriptionContains": "Recents"},
                              view_to_check = {"resourceId": "com.android.dialer:id/floating_action_button"},
                              wait_time = self.wait_time)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "com.android.dialer:id/floating_action_button"},
                              view_to_check = {"resourceId" :"com.android.dialer:id/dialpad_floating_action_button"},
                              wait_time = self.wait_time)()

    def check_condition(self):
        # Check performed in do()
        return True



class send_sms(android_step):

    def __init__(self, number, content, view_to_check = None, view_presence = True, wait_time = 5000, **kwargs):
        android_step.__init__(self, **kwargs)
        self.wait_time = wait_time
        self.number = number
        self.content = content
        self.view_to_check = view_to_check
        self.view_presence = view_presence

    def do(self):
        open_new_conversation(serial = self.serial,
                              number = self.number)()
        ui_steps.edit_text(serial = self.serial,
                            view_to_find = {"resourceId": "com.google.android.apps.messaging:id/compose_message_text"},
                            value = self.content)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "com.google.android.apps.messaging:id/self_send_icon"},
                              view_to_check  = self.view_to_check,
                              view_presence = self.view_presence,
                              wait_time = self.wait_time)()
    def check_condition(self):
        # Check performed in do()
        return True

class delete_conversation(android_step):

    def __init__(self, number, wait_time = 1000, **kwargs):
        android_step.__init__(self, **kwargs)
        self.wait_time = wait_time
        self.number = number

    def do(self):
        open_new_conversation(serial = self.serial,
                          number = self.number)()
        ui_steps.edit_text(serial = self.serial,
                           view_to_find = {"resourceId": "com.google.android.apps.messaging:id/compose_message_text"},
                           value = " ")()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"description": "More options"},
                              view_to_check = {"text": "Delete"},
                              wait_time = self.wait_time)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "Delete"},
                              view_to_check = {"resourceId": "android:id/alertTitle",
                                                "text": "Delete this conversation?"},
                              wait_time = self.wait_time)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "Delete"},
                              view_to_check = {"resourceId": "com.google.android.apps.messaging:id/start_new_conversation_button"},
                              wait_time = self.wait_time)()

    def check_condition(self):
        # Check performed in do()
        return True


class dial_number(ui_step):

    def __init__(self, number, **kwargs):
        self.number = number
        ui_step.__init__(self, **kwargs)

    def do(self):
        for digit in self.number:
            self.uidevice(text = digit).click()

    def check_condition(self):
        # Check performed in do()
        return True


class call_a_number(ui_step):

    def __init__(self, number, view_to_check = {"text": "Dialing"}, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.number = number
        self.wait_time = wait_time
        self.view_to_check = view_to_check


    def do(self):
        open_phone_dialer(serial = self.serial)()
        dial_number(serial = self.serial,
                    number = self.number)()

        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "com.android.dialer:id/dialpad_floating_action_button"},
                              view_to_check = self.view_to_check,
                              wait_time = self.wait_time)()

    def check_condition(self):
        # Check performed in do()
        return True

class end_call(ui_step):

    def __init__(self, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time


    def do(self):
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId": "com.android.dialer:id/floating_end_call_action_button"},
                              view_to_check = {"resourceId": "com.android.dialer:id/floating_end_call_action_button"},
                              view_presence = False,
                              wait_time = self.wait_time)()

    def check_condition(self):
        # Check performed in do()
        return True

class open_cellular_settings(android_step):

    """ description:
            Opens the Cellular Networks Settings page using UI or intent.

        usage:
            UI: telephony_steps.open_cellular_settings(serial=self.serial)()
            intent: telephony_steps.open_cellular_settings(serial=self.serial,
                                                           intent=True)()
        tags:
            ui, android, settings, cellular, intent, root
   """
    def __init__(self, intent = False, wait_time = 1000, **kwargs):
        self.intent = intent
        self.wait_time = wait_time
        android_step.__init__(self, **kwargs)

    def do(self):
        if self.intent:
            adb_steps.root_connect_device(serial = self.serial)()
            adb_steps.am_start_command(serial = self.serial,
                                        component = "com.android.phone/.Settings")()
        else:
            ui_steps.open_settings_app(serial = self.serial,
                                        view_to_find = {"text":"More"},
                                        view_to_check = {"text":"Cellular networks"},
                                        wait_time = self.wait_time)()
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text":"Cellular networks"},
                                  view_to_check = {"text":"Cellular network settings"},
                                  wait_time = self.wait_time)()

    def check_condition(self):
        # Check performed in do()
        return True

class set_cellular_network_type(ui_step):

    def __init__(self, network_type, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.network_type = network_type
        self.wait_time = wait_time

    def do(self):
        open_cellular_settings(serial = self.serial,
                               intent = True)()
        ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text":"Preferred network type"},
                                  view_to_check = {"resourceId":"android:id/alertTitle",
                                                   "text": "Preferred network type"},
                                  wait_time = self.wait_time)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": self.network_type},
                              view_to_check = {"resourceId":"android:id/summary",
                                                "text": self.network_type},
                              wait_time = self.wait_time)()


    def check_condition(self):
        # Check performed in do()
        return True

class open_data_usage_settings(adb_step):

    """ description:
            Opens the Cellular Networks Settings page using an intent.

        usage:
            telephony_steps.open_cellular_settings(serial=self.serial)()

        tags:
            ui, android, settings, data, intent
    """

    def do(self):
        adb_steps.am_start_command(serial = self.serial,
                                   component = "com.android.settings/.Settings\$DataUsageSummaryActivity")()
    def check_condition(self):
        # Check performed in do()
        return True

class set_cellular_data_state(ui_step):

    def __init__(self, state = "ON", wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.state = state
        self.wait_time = wait_time

        if self.state == "OFF":
            self.click_to_close_popup = {"text":"OK"}
        else:
            self.click_to_close_popup = None


    def do(self):
        self.step_data = ui_steps.click_switch(serial = self.serial,
                                               view_to_find = {"text": "Cellular data"},
                                               state = self.state,
                                               right_of = True,
                                               click_to_close_popup = self.click_to_close_popup,
                                               wait_time = self.wait_time)()

    def check_condition(self):
        # Check performed in do()
        return True

class check_cellular_network_type(ui_step):

    def __init__(self, cellular_network_type, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.cellular_network_type = cellular_network_type
        self.wait_time = wait_time

    def do(self):
        ui_steps.open_settings_app(serial = self.serial,
                                    view_to_find = {"text": "About phone"},
                                    view_to_check = {"text": "Status"},
                                    wait_time = self.wait_time)()
        ui_steps.click_view(serial = self.serial,
                            view = self.uidevice(text = "Status"),
                            view_to_check = {"text": "SIM status"},
                            wait_time = self.wait_time)()
        ui_steps.click_view(serial = self.serial,
                            view = self.uidevice(text = "SIM status"),
                            view_to_check = {"text": "Cellular network type"},
                            wait_time = self.wait_time)()


    def check_condition(self):
        return self.uidevice(textContains = self.cellular_network_type).wait.exists(timeout = self.wait_time)

class check_imei_code(ui_step):

    def __init__(self, imei_code_array, wait_time=5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time
        self.imei_code_array = imei_code_array
        self.set_passm("IMEI Codes {0} is (are) valid!".format(self.imei_code_array))

    def do(self):
        self.step_data = True
        for i in range(len(self.imei_code_array)):
            if not self.uidevice(text = self.imei_code_array[i]).wait.exists(timeout = self.wait_time):
                self.step_data = False
                self.set_errorm("", "IMEI Code {0} is not valid!".format(self.imei_code_array))
                return
            elif not (re.match("\d", self.imei_code_array[i])):
                self.step_data = False
                self.set_errorm("", "IMEI Code {0} is not only numeric!".format(self.imei_code_array))
                return
            elif len(self.imei_code_array) == 15:
                self.step_data = False
                self.self.set_errorm("", "IMEI Code {0} is not 15 characters long!".format(self.imei_code_array))
                return

    def check_condition(self):
        return self.step_data


class imeiLuhnCheck(ui_step):

    def __init__(self, imei_code_array, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time
        self.imei_code_array = imei_code_array
        self.set_passm("The IMEI Code {0} is valid through Luhn algorithm check".format(self.imei_code_array))

    def do(self):
        self.step_data = True
        for i in range(len(self.imei_code_array)):
            if self.uidevice(text = self.imei_code_array[i]).wait.exists(timeout = self.wait_time):
                self.imeiCodeText = self.uidevice(text = self.imei_code_array[i]).info["text"]

                digits_of_imei = [int(d) for d in self.imeiCodeText]
                odd_digits = digits_of_imei[:-1:2]
                even_digits = digits_of_imei[1:-1:2]
                last_digit = digits_of_imei[-1]
                checkSum = sum(odd_digits)

                for d in even_digits:
                    checkSum += (d*2 % 10 + d*2/10)

                checkSum *= 9

                if checkSum % 10 != last_digit:
                    self.step_data = False
                    self.self.set_errorm("", "The IMEI Code {0} is not valid through Luhn algorithm check".format(self.imei_code_array))
                    return

    def check_condition(self):
        return self.step_data

class open_sim_pin_settings(ui_step):

    def __init__(self, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time

    def do(self):
        ui_steps.open_settings_app(serial = self.serial,
                               view_to_find = {"text": "Security"},
                               view_to_check = {"text": "SIM card lock"},
                               wait_time = self.wait_time)()
        ui_steps.click_view(serial = self.serial,
                            view = self.uidevice(text = "Set up SIM card lock"),
                            view_to_check = {"text": "SIM card lock settings"},
                            wait_time = self.wait_time)()
    def check_condition(self):
        return True

class set_sim_pin(ui_step):

    def __init__(self, state = "ON", pin = "1234", wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.pin = pin
        self.state = state
        self.view_to_find = {'text':'Lock SIM card'}
        self.wait_time = wait_time
        self.set_passm("Set switch {0} to {1}".format(self.view_to_find, self.state))
        self.set_errorm("", "Could not set switch {0} to {1}".format(self.view_to_find, self.state))

    def do(self):
        open_sim_pin_settings(serial = self.serial)()
        ui_steps.wait_for_view(view_to_find = self.view_to_find,
                      serial = self.serial)()
        self.switch = self.uidevice(**self.view_to_find).right(
                            className = "android.widget.Switch")

        if self.switch.info['text'] == self.state:
            self.step_data = False
        else:
            self.switch.click.wait()
            self.step_data = True
            ui_steps.edit_text(serial = self.serial,
                               view_to_find = {"resourceId": "android:id/edit"},
                               is_password = True,
                               value = self.pin)()
            ui_steps.click_button(serial = self.serial,
                         print_error = "Failed to close popup",
                         blocking = True,
                         view_to_find = {'text':'OK'})()

    def check_condition(self):
        return self.uidevice(**self.view_to_find).right(
                            className = "android.widget.Switch", text = self.state).\
                            wait.exists(timeout=self.wait_time)

class check_pin_is_requested(adb_step):

    def __init__(self,
                 enabled_pin = True,
                 wait_time = 120000,
                 with_reboot = True,
                 timeout_wait_adb = 120,
                 timeout_wait_ui = 180,
                 **kwargs):
        adb_step.__init__(self, **kwargs)
        self.enabled_pin = enabled_pin
        self.wait_time = wait_time
        self.with_reboot = with_reboot
        self.timeout_wait_adb = timeout_wait_adb
        self.timeout_wait_ui = timeout_wait_ui
        if self.enabled_pin:
            self.set_passm("Pin is enabled")
            self.set_errorm("", "Pin is not enabled")
        else:
            self.set_passm("Pin is not enabled")
            self.set_errorm("", "Pin is enabled, but it should not")

    def do(self):
        if self.with_reboot:
            print "[ {0} ] Rebooting".format(self.serial)
            self.adb_connection.reboot_device()
        local_steps.wait_for_adb(serial = self.serial,
                                 timeout = self.timeout_wait_adb)()
        adb_steps.wait_for_ui_processes(serial = self.serial,
                                        timeout = self.timeout_wait_ui)()

    def check_condition(self):
        if self.enabled_pin:
            return ui_steps.wait_for_view(serial = self.serial,
                                    view_to_find = {"resourceId":"com.android.systemui:id/simPinEntry"},
                                    timeout = self.wait_time)()
        else:
            return ui_utils.is_homescreen(serial = self.serial)


class enter_pin(ui_step):

    def __init__(self,
                 pin = "1234",
                 correct = True,
                 third_attempt = False,
                 phone_is_locked = False,
                 wait_time = 10000,
                 setup_wizard = False,
                 **kwargs):
        ui_step.__init__(self, **kwargs)
        self.pin = pin
        self.correct = correct
        self.third_attempt = third_attempt
        self.wait_time = wait_time
        self.setup_wizard = setup_wizard
        self.phone_is_locked = phone_is_locked
        if phone_is_locked:
            self.set_passm("Authorization failed. PUK is required to unlock the DUT")
            self.set_errorm("","Authorization succeeded without PUK even though the phone was locked")
        elif correct:
            self.set_passm("PIN {0} check was successful and DUT was unlocked".format(self.pin))
            self.set_errorm("", "PIN {0} check failed and DUT is still locked".format(self.pin))
        else:
            if not third_attempt:
                self.set_passm("PIN {0} is incorrect and PIN is asked again".format(self.pin))
                self.set_errorm("", "PIN is not asked again even though PIN {0} is not correct  ".format(self.pin))
            else:
                self.set_passm("PUK is required because PIN was entered wrong for the third time")
                self.set_errorm("","PUK is not required event though PIN was entered wrong for the third time")

    def do(self):
        dial_number(serial = self.serial, number = self.pin)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId":"com.android.systemui:id/key_enter"})()
        self.uidevice(resourceId = "com.android.systemui:id/key_enter").wait.gone(timeout = self.wait_time)
        if not self.correct and not self.phone_is_locked:
            if not self.third_attempt:
                ui_steps.wait_for_view(serial = self.serial,
                                                view_to_find = {"textContains":"Incorrect SIM PIN code, you have"},
                                                wait_time = self.wait_time)()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"text":"OK"},
                                      view_to_check = {"resourceId":"com.android.systemui:id/simPinEntry"},
                                      wait_time = self.wait_time)()
            else:
                ui_steps.wait_for_view(serial = self.serial,
                                view_to_find = {"text":"Incorrect SIM PIN code; you must now contact your operator to unlock your device."},
                                wait_time = self.wait_time)()
                ui_steps.click_button(serial = self.serial,
                                      view_to_find = {"text":"OK"},
                                      view_to_check = {"resourceId":"com.android.systemui:id/pukEntry"},
                                      wait_time = self.wait_time)()

    def check_condition(self):
        if self.phone_is_locked:
            return ui_steps.wait_for_view(serial = self.serial,
                                    view_to_find = {"resourceId":"com.android.systemui:id/pukEntry"},
                                    timeout = self.wait_time)()
        elif self.correct:
            if self.setup_wizard:
                if self.uidevice(resourceId = "com.android.systemui:id/lock_icon"):
                    self.uidevice.swipe(200, 500, 200, 0, 10)
                if adb_utils.is_power_state(serial = self.serial,
                                            state = "OFF"):
                    self.uidevice.wakeup()
                return ui_utils.is_view_displayed(serial = self.serial,
                                                  view_to_find = {"resourceId":
                                                  "com.google.android.setupwizard:id/start"})
            else:
                return ui_utils.is_homescreen(serial = self.serial)
        else:
            # Check performed in do()
            return True

class enter_puk(adb_step):

    def __init__(self, puk, wait_time = 10000,  **kwargs):
        adb_step.__init__(self, **kwargs)
        self.puk = puk
        self.wait_time = wait_time

    def do(self):
        dial_number(serial = self.serial, number = self.puk)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId":"com.android.systemui:id/key_enter"},
                              view_to_check = {'text':"Enter desired PIN code"})()

    def check_condition(self):
        # Check performed in do()
        return True


class unlock_sim(adb_step):
    def __init__(self, puk, new_pin, correct = True, last_attempts = False, wait_time = 10000,  **kwargs):
        adb_step.__init__(self, **kwargs)
        self.puk = puk
        self.correct = correct
        self.last_attempts = last_attempts
        if correct:
            self.set_passm("PUK {0} check was successful and DUT was unlocked".format(self.puk))
            self.set_errorm("", "PUK {0} check failed and DUT is still locked".format(self.puk))
            self.view_to_check = None
        else:
            self.set_passm("PUK {0} check failed and DUT is still locked".format(self.puk))
            self.set_errorm("", "PUK {0} check was successful even though PUK is not correct".format(self.puk))
            if last_attempts:
                self.view_to_check = {'textContains':'before SIM becomes permanently unusable.'}
                self.view_to_check2 = {'text':'SIM is now disabled. Enter PUK code to continue. Contact carrier for details.'}
            else:
                self.view_to_check = {'text':'SIM is now disabled. Enter PUK code to continue. Contact carrier for details.'}
        self.new_pin = new_pin
        self.wait_time = wait_time

    def do(self):
        enter_puk(serial = self.serial, puk = self.puk)()
        dial_number(serial = self.serial, number = self.new_pin)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId":"com.android.systemui:id/key_enter"},
                              view_to_check = {'text':"Confirm desired PIN code"})()
        dial_number(serial = self.serial, number = self.new_pin)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId":"com.android.systemui:id/key_enter"},
                              view_to_check = self.view_to_check)()
        if self.last_attempts:
            ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text":"OK"},
                              view_to_check = self.view_to_check2)()

        ui_steps.unlock_device(serial=self.serial)()
        ui_steps.press_home(serial=self.serial)()

    def check_condition(self):
        if self.correct:
            return ui_utils.is_homescreen(serial = self.serial)
        else:
            # Check performed in do()
            return True


class checkSVCode(ui_step):

    def __init__(self, sv_code, wait_time=5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time
        self.sv_code = sv_code
        self.set_passm("The SV Code {0} exists and is 2 digits long!".format(self.sv_code))

    def do(self):
        self.step_data = True

        if self.sv_code is None:
            self.step_data = False
            self.set_errorm("", "SV Code {0} does not exist!".format(self.sv_code))
            return

        elif len(self.sv_code) != 2:
            self.step_data = False
            self.set_errorm("", "SV Code {0} does not have 2 digits!".format(self.sv_code))
            return

        else:
            for digit in self.sv_code:
                if not digit.isdigit():
                    self.step_data = False
                    self.set_errorm("", "SV Codes {0}: {1} is not a digit!".format(self.sv_code, digit))
                    return

    def check_condition(self):
        return self.step_data


class setup_set_time_and_date(ui_step):

    def __init__(self, reference_cellular_data_switch_state_value, wait_time = 5000, **kwargs):
        self.reference_cellular_data_switch_state_value = reference_cellular_data_switch_state_value
        self.wait_time = wait_time
        self.step_data = None
        ui_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.open_settings_app(serial= self.serial,
                           view_to_find ={"text": "Data usage"},
                           view_to_check = {"text": "Cellular data"})()

        reference_cellular_data_switch_state_value = self.uidevice(className = "android.widget.Switch").info["text"]

        self.step_data = reference_cellular_data_switch_state_value

        if self.uidevice(className = "android.widget.Switch").info["text"] != self.reference_cellular_data_switch_state_value:
            ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text": "Cellular data"})()

            if self.uidevice(resourceId = "android:id/message").wait.exists(timeout = self.wait_time):
                ui_steps.click_button(serial = self.serial,
                                  view_to_find = {"text": "OK"},
                                  view_to_check = {"text": "Cellular data"})()

        ui_steps.press_home(serial = self.serial)()

    def check_condition(self):
        # Check performed in do()
        return True


class get_device_date(ui_step, adb_step):

    def __init__(self, reference_cellular_data_switch_state_value, ntp_switch_state_value, **kwargs):
        self.reference_cellular_data_switch_state_value = reference_cellular_data_switch_state_value
        self.ntp_switch_state_value = ntp_switch_state_value
        adb_step.__init__(self, **kwargs)
        ui_step.__init__(self, **kwargs)

    def wait_for_date_to_change(self, ref_date, date_difference = 5):
            for i in range(10):
                time.sleep(1)
                date_dut = adb_utils.get_dut_date(serial=self.serial)
                date_dut = int(datetime.datetime.strftime(date_dut, "%Y%m%d%H%M"))
                ref_date = int(datetime.datetime.strftime(ref_date, "%Y%m%d%H%M"))
                diff_date = abs(ref_date - date_dut)

                if diff_date < date_difference:
                    self.set_passm("The date on DUT and Host differs by less than 5 min!")
                    return True
                self.set_errorm("", "The date on DUT and Host differs by more than 5 min!")
                self.step_data = False
                return False

    def do(self):
        self.step_data = True
        date_dut = adb_utils.get_dut_date(serial = self.serial)
        date_host = common_utils.get_host_date()
        diff_date = common_utils.get_date_diff(date_dut, date_host)

        if self.reference_cellular_data_switch_state_value == "ON" and self.ntp_switch_state_value == "ON" and diff_date > 5:

            self.set_passm("The Date on the DUT and HOST differs by more than 5 minutes!")
            ui_steps.click_button(serial = self.serial,
                            view_to_find = {"text": "Automatic date & time"},
                            view_to_check = {"text": "Set date"})()

            self.wait_for_date_to_change(date_host)

        elif self.reference_cellular_data_switch_state_value == "ON" and self.ntp_switch_state_value == "OFF" and diff_date > 5:
            self.set_passm("The date on DUT and host differs by more than 5 min!")

    def check_condition(self):
        return self.step_data

class enable_disable_nitz_status(ui_step):

    """ description:
            Enables/Disables the Automatic Time Zone and checks if the date/time corresponds.

        usage:
            telephony_steps.enable_disable_nitz_status(serial=self.serial, time_zone_switch_value = True for "ON"/.\
                                                                                                False for "OFF")()

        tags:
            ui, android, settings, timezone
    """

    def __init__(self, time_zone_switch_value, enable_back_to_on, wait_time = 5000, **kwargs):
        self.time_zone_switch_value = time_zone_switch_value
        self.enable_back_to_on = enable_back_to_on
        self.wait_time = wait_time
        ui_step.__init__(self, **kwargs)

    def do(self):
        self.step_data = True

        initial_date_dut = adb_utils.get_dut_date(serial=self.serial, gmt_date=False)
        initial_date_dut = int(datetime.datetime.strftime(initial_date_dut, "%Y%m%d%H%M"))

        ui_steps.enable_disable_auto_timezone(serial=self.serial, time_zone_switch_value = False)()

        ui_steps.click_button(serial=self.serial,
                             view_to_find={"text": "Select time zone"},
                             view_to_check={"resourceId": "android:id/text1"})()
        if self.uidevice(resourceId="android:id/text1").wait.exists(timeout=3000):
            nr_of_timezones = self.uidevice(resourceId="android:id/text1").count
            self.uidevice(resourceId="android:id/text1", instance=nr_of_timezones - 1).click()

        self.auto_timezone_checkbox = self.uidevice(className="android.widget.ListView", resourceId="android:id/list").\
            child_by_text("Automatic time zone", className="android.widget.LinearLayout").\
            child(className="android.widget.Switch")

        final_date_dut = adb_utils.get_dut_date(serial=self.serial, gmt_date=False)
        final_date_dut = int(datetime.datetime.strftime(final_date_dut, "%Y%m%d%H%M"))
        diff_date = abs(final_date_dut - initial_date_dut)

        if self.enable_back_to_on and diff_date > 100:
            self.auto_timezone_checkbox.click()
            final_date_dut = adb_utils.get_dut_date(serial=self.serial, gmt_date=False)
            final_date_dut = int(datetime.datetime.strftime(final_date_dut, "%Y%m%d%H%M"))
            diff_date = abs(final_date_dut - initial_date_dut)

            if diff_date < 100:
                self.set_passm("The time on DUT differs by less than 1 hour after timezone change!")
            else:
                self.step_data = False
                self.set_errorm("", "The date on DUT differs by more than 1 hour after timezone change!")

        elif not self.enable_back_to_on:
            if diff_date > 100:
                self.set_passm("The time on DUT differs by more than 1 hour after timezone change!")
            else:
                self.step_data = False
                self.set_errorm("", "Automatic time zone button did not work as expected!")

    def check_condition(self):
        #Check performed in do()
        return self.step_data


class change_pin(ui_step):
    """ description:
        Allows changing the PIN of the DUT's SIM

    usage:
        telephony_steps.change_pin(serial, old_pin, new_pin, lock_sim_switch_value)()

    tags:
        ui, android, settings, pin, sim card
    """
    def __init__(self, old_pin, new_pin1, lock_sim_switch_value, new_pin2 = None, wait_time = 5000, **kwargs):
        self.old_pin = old_pin
        self.new_pin1 = new_pin1
        self.lock_sim_switch_value = lock_sim_switch_value
        self.wait_time = wait_time
        ui_step.__init__(self, **kwargs)

        if new_pin2 is None:
            self.new_pin2 = new_pin1
        else:
            self.new_pin2 = new_pin2

    def do(self):
        open_sim_pin_settings(serial = self.serial)()
        ui_steps.click_button(serial=self.serial,
                             view_to_find={"text": "Change SIM PIN"},
                             view_to_check={"resourceId":"android:id/message", "text": "Old SIM PIN"})()

        self.step_data = True

        if len(self.new_pin1) < 4 or len(self.new_pin1) > 8:
            # Enter old pin
            ui_steps.edit_text(serial = self.serial,
                               is_password = True,
                               view_to_find={"resourceId": "android:id/edit"},
                               value = self.old_pin)()

            ui_steps.click_button(serial=self.serial,
                                 view_to_find={"resourceId": "android:id/button1", "text": "OK"},
                                 view_to_check={"resourceId": "android:id/message", "text": "New SIM PIN"})()

            #   Enter new pin with less than 4 digits / more than 8 digits
            ui_steps.edit_text(serial = self.serial,
                               is_password = True,
                               view_to_find={"resourceId": "android:id/edit"},
                               value = self.new_pin1)()
            ui_steps.click_button(serial=self.serial,
                                 view_to_find={"resourceId": "android:id/button1", "text": "OK"},
                                 view_to_check={"resourceId": "android:id/alertTitle", "text": "SIM PIN"})()

            if self.uidevice(resourceId="android:id/message").wait.exists(timeout = self.wait_time):
                if "Incorrect" in self.uidevice(resourceId="android:id/message").info["text"]:
                    self.set_passm("SIM PIN cannot be changed with a PIN less than 4 digits or more than 8 digits!")
            else:
                self.step_data = False
                self.set_errorm("", "ERROR! PIN SIM might have been changed with a PIN less than 4 digits or more than 8 digits")

        else:
            #   Enter old pin
            ui_steps.edit_text(serial = self.serial,
                               is_password = True,
                               view_to_find={"resourceId": "android:id/edit"},
                               value = self.old_pin)()
            ui_steps.click_button(serial=self.serial,
                                 view_to_find={"resourceId": "android:id/button1", "text": "OK"},
                                 view_to_check={"resourceId": "android:id/message", "text": "New SIM PIN"})()

            #   Enter new pin with at least 4 digits and maximum 8 digits
            ui_steps.edit_text(serial = self.serial,
                               is_password = True,
                               view_to_find={"resourceId": "android:id/edit"},
                               value = self.new_pin1)()
            ui_steps.click_button(serial=self.serial,
                                 view_to_find={"resourceId": "android:id/button1", "text": "OK"},
                                 view_to_check={"resourceId": "android:id/message", "text": u"Re\u2011type new PIN"})()

            #   Re-enter the new pin
            #   If the new PINs coincide, enter new_pin1 the second time
            if self.new_pin1 == self.new_pin2:
                ui_steps.edit_text(serial = self.serial,
                                   is_password = True,
                                   view_to_find={"resourceId": "android:id/edit"},
                                   value = self.new_pin1)()
                ui_steps.click_button(serial=self.serial,
                                     view_to_find={"resourceId": "android:id/button1", "text": "OK"},
                                     view_to_check={"resourceId":"android:id/title", "text": "Change SIM PIN"})()

                self.lock_sim_card_checkbox = self.uidevice(className="android.widget.FrameLayout").\
                child_by_text("Lock SIM card", className="android.widget.LinearLayout").\
                child(className="android.widget.Switch")

                #   If the option should not be enabled, click to deactivate
                if not self.lock_sim_switch_value and self.lock_sim_card_checkbox.info["checked"]:
                    self.lock_sim_card_checkbox.click()
                    ui_steps.edit_text(serial = self.serial,
                                       is_password = True,
                                       view_to_find={"resourceId": "android:id/edit"},
                                       value = self.new_pin1)()
                    ui_steps.click_button(serial=self.serial,
                                     view_to_find={"resourceId": "android:id/button1", "text": "OK"},
                                     view_to_check={"text": "Change SIM PIN"})()

                    self.step_data = True
                    if self.lock_sim_card_checkbox.info["checked"]:
                        self.step_data = False
                        self.set_errorm("", "Could not {0} the Lock SIM card option".format("disable" if self.lock_sim_card_checkbox.info["checked"] else "enable"))
                    else:
                        self.set_passm("The PIN was changed with a pin between 4 and 8 digits!")

            else:
                # If the PINs do not coincide, enter new_pin2 the second time
                ui_steps.edit_text(serial = self.serial,
                                   is_password = True,
                                   view_to_find={"resourceId": "android:id/edit"},
                                   value = self.new_pin2)()
                ui_steps.click_button(serial=self.serial,
                                     view_to_find={"resourceId": "android:id/button1", "text": "OK"},
                                     view_to_check={"resourceId": "android:id/alertTitle", "text": "SIM PIN"})()

                if self.uidevice(resourceId="android:id/message").wait.exists(timeout = self.wait_time):
                    if u"PINs don\u2019t match" in self.uidevice(resourceId="android:id/message").info["text"]:
                        self.set_passm("New PINs do not match!")
                    else:
                        self.step_data = False
                        self.set_errorm("", "SIM PIN1 was changed even if PINs are different!")

    def check_condition(self):
        #Check performed in do()
        return self.step_data


class enter_wrong_old_pin(ui_step):
    """ description:
        This steps checks that entering a wrong pin is not permitted when changing the SIM's PIN

        usage:
        telephony_steps.enter_wrong_pin(serial, wrong_old_pin, new_pin)()

        tags:
        ui, android, settings, pin, sim card
    """
    def __init__(self, wrong_old_pin, new_pin, wait_time = 5000, **kwargs):
        self.wrong_old_pin = wrong_old_pin
        self.new_pin = new_pin
        self.wait_time = wait_time
        ui_step.__init__(self, **kwargs)

    def do(self):

        open_sim_pin_settings(serial = self.serial)()

        ui_steps.click_button(serial=self.serial,
                             view_to_find={"text": "Change SIM PIN"},
                             view_to_check={"resourceId":"android:id/message", "text": "Old SIM PIN"})()

        self.step_data = True

        if self.wrong_old_pin != telephony_defaults.sim['default_pin']:
            #   Enter the old pin, but different from the default one
            ui_steps.edit_text(serial = self.serial,
                               is_password = True,
                               view_to_find={"resourceId": "android:id/edit"},
                               value = self.wrong_old_pin)()
            ui_steps.click_button(serial=self.serial,
                                 view_to_find={"resourceId": "android:id/button1", "text": "OK"},
                                 view_to_check={"resourceId": "android:id/message", "text": "New SIM PIN"})()

            #   Enter new pin with at least 4 digits and maximum 8 digits
            ui_steps.edit_text(serial = self.serial,
                               is_password = True,
                               view_to_find={"resourceId": "android:id/edit"},
                               value = self.new_pin)()
            ui_steps.click_button(serial=self.serial,
                                 view_to_find={"resourceId": "android:id/button1", "text": "OK"},
                                 view_to_check={"resourceId": "android:id/message", "text": u"Re\u2011type new PIN"})()

            #   Re-enter the new pin
            ui_steps.edit_text(serial = self.serial,
                               is_password = True,
                               view_to_find={"resourceId": "android:id/edit"},
                               value = self.new_pin)()
            ui_steps.click_button(serial=self.serial,
                                 view_to_find={"resourceId": "android:id/button1", "text": "OK"},
                                 view_to_check={"resourceId":"android:id/title", "text": "Change SIM PIN"})()

            if logcat_steps.grep_for("Incorrect SIM PIN code, you have", serial = self.serial):
                self.set_passm("PIN1 cannot be changed as old PIN1 is incorrect!")
            else:
                self.step_data = False
                self.set_errorm("", "PIN1 could be changed when entering an incorrect PIN1!")

        else:
            self.step_data = False
            self.set_errorm("", "Old PIN is the same as default PIN!")

    def check_condition(self):
        #Check performed in do()
        return self.step_data


class enter_puk_with_mmi_code(ui_step):

    def __init__(self, puk_code, pin_code, wait_time = 5000, **kwargs):
        ui_step.__init__(self, **kwargs)
        self.wait_time = wait_time
        self.puk_code = puk_code
        self.pin_code = pin_code

    def do(self):
        self.uidevice.wait.idle()
        self.step_data = True
        if self.uidevice(resourceId = "com.android.phone:id/digits").wait.exists(timeout = self.wait_time):
           adb_steps.command(serial = self.serial,
                             command ="input text {0}".format("**05*" + self.puk_code + "*" + self.pin_code + "*" + self.pin_code + "#" ))
           self.set_passm("MMI Code was successfully entered")

        else:
            self.step_data = False
            self.set_errorm("", "View to enter MMI Code did not appear!")

    def check_condition(self):
        return self.step_data


class create_or_modify_APN(ui_step):
    """ description:
        This steps creates or modifies an APN

        usage:
        telephony_steps.create_or_modify_APN(serial = serial,
                                            new_apn = True,
                                            apn_name = " test123")()

        tags:
        ui, android, settings, pin, sim card, puk
    """
    def __init__(self, new_apn, no_of_default_apns, restore_to_default, **kwargs):
        self.new_apn = new_apn
        self.no_of_default_apns = no_of_default_apns
        self.restore_to_default = restore_to_default
        ui_step.__init__(self, **kwargs)

    def check_apns_in_view(self, no_of_default_apns):
        self.step_data = True
        self.no_of_default_apns = no_of_default_apns
        for instances in range(0, self.no_of_default_apns):
            if self.uidevice(className = "android.widget.LinearLayout", instance = instances).wait.exists(timeout = 1000):
                self.set_passm("All default APNs appeared in view!")
            else:
                self.step_data = False
                self.set_errorm("", "Default APNs did not appear in view!")

    def do(self):
        ui_steps.open_settings(serial = self.serial)()

        views_to_find = [
                {"text": "More"},
                {"text": "Cellular networks"},
                {"text": "Access Point Names"}
        ]
        views_to_check = [
                {'text':"Airplane mode"},
                {'text':"Access Point Names"},
                {'text':"APNs"}
        ]

        steps = [
                ui_steps.click_button_with_scroll,
                ui_steps.click_button_with_scroll,
                ui_steps.click_button_with_scroll
        ]

        for i in range(len(views_to_find)):
                if steps[i] is ui_steps.click_button_with_scroll:
                    params = {"view_to_find": views_to_find[i],
                              "view_to_check": views_to_check[i]}

                steps[i](serial = self.serial,
                         **params)()

        self.check_apns_in_view(self.no_of_default_apns)

        if self.restore_to_default is True:
            views_to_find = [
                {"description": "More options"},
                {"text": "Reset to default"}]
            views_to_check = [
                {'text':"Reset to default"},
                None]

            steps = [
                ui_steps.click_button_with_scroll,
                ui_steps.click_button_with_scroll]

            for i in range(len(views_to_find)):
                if steps[i] is ui_steps.click_button_with_scroll:
                    params = {"view_to_find": views_to_find[i],
                              "view_to_check": views_to_check[i]}
                steps[i](serial = self.serial,
                         **params)()

            #   A sleep is required to let the APN refresh after restoring to default
            time.sleep(5)

        self.check_apns_in_view(self.no_of_default_apns)

        if self.new_apn is True:
            views_to_find = [
                {"className": "android.widget.TextView", "description": "New APN"},
                {"text":"Name", "resourceId": "android:id/title"},
                {"resourceId": "android:id/edit"},
                {"text": "OK"},
                {"text": "APN"},
                {"resourceId": "android:id/edit"},
                {"text": "OK"},
                {"description": "More options"},
                {"text": "Save", "resourceId": "android:id/title"}
            ]
            views_to_check = [
                {'text':"Edit access point"},
                {'text':"Name", "resourceId":"android:id/alertTitle"},
                "Testing123",
                {"resourceId": "android:id/title", "text": "APN"},
                {"resourceId": "android:id/alertTitle", "text": "APN"},
                "testing1234",
                {"resourceId": "android:id/title", "text": "APN"},
                {"resourceId": "android:id/title", "text": "Save"},
                {"text": "APNs"}
            ]

            steps = [
                ui_steps.click_button_with_scroll,
                ui_steps.click_button_with_scroll,
                ui_steps.edit_text,
                ui_steps.click_button_with_scroll,
                ui_steps.click_button_with_scroll,
                ui_steps.edit_text,
                ui_steps.click_button_with_scroll,
                ui_steps.click_button_with_scroll,
                ui_steps.click_button_with_scroll,
            ]

            for i in range(len(views_to_find)):
                if steps[i] is ui_steps.click_button_with_scroll:
                    params = {"view_to_find": views_to_find[i],
                              "view_to_check": views_to_check[i]}
                elif steps[i] is ui_steps.edit_text:
                    params = {"view_to_find": views_to_find[i],
                              "value": views_to_check[i]}

                steps[i](serial = self.serial,
                         **params)()

            ui_steps.wait_for_view(view_to_find = {"text": "Testing123"}, serial = self.serial)()
            adb_steps.put_device_into_sleep_mode(serial = self.serial)()

            # A delay is necessary between sleep mode and waking up the device
            time.sleep(3)
            adb_steps.wake_up_device(serial = self.serial)()
            ui_steps.wait_for_view(view_to_find = {"text": "Testing123"}, serial = self.serial)()

    def check_condition(self):
        #Check performed in do()
        return self.step_data


class delete_apn(ui_step):
    """ description:
        This steps deletes an existing APN

        usage:
        telephony_steps.delete_apn(serial = serial,
                                            )()

        tags:
        ui, android, settings, pin, sim card, puk
    """
    def __init__(self,  **kwargs):
        ui_step.__init__(self, **kwargs)

    def do(self):
        ui_steps.open_settings(serial = self.serial)()

        views_to_find = [
            {"text":"More"},
            {"text":"Cellular networks"},
            {"text":"Access Point Names"},
            {"resourceId":"android:id/title", "text": "Testing123"},
            {"description": "More options"},
            {"text": "Delete APN", "resourceId": "android:id/title"}
        ]
        views_to_check = [
            {'text':"Airplane mode"},
            {'text':"Access Point Names"},
            {'text':"APNs"},
            {"resourceId":"android:id/title", "text":"Name"},
            {"resourceId": "android:id/title", "text": "Delete APN"},
            {"text": "APNs"}
            ]

        for i in range(len(views_to_find)):
            ui_steps.click_button(serial = self.serial,
                                    view_to_find = views_to_find[i],
                                    view_to_check = views_to_check[i])()

    def check_condition(self):
        #Check performed in do()
        return True


class wake_up_device_with_sim_pin(adb_step):
    """ description:
        This steps enters the pin in the screen after reboot (if the PIN is enabled)

        usage:
        telephony_steps.wake_up_device_with_pin(serial, pin)()

        tags:
        ui, android, settings, pin, sim card
    """
    def __init__(self, sim_pin, wait_time = 5000, **kwargs):
        adb_step.__init__(self, **kwargs)
        self.sim_pin = sim_pin
        self.wait_time = wait_time

    def do(self):
        dial_number(serial = self.serial, number = self.sim_pin)()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"resourceId":"com.android.systemui:id/key_enter"},
                              view_to_check = {'resourceId':"com.google.android.googlequicksearchbox:id/launcher_search_button"})()

    def check_condition(self):
        # Check performed in do()
        return True