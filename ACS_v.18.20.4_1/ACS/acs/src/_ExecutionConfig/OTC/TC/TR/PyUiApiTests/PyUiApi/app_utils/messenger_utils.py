import datetime
import hashlib
import os
import time

from PyUiApi.common.uiautomator_utils import *
from PyUiApi.common.status_bar import *
from PyUiApi.common.uiautomator_extension import *
from PyUiApi.adb_helper.adb_utils import *
from PyUiApi.multi_dut_support.dut_manager import *
from PyUiApi.adb_helper import *

MESSAGING_STARTUP_TIMEOUT = 20

messaging_launcher_string = "com.google.android.apps.messaging/.ui.ConversationListActivity"


class Messaging(object):
    SMS_RECEIVE_TIME = 20
    start_time = 0
    end_time = 0

    @staticmethod
    def launch():
        AdbUtils.start_activity_from_shell(messaging_launcher_string)
        if not Messaging.messaging_launched:
            UiAutomatorUtils.launch_app_from_apps_menu(MESSAGING_SHORTCUT_NAME)

    @staticmethod
    def messaging_launched():
        if d(resourceId="Messenger", classmethod="android.widget.TextView").wait.exists(timeout=5000):
            return True
        else:
            return False

    @staticmethod
    def send_sms(receiver_number, message, timed=False):
        Messaging.fill_sms_info(receiver_number, message)
        if d(resourceId=MESSAGING_SEND).wait.exists(timeout=3000):
            if timed:
                Messaging.start_time = time.time()
            d(resourceId=MESSAGING_SEND).click()
        LOG.info("Sending sms message {} to number {}".format(message, receiver_number))

    @staticmethod
    def fill_sms_info(receiver_number, message):
        if d(resourceId=MESSAGING_START_CONVERSATION).wait.exists(timeout=3000):
            d(resourceId=MESSAGING_START_CONVERSATION).click()
        if d(text=MESSAGING_ALL_CONTACTS_TXT).wait.exists(timeout=3000):
            d(text=MESSAGING_ALL_CONTACTS_TXT).click()
        if d(resourceId=MESSAGING_RECEIPIENT).wait.exists(timeout=5000):
            if type(receiver_number) is list:
                phone_numbers = ",".join(receiver_number)
                AdbUtils.input_text(phone_numbers)
                d.press("enter")
                if d(resourceId=MESSAGING_CONFIRM_PARTICIPANTS).wait.exists(timeout=5000):
                    d(resourceId=MESSAGING_CONFIRM_PARTICIPANTS).click()
            else:
                d(resourceId=MESSAGING_RECEIPIENT).set_text(receiver_number)
            d(resourceId=MESSAGING_RECEIPIENT).set_text(receiver_number)
        time.sleep(5)
        d.press("enter")
        d.wait.idle()
        if d(resourceId=MESSAGING_MESSAGE_BOX).wait.exists(timeout=5000):
            d(resourceId=MESSAGING_MESSAGE_BOX).set_text(message)

    @staticmethod
    def generate_unique_sms():
        date_and_time = datetime.datetime.now()
        hash_obj = hashlib.sha1()
        hash_obj.update(str(date_and_time))
        LOG.info("Unique sms generated: " + hash_obj.hexdigest())
        return hash_obj.hexdigest()

    @staticmethod
    def reply_sms(receiver_number, message):
        StatusBar.open_notifications()
        if d(text="Reply").wait.exists(timeout=3000):
            phone_number = d(resourceId="android:id/title")[0].info["text"]
            d(text="Reply").click()
            if d(resourceId=MESSAGING_MESSAGE_BOX).wait.exists(timeout=5000):
                d(resourceId=MESSAGING_MESSAGE_BOX).set_text(message)
            if d(resourceId=MESSAGING_SEND).wait.exists(timeout=3000):
                d(resourceId=MESSAGING_SEND).click()

    @staticmethod
    def check_sms_received(sms):
        if d(resourceId=MESSAGING_SENDERS_LIST).wait.exists(timeout=25000):
            Messaging.end_time = time.time()
            d(resourceId=MESSAGING_SENDERS_LIST)[0].click()
        if d(text=sms).wait.exists(timeout=5000):
            return True
        return False

    @staticmethod
    def check_sms_received_in_statusbar(sms):
        StatusBar.open_notifications()
        return d(textContains=sms).wait.exists(timeout=1000)

    @staticmethod
    def check_last_sms_sent(sms):
        if d(resourceId=MESSAGING_SWIPEABLE_CONTENT).wait.exists(timeout=5000):
            d(resourceId=MESSAGING_SWIPEABLE_CONTENT)[0].click()
        if d(resourceId=MESSAGING_TEXT).wait.exists(timeout=5000):
            if d(resourceId=MESSAGING_TEXT)[len(d(resourceId=MESSAGING_TEXT)) - 1].text == sms:
                return True
        return False

    @staticmethod
    def remove_contact(name):
        if d(resourceId=MESSAGING_START_CONVERSATION).wait.exists(timeout=3000):
            d(resourceId=MESSAGING_START_CONVERSATION).click()
        if d(text=MESSAGING_ALL_CONTACTS_TXT).wait.exists(timeout=3000):
            d(text=MESSAGING_ALL_CONTACTS_TXT).click()
        if d(resourceId=MESSAGING_CONTACT_NAME).wait.exists(timeout=3000):
            indexes = d(resourceId=MESSAGING_CONTACT_NAME).count
            for i in range(indexes):
                if d(resourceId=MESSAGING_CONTACT_NAME)[i].text == name:
                    if d(resourceId=MESSAGING_CONTACT_ICON)[i].wait.exists(timeout=3000):
                        d(resourceId=MESSAGING_CONTACT_ICON)[i].click()
                        time.sleep(2)
                        if d(description=MESSAGING_MORE_OPTIONS_TXT).wait.exists(timeout=7000):
                            d(description=MESSAGING_MORE_OPTIONS_TXT).click()
                        if d(text=MESSAGING_DELETE_TXT).wait.exists(timeout=7000):
                            d(text=MESSAGING_DELETE_TXT).click()
                        if d(text=MESSAGING_CONFIRMATION_TXT).wait.exists(timeout=7000):
                            d(text=MESSAGING_CONFIRMATION_TXT).click()
                        break

    @staticmethod
    def remove_all_contacts():
        if d(resourceId=MESSAGING_START_CONVERSATION).wait.exists(timeout=3000):
            d(resourceId=MESSAGING_START_CONVERSATION).click()
        if d(text=MESSAGING_ALL_CONTACTS_TXT).wait.exists(timeout=3000):
            d(text=MESSAGING_ALL_CONTACTS_TXT).click()
        while (d(resourceId=MESSAGING_CONTACT_ICON).wait.exists(timeout=3000)):
            d(resourceId=MESSAGING_CONTACT_ICON)[0].click()
            if d(description=MESSAGING_MORE_OPTIONS_TXT).wait.exists(timeout=7000):
                d(description=MESSAGING_MORE_OPTIONS_TXT).click()
            if d(text=MESSAGING_DELETE_TXT).wait.exists(timeout=7000):
                d(text=MESSAGING_DELETE_TXT).click()
            if d(text=MESSAGING_CONFIRMATION_TXT).wait.exists(timeout=7000):
                d(text=MESSAGING_CONFIRMATION_TXT).click()
            time.sleep(2)

    @staticmethod
    def add_contact(name, number):
        Messaging.remove_contact(name)

        view_navigator = ViewNavigator()
        view_navigator.navigate_views(
            [MESSAGING_START_CONVERSATION, MESSAGING_ALL_CONTACTS_TXT, ("resourceId", MESSAGING_RECEIPIENT, number)])
        d.press("enter")
        time.sleep(5)
        view_navigator.navigate_views([MESSAGING_MESSAGE_BOX, MESSAGING_MORE_OPTIONS_TXT, MESSAGING_ADD_CONTACT_TXT,
                                       MESSAGING_ADD_CONTACT_UPPER_TXT, MESSAGING_CREATE_NEW_CONTACT_TXT,
                                       MESSAGING_KEEP_LOCAL_TXT, ("text", MESSAGING_CONTACT_NAME_TXT, name),
                                       MESSAGING_CONTACT_SAVE_TXT])

    @staticmethod
    def send_sms_to_contact(name, message):
        if d(resourceId=MESSAGING_START_CONVERSATION).wait.exists(timeout=3000):
            d(resourceId=MESSAGING_START_CONVERSATION).click()
        if d(text=MESSAGING_ALL_CONTACTS_TXT).wait.exists(timeout=3000):
            d(text=MESSAGING_ALL_CONTACTS_TXT).click()
        if d(text=name).wait.exists(timeout=5000):
            d(text=name).click()
        if d(resourceId=MESSAGING_MESSAGE_BOX).wait.exists(timeout=5000):
            d(resourceId=MESSAGING_MESSAGE_BOX).set_text(message)
        if d(resourceId=MESSAGING_SEND).wait.exists(timeout=3000):
            d(resourceId=MESSAGING_SEND).click()

    @staticmethod
    def open_settings():
        if d(description=MESSAGING_MORE_OPTIONS_TXT).wait.exists(timeout=2000):
            d(description=MESSAGING_MORE_OPTIONS_TXT).click()
        if d(text=MESSAGING_SETTINGS_TXT).wait.exists(timeout=3000):
            d(text=MESSAGING_SETTINGS_TXT).click()

    @staticmethod
    def enable_sms_encoding(enable_switch):
        Messaging.open_settings()
        if d(text=MESSAGING_ADVANCED_TXT).wait.exists(timeout=5000):
            d(text=MESSAGING_ADVANCED_TXT).click()

    @staticmethod
    def switch_ucs2_encoding(enable_switch):
        '''
        :param enable_switch: use 'true' or 'false' for the state of the ucs2 encoding
        '''
        UCS2_encoding_txt = "Simple characters only"
        classname_switch = "android.widget.Switch"
        Messaging.launch()
        Messaging.open_settings()
        if d(text=MESSAGING_ADVANCED_TXT).wait.exists(timeout=5000):
            d(text=MESSAGING_ADVANCED_TXT).click()
        d(text="Advanced settings").wait.exists(timeout=5000)
        mapping = UiAutomatorExtended.get_mapping_using_class(classname_switch)
        if "".join([key[1] for key in mapping if key[0] == UCS2_encoding_txt]) != enable_switch:
            UiAutomatorExtended.change_status_for_text(UCS2_encoding_txt, classname_switch)

    @staticmethod
    def open_last_sms_received():
        if d(resourceId=MESSAGING_SWIPEABLE_CONTENT).wait.exists(timeout=5000):
            d(resourceId=MESSAGING_SWIPEABLE_CONTENT)[0].click()
            return True
        return False

    @staticmethod
    def long_press_on_last_msg():
        if d(className=ANDROID_FRAME_LAYOUT, packageName=MESSAGING_PACKAGE, longClickable="True"):
            swipeable_object = d(className=ANDROID_FRAME_LAYOUT, packageName=MESSAGING_PACKAGE,
                                 longClickable="True")
            swipeable_object[len(swipeable_object) - 1].long_click()
            return True
        return False

    @staticmethod
    def retrieve_last_msg():
        if d(resourceId=MESSAGING_TEXT).wait.exists(timeout=5000):
            if len(d(resourceId=MESSAGING_TEXT)) == 1:
                return d(resourceId=MESSAGING_TEXT).text
            else:
                return d(resourceId=MESSAGING_TEXT)[len(d(resourceId=MESSAGING_TEXT)) - 1].text
        return ""

    @staticmethod
    def forward_last_sms(receiver_number, text_added_to_message=""):
        if Messaging.open_last_sms_received():
            text = Messaging.retrieve_last_msg()
        else:
            return False
        if Messaging.long_press_on_last_msg():
            if d(description=MESSAGING_FORWARD_TXT).wait.exists(timeout=5000):
                d(description=MESSAGING_FORWARD_TXT).click()
                Messaging.send_sms(receiver_number, text + text_added_to_message)
                return True
            else:
                return False
        else:
            return False

    @staticmethod
    def reset_timer():
        Messaging.start_time = 0
        Messaging.end_time = 0
