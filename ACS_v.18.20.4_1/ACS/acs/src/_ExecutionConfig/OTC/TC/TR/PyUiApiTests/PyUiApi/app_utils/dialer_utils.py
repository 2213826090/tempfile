from PyUiApi.common.uiautomator_utils import *
from PyUiApi.adb_helper.adb_utils import *
from PyUiApi.multi_dut_support.dut_manager import *
from PyUiApi.dut_info.dut_info import *
from PyUiApi.common.status_bar import *
from PyUiApi.app_utils.contacts_utils import Contacts

PHONE_launcher_string = "com.android.dialer/.DialtactsActivity"
TELEPHONE_launcher_string = "com.google.android.dialer/.extensions.GoogleDialtactsActivity"
WAIT_AFTER_DIAL = 5

WRONG_PIN_2 = 33333
FDN_MMI_PREFIX = "**052*"
FDN_MMI_STAR = "*"
FDN_MMI_SUFFIX = "#"


class Dialer(object):
    app_string = None
    DIALER_SEARCH_TXT = None

    @staticmethod
    def set_app_type():
        """
        This method must be called in order to switch between PHONE and TELEPHONE apps
        """
        AdbUtils.run_adb_cmd("am start -a android.intent.action.DIAL")
        output = AdbUtils.run_adb_cmd("dumpsys activity activities")
        if PHONE_launcher_string in output:
            Dialer.app_string = PHONE_launcher_string
            Dialer.DIALER_SEARCH_TXT = DIALER_PHONE_SEARCH_TXT
        elif TELEPHONE_launcher_string in output:
            Dialer.app_string = TELEPHONE_launcher_string
            Dialer.DIALER_SEARCH_TXT = DIALER_TELEPHONE_SEARCH_TXT
        else:
            Dialer.app_string = None

    @staticmethod
    def launch():
        Dialer.set_app_type()
        AdbUtils.start_activity_from_shell(Dialer.app_string)
        if not Dialer.dialer_launched:
            UiAutomatorUtils.launch_app_from_apps_menu(Dialer.app_string)

    @staticmethod
    def dialer_launched():
        if d(text=Dialer.DIALER_SEARCH_TXT).wait.exists(timeout=5000):
            return True
        else:
            return False

    @staticmethod
    def dial_number(number):
        if not d(text=Dialer.DIALER_SEARCH_TXT).wait.exists(timeout=5000):
            LOG.info('Failed to find %s' % Dialer.DIALER_SEARCH_TXT)
            return False
        d(text=Dialer.DIALER_SEARCH_TXT).click()
        if not d(resourceId=DIALER_SEARCH_VIEW).wait.exists(timeout=5000):
            LOG.info('Failed to find %s' % DIALER_SEARCH_VIEW)
            return False
        d(resourceId=DIALER_SEARCH_VIEW).set_text(str(number))
        d.press("enter")
        time.sleep(1)  # Wait for the UI to sync
        if not d(textContains=DIALER_CALL_TXT).wait.exists(timeout=5000):
            LOG.info('Failed to find %s.' % DIALER_CALL_TXT)
            return False
        if not d(textContains=DIALER_CALL_TXT).click.wait(timeout=10000):
            return False
        return True

    @staticmethod
    def is_call_active(with_number=None):
        """
        Check that a call is active
        :param with_number: the number it is paired
        :return: True/False
        """
        if not d(resourceId=DIALER_CONTACT_NAME).wait.exists(timeout=5000):
            LOG.error('Failed to find %s' % DIALER_CONTACT_NAME)
            return False
        if with_number:
            found_number = d(resourceId=DIALER_CONTACT_NAME).info['text'].replace(' ', '')
            if found_number != with_number:
                LOG.warning('Found number %s' % found_number)
                return False
        return True

    @staticmethod
    def dial_number_using_keyboard(number):
        if d(description=DIALER_SPEED_DIAL).wait.exists(timeout=5000):
            d(description=DIALER_SPEED_DIAL).click()
        if d(resourceId=DIALER_KEYBOARD).wait.exists(timeout=5000):
            d(resourceId=DIALER_KEYBOARD).click()
        if d(resourceId=DIALER_DIGITS_INPUT).wait.exists(timeout=5000):
            d(resourceId=DIALER_DIGITS_INPUT).set_text(str(number))
        d.press("enter")
        if d(resourceId=DIALER_CALL).wait.exists(timeout=5000):
            d(resourceId=DIALER_CALL).click()

    @staticmethod
    def navigate_to_call_settings():
        if d(resourceId=DIALER_OPTIONS).wait.exists(timeout=5000):
            d(resourceId=DIALER_OPTIONS).click()
        if d(text=DIALER_SETTINGS_TXT).wait.exists(timeout=5000):
            d(text=DIALER_SETTINGS_TXT).click()
        if d(text=DIALER_CALLS_TXT).wait.exists(timeout=5000):
            d(text=DIALER_CALLS_TXT).click()

    @staticmethod
    def navigate_to_aditional_settings():
        Dialer.navigate_to_call_settings()
        if d(text=DIALER_ADDITIONAL_SETTINGS_TXT).wait.exists(timeout=5000):
            d(text=DIALER_ADDITIONAL_SETTINGS_TXT).click()

    @staticmethod
    def get_caller_id_value():
        Dialer.navigate_to_aditional_settings()
        time.sleep(3)
        if d(text=DIALER_CALLER_ID).wait.exists(timeout=3000):
            return d(resourceId=DIALER_SUMMARY)[0].text

    @staticmethod
    def set_caller_id(set_value):
        available_values = ["Network default", "Hide number", "Show number"]
        if set_value not in available_values:
            return
        Dialer.navigate_to_aditional_settings()
        time.sleep(7)
        if d(text=DIALER_CALLER_ID).wait.exists(timeout=5000):
            d(text=DIALER_CALLER_ID).click()
        if d(text=set_value).wait.exists(timeout=5000):
            d(text=set_value).click()

    @staticmethod
    def enable_call_waiting():
        """
        Enable Call waiting from Additional settings
        :return: True/False
        """
        status = Dialer.get_call_waiting_status()
        if status is None:
            LOG.error('Failed to determine Call waiting status.')
            return False
        if status:
            LOG.info('Call waiting already enabled. Nothing to do.')
            return True
        d(text=DIALER_CALL_WAITING_TXT).right(className=ANDROID_CHECKBOX_CLASSNAME).click()
        # The ui refreshes
        time.sleep(3)
        if not d(text=DIALER_CALL_WAITING_TXT).wait.exists(timeout=10000):
            LOG.error('Failed to find %s after refresh.' % ANDROID_CHECKBOX_CLASSNAME)
            return False
        new_status = d(text=DIALER_CALL_WAITING_TXT).right(className=ANDROID_CHECKBOX_CLASSNAME).info['checked']
        if status == new_status:
            LOG.error('Failed to enable call waiting.')
            return False
        return True

    @staticmethod
    def disable_call_waiting():
        """
        Disable Call waiting from Additional settings
        :return: True/False
        """
        status = Dialer.get_call_waiting_status()
        if status is None:
            LOG.error('Failed to determine Call waiting status.')
            return False
        if not status:
            LOG.info('Call waiting already disabled. Nothing to do.')
            return True
        d(text=DIALER_CALL_WAITING_TXT).right(className=ANDROID_CHECKBOX_CLASSNAME).click()
        # The ui refreshes
        time.sleep(3)
        if not d(text=DIALER_CALL_WAITING_TXT).wait.exists(timeout=10000):
            LOG.error('Failed to find %s after refresh.' % ANDROID_CHECKBOX_CLASSNAME)
            return False
        new_status = d(text=DIALER_CALL_WAITING_TXT).right(className=ANDROID_CHECKBOX_CLASSNAME).info['checked']
        if status == new_status:
            LOG.error('Failed to disable call waiting.')
            return False
        return True

    @staticmethod
    def get_call_waiting_status():
        """
        Get Call waiting status from Additional settings
        :return: True/False/None (enabled/disabled/error)
        """
        Dialer.navigate_to_aditional_settings()
        time.sleep(5)  # Wait for the ui to sync
        if not d(text=DIALER_CALL_WAITING_TXT).wait.exists(timeout=10000):
            LOG.error('Failed to find %s.' % DIALER_CALL_WAITING_TXT)
            return None
        if not d(text=DIALER_CALL_WAITING_TXT).right(className=ANDROID_CHECKBOX_CLASSNAME).wait.exists():
            LOG.error('Failed to find %s' % ANDROID_CHECKBOX_CLASSNAME)
            return None
        status = d(text=DIALER_CALL_WAITING_TXT).right(className=ANDROID_CHECKBOX_CLASSNAME).info['checked']
        return status

    @staticmethod
    def set_voicemail_number(number):
        Dialer.navigate_to_call_settings()
        if d(text=DIALER_VOICEMAIL_TXT).wait.exists(timeout=5000):
            d(text=DIALER_VOICEMAIL_TXT).click()
        if d(text=DIALER_SETUP_TXT).wait.exists(timeout=5000):
            d(text=DIALER_SETUP_TXT).click()
        d(resourceId="Voicemail number").wait.exists(timeout=5000)
        if d(resourceId=DIALER_SUMMARY).wait.exists(timeout=5000):
            d(resourceId=DIALER_SUMMARY).click()
        if d(resourceId=DIALER_VOICEMAIL_EDITBOX).wait.exists(timeout=5000):
            d(resourceId=DIALER_VOICEMAIL_EDITBOX).set_text(number)
        if d(text=DIALER_CONFIRMATION).wait.exists(timeout=5000):
            d(text=DIALER_CONFIRMATION).click()
        if d(text=DIALER_CONFIRMATION).wait.exists(timeout=5000):
            d(text=DIALER_CONFIRMATION).click()

    @staticmethod
    def answer_call(check_call_number=False, phone_number=None):
        StatusBar.open_notifications()
        time.sleep(1)
        StatusBar.answer_phone()
        d.dump()
        if check_call_number and phone_number:
            phone_obtained = StatusBar.get_first_status_element().replace(" ", "")
            assert phone_obtained == phone_number, "Phone number isn't displayed on the remote phone. Expected/obtained: " + \
                                                   phone_number + "/" + phone_obtained
        StatusBar.close_notifications()

    @staticmethod
    def answer_incoming_call(from_number=None):
        """
        Answer incoming call while another call is active
        by swiping right
        :param from_number: The number/contact dialing
        :return: True/False
        """
        if not d(text=DIALER_INCOMING_CALL_TXT).wait.exists(timeout=10000):
            LOG.error('Failed to find %s' % DIALER_INCOMING_CALL_TXT)
            return False

        # Check that incoming call is from 'from_number'
        if from_number:
            if not Dialer.check_incoming_call(from_number):
                return False

        if not d(className=ANDROID_VIEW_CLASSNAME).wait.exists(timeout=10000):
            LOG.error('Failed to find %s' % ANDROID_VIEW_CLASSNAME)
            return False
        d(className=ANDROID_VIEW_CLASSNAME).swipe.right()
        if not d(text=DIALER_INCOMING_CALL_TXT).wait.gone(timeout=5000):
            LOG.error('Failed to answer incoming call.')
            return False
        return True

    @staticmethod
    def dismiss_incoming_call(from_number=None):
        """
        Dismiss incoming call while another call is active
        by swiping left
        :param from_number: The number/contact dialing
        :return: True/False
        """
        if not d(text=DIALER_INCOMING_CALL_TXT).wait.exists(timeout=10000):
            LOG.error('Failed to find %s' % DIALER_INCOMING_CALL_TXT)
            return False

        # Check that incoming call is from 'from_number'
        if from_number:
            if not Dialer.check_incoming_call(from_number):
                return False

        if not d(className=ANDROID_VIEW_CLASSNAME).wait.exists(timeout=10000):
            LOG.error('Failed to find %s' % ANDROID_VIEW_CLASSNAME)
            return False
        d(className=ANDROID_VIEW_CLASSNAME).swipe.left()
        if not d(text=DIALER_INCOMING_CALL_TXT).wait.gone(timeout=5000):
            LOG.error('Failed to dismiss incoming call.')
            return False
        return True

    @staticmethod
    def check_incoming_call(from_number):
        """
        Check that incoming call is from 'from_number'
        This is from the swipe right/left view
        :param from_number: The number/contact dialing
        :return: True/False
        """
        if not d(text=DIALER_INCOMING_CALL_TXT).wait.exists(timeout=10000):
            LOG.error('Failed to find %s' % DIALER_INCOMING_CALL_TXT)
            return False

        # Format number to: xxxx xxx xxx
        nr = from_number.replace(' ', '')
        nr_fmt = '%s %s %s' % (nr[0:4], nr[4:7], nr[7:10])
        if not d(text=nr_fmt).wait.exists(timeout=5000):
            LOG.error('Failed to find %s.' % nr_fmt)
            if d(resourceId=DIALER_CONTACT_NAME).wait.exists():
                found_nr = d(resourceId=DIALER_CONTACT_NAME).info['text']
                LOG.warning('Found incoming call from %s.' % found_nr)
            return False
        return True

    @staticmethod
    def end_call(timeout=10000):
        if d(resourceId=DIALER_END_CALL).wait.exists(timeout=timeout):
            d(resourceId=DIALER_END_CALL).click()
            return True
        return False

    @staticmethod
    def hold_call():
        if not d(description=DIALER_HOLD_CALL_DESC).wait.exists(timeout=5000):
            LOG.info('Failed to find %s' % DIALER_HOLD_CALL_DESC)
            return False
        d(description=DIALER_HOLD_CALL_DESC).click()
        if not d(description=DIALER_HOLD_CALL_DESC).wait.gone(timeout=5000):
            LOG.info('%s did not disappeared.' % DIALER_HOLD_CALL_DESC)
            return False
        return True

    @staticmethod
    def resume_call():
        if not d(description=DIALER_RESUME_CALL_DESC).wait.exists(timeout=5000):
            LOG.info('Failed to find %s' % DIALER_RESUME_CALL_DESC)
            return False
        d(description=DIALER_RESUME_CALL_DESC).click()
        if not d(description=DIALER_RESUME_CALL_DESC).wait.gone(timeout=5000):
            LOG.info('%s did not disappeared.' % DIALER_RESUME_CALL_DESC)
            return False
        return True

    @staticmethod
    def is_call_on_hold(phone_number=None):
        """
        Check that call is on hold
        :param phone_number: The phone number being on hold
        :return: True/False
        """
        if not d(text=DIALER_ON_HOLD_TXT).wait.exists(timeout=5000):
            LOG.error('Failed to find %s' % DIALER_ON_HOLD_TXT)
            return False
        if phone_number:
            # Format number to: xxxx xxx xxx
            nr = phone_number.replace(' ', '')
            nr_fmt = '%s %s %s' % (nr[0:4], nr[4:7], nr[7:10])
            # Case: 2 remotes calls
            if d(text=nr_fmt, resourceId=DIALER_SECONDARY_RESID).wait.exists():
                LOG.info('Two remote calls. %s on hold.' % nr_fmt)
                return True
            # Case: 1 remote call
            elif d(text=nr_fmt).wait.exists():
                LOG.info('One remote call. %s on hold.' % nr_fmt)
                return True
            else:
                LOG.error('Failed to find hold call %s' % nr_fmt)
                return False
        return True

    @staticmethod
    def voice_call(phone_originating, phone_receiving, check_number=False, check_message=True,
                   private_number=False, screen_locked=False):

        dut_manager.activate_phone(phone_originating)
        Dialer.launch()
        time.sleep(2)
        Dialer.dial_number_using_keyboard(dut_manager.acs_config[phone_receiving][u'phoneNumber'])
        time.sleep(WAIT_AFTER_DIAL)
        dut_manager.activate_phone(phone_receiving)

        if screen_locked:
            Dialer.voice_call_with_screen_locked()
        elif check_message:
            d.press.home()
            Dialer.voice_call_verify_incoming_call()
        if check_number:
            if private_number:
                Dialer.voice_call_check_private_number()
            else:
                Dialer.voice_call_check_number(phone_originating)

    @staticmethod
    def voice_call_check_number(phone_originating):
        d(resourceId=NOTIFICATIONS_NOTIFICATION_TITLE_RESID).wait.exists(timeout=5000)
        assert (d(resourceId=NOTIFICATIONS_NOTIFICATION_TITLE_RESID)[0].text.replace(" ", "") ==
                dut_manager.acs_config[phone_originating][
                    u'phoneNumber']), "Did not detect the incoming phone number in the notification bar: " + \
                                      dut_manager.acs_config[phone_originating][u'phoneNumber']

    @staticmethod
    def voice_call_check_private_number():
        d(resourceId=NOTIFICATIONS_NOTIFICATION_TITLE_RESID).wait.exists(timeout=5000)
        assert (d(resourceId=NOTIFICATIONS_NOTIFICATION_TITLE_RESID)[0].text == "Private number"), \
            "Did not detect a call from a private number"

    @staticmethod
    def voice_call_verify_incoming_call():
        StatusBar.open_notifications()
        time.sleep(2)
        d(resourceId=NOTIFICATIONS_NOTIFICATION_TEXT_RESID).wait.exists(timeout=5000)
        assert (d(resourceId=NOTIFICATIONS_NOTIFICATION_TEXT_RESID)[0].text == "Incoming call"), \
            "Did not detect the incoming call text in the notification bar"

    @staticmethod
    def voice_call_with_screen_locked():
        assert d(resourceId=DIALER_GLOW_PAD_VIEW_RESID).wait.exists(timeout=15000), \
            "Did not detect glow button from calling screen"
        Dialer.voice_call_verify_incoming_call()

    @staticmethod
    def is_import_from_sim_open():
        return d(packageName=IMPORT_FROM_SIM_PACKAGE_NAME, text=IMPORT_FROM_SIM_TITLE_TXT).wait.exists(timeout=5000)

    @staticmethod
    def import_contact_from_sim(contact_name):
        try:
            contact = d(className=ANDROID_WIDGET_LIST_VIEW, resourceId=ANDROID_WIDGET_LIST_RESID) \
                .child_by_text(contact_name, resourceId=IMPORT_FROM_SIM_CONTACT_RESID)
        except:
            LOG.info('Contact "%s" not found.' % contact_name)
            return None
        UiAutomatorUtils.long_click(contact)
        if not d(text=contact_name, resourceId=ANDROID_WIDGET_ALERT_TITLE).wait.exists(timeout=5000):
            LOG.info("'Import contact' popup did not show up.")
            return None
        if not d(text=IMPORT_FROM_SIM_IMPORT_TXT, resourceId=ANDROID_WIDGET_TITLE_RESID).wait.exists(timeout=5000):
            LOG.info("'Import' button not found.")
            return None
        d(text=IMPORT_FROM_SIM_IMPORT_TXT, resourceId=ANDROID_WIDGET_TITLE_RESID).click()
        time.sleep(2)
        if d(text=contact_name, resourceId=ANDROID_WIDGET_ALERT_TITLE).wait.exists(timeout=2000):
            LOG.info("'Import contact' popup still open.")
            d.press.back()
            return None
        return True

    @staticmethod
    def import_all_contacts_from_sim():
        if not d(description=CONTACTS_MORE_OPTIONS_TXT,
                 className=ANDROID_WIDGET_IMAGE_BUTTON).wait.exists(timeout=5000):
            LOG.info("Failed to find 'More Options' menu.")
            return None
        d(description=CONTACTS_MORE_OPTIONS_TXT, className=ANDROID_WIDGET_IMAGE_BUTTON).click()
        if not d(text=IMPORT_FORM_SIM_ALL_TXT, className=ANDROID_WIDGET_TEXT_VIEW).wait.exists(timeout=5000):
            LOG.info("Failed to find 'Import all' option.")
            return None
        d(text=IMPORT_FORM_SIM_ALL_TXT, className=ANDROID_WIDGET_TEXT_VIEW).click()
        # Depending on how much data is to be imported the time may vary
        time.sleep(3)
        return True

    @staticmethod
    def get_all_contacts_from_sim():
        return Contacts.get_all_contacts_from(IMPORT_FROM_SIM_CONTACT_RESID)

    @staticmethod
    def is_ussd_open():
        send_btn = d(text=USSD_SEND_TXT, className=ANDROID_WIDGET_BUTTON).wait.exists(timeout=10000)
        cancel_btn = d(text=USSD_CANCEL_TXT, className=ANDROID_WIDGET_BUTTON).wait.exists(timeout=10000)
        if send_btn and cancel_btn:
            LOG.info("Found ussd buttons: %s, %s" % (USSD_SEND_TXT, USSD_CANCEL_TXT))
            return True
        ok_btn = d(text=USSD_OK_TXT, className=ANDROID_WIDGET_BUTTON).wait.exists(timeout=10000)
        if ok_btn:
            LOG.info("Found ussd button: %s" % USSD_OK_TXT)
            return True
        LOG.info("USSD popup not open.")
        return False

    @staticmethod
    def get_ussd_message():
        if not d(resourceId=DIALER_MMI_POPUP).wait.exists(timeout=2000):
            LOG.info("Failed to find USSD message.")
            return None
        msg = d(resourceId=DIALER_MMI_POPUP).info['text']
        # NOTE: Do I want to attempt to make dictionary out of msg?
        msg_dict = dict(re.findall('\n(\d+)\s+(.+)', msg))
        if msg_dict:
            return msg_dict
        return msg

    @staticmethod
    def send_ussd(option):
        if not d(resourceId=USSD_INPUT_RESID).wait.exists(timeout=2000):
            LOG.info("Failed to find the input field.")
            return False
        d(resourceId=USSD_INPUT_RESID).set_text(option)
        if not d(text=USSD_SEND_TXT, className=ANDROID_WIDGET_BUTTON).wait.exists(timeout=2000):
            LOG.info("Failed to find '%s' button." % USSD_SEND_TXT)
            return False
        if not d(text=USSD_SEND_TXT, className=ANDROID_WIDGET_BUTTON).click.wait(timeout=5000):
            LOG.info("Failed to send option: %s" % option)
            return False
        return True

    @staticmethod
    def cancel_ussd():
        if not d(text=USSD_CANCEL_TXT, className=ANDROID_WIDGET_BUTTON).wait.exists(timeout=2000):
            LOG.info("Failed to find '%s' button." % USSD_CANCEL_TXT)
            return False
        d(text=USSD_CANCEL_TXT, className=ANDROID_WIDGET_BUTTON).click()
        # Check that USSD popup disappeared
        if Dialer.is_ussd_open():
            LOG.info('USSD popup is still open.')
            return False
        return True

    @staticmethod
    def ok_ussd():
        if not d(text=USSD_OK_TXT, className=ANDROID_WIDGET_BUTTON).wait.exists(timeout=2000):
            LOG.info("Failed to find '%s' button." % USSD_OK_TXT)
            return False
        d(text=USSD_OK_TXT, className=ANDROID_WIDGET_BUTTON).click()
        # Check that USSD popup disappeared
        if Dialer.is_ussd_open():
            LOG.info('USSD popup is still open.')
            return False
        return True

    @staticmethod
    def close_ussd():
        return Dialer.cancel_ussd() or Dialer.ok_ussd()

    @staticmethod
    def mmi_call(mmi_code, mmi_expected_response):
        Dialer.dial_number_using_keyboard(mmi_code)
        d(resourceId=DIALER_MMI_POPUP).wait.exists(timeout=5000)
        if mmi_expected_response in d(resourceId=DIALER_MMI_POPUP).text:
            d(text=DIALER_CONFIRMATION).click()
            return True
        d(text=DIALER_CONFIRMATION).click()
        return False

    @staticmethod
    def navigate_to_fdn():
        Dialer.launch()
        Dialer.navigate_to_call_settings()
        if not d(text=DIALER_FDN_SETTINGS_TXT).wait.exists(timeout=2000):
            LOG.info("Failed to find 'Fixed Dialing Numbers' option")
            return False
        d(text=DIALER_FDN_SETTINGS_TXT).click()
        return True

    @staticmethod
    def check_fdn_status():
        if d(resourceId=ANDROID_WIDGET_TITLE_RESID,
             text=DIALER_DISABLE_FDN_TXT).wait.exists(timeout=2000):
            return True
        elif d(resourceId=ANDROID_WIDGET_TITLE_RESID,
               text=DIALER_ENABLE_FDN_TXT).wait.exists(timeout=2000):
            return False
        else:
            LOG.info("Could not determine the FDN status")
            return None

    @staticmethod
    def activate_fdn(pin_2):
        Dialer.navigate_to_fdn()

        if Dialer.check_fdn_status():
            LOG.info("FDN already enabled")
            return True

        if Dialer.check_fdn_status() is None:
            return False

        d(resourceId=ANDROID_WIDGET_TITLE_RESID,
          text=DIALER_ENABLE_FDN_TXT).click()
        if not d(className=ANDROID_WIDGET_EDIT_TEXT).wait.exists(timeout=2000):
            LOG.info("Failed to find edit text field")
            return False
        d(className=ANDROID_WIDGET_EDIT_TEXT).set_text(pin_2)
        UiAutomatorUtils.click_view_with_text(POPUP_OK)

        return Dialer.check_fdn_status()

    @staticmethod
    def deactivate_fdn(pin_2):
        Dialer.navigate_to_fdn()

        if not Dialer.check_fdn_status():
            LOG.info("FDN already disabled")
            return True

        if Dialer.check_fdn_status() is None:
            return False

        d(resourceId=ANDROID_WIDGET_TITLE_RESID,
          text=DIALER_DISABLE_FDN_TXT).click()
        if not d(className=ANDROID_WIDGET_EDIT_TEXT).wait.exists(timeout=2000):
            LOG.info("Failed to find edit text field")
            return False
        d(className=ANDROID_WIDGET_EDIT_TEXT).set_text(pin_2)
        UiAutomatorUtils.click_view_with_text(POPUP_OK)

        return not Dialer.check_fdn_status()

    @staticmethod
    def block_pin_2_change_using_puk_2(pin_2, puk_2):
        Dialer.navigate_to_fdn()
        Dialer.block_fdn()
        return Dialer.change_pin_2_using_puk_2(pin_2, puk_2)

    @staticmethod
    def block_fdn():
        for i in range(3):
            if not Dialer.check_fdn_status():
                LOG.info("Failed to find 'Disable FDN' option")
                return False
            d(resourceId=ANDROID_WIDGET_TITLE_RESID,
              text=DIALER_DISABLE_FDN_TXT).click()
            if not d(className=ANDROID_WIDGET_EDIT_TEXT).wait.exists(
                    timeout=2000):
                LOG.info("Failed to find edit text field")
                return False
            d(className=ANDROID_WIDGET_EDIT_TEXT).set_text(WRONG_PIN_2)
            UiAutomatorUtils.click_view_with_text(POPUP_OK)
        if not Dialer.verify_enter_puk_2_appears():
            LOG.info("PUK2 was not requested")
            return False
        return True

    @staticmethod
    def change_pin_2_using_puk_2(pin_2, puk_2):
        d(className=ANDROID_WIDGET_EDIT_TEXT).set_text(puk_2)
        UiAutomatorUtils.click_view_with_text(POPUP_OK)

        for i in range(2):
            time.sleep(1)
            d(className=ANDROID_WIDGET_EDIT_TEXT).set_text(pin_2)
            UiAutomatorUtils.click_view_with_text(POPUP_OK)

        # after changing PIN2 through PUK2 the Change PIN2 box should request
        # entering the Old PIN2 --> unlock through PUK2 was successful
        return Dialer.verify_fdn_unblocked()

    @staticmethod
    def change_pin_2_using_puk_2_mmi(pin_2, puk_2):
        Dialer.launch()
        mmi_unblock_code = FDN_MMI_PREFIX + str(puk_2) + FDN_MMI_STAR + str(
                pin_2) + FDN_MMI_STAR + str(pin_2) + FDN_MMI_SUFFIX
        Dialer.dial_number_using_keyboard(mmi_unblock_code)
        message_after_mmi = UiAutomatorUtils.get_attr_content_matching_text(
                DIALER_PIN_CHANGE_TXT)
        if DIALER_PIN_CHANGE_TXT not in message_after_mmi:
            LOG.info("Success message box didn't appear")
            return False
        UiAutomatorUtils.click_view_with_text(POPUP_OK)
        Dialer.navigate_to_fdn()
        return Dialer.verify_fdn_unblocked()

    @staticmethod
    def verify_fdn_unblocked():
        if not d(resourceId=ANDROID_WIDGET_TITLE_RESID,
                 text=DIALER_CHANGE_PIN_2_TXT).wait.exists(timeout=2000):
            LOG.info("Failed to find 'Change PIN2' option")
            return False
        d(resourceId=ANDROID_WIDGET_TITLE_RESID,
          text=DIALER_CHANGE_PIN_2_TXT).click()
        if not d(resourceId=ANDROID_MESSAGE_RESID,
                 text=DIALER_OLD_PIN_2_TXT).wait.exists(timeout=2000):
            LOG.info("Failed to find 'Old PIN2' message")
            return False
        d.press.back()
        d.press.back()
        return True

    @staticmethod
    def verify_enter_puk_2_appears():
        if not d(resourceId=ANDROID_WIDGET_TITLE_RESID,
                 text=DIALER_CHANGE_PIN_2_TXT).wait.exists(timeout=2000):
            LOG.info("Failed to find 'Change PIN2' option")
            return False
        d(resourceId=ANDROID_WIDGET_TITLE_RESID,
          text=DIALER_CHANGE_PIN_2_TXT).click()
        if not d(resourceId=ANDROID_MESSAGE_RESID,
                 text=DIALER_ENTER_PUK_2_TXT).wait.exists(timeout=2000):
            LOG.info("Failed to find 'Enter PUK2 code' message")
            return False
        return True

