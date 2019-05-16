# -*- coding: UTF-8 -*-
from PyUiApi.common.system_utils import *
from PyUiApi.app_utils.messenger_utils import *
from PyUiApi.app_utils.settings_utils import *
from PyUiApi.app_utils.dialer_utils import *
from PyUiApi.multi_dut_support.dut_manager import *

MESSAGE_DELIVERY_TIME = 20


class SMSTests(unittest.TestCase):
    TAG = "SMSTests"

    def setUp(self):
        UiAutomatorUtils.unlock_screen()
        UiAutomatorUtils.close_all_tasks()
        self.initial_orientation = d.orientation

        try:
            self.phone1_number = dut_manager.acs_config[u'PHONE1'][u'phoneNumber']
            self.phone2_number = dut_manager.acs_config[u'PHONE2'][u'phoneNumber']
            self.phone3_number = dut_manager.acs_config[u'PHONE3'][u'phoneNumber']
        except:
            LOG.info("Phone numbers for devices are not defined in bench config")

    def tearDown(self):
        if d.orientation != self.initial_orientation:
            d.orientation = self.initial_orientation
            d.freeze_rotation(False)
        dut_manager.activate_phone("PHONE1")
        Settings.launch()
        Settings.change_mobile_network("2G")
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE2")
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE3")
        UiAutomatorUtils.close_all_tasks()

    def test_send_sms(self):
        """
        ST_TELEPHONY_MSG_SMS_???
        """
        # DUT1
        Messaging.launch()
        sms_txt = Messaging.generate_unique_sms()
        Messaging.send_sms(self.phone2_number, sms_txt)

        # DUT2
        dut_manager.activate_phone("PHONE2")
        Messaging.launch()

        self.assertTrue(Messaging.check_sms_received(sms_txt),
                        "Did not detect the following sms in messenger app: " + sms_txt)

    def test_check_sms_reply(self):
        """
        ST_TELEPHONY_MSG_SMS_007
        """

        send_msg = "send"
        reply_msg = "reply_msg"

        # DUT1
        Messaging.launch()
        Messaging.send_sms(self.phone2_number, send_msg)
        UiAutomatorUtils.close_all_tasks()

        # DUT2
        dut_manager.activate_phone("PHONE2")
        time.sleep(MESSAGE_DELIVERY_TIME)
        Messaging.reply_sms(self.phone1_number, reply_msg)

        # DUT1
        dut_manager.activate_phone("PHONE1")
        time.sleep(MESSAGE_DELIVERY_TIME)
        SMSFoundInStatusBar = Messaging.check_sms_received_in_statusbar(reply_msg)
        d.press.home()
        Messaging.launch()
        SMSFoundInApp = Messaging.check_sms_received(reply_msg)
        self.assertTrue(SMSFoundInStatusBar and SMSFoundInApp,
                        "Did not detect reply message. SMS present in statusbar and in mess app: " + str(
                            SMSFoundInStatusBar) + " " + str(SMSFoundInApp))

    def test_7bit_gms_alphabet_sms(self):
        """
        ST_TELEPHONY_MSG_SMS_011
        """
        sms_txt = Messaging.generate_unique_sms() + '___' + \
                  '@ΔSP0¡P¿£_!1AQa$Φ\"2BRb¥Γ#3CScèΛ¤4DTdéΩ%5EUeùΠ&6FVfìΨ\'7GWgòΣ(8HXhÇΘ)9IYi\Ξ*:JZjØ€+KÄøÆ""<LÖläæ-=MÑmÅß0>NÜnåÉ/?O§o^{[~]}'
        Messaging.launch()
        Messaging.send_sms(self.phone2_number, sms_txt)

        # DUT2
        d.press.home()
        dut_manager.activate_phone("PHONE2")
        Messaging.launch()

        self.assertTrue(Messaging.check_sms_received(sms_txt),
                        "Did not detect the following sms in messenger app: " + sms_txt)

    def test_320_char_sms(self):
        """
        ST_TELEPHONY_MSG_SMS_104
        """
        sms_txt = "abcdefghij" * 32
        Messaging.launch()
        Messaging.send_sms(self.phone2_number, sms_txt)

        # DUT2
        d.press.home()
        dut_manager.activate_phone("PHONE2")
        Messaging.launch()

        self.assertTrue(Messaging.check_sms_received(sms_txt),
                        "Did not detect the following sms in messenger app: " + sms_txt)

    def test_UCS2_alphabet(self):
        """
        ST_TELEPHONY_MSG_SMS_111
        """
        # 70 char limit for UCS2 sms
        sms_txt = "íáڅकਊઊଌஔಊഈ" * 7
        Messaging.launch()
        Messaging.send_sms(self.phone2_number, sms_txt)

        # DUT2
        d.press.home()
        dut_manager.activate_phone("PHONE2")
        Messaging.launch()

        self.assertTrue(Messaging.check_sms_received(sms_txt),
                        "Did not detect the following sms in messenger app: " + sms_txt)

    def test_check_sms_characters_left(self):
        """
        ST_TELEPHONY_MSG_SMS_013
        """
        # 155 char limit for standard sms
        sms_txt = "0123456789" * 15 + "01234"
        Messaging.launch()
        if d(resourceId=MESSAGING_START_CONVERSATION).wait.exists(timeout=7000):
            d(resourceId=MESSAGING_START_CONVERSATION).click()
        if d(text=MESSAGING_ALL_CONTACTS_TXT).wait.exists(timeout=3000):
            d(text=MESSAGING_ALL_CONTACTS_TXT).click()
        if d(resourceId=MESSAGING_RECEIPIENT).wait.exists(timeout=5000):
            d(resourceId=MESSAGING_RECEIPIENT).set_text("000111222")
        d.press("enter")
        d.wait.idle()
        if d(resourceId=MESSAGING_MESSAGE_BOX).wait.exists(timeout=5000):
            d(resourceId=MESSAGING_MESSAGE_BOX).set_text(sms_txt)
        if d(resourceId=MESSAGING_CHAR_COUNTER).wait.exists(timeout=5000):
            char_counter = d(resourceId=MESSAGING_CHAR_COUNTER).info["text"]
            self.assertTrue(int(char_counter) == 5,
                            "Char counter should be 5 but is: " + char_counter)

    def test_send_sms_to_contact(self):
        """
        ST_TELEPHONY_MSG_SMS_010
        """
        sms_txt = "sms_from_contact"
        sms_contact_name = "SMS_CONTACT"
        Messaging.launch()
        Messaging.remove_contact(sms_contact_name)
        UiAutomatorUtils.close_all_tasks()
        Messaging.launch()
        Messaging.add_contact(sms_contact_name, str(self.phone2_number))
        UiAutomatorUtils.close_all_tasks()
        Messaging.launch()
        Messaging.send_sms_to_contact(sms_contact_name, sms_txt)
        dut_manager.activate_phone("PHONE2")
        Messaging.launch()
        self.assertTrue(Messaging.check_sms_received(sms_txt),
                        "Did not detect the following sms in messenger app: " + sms_txt)

    def test_receive_sms_from_contact(self):
        """
        ST_TELEPHONY_MSG_SMS_110
        """

        sms_contact_name = "SMS_CONTACT"
        sms_txt = "sms_received_from_contact"
        Messaging.launch()
        Messaging.add_contact(sms_contact_name, str(self.phone2_number))
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE2")
        Messaging.launch()
        Messaging.send_sms(self.phone1_number, sms_txt)
        dut_manager.activate_phone("PHONE1")
        Messaging.launch()
        self.assertTrue(Messaging.check_sms_received(sms_txt),
                        "Did not detect the following sms in messenger app: " + sms_txt)
        self.assertTrue(d(text=sms_contact_name).wait.exists(timeout=5000),
                        "Did not detect the following contact name in received sms: " + sms_contact_name)

    def test_UCS2_sms_size(self):
        """
        ST_TELEPHONY_MSG_SMS_012
        """
        # case where sms is encoded as UCS2
        sms_txt = "comment ça va,monsieur 张?" * 8
        UCS2_encoding_txt = "Simple characters only"
        classname_switch = "android.widget.Switch"
        outcome_case1, outcome_case2 = False, False
        char_counter1, char_counter2 = 0, 0

        Messaging.launch()
        Messaging.switch_ucs2_encoding('false')
        UiAutomatorUtils.close_all_tasks()

        Messaging.launch()
        Messaging.fill_sms_info(self.phone1_number, sms_txt)
        if d(resourceId=MESSAGING_CHAR_COUNTER).wait.exists(timeout=5000):
            char_counter1 = d(resourceId=MESSAGING_CHAR_COUNTER).info["text"]
            outcome_case1 = char_counter1 == "1 / 3"

        # case where sms is not encoded as UCS2
        UiAutomatorUtils.close_all_tasks()
        Messaging.launch()
        Messaging.switch_ucs2_encoding('true')

        UiAutomatorUtils.close_all_tasks()
        Messaging.launch()
        Messaging.open_settings()
        if d(text=MESSAGING_ADVANCED_TXT).wait.exists(timeout=5000):
            d(text=MESSAGING_ADVANCED_TXT).click()
        d(text="Advanced settings").wait.exists(timeout=5000)
        mapping = UiAutomatorExtended.get_mapping_using_class(classname_switch)
        if [key[1] for key in mapping if key[0] == UCS2_encoding_txt] == "true":
            UiAutomatorExtended.change_status_for_text(UCS2_encoding_txt, classname_switch)
        Messaging.launch()
        Messaging.fill_sms_info(self.phone1_number, sms_txt)
        if d(resourceId=MESSAGING_CHAR_COUNTER).wait.exists(timeout=5000):
            char_counter2 = d(resourceId=MESSAGING_CHAR_COUNTER).info["text"]
            outcome_case2 = char_counter2 == "106 / 2"

        Messaging.launch()
        Messaging.switch_ucs2_encoding('false')
        self.assertTrue(outcome_case1 is True, "Char counter should be 1 / 3 but is: " + char_counter1)
        self.assertTrue(outcome_case2 is True, "Char counter should be 106 / 2 but is: " + char_counter2)

    def test_multiple_page_sms(self):
        """
        ST_TELEPHONY_MSG_SMS_002
        """
        sms_txt = "0123456789" * 32 + "_this_is_over_320_characters"

        dut_manager.activate_phone("PHONE2")
        Messaging.launch()
        Messaging.fill_sms_info(self.phone1_number, sms_txt)
        self.assertTrue(d(resourceId=MESSAGING_CHAR_COUNTER).wait.exists(timeout=5000), "Char counter not found")
        self.assertTrue(d(resourceId=MESSAGING_CHAR_COUNTER).info["text"] == "111 / 3",
                        "SMS should appear across multiple pages. Expected - Obtained: " + "111 / 3  -  " +
                        d(resourceId=MESSAGING_CHAR_COUNTER).info["text"])
        if d(resourceId=MESSAGING_SEND).wait.exists(timeout=3000):
            d(resourceId=MESSAGING_SEND).click()

        time.sleep(MESSAGE_DELIVERY_TIME)
        dut_manager.activate_phone("PHONE1")
        Messaging.launch()
        self.assertTrue(Messaging.check_sms_received(sms_txt),
                        "Did not detect the following sms in messenger app: " + sms_txt)

    def test_sms_over_3G(self):
        """
        ST_TELEPHONY_MSG_SMS_003
        """
        sms_txt = "0123456789" * 16

        Messaging.launch()
        Messaging.send_sms(self.phone2_number, sms_txt)
        UiAutomatorUtils.close_all_tasks()
        Messaging.launch()
        self.assertTrue(Messaging.check_last_sms_sent(sms_txt), "Message not sent")
        time.sleep(MESSAGE_DELIVERY_TIME)
        dut_manager.activate_phone("PHONE2")
        Messaging.launch()
        self.assertTrue(Messaging.check_sms_received(sms_txt),
                        "Did not detect the following sms in messenger app: " + sms_txt)

    def test_forward_sms(self):
        """
        ST_TELEPHONY_MSG_SMS_008
        """
        sms_txt = "message_to_be_forwarded"
        forwarded_txt = "_text_added_to_forwarded_msg"

        dut_manager.activate_phone("PHONE2")
        Messaging.launch()
        Messaging.send_sms(self.phone1_number, sms_txt)
        time.sleep(MESSAGE_DELIVERY_TIME)

        dut_manager.activate_phone("PHONE1")
        UiAutomatorUtils.close_all_tasks()
        Messaging.launch()
        self.assertTrue(Messaging.forward_last_sms(self.phone2_number, forwarded_txt),
                        "Message failed in the process of forwarding the message")

        dut_manager.activate_phone("PHONE2")
        UiAutomatorUtils.close_all_tasks()
        time.sleep(MESSAGE_DELIVERY_TIME)
        Messaging.launch()
        self.assertTrue(Messaging.check_last_sms_sent(sms_txt + forwarded_txt),
                        "Message was not forwarded successfully")

    def test_national_characters(self):
        """
        ST_TELEPHONY_MSG_SMS_015
        """
        french_txt = "Ça, c’est un message écrit en français ! Cédilles et accents doivent être là."
        spanish_txt = "¡Este es un mensaje escrito en español! ¿No es ?"
        finish_txt = "Tämä viesti on kirjoitettu suomeksi! Eikö olekin?"
        german_txt = "Hier ist eine Nachricht in deutscher Sprache geschrieben. Suchen Sie nach dem Umlaut und Eszett im Wort Füßen."
        turkish_txt = "Burada Türkçe yazılmış bir mesajdır. Değil mi?"
        arabic_txt = "ك ذل ك؟ أل يس .ال عرب ية ب ال ل غة مك توب ة ر سال ة هو هنا"
        chinese_txt = "这里是用中文写的消息。是不是 ?"

        Messaging.launch()
        Messaging.send_sms(self.phone2_number, french_txt)
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE2")
        time.sleep(SHORT_MESSAGE_DELIVERY_TIME)
        Messaging.launch()
        self.assertTrue(Messaging.check_sms_received(french_txt),
                        "Did not detect the following sms in messenger app: " + french_txt)
        UiAutomatorUtils.close_all_tasks()

        dut_manager.activate_phone("PHONE1")
        Messaging.launch()
        Messaging.send_sms(self.phone2_number, spanish_txt)
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE2")
        time.sleep(SHORT_MESSAGE_DELIVERY_TIME)
        Messaging.launch()
        self.assertTrue(Messaging.check_sms_received(spanish_txt),
                        "Did not detect the following sms in messenger app: " + spanish_txt)
        UiAutomatorUtils.close_all_tasks()

        dut_manager.activate_phone("PHONE1")
        Messaging.launch()
        Messaging.send_sms(self.phone2_number, finish_txt)
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE2")
        time.sleep(SHORT_MESSAGE_DELIVERY_TIME)
        Messaging.launch()
        self.assertTrue(Messaging.check_sms_received(finish_txt),
                        "Did not detect the following sms in messenger app: " + finish_txt)
        UiAutomatorUtils.close_all_tasks()

        dut_manager.activate_phone("PHONE1")
        Messaging.launch()
        Messaging.send_sms(self.phone2_number, german_txt)
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE2")
        time.sleep(SHORT_MESSAGE_DELIVERY_TIME)
        Messaging.launch()
        self.assertTrue(Messaging.check_sms_received(german_txt),
                        "Did not detect the following sms in messenger app: " + german_txt)
        UiAutomatorUtils.close_all_tasks()

        dut_manager.activate_phone("PHONE1")
        Messaging.launch()
        Messaging.send_sms(self.phone2_number, turkish_txt)
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE2")
        time.sleep(SHORT_MESSAGE_DELIVERY_TIME)
        Messaging.launch()
        self.assertTrue(Messaging.check_sms_received(turkish_txt),
                        "Did not detect the following sms in messenger app: " + turkish_txt)
        UiAutomatorUtils.close_all_tasks()

        dut_manager.activate_phone("PHONE1")
        Messaging.launch()
        Messaging.send_sms(self.phone2_number, arabic_txt)
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE2")
        time.sleep(SHORT_MESSAGE_DELIVERY_TIME)
        Messaging.launch()
        self.assertTrue(Messaging.check_sms_received(arabic_txt),
                        "Did not detect the following sms in messenger app: " + arabic_txt)
        UiAutomatorUtils.close_all_tasks()

        dut_manager.activate_phone("PHONE1")
        Messaging.launch()
        Messaging.send_sms(self.phone2_number, chinese_txt)
        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE2")
        time.sleep(SHORT_MESSAGE_DELIVERY_TIME)
        Messaging.launch()
        self.assertTrue(Messaging.check_sms_received(chinese_txt),
                        "Did not detect the following sms in messenger app: " + chinese_txt)

    def test_time_to_send(self):
        """
        ST_TELEPHONY_MSG_SMS_018
        """
        long_sms_txt = "timed_sms"
        error_deviation = 5

        dut_manager.activate_phone("PHONE2")
        Messaging.launch()
        Messaging.send_sms(self.phone1_number, long_sms_txt, timed=True)

        dut_manager.activate_phone("PHONE1")
        Messaging.launch()
        self.assertTrue(Messaging.check_sms_received(long_sms_txt),
                        "Did not detect the following sms in messenger app: " + long_sms_txt)
        self.assertTrue(Messaging.end_time != 0 and Messaging.start_time != 0, "Timer recording failed")
        reference_time = Messaging.end_time - Messaging.start_time

        UiAutomatorUtils.close_all_tasks()
        dut_manager.activate_phone("PHONE2")
        UiAutomatorUtils.close_all_tasks()
        Messaging.reset_timer()

        dut_manager.activate_phone("PHONE1")
        Messaging.launch()
        Messaging.send_sms(self.phone2_number, long_sms_txt, timed=True)

        dut_manager.activate_phone("PHONE2")
        Messaging.launch()
        self.assertTrue(Messaging.check_sms_received(long_sms_txt),
                        "Did not detect the following sms in messenger app: " + long_sms_txt)
        self.assertTrue(Messaging.end_time != 0 and Messaging.start_time != 0, "Timer recording failed")
        dut_time = Messaging.end_time - Messaging.start_time
        self.assertTrue((error_deviation - abs(dut_time - reference_time)) > 0,
                        "Dut time was slower than reference device")

    def test_multiple_recipients(self):
        """
        ST_TELEPHONY_MSG_SMS_006
        """
        txt_message = "multiple receivers sms"

        Messaging.launch()
        Messaging.fill_sms_info([self.phone2_number, self.phone3_number], txt_message)
        if d(resourceId=MESSAGING_SEND).wait.exists(timeout=3000):
            d(resourceId=MESSAGING_SEND).click()

        time.sleep(MESSAGE_DELIVERY_TIME)
        dut_manager.activate_phone("PHONE2")
        self.assertTrue(Messaging.check_sms_received(txt_message),
                        "Did not detect the following sms in messenger app: " + txt_message)

        time.sleep(MESSAGE_DELIVERY_TIME)
        dut_manager.activate_phone("PHONE3")
        self.assertTrue(Messaging.check_sms_received(txt_message),
                        "Did not detect the following sms in messenger app: " + txt_message)
