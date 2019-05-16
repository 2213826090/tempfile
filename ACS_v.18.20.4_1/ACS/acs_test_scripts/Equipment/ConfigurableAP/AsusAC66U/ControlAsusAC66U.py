"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: implementation of ASUS AC66U family configurable AP
:since:04/11/2014
:author: sdebuiss, jfranchx
"""

from selenium import webdriver
from selenium.webdriver.support.ui import Select
import time


class ControlAsusRtAC66U:
    """
    Implementation of AsusRtAC66U control.
    This code requires python module Selenium to control the web page. Be careful to install it before using this code.

    # Developed with ASUS RT AC66U - Firmware version 3.0.0.4.376_2524
    # WARNING : some features are currently not supported completely
    # Need to be checked or implemented for Wireless option menu :
    # - Legacy Wireless Mode (some options are only available in this mode)
    # - Wireless MAC Filter options
    # - WPS options
    # - WDS options
    """

    # Module Constants
    ASUS_RT_AC66U_AUTHENTICATION_METHOD = ['Open System', 'WPA2-Personal', 'WPA2-Enterprise', 'WPA-Auto-Personal',
                                           'WPA-Auto-Enterprise']
    ASUS_RT_AC66U_AUTHENTICATION_LEGACY_METHOD = ['Open System', "Shared Key", 'WPA-Personal', 'WPA2-Personal',
                                                  'WPA-Enterprise', 'WPA2-Enterprise', 'WPA-Auto-Personal',
                                                  'WPA-Auto-Enterprise', "Radius with 802.1x"]
    ASUS_RT_AC66U_RADIOS = ["2.4GHz", "5GHz"]
    ASUS_RT_AC66U_BANDWIDTHS_2_4GHZ = ["20 MHz", "40 MHz", "20/40 MHz"]
    ASUS_RT_AC66U_BANDWIDTHS_5GHZ = ["20 MHz", "40 MHz", "80 MHz", "20/40/80 MHz"]
    ASUS_RT_AC66U_ENCRYPTION_MODE = ["AES", "TKIP", "TKIP+AES", "WEP-64bits", "WEP-128bits"]
    ASUS_RT_AC66U_SSID_HIDDEN_VALUES = ["ON", "OFF", "1", "0"]
    ASUS_IMPLICITLY_WAIT = 30
    ASUS_WIRELESS_PAGE_GENERAL = 1
    ASUS_WIRELESS_PAGE_WPS = 2
    ASUS_WIRELESS_PAGE_WDS = 3
    ASUS_WIRELESS_PAGE_MAC_FILTER = 4
    ASUS_WIRELESS_PAGE_RADIUS_SETTINGS = 5
    ASUS_WIRELESS_PAGE_PROFESSIONAL = 6
    ASUS_WIRELESS_PAGES = [ASUS_WIRELESS_PAGE_GENERAL, ASUS_WIRELESS_PAGE_WPS, ASUS_WIRELESS_PAGE_WDS,
                           ASUS_WIRELESS_PAGE_MAC_FILTER, ASUS_WIRELESS_PAGE_RADIUS_SETTINGS,
                           ASUS_WIRELESS_PAGE_PROFESSIONAL]

    EXCEPTION_ERROR_CODE_INVALID_PARAMETER = "0"
    EXCEPTION_ERROR_CODE_OPERATION_FAILED = "1"

    def __init__(self):
        """
         Constructor
        """
        self._driver = None

        # Set default functions
        self._asus_print = self.__asus_print_default
        self._raise_exception = self.__raise_exception_default

        # Set default values
        self._exception_error_code_invalid_parameter = self.EXCEPTION_ERROR_CODE_INVALID_PARAMETER
        self._exception_error_code_operation_failed = self.EXCEPTION_ERROR_CODE_OPERATION_FAILED
        self._processing_timeout = 60
        self._loading_timeout = 120

    # ------------------------------------------------------------------------------------------------------------------
    # Module private functions
    def __wait_processing(self):
        """
        Function used to wait during loading processing. Default timeout is fixed to 60 seconds.
        """
        init_time = time.time()
        processing = True
        while processing is True and (time.time() - init_time < self._processing_timeout):
            time.sleep(1)
            if self._driver.find_element_by_id("Loading").is_displayed() is not True:
                processing = False

        if processing is True:
            msg = "ASUS802.11AC: wait_processing timeout"
            self._raise_exception(self._exception_error_code_operation_failed, msg)

    def __wait_loading(self):
        """
        Function used to wait during configuration file loading. Default timeout is fixed to 120 seconds.
        """
        init_time = time.time()
        loading = True
        has_been_displayed = False
        while loading is True and (time.time() - init_time < self._loading_timeout):
            try:
                loading_bar = self._driver.find_element_by_id("LoadingBar")
                if loading_bar.is_displayed():
                    has_been_displayed = True
                    time.sleep(2)
                    self._asus_print("ASUS802.11AC: Loading bar is visible")
            except Exception as exception:
                if not has_been_displayed:
                    msg = "ASUS802.11AC: Loading bar never seen : %s" % str(exception)
                    self._asus_print(msg)
                    self._raise_exception(self._exception_error_code_operation_failed, msg)
                else:
                    self._asus_print("ASUS802.11AC: Loading bar no more visible")
                    loading = False

        if loading is True:
            msg = "ASUS802.11AC: wait_loading timeout"
            self._raise_exception(self._exception_error_code_operation_failed, msg)

    def _load_wireless_page_and_apply_setting(self, page_to_load, function_to_set_field, *args):
        """
        Function to load wireless general page, set and apply a new parameter

        :type page_to_load: int
        :param page_to_load: Wireless page where the field can be set.
        :type function_to_set_field: method
        :param function_to_set_field: method to set a specific field on the page
        :type args: arguments
        :param args: arguments of the function function_to_set_field
        """
        if page_to_load not in self.ASUS_WIRELESS_PAGES:
            self._raise_exception(self._exception_error_code_invalid_parameter,
                                  "Invalid page to load in Wireless options : %s" % page_to_load)

        self._driver.find_element_by_id("option_str1").click()
        self._driver.find_element_by_xpath(
            "//div[@id='tabMenu']/table/tbody/tr/td[%s]/div/span/table/tbody/tr/td" % page_to_load).click()
        function_to_set_field(*args)
        if page_to_load == self.ASUS_WIRELESS_PAGE_GENERAL:
            self._driver.find_element_by_css_selector("input#applyButton.button_gen").click()
        else:
            self._driver.find_element_by_css_selector("input.button_gen").click()
        self.__wait_processing()

    def _load_wireless_page_and_get_setting(self, page_to_load, function_to_get_field, *args):
        """
        Function to load wireless general page, set and apply a new parameter

        :type page_to_load: int
        :param page_to_load: Wireless page where the field can be get.
        :type function_to_get_field: method
        :param function_to_get_field: method to get value of a specific field on the page
        :type args: arguments
        :param args: arguments of the function function_to_get_field

        :rtype: function_to_get_field return type
        :return: return the value provided by function_to_get_field function
        """
        if page_to_load not in self.ASUS_WIRELESS_PAGES:
            self._raise_exception(self._exception_error_code_invalid_parameter,
                                  "Invalid page to load in Wireless options : %s" % page_to_load)

        self._driver.find_element_by_id("option_str1").click()
        self._driver.find_element_by_xpath(
            "//div[@id='tabMenu']/table/tbody/tr/td[%s]/div/span/table/tbody/tr/td" % page_to_load).click()
        return function_to_get_field(*args)

    def _load_admin_page_and_upload_config_file(self, config_file):
        """
        Function to load a config file to the AP

        :type config_file: str
        :param config_file: The path to the config file
        """
        self._driver.find_element_by_xpath("//div[@id='subMenu']/div[@id='option9']").click()
        self._driver.find_element_by_xpath("//div[@id='tabMenu']/table/tbody/tr/td[4]/div").click()
        self._driver.find_element_by_xpath("//table[@id='FormTitle']/tbody/tr/td/table/tbody/tr[3]/td/div/table/tbody/tr/td[2]/input").send_keys(config_file)
        self._driver.find_element_by_xpath("//table[@id='FormTitle']/tbody/tr/td/table/tbody/tr[3]/td/div/table/tbody/tr/td[1]/input").click()
        self.__wait_loading()

    # ------------------------------------------------------------------------------------------------------------------
    # Module private functions - Get/Set field or setting functions
    # Take care to have correctly loaded their page before call !
    def __get_field_channel(self, radio):
        """
        Get field channel. Wireless general page must be loaded before.

        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz
        """
        self._check_param_radio(radio)
        Select(self._driver.find_element_by_css_selector("select[name=\"wl_unit\"]")).select_by_visible_text(radio)
        return Select(self._driver.find_element_by_name("wl_channel")).first_selected_option

    def __set_field_authentication_type(self, authentication_type, passphrase, radio, encryption_mode=None):
        """
        Set field authentication type. Wireless general page must be loaded before.

        :type authentication_type: str
        :param authentication_type: Authentication supported by the equipment. Possible values :
            'Open System', 'WPA2-Personal', 'WPA2-Enterprise', 'WPA-Auto-Personal', 'WPA-Auto-Enterprise'
        :type passphrase: str
        :param passphrase: Passphrase used for the authentication
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz
        :type encryption_mode: str
        :param encryption_mode: type of encryption
        """
        self._check_param_radio(radio)
        Select(self._driver.find_element_by_css_selector("select[name=\"wl_unit\"]")).select_by_visible_text(radio)
        Select(self._driver.find_element_by_name("wl_auth_mode_x")).select_by_visible_text(authentication_type)
        if authentication_type in ["WPA-Auto-Personal", "WPA2-Personal"]:
            self._driver.find_element_by_name("wl_wpa_psk").clear()
            self._driver.find_element_by_name("wl_wpa_psk").send_keys(passphrase)
        if authentication_type in ["WPA-Auto-Personal", "WPA-Auto-Enterprise"]:
            Select(self._driver.find_element_by_name("wl_crypto")).select_by_visible_text(encryption_mode)

    def __set_field_bandwidth(self, bandwidth, radio):
        """
        Set field bandwidth. Wireless general page must be loaded before.

        :type bandwidth: str
        :param bandwidth: bandwidth to set - values :
            2.4GHz : "20 MHz","40 MHz","20/40 MHz"
            5GHz : "20 MHz","40 MHz","80 MHz","20/40/80 MHz"
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz
        """
        self._check_param_radio(radio)
        Select(self._driver.find_element_by_css_selector("select[name=\"wl_unit\"]")).select_by_visible_text(radio)
        Select(self._driver.find_element_by_name("wl_bw")).select_by_visible_text(str(bandwidth))

    def __set_field_beacon(self, beacon, radio):
        """
        Set field beacon. Wireless professional page must be loaded before.

        :type beacon: str
        :param beacon: beacon value
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz
        """
        self._check_param_radio(radio)
        Select(self._driver.find_element_by_css_selector("select[name=\"wl_unit\"]")).select_by_visible_text(radio)
        self._driver.find_element_by_name("wl_bcn").clear()
        self._driver.find_element_by_name("wl_bcn").send_keys(beacon)

    def __set_field_channel(self, radio, channel, extension_channel=None):
        """
        Set field channel. Wireless general page must be loaded before.

        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz
        :type channel: integer or Digit-String or str
        :param channel: The wifi channel to set. Possible values : Auto,1,2,3,4,5,6,7,8,9,10,11,12,13,36,40,44,48
        :type extension_channel: str
        :param extension_channel: If 40MHz is used, set "Above" of "Below" channel. Optional.
        """
        self._check_param_radio(radio)
        Select(self._driver.find_element_by_css_selector("select[name=\"wl_unit\"]")).select_by_visible_text(radio)
        Select(self._driver.find_element_by_name("wl_channel")).select_by_visible_text(str(channel))
        if extension_channel is not None and radio == "2.4GHz":
            if extension_channel not in ["Above", "Below"]:
                self._raise_exception(self._exception_error_code_invalid_parameter,
                                      "Unknown extension channel parameter : %s" % extension_channel)
            Select(self._driver.find_element_by_css_selector("select[name=\"wl_nctrlsb\"]")).select_by_visible_text(
                extension_channel)

    def __set_field_disable_radio(self, radio):
        """
        Set field to disable radio. Wireless professional page must be loaded before.

        :type radio: str
        :param radio: type of radio to enable. Possible values : 2.4GHz, 5GHz
        """
        self._check_param_radio(radio)
        Select(self._driver.find_element_by_css_selector("select[name=\"wl_unit\"]")).select_by_visible_text(radio)
        self._driver.find_element_by_xpath("//*[@name='wl_radio' and @type='radio' and @value='0']").click()
        self.__wait_processing()

    def __set_field_dtim(self, dtim, radio):
        """
        Set field dtim. Wireless professional page must be loaded before.

        :type dtim: str
        :param dtim: dtim value
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz
        """
        self._check_param_radio(radio)
        Select(self._driver.find_element_by_css_selector("select[name=\"wl_unit\"]")).select_by_visible_text(radio)
        self._driver.find_element_by_name("wl_dtim").clear()
        self._driver.find_element_by_name("wl_dtim").send_keys(dtim)

    def __set_field_enable_radio(self, radio):
        """
        Set field to enable radio. Wireless professional page must be loaded before.

        :type radio: str
        :param radio: type of radio to enable. Possible values : 2.4GHz, 5GHz
        """
        self._check_param_radio(radio)
        Select(self._driver.find_element_by_css_selector("select[name=\"wl_unit\"]")).select_by_visible_text(radio)
        self._driver.find_element_by_xpath("//*[@name='wl_radio' and @type='radio' and @value='1']").click()
        self.__wait_processing()

    def __set_field_ssid(self, ssid, hide, radio):
        """
        Set field to create a SSID. Wireless general page must be loaded before.

        :type ssid: str
        :param ssid: ssid to apply on the AP
        :type hide: str
        :param hide: set "ON" or "1" to hide the SSID, use "OFF" or "0" to broadcast SSID
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz
        """
        self._check_param_radio(radio)
        Select(self._driver.find_element_by_css_selector("select[name=\"wl_unit\"]")).select_by_visible_text(radio)
        self._driver.find_element_by_id("wl_ssid").clear()
        self._driver.find_element_by_id("wl_ssid").send_keys(ssid)
        if hide in ("ON", "1"):
            self._driver.find_element_by_xpath("//*[@name='wl_closed' and @type='radio' and @value='1']").click()
        else:
            self._driver.find_element_by_xpath("//*[@name='wl_closed' and @type='radio' and @value='0']").click()

    def __set_field_standard(self, standard, radio):
        """
        Set field standard. Wireless general page must be loaded before.

        :type standard: str
        :param standard: standard to set on the AP. Possible values :
            2.4GHz : Auto, Legacy, N Only
            5GHz : Auto, Legacy, N + AC
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz
        """
        self._check_param_radio(radio)
        Select(self._driver.find_element_by_css_selector("select[name=\"wl_unit\"]")).select_by_visible_text(radio)
        Select(self._driver.find_element_by_css_selector("select[name=\"wl_nmode_x\"]")).select_by_visible_text(
            standard)

    def __set_field_tx_power_adjustment(self, tx_value, radio):
        """
        Set field authentication type. Wireless general page must be loaded before.

        :type tx_value: str
        :param tx_value: Value of power to set. Range is from 1 to 100.
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz
        """
        self._check_param_radio(radio)
        Select(self._driver.find_element_by_css_selector("select[name=\"wl_unit\"]")).select_by_visible_text(radio)
        self._driver.find_element_by_name("wl_txpower").clear()
        self._driver.find_element_by_name("wl_txpower").send_keys(tx_value)

    def __set_field_wmm(self, wmm_mode, radio):
        """
        Set field wmm. Wireless professional page must be loaded before.

        :type wmm_mode: str
        :param wmm_mode: wmm value. "Enable" or "Disable"
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz
        """
        self._check_param_radio(radio)
        Select(self._driver.find_element_by_css_selector("select[name=\"wl_unit\"]")).select_by_visible_text(radio)
        Select(self._driver.find_element_by_css_selector("select[name=\"wl_wme_apsd\"]")).select_by_visible_text(
            wmm_mode)

    def __set_fields_page_general(self, radio, ssid, hide, standard, bandwidth, channel, extension_channel,
                                  authentication_type, encryption_mode, passphrase):
        """
        Function to configure the wireless general page. If an argument is set to None, it is not changed.

        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz
        :type ssid: str
        :param ssid: ssid to apply on the AP
        :type hide: str
        :param hide: set "ON" or "1" to hide the SSID, use "OFF" or "0" to broadcast SSID
        :type standard: str
        :param standard: standard to set on the AP. Possible values :
            2.4GHz : 2.4GHz_Auto, 2.4GHz_Legacy, N Only - (b, g, bg, bgn, n2.4G configure on Auto mode)
            5GHz : 5GHz_Auto, 5GHz_Legacy, N + AC  - (a, an, n5G, ac configure on Auto mode)
        :type bandwidth: str
        :param bandwidth: bandwidth to set - values :
            2.4GHz : "20 MHz","40 MHz","20/40 MHz"
            5GHz : "20 MHz","40 MHz","80 MHz","20/40/80 MHz"
        :type channel: integer or Digit-String or str
        :param channel: The wifi channel to set. Possible values : Auto,1,2,3,4,5,6,7,8,9,10,11,12,13,36,40,44,48
        :type extension_channel: str
        :param extension_channel: If 40MHz is used, set "Above" of "Below" channel.
        :type authentication_type: str
        :param authentication_type: Authentication supported by the equipment. Possible values :
            'Open System', 'WPA2-Personal', 'WPA2-Enterprise', 'WPA-Auto-Personal', 'WPA-Auto-Enterprise'
        :type encryption_mode: str
        :param encryption_mode: type of encryption
        :type passphrase: str
        :param passphrase: Passphrase used for the authentication
        """
        self._check_param_radio(radio)
        if ssid is not None and hide is not None:
            self.__set_field_ssid(ssid, hide, radio)
        if standard is not None:
            asus_standard = "Auto"
            if standard == "N Only":
                asus_standard = "N Only"
            elif standard == "N + AC":
                asus_standard = "N + AC"
            elif standard == "2.4GHz_Legacy" or standard == "5GHz_Legacy":
                asus_standard = "Legacy"
            self.__set_field_standard(asus_standard, radio)
        if bandwidth is not None:
            self._check_param_bandwidth(bandwidth, radio)
            self.__set_field_bandwidth(bandwidth, radio)
        if channel is not None:
            self._check_param_channel(channel, radio)
            self.__set_field_channel(radio, channel, extension_channel)
        if authentication_type is not None:
            self._check_param_authentication_type(authentication_type)
            if encryption_mode is not None:
                self._check_param_encryption_mode(encryption_mode)
            self.__set_field_authentication_type(authentication_type, passphrase, radio, encryption_mode)

    def __set_fields_page_professional(self, radio, radio_state, dtim, beacon, wmm, tx_power):
        """
        Function to configure the wireless general page. If an argument is set to None, it is not changed.

        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz
        :type radio_state: str
        :param radio_state: Enable/Disable radio. Possible values : "Enable", "Disable"
        :type dtim: str
        :param dtim: dtim value
        :type beacon: str
        :param beacon: beacon value
        :type wmm: str
        :param wmm: wmm value. "Enable" or "Disable"
        :type tx_power: str
        :param tx_power: Value of power to set. Range is from 1 to 100.
        """
        self._check_param_radio(radio)
        if radio_state is not None:
            if radio_state == "Enable":
                self.__set_field_enable_radio(radio)
            elif radio_state == "Disable":
                self.__set_field_disable_radio(radio)
            else:
                self._raise_exception(self._exception_error_code_invalid_parameter,
                                      "Invalid radio state parameter : %s" % radio_state)
        if dtim is not None:
            self.__set_field_dtim(dtim, radio)
        if beacon is not None:
            self.__set_field_beacon(beacon, radio)
        if wmm is not None:
            self._check_param_wmm(wmm)
            self.__set_field_wmm(wmm, radio)
        if tx_power is not None:
            self._check_param_tx_power(tx_power)
            self.__set_field_tx_power_adjustment(tx_power, radio)

    def __set_fields_page_radius(self, radius_ip, radius_port, radius_secret, radio):
        """
        Set field authentication type. Wireless general page must be loaded before.

        :type radius_ip: str
        :param radius_ip: Address of the radius server
        :type radius_port: str
        :param radius_port: port to connect to the radius server
        :type radius_secret: str
        :param radius_secret: Password to communicate between AP and Radius server
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz
        """
        self._check_param_radio(radio)
        Select(self._driver.find_element_by_css_selector("select[name=\"wl_unit\"]")).select_by_visible_text(radio)
        self._driver.find_element_by_name("wl_radius_ipaddr").clear()
        self._driver.find_element_by_name("wl_radius_ipaddr").send_keys(radius_ip)
        self._driver.find_element_by_name("wl_radius_port").clear()
        self._driver.find_element_by_name("wl_radius_port").send_keys(radius_port)
        self._driver.find_element_by_name("wl_radius_key").clear()
        self._driver.find_element_by_name("wl_radius_key").send_keys(radius_secret)

    # ------------------------------------------------------------------------------------------------------------------
    # Module private functions - Checks and configure functions parameters
    def _check_param_authentication_type(self, authentication_type):
        """
        Check authentication type

        :type authentication_type: str
        :param authentication_type: authentication type parameter
        """
        if authentication_type not in self.ASUS_RT_AC66U_AUTHENTICATION_METHOD:
            if authentication_type in self.ASUS_RT_AC66U_AUTHENTICATION_LEGACY_METHOD:
                self._raise_exception(self._exception_error_code_invalid_parameter,
                                      "Authentication method not supported by ACS : %s - Require new implementation" % authentication_type)
            self._raise_exception(self._exception_error_code_invalid_parameter,
                                  "Invalid value for authentication type : %s" % authentication_type)

    def _check_param_bandwidth(self, bandwidth, radio):
        """
        Check bandwidth parameter.

        :type bandwidth: str
        :param bandwidth: bandwidth parameter
        :type radio: str
        :param radio: radio parameter
        """
        self._check_param_radio(radio)
        if radio == "2.4GHz":
            if bandwidth not in self.ASUS_RT_AC66U_BANDWIDTHS_2_4GHZ:
                self._raise_exception(self._exception_error_code_invalid_parameter,
                                      "Invalid value for bandwidth parameter %s with %s radio" % (bandwidth, radio))
        elif radio == "5GHz":
            if bandwidth not in self.ASUS_RT_AC66U_BANDWIDTHS_5GHZ:
                self._raise_exception(self._exception_error_code_invalid_parameter,
                                      "Invalid value for bandwidth parameter %s with %s radio" % (bandwidth, radio))

    def _check_param_channel(self, channel, radio, extension_channel=None):
        """
        Check channel parameters.

        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz
        :type channel: integer or Digit-String or str
        :param channel: The wifi channel to set. Possible values : Auto,1,2,3,4,5,6,7,8,9,10,11,12,13,36,40,44,48
        :type extension_channel: str
        :param extension_channel: If 40MHz is used, set "Above" of "Below" channel. Optional.
        """
        self._check_param_radio(radio)
        if radio == "2.4GHz":
            if channel not in ["Auto", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14"]:
                self._raise_exception(self._exception_error_code_invalid_parameter,
                                      "Invalid channel %s for %s radio" % (channel, radio))
        if radio == "5GHz":
            if channel not in ["Auto", "36", "40", "44", "48"]:
                self._raise_exception(self._exception_error_code_invalid_parameter,
                                      "Invalid channel %s for %s radio" % (channel, radio))

        if extension_channel is not None:
            if extension_channel not in ["Above", "Below"]:
                self._raise_exception(self._exception_error_code_invalid_parameter,
                                      "Invalid extension channel parameter : %s" % extension_channel)

    def _check_param_encryption_mode(self, encryption_mode):
        """
        Check encryption mode

        :type encryption_mode: str
        :param encryption_mode: encryption mode parameter
        """
        if encryption_mode not in self.ASUS_RT_AC66U_ENCRYPTION_MODE:
            self._raise_exception(self._exception_error_code_invalid_parameter,
                                  "Invalid encryption mode : %s" % encryption_mode)
        if encryption_mode in ["WEP-64bits", "WEP-128bits"]:
            self._raise_exception(self._exception_error_code_invalid_parameter,
                                  "Encryption mode %s is not supported with current implementation. It requires Legacy mode support." % encryption_mode)

    def _check_param_radio(self, radio, all_authorized=False):
        """
        Check radio parameter.

        :type radio: str
        :param radio: radio parameter
        :type all_authorized: bool
        :param all_authorized: if value "all" is authorized as parameter, set to True
        """
        if all_authorized is True:
            if radio not in self.ASUS_RT_AC66U_RADIOS and radio != "all":
                self._raise_exception(self._exception_error_code_invalid_parameter,
                                      "Invalid value for radio parameter : %s" % radio)
        else:
            if radio not in self.ASUS_RT_AC66U_RADIOS:
                self._raise_exception(self._exception_error_code_invalid_parameter,
                                      "Invalid value for radio parameter : %s" % radio)

    def _check_param_tx_power(self, tx_power):
        """
        Check tx_power value

        :type tx_power: str
        :param tx_power: tx_power parameter
        """
        if int(tx_power) < 1 or int(tx_power) > 100:
            self._raise_exception(self._exception_error_code_invalid_parameter,
                                  "TX Power %s is out of range" % tx_power)

    def _check_param_wmm(self, wmm):
        """
        Check wmm value

        :type wmm: str
        :param wmm: wmm parameter
        """
        if wmm not in ['Enable', 'Disable']:
            self._raise_exception(self._exception_error_code_invalid_parameter, "Invalid wmm mode : %s" % wmm)

    # ------------------------------------------------------------------------------------------------------------------
    # Module public functions - Should be overwritten if used with other framework
    @staticmethod
    def __asus_print_default(msg):
        """
        Default function to print some logs
        """
        print msg

    @staticmethod
    def __raise_exception_default(error_code="DefaultCode", error_msg="DefaultMsg"):
        """
        Default function to raise an exception
        """
        raise Exception(error_msg)

    # ------------------------------------------------------------------------------------------------------------------
    # Module public functions - Control Asus AC66U API
    def configure_module(self, print_log=None, raise_exception=None, exception_error_code_invalid_parameter=None,
                         exception_error_code_operation_failed=None, processing_timeout=None):
        """
        Initialize and configure the module to control the AP. Must be called before other functions !

        :type print_log: method object
        :param print_log: method to print logs to link to your framework
        :type raise_exception: method object
        :param raise_exception: method to raise exceptions to link to your framework
        :type exception_error_code_invalid_parameter: str
        :param exception_error_code_invalid_parameter: error code for invalid parameters exceptions
        :type exception_error_code_operation_failed: str
        :param exception_error_code_operation_failed: error code for failed operations exceptions
        """
        if print_log is not None:
            self._asus_print = print_log
        if raise_exception is not None:
            self._raise_exception = raise_exception
        if exception_error_code_invalid_parameter is not None:
            self._exception_error_code_invalid_parameter = exception_error_code_invalid_parameter
        if exception_error_code_operation_failed is not None:
            self._exception_error_code_operation_failed = exception_error_code_operation_failed
        if processing_timeout is not None:
            self._processing_timeout = processing_timeout

        self._asus_print("ASUS802.11AC: init Firefox web driver")
        profile = webdriver.FirefoxProfile()
        profile.set_preference("network.proxy.type", 0)
        self._driver = webdriver.Firefox(profile)

    def connect_to_asus(self, ip, login, password):
        """
        Connect to AP
        """
        self._asus_print("ASUS802.11AC: Login web page")
        self._driver.implicitly_wait(self.ASUS_IMPLICITLY_WAIT)
        self._driver.get("http://" + login + ":" + password + "@" + ip + "/")

    def disconnect_from_asus(self):
        """
        Disconnect from AP
        """
        self._driver.quit()

    def load_config_file(self, config_file):
        try:
            self._load_admin_page_and_upload_config_file(config_file)
        except Exception as exception:
            msg = "ASUS802.11AC: Error during load_config_file : %s" % str(exception)
            self._asus_print(msg)
            self._raise_exception(self._exception_error_code_operation_failed, msg)

    def enable_wireless(self, radio="all"):
        """
        Enable AP WiFi radio

        :type radio: str
        :param radio: type of radio to enable. Possible values : 2.4GHz, 5GHz or all
        """

        self._asus_print("ASUS802.11AC: Enable Wireless on %s radio" % radio)
        self._check_param_radio(radio, True)

        if radio != "5GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_PROFESSIONAL,
                                                       self.__set_field_enable_radio, "2.4GHz")
        if radio != "2.4GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_PROFESSIONAL,
                                                       self.__set_field_enable_radio, "5GHz")

    def disable_wireless(self, radio="all"):
        """
        Disable AP WiFi radio

        :type radio: str
        :param radio: type of radio to disable. Possible values : 2.4GHz, 5GHz or all
        """
        self._asus_print("ASUS802.11AC: Disable Wireless %s radio" % radio)
        self._check_param_radio(radio, True)

        if radio != "5GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_PROFESSIONAL,
                                                       self.__set_field_disable_radio, "2.4GHz")
        if radio != "2.4GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_PROFESSIONAL,
                                                       self.__set_field_disable_radio, "5GHz")

    def create_ssid(self, ssid, hide="OFF", radio="all"):
        """
        Create an SSID on Asus

        :type ssid: str
        :param ssid: ssid to apply on the AP
        :type hide: str
        :param hide: set "ON" or "1" to hide the SSID, use "OFF" or "0" to broadcast SSID
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz, all
        """
        self._asus_print("ASUS802.11AC: CreateSSID configuration will be applied to both radio")
        self._check_param_radio(radio, True)
        if hide not in self.ASUS_RT_AC66U_SSID_HIDDEN_VALUES:
            self._raise_exception(self._exception_error_code_invalid_parameter,
                                  "Unknown value for hide parameter : %s" % hide)

        if radio != "5GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_GENERAL, self.__set_field_ssid, ssid,
                                                       hide, "2.4GHz")
        if radio != "2.4GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_GENERAL, self.__set_field_ssid, ssid,
                                                       hide, "5GHz")

    def set_wifi_bandwidth(self, bandwidth, radio):
        """
        Set bandwidth on Asus

        :type bandwidth: str
        :param bandwidth: bandwidth to set - values :
            2.4GHz : "20 MHz","40 MHz","20/40 MHz"
            5GHz : "20 MHz","40 MHz","80 MHz","20/40/80 MHz"
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz
        """
        self._asus_print("ASUS802.11AC: SetBandwidth %s on %s radio" % (bandwidth, radio))
        self._check_param_bandwidth(bandwidth, radio)

        self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_GENERAL, self.__set_field_bandwidth,
                                                   bandwidth, radio)

    def set_wifi_channel(self, radio, channel, extension_channel=None):
        """
        Set wifi channel

        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz
        :type channel: integer or Digit-String or str
        :param channel: The wifi channel to set. Possible values : Auto,1,2,3,4,5,6,7,8,9,10,11,12,13,36,40,44,48
        :type extension_channel: str
        :param extension_channel: If 40MHz is used, set "Above" of "Below" channel. Optional.
        """
        self._asus_print("ASUS802.11AC: Set Channel %s" % channel)
        self._check_param_radio(radio)
        self._check_param_channel(channel, radio, extension_channel)

        self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_GENERAL, self.__set_field_channel, radio,
                                                   channel, extension_channel)

    def get_selected_channel(self, radio):
        """
        Return the current channel for a given radio.

        :type radio: str
        :param radio: type of radio. Possible values : "2.4GHz", "5GHz"

        :rtype: str
        :return: the current channel of the radio
        """
        self._asus_print("ASUS802.11AC: Get selected channel on %s radio" % radio)
        self._check_param_radio(radio)

        element = self._load_wireless_page_and_get_setting(self.ASUS_WIRELESS_PAGE_GENERAL, self.__get_field_channel,
                                                           radio)
        if element is not None:
            return element.get_attribute("value")
        else:
            self._raise_exception(self._exception_error_code_operation_failed, "Can't find current channel")

    def set_wifi_beacon(self, beacon, radio="all"):
        """
        Set Wifi beacon interval

        :type beacon: int
        :param beacon: interval in ms
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz, all
        """

        self._asus_print("ASUS802.11AC: Set Beacon interval to " + str(beacon) + " on " + str(radio) + " radio")
        self._check_param_radio(radio, True)

        if radio != "5GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_PROFESSIONAL, self.__set_field_beacon,
                                                       beacon, "2.4GHz")
        if radio != "2.4GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_PROFESSIONAL, self.__set_field_beacon,
                                                       beacon, "5GHz")

    def set_wifi_dtim(self, dtim, radio="all"):
        """
        Set Wifi DTIM

        :type dtim: int
        :param dtim: interval in ms
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz, all
        """
        self._asus_print("ASUS802.11AC: Set DTIM to " + str(dtim) + " on " + str(radio) + " radio")
        self._check_param_radio(radio, True)

        if radio != "5GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_PROFESSIONAL, self.__set_field_dtim,
                                                       dtim, "2.4GHz")
        if radio != "2.4GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_PROFESSIONAL, self.__set_field_dtim,
                                                       dtim, "5GHz")

    def set_wifi_wmm(self, mode, radio="all"):
        """
        Enable/Disable Wifi wireless Multimedia extensions

        :type mode: str or int
        :param mode: can be 'Enable' or 'Disable'
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz, all
        """
        self._asus_print("ASUS802.11AC: Set WMM to " + str(mode) + " on " + str(radio) + " radio")
        self._check_param_radio(radio, True)
        self._check_param_wmm(mode)

        if radio != "5GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_PROFESSIONAL, self.__set_field_wmm, mode,
                                                       "2.4GHz")
        if radio != "2.4GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_PROFESSIONAL, self.__set_field_wmm, mode,
                                                       "5GHz")

    def set_wifi_standard(self, standard, mimo=False):
        """
        Configure WiFi standard of Asus AP

        :type standard: str
        :param standard: standard to set on the AP. Possible values :
            2.4GHz : 2.4GHz_Auto, 2.4GHz_Legacy, N Only - (b, g, bg, bgn, n2.4G configure on Auto mode)
            5GHz : 5GHz_Auto, 5GHz_Legacy, N + AC  - (a, an, n5G, ac configure on Auto mode)
        :type mimo: boolean
        :param mimo: enable or disable mimo
        """
        self._asus_print("ASUS802.11AC: Set Standard %s" % standard)
        asus_radio = None
        if standard in ['2.4GHz_Auto', '2.4GHz_Legacy', 'N Only', 'b', 'g', 'bg', 'bgn', 'n2.4G']:
            asus_radio = "2.4GHz"
        elif standard in ['5GHz_Auto', '5GHz_Legacy', 'N + AC', 'a', 'an', 'n5G', 'ac']:
            asus_radio = "5GHz"
        else:
            self._raise_exception(self._exception_error_code_invalid_parameter,
                                  "Invalid standard parameter : %s" % standard)

        if mimo is not False:
            self._asus_print("MIMO not implemented on Asus AC66U AP")
            self._raise_exception(self._exception_error_code_invalid_parameter, "MIMO not implemented")

        asus_standard = "Auto"
        if standard == "N Only":
            asus_standard = "N Only"
        elif standard == "N + AC":
            asus_standard = "N + AC"
        elif standard == "2.4GHz_Legacy" or standard == "5GHz_Legacy":
            asus_standard = "Legacy"

        self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_GENERAL, self.__set_field_standard,
                                                   asus_standard, asus_radio)

    def set_wifi_authentication(self, authentication_type, passphrase=None, radio="all", encryption_mode=None):
        """
        Set the authentication on the Asus.

        :type authentication_type: str
        :param authentication_type: Authentication supported by the equipment. Possible values :
            'Open System', 'WPA2-Personal', 'WPA2-Enterprise', 'WPA-Auto-Personal', 'WPA-Auto-Enterprise'
        :type passphrase: str
        :param passphrase: Passphrase used for the authentication
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz, all
        :type encryption_mode: str
        :param encryption_mode: type of encryption
        """
        self._asus_print(
            "ASUS802.11AC: Set WiFi authentication - Type %s - Passphrase %s" % (authentication_type, passphrase))
        self._check_param_authentication_type(authentication_type)
        self._check_param_radio(radio, True)
        self._check_param_encryption_mode(encryption_mode)

        if radio != "5GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_GENERAL,
                                                       self.__set_field_authentication_type, authentication_type,
                                                       passphrase, "2.4GHz", encryption_mode)
        if radio != "2.4GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_GENERAL,
                                                       self.__set_field_authentication_type, authentication_type,
                                                       passphrase, "5GHz", encryption_mode)

    def set_wifi_tx_power_adjustment(self, tx_value, radio="all"):
        """
        Set TX power to a specific value for a radio.

        :type tx_value: str
        :param tx_value: Value of power to set. Range is from 1 to 100.
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz, all
        """
        self._asus_print("ASUS802.11AC: Set tx power adjustment to " + str(tx_value) + " on " + radio + " radio")
        self._check_param_radio(radio, True)
        self._check_param_tx_power(tx_value)

        if radio != "5GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_PROFESSIONAL,
                                                       self.__set_field_tx_power_adjustment, tx_value, "2.4GHz")
        if radio != "2.4GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_PROFESSIONAL,
                                                       self.__set_field_tx_power_adjustment, tx_value, "5GHz")

    # ------------------------------------------------------------------------------------------------------------------
    # Module public functions - Page configurations API
    def set_page_wireless_general(self, radio="all", ssid=None, hide=None, standard=None, bandwidth=None, channel=None,
                                  extension_channel=None, authentication_type=None, encryption_mode=None,
                                  passphrase=None):
        """
        Function to configure the wireless general page. If an argument is set to None, it is not changed.

        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz, all
        :type ssid: str
        :param ssid: ssid to apply on the AP
        :type hide: str
        :param hide: set "ON" or "1" to hide the SSID, use "OFF" or "0" to broadcast SSID
        :type standard: str
        :param standard: standard to set on the AP. Possible values :
            2.4GHz : 2.4GHz_Auto, 2.4GHz_Legacy, N Only - (b, g, bg, bgn, n2.4G configure on Auto mode)
            5GHz : 5GHz_Auto, 5GHz_Legacy, N + AC  - (a, an, n5G, ac configure on Auto mode)
        :type bandwidth: str
        :param bandwidth: bandwidth to set - values :
            2.4GHz : "20 MHz","40 MHz","20/40 MHz"
            5GHz : "20 MHz","40 MHz","80 MHz","20/40/80 MHz"
        :type channel: integer or Digit-String or str
        :param channel: The wifi channel to set. Possible values : Auto,1,2,3,4,5,6,7,8,9,10,11,12,13,36,40,44,48
        :type extension_channel: str
        :param extension_channel: If 40MHz is used, set "Above" of "Below" channel. Optional.
        :type authentication_type: str
        :param authentication_type: Authentication supported by the equipment. Possible values :
            'Open System', 'WPA2-Personal', 'WPA2-Enterprise', 'WPA-Auto-Personal', 'WPA-Auto-Enterprise'
        :type encryption_mode: str
        :param encryption_mode: type of encryption
        :type passphrase: str
        :param passphrase: Passphrase used for the authentication
        """
        self._asus_print("ASUS802.11AC: Configure wireless general page - Radio %s" % radio)
        self._check_param_radio(radio, True)

        if radio != "5GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_GENERAL, self.__set_fields_page_general,
                                                       "2.4GHz", ssid, hide, standard, bandwidth, channel,
                                                       extension_channel, authentication_type, encryption_mode,
                                                       passphrase)
        if radio != "2.4GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_GENERAL, self.__set_fields_page_general,
                                                       "5GHz", ssid, hide, standard, bandwidth, channel,
                                                       extension_channel, authentication_type, encryption_mode,
                                                       passphrase)

    def set_page_wireless_professional(self, radio="all", radio_state=None, dtim=None, beacon=None, wmm=None,
                                       tx_value=None):
        """
        Function to configure the wireless professional page. If an argument is set to None, it is not changed.

        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz, all
        :type radio_state: str
        :param radio_state: Enable/Disable radio. Possible values : "Enable", "Disable"
        :type dtim: str
        :param dtim: dtim value
        :type beacon: str
        :param beacon: beacon value
        :type wmm: str
        :param wmm: wmm value. Possible values : "Enable" or "Disable"
        :type tx_value: str
        :param tx_value: Value of power to set. Range is from 1 to 100.
        """
        self._asus_print("ASUS802.11AC: Configure wireless professional page - Radio %s" % radio)
        self._check_param_radio(radio, True)

        if radio != "5GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_PROFESSIONAL,
                                                       self.__set_fields_page_professional, "2.4GHz", radio_state, dtim,
                                                       beacon, wmm, tx_value)
        if radio != "2.4GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_PROFESSIONAL,
                                                       self.__set_fields_page_professional, "5GHz", radio_state, dtim,
                                                       beacon, wmm, tx_value)

    def set_page_wireless_radius(self, radius_ip, radius_port, radius_secret, radio="all"):
        """
        Configure radius on the AP

        :type radius_ip: str
        :param radius_ip: Address of the radius server (optional)
        :type radius_port: str
        :param radius_port: port to connect to the radius server (optional)
        :type radius_secret: str
        :param radius_secret: Password to communicate between AP and Radius server (optional)
        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz, all
        """
        self._asus_print("ASUS802.11AC: Configure wireless radius page - IP %s - Port %s - Secret %s - Radio %s" % (
            radius_ip, radius_port, radius_secret, radio))
        self._check_param_radio(radio, True)

        if radio != "5GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_RADIUS_SETTINGS,
                                                       self.__set_fields_page_radius, radius_ip, radius_port,
                                                       radius_secret, "2.4GHz")
        if radio != "2.4GHz":
            self._load_wireless_page_and_apply_setting(self.ASUS_WIRELESS_PAGE_RADIUS_SETTINGS,
                                                       self.__set_fields_page_radius, radius_ip, radius_port,
                                                       radius_secret, "5GHz")

    def set_full_config(self, radio="all", radio_state=None, ssid=None, hide=None, standard=None, bandwidth=None,
                        channel=None, extension_channel=None, authentication_type=None, encryption_mode=None,
                        passphrase=None, radius_ip=None, radius_port=None, radius_secret=None, beacon=None, dtim=None,
                        wmm=None, tx_value=None):
        """
        Set full configuration of the AP with given parameters. All parameters are optional.
        Only given parameters will be set during the config. If parameter value is "None", it is ignored.

        :type radio: str
        :param radio: Asus radio to apply the ssid. Possible values : 2.4GHz, 5GHz, all
        :type radio_state: str
        :param radio_state: Enable/Disable radio. Possible values : "Enable", "Disable"
        :type ssid: str
        :param ssid: ssid to apply on the AP
        :type hide: str
        :param hide: set "ON" or "1" to hide the SSID, use "OFF" or "0" to broadcast SSID
        :type standard: str
        :param standard: standard to set on the AP. Possible values :
            2.4GHz : 2.4GHz_Auto, 2.4GHz_Legacy, N Only - (b, g, bg, bgn, n2.4G configure on Auto mode)
            5GHz : 5GHz_Auto, 5GHz_Legacy, N + AC  - (a, an, n5G, ac configure on Auto mode)
        :type bandwidth: str
        :param bandwidth: bandwidth to set - values :
            2.4GHz : "20 MHz","40 MHz","20/40 MHz"
            5GHz : "20 MHz","40 MHz","80 MHz","20/40/80 MHz"
        :type channel: integer or Digit-String or str
        :param channel: The wifi channel to set. Possible values : Auto,1,2,3,4,5,6,7,8,9,10,11,12,13,36,40,44,48
        :type extension_channel: str
        :param extension_channel: If 40MHz is used, set "Above" of "Below" channel. Optional.
        :type authentication_type: str
        :param authentication_type: Authentication supported by the equipment. Possible values :
            'Open System', 'WPA2-Personal', 'WPA2-Enterprise', 'WPA-Auto-Personal', 'WPA-Auto-Enterprise'
        :type encryption_mode: str
        :param encryption_mode: type of encryption
        :type passphrase: str
        :param passphrase: Passphrase used for the authentication
        :type radius_ip: str
        :param radius_ip: Address of the radius server
        :type radius_port: str
        :param radius_port: port to connect to the radius server
        :type radius_secret: str
        :param radius_secret: Password to communicate between AP and Radius server
        :type beacon: str
        :param beacon: beacon value
        :type dtim: str
        :param dtim: dtim value
        :type wmm: str
        :param wmm: wmm value. Possible values : "Enable" or "Disable"
        :type tx_value: str
        :param tx_value: Value of power to set. Range is from 1 to 100.
        """
        self._asus_print("ASUS802.11AC: Full configuration - Pages used : General, Radius, Professional")

        self.set_page_wireless_general(radio=radio, ssid=ssid, hide=hide, standard=standard, bandwidth=bandwidth,
                                       channel=channel, extension_channel=extension_channel,
                                       authentication_type=authentication_type, encryption_mode=encryption_mode,
                                       passphrase=passphrase)
        self.set_page_wireless_radius(radius_ip=radius_ip, radius_port=radius_port, radius_secret=radius_secret,
                                      radio=radio)
        self.set_page_wireless_professional(radio=radio, radio_state=radio_state, dtim=dtim, beacon=beacon, wmm=wmm,
                                            tx_value=tx_value)

    # ------------------------------------------------------------------------------------------------------------------
    # Module public functions - API functions which are not fully implemented or need rework
    def load_settings_file(self, file_path):
        """
        Used to load a configured settings file

        :type file_path: str
        :param file_path: File to load on the AP
        """
        self._asus_print("ASUS802.11AC: restore settings")
        self._driver.find_element_by_xpath("(//div[@id='option_str1'])[7]").click()
        self._driver.find_element_by_xpath(
            "//div[@id='tabMenu']/table/tbody/tr/td[4]/div/span/table/tbody/tr/td").click()
        self._driver.find_element_by_name("file").send_keys(file_path)
        self._driver.find_element_by_css_selector("div > table > tbody > tr > td > input.button_gen").click()
        self.__wait_loading()

    def set_acl_mode(self, mode):
        self._asus_print("ASUS802.11AC: set acl mode not implemented - can't set mode %s" % mode)

    def add_mac_address_to_acl(self):
        self._asus_print("ASUS802.11AC: add acl mac@ not implemented")

    def get_regulatorydomain(self):
        """
        Get the current AP regulatory domain
        """
        self._asus_print("ASUS802.11AC: get_regulatorydomain not implemented")
        return "invalid_data"
