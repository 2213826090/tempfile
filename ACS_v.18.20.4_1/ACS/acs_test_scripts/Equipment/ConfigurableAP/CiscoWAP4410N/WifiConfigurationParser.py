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
:summary: implementation of Parser to read and update the wifi configuration file
:since:30/01/2012
:author: ssavrimoutou
"""

import urllib2
import mimetools
import binascii
import time
import os
from HTMLParser import HTMLParser
from ErrorHandling.TestEquipmentException import TestEquipmentException


class WifiConfigurationSections(object):

    """
    List all the sections available for the configuration file
    """
    WIRELESS_BASIC = '[Wireless_Basic]'
    WIRELESS_ADVANCED = '[Wireless_Advanced]'
    WIRELESS_SECURITY = {'1': '[Wireless_security_1]',
                         '2': '[Wireless_security_2]',
                         '3': '[Wireless_security_3]',
                         '4': '[Wireless_security_4]'}

    WIRELESS_CONTROL = {'1': '[Wireless_control_1]',
                        '2': '[Wireless_control_2]',
                        '3': '[Wireless_control_3]',
                        '4': '[Wireless_control_4]'}
    WIRELESS_VLAN_QOS = '[VLAN_QoS]'


class WifiConfigurationParser:

    """
    Parser to read and update the wifi AP configuration file
    The File contains multiple sections, Each sections will be accessible through methods
    """

    def __init__(self, eqt_logger, host, login, passwd):
        """
        Constructor
        """
        # Initialize the variables
        self._logger = eqt_logger
        self._host = host
        self._login = login
        self._passwd = passwd

        self._file_data = None
        self._crc = 0
        self._profile = '1'

    def get_logger(self):
        """
        Gets the internal logger of the equipment
        """
        return self._logger

    def _get_pos(self, item, base=None):
        """
        Get position of an item from the configuration file

        :type item: String
        :param item: item to find

        :type base: String
        :param base: base to refer to
        """
        of = 0
        if base is not None:
            of = self._file_data.find(base)
            if of == -1:
                of = 0
            else:
                of += len(base)
        offset = self._file_data[of:].find(item + '=')
        if offset != -1:
            pos = of + offset + len(item + '=')
            end = self._file_data[pos:].find('\n')
            if end != -1:
                end2 = self._file_data[pos:pos + end].find('\r')
                if end2 != -1 and end2 < end:
                    end = end2
                end2 = self._file_data[pos:pos + end].find(' ')
                if end2 != -1 and end2 < end:
                    end = end2
                return pos, end
        return -1, 0

    def get_value(self, item, base=None):
        """
        Get value of an item
        :type item: String
        :param item: item to find

        :type base: String
        :param base: base to refer to
        """
        pos, end = self._get_pos(item, base)
        if pos != -1:
            return self._file_data[pos:pos + end]
        return None

    def set_value(self, item, value, base=None):
        """
        Set value of an item
        :type item: String
        :param item: item to find

        :type value: String
        :param value: value of the item

        :type base: String
        :param base: base to refer to
        """
        pos, end = self._get_pos(item, base)
        if pos != -1:
            self._file_data = self._file_data[:pos] + str(value) + self._file_data[pos + end:]
        else:
            raise ValueError("item %s not found" % item)

    # Section [Wireless Basic]
    def get_wb_value(self, item):
        """
        get wireless basic value
        :type item: String
        :param item: item to find
        """
        return self.get_value(item, WifiConfigurationSections.WIRELESS_BASIC)

    def set_wb_value(self, item, value):
        """
        set wireless basic value
        :type item: String
        :param item: item to find
        :type value: String
        :param value: value of the item
        """
        self.set_value(item, value, WifiConfigurationSections.WIRELESS_BASIC)

    # Section [Wireless Advanced]
    def get_wa_value(self, item):
        """
        get wireless advanced value
        :type item: String
        :param item: item to find
        """
        return self.get_value(item, WifiConfigurationSections.WIRELESS_ADVANCED)

    def set_wa_value(self, item, value):
        """
        set wireless advanced value
        :type item: String
        :param item: item to find
        :type value: String
        :param value: value of the item
        """
        self.set_value(item, value, WifiConfigurationSections.WIRELESS_ADVANCED)

    def get_wa_keyof(self, wa_dict, item):
        """
        Get key of an item
        :type wa_dict: dict
        :param wa_dict: dictionary reference

        :type item: String
        :param item: item to find
        """
        return wa_dict.keys()[wa_dict.values().index(self.get_wa_value(item))]

    # Section [Wireless Security]
    def get_ws_value(self, item):
        """
        get wireless security value
        :type item: String
        :param item: item to find
        """
        wireless_security = \
            WifiConfigurationSections.WIRELESS_SECURITY[self._profile]
        return self.get_value(item, wireless_security)

    def set_ws_value(self, item, value):
        """
        set wireless security value
        :type item: String
        :param item: item to find
        :type value: String
        :param value: value of the item
        """
        wireless_security = \
            WifiConfigurationSections.WIRELESS_SECURITY[self._profile]
        self.set_value(item, value, wireless_security)

    def get_ws_keyof(self, ws_dict, item):
        """
        Get key of an item
        :type ws_dict: dict
        :param ws_dict: dictionary reference

        :type item: String
        :param item: item to find
        """
        return ws_dict.keys()[ws_dict.values().index(self.get_ws_value(item))]

    # Section [Wireless Control]
    def get_wc_value(self, item):
        """
        get wireless control value
        :type item: String
        :param item: item to find
        """
        wireless_control = \
            WifiConfigurationSections.WIRELESS_CONTROL[self._profile]

        return self.get_value(item, wireless_control)

    def set_wc_value(self, item, value):
        """
        get wireless control value
        :type item: String
        :param item: item to find
        :type value: String
        :param value: value of the item
        """
        wireless_control = \
            WifiConfigurationSections.WIRELESS_CONTROL[self._profile]

        self.set_value(item, value, wireless_control)

    def get_wc_keyof(self, wc_dict, item):
        """
        Get key of an item
        :type wc_dict: dict
        :param wc_dict: dictionary reference

        :type item: String
        :param item: item to find
        """
        return wc_dict.keys()[wc_dict.values().index(self.get_wc_value(item))]

    # Section [VLAN_QoS]
    def get_qos_value(self, item):
        """
        get wireless qos value
        :type item: String
        :param item: item to find
        """
        return self.get_value(item, WifiConfigurationSections.WIRELESS_VLAN_QOS)

    def set_qos_value(self, item, value):
        """
        get wireless qos value
        :type item: String
        :param item: item to find
        :type value: String
        :param value: value of the item
        """
        self.set_value(item, value, WifiConfigurationSections.WIRELESS_VLAN_QOS)

    def get_qos_keyof(self, qos_dict, item):
        """
        Get key of an item
        :type qos_dict: dict
        :param qos_dict: dictionary reference

        :type item: String
        :param item: item to find
        """
        return qos_dict.keys()[qos_dict.values().index(self.get_qos_value(item))]

    def __update_configuration_checksum(self):
        """
        Update the wifi configuration checksum
        """
        self._crc = binascii.crc32(self._file_data)

    def __check_configuration_checksum(self):
        """
        Check if config has changed and return False if not
        """
        return self._crc != binascii.crc32(self._file_data)

    def read_configuration(self, file_name):
        """
        Read configuration from a backup file

        :type file_name: String
        :param file_name: file to read
        """
        self.get_logger().debug("Read configuration from file '%s'" % str(file_name))

        self._file_data = open(file_name, 'rb').read()
        self.__update_configuration_checksum()

    def write_configuration(self, file_name):
        """
        Write configuration to a backup file

        :type file_name: String
        :param file_name: file to read
        """
        self.get_logger().debug("Write configuration to file '%s'" % str(file_name))

        if self.__check_configuration_checksum():
            open(file_name, 'wb').write(self._file_data)

    def download_configuration(self):
        """
        Download configuration from the router
        """
        self._file_data = self._get_ap_page('/download.cgi?next_file=wap4410n.cfg')

        if ('<HTML>' in self._file_data) or ('<html>' in self._file_data):
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, "invalid configuration page")
        else:
            self.__update_configuration_checksum()

    def upload_configuration(self, configuration_timer):
        """
        Upload configuration to the router

        :type configuration_timer: float
        :param configuration_timer: time to wait if a modification occurred \
                                    in the configuration in order to be \
                                    taken into account
        """
        self.get_logger().debug("Upload configuration to the equipment")

        if self.__check_configuration_checksum():
            boundary = mimetools.choose_boundary()
            b = []
            b.append('--' + boundary)
            b.append('Content-Disposition: form-data; name="uploadType"')
            b.append('')
            b.append('config')
            b.append('--' + boundary + '--')
            b.append('Content-Disposition: form-data; name="restorefilename"; filename="wap4410n.cfg"')
            b.append('Content-Type: application/octet-stream')
            b.append('')
            b.append(self._file_data)
            b.append('--' + boundary + '--')
            b.append('')
            body = '\r\n'.join(b)
            headers = {'Content-Type': 'multipart/form-data; boundary=%s' % boundary,
                       'Cookie': 'LoginPWD=%s; LoginName=%s' % (self._login, self._passwd),
                       'Content-Length': str(len(body))}
            r = urllib2.Request('http://%s%s' % (self._host, '/upload.cgi?next_file=ConfigManagement.htm'), body, headers)
            result = urllib2.urlopen(r).read()
            if result.find('ProgressBar_indeterminate.gif') == -1:
                msg = "Failed to upload configuration"
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
            time.sleep(configuration_timer)

    def _get_wps_setup_page(self):
        """
        Gets the WPS webpage from the the access point.

        :rtype: _ElementStringResult
        :return: The AP's WPS webpage.
        """
        file_data = self._get_ap_page('/WpsSetup.htm')
        return file_data

    def get_wps_pin(self):
        """
        Gets the WPS pin code from the access point webpage

        :rtype: int
        :return: The AP's WPS PIN code.
        """
        # Get AP's webpage
        wps_setup_page = self._get_wps_setup_page()

        parser = WPSPinHTMLParser()
        # Parse the page to get the pin code

        parser.feed(wps_setup_page)

        pin_code = parser.get_wps_pin()

        # Check if we found the pin
        if pin_code is not None:
            self.get_logger().info("Got WPS PIN (%d)" % pin_code)
        else:
            msg = "Failed to get WPS pin (pin not found during parsing)"
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        return pin_code

    def set_wps_pin(self, pin):
        """
        Sets the WPS pin code into the access point

        :type pin: String or int
        :param The AP's WPS PIN code.
        """

        if isinstance(pin, (int, long)):
            pin = str(pin)

        body = 'wps_status=&wl_enrolee_pin=%s&h_wps_cur_status=&todo=set_enrolee_pin&this_file=WpsSetup.htm&next_file=WpsSetup.htm&message=' % pin
        page = '/setup.cgi?next_file=WpsSetup.htm'

        self._get_ap_page(page, body)

    def do_wps_pbc(self):
        """
        Gets the WPS push button code from the access point webpage to simulate a push button.
        """

        body = 'Input.x=57&Input.y=37&wps_status=&wl_enrolee_pin=&h_wps_cur_status=&todo=set_pbc&this_file=WpsSetup.htm&next_file=WpsSetup.htm&message='
        page = '/setup.cgi?next_file=WpsSetup.htm'

        self._get_ap_page(page, body)

    def _get_ap_page(self, page, body=None):
        """
        Downloads a webpage from the the access point.

        :type page: String
        :param page: The page you want to download

        :type body: String
        :param body: The page parameters.

        :rtype: _ElementStringResult
        :return: The AP's asked webpage.
        """
        self.get_logger().debug("Download %s from the equipment" % page)

        # Disable proxy for the AP.
        no_proxy_var = os.getenv("no_proxy", "")
        os.environ['no_proxy'] = no_proxy_var + ',' + self._host

        # Load the login page in order to avoid having a bad configuration downloading
        # it later on.
        # If we don't do that, sometimes, the AP return the login page instead of the
        # configuration file. So we force a fake login to avoid this.
        headers = {'Cookie': 'LoginName=%s; LoginPWD=%s' % (self._login, self._passwd)}
        r = urllib2.Request('http://%s%s' % (self._host, '/login.htm'), None, headers)
        urllib2.urlopen(r).read()

        r = urllib2.Request('http://%s%s' % (self._host, page), body, headers)
        file_data = urllib2.urlopen(r).read()
        return file_data


class WPSPinHTMLParser(HTMLParser):

    """
    This class handles the parsing of the AP page where the WPS PIN is written.
    Its goal is to parse the webpage to set the _wps_pin variable.
    """

    def __init__(self):
        """
        Constructor
        """
        HTMLParser.__init__(self)
        # Initialize the variables
        self._is_wf_msg3_s1_found = False
        self._wps_pin = None

    def handle_data(self, data):
        """
        Handles a data field

        :type data: String
        :param date : Current parser data to analyze.
        """
        if (self._is_wf_msg3_s1_found == False) and ('wf_msg3_s1' in data):
            self._is_wf_msg3_s1_found = True
        elif (self._wps_pin is None) and (str(data).strip().isdigit()):
            # Next data after wf_msg3_s1 is the pin code!
            self._wps_pin = int(data)

    def get_wps_pin(self):
        """
        Gets the WPS pin code parsed from the access point webpage

        :rtype: int
        :return: The AP's WPS PIN code if it worked, None else.
        """
        return self._wps_pin
