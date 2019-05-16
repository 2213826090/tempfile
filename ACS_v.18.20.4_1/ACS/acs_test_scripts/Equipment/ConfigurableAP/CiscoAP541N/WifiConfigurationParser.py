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
:since:03/03/2012
:author: jpstierlin
"""

import urllib2
import mimetools
from xml.dom import minidom
import re
import time


from ErrorHandling.TestEquipmentException import TestEquipmentException


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

        self._radio = None
        self._bss = None
        self._interface = None
        self._dom = None
        self._cookieName = None
        self._cookieValue = None
        self._changed = False

    def get_logger(self):
        """
        Gets the internal logger of the equipment
        """
        return self._logger

    def _gettagvalue(self, node, tag):
        """
        Get the value of tag 'tag' from xml node 'node'

        :type node: xml.dom.Node
        :param node: xml node that contains the tag to get

        :type tag: str
        :param tag: tag name
        """
        return node.getElementsByTagName(tag)[0].childNodes[0].nodeValue

    def _settagvalue(self, node, tag, value):
        """"
        Set the value of tag 'tag' from xml node 'node'

        :type node: xml.dom.Node
        :param node: xml node that contains the tag to set

        :type tag: str
        :param tag: tag name

        :type value: str
        :param value: tag value
        """
        if self._gettagvalue(node, tag) != str(value):
            node.getElementsByTagName(tag)[0].childNodes[0].nodeValue = str(value)
            self._changed = True

    def getradiovalue(self, tag):
        """
        Get the value of tag 'tag' from the radio settings

        :type tag: str
        :param tag: tag name
        """
        return self._gettagvalue(self._radio, tag)

    def setradiovalue(self, tag, value):
        """
        Set the value of tag 'tag' from the radio settings

        :type tag: str
        :param tag: tag name

        :type value: str
        :param value: tag value
        """
        self._settagvalue(self._radio, tag, value)

    def addradiovalue(self, tag, value):
        """
        Set the value of non existing tag 'tag' from the radio settings

        :type tag: str
        :param tag: tag name

        :type value: str
        :param value: tag value
        """
        # add node
        node = self._dom.createElement(tag)
        value = self._dom.createTextNode(value)
        node.appendChild(value)
        self._radio.appendChild(node)
        self._changed = True

    def getbssvalue(self, tag):
        """
        Get the value of tag 'tag' from the bss settings
        """
        return self._gettagvalue(self._bss, tag)

    def setbssvalue(self, tag, value):
        """
        Set the value of tag 'tag' from the bss settings

        :type tag: str
        :param tag: tag name

        :type value: str
        :param value: tag value
        """
        self._settagvalue(self._bss, tag, value)

    def getinterfacevalue(self, tag):
        """
        Get the value of tag 'tag' from the interface settings

        :type tag: str
        :param tag: tag name
        """
        return self._gettagvalue(self._interface, tag)

    def setinterfacevalue(self, tag, value):
        """
        Set the value of tag 'tag' from the interface settings

        :type tag: str
        :param tag: tag name

        :type value: str
        :param value: tag value
        """
        self._settagvalue(self._interface, tag, value)

    def setrates(self, tag, values):
        """
        Set the value of rates 'tag' to the rates list

        :type tag: str
        :param tag: 'basic-rate' or 'supported-rate'

        :type value: list
        :param value: str list of the rates to set
        """
        index = 0
        for node in self._dom.getElementsByTagName(tag):
            if node.hasAttribute('name') and node.attributes['name'].value == 'wlan0':
                if index >= len(values):
                    # delete node
                    node.parentNode.removeChild(node)
                    self._changed = True
                elif node.getElementsByTagName('rate')[0].childNodes[0].nodeValue != values[index]:
                    node.getElementsByTagName('rate')[0].childNodes[0].nodeValue = values[index]
                    self._changed = True
                index += 1
        while index < len(values):
            # add node
            node = self._dom.createElement(tag)
            node.setAttribute('name', 'wlan0')
            rate = self._dom.createElement('rate')
            value = self._dom.createTextNode(values[index])
            rate.appendChild(value)
            node.appendChild(rate)
            self._dom.childNodes[-1].appendChild(node)
            self._changed = True
            index += 1

    def init_configuration(self):
        """
        cleanup previous configurations
        """
        # Enable radio wlan0
        for node in self._dom.getElementsByTagName('radio'):
            if node.hasAttribute('name') and node.attributes['name'].value == 'wlan0':
                self._radio = node
                break

        self.setradiovalue('status', 'up')

        # Enable virtual ssid 0, disable other virtual ssids
        for node in self._dom.getElementsByTagName('bss'):
            if node.hasAttribute('name'):
                value = node.attributes['name'].value
                if value == 'wlan0bssvap0':
                    self._bss = node
                elif value.startswith('wlan0bssvap'):
                    self._settagvalue(node, 'status', 'down')

        self.setbssvalue('status', 'up')
        self.setbssvalue('radio', 'wlan0')
        self.setbssvalue('beacon-interface', 'wlan0')

        # Enable interface wlan0 on first virtual ssids, disable other virtual intefaces
        for node in self._dom.getElementsByTagName('interface'):
            if node.hasAttribute('name'):
                value = node.attributes['name'].value
                if value == 'wlan0':
                    self._interface = node
                elif value.startswith('wlan0'):
                    self._settagvalue(node, 'status', 'down')

        self.setinterfacevalue('status', 'up')
        self.setinterfacevalue('bss', 'wlan0bssvap0')

        # Disable virtual access points
        for node in self._dom.getElementsByTagName('vap'):
            if node.hasAttribute('name'):
                value = node.attributes['name'].value
                if value == 'vap0':
                    self._settagvalue(node, 'status', 'up')
                elif value.startswith('vap'):
                    self._settagvalue(node, 'status', 'down')

    def read_configuration(self, fileName):
        """
        Read configuration from a backup file

        :type fileName: str
        :param fileName: file name
        """
        data = open(fileName, 'rb').read()
        self._dom = minidom.parseString(data[:-1])
        self._changed = False

    def write_configuration(self, fileName):
        """
        Write configuration to a backup file

        :type fileName: str
        :param fileName: file name
        """
        open(fileName, 'wb').write(self._dom.toxml())

    def download_configuration(self):
        """
        Download configuration from the router
        """
        self.login()
        headers = {'Cookie': "%s=%s" % (self._cookieName, self._cookieValue)}
        r = urllib2.Request('http://%s%s' % (self._host, '/config-dump.cgi'), "configBackup=&downloadServerip=", headers)
        data = urllib2.urlopen(r).read()
        self._dom = minidom.parseString(data[:-1])
        self._changed = False

    def upload_configuration(self, configuration_timer):
        """
        Upload configuration to the router

        :type configuration_timer: int
        :param configuration_timer: time to wait for AP to reboot in seconds
        """
        if self._changed:
            boundary = mimetools.choose_boundary()
            b = []
            b.append('--' + boundary)
            b.append('Content-Disposition: form-data; name="%s"; filename="%s"' % ('config-file', 'config.xml'))
            b.append('Content-Type: text/xml')
            b.append('')
            b.append(self._dom.toxml() + '\x00')
            b.append('--' + boundary + '--')
            b.append('')
            body = '\r\n'.join(b)
            headers = {'Content-Type': 'multipart/form-data; boundary=%s' % boundary,
                       'Cookie': "%s=%s" % (self._cookieName, self._cookieValue),
                       'Content-Length': str(len(body))}
            r = urllib2.Request('http://%s%s' % (self._host, '/admin.cgi?action=config_man'), body, headers)
            result = urllib2.urlopen(r).read()
            if result.find('config_restore_success.html') == -1:
                msg = "CiscoAP541N: upload_configuration failed"
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
            self._changed = False
            time.sleep(configuration_timer)

    def login(self):
        """
        Connect to the router administration web interface
        """
        r = urllib2.Request('http://%s%s' % (self._host, '/admin.cgi?action=logon'),
                            'i_username=%s&i_password=%s&login=Log+In' % (self._login, self._passwd))
        data = urllib2.urlopen(r).read()
        m = re.search("var cookieName = \"([A-Za-z]+)\";", data)
        if m is None:
            msg = "CiscoAP541N: Authentication cookie name not found"
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
        self._cookieName = m.groups()[0]
        m = re.search("var cookieValue = \"([A-Za-z]+)\";", data)
        if m is None:
            msg = "CiscoAP541N: Authentication cookie value not found"
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
        self._cookieValue = m.groups()[0]

    def logout(self):
        """
        Disconnect from the router administration web interface
        """
        headers = {'Cookie': "%s=%s" % (self._cookieName, self._cookieValue)}
        urllib2.Request('http://%s%s' % (self._host, '/admin.cgi?action=logout'), None, headers)
