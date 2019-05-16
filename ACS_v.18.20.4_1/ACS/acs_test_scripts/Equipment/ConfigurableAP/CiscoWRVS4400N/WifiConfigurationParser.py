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
import struct
import time

from ErrorHandling.TestEquipmentException import TestEquipmentException


class WifiConfigurationParser:

    """
    Parser to read and update the wifi AP configuration file
    """

    def __init__(self, eqt_logger, host):
        """
        Constructor
        """
        # Initialize the variables
        self._logger = eqt_logger
        self._host = host

        self._file_data = None
        self._crc_offset = 0

    def get_logger(self):
        """
        Gets the internal logger of the equipment
        """
        return self._logger

    def get_value(self, item):
        offset = self._file_data.find(item + '=')
        if offset != -1:
            pos = offset + len(item + '=')
            end = self._file_data[pos:].find('\x00')
            if end != -1:
                return self._file_data[pos:pos + end]
        return None

    def set_value(self, item, value):
        offset = self._file_data.find(item + '=')
        if offset != -1:
            pos = offset + len(item + '=')
            end = self._file_data[pos:].find('\x00')
            if end != -1:
                self._file_data = self._file_data[:pos] + str(value) + self._file_data[pos + end:]
                self._crc_offset += len(str(value)) - end
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "item %s not found" % item)

    def __update_configuration_checksum(self):
        """
        Update the wifi configuration checksum
        """
        (self._crc_offset,) = struct.unpack('i', self._file_data[32:36])
        (crc,) = struct.unpack('i', self._file_data[36 + self._crc_offset:36 + self._crc_offset + 4])
        computed_crc = binascii.crc32(self._file_data[36:36 + self._crc_offset])
        if computed_crc != crc:
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR,
                                "crc mismatch. found: %08x calc: %08x" % (crc, computed_crc))

    def __check_configuration_checksum(self):
        """
        Check if config has changed and return False if not
        """
        computed_crc = binascii.crc32(self._file_data[36:36 + self._crc_offset])
        (crc,) = struct.unpack('i', self._file_data[36 + self._crc_offset:36 + self._crc_offset + 4])
        if computed_crc != crc:
            self._file_data = self._file_data[:32]\
                + struct.pack('i', self._crc_offset)\
                + self._file_data[36:36 + self._crc_offset]\
                + struct.pack('i', computed_crc)\
                + self._file_data[40 + self._crc_offset:]
            return True
        return False

    def download_configuration(self):
        """
        Download configuration from the router
        """
        self.get_logger().debug("Download configuration from the equipment")

        r = urllib2.Request('http://%s%s' % (self._host, '/setup.cgi?next_file=Routercfg.cfg&todo=backup_config'))
        self._file_data = urllib2.urlopen(r).read()
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
            l = []
            l.append('--' + boundary)
            l.append('Content-Disposition: form-data; name="%s"; filename="%s"' % ('restoreFilename', 'Routercfg.cfg'))
            l.append('Content-Type: application/octet-stream')
            l.append('')
            l.append(self._file_data)
            l.append('--' + boundary + '--')
            l.append('')
            body = '\r\n'.join(l)
            headers = {'Content-Type': 'multipart/form-data; boundary=%s' % boundary,
                       'Content-Length': str(len(body))}
            r = urllib2.Request('http://%s%s' % (self._host, '/restore_config.cgi'), body, headers)
            result = urllib2.urlopen(r).read()
            if result.find('reboot_guage.htm') == -1:
                msg = "Upload_configuration failed"
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
            time.sleep(configuration_timer)
