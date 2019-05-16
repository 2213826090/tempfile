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
:summary: virtual interface with configurable Access Point
:since:25/05/2012
:author: jpstierlin
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class ISniffer(object):

    """
    Virtual interface for WiFi Sniffer Equipment
    """

    def init(self):
        """
        Initializes the equipment and establishes the connection.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Releases equipment resources and close connection.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def start_capture(self, channel, local_output_file, save_sniff_in_ascii=False, dut_mac_addr="", ssid=""):
        """
        Start WiFi capture on the equipment
        You have to call stop_capture(...) after start_capture(...)

        :type channel: int
        :param channel: WiFi channel number to sniff

        :type local_output_file: str
        :param local_output_file: pathname of the output capture file

        :type save_sniff_in_ascii: boolean
        :param save_sniff_in_ascii: enable ascii format for sniff logs

        :type dut_mac_addr: str
        :param dut_mac_addr: mac address to filter on

        :type ssid: str
        :param ssid: ssid address to filter on
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def stop_capture(self, donot_save_log):
        """
        Stop WiFi capture on the equipment, and place the capture file
        in the current _Report directory

        :type donot_save_log: Boolean
        :param donot_save_log: If True, don't save the capture file
                             to local self._local_out path
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def start_wifi_scan_monitor(self, channel, dut_mac_addr):
        """
        Start WiFi capture in order to monitor DUT wifi scan period
        You have to call stop_wifi_scan_monitor(...)
        after start_wifi_scan_monitor(...)

        :type channel: int
        :param channel: WiFi channel number to sniff

        :type dut_mac_addr: str
        :param dut_mac_addr: DUT WiFi mac address
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def stop_wifi_scan_monitor(self, log_filename):
        """
        Stop WiFi capture on the equipment and retrieve scan dates array

        :type log_filename: str
        :param log_filename: pathname of the output capture file

        :rtype: list of float
        :return: list of date in seconds when a WiFi scans occur.
                The origin of these dates is the sniffer start time stamp.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_capture_file(self, log_file_name):
        """
        get the capture file and save it
        in the current _Report directory without stopping the capture
        :type log_filename: str
        :param log_filename: pathname of the output capture file
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
