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
:summary: Implementation of WireShark Sniffer Equipment
:since:25/05/2012
:author: jpstierlin
"""

import os
import re

from acs_test_scripts.Equipment.Sniffer.Common.Common import GenericSniffer
from ErrorHandling.TestEquipmentException import TestEquipmentException


class WireShark(GenericSniffer):

    """
    Implementation of WireShark Sniffer Equipment (using tshark package)
    """

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        """
        GenericSniffer.__init__(self, name, model, eqt_params, bench_params)

        self._remote_out = "/tmp/tshark.cap"
        self._sniffer_cmd = "tshark"

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
        self._local_out = local_output_file

        tshark_version = self._computer.run_cmd("tshark -v")
        filter_options = self.__get_filter_option(tshark_version)

        # Build the sniff command
        sniff = self._build_sniffer_cmd(save_sniff_in_ascii, dut_mac_addr, ssid, filter_options)

        self.__start_tshark_capture(channel, sniff)

    def stop_capture(self, donot_save_log):
        """
        Stop WiFi capture on the equipment, and place the capture file
        in the current _Report directory

        :type donot_save_log: Boolean
        :param donot_save_log: If True, don't save the capture file
                             to local self._local_out path
        """
        # Ensure that the connection to the Computer is active
        self._computer.init()

        self._computer.run_cmd("killall tshark")
        self._sniff_ongoing = False

        if not donot_save_log:
            self._computer.copy_file_in_local_path(self._remote_out, self._local_out)
        # remove temp capture file on sniffer
        self._computer.run_cmd("rm -f %s" % self._remote_out)

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
        # Sniff with self._wifi_interface hardware interface, flushing std output
        sniff = 'tshark -l -i%s ' % self._wifi_interface
        # Sniff filter on DUT MAC address
        sniff += '-R \'wlan.addr==%s ' % dut_mac_addr
        # filter probe request only
        sniff += '&& wlan.fc.type_subtype==0x04 '
        # filter without specific SSID
        sniff += '&& wlan_mgt.ssid==""\' broadcast > %s &' % self._remote_out
        self.__start_tshark_capture(channel, sniff)

    def stop_wifi_scan_monitor(self, log_filename):
        """
        Stop WiFi capture on the equipment and retrieve scan dates array

        :type log_filename: str
        :param log_filename: pathname of the output capture file

        :rtype: list of floats
        :return: list of date in seconds when a WiFi scan occurs.
                The origin of these dates is the sniffer start time stamp.
        """
        self._logger.debug("stop wifi scan monitor - filename: %s" % log_filename)

        # Ensure that the connection to the Computer is active
        self._computer.init()

        # Stop the sniff and retrieve the log file
        self._computer.run_cmd("killall tshark")
        self._sniff_ongoing = False

        attempt = 3
        while not os.path.isfile(log_filename) and attempt > 0:
            self._computer.copy_file_in_local_path(self._remote_out, log_filename)
            attempt -= 1
        # Check log file existence
        if not os.path.isfile(log_filename):
            msg = "Unable to retrieve sniffer log file: %s" % log_filename
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        # remove temp capture file on sniffer
        self._computer.run_cmd("rm -f %s" % self._remote_out)

        # Parse the log file and build the date list
        filed = open(log_filename)
        try:
            nextline = filed.readline()
            datelist = list()
            self.get_logger().debug("-- WiFi sniff log file --")
            while nextline != "":
                self.get_logger().debug(nextline[:-1])
                res = re.match(r'^[ ]*([0-9]+\.[0-9]+)', nextline)
                if res:
                    datelist.append(float(res.group(1)))
                nextline = filed.readline()
            self.get_logger().debug("-- END OF WiFi sniff log file --")
        finally:
            filed.close()

        return datelist

    def __start_tshark_capture(self, channel, sniff_cmd):
        """
        Start WiFi capture in order to monitor DUT wifi scan period

        :type channel: int
        :param channel: WiFi channel number to sniff
        :type sniff_cmd: str
        :param sniff_cmd: Sniff command to use
        """
        if self._sniff_ongoing:
            msg = "Cannot start 2 captures in the same time"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        self.get_logger().info("start_capture on interface:%s, channel:%d" %
                               (self._wifi_interface, int(channel)))

        if "ra" not in self._wifi_interface:
            # With ralink interface, the mode monitor should be activated
            # while the interface is UP
            self._computer.run_cmd("ifconfig %s down" % self._wifi_interface)

        self._computer.run_cmd("iwconfig %s mode monitor" % self._wifi_interface)
        self._computer.run_cmd("iwconfig %s channel %d" % (self._wifi_interface, int(channel)))

        if "ra" not in self._wifi_interface:
            self._computer.run_cmd("ifconfig %s up" % self._wifi_interface)

            # In some cases, channel configuration should be done with wlan interface UP
            self._computer.run_cmd("iwconfig %s channel %d" % (self._wifi_interface, int(channel)))

        self._computer.run_cmd("rm -f %s" % self._remote_out)
        out = self._computer.run_cmd("nohup " + sniff_cmd)

        if "The capture session could not be initiated" in out["err"]:
            msg = "Sniffer trace does not start. Ensure you have root privileges."
            self._logger.warning(msg)
        else:
            self._sniff_ongoing = True

    def __get_filter_option(self, tshark_version):
        """
        Return filter options aligned with tshark version

        :type tshark_version: str
        :param tshark_version: result of 'tshark -v' command

        :rtype: str
        :return: parameter to use as filter option
        """
        def __normalize_version(version):
            return [int(x) for x in re.sub(r'(\.0+)*$', '', version).split(".")]

        if "TShark" in tshark_version['std']:
            versions = re.findall(r'TShark[ (a-zA-Z) ]*([0-9].[0-9]+)', tshark_version['std'])
            if versions is not None:
                msg = "TShark version found : %s" % versions[0]

                # Options changes with new versions of tshark
                if cmp(__normalize_version(versions[0]), __normalize_version("1.8")) > 0:
                    msg += " - Version greater than 1.8, use -Y"
                    options = "-Y"
                else:
                    msg += " - Version lower than 1.8, use -R"
                    options = "-R"

                self._logger.debug(msg)
                return options

        self._logger.debug("TShark version can't be found, use default parameter -R")
        return "-R"
