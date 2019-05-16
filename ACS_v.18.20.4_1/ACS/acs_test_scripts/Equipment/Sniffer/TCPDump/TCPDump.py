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
:summary: Implementation of TCPDump Sniffer Equipment
:since:25/05/2012
:author: jpstierlin
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.Sniffer.Common.Common import GenericSniffer


class TCPDump(GenericSniffer):

    """
    Implementation of TCPDump Sniffer Equipment
    """

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        """
        GenericSniffer.__init__(self, name, model, eqt_params, bench_params)

        self._remote_out = "/tmp/tcpdump.cap"
        self._sniffer_cmd = "tcpdump"

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
        if self._sniff_ongoing:
            msg = "Cannot start 2 captures in the same time"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        self._local_out = local_output_file

        self.get_logger().info(
            "start_capture on interface:%s, channel:%d, output file:%s" %
            (self._wifi_interface, int(channel), local_output_file))

        if "ra" not in self._wifi_interface:
            # With ralink interface, the mode monitor should be activated
            # while the interface is UP
            self._computer.run_cmd("ifconfig %s down" % self._wifi_interface)

        self._computer.run_cmd("iwconfig %s mode monitor" % self._wifi_interface)
        self._computer.run_cmd("iwconfig %s channel %d" % (self._wifi_interface, int(channel)))

        if "ra" not in self._wifi_interface:
            self._computer.run_cmd("ifconfig %s up" % self._wifi_interface)

        self._computer.run_cmd("rm -f %s" % self._remote_out)

        # Build the sniff command
        sniff = self._build_sniffer_cmd(save_sniff_in_ascii, dut_mac_addr, ssid)

        out = self._computer.run_cmd("nohup " + sniff, 5)

        if "You don't have permission to capture on that device" in out["err"]:
            msg = "Sniffer trace does not start. Ensure you have root privileges."
            self._logger.warning(msg)
        else:
            self._sniff_ongoing = True

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

        self._computer.run_cmd("killall tcpdump")
        self._sniff_ongoing = False

        if not donot_save_log:
            self._computer.copy_file_in_local_path(self._remote_out, self._local_out)
        # remove temp capture file on sniffer
        self._computer.run_cmd("rm -f %s" % self._remote_out)
