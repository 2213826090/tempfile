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
:summary: Common class to all Sniffer Equipments
:since:19/06/2013
:author: apairex
"""

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.Sniffer.Interface.ISniffer import ISniffer
from ErrorHandling.TestEquipmentException import TestEquipmentException


class GenericSniffer(EquipmentBase, ISniffer):

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        """
        # Initialize class parent
        ISniffer.__init__(self)
        EquipmentBase.__init__(self, name, model, eqt_params)

        computer = str(bench_params.get_param_value("Computer"))
        # NOTE: import here to avoid circular dependency on
        # EquipmentManager if imported at top level
        from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager
        self._em = EquipmentManager()
        self._computer = self._em.get_computer(computer)

        self._ssh_process = None
        self._ssh_queue = None
        self._local_out = ""
        self._sniff_ongoing = False
        self._wifi_interface = bench_params.get_param_value("Interface", "")
        if self._wifi_interface == "":
            self._wifi_interface = self._computer.get_wifi_interface()

        self._sniffer_tool_checked = False
        self._sniffer_cmd = "NOT_YET_DEFINED"

    def init(self):
        """
        Initializes the equipment and establishes the connection.
        """
        self.get_logger().info("init")

        # Open connection to sniffer
        self._computer.init(True)

        # Check Sniffer binary tool existence
        self._check_sniffer_tool(self._sniffer_cmd)

        # End all potential ongoing sniffer traces
        self._computer.run_cmd("killall %s" % self._sniffer_cmd)
        self._sniff_ongoing = False

    def release(self):
        """
        Releases equipment resources and close connection.
        """
        self.get_logger().info("release")
        self._computer.release()

    def get_capture_file(self, log_file_name):
        """
        get the capture file and save it
        in the current _Report directory without stopping the capture
        :type log_filename: str
        :param log_filename: pathname of the output capture file
        """
        self._computer.copy_file_in_local_path(self._remote_out, log_file_name)

    def _check_sniffer_tool(self, binary_to_check):
        """
        Check that binary_to_check command is installed on the host computer

        :type binary_to_check: str
        :param binary_to_check: Binary cmd to check if available on the Computer
        """
        if not self._sniffer_tool_checked:

            if self._computer.get_model() == "LOCAL_COMPUTER" \
                    and self._computer.get_os() == self._computer.WINDOWS:
                msg = "Sniffer must be installed on a LOCAL or REMOTE Linux computer"
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED, msg)

            out = self._computer.run_cmd("which %s" % binary_to_check)
            if ("/%s" % binary_to_check) not in out["std"]:
                msg = "%s is not available on %s" % (binary_to_check, self._computer.get_name())
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED, msg)

            self._sniffer_tool_checked = True

    def _build_sniffer_cmd(self, save_sniff_in_ascii=False, dut_mac_addr="", ssid="", filter_option="-R"):
        """
        Build the sniff command

        :type save_sniff_in_ascii: boolean
        :param save_sniff_in_ascii: enable ascii format for sniff logs

        :type dut_mac_addr: str
        :param dut_mac_addr: mac address to filter on

        :type ssid: str
        :param ssid: ssid address to filter on

        :type filter: str
        :param filter: special option for filter (-R or -Y)
        """
        sniff = "%s -i%s -s0 " % (self._sniffer_cmd, self._wifi_interface)

        # Build the sniff_filter
        if dut_mac_addr or ssid:
            sniff_filter = ""
            if dut_mac_addr:
                sniff_filter += 'wlan.addr==%s' % dut_mac_addr
            if dut_mac_addr and ssid:
                sniff_filter += " || "
            if ssid:
                sniff_filter += '(wlan.fc.type_subtype==0x08 && wlan_mgt.ssid=="%s")' % ssid
            sniff += '%s "%s" ' % (filter_option, sniff_filter)

        if save_sniff_in_ascii:
            sniff += '-V > %s' % self._remote_out
        else:
            sniff += "-w%s" % self._remote_out

        return sniff
