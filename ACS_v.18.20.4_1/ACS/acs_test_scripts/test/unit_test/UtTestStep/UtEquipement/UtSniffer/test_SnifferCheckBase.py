# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:summary: Unit test module
:since: 28/08/14
:author: jfranchx
"""

import os
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase


class SnifferCheckBaseTest(UTestTestStepBase):

    LINE_NULL = "\n"
    LINE_FRAME = "Frame 7: 241 bytes on wire (1928 bits), 241 bytes captured (1928 bits) on interface 0\n"
    LINE_BEACON_FRAME = "IEEE 802.11 Beacon frame, Flags: ........C\n"
    LINE_ASSOCIATION_REQUEST_FRAME = "IEEE 802.11 Association Request, Flags: ........C\n"
    LINE_HT_CAPABILITIES = "Tag: HT Capabilities (802.11n D1.10)\n"
    LINE_HT_CAPABILITIES_20MHZ = "HT Capabilities Info: 0x000c\n"
    LINE_HT_CAPABILITIES_40MHZ = "HT Capabilities Info: 0x000e\n"
    LINE_VHT_CAPABILITIES = "Tag: VHT Capabilities (IEEE Stc 802.11ac/D3.1)\n"
    LINE_VHT_OPERATIONS = "Tag: VHT Operation (IEEE Stc 802.11ac/D3.1)\n"
    LINE_20MHZ = ".... .... .... ..0. = HT Support channel width: Transmitter supports 20MHz and 40MHz operation\n"
    LINE_40MHZ = ".... .... .... ..1. = HT Support channel width: Transmitter supports 20MHz and 40MHz operation\n"
    LINE_80MHZ = "Channel Width: 80 MHz (0x01)\n"
    LINE_MAC_SOURCE = "Source address: Company_44:55:66 (11:22:33:44:55:66)\n"
    LINE_MAC_RECEIVER = "Receiver address: Company_DD:EE:FF (AA:BB:CC:DD:EE:FF)\n"

    FILE_NAME_20MHZ = "sniffer_log_20mhz"
    FILE_NAME_40MHZ = "sniffer_log_40mhz"
    FILE_NAME_80MHZ = "sniffer_log_80mhz"
    FILE_NAME_20MHZ_DUT_FAIL = "sniffer_log_20mhz_dut_fail"
    FILE_NAME_40MHZ_DUT_FAIL = "sniffer_log_40mhz_dut_fail"
    FILE_NAME_80MHZ_DUT_FAIL = "sniffer_log_80mhz_dut_fail"
    FILE_NAME_20MHZ_WITH_CISCO = "sniffer_log_20mhz_with_cisco"
    FILE_NAME_NULL = "sniffer_log_null"

    AP_CISCO = "CISCO_1250"
    AP_ASUS = "ASUS_AP"

    BANDWIDTH_PARAMETER_20MHZ = "20MHZ"
    BANDWIDTH_PARAMETER_40MHZ = "40MHZ"
    BANDWIDTH_PARAMETER_80MHZ = "80MHZ"
    CAPABILITIES_PARAMETER_HT = "HT_CAPABILITIES"
    CAPABILITIES_PARAMETER_VHT = "VHT_CAPABILITIES"

    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._sut = None

    def tearDown(self):
        UTestTestStepBase.tearDown(self)

    def _create_sniffer_log_file_20mhz(self):
        # Create a sniffer log file
        current_file = open(self.FILE_NAME_20MHZ, "a+")
        current_file.write(self.LINE_NULL)
        current_file.write(self.LINE_FRAME)
        current_file.write(self.LINE_BEACON_FRAME)
        current_file.write(self.LINE_MAC_RECEIVER)
        current_file.write(self.LINE_MAC_SOURCE)
        current_file.write(self.LINE_HT_CAPABILITIES)
        current_file.write(self.LINE_HT_CAPABILITIES_20MHZ)
        current_file.write(self.LINE_20MHZ)
        current_file.write(self.LINE_NULL)
        current_file.write(self.LINE_FRAME)
        current_file.write(self.LINE_ASSOCIATION_REQUEST_FRAME)
        current_file.write(self.LINE_MAC_RECEIVER)
        current_file.write(self.LINE_MAC_SOURCE)
        current_file.write(self.LINE_HT_CAPABILITIES)
        current_file.write(self.LINE_HT_CAPABILITIES_20MHZ)
        current_file.write(self.LINE_20MHZ)
        current_file.write(self.LINE_NULL)
        current_file.close()

    def _create_sniffer_log_file_40mhz(self):
        # Create a sniffer log file
        current_file = open(self.FILE_NAME_40MHZ, "a+")
        current_file.write(self.LINE_NULL)
        current_file.write(self.LINE_FRAME)
        current_file.write(self.LINE_BEACON_FRAME)
        current_file.write(self.LINE_MAC_RECEIVER)
        current_file.write(self.LINE_MAC_SOURCE)
        current_file.write(self.LINE_HT_CAPABILITIES)
        current_file.write(self.LINE_HT_CAPABILITIES_40MHZ)
        current_file.write(self.LINE_40MHZ)
        current_file.write(self.LINE_NULL)
        current_file.write(self.LINE_FRAME)
        current_file.write(self.LINE_ASSOCIATION_REQUEST_FRAME)
        current_file.write(self.LINE_MAC_RECEIVER)
        current_file.write(self.LINE_MAC_SOURCE)
        current_file.write(self.LINE_HT_CAPABILITIES)
        current_file.write(self.LINE_HT_CAPABILITIES_40MHZ)
        current_file.write(self.LINE_40MHZ)
        current_file.write(self.LINE_NULL)
        current_file.close()

    def _create_sniffer_log_file_80mhz(self):
        # Create a sniffer log file
        current_file = open(self.FILE_NAME_80MHZ, "a+")
        current_file.write(self.LINE_NULL)
        current_file.write(self.LINE_FRAME)
        current_file.write(self.LINE_BEACON_FRAME)
        current_file.write(self.LINE_MAC_RECEIVER)
        current_file.write(self.LINE_MAC_SOURCE)
        current_file.write(self.LINE_HT_CAPABILITIES)
        current_file.write(self.LINE_HT_CAPABILITIES_40MHZ)
        current_file.write(self.LINE_VHT_CAPABILITIES)
        current_file.write(self.LINE_VHT_OPERATIONS)
        current_file.write(self.LINE_80MHZ)
        current_file.write(self.LINE_NULL)
        current_file.write(self.LINE_FRAME)
        current_file.write(self.LINE_ASSOCIATION_REQUEST_FRAME)
        current_file.write(self.LINE_MAC_RECEIVER)
        current_file.write(self.LINE_MAC_SOURCE)
        current_file.write(self.LINE_HT_CAPABILITIES)
        current_file.write(self.LINE_HT_CAPABILITIES_40MHZ)
        current_file.write(self.LINE_VHT_CAPABILITIES)
        current_file.write(self.LINE_VHT_OPERATIONS)
        current_file.write(self.LINE_80MHZ)
        current_file.write(self.LINE_NULL)
        current_file.close()

    def _create_sniffer_log_file_20mhz_dut_fail(self):
        # Create a sniffer log file
        current_file = open(self.FILE_NAME_20MHZ_DUT_FAIL, "a+")
        current_file.write(self.LINE_NULL)
        current_file.write(self.LINE_FRAME)
        current_file.write(self.LINE_BEACON_FRAME)
        current_file.write(self.LINE_HT_CAPABILITIES)
        current_file.write(self.LINE_HT_CAPABILITIES_20MHZ)
        current_file.write(self.LINE_NULL)
        current_file.close()

    def _create_sniffer_log_file_40mhz_dut_fail(self):
        # Create a sniffer log file
        current_file = open(self.FILE_NAME_40MHZ_DUT_FAIL, "a+")
        current_file.write(self.LINE_NULL)
        current_file.write(self.LINE_FRAME)
        current_file.write(self.LINE_BEACON_FRAME)
        current_file.write(self.LINE_HT_CAPABILITIES)
        current_file.write(self.LINE_HT_CAPABILITIES_40MHZ)
        current_file.write(self.LINE_NULL)
        current_file.close()

    def _create_sniffer_log_file_80mhz_dut_fail(self):
        # Create a sniffer log file
        current_file = open(self.FILE_NAME_80MHZ_DUT_FAIL, "a+")
        current_file.write(self.LINE_NULL)
        current_file.write(self.LINE_FRAME)
        current_file.write(self.LINE_BEACON_FRAME)
        current_file.write(self.LINE_HT_CAPABILITIES)
        current_file.write(self.LINE_HT_CAPABILITIES_40MHZ)
        current_file.write(self.LINE_VHT_CAPABILITIES)
        current_file.write(self.LINE_VHT_OPERATIONS)
        current_file.write(self.LINE_80MHZ)
        current_file.write(self.LINE_NULL)
        current_file.close()

    def _create_sniffer_log_file_20mhz_with_cisco(self):
        # Create a sniffer log file
        current_file = open(self.FILE_NAME_20MHZ_WITH_CISCO, "a+")
        current_file.write(self.LINE_NULL)
        current_file.write(self.LINE_FRAME)
        current_file.write(self.LINE_ASSOCIATION_REQUEST_FRAME)
        current_file.write(self.LINE_HT_CAPABILITIES)
        current_file.write(self.LINE_HT_CAPABILITIES_20MHZ)
        current_file.write(self.LINE_NULL)
        current_file.close()

    def _create_sniffer_log_file_null(self):
        # Create a sniffer log file
        current_file = open(self.FILE_NAME_NULL, "a+")
        current_file.write(self.LINE_NULL)
        current_file.write(self.LINE_NULL)
        current_file.close()

    def _delete_sniffer_log_file(self, name):
        os.remove(name)