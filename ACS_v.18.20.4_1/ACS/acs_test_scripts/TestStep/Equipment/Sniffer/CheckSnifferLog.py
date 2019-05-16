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
:summary: This file implements a Test Step for make some check in sniffer log
:since 09/01/2015
:author: jfranchx
"""

import os
import re
from Core.TestStep.TestStepBase import TestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.TestEquipmentException import TestEquipmentException


class CheckSnifferLog(TestStepBase):
    """
    Implements Sniffer check class
    """

    CHECK_FRAME_BEACON = "CHECK_FRAME_BEACON"
    CHECK_FRAME_ASSOCIATION_REQUEST = "CHECK_FRAME_ASSOCIATION_REQUEST"
    CHECK_BANDWIDTH = "CHECK_BANDWIDTH"
    CHECK_CAPABILITIES = "CHECK_CAPABILITIES"

    PARAM_BANDWIDTH_20MHZ = "20MHZ"
    PARAM_BANDWIDTH_40MHZ = "40MHZ"
    PARAM_BANDWIDTH_80MHZ = "80MHZ"
    PARAM_CAPABILITY_HT = "HT_CAPABILITIES"
    PARAM_CAPABILITY_VHT = "VHT_CAPABILITIES"

    CAPABILITY_HT = "HT Capabilities"
    CAPABILITY_VHT = "VHT Capabilities"

    def run(self, context):
        """
        Run test step
        """
        TestStepBase.run(self, context)

        # Possibilities : check Beacon frame, Association Request Frame
        # Want to check : HT/VHT Capabilities, 20/40/80MHz bandwidths, BlockACK

        if self._pars.frame_to_check == self.CHECK_FRAME_BEACON:
            frame_to_check = "IEEE 802.11 Beacon"
        elif self._pars.frame_to_check == self.CHECK_FRAME_ASSOCIATION_REQUEST:
            frame_to_check = "IEEE 802.11 Association Request"
        else:
            msg = "FRAME_TO_CHECK is not valid %s" % self._pars.frame_to_check
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        if self._pars.param_to_check == self.CHECK_CAPABILITIES:
            if self._pars.param_value == self.PARAM_CAPABILITY_HT:
                param_value = self.CAPABILITY_HT
            elif self._pars.param_value == self.PARAM_CAPABILITY_VHT:
                param_value = self.CAPABILITY_VHT
            else:
                msg = "PARAM_VALUE is not valid : %s" % self._pars.param_value
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        elif self._pars.param_to_check == self.CHECK_BANDWIDTH:
            param_value = self._pars.param_value
        else:
            msg = "PARAM_TO_CHECK is not valid %s" % self._pars.param_to_check
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        is_frame = False
        is_valid_frame = False
        is_valid_mac_addr_source = False
        is_valid_mac_addr_receiver = False

        check_mac_addr_source = False
        check_mac_addr_receiver = False
        if self._pars.mac_addr_source is not None:
            check_mac_addr_source = True
        if self._pars.mac_addr_receiver is not None:
            check_mac_addr_receiver = True

        is_bandwidth_capability = False
        ts_final_result = False


        # Open sniffer log file
        if not os.path.isfile(self._pars.sniffer_log_file):
            msg = "Sniffer log file does not exist"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
        sniff_file = open(self._pars.sniffer_log_file)

        self._logger.debug("Start sniffer log parsing")
        try:
            # Read the file and stop if check is found
            for nextline in sniff_file:
                # Check if we are out of a frame
                if nextline == "\n":
                    is_frame = False
                    is_valid_frame = False
                    is_valid_mac_addr_source = False
                    is_valid_mac_addr_receiver = False
                    is_bandwidth_capability = False

                # Check frame
                if is_frame is False:
                    if nextline.startswith("Frame"):
                        is_frame = True
                    continue

                # Check frame must be checked
                if is_valid_frame is False:
                    if frame_to_check in nextline:
                        is_valid_frame = True
                    continue

                # Check WiFi MAC addresses if required
                if is_valid_mac_addr_source is False and check_mac_addr_source:
                    source_mac_addr = re.findall(r'Source address: [a-zA-Z0-9_:]* [(]([a-fA-F0-9:-]{17})[)]', nextline)
                    if len(source_mac_addr) > 0:
                        if source_mac_addr[0].lower() == self._pars.mac_addr_source.lower():
                            is_valid_mac_addr_source = True
                        continue
                if is_valid_mac_addr_receiver is False and check_mac_addr_receiver:
                    receive_mac_addr = re.findall(r'Receiver address: [a-zA-Z0-9_:]* [(]([a-fA-F0-9:-]{17})[)]',
                                                  nextline)
                    if len(receive_mac_addr) > 0:
                        if receive_mac_addr[0].lower() == self._pars.mac_addr_receiver.lower():
                            is_valid_mac_addr_receiver = True
                        continue

                if (check_mac_addr_source and is_valid_mac_addr_source is False) or (
                        check_mac_addr_receiver and is_valid_mac_addr_receiver is False):
                    # MAC addresses are not checked, read next line
                    continue

                # Check frame params
                if self._pars.param_to_check == self.CHECK_CAPABILITIES:
                    if self._check_capability(param_value, nextline):
                        ts_final_result = True
                elif self._pars.param_to_check == self.CHECK_BANDWIDTH:
                    if is_bandwidth_capability is False:
                        if param_value == self.PARAM_BANDWIDTH_80MHZ:
                            bandwidth_capability = self.CAPABILITY_VHT
                        else:
                            bandwidth_capability = self.CAPABILITY_HT
                        is_bandwidth_capability = self._check_capability(bandwidth_capability, nextline)
                        continue

                    if self._check_bandwidth(param_value, nextline):
                        ts_final_result = True

                # Stop the loop if result is ok
                if ts_final_result:
                    break

        finally:
            sniff_file.close()
            self._logger.debug("Stop sniffer log parsing")

        if ts_final_result is not True:
            msg = "CHECK SNIFFER LOG FAIL - Unable to check parameters : %s %s %s" % (self._pars.frame_to_check,
                                                                                      self._pars.param_to_check,
                                                                                      self._pars.param_value)
            if check_mac_addr_source:
                msg += " - with %s as source" % self._pars.mac_addr_source
            if check_mac_addr_receiver:
                msg += " - with %s as receiver" % self._pars.mac_addr_receiver
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)

    def _check_capability(self, capability, current_line):
        """
        Function to check a capability in a frame.
        Execute this function when you are sure the frame is OK.

        :type capability: str
        :param capability: type of capability to check
        :type current_line: str
        :param current_line: line of sniffer log to analyze

        :rtype: boolean
        :return: if capability is ok or not
        """
        if capability == self.CAPABILITY_VHT:
            if "Tag: VHT Capabilities" in current_line and "802.11ac" in current_line:
                self._logger.debug("VHT Capabilities checked")
                return True
        elif capability == self.CAPABILITY_HT:
            if "Tag: HT Capabilities (802.11n D1.10)" in current_line:
                self._logger.debug("HT Capabilities checked")
                return True

        return False

    def _check_bandwidth(self, bandwidth, current_line):
        """
        Function to check a bandwidth in a frame.
        Execute this function when you are sure the frame is OK.

        :type bandwidth: str
        :param bandwidth: value of bandwidth to check
        :type current_line: str
        :param current_line: line of sniffer log to analyze

        :rtype: boolean
        :return: if bandwidth is ok or not
        """
        if bandwidth == self.PARAM_BANDWIDTH_80MHZ:
            if "Channel Width: 80 MHz (0x01)" in current_line:
                self._logger.debug("Bandwidth 80MHz checked")
                return True
        else:
            if "= HT Support channel width: " in current_line:
                result_rgx = re.findall(r'\.\.([01])\.', current_line)
                if len(result_rgx) > 0:
                    if result_rgx[0] == '0' and bandwidth == self.PARAM_BANDWIDTH_20MHZ:
                        self._logger.debug("Bandwidth 20MHz checked")
                        return True
                    if result_rgx[0] == '1' and bandwidth == self.PARAM_BANDWIDTH_40MHZ:
                        self._logger.debug("Bandwidth 40MHz checked")
                        return True
        return False
