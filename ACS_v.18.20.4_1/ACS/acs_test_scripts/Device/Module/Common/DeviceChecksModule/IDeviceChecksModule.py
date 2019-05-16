#!/usr/bin/env python
# -*- coding: utf-8 -*-

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

@organization: INTEL SVE DSV
@summary: Device Checks module Interface for accessing and checking the registers
@since: 12/14/2014
@author: srdubbak
"""
import abc
from ErrorHandling.DeviceException import DeviceException


class IDeviceChecksModule(object):
    """
    DeviceChecksModule interface
    """
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def init(self):
        """
        Initialize nw module

        :rtype: Utilities.Utilities.Global
        :return: Init status
        """
    def read_mmio_reg_32(self, reg_address=None):

        """
        Reads the value in the register address specified.
        :rtype : string
        :return : value in the address
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)


    def get_mmio_reg_address(self, reg_name=None):
        """
        List of Display Registers and its values

        reg_name type: str
        reg_name param value : display register name
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_mipi_errors(self):
        """
            Will check that all bits except for Special Packet Sent interrupt
             and Turn Around Ack Timeout are set to a specific value.

        RETURN
            return_str and reg_val:
            return_str will be either "PASS", "INCONCLUSIVE", "FAIL",
                "Panel is command mode" INCONCLUSIVE is return if mipi is off.

            reg_val will be the value read for this register. If return_str is
            "INCONCLUSIVE", then reg_val is meaningless.

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_hdmi_errors(self):
        """
            Will check the specific bit for buffer underflow.

        RETURN
            return_str and reg_val:
            return_str will be either "PASS", "INCONCLUSIVE", "FAIL",

            reg_val will be the value read for this register. If return_str is
            "INCONCLUSIVE", then reg_val is meaningless.
        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def check_pipe_hang(self):
        """
            This method checks for a pipe hang by checking the register
            This only works for video mode panels.

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)

    def enable_tearing_effect_detection(self):
        """

        This method enables the tearing effect detection.
        This only works for command mode panels.

        RETURN
            "PASS" of TE mean tearing effect detection is enabled, "FAIL" is it is not
            "INCONCLUSIVE, Panel is video mode" is this is a video mode panel.

        """
        raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED)
