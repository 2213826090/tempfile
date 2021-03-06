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

:organization: INTEL MCG PSI
:summary: This file implements the DeviceCapabilityManager class used by the device
:since: 12/08/2014
:author: kturban
"""
import os
from Core.PathManager import Paths
from Device.DeviceCapability.DeviceCapability import DeviceCapability
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Parsers.Parser import Parser
from UtilitiesFWK.FakeClass import FakeClass


class DeviceCapabilityManager(object):

    """
    Manage device capability
    """

    @staticmethod
    def is_capable(capability, device_capabilities):
        """
        This method check if the requested capabilities is inside a capability list
        The requested capabilities can be :
            - a capability name
            - a list of capability name separated by 'or' keyword. This method will implement the 'or' logical
              method will return true if at least of capability is in the capability list
              example: 'cap1 or cap2 or cap3'
            - a list of capability name separated by 'and' keyword. This method will implement the 'and' logical
              method will return true if all capabilities are in the capability list
              example: 'cap1 and cap2 and cap3'

        :param capability: a request capability
        :type capability: string

        :param device_capabilities: list of capability
        :type device_capabilities: list

        :return a boolean
        """
        is_able = False
        # retrieve device capability as a string (equal to the capability name)
        capability_list = [str(cap) for cap in device_capabilities]

        possible_capabilities_or = capability.split(" or ")
        possible_capabilities_and = capability.split(" and ")
        is_or_logical = len(possible_capabilities_or) > 1
        is_and_logical = len(possible_capabilities_and) > 1

        if is_or_logical and is_and_logical:
            raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED,
                                  "cannot compute 'and' and 'or' logical words in the same sentence. "
                                  "Please use multiple decorator lines")
        elif not is_or_logical and not is_and_logical:
            # no or/and into the request
            is_able = capability in capability_list
        else:
            if is_or_logical:
                is_able = any([x for x in possible_capabilities_or if x in capability_list])
            elif is_and_logical:
                is_able = all([False for x in possible_capabilities_and if x not in capability_list])
        return is_able

    @staticmethod
    def load_device_capabilities(capability_filename, logger=None):
        """
        Load all device capabilities from capability file

        :param capability: list of capability
        :type capability: list

        :param capability: list of capability
        :type capability: list

        :rtype: list
        :return: list of capabilities for this device
        """
        if not logger:
            logger = FakeClass()
        device_capabilities = []
        device_capabilities_file = str(capability_filename)

        deviceCapability_shema_file = os.path.join(Paths.FWK_DEVICE_CAPABILITIES_CATALOG, "DeviceCapabilities.xsd")

        if not device_capabilities_file:
            logger.warning("Missing deviceCapabilities parameter value in device configuration")
        else:
            # Try to load it from _ExecutionConfig folder
            device_capabilities_path = os.path.join(Paths().EXECUTION_CONFIG, device_capabilities_file)
            if not os.path.isfile(device_capabilities_path):
                # if not found, try to load it from _Catalogs folder
                device_capabilities_path = os.path.join(Paths().DEVICE_CATALOG, device_capabilities_file)
            if os.path.isfile(device_capabilities_path):
                device_cap_parser = Parser().get_parser(device_capabilities_path, deviceCapability_shema_file)
                logger.debug("Device capabilities : ")
                capabilities = device_cap_parser.get("/DeviceCapabilities/*")
                if not capabilities:
                    logger.debug("- no device capability")
                else:
                    for capability_node in capabilities:
                        # Get the teststep Id
                        capability = capability_node.text
                        logger.debug("- capability : {0}".format(capability))
                        device_capabilities.append(DeviceCapability({'name': capability}))
            else:
                logger.warning("{0} not found, device will have no capabilities".format(device_capabilities_file))
        return device_capabilities

if __name__ == "__main__":
    print DeviceCapabilityManager.is_capable("cap1 or cap2", ["cap1", "cap2"])
    print DeviceCapabilityManager.is_capable("cap1 and cap2", ["cap1", "cap2"])
    print DeviceCapabilityManager.is_capable("cap1 and cap3", ["cap1", "cap2"])
