"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: interface for controlling USB hub
:author: cbonnard
:since: 13/01/2014
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IUSBHub(object):

    """
    Class USBHub: virtual interface for controlling USB hub
    """
    PRODUCT_ID_PARAM = "ProductId"
    VENDOR_ID_PARAM = "VendorId"
    USB_BUS_PARAM = "USBBus"
    USB_HOST_DUT_PORT_PARAM = "HostDUTPortNumber"

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def enable_port(self, port_number):
        """
        activate selected port

        :rtype: bool
        :return: status of operation
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def disable_port(self, port_number):
        """
        deactivate selected port

        :rtype: bool
        :return: status of operation
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def enable_port_usb_host_device(self):
        """
        activate port that controls host-dut usb line

        :rtype: bool
        :return: status of operation
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def disable_port_usb_host_device(self):
        """
        deactivate port that controls host-dut usb line

        :rtype: bool
        :return: status of operation
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
            Release resources if any
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)