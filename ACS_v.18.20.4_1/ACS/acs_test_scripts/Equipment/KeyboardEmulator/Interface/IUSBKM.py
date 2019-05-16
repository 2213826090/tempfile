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

:organization: INTEL PCCG CI
:summary: virtual interface with USBKM Keyboard emulator
simulators
:since: 05/12/2013
:author: Sivarama Krishna Vellanki
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException


class IUSBKM(object):

    """
    IUSBKM class: virtual interface with usb to serial keyboard emulators.
    """
    COM_PORT_ID_PARAM = "ComPort"

    def init(self):
        """
        Initializes the equipment. Final state: the equipment is ready to use.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def write_keys(self, keys):
        """
        Write keys to usbkm using a set of predefined keys

        :type keys: list or str
        :param keys: alphanumeric str or list of commands to be sent
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Release usbkm device
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
