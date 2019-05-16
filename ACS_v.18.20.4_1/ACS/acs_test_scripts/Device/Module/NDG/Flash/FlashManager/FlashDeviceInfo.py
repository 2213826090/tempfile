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

:organization: INTEL NDG SW DEV
:summary: This file implements the Flash device information passed to Flash tool
:since: 09/07/2013
:author: jreynaux
"""
from UtilitiesFWK.AttributeDict import AttributeDict


class FlashDeviceInfo(AttributeDict):

    """
    This class collects all device information needed to flash a device with a flash tool
    """

    def __init__(self, serial_number=None, board_type=None, change_device_state_fct=None, software_release=None):
        """
        Constructor:
        Allow to pass information to flash tool libraries

        :type serial_number: str
        :param serial_number: device serial number
        :type board_type: str
        :param board_type: type of device
        :type change_device_state_fct: str
        :param change_device_state_fct: method allowing to trig device state changes
        """
        # pylint: disable=super-on-old-class
        super(FlashDeviceInfo, self).__init__()

        self.device_serial_number = serial_number
        self.board_type = board_type
        self.change_device_state_fct = change_device_state_fct
        self.software_release = software_release