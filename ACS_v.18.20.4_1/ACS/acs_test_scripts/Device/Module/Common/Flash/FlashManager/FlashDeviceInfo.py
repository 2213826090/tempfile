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
:summary: This file implements the Flash device information passed to Flash tool
:since: 09/07/2013
:author: lbavois
"""
from UtilitiesFWK.AttributeDict import AttributeDict


class FlashDeviceInfo(AttributeDict):

    """
    This class collects all device information needed to flash a device with a flash tool
    """

    def __init__(self, serial_number=None, soc_serial_number=None, pos_serial_number=None, change_device_state_fct=None):
        """
        Constructor:
        Allow to pass information to flash tool libraries

        :type serial_number: str
        :param serial_number: device serial number
        :type soc_serial_number: str
        :param soc_serial_number: SOC serial numbe
        :type pos_serial_number: str
        :param pos_serial_number: device pos serial numberr
        :type change_device_state_fct: str
        :param change_device_state_fct: method allowing to trig device state changes
        """
        # pylint: disable=super-on-old-class
        super(FlashDeviceInfo, self).__init__()

        self.device_pos_serial_number = pos_serial_number
        self.device_serial_number = serial_number
        self.device_soc_serial_number = soc_serial_number
        self.change_device_state_fct = change_device_state_fct
