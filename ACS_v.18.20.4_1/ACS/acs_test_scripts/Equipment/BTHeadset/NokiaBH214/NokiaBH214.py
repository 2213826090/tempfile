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
:summary: Nokia BH214 implementation
:since: 19/02/2013
:author: cmichelx
"""

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.BTHeadset.Interface.IBTHeadset import IBTHeadset
from acs_test_scripts.Equipment.BTHeadset.Common.Common import GenericBTHeadset
from ErrorHandling.TestEquipmentException import TestEquipmentException
import time


class NokiaBH214(GenericBTHeadset):

    """
    Class that implements NokiaBH214 equipment
    """

    def set_power(self, state=True):
        """
        Set BT headset power ON or OFF
        Control the on/off line
        :type state: boolean
        :param state: true to power ON, false to power OFF
        """
        self._raise_error_if_any_is_none([self._powerbutton, self._powerontimer, self._powerofftimer],
                                          "powerButton/powerOnTimer/powerOffTimer not configured; No action taken!")
        if state:
            self._press_relay(self._powerbutton, self._powerontimer)
        else:
            self._press_relay(self._powerbutton, self._powerofftimer)

    def set_discoverable(self):
        """
        Set BT headset in discoverable mode (able to be paired)
        Control the on/off line to switch to pairing mode
        """
        self._raise_error_if_any_is_none([self._powerbutton, self._pairingtimer],
                                          "powerButton/pairingTimer not configured; No action taken!")
        self._press_relay(self._powerbutton, self._pairingtimer)

