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

:organization: INTEL OTC Android
:summary: Implement specific functionalities of the NokiaBH111 equipment
:since: 4/10/15
:author: mmaraci
"""
from acs_test_scripts.Equipment.BTHeadset.NokiaBH214.NokiaBH214 import NokiaBH214


class NokiaBH111(NokiaBH214):

    """
    Class that implements the Nokia BH111 equipment
    """

    def set_volume(self, state):
        """
        For this equipment, in order to perform a volume change, you must first press
        the opposite volume button for 100ms and then the button for the volume change
        that you need for another 100ms.
        :param state: true for volume UP, false for volume down
        :return:
        """


        self._raise_error_if_any_is_none([self._volupbutton, self._voldownbutton, self._defaultshortkeypresstimer],
                            "volUpButton/volDownButton/defaultShortKeyPressTimer not configured; No action taken!")

        if state:
            #turn volume UP
            self._press_relay(self._voldownbutton, 0.1)
            self._press_relay(self._volupbutton, self._defaultshortkeypresstimer)
        else:
            #turn volume DOWN
            self._press_relay(self._volupbutton, 0.1)
            self._press_relay(self._voldownbutton, self._defaultshortkeypresstimer)