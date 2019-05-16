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
:summary: This file implements a Test Step to Switch ON/OFF the HID mouse or set it pairable
:since:15/07/2013
:author: fbongiax
"""


from acs_test_scripts.TestStep.Equipment.Bluetooth.HidMouseBase import HidMouseBase


class SetHidMouseState(HidMouseBase):
    """
    Implements a test step to power on / off / set discoverable an HID mouse
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory=None):
        """
        Constructor
        """
        HidMouseBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):
        """
        Run method
        """
        HidMouseBase.run(self, context)

        state = self._pars.state.lower() if self._pars.state else None
        if not state:
            self._raise_config_exception("STATE argument not provided")
        elif state == "on":
            self._api.switch_on()
        elif state == "off":
            self._api.switch_off()
        elif state == "pairable":
            self._api.set_discoverable()
        else:
            self._raise_config_exception("STATE argument's value (%s) is invalid" % state)
