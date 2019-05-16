"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL OTC SSG
:summary: This file implements a Test Step to Click one of the buttons on an HOGP
 mouse
:since:23/07/2015
:author: mmaraci
"""

import time
from acs_test_scripts.TestStep.Equipment.Bluetooth.HogpMouseBase import HogpMouseBase


class HogpMouseClick(HogpMouseBase):
    """
    Implements HOGP mouse click
    """
    STR_BTN_SINGLE = "single"

    def __init__(self, tc_conf, global_conf, ts_conf, factory=None):
        """
        Constructor
        """
        HogpMouseBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def run(self, context):
        """
        Run method
        """
        HogpMouseBase.run(self, context)

        self._assert_args_syntax_is_correct()

        buttons = self._pars.buttons.split(",")

        for button in buttons:
            assert button, "At this point BUTTONS syntax should be correct as checked by the " \
                           "framework"
            self._click(button.strip().lower())
            time.sleep(int(self._pars.interval_secs))

    def _assert_args_syntax_is_correct(self):
        """
        TestStep Framework is supposed to check the arguments
        """
        assert self._pars.buttons, "At this point BUTTONS syntax should be correct as checked by " \
                                   "the framework"
        assert self._pars.interval_secs is not None and isinstance(self._pars.interval_secs, float), \
        "At this point INTERVAL_SECS syntax should be correct as checked by the framework"

    def _click(self, button):
        """
        Commands a button click
        """
        assert button in [self.STR_BTN_SINGLE], \
            "BUTTON possible values should have been checked by the framework and be corrected at" \
            " this point"

        self._logger.info("Commanding %s click" % button)

        self._api.perform_click()