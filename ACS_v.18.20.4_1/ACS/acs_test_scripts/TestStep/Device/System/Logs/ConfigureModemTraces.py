"""

@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

@organization: UMG PSI Validation
@summary: This file implements the step to enable modem logs.

@author: emarchan

"""
from acs_test_scripts.TestStep.Device.Wireless.Cellular.CellularBase import CellularBase

class ConfigureModemTraces(CellularBase):
    """
    Check if a modem panic occurred by looking at the AP Logs.
    """
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        CellularBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._hsi_speed = None
        self._traces_level = None
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        CellularBase.run(self, context)

        self._hsi_speed = self._pars.modem_hsi_speed
        assert self._hsi_speed in ["NO", "DEF", "HIGH"], \
            "passed value for modem_hsi_speed (%s) is invalid at this stage" % self._hsi_speed

        assert any(char.isdigit() for char in self._pars.modem_traces_level), \
            "passed value for modem_traces_level (%s) is invalid at this stage" % self._pars.modem_traces_level


        if self._hsi_speed == "NO":
            self._hsi_speed = "u"
        elif self._hsi_speed == "DEF":
            self._hsi_speed = "d"
        else:
            self._hsi_speed = "h"
        self._traces_level = int(self._pars.modem_traces_level)

        self._modem_api.configure_modem_trace(self._hsi_speed, self._traces_level)

