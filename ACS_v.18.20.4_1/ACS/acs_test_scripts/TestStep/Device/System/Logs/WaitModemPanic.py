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
@summary: This file implements the step to wait for a given time a modem panic.

@author: emarchan

"""
from acs_test_scripts.TestStep.Device.System.Logs.SysDebugBase import SysDebugBase
from lxml import etree
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import TestConst
import time


class WaitModemPanic(SysDebugBase):
    """
    Check if a modem panic occurred by looking at the AP Logs.
    """
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        SysDebugBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.sysdebug_log = None
        self._initial_time = None
        assert any(char.isdigit() for char in self._pars.modem_panic_timeout), \
            "passed value for modem_panic_timeout (%s) is invalid at this stage" % self._pars.modem_panic_timeout
        self._timeout = int(self._pars.modem_panic_timeout)
        self._wait_for = 2
        self._state_reached = False

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        SysDebugBase.run(self, context)

        self._wait_until_timeout_occurs()
        self._logger.debug(self.ts_verdict_msg)

    def _get_sysdebug_logs(self):
        """
        Gets the system debug output
        """
        return self._sysdebug_apis.report_current_data()

    def _wait_until_timeout_occurs(self):
        self._initial_time = time.time()
        panic_occurred = self._has_modem_panic_occurred()
        while panic_occurred is False and not self._time_out_reached():
            time.sleep(self._wait_for)
            panic_occurred = self._has_modem_panic_occurred()

        if panic_occurred is True:
            self.ts_verdict_msg = "VERDICT: A modem panic occurred."
            self._context.set_info(self._pars.modem_panic_occurred, TestConst.STR_TRUE)
        else:
            # Timeout reached
            self.ts_verdict_msg = "VERDICT: No modem panic occurred."
            self._context.set_info(self._pars.modem_panic_occurred, TestConst.STR_FALSE)

    def _time_out_reached(self):
        return time.time() - self._initial_time >= self._timeout;

    def _has_modem_panic_occurred(self):
        result = False
        self.sysdebug_log = self._get_sysdebug_logs()
        modem_panics_logs = None
        for cur_log in self.sysdebug_log.iter():
            if cur_log.tag == "ModemPanics":
                modem_panics_logs = cur_log
                break
        if modem_panics_logs is None:
            self._raise_device_exception(DeviceException.OPERATION_FAILED, "Can't find ModemPanics in the logs, did you enable it at init?")

        nb_modem_panic = len (modem_panics_logs)
        self._logger.debug("WaitModemPanic has detected %d modem panics." % nb_modem_panic)

        if nb_modem_panic > 0:
            result = True
        return result

