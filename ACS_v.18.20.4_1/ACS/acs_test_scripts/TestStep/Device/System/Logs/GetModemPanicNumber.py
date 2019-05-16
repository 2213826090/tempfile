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
@summary: This file implements the step to check if a modem panic occurred by looking at the AP Logs.

@author: emarchan

"""
from acs_test_scripts.TestStep.Device.System.Logs.SysDebugBase import SysDebugBase
from lxml import etree
from ErrorHandling.DeviceException import DeviceException


class GetModemPanicNumber(SysDebugBase):
    """
    Check if a modem panic occurred by looking at the AP Logs.
    """
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        SysDebugBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.sysdebug_log = None

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        SysDebugBase.run(self, context)
        self.sysdebug_log = self._get_sysdebug_logs()
        modem_panics_logs = None
        for cur_log in self.sysdebug_log.iter():
            if cur_log.tag == "ModemPanics":
                modem_panics_logs = cur_log
                break
        if modem_panics_logs is None:
            self._raise_device_exception(DeviceException.OPERATION_FAILED, "Can't find ModemPanics in the logs, did you enable it at init?")

        for cur_panic in modem_panics_logs:
            self._logger.debug(cur_panic.tag)

        nb_modem_panic = len (modem_panics_logs)

        self.ts_verdict_msg = "VERDICT: %d modem panic found." % nb_modem_panic
        context.set_info(self._pars.modem_panic_count, str(nb_modem_panic))
        self._logger.debug(self.ts_verdict_msg)

    def _get_sysdebug_logs(self):
        """
        Gets the system debug output
        """
        return self._sysdebug_apis.report()