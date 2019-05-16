"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG
:summary: This file implements the step to get the current wifi softAP used channel when using LTE.
:since: 2015-01-09
:author: emarchan

"""

from acs_test_scripts.TestStep.Device.System.Logs.SysDebugBase import SysDebugBase
from lxml import etree
from ErrorHandling.DeviceException import DeviceException
from time import sleep


class WifiGetHotspotChannel(SysDebugBase):
    """
    Gets the safe channels when using LTE.
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        SysDebugBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._result = None
        self._timeout = 30
        self._delay_between_checks = 1

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        SysDebugBase.run(self, context)

        self.ts_verdict_msg = "VERDICT: Can't find the Wi-Fi SoftAP channel in logs."

        while self._result == None and self._timeout > 0:
            sysdebug_log = self._get_sysdebug_logs()
            for cur_log in sysdebug_log.iter():
                if cur_log.tag == "WifiSoftApChannel":
                    value = cur_log.get('value')
                    if value is not None:
                        self._result = int(value)
                    break

            if self._result == None:
                self._logger.debug("Can't find the Wi-Fi SoftAP channel in logs (%d tries remaining)." % (self._timeout))
                self._timeout = self._timeout - 1
                sleep(self._delay_between_checks)

            else:
                self.ts_verdict_msg = "VERDICT: Wi-Fi SoftAP channel is %d." % (self._result)
                break
        if self._result == None:
            self._raise_device_exception(self.ts_verdict_msg)
        self._logger.debug(self.ts_verdict_msg)
        context.set_info(self._pars.save_wifi_hostpot_channel, self._result)

    def _get_sysdebug_logs(self):
        """
        Gets the system debug output
        """
        return self._sysdebug_apis.report_current_data()
