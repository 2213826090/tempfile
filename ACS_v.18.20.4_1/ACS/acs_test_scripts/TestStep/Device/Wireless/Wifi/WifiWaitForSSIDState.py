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

:organization: INTEL NDG SW
:summary: This file implements a Test Step for Wifi wait for a SSID state
:since 11/09/2014
:author: jfranchx
"""
import time
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiBase import WifiBase
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import split_and_strip


class WifiWaitForSSIDState(WifiBase):
    """
    Implements wait for SSID state test step for Wifi
    """

    # Constants
    STR_SEPARATOR = ","

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        WifiBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._initial_time = None
        self._wait_for = 1
        self._state_reached = False
        self._all_parameters_checked = False

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        WifiBase.run(self, context)

        # # Get the check sequence (it might just be one single operation)
        state_sequence = split_and_strip(self._pars.state, self.STR_SEPARATOR)

        msg = "All parameters checked :"
        self._initial_time = time.time()
        while not (self._time_out_reached() or self._all_parameters_checked):

            msg = "All parameters checked :"
            self._all_parameters_checked = True
            for current_state in state_sequence:
                if current_state == "CONNECTED":
                    ssids = self._api.list_ssids("wifi", "connected")
                    if self._pars.ssid not in ssids:
                        self._all_parameters_checked = False

                elif current_state == "NOT_CONNECTED":
                    ssids = self._api.list_ssids("wifi", "connected")
                    if self._pars.ssid in ssids:
                        self._all_parameters_checked = False

                elif current_state == "REMEMBERED":
                    ssids = self._api.list_ssids("wifi", "remembered")
                    if self._pars.ssid not in ssids:
                        self._all_parameters_checked = False

                elif current_state == "NOT_REMEMBERED":
                    ssids = self._api.list_ssids("wifi", "remembered")
                    if self._pars.ssid in ssids:
                        self._all_parameters_checked = False

                elif current_state == "VISIBLE":
                    ssids = self._api.list_ssids("wifi", "visible")
                    if self._pars.ssid not in ssids:
                        self._all_parameters_checked = False

                elif current_state == "NOT_VISIBLE":
                    ssids = self._api.list_ssids("wifi", "visible")
                    if self._pars.ssid in ssids:
                        self._all_parameters_checked = False

                elif current_state == "ALL":
                    ssids = self._api.list_ssids("wifi", "all")
                    if self._pars.ssid not in ssids:
                        self._all_parameters_checked = False

                else:
                    msg = "Invalid parameter state : %s" % current_state
                    self._logger.error(msg)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

                # Compile verdict message
                msg += " - %s" % current_state
            time.sleep(self._wait_for)

        if self._time_out_reached() and not self._all_parameters_checked:
            self._raise_device_exception(
                "Expected SSID State fail : %s - Timeout reached after %ss" % (self._pars.state, self._pars.timeout))

        self._logger.debug(msg)
        self.ts_verdict_msg = msg


    def _time_out_reached(self):
        return time.time() - self._initial_time >= self._pars.timeout;
