"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:summary: This file implements Test Step for Compute WiFi reconnection time based on aplogs
:since 04/12/2014
:author: jfranchx
"""
import time
import re

from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Utilities.NetworkingUtilities import SupplicantState
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.AcsToolException import AcsToolException


class ComputeWifiReconnectionTimeWithLogs(DeviceTestStepBase):
    """
    Implements Compute Wifi Reconnection time with logs class
    """

    DEFAULT_TIMEOUT = 120

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._networking_api = self._device.get_uecmd("Networking")

    #------------------------------------------------------------------------------

    def run(self, context):
        """
        Run the step
        """
        DeviceTestStepBase.run(self, context)

        if self._pars.timeout is None:
            timeout = self.DEFAULT_TIMEOUT
        else:
            timeout = int(self._pars.timeout)

        self._networking_api.start_wifi_connection_log()

        # Get the pointer on the list of lines that matches the trigger
        pattern = "regex:" + SupplicantState.STATE_LOGCAT_FILTER
        msg_triggered = self._networking_api._device_logger.get_message_triggered_status(pattern)
        compute_time_start = -1
        compute_time_complete = -1

        # Instantiate the SupplicantState object that will store the latest
        # supplicant state read from the logcat
        state = SupplicantState()

        # Control the line content
        start_time = time.time()
        while start_time + timeout > time.time():
            # Check if supplicant state has changed within a reasonable time
            if state.get_state_always_undef() and start_time + (timeout / 5) < time.time():
                # If not, we can consider that the connection failed
                msg = "No supplicant state changes. Network should be out of range"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            if len(msg_triggered) == 0:
                # No new line has been retrieved from logcat, wait for a while
                time.sleep(0.5)
                continue

            # Process the next line
            log_line = msg_triggered[0]
            self._logger.debug("logcat line: " + str(log_line))

            # Scan the supplicant state from the log line
            state_read = str(SupplicantState(log_line))
            state_previous = state.get_state()

            # Update the supplicant state machine only if the
            # state read is in the next possible states
            if state.set_state(state_read):
                self._logger.debug("Supplicant state set: " + str(state))

                if state_previous == SupplicantState.DISCONNECTED and state.get_state() == SupplicantState.SCANNING and compute_time_start == -1:
                    compute_time_start = self.__compute_time(log_line)
                elif state_previous == SupplicantState.GROUP_HANDSHAKE and state.get_state() == SupplicantState.COMPLETED and compute_time_start != -1:
                    compute_time_complete = self.__compute_time(log_line) - compute_time_start

                if state.is_connection_success():
                    if compute_time_complete == -1:
                        msg = "Error computing time of connection"
                        self._logger.error(msg)
                        raise DeviceException(DeviceException.OPERATION_FAILED, msg)
                    # Connection success
                    msg = "Connection SUCCESS in %ss" % compute_time_complete
                    self._logger.info(msg)
                    break

                if state.is_connection_failure():
                    # All tries fail
                    msg = "MAX connection tries reached (%d)" \
                          % SupplicantState.MAX_CONNECTION_TRIES
                    msg += " -> FAILURE"
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

            # Remove the 1st line of the list
            del msg_triggered[0]

        # Clean LogCatParser
        self._networking_api._device_logger.remove_trigger_message(pattern)

        if compute_time_complete != -1:
            # We have the value, let's save it into the context
            context.set_info(self._pars.save_wifi_reconnection_time, compute_time_complete)
            self.ts_verdict_msg = "VERDICT: %s stored as {0}".format(compute_time_complete) % self._pars.save_wifi_reconnection_time
            self._logger.debug(self.ts_verdict_msg)
            return

        # If we scanned at least 1 connection failure then we return a failure
        if state.is_connection_failed_once():
            msg = "Connection failed at least once."
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # No significant connection status has been parsed from the logcat
        msg = "__check_logs_and_compute_time() timeout"
        self._logger.error(msg)
        raise DeviceException(DeviceException.OPERATION_FAILED, msg)

    def __compute_time(self, log_line):
        """
        Compute time from logcat line to float.

        :type   log_line: string
        :param  log_line: log_line with time

        :rtype: float
        :return : reconnection duration
        """
        try:
            time_search = re.search(r'[0-2][0-9]:[0-5][0-9]:[0-5][0-9].[0-9]+', log_line)
            hours, minutes, seconds = str(time_search.group(0)).split(':')
            result = float(hours) * 3600 + float(minutes) * 60 + float(seconds)
        except ValueError:
            msg = "Error __compute_time in ComputeWifiReconnectionTimeWithLogs class"
            self._logger.error(msg)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        return result
