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
:summary: This file implements Test Step for get the time of transfer of Opp transfer based on aplogs
:since 11/03/2015
:author: jfranchx
"""
import time
import re

from acs_test_scripts.Utilities.CommunicationUtilities import ConfigsParser
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsToolException import AcsToolException


class BtOppPiGetLogsThroughput(DeviceTestStepBase):
    """
    Implements Get time of opp transfer with logs class
    """
    # Coefficient is based on a transfer throughput of 500Kbps (8 /500000)
    DEFAULT_TIMEOUT_COEFF = 0.000016

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
            timeout = float(float(self._pars.opp_file_size) * self.DEFAULT_TIMEOUT_COEFF)
        else:
            timeout = float(self._pars.timeout)
        self._logger.debug("Timeout is set to : %s" % timeout)

        if self._pars.opp_direction == "UPLOAD":
            opp_regex_start = r'BtOppObexClient.*OBEX\ session\ created'
            opp_regex_end = r'BtOppObexClient.*Stop'
            logcat_pattern = "BtOppObexClient"
        else:
            opp_regex_start = r'BtOppObexServer.*Server\ unblocked'
            opp_regex_end = r'BtOppObexServer.*Stop'
            logcat_pattern = "BtOppObexServer"

        # Get the pointer on the list of lines that matches the trigger
        parsed_logcat = self._device.get_device_logger().get_message_triggered_status(logcat_pattern)

        opp_time_start = None
        opp_time_stop = None
        end_time = time.time() + timeout
        while time.time() < end_time and (opp_time_start is None or opp_time_stop is None):
            if len(parsed_logcat) == 0:
                # No new line has been retrieved from logcat, wait for a while
                time.sleep(0.5)
                continue

            log_line = parsed_logcat[0]
            if opp_time_start is None:
                if re.search(opp_regex_start, log_line) is not None:
                    regex_time_start = re.findall(r'([0-2][0-9]:[0-5][0-9]:[0-5][0-9]\.[0-9][0-9][0-9])', log_line)
                    regex_time_start = regex_time_start[0]
                    x = time.strptime(regex_time_start.split('.')[0], '%H:%M:%S')
                    opp_time_start = ((x.tm_hour * 60 + x.tm_min) * 60 + x.tm_sec) * 1000 + int(regex_time_start.split('.')[1])

            if opp_time_stop is None:
                if re.search(opp_regex_end, log_line) is not None:
                    regex_time_start = re.findall(r'([0-2][0-9]:[0-5][0-9]:[0-5][0-9]\.[0-9][0-9][0-9])', log_line)
                    regex_time_start = regex_time_start[0]
                    x = time.strptime(regex_time_start.split('.')[0], '%H:%M:%S')
                    opp_time_stop = ((x.tm_hour * 60 + x.tm_min) * 60 + x.tm_sec) * 1000 + int(regex_time_start.split('.')[1])

            # Remove the 1st line of the list
            del parsed_logcat[0]

        if opp_time_start is None or opp_time_stop is None:
            msg = "Can't find start or stop time of Opp transfer"
            self._logger.error(msg)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)

        result_time = float(opp_time_stop - opp_time_start)/1000
        # Compute throughput in Kbps
        result_throughput = ((float(self._pars.opp_file_size) * 8)/result_time)/1000

        # Save result in context
        context.set_info(self._pars.save_opp_throughput, result_throughput)
        self.ts_verdict_msg = "VERDICT: %s stored as {0}".format(result_throughput) % self._pars.save_opp_throughput
        self._logger.debug(self.ts_verdict_msg)
