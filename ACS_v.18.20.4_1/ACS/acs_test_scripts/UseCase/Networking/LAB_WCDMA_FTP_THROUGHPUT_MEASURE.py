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
:summary: WCDMA FTP UC with data throughput measurement via network simulator
:author: hbian
:since:09/12/2012
"""

import time
import os
import re

from LAB_WCDMA_BASE import LabWcdmaBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabWcdmaFtpThroughputMeasure(LabWcdmaBase):

    """
    Lab Wcdma ftp test with data throughput measure.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_WCDMA_BASE Init function
        LabWcdmaBase.__init__(self, tc_name, global_config)

        # Read the ftp file name from UseCase xml Parameter
        if self._direction == "DL":
            self._ftp_filename = os.path.join(
                self._ftp_path,
                self._tc_parameters.get_param_value("DL_FILENAME"))
        elif self._direction == "UL":
            # Read the UL_FILE value from UseCase xml Parameter
            self._ftp_filename = os.path.join(
                self._ftp_path,
                self._tc_parameters.get_param_value("UL_FILENAME"))

        # Read the MEASUREMENT_DURATION from UseCase xml Parameter
        self._measurement_duration = self._tc_parameters.\
            get_param_value("MEASUREMENT_DURATION")
        if self._measurement_duration is not None and \
                str(self._measurement_duration).isdigit():
            self._measurement_duration = int(self._measurement_duration)
        else:
            self._measurement_duration = None

        # Read the XFER_TIMEOUT from UseCase xml Parameter
        self._xfer_timeout = self._tc_parameters.get_param_value("XFER_TIMEOUT")
        if self._xfer_timeout is not None and str(self._xfer_timeout).isdigit():
            self._xfer_timeout = int(self._xfer_timeout)
        else:
            self._xfer_timeout = None

        self._failure_throughput_dict = {}

        # Read the FAILURE_OTATX from UseCase xml Parameter
        failure_targets = self._tc_parameters.get_param_value("FAILURE_TARGETS").\
            replace(" ", "").split(",")

        for failure in failure_targets:
            self._failure_throughput_dict.update(
                {str(failure.split(":")[0]): float(failure.split(":")[1])})

        self._wait_time_before_measure = 5
        self._device_logger = None
        self._start_ftp_trigger_msg = ""
        self._ftp_task_id = None
        self._op_code_message = ""
        self._op_code = None
        self._timeout = None

    def set_up(self):
        """
        Initialize the test
        """

        LabWcdmaBase.set_up(self)

        if self._direction not in ["UL", "DL"]:
            self._error.Msg = "%s is not a known xfer direction" % \
                self._direction
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   self._error.Msg)

        if self._measurement_duration is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "MEASUREMENT_DURATION should be int")

        if self._xfer_timeout is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "XFER_TIMEOUT should be int")

        if (self._measurement_duration + self._wait_time_before_measure)\
                > self._xfer_timeout:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "XFER_TIMEOUT should be always greater than"
                                   "MEASUREMENT_DURATION+%d (measurement start after"
                                   " %d seconds ftp starts) "
                                   % (self._wait_time_before_measure, self._wait_time_before_measure))

        return Global.SUCCESS, self._error.Msg

    def run_test(self):
        """
        Execute the test
        """
        # Call LAB_HSPA_BASE Run function
        LabWcdmaBase.run_test(self)

        result = Global.SUCCESS
        measure_throughput_msg = "Measured throughput:"
        failure_throughput_msg = "Failure throughput:"

        # Run FTP transfer using FTP parameters :
        # - LAB_SERVER parameters (ip, username, password)
        # - DIRECTION
        # - DL_FILE or UL_FILE
        # - XFER_TIMEOUT
        time.sleep(self._wait_btwn_cmd)

        # Start to search the trigger_msg in the logcat log
        self._device_logger = self._device.get_device_logger()
        self._start_ftp_trigger_msg = "status::SUCCESS - output::RECEIVED - " \
            "function::startFtpXfer"
        self._device_logger.add_trigger_message(self._start_ftp_trigger_msg)

        self._logger.info("FTP transfer " + str(self._direction) +
                          " for " + str(self._ftp_filename) + "...")

        self._ftp_task_id = self._networking_api.start_ftp_xfer(
            self._direction,
            self._server_ip_address,
            self._username,
            self._password,
            self._ftp_filename,
            self._device.get_ftpdir_path()
        )

        # Get the ftp start log msg and retrieve the OP code from the message
        self._op_code_message = self._device_logger.\
            get_message_triggered_status(self._start_ftp_trigger_msg)
        self._device_logger.remove_trigger_message(self._start_ftp_trigger_msg)
        self._op_code = re.findall(r'ACS\_\d*', str(self._op_code_message))[0]

        # wait 5 seconds for ensure that transfer is established
        self._logger.info(
            "Wait %d seconds for ensure that transfer is established" %
            self._wait_time_before_measure)

        time.sleep(self._wait_time_before_measure)

        data_throughput_dict = self._ns.get_data_throughput(
            self._measurement_duration,
            self._failure_throughput_dict.keys())

        # Timeout to wait for the ftp success message
        self._timeout = self._xfer_timeout - self._measurement_duration\
            - self._wait_time_before_measure
        self._device_logger.add_trigger_message(self._op_code)

        # Create success_msg which send by ACS_AGENT.apk
        ftp_filename = os.path.split(self._ftp_filename)[1]
        success_msg = "Ftp %s of file %s finish success" % (self._direction,
                                                            ftp_filename)

        # Parse the logcat log, Wait for the success_msg
        while self._timeout > 0:
            triggered_status = self._device_logger.\
                get_message_triggered_status(self._op_code)
            # When the  success message  is in the message of ACS op code
            if success_msg in str(triggered_status):
                self._logger.info(str(triggered_status))
                self._device_logger.remove_trigger_message(self._op_code)
                break
            else:
                time.sleep(1)
                self._timeout -= 1
        else:
            self._device_logger.remove_trigger_message(self._op_code)
            self._networking_api.stop_ftp_xfer(self._ftp_task_id)
            self._error.Msg = "The FTP transfer doesn't finish in the timeout"
            raise DeviceException(DeviceException.TIMEOUT_REACHED, self._error.Msg)

        for data_type in data_throughput_dict.keys():
            if "Mbps" in data_throughput_dict[data_type][0]:
                # Convert to kbps for result compare
                if float(data_throughput_dict[data_type][0][0:-4]) * 1000 < \
                        self._failure_throughput_dict[data_type]:
                    result = Global.FAILURE
            elif "Kbps" in data_throughput_dict[data_type][0]:
                if float(data_throughput_dict[data_type][0][0:-4]) < \
                        self._failure_throughput_dict[data_type]:
                    result = Global.FAILURE
            elif "bps" in data_throughput_dict[data_type][0]:
                if float(data_throughput_dict[data_type][0][0:-4]) / 1000 < \
                        self._failure_throughput_dict[data_type]:
                    result = Global.FAILURE
            else:
                self._error.Msg = "unknown measure result unit: %s " % \
                    str(data_throughput_dict[data_type][0])
                raise DeviceException(DeviceException.PROHIBITIVE_MEASURE,
                                       self._error.Msg)

            measure_throughput_msg = measure_throughput_msg + str(data_type)\
                + ":" + str(data_throughput_dict[data_type][0]) + " "
            failure_throughput_msg = failure_throughput_msg + str(data_type)\
                + ":" + str(self._failure_throughput_dict[data_type]) + "Kbps" + " "

        self._error.Msg = measure_throughput_msg + "- " + failure_throughput_msg

        return result, self._error.Msg
