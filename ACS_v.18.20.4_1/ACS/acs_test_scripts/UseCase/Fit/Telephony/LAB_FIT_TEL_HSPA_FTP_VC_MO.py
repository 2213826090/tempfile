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
:summary:  This file implements usecase that do a ftp transfer and a voice call
on HSPA Network
:since: 14/11/2012
:author: Hbianx
"""


import time
import os
import re
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.Networking.LAB_HSPA_BASE import LabHspaBase
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.TestEquipmentException import TestEquipmentException


class LabFitTelHspaFtpVcMo(LabHspaBase):

    """
    Lab Hspa network Ftp and voice call at the same time
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        self._ftp_data_transfer_state_timeout = 10

        # Call LabMobilityBase Init function
        LabHspaBase.__init__(self, tc_name, global_config)

        # Set ps_data_ergch_information_state to OFF
        self._ps_data_ergch_information_state = "OFF"

        # Read agilent network from testcase xml Parameter
        self._rbt_channel_type = \
            self._tc_parameters.get_param_value("RBT_CHANNEL_TYPE")

        if self._cqi_scheme == "FIXED":
            self._cqi = int(self._tc_parameters.get_param_value("CQI"))

        # Read the DIRECTION value from UseCase xml Parameter
        self._ftp_direction = self._tc_parameters.get_param_value("DIRECTION")

        # Read the DL_FILE value from UseCase xml Parameter
        # ToCheck:Actually, this code works with Linux FTP server
        # But need to check if os.path.join work with a Windows FTp server
        # Maybe need a os.sep
        if self._ftp_direction == "DL":
            self._ftp_filename = os.path.join(
                self._ftp_path,
                self._tc_parameters.get_param_value("DL_FILENAME"))
        elif self._ftp_direction == "UL":
            # Read the UL_FILE value from UseCase xml Parameter
            self._ftp_filename = os.path.join(
                self._ftp_path,
                self._tc_parameters.get_param_value("UL_FILENAME"))

        # Read the MEASUREMENT_DURATION from UseCase xml Parameter
        self._measurement_duration = self._tc_parameters.\
            get_param_value("MEASUREMENT_DURATION")
        if isinstance(self._measurement_duration, str) and \
                self._measurement_duration.isdigit():
            self._measurement_duration = int(self._measurement_duration)
        else:
            self._measurement_duration = None

        # Read the XFER_TIMEOUT from UseCase xml Parameter
        self._xfer_timeout = self._tc_parameters.get_param_value("XFER_TIMEOUT")
        if isinstance(self._xfer_timeout, str) and self._xfer_timeout.isdigit():
            self._xfer_timeout = int(self._xfer_timeout)
        else:
            self._xfer_timeout = None

        # Read the FAILURE_OTATX from UseCase xml Parameter
        self._failure_throughput_dict = {}

        failure_targets = self._tc_parameters.get_param_value("FAILURE_TARGETS").\
            replace(" ", "").split(",")

        for failure in failure_targets:
            self._failure_throughput_dict.update(
                {str(failure.split(":")[0]): float(failure.split(":")[1])})

        self._wait_time_before_measure = 5

        # Read PHONE_NUMBER from testcase xml parameters
        if (self._tc_parameters.get_param_value("PHONE_NUMBER") not in (None, '')) \
                and str(self._tc_parameters.get_param_value("PHONE_NUMBER")).isdigit():
            self._phone_number = self._tc_parameters.get_param_value("PHONE_NUMBER")
        else:
            self._phone_number = str(self._device.get_phone_number())

        self._call_duration = \
            self._tc_parameters.get_param_value("CALL_DURATION")
        if isinstance(self._call_duration, str) and self._call_duration.isdigit():
            self._call_duration = int(self._call_duration)
        else:
            self._call_duration = None

        self._call_setup_timeout = \
            int(self._dut_config.get("callSetupTimeout"))

        self._voicecall_api = self._device.get_uecmd("VoiceCall")

        # Create cellular network simulator and retrieve 3G API
        self._ns_cell_3g = self._ns.get_cell_3g()
        self._ns_voice_call_3g = self._ns_cell_3g.get_voice_call()

        # Get UECmdLayer for VoiceCall
        self._voicecall_api = self._device.get_uecmd("VoiceCall")

    def set_up(self):
        """
        Initialize the test
        """

        LabHspaBase.set_up(self)

        if self._ftp_direction not in ["UL", "DL"]:
            self._error.Msg = "%s is not a known xfer direction" % self._ftp_direction
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, self._error.Msg)

        if self._measurement_duration is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "MEASUREMENT_DURATION should be int")

        if self._xfer_timeout is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "XFER_TIMEOUT should be int")

        if self._call_duration is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "CALL_DURATION should be int")

        if (self._measurement_duration + self._wait_time_before_measure) > self._call_duration:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "call_duration should be always greater than"
                                     "MEASUREMENT_DURATION+%d (measurement start after %d seconds ftp starts)"
                                     % (self._wait_time_before_measure, self._wait_time_before_measure))

        return Global.SUCCESS, self._error.Msg

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call LabHspaBase Run function
        LabHspaBase.run_test(self)

        result = Global.SUCCESS
        measure_throughput_msg = "Measured throughput:"
        failure_throughput_msg = "Failure throughput:"

        # Start to search the trigger_msg in the logcat log
        self._device_logger = self._device.get_device_logger()
        self._start_ftp_trigger_msg = "status::SUCCESS - output::RECEIVED - function::startFtpXfer"
        self._device_logger.add_trigger_message(self._start_ftp_trigger_msg)

        self._logger.info("FTP transfer " + str(self._ftp_direction) + " for " + str(self._ftp_filename) + "...")

        self._ftp_task_id = self._networking_api.start_ftp_xfer(
            self._ftp_direction,
            self._server_ip_address,
            self._username,
            self._password,
            self._ftp_filename,
            self._device.get_ftpdir_path()
        )

        # Get the ftp start log msg and retrieve the OP code from the message
        self._op_code_message = self._device_logger.get_message_triggered_status(self._start_ftp_trigger_msg)
        self._device_logger.remove_trigger_message(self._start_ftp_trigger_msg)
        self._op_code = re.findall(r'ACS\_\d*', str(self._op_code_message))[0]

        self._logger.info("Wait %d seconds for ensure that transfer is established" % self._wait_time_before_measure)
        time.sleep(self._wait_time_before_measure)

        self._voicecall_api.dial(self._phone_number)

        # Check call state "CONNECTED" before callSetupTimeout seconds
        self._ns_voice_call_3g.check_call_connected(self._call_setup_timeout)

        self._logger.info("Wait %d seconds for ensure that transfer is stable" % self._wait_time_before_measure)
        time.sleep(self._wait_time_before_measure)

        data_throughput_dict = self._ns.get_data_throughput(
            self._measurement_duration,
            self._failure_throughput_dict.keys())

        # Check call is connected for CALL_DURATION seconds
        self._ns_voice_call_3g.is_voice_call_connected(self._call_duration)

        # Release the voice call
        self._ns_voice_call_3g.voice_call_network_release()

        self._timeout = self._xfer_timeout - self._wait_time_before_measure - self._call_duration

        self._device_logger.add_trigger_message(self._op_code)

        # Create success_msg to check ftp transfers finish success
        ftp_filename = os.path.split(self._ftp_filename)[1]
        success_msg = "Ftp %s of file %s finish success" % (self._ftp_direction, ftp_filename)

        # Wait to ftp imeout to verify ftp transfers finish success
        while self._timeout > 0:
            triggered_status = self._device_logger.get_message_triggered_status(self._op_code)
            # When the  success message  is in the message of ACS op code
            if success_msg in str(triggered_status):
                self._logger.info(str(triggered_status))
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
                # Convert to Mbps for result compare
                if float(data_throughput_dict[data_type][0][0:-4]) < self._failure_throughput_dict[data_type]:
                    result = Global.FAILURE
            elif "Kbps" in data_throughput_dict[data_type][0]:
                if float(data_throughput_dict[data_type][0][0:-4]) < self._failure_throughput_dict[data_type] * 1000:
                    result = Global.FAILURE
            elif "bps" in data_throughput_dict[data_type][0]:
                if float(data_throughput_dict[data_type][0][0:-4]) < self._failure_throughput_dict[data_type] * 1000 * 1000:
                    result = Global.FAILURE
            else:
                self._error.Msg = "unknown measure result unit: %s " % str(data_throughput_dict[data_type][0])
                raise TestEquipmentException(TestEquipmentException.PROHIBITIVE_MEASURE, self._error.Msg)

            measure_throughput_msg = measure_throughput_msg + str(data_type)\
                + ":" + str(data_throughput_dict[data_type][0]) + " "
            failure_throughput_msg = failure_throughput_msg + str(data_type)\
                + ":" + str(self._failure_throughput_dict[data_type]) + "Mbps" + " "

        self._error.Msg = measure_throughput_msg + "- " + failure_throughput_msg

        return result, self._error.Msg
