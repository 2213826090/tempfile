"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file implements HSPA multi FTP transfer UC
:author: mbrisbax
:since:18/12/2014
"""
import time
import copy
from LAB_HSPA_FTP import LabHspaFtp
from acs_test_scripts.Utilities.FtpUtilities import MultipleFtpTransfer
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.CommunicationUtilities import throughput_targets_string


class LabHspaMultiFtp(LabHspaFtp):

    """
    Lab HSPA ftp test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_HSPA_FTP Init function
        LabHspaFtp.__init__(self, tc_name, global_config)

        # Read the DL_FILE value from UseCase xml Parameter
        self._nb_transfer = self._tc_parameters.get_param_value("NB_TRANSFER", 1, int)

        # Update the failure targets
        self._throughput_targets_8960 = copy.deepcopy(self._throughput_targets)
        self._throughput_targets_8960.set_failure_throughput_from_config(self._dut_config, "NFT")
        # Log Throughput targets for HSPA
        self._logger.info("OTA Throughput to reach on 8960: ")
        self._logger.info(throughput_targets_string(self._throughput_targets_8960))

        self._measurement_duration = 10
        self._wait_time_before_measure = 40

        self._multi_ftp = MultipleFtpTransfer(self._nb_transfer,
                                              self._server_ip_address,
                                              self._direction,
                                              self._username,
                                              self._password,
                                              self._ns_DUT_IP_Address,
                                              self._ftp_api,
                                              self._filename,
                                              self._dlfilename,
                                              self._logger,
                                              self._xfer_timeout,
                                              self._throughput_targets.ul_failure.value,
                                              self._throughput_targets.dl_failure.value,
                                              self._device.binaries_path,
                                              self._device.binaries_path)

# ------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Run FTP transfer using FTP parameters :
        # - LAB_SERVER parameters (ip, username, password)
        # - DIRECTION
        # - DL_FILE or UL_FILE
        # - XFER_TIMEOUT
        # Launch FTP transfers
        status, msg = self._multi_ftp.launch()
        if status == Global.FAILURE:
            self._logger.error(msg)
            self._multi_ftp.stop()
            return Global.FAILURE, msg

        # Get Over The Air (OTA) data rates on 8960
        if self._direction == "DL":
            measure_list = ("OTRX")
        elif self._direction == "UL":
            measure_list = ("OTATx")
        elif self._direction == "BOTH":
            measure_list = ("OTRX", "OTATx")
        # Wait FTP transfer are on steady state
        time.sleep(self._wait_time_before_measure)
        # Get data rates from 8960
        data_throughput_dict = self._ns.get_data_throughput(
            self._measurement_duration,
            measure_list)
        for data_type in data_throughput_dict.keys():
            # Convert measured average data rate to Kbps
            if "Mbps" in data_throughput_dict[data_type][0]:
                data_throughput_dict[data_type][0] = float(data_throughput_dict[data_type][0][0:-4]) * 1000
            elif "Kbps" in data_throughput_dict[data_type][0]:
                data_throughput_dict[data_type][0] = float(data_throughput_dict[data_type][0][0:-4])
            elif "bps" in data_throughput_dict[data_type][0]:
                data_throughput_dict[data_type][0] = float(data_throughput_dict[data_type][0][0:-4]) / 1000
            else:
                self._error.Msg = "unknown measure result unit: %s " % \
                    str(data_throughput_dict[data_type][0])
                self._multi_ftp.stop_multiple_ftp_transfers()
                return Global.FAILURE, self._error.Msg
            # Check measured data rate is greater than failure data rate
            if data_type == "OTATx":
                self._logger.info("Overall UL throughput over the air measured on 8960: %s kbps, target: %s kbps"
                                  % (data_throughput_dict[data_type][0], self._throughput_targets_8960.ul_failure.value))
                if data_throughput_dict[data_type][0] < self._throughput_targets_8960.ul_failure.value:
                    self._logger.error(" Failure over the air UL data rate is not reached")
                    self._multi_ftp.stop()
                    return Global.FAILURE, " Failure over the air UL data rate is not reached on 8960: measured - throughput: %s kbps, - target: %s kbps" \
                                           % (data_throughput_dict[data_type][0], self._throughput_targets_8960.ul_failure.value)
            elif data_type == "OTRX":
                self._logger.info("Overall DL throughput over the air measured on 8960: %s kbps, target: %s kbps"
                                  % (data_throughput_dict[data_type][0], self._throughput_targets_8960.dl_failure.value))
                if data_throughput_dict[data_type][0] < self._throughput_targets_8960.dl_failure.value:
                    self._logger.error(" Failure over the air DL data rate is not reached")
                    self._multi_ftp.stop()
                    return Global.FAILURE, " Failure over the air DL data rate is not reached on 8960: - measured throughput: %s kbps, - target: %s kbps" \
                                           % (data_throughput_dict[data_type][0], self._throughput_targets_8960.dl_failure.value)

        # Wait end of data transfers and checks their throughput
        status, msg = self._multi_ftp.wait_end_of_transfers()
        if status == Global.FAILURE:
            self._logger.error(msg)

        return status, msg

# ------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Call UseCaseBase Tear down
        LabHspaFtp.tear_down(self)

        self._multi_ftp.stop()

        return Global.SUCCESS, "No errors"
