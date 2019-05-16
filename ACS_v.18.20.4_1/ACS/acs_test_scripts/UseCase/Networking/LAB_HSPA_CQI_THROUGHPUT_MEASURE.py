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
:summary: This file implements HSPA Cqi throughput measure
:author: hbianx
:since:17/07/2013
"""
import time
import os
import math

from LAB_HSPA_BASE import LabHspaBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.AcsConfigException import AcsConfigException


class LabHspaPdpCqiThroughputMeasure(LabHspaBase):
    """
    Lab HSPA CQI Measure
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_HSPA_BASE Init function
        LabHspaBase.__init__(self, tc_name, global_config)

        self._measure_cell_power_list = self._tc_parameters.\
                    get_param_value("MEASURE_CELL_POWER_LIST").split(",")
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

        self._cqi_result_dico = {}
        self._ftp_data_transfer_state_timeout = 2

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Set up the test configuration
        """

        LabHspaBase.set_up(self)

        self._data_3g.set_cqi_test_set_up()

        if 0 == len(self._measure_cell_power_list):
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                       "MEASURE_CELL_POWER_LIST should contain"
                                        "at least one cell power value")

        return Global.SUCCESS, self._error.Msg

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call LAB_HSPA_BASE Run function
        LabHspaBase.run_test(self)

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

        # Measure the information_bit_throughput for all the cell powers in the list
        for cell_power in self._measure_cell_power_list:
            self._ns_cell_3g.set_cell_power(int(cell_power))
            # Wait for the device adjust to new cell power
            time.sleep(3)
            information_bit_throughput = 0
            measure_times = 0
            # Measure 5 times and calculate the average
            for i in xrange(5):
                # Ensure the data transfer is still ongoing before measurement
                self._data_3g.check_data_connection_transferring(self._ftp_data_transfer_state_timeout, True)
                xfer_status = self._networking_api.get_ftp_xfer_status()
                # Quit from measurement,  once ftp tranfert stop
                if "transferring" != xfer_status:
                    msg = "FTP transfer is %s before measurement start " % xfer_status
                    self._logger.info(msg)
                    break

                try:
                    throughput = self._data_3g.get_information_bit_throughput()
                except TestEquipmentException:
                    self._logger.info("The %d th measurement of power %s is failed"\
                                       % (i, cell_power))
                    throughput = float("nan")

                if  math.isnan(throughput):
                    self._logger.debug("throughput can't be retrieved")
                else:
                    information_bit_throughput = information_bit_throughput + throughput
                    measure_times += 1

            if 0 == measure_times:
                self._logger.info("The measurement of power %d is failed" % cell_power)
                self._cqi_result_dico.update({cell_power: "N/A"})
            else:
                average_bit_throughput = round(information_bit_throughput / measure_times)
                self._cqi_result_dico.update({cell_power: average_bit_throughput})
            self._logger.debug("cqi_results: %s " % str(self._cqi_result_dico))

        # Create the output message
        for key in self._measure_cell_power_list:
            self._error.Msg = self._error.Msg + " Power: " + str(key) + "db throughput: "\
                        + str(self._cqi_result_dico[key]) + " kbps "

        self._networking_api.stop_ftp_xfer(self._ftp_task_id)

        return Global.SUCCESS, self._error.Msg
