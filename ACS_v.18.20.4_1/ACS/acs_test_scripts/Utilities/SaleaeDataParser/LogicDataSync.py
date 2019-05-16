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

:organization: INTEL CCG
:summary: This file implements the class to merge the Rx and Tx lines exported by Saleae logic analyzer in CSV format
then export them in a list. The timestamp will be the DUT one.

STEP 2

:since: 2015-01-27
:author: emarchan
"""

from operator import itemgetter
from Constants import OUTPUT_LIST_FIELDS
import re
from LogicDataDebug import LogicDataDebug
from datetime import datetime, date
import numpy
from time import mktime

class LogicDataSync():
    """
    Syncs the LogicDataMerger with aplog then exports items in a list
    The output format is OUTPUT_LIST_FIELDS in Constants
    The timestamp will be the DUT one.
    """

    def __init__(self, input_data=None, aplog_file=None):
        """
        Creator
        :param input_data: output from a LogicDataMerger.
        :type input_data LogicDataMerger
        :param aplog_file: aplog file to sync the logic data with
        :type aplog_file: string
        """
        if input_data is None:
            self._input_data = []
        else:
            self._input_data = input_data
        self._ap_log_file = aplog_file
        self._data = []
        self._logger = LogicDataDebug()

    def sync_data(self):
        """
        Makes the input list starts from 0
        Sorts it in ascending timestamp
        Replace the Salae time given by the input into the real time
        """
        if self._input_data != [] and self._ap_log_file is not None:
            index = OUTPUT_LIST_FIELDS.index('TSTAMP')
            self._data = sorted(self._data, key=itemgetter(OUTPUT_LIST_FIELDS.index('TSTAMP')))
            new_data_list = []
            start_offset = float(self._input_data[0][index])
            aplog_offset = self._find_start_in_ap_log()
            for cur_data in self._input_data:
                # Copy current line
                new_data = cur_data

                # Align the timestamp to have the 1st element starts from 0 adding the ap log offset
                new_data[index] = float(new_data[index]) - start_offset + aplog_offset

                # Append the result in the output list
                new_data_list.append(new_data)
            self._data = new_data_list
        else:
            self._logger.error("LogicDataSync - Please check sync_data input data")

    def _find_start_in_ap_log(self):
        """
        Parses the aplog and find the beginning of the simulation.
        """
        aplog_fd = open(self._ap_log_file)
        ap_start_out = 0.0
        """
        01-27 18:15:46.690 D/WifiService(  647): setWifiEnabled: true pid=860, uid=10019
        """
        for cur_line in aplog_fd:
            if re.match(".*WifiService.*setWifiEnabled.*", cur_line):
                real_date = cur_line.split()[0]
                (month, day) = real_date.split('-')
                # We don't have the date in the aplog, we assume it's the same as today.
                year = date.today().year
                real_time = cur_line.split()[1]
                real_time = real_time.replace('.', ':')
                (hour, mins, sec, msecs) = real_time.split(":")
                ap_start = datetime(int(year), int(month), int(day), int(hour), int(mins), int(sec), int(msecs))
                ap_start_out = numpy.float128(mktime(ap_start.timetuple())) + numpy.float128(ap_start.microsecond * 1e-6)

                break
        self._logger.info("Found APLOG start = %d" % ap_start_out)
        return ap_start_out

    def get_data(self):
        """
        Returns the formatted and synchronized Saleae data
        :return: Formatted Saleae data
        :rtype: list of list  (see class description for format)
        """
        return self._data

