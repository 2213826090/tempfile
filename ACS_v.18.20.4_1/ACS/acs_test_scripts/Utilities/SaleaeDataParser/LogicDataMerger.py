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
then export them in a list. The timestamp will be the saleae one.

STEP 1

:since: 2015-01-27
:author: emarchan
"""

import os
from csv import DictReader as csv_reader
from Constants import DATA_DIRECTION_C2M, DATA_DIRECTION_M2C, CSV_INPUT_FILE_FIELDS
from LogicDataDebug import LogicDataDebug


class LogicDataMerger():
    """
    Merges the Rx and Tx lines exported by Saleae logic analyzer in CSV format then exports them in a list
    The output format is OUTPUT_LIST_FIELDS in Constants.
    The timestamp will be the saleae one.
    """

    SALEAE_DELIMITER = ','
    def __init__(self, c2m_csv, m2c_csv):
        self._data = []
        self._c2m_file = c2m_csv
        self._m2c_file = m2c_csv
        self._logger = LogicDataDebug()

    def get_data(self):
        """
        Returns the formatted Saleae data
        :return: Formatted Saleae data
        :rtype: list of list  (see class description for format)
        """
        return self._data

    def _get_csv_data(self, input_file, cur_dir):
        """
        Add an entry in the current data tree.
        :param input_file: input file to parse
        :type input_file: string
        :param cur_dir: Direction of the data (DATA_DIRECTION_M2C or DATA_DIRECTION_C2M)
        :type cur_dir: String
        """
        if (os.path.isfile(input_file)):
            self._logger.info("LogicDataMerger: Getting data from %s..." % input_file)
            reader = csv_reader(open(input_file, 'r'), fieldnames=CSV_INPUT_FILE_FIELDS)
            cur_line = len(self._data)
            # Skip header
            next(reader, None)
            # Parse the rest of the file
            for row in reader:
                if len(row) >= len(CSV_INPUT_FILE_FIELDS):
                    # Convert framing error
                    if row['RX_FRAMING_ERROR'] != "":
                        framing_error = 1
                    else:
                        framing_error = 0
                    # Format output string
                    new_data = [cur_line, row['TSTAMP'], cur_dir, row['VALUE'], framing_error]
                    self._data.append(new_data)
                else:
                    self._logger.error("LogicDataMerger: Can't compute line %d, ensure the file has a good format!" % cur_line)
                cur_line = cur_line + 1
        else:
            self._logger.error("LogicDataMerger: Can't open Input data file %s" % input_file)

    def parse_data(self):
        """
        Parse the CSV files
        """
        # Get the inputs
        self._get_csv_data(self._c2m_file, DATA_DIRECTION_C2M)
        self._get_csv_data(self._m2c_file, DATA_DIRECTION_M2C)
        self._logger.info("LogicDataMerger: Loaded %d entries" % len(self._data))
