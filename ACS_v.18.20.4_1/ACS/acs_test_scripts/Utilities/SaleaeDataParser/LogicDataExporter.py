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
:summary: This file implements the class to export the formatted Logic data after the sync for the SaleaeDataParser.

STEP 3

:since: 2015-01-27
:author: emarchan
"""

from lxml import etree
import time
from Constants import DATA_DIRECTION_C2M, DATA_DIRECTION_M2C, OUTPUT_LIST_FIELDS, RT_FIELDS_TYPES, hex_to_bin_str
from LogicDataDebug import LogicDataDebug

class LogicDataExporter():
    """
    Generates the formatted Logic data after the sync
    """
    def __init__(self, cmd_set, input_data=None):
        """
        Creator
        :param input_data: output from a LogicDataSync.
        :type input_data LogicDataSync
        :param cmd_set: Commands sets mode: standard ("standard") or vendor specific ("lnp" for instance).
        :type cmd_set: string
        """

        self._root = etree.Element("DATAS")
        current = self._root
        current.set("mode", cmd_set.upper())

        if input_data is None:
            self._input_data = []
        else:
            self._input_data = input_data
        self._logger = LogicDataDebug()

    def parse_data(self):
        """
        Generated the
        """
        index_id = OUTPUT_LIST_FIELDS.index('ID')
        index_tstamp = OUTPUT_LIST_FIELDS.index('TSTAMP')
        index_dir = OUTPUT_LIST_FIELDS.index('DIRECTION')
        index_value = OUTPUT_LIST_FIELDS.index('VALUE')
        index_framing_err = OUTPUT_LIST_FIELDS.index('RX_FRAMING_ERROR')
        for cur_item in self._input_data:
            """
            xmltree = etree.Element("data")
            etree.SubElement(xmltree, "id").text = str(cur_item[index_id])
            etree.SubElement(xmltree, "tstamp").text = str(cur_item[index_tstamp])
            etree.SubElement(xmltree, "direction").text = cur_item[index_dir]
            etree.SubElement(xmltree, "value").text = cur_item[index_value]
            """

            xmltree = etree.Element("DATA", id=str(cur_item[index_id]),
                                           time_stamp="%.12f" % cur_item[index_tstamp],
                                           direction=cur_item[index_dir],
                                           value=cur_item[index_value],
                                           value_bin=hex_to_bin_str(cur_item[index_value]),
                                           framing_error=str(cur_item[index_framing_err]))

            self._root.append(xmltree)

    def generate_dummy_data(self, number_of_entries):
        """
        Fills the data tree with dummy values.
        :param number_of_lines: Number of entries to generate
        :type number_of_lines: int
        """
        cur_tstamp = time.time()
        cur_dir = DATA_DIRECTION_M2C
        start_time = time.time()
        self._logger.info("Generating %d fake entries..." % number_of_entries)
        for cur_data_num in range(0, number_of_entries):
            if cur_data_num == 1:
                framing_err = "1"
            else:
                framing_err = "0"
            cur_value = RT_FIELDS_TYPES.keys()[cur_data_num % len(RT_FIELDS_TYPES)]
            cur_value = hex(cur_value).replace('0x', '')
            while len(cur_value) < 2:
                cur_value = "0" + cur_value
            xmltree = etree.Element("DATA", id=str(cur_data_num),
                                           time_stamp="%.12f" % (cur_tstamp),
                                           direction=cur_dir,
                                           value=str(cur_value),
                                           value_bin=hex_to_bin_str(cur_value),
                                           framing_error=framing_err)

            self._root.append(xmltree)


            cur_tstamp = cur_tstamp + 1
            if cur_dir == DATA_DIRECTION_M2C:
                cur_dir = DATA_DIRECTION_C2M
            else:
                cur_dir = DATA_DIRECTION_M2C

        self._logger.info("Generated %d fake entries in %d s" % (number_of_entries, (time.time() - start_time)))

    def save_data_into_file(self, output_file):
        """
        Saves the current data tree into a file.
        :param output_file: File to save the data into.
        :type output_file: string
        """
        file_descr = open(output_file, "w")
        file_descr.writelines(etree.tostring(self._root, pretty_print=True))
        file_descr.close()

    def get_data(self):
        """
        Returns the data tree
        :return data tree
        :rtype: XML data
        """
        return self._root

    def __repr__(self):
        """
        Defines the print format of the data tree when you call print(LogicDataExporter instance)
        """
        return etree.tostring(self._root, pretty_print=True)
