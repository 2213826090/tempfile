#!/usr/bin/env python
# pylint: disable=C0103
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


"""
from LogicDataMerger import LogicDataMerger
from LogicDataSync import LogicDataSync
from LogicDataExporter import LogicDataExporter
from LogicDataParser import LogicDataParser
from Constants import DATA_DIRECTION_C2M
import os
from optparse import OptionParser
from lxml import etree
from LogicDataDebug import LogicDataDebug

def parsed_xml_to_file(xml_data, file_name):
    xml_data.insert(0, etree.Comment("   Connectivity to modem data:" + os.path.relpath(options.c2m_file) + "   "))
    xml_data.insert(1, etree.Comment("   Modem to connectivity data:" + os.path.relpath(options.m2c_file) + "   "))
    xml_data.insert(2, etree.Comment("   Aplog file used to sync the data:" + os.path.relpath(options.aplog_file) + "   "))
    xml_data.insert(3, etree.Comment("   Formatted data from Saleae:" + os.path.relpath(options.xml3_file) + "   "))
    if (len(options.acs_file_comment) > 0):
        xml_data.insert(4, etree.Comment("   " + options.acs_file_comment + "   "))

    _logger.debug(etree.tostring(xml_data, pretty_print=True))
    # create step 4 xml file
    file_descr = open(file_name, "w")
    file_descr.writelines(etree.tostring(xml_data, pretty_print=True))
    file_descr.close()

class FormatLogicDataForVerdict():

    def __init__(self, cmd_set="standard", c2m_csv=None, m2c_csv=None, aplog_file=None):
        self._c2m_csv = c2m_csv
        self._m2c_csv = m2c_csv
        self._aplog_file = aplog_file

        self._ld_merge = None
        self._ld_sync = None
        self._ld_exp = None

        self._cmd_set = cmd_set
        self._logger = LogicDataDebug()

    def run_parsing(self):
        """
        STEP 1 (Merge CSV data from Salaea logic analyzer)
        """
        self._logger.info("STEP 1 (Merge CSV data from Salaea logic analyzer)")
        self._ld_merge = LogicDataMerger(c2m_csv=self._c2m_csv, m2c_csv=self._m2c_csv)
        self._ld_merge.parse_data()
        ld_merge_out = self._ld_merge.get_data()
        # self._logger.debug("Step 1 output:")
        # self._logger.debug(ld_merge_out)

        """
        STEP 2 (Sync aplog)
        """
        self._logger.info("STEP 2 (Sync aplog)")
        self._ld_sync = LogicDataSync(ld_merge_out, self._aplog_file)
        self._ld_sync.sync_data()
        ld_sync_output = self._ld_sync.get_data()
        # self._logger.debug("Step 2 output:")
        # self._logger.debug(ld_sync_output)

        """
        STEP 3 (Generate the formatted XML data)
        """
        self._logger.info("STEP 3 (Generate the formatted XML data)")
        self._ld_exp = LogicDataExporter(self._cmd_set, ld_sync_output)
        self._ld_exp.parse_data()

    def save_data_into_file(self, xml_out_file):
        return self._ld_exp.save_data_into_file(xml_out_file)

    def get_data(self):
        if self._ld_exp is None:
            return []
        else:
            return self._ld_exp.get_data()

    def __repr__(self):
        return self._ld_exp.__repr__()

    def set_acs_logger(self, logger):
        self._logger.set_acs_logger(logger)

if __name__ == "__main__":

    """
    Command line interface options parser
    """

    script_path = os.path.dirname(os.path.realpath(__file__))
    parser = OptionParser()


    parser.add_option("--cmd_set",
                              help="REQUIRED Set datum as standard(cmd_set=standard) or lnp (cmd_set=lnp).",
                              metavar="STRING",
                              dest="cmd_set",
                              default="standard")

    parser.add_option("--c2m_file",
                              help="CSV file containing the connectivity to modem data from Saleae.",
                              metavar="FILE",
                              default="%s/sample_data/export_c2m.csv" % (script_path),
                              dest="c2m_file")
    parser.add_option("--m2c_file",
                              help="CSV file containing the modem to connectivity data from Saleae.",
                              metavar="FILE",
                              default="%s/sample_data/export_m2c.csv" % (script_path),
                              dest="m2c_file")

    parser.add_option("--aplog_file",
                              help="Aplog file used to sync the data between Saleae and the DUT",
                              metavar="FILE",
                              default="%s/sample_data/aplog" % (script_path),
                              dest="aplog_file")

    parser.add_option("--xml_file",
                              help="XML file containing the formatted data exported from Saleae.",
                              metavar="FILE",
                              default="%s/sample_data/step3.xml" % (script_path),
                              dest="xml3_file")

    parser.add_option("--parsed_file",
                              help="XML output file containing parsed data.",
                              metavar="STRING",
                              default="%s/sample_data/parsed_data" % (script_path),
                              dest="parsed_file")

    parser.add_option("--comment",
                              help="Comment inserted in XML output file.",
                              metavar="STRING",
                              default="",
                              dest="acs_file_comment")

    parser.add_option("--rf_edge",
                              help="Cleanup the file of the wrong rising/falling edges.",
                              metavar="FILE",
                              default="",
                              dest="rf_edge_file")

    parser.add_option("--verbose",
                              help="Prints debug information",
                              action="store_true", dest="verbose", default=False)

    (options, args) = parser.parse_args()

    if options.cmd_set not in ["standard", "lnp"]:
        parser.error('--cmd_set=(standard|lnp) is required')

    if (len(options.rf_edge_file) > 0):
        if (not os.path.exists(options.rf_edge_file)):
            parser.error('"%s" does not exist' % options.rf_edge_file)
        else:
            ldp = LogicDataParser()
            ldp.check_rising_falling_edge(options.rf_edge_file)
        os.sys.exit()

    if (not os.path.exists(options.c2m_file)):
        parser.error('"%s" does not exist' % options.c2m_file)
    if (not os.path.exists(options.m2c_file)):
        parser.error('"%s" does not exist' % options.m2c_file)
    if (not os.path.exists(options.aplog_file)):
        parser.error('"%s" does not exist' % options.aplog_file)
    xml_out_path = os.path.dirname(options.xml3_file)
    if (not os.path.isdir(xml_out_path)):
        parser.error('"%s" does not exist' % xml_out_path)

    debug_mode = options.verbose
    _logger = LogicDataDebug()
    if debug_mode is True:
        _logger.set_logging_level('debug')
    else:
        _logger.set_logging_level('info')

    # Example 1: Parses the input files
    _logger.info("example 1: parses the input files using input data from:\n"
        + "c2m_file = " + options.c2m_file + "\n"
        + "m2c_file = " + options.m2c_file + "\n"
        + "aplog_file = " + options.aplog_file)

    step123 = FormatLogicDataForVerdict(cmd_set=options.cmd_set,
        c2m_csv=options.c2m_file
        , m2c_csv=options.m2c_file
        , aplog_file=options.aplog_file)
    step123.run_parsing()

    # Example 1bis: Simulates the parsing of the input files
    # _logger.info(log_tag()+"Example 1bis: Steps 1-2-3, Simulates the parsing of the input files")
    step123 = LogicDataExporter(cmd_set=options.cmd_set, input_data=None)
    step123.generate_dummy_data(10)

    # save data into file
    step123.save_data_into_file(options.xml3_file)
    _logger.debug("Step 3(Generate the formatted XML data) output (" + options.xml3_file + "):")
    _logger.debug(step123)

    """
    STEP 4 (Parse the formatted XML data)
    """
    _logger.info("STEP 4 (Parse the formatted XML data)")
    xml_data = step123.get_data()
    ldp = LogicDataParser()
    ldp.get_input_data_from_file(options.xml3_file)

    # ldp.print_input_data()

    # Example: Filter messages matching a given direction.
    _logger.info("Filtering data matching %s" % DATA_DIRECTION_C2M)
    c2m_elems = ldp.get_data_matching_direction(DATA_DIRECTION_C2M)
    _logger.debug(c2m_elems)

    """
    STEP 5 (Get the data for ACS verdict parsing)
    """
    _logger.info("STEP 5 parsed:" + options.parsed_file)
    parsed_xml = ldp.run()
    parsed_xml_to_file(parsed_xml, options.parsed_file)
    print etree.tostring(parsed_xml, pretty_print=True)

