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

:organization: INTEL MCG
:summary: This file implements the step to format the Real-time coex data from a logic analyzer into an XML to compute pass/fail criteria.
:since: 2015-03-19
:author: emarchan

"""

from Core.TestStep.TestStepBase import TestStepBase
from acs_test_scripts.Utilities.SaleaeDataParser.FormatLogicDataForVerdict import FormatLogicDataForVerdict
from acs_test_scripts.Utilities.SaleaeDataParser.LogicDataParser import LogicDataParser
from lxml import etree

class LogicDataToCellCoexRt(TestStepBase):
    """
    Formats the Real-time coex data from a logic analyzer into an XML to compute pass/fail criteria.
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._parser = None

    def run(self, context):
        """
        Runs the test step
        """
        TestStepBase.run(self, context)

        # Instantiate the Logic data formatter
        self._parser = FormatLogicDataForVerdict(cmd_set=self._pars.cmd_set,
                                                 c2m_csv=self._pars.c2m_csv,
                                                 m2c_csv=self._pars.m2c_csv,
                                                 aplog_file=self._pars.aplog)

        # Set the Data formatter logging mode to ACS.
        self._parser.set_acs_logger(self._logger)

        # Parse the input data
        self._parser.run_parsing()

        self._logger.info("STEP 4 (Parse the formatted XML data)")
        xml_formatter_in = self._parser.get_data()

        ldp = LogicDataParser()

        ldp.get_input_data_from_file("/tmp/toto")
        ldp.print_input_data()

        ldp.get_input_data_from_variable(xml_formatter_in)


        ldp.print_input_data()
        xml_formatter_out = ldp.run()
        # Save data into context
        context.set_info(self._pars.formatted_data_output, etree.tostring(xml_formatter_out, pretty_print=True))
