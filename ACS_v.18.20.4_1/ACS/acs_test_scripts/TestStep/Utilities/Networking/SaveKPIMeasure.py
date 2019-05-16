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
:summary: This file implements Test Step for save KPI measures
:since 06/05/2015
:author: jfranchx
"""

import os.path
from Core.TestStep.TestStepBase import TestStepBase


class SaveKPIMeasure(TestStepBase):
    """
    Implements save KPI measure class
    """

    DEFAULT_FILE_NAME = "KPI_RESULTS.log"

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    #------------------------------------------------------------------------------

    def run(self, context):
        """
        Run the step
        """
        TestStepBase.run(self, context)

        # Default file
        if self._pars.file_name is None:
            filename = self.DEFAULT_FILE_NAME
        else:
            filename = str(self._pars.file_name) + ".log"

        fullname = os.path.join(self._global_conf.campaignConfig.get("campaignReportTree").get_report_path(), filename)

        # Create the file if it doesn't exist
        if not os.path.exists(fullname):
            result_file = open(fullname, "w+")
            result_file.write("*** KPI RESULTS ***\n")
            result_file.close()

        # Add new KPI measure to the report
        result_file = open(fullname, "a+")
        if self._pars.comments is not None:
            line_end = " - NOTE : %s\n" % self._pars.comments
        else:
            line_end = "\n"
        if self._pars.wifi_rssi is not None:
            line_wifi_rssi = " - RSSI : %sdBm" % self._pars.wifi_rssi
        else:
            line_wifi_rssi = ""

        new_line = "%s - TARGET : %s%s - MEASURE : %s%s %s%s" % (self._testcase_name,
                                                                 self._pars.target_value,
                                                                 self._pars.target_unit,
                                                                 self._pars.measure_value,
                                                                 self._pars.measure_unit,
                                                                 line_wifi_rssi,
                                                                 line_end)
        result_file.write("%s" % new_line)
        result_file.close()


