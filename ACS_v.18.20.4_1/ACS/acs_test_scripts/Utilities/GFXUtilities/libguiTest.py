"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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

:organization: INTEL OTC ANDROID QA

:since: 11/10/14
:author: mmaraci
"""
from OGLConform import Auxiliary
from acs_test_scripts.Utilities.GFXUtilities.GFXFunctions import *
from acs_test_scripts.Utilities.GFXUtilities.GFXFunctions import write_report


class libguiTest(object):

    def __init__(self):
        """

        """

        self._dut_location = "data/app/"
        self._adb_bin = self._dut_location + "libgui_test"
        self._test_option = " --gtest_filter="
        self._aux = Auxiliary()

    def run_test(self, test_name):
        """
        A method to run a a single test
        """
        result = ''
        run_cmd = self._adb_bin + self._test_option + test_name + "| tail -n1"
        out_result, out_file = self._aux.send_cmd(run_cmd, 120)
        if "PASSED" in out_file:
            result = "PASSED"
        elif "FAILED" in out_file:
            result = "FAILED"
        return result

    def execute_single_run(self, test_name, result_filename):
        """
        This method uses run_test to run a single test then calls write_report from
        GFXFunctions to store the obtained results
        """
        test_result = self.run_test(test_name)
        write_report(test_name, test_result, result_filename)
        return test_result
