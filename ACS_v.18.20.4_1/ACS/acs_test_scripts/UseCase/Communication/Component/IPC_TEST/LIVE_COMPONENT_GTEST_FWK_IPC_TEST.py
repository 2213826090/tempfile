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
:summary: This file implements IPC TEST Use Case via Google Test Framework
:since: 07/04/2013
:author: jreynaux
"""
# Module name follows ACS conventions
# pylint: disable=C0103

from UtilitiesFWK.Utilities import Global, Verdict
# We locally disable some errors that PyDev may see
# pylint: disable=F0401,E0611
from acs_test_scripts.UseCase.Communication.Component.LIVE_GTEST_FWK_BASE import LiveGTestFwkBase
from Core.Report.SecondaryTestReport import SecondaryTestReport
# We restore the previously disabled errors.
# pylint: enable=F0401,E0611


class LiveComponentGTestFwkIpcTest(LiveGTestFwkBase):

    """
    Live GTester MTS Use Case class.
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Call UseCase base __init__ method
        LiveGTestFwkBase.__init__(self, tc_name, global_config)

        # Enable ACS secondary reports
        self._secondary_report = SecondaryTestReport(
            self._device.get_report_tree().get_report_path())

    def set_up(self):

        LiveGTestFwkBase.set_up(self)

        # Format the command with ipc-test application
        self._command = self._build_gtest_fwk_command("/system/bin/ipc-test")

        # Return the verdict
        return Global.SUCCESS, "No error."

    def run_test(self):
        """
        Executes the test.
        """
        # Run the inherited 'run_test' method
        LiveGTestFwkBase.run_test(self)

        # Run the command
        self.runner.run_gtest_fwk_command(self._command)

        # Retrieve result object
        result_object = self.runner.last_result

        # Parse and compute results
        self._parse_and_compute_result(result_object)

        # Compute the verdict
        self._logger.debug("Compute verdict.")
        verdict = result_object.verdict

        output = "IPC-test execution successfull."
        if verdict != Global.SUCCESS:
            output = result_object.log_messages
            # Convert the output to a str if needed
            if isinstance(output, list):
                output = " ".join(map(str, output))  # pylint: disable=W0141

        results = result_object.multiple_results
        # In case of multiple execution
        if results is not None:
            # Add the results to test reports
            self._secondary_report.add_result_from_dict(results, self.get_name(), self.tc_order)
            # Adjust verdict according to
            if self._count_verdict_in_multiple_result(Verdict.FAIL, results) > 0:
                verdict = Global.FAILURE
                output = "IPC-test execution failed."
            elif self._count_verdict_in_multiple_result(Verdict.BLOCKED, results) > 0:
                verdict = Global.BLOCKED
                output = "IPC-test execution blocked."
            else:
                verdict = Global.SUCCESS
                output = "IPC-test execution successfull."

        # Return the verdict
        return verdict, output
