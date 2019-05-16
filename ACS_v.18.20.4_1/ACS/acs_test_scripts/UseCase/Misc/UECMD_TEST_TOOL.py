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
:summary: This file implements the base for UECmd generic testing
:author: apairex
:since:27/03/2013
"""
from Core.Report.SecondaryTestReport import SecondaryTestReport
import traceback
import sys
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.AcsBaseException import AcsBaseException
from Core.Report.Live.LiveReporting import LiveReporting


class UECmdTestTool:

    """
    Base for UECmd generic testing
    """

    def __init__(self, tc_name, logger, device):
        """
        Constructor
        """
        self.__tc_name = tc_name
        self.__acs_logger = logger
        self.__secondary_report = None
        self.__sub_verdict = {}
        self.__tc_order = 0

        # Instantiate the Secondary Report object
        self.__secondary_report = SecondaryTestReport(device.get_report_tree().get_report_path())

        # pylint: disable=C0103
        self._PASS = self.__secondary_report.verdict.PASS
        self._FAIL = self.__secondary_report.verdict.FAIL
        self._BLOCKED = self.__secondary_report.verdict.BLOCKED

        # index to the function items in the general dictionary
        self._NAME = "function_name"
        self._FCT = "function_pointer"
        self._PARMS = "parameters"
        self._DEP = "dependencies"
        self._EXP_RES = "expected_result"

#------------------------------------------------------------------------------

    def set_up(self, tc_order):
        """
        Initialization of the tool.
        """
        self.__sub_verdict = {}
        self.__tc_order = tc_order

    def _check_uecmd(self, fct_bundle):
        """
        Generic function to test UECmd

        :type fct_bundle: dictionary
        :param fct_bundle: Dictionary which contains all information to run
                    UECmd function test:
                    - Name of the Function to test (optional: If not given,
                use the name of the function retrieve from the pointer to UECmd)
                    - Pointer to the UECmd function to test (Mandatory)
                    - Parameters to call the UECmd function with (Optional:
                if not given, no parameter required to call the function)
                    - List of Test names that should be PASSed in order to run this test
                (Optional)
                    - Expected result in response of UECmd call
                (Optional)

        :rtype: str
        :return: Verdict.FAIL/PASS/BLOCKED
        """
        # Retrieve input parameters
        uecmd_fct = fct_bundle[self._FCT]
        default_name = uecmd_fct.__name__
        fct_name = fct_bundle.get(self._NAME, default_name)
        parameters = fct_bundle.get(self._PARMS, [])
        depends_on = fct_bundle.get(self._DEP, [])
        expected_result = fct_bundle.get(self._EXP_RES, None)
        verdict = self._BLOCKED

        # Checks dependencies
        for dependent_test in depends_on:
            if dependent_test not in self.__sub_verdict or not self.__sub_verdict[dependent_test]:
                # The test cannot be run
                self.__sub_verdict[fct_name] = False
                msg = "%s BLOCK due to %s FAILED" % (fct_name, dependent_test)
                self.__acs_logger.error(msg)
                verdict = self._BLOCKED
                self.__secondary_report.add_result(fct_name, verdict,
                                                   msg, self.__tc_name, self.__tc_order)
                return verdict

        # Test the UECmd
        try:
            self.__acs_logger.debug("Test %s UECmd." % fct_name)
            # pylint: disable=W0142
            ret = uecmd_fct(*parameters)
            self.__acs_logger.info("%s returns: %s" % (fct_name, ret))

            # Check returned value
            if expected_result is not None and ret != expected_result:
                verdict = self._FAIL
                comment = "UECommand FAIL - expected result: %s. Got: %s " % (expected_result, ret)
                self.__sub_verdict[fct_name] = False
            else:
                verdict = self._PASS
                comment = "UECommand PASS"
                self.__sub_verdict[fct_name] = True

        except AcsBaseException as euce:
            self.__acs_logger.warning("DeviceException occurs: " + str(euce))
            comment = "DeviceException occurs: " + str(euce)
            verdict = self._FAIL
            self.__sub_verdict[fct_name] = False
            traceback.print_exc(file=sys.stdout)

        except Exception as exc:  # pylint: disable=W0703
            self.__acs_logger.error("Exception occurs: %s" % str(exc))
            verdict = self._FAIL
            comment = "Exception occurs: %s" % str(exc)
            self.__sub_verdict[fct_name] = False
            traceback.print_exc(file=sys.stdout)

        # Record the result in the secondary report
        self.__secondary_report.add_result(fct_name, verdict, comment,
                                           self.__tc_name, self.__tc_order)
        # for TCR compatibility
        dico = {fct_name : "verdict %s  - %s" % (verdict, comment)}
        LiveReporting.instance().update_running_tc_info(test_info=dico)

        return verdict

    def _compute_general_verdict(self):
        """
        Compute the general verdict.
        Method to call at the end of run test
        """
        # Get the list of functions which failed
        fct_fail = list()
        for fct, res in self.__sub_verdict.iteritems():
            if not res:
                fct_fail.append(fct)

        if fct_fail:
            msg = "Some function failed: " + str(fct_fail)
            self.__acs_logger.error(msg)
            raise AcsToolException(AcsToolException.OPERATION_FAILED, msg)
