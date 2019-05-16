"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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

@organization: INTEL MCG PSI
@summary: This module implements a test step to compute droidbot scores
@since: 17/11/14
@author: kturban
"""
import json
import os
import numpy
import scipy
from Device.DeviceManager import DeviceManager
from Core.TestStep.TestStepBase import TestStepBase
from Core.Report.SecondaryTestReport import SecondaryTestReport
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.Utilities.PnPUtilities import get_math_function

class ComputeScore(TestStepBase):
    """
    Compute a list of score values
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

    def hack_report(self, computed_result, unit_score, run_count, expected_success, success_count):
        """
        Hack to add test step comment in AWR database for now
        Test steps result are not yet available in AWR database (and not planned...)
        """
        device = DeviceManager._device_instances.get("PHONE1")
        external_tc_report = SecondaryTestReport(device.get_report_tree().get_report_path())

        run_msg = "Run: {0} - Success: {1}/{2} - ".format(run_count, success_count, expected_success)
        if computed_result is None:
            verdict = external_tc_report.verdict.FAIL
            message = "No score found"
        else:
            message = "Score: {0} ({1})".format(computed_result, unit_score)
            if success_count < expected_success:
                verdict = external_tc_report.verdict.FAIL
            else:
                verdict = external_tc_report.verdict.PASS

        external_tc_report.add_result(self._pars.result_name,
                                      verdict, run_msg + message,
                                      self._testcase_name,
                                      self._conf.tc_order)

        return (verdict == external_tc_report.verdict.PASS), run_msg

    def get_result_value_and_desc(self, droidbot_result, score_name):
        """
        example of droidbot json output:
        {"test": "com.intel.droidbot.perf.BenchmarkPI#test",
        "run": 1,
        "scores": [{"name": "score", "lib": true, "value": 306, "desc": "ms"},
                   {"name": "duration", "lib": true, "value": 5, "desc": "Seconds"}]}
        This method will extract a score type from "scores" key dict
        :type score_name: str
        :param score_name: value will be extracted according to this score name
        """
        score_value = None
        desc_value = None

        run_count = droidbot_result.get("numRun")
        expected_run = droidbot_result.get("numExpectedRuns")

        scores = droidbot_result.get("scores")
        if scores:
            for score in scores:
                if score.get("name", "") == score_name:
                    score_value = score.get("value")
                    desc_value = score.get("desc")
        return score_value, desc_value, run_count, expected_run

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        # compute result
        computed_result = None
        raw_results = []
        data = None
        unit_score = ""
        total_run = 0
        expected_run = 0

        functor = get_math_function(self._pars.operation)

        # load json results
        json_result = self._pars.json_droidbot_result
        if json_result:
            data = json.loads(json_result)
            if data:
                # retrieve score for each iteration result
                for res in data:
                    score, unit_score, total_run, expected_run = \
                        self.get_result_value_and_desc(res, self._pars.score_name)
                    if score:
                        raw_results.append(score)
            if raw_results:
                computed_result = str(functor(raw_results))
                # set scores in test step context for reuse in another step
                context.set_info(self._pars.result, computed_result)
                self.ts_verdict_msg = "Computed {0} for scores : {1}({2})".format(self._pars.operation.lower(),
                                                                                  computed_result, unit_score)
                self._logger.info("Computed {0} for scores : {1}({2})".format(self._pars.operation.lower(),
                                                                              computed_result, unit_score))
                score_as_str = ";".join(str(x) for x in raw_results)
                self.ts_verdict_msg = "Scores in {0} : {1}".format(unit_score, score_as_str)
                self._logger.info("Scores in {0} : {1}".format(unit_score, score_as_str))
            else:
                self._logger.warning("Data can't be loaded from droidbot results")
                self.ts_verdict_msg = "No result"
        else:
            self._logger.warning("Result from Droidbot doest not exist, cannot compute values")
            self.ts_verdict_msg = "No result"
        # in order to store data in AWR database
        success, msg = self.hack_report(computed_result, unit_score, total_run, expected_run, len(raw_results))
        if not success:
            raise DeviceException(DeviceException.OPERATION_FAILED, "Not enough results: {0}".format(msg))
