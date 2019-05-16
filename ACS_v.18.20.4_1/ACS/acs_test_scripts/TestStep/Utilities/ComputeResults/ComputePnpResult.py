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

@organization: INTEL MCG PSI
@summary: This module implements Sphinx Auto-generator of Documentation
@since: 17/11/14
@author: kturban
"""

import os
import time

from Core.PathManager import Paths
from Core.TestStep.TestStepBase import TestStepBase
from Core.Report.SecondaryTestReport import SecondaryTestReport
from Device.DeviceManager import DeviceManager
from ErrorHandling.TestEquipmentException import TestEquipmentException
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.PnPUtilities import PnPResults
from acs_test_scripts.Utilities.PnPUtilities import get_math_function

class ComputePnpResultBase(TestStepBase):
    """
    Base class for PnP test steps to manage PnP context
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        TestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._attributes = {}
        self._pnp_results = None
        self._device = DeviceManager().get_device("PHONE1")
        self._dut_name = DeviceManager().get_device_config("PHONE1").get("Name")
        self._report_tree = global_conf.campaignConfig.get("campaignReportTree")

    def update_report(self, pnp_verdict, result_name, measure):
        """
        Hack to add test step comment in AWR database for now
        Test steps result are not yet available in AWR database (and not planned...)
        """
        device = DeviceManager._device_instances.get("PHONE1")
        external_tc_report = SecondaryTestReport(device.get_report_tree().get_report_path())

        if pnp_verdict == Global.FAILURE:
            verdict = external_tc_report.verdict.FAIL
        else:
            verdict = external_tc_report.verdict.PASS

        external_tc_report.add_result(result_name,
                                      verdict,
                                      measure,
                                      self._testcase_name,
                                      self._conf.tc_order)

    def _run(self):
        """
        Run specific test step action
        """

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        TestStepBase.run(self, context)

        self._pnp_results = context.get_info(self._pars.stored_pnp_result)
        if self._pnp_results is None:
            raise TestEquipmentException(TestEquipmentException.READ_PARAMETER_ERROR,
                                         "Context key '{0}' does not content initialized PnpResult object".format(
                                         self._pars.stored_pnp_result))
        verdict, output = self._run()

        self._pnp_results.update(self._attributes)

        if verdict == Global.FAILURE:
            self._logger.info(output)

        self.ts_verdict_msg = output


class PnPInitContext(ComputePnpResultBase):
    """
    Set test duration to context
    """

    def run(self, context):
        """
        Runs the test step
        """
        attributes = dict()
        attributes["date"] = time.strftime("%Y-%m-%d %H:%M:%S")
        attributes["id"] = os.path.basename(self._testcase_name)
        failure_file = os.path.join(Paths.EXECUTION_CONFIG, self._device.get_config("FailureFile"))
        target_file = os.path.join(Paths.EXECUTION_CONFIG, self._device.get_config("TargetFile"))

        iteration = False
        if self._tc_parameters.get_b2b_iteration() > 1:
            iteration = True

        # init pnp result object
        pnp_results = PnPResults(self._report_tree,
                                 self._dut_name,
                                 failure_file,
                                 target_file,
                                 attributes,
                                 iteration)
        # store object into the context
        context.set_info(self._pars.save_as, pnp_results)


class PnPReportResidencyVerdict(ComputePnpResultBase):
    """
    Compute verdict for PnP Test cases
    """

    def _run(self):
        """
        Runs the test step
        """
        verdict_name = ""
        sleepmode_api = self._context.get_info("SleepModeApi")
        if sleepmode_api is None:
            raise TestEquipmentException(TestEquipmentException.PLATFORM_ERROR, "Cannot retrieve sleep mode API")

        test_duration = self._pars.test_duration

        self._attributes["duration"] = str(test_duration)
        self._attributes["residency_mode"] = sleepmode_api.get_sleep_mode()

        if not sleepmode_api.device_has_residencies():
            return Global.SUCCESS, "Device has no residency file defined, skipping"

        verdict_name += "RESIDENCY_MEASURE({0})"
        measures = self._pnp_results.retrieve_residency_measures(sleepmode_api.get_sleep_mode())
        if not measures:
            raise TestEquipmentException(TestEquipmentException.MEASUREMENT_ERROR, "Cannot retrieve measure information")
        sleepmode_to_report = set([sleepmode_api.get_sleep_mode()])
        if self._pars.residency_to_report:
            sleepmode_to_report.update(set(self._pars.residency_to_report))

        verdict_msg = ""
        for residency in sleepmode_to_report:
            output_measure = ""
            if not residency in measures:
                self._logger.warning("Measure does not contains '{0}' residency information".format(residency))
                continue
            for value_type, value in measures[residency][-1].items():
                output_measure += "{0}={1};".format(value_type, value)
            self.update_report(Global.SUCCESS,
                               verdict_name.format(residency),
                               output_measure)
            if verdict_msg:
                verdict_msg += " | "
            verdict_msg += verdict_name.format(residency) + ":" + output_measure

        if verdict_msg:
            self._pnp_results.update({"residency_msg": verdict_msg})

        return Global.SUCCESS, "Measure : {0}".format(measures)

class PnPComputeResidencyVerdict(ComputePnpResultBase):
    """
    Compute verdict for PnP Test cases
    """

    def _compute_score(self, measures, sleep_mode):
        """
        Compute result according to specified operation
        """
        functor = get_math_function(self._pars.operation)
        raw_results = []
        if not sleep_mode in measures:
            raise TestEquipmentException(TestEquipmentException.MEASUREMENT_ERROR,
                                         "Measure does not contains '{0}' residency information".format(sleep_mode))
        for measure in measures[sleep_mode]:
            value = measure.get("residency")
            if value is None:
                raise TestEquipmentException(TestEquipmentException.MEASUREMENT_ERROR,
                                             "Cannot retrieve residency {0}".format(sleep_mode))
            raw_results.append(value)
        computed_result = str(functor(raw_results))
        return computed_result

    def _run(self):
        """
        Runs the test step
        """
        verdict_name = ""
        sleepmode_api = self._context.get_info("SleepModeApi")
        if sleepmode_api is None:
            raise TestEquipmentException(TestEquipmentException.PLATFORM_ERROR, "Cannot retrieve sleep mode API")

        test_duration = self._pars.test_duration

        self._attributes["duration"] = str(test_duration)
        self._attributes["residency_mode"] = sleepmode_api.get_sleep_mode()
        # do we need compute verdict according to failure/target ?

        if not sleepmode_api.device_has_residencies():
            return Global.SUCCESS, "Device has no residency file defined, skipping"

        verdict_name = "{0}#".format(self._pars.operation)
        verdict_name += "RESIDENCY_MEASURE({0})"
        measures = self._pnp_results.retrieve_residency_measures(sleepmode_api.get_sleep_mode())
        if not measures:
            raise TestEquipmentException(TestEquipmentException.MEASUREMENT_ERROR, "Cannot retrieve measure information")

        sleepmode_to_report = sleepmode_api.get_sleep_mode()
        measure = self._compute_score(measures, sleepmode_to_report)

        code = self._pnp_results.check_residency_verdict(float(measure))

        self.update_report(code, verdict_name.format(sleepmode_to_report), measure)

        if code == Global.FAILURE:
            verdict_value = "FAIL"
        else:
            verdict_value = "PASS"

        self._pnp_results.update({"residency_verdict": verdict_value})

        if not self._pars.verdict_criteria or "both" in str(self._pars.verdict_criteria).lower():
            if "fail" not in self._pnp_results.get_value("verdict").lower():
                self._pnp_results.update({"verdict": verdict_value})
        elif "residency" in str(self._pars.verdict_criteria).lower():
            self._pnp_results.update({"verdict": verdict_value})

        return code, "Computed measure(s) : {0}".format(measure)

class PnPAddToReport(ComputePnpResultBase):
    """
    Write PnP report
    """

    def _run(self):
        """
        Run action for steps
        """
        report = self._context.get_info(self._pars.stored_report)
        self._pnp_results.append(report)

        return Global.SUCCESS, ""


class PnPWriteReport(ComputePnpResultBase):
    """
    Write PnP report
    """

    def _run(self):
        """
        Run action for steps
        """
        report_path = self._pnp_results.write(self._pars.calcul_method)
        self._logger.info("Write pnp result as '{0}'".format(report_path))
        return Global.SUCCESS, ""
