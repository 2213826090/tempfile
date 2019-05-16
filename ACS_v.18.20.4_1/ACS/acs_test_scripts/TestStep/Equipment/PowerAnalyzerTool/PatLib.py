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
@summary: This file implements a Test Step to manage PatLib
@since 17/11/2014
@author: pbluniex
"""
import os
import time

from acs_test_scripts.Utilities.PnPUtilities import PatLibUtil
from acs_test_scripts.Utilities.PnPUtilities import get_math_function
from Core.TestStep.EquipmentTestStepBase import EquipmentTestStepBase
from Core.Report.SecondaryTestReport import SecondaryTestReport
from Device.DeviceManager import DeviceManager
from ErrorHandling.AcsToolException import AcsToolException
from UtilitiesFWK.Utilities import Global

class PatLibBase(EquipmentTestStepBase):
    """
    Manage PatLib
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        EquipmentTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._dut_config = DeviceManager().get_device_config("PHONE1")
        self._patlib = None
        self._report_tree = global_conf.campaignConfig.get("campaignReportTree")
        self._patlib_util = None

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
        Specific run for test step
        """
        raise AcsToolException(AcsToolException.FEATURE_NOT_IMPLEMENTED,
                               "_run is not implemented for this test step")

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        EquipmentTestStepBase.run(self, context)
        self._patlib_util = PatLibUtil(self._equipment_parameters,
                                       self._dut_config.get("Name"),
                                       os.path.basename(self._conf.get_name()),
                                       self._logger)
        #self._patlib = self._context.get_info(self._pars.stored_pat_lib_instance)
        self._patlib = self._equipment_manager.get_power_analyzer_tool(self._pars.eqt)
        self._run()

class PatLibInit(PatLibBase):
    """
    Initialisation of PatLib
    """

    def _run(self):
        """
        Init patlib equipment
        """
        self._patlib.init(self._patlib_util.conf_file)


class PatLibAcquisition(PatLibBase):
    """
    Start/Stop acquisition with PatLib
    """
    def _run(self):
        """
        Run stop acquisition
        """
        if self._patlib is None:
            raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR,
                                  "Cannot do acquisition, PatLib has not been initialized")
        acquisition_status = self._context.get_info("AcquisitionStatus")

        if self._pars.action.lower() == "start":
            # check acquisition status
            if acquisition_status and acquisition_status != "Stopped":
                raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR,
                                       "Cannot start acquisition, wrong acquisition status : %s" % acquisition_status)
                # acquisition status is correct, start it!
            self._patlib.start_acquisition(True)
            # store in context a variable about acquisition status
            self._context.set_info("AcquisitionStatus", "Started")
        elif self._pars.action.lower() == "stop":
            # check acquisition status
            if not acquisition_status:
                raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR,
                                       "Cannot stop, no acquisition previously started!")
            elif acquisition_status.lower() != "started":
                raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR,
                                       "Cannot stop acquisition, wrong initial acquisition status : %s" % acquisition_status)
            # acquisition status is correct, stop it!
            self._patlib.stop_acquisition()
            self._context.set_info("AcquisitionStatus", "Stopped")


class PatLibGetAcquisition(PatLibBase):
    """
    Start/Stop acquisition with PatLib
    """
    def _run(self):
        """
        Run stop acquisition
        """
        if self._patlib is None:
            raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR,
                                  "Cannot do acquisition, PatLib has not been initialized")
        acquisition_status = self._context.get_info("AcquisitionStatus")

        if acquisition_status == "Stopped":
            # retrieve all measurements of previous acquisition
            measures = self._patlib.get_measurements()
            if not measures:
                raise AcsToolException(AcsToolException.OPERATION_FAILED, "Cannot retrieve any measure")
            # retrieve all measures according to rail name
            measures = [measure
                        for measure in measures if measure.get("name") == self._pars.measure_name]
            if not measures:
                raise AcsToolException(AcsToolException.OPERATION_FAILED,
                                       "Cannot retrieve measure for {0}".format(self._pars.measure_name))
            # retrieve all measures according to measure type (average or max or min)
            measures = [measure[self._pars.measure_type] for measure in measures
                        if measure.get(self._pars.measure_type) is not None]
            if not measures:
                raise AcsToolException(AcsToolException.OPERATION_FAILED,
                                       "Cannot retrieve measure {0} for {1}".format(self._pars.measure_type,
                                                                                    self._pars.measure_name))
            self._context.set_info(self._pars.save_as, measures[-1])
        else:
            raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR,
                                   "Cannot get acquisition: Status %s instead of Stopped " % acquisition_status)


class PatLibReportData(PatLibBase):
    """
    Report patlib acquisition
    """

    def _run(self):
        """
        Run stop acquisition
        """
        if self._patlib is None:
            raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR,
                                   "Cannot report acquisition, PatLib has not been initialized")
        report = self._patlib.report(self._pars.do_power_calculation)
        self._context.set_info(self._pars.save_as, report)


class PatLibExportData(PatLibBase):
    """
    Export patlib acquisition
    """
    def _run(self):
        """
        Run stop acquisition
        """
        if self._patlib is None:
            raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR,
                                   "Cannot report acquisition, PatLib has not been initialized")
        if self._pars.save_raw_data:
            tc_date = time.strftime("%Y-%m-%d %H:%M:%S")

            self._patlib_util.clear_data(self._patlib)

            self._logger.info("Save raw data")
            _, result_location = self._patlib_util.export(self._patlib,
                                                          self._report_tree,
                                                          tc_date)
            self._logger.info("Generate power graph")
            pwr_graph_dat = self._patlib_util.export_plot(self._patlib,
                                                          self._report_tree,
                                                          tc_date)
            self._logger.info("Compute power indicators")
            pwr_ind = self._patlib_util.export_indicators(self._patlib,
                                                          self._report_tree,
                                                          tc_date)

            pnp_results = self._context.get_info(self._pars.stored_pnp_result)
            if not pnp_results:
                raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR,
                                       "Context key '{0}' does not content initialized PnpResult object".format(
                                           self._pars.stored_pnp_result))
            pnp_results.append(pwr_graph_dat)
            pnp_results.append(pwr_ind)

            self._logger.info("Raw data saved as '{0}'".format(result_location))
            self._context.set_info(self._pars.save_as, result_location)
        else:
            self._logger.info("No raw data will be saved")


class PnPReportPowerVerdict(PatLibBase):
    """
    Export patlib acquisition
    """
    def _run(self):
        """
        Compute power verdict according to targets
        """
        if self._patlib is None:
            raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR,
                                   "Cannot report acquisition, PatLib has not been initialized")

        pnp_results = self._context.get_info(self._pars.stored_pnp_result)
        if not pnp_results:
            raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR,
                                   "Context key '{0}' does not content initialized PnpResult object".format(
                                       self._pars.stored_pnp_result))

        pnp_results.update({"verdict_rail": str(self._patlib_util.power_rail)})
        # initialize store data to false
        self._context.set_info(self._pars.store_raw_data, "False")

        verdict_name = "POWER_MEASURE({0})"
        # retrieve measure information
        measures = pnp_results.retrieve_power_measures(self._patlib_util.power_rail)
        if not measures:
            raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR, "Cannot retrieve measure information")

        measure = None
        verdict_msg = ""

        for rail in measures:
            output_measure = ""
            for value_type, value in measures[rail][-1].items():
                if value_type == "average":
                    measure = value
                output_measure += "{0}={1};".format(value_type, value)
            self.update_report(Global.SUCCESS, verdict_name.format(rail), output_measure)
            if verdict_msg:
                verdict_msg += " | "
            verdict_msg += verdict_name.format(rail) + ":" + output_measure

        if verdict_msg:
            pnp_results.update({"power_msg": verdict_msg})

        if measure is None:
            raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR, "Cannot retrieve average power measure")

        if pnp_results.check_rail_verdict(measure) == Global.FAILURE:
            self._context.set_info(self._pars.store_raw_data, "True")

class PnPComputePowerVerdict(PatLibBase):
    """
    Export patlib acquisition
    """
    def _compute_score(self, measures, rail):
        """
        Compute result according to specified operation
        """
        functor = get_math_function(self._pars.operation)
        raw_results = []
        if not rail in measures:
            raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR,
                                   "Measure does not contains '{0}' residency information".format(rail))
        for measure in measures[rail]:
            value = measure.get("average")
            if value is None:
                raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR,
                                       "Cannot retrieve residency {0}".format(rail))
            raw_results.append(value)
        computed_result = str(functor(raw_results))
        return computed_result

    def _run(self):
        """
        Compute power verdict according to targets
        """
        if self._patlib is None:
            raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR,
                                   "Cannot report acquisition, PatLib has not been initialized")

        pnp_results = self._context.get_info(self._pars.stored_pnp_result)
        if not pnp_results:
            raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR,
                                   "Context key '{0}' does not content initialized PnpResult object".format(
                                       self._pars.stored_pnp_result))

        pnp_results.update({"verdict_rail": str(self._patlib_util.power_rail)})

        verdict_name = "{0}#".format(self._pars.operation)
        verdict_name += "POWER_MEASURE({0})"
        # retrieve measure information
        measures = pnp_results.retrieve_power_measures(self._patlib_util.power_rail)
        if not measures:
            raise AcsToolException(AcsToolException.PROHIBITIVE_BEHAVIOR, "Cannot retrieve measure information")

        rail_to_report = self._patlib_util.power_rail
        # compute the score
        measure = self._compute_score(measures, rail_to_report)

        code = pnp_results.check_rail_verdict(float(measure))

        self.update_report(code, verdict_name.format(rail_to_report), measure)

        if code == Global.FAILURE:
            verdict_value = "FAIL"
        else:
            verdict_value = "PASS"

        pnp_results.update({"power_verdict": verdict_value})

        if not self._pars.verdict_criteria or "both" in str(self._pars.verdict_criteria).lower():
            if "fail" not in pnp_results.get_value("verdict").lower():
                pnp_results.update({"verdict": verdict_value})
        elif "power" in str(self._pars.verdict_criteria).lower():
            pnp_results.update({"verdict": verdict_value})
