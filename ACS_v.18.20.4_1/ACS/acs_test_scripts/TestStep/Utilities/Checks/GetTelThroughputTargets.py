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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.

:summary: This file implements a Test Step for retrieving the telephony throughput targets
:author: jduran4x
:since 12/09/2014
:organization: INTEL QCTV
"""

from Core.TestStep.TestStepBase import TestStepBase
from Device.DeviceManager import DeviceManager
from acs_test_scripts.Utilities.CommunicationUtilities import TelephonyConfigsParser
from ErrorHandling.AcsBaseException import AcsBaseException


class GetTelThroughputTargets(TestStepBase):
    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        TestStepBase.run(self, context)

        if self._pars.ul_conf.lower() == "none":
            self._pars.ul_conf = None
        if self._pars.dl_conf.lower() == "none":
            self._pars.dl_conf = None
        parser = TelephonyConfigsParser("Throughput_Targets")
        sections = self._pars.section.upper().split(";")
        # there is at least 1 section
        if sections[0] in ("GPRS", "EGPRS"):
            throughput_targets = parser.parse_gprs_egprs_theoretical_targets(self._pars.section,
                                                                             self._pars.multislot,
                                                                             self._pars.dl_conf,
                                                                             self._pars.ul_conf)
        elif sections[0] == "WCDMA":
            throughput_targets = parser.parse_wcdma_theoretical_targets(self._pars.dl_conf,
                                                                        self._pars.ul_conf)
        elif sections[0] in ("HSPA", "HSDPA"):
            # when HSDPA, the uplink need to be given. so expecting 2 sections
            if len(sections) > 1:
                hsupa_cat = None
                ul_rab = None
                if sections[1] == "HSUPA":
                    hsupa_cat = self._pars.ul_conf
                elif sections[1] == "WCDMA":
                    ul_rab = self._pars.ul_conf
                else:
                    raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                           "expected HSUPA or WCDMA for uplink throughput targets determination")
                throughput_targets = parser.parse_hspa_theoretical_targets(self._pars.dl_conf, hsupa_cat, ul_rab)
            else:
                raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                       "with %s" % sections[0]
                                       + ", a second section name for uplink targets determination is expected"
                                       "(HSUPA or WCDMA).")
        elif sections[0] == "TD-SCDMA":
            throughput_targets = parser.parse_tdscdma_theoretical_targets()
        elif sections[0] == "LIVE_WCDMA":
            throughput_targets = parser.parse_live_wcdma_targets()
        elif sections[0] == "LTE":
            if self._pars.dl_conf is not None:
                lte_cat = self._pars.dl_conf
            elif self._pars.ul_conf is not None:
                lte_cat = self._pars.ul_conf
            else:
                raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                       "either DL_CONF or UL_CONF should be set.")
            throughput_targets = parser.parse_lte_theoretical_targets(lte_cat)
        elif sections[0] == "LTE_TDD":
            if self._pars.dl_conf is not None:
                lte_cat = self._pars.dl_conf
                direction = "DL"
            elif self._pars.ul_conf is not None:
                lte_cat = self._pars.ul_conf
                direction = "UL"
            else:
                raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                       "either DL_CONF or UL_CONF should be set.")
            throughput_targets = parser.parse_lte_tdd_targets(lte_cat, direction)
        else:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "invalid section name.")

        # Get DUT configuration
        dut_config = DeviceManager().get_device_config("PHONE1")
        # Get customized failure Targets
        failure_targets = \
            str(self._tc_parameters.get_param_value("FAILURE_TARGETS", "FUTE"))
        # Detect if it is a KPI test or not
        kpi_test = self._tc_parameters.get_param_value("KPI_TEST", False, "str_to_bool")

        # Update the failure targets
        throughput_targets.set_failure_throughput_from_config(dut_config,
                                                              failure_targets,
                                                              kpi_test,
                                                              self._name)


        context.set_info(self._pars.save_throughput_as+":DL_TARGET", throughput_targets.dl_target.value)
        context.set_info(self._pars.save_throughput_as+":DL_FAILURE", throughput_targets.dl_failure.value)
        context.set_info(self._pars.save_throughput_as+":UL_TARGET", throughput_targets.ul_target.value)
        context.set_info(self._pars.save_throughput_as+":UL_FAILURE", throughput_targets.ul_failure.value)
        context.set_info(self._pars.save_throughput_as+":DL_UNIT", throughput_targets.dl_target.unit)
        context.set_info(self._pars.save_throughput_as+":UL_UNIT", throughput_targets.ul_target.unit)