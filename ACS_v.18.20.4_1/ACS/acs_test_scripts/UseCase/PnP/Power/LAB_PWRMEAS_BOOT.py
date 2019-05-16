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
:summary: This file implements the Boot UC for PnP purpose
:since: 14/02/2013
:author: pbluniex
"""
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global


class LabPwrMeasBoot(UseCaseBase):

    """
    Lab System Boot UC Class.
    """
    _boot_timeout = None
    _settledown_duration = None
    _patlib = None

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)
        self._handle_power_cycle = False

        # Get bootTimeout value either from parameter file
        # or default value from phone catalog
        self._boot_timeout = self._tc_parameters.get_param_value("BOOT_TIMEOUT", default_cast_type=int)

        # Get settledown duration value either from parameter file
        # or default value from phone catalog
        self._settledown_duration = self._tc_parameters.get_param_value("SETTLEDOWN_DURATION", default_cast_type=int)

        # Check if calibration is requested
        do_calibration = self._tc_parameters.get_param_value("DAQ_CALIBRATION")
        if do_calibration and do_calibration.lower() in ["yes", "true", "1"]:
            self._do_calibration = 1

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Run UC base run_test
        UseCaseBase.run_test(self)

        # Check current state of the phone and act accordingly
        if self._device.is_booted():
            # Phone is booted and connected to host PC
            (return_code, return_msg) = self._device.switch_off()

            if return_code != Global.SUCCESS:
                # Return the error from switch off
                return return_code, return_msg

        if self._do_calibration:
            self._patlib = self._em.get_power_analyzer_tool("POWER_ANALYZER_TOOL")
            self._logger.info("Run PowerAnalyzerTool calibration")
            self._patlib.run_self_calibration()

        return self._device.switch_on(self._boot_timeout,
                                      self._settledown_duration)

    def tear_down(self):
        """
        End and dispose the test
        """

        # Force home screen
        keyevent = self._device.get_uecmd("KeyEvent")
        keyevent.back()
        keyevent.home()

        return Global.SUCCESS, "No error"
