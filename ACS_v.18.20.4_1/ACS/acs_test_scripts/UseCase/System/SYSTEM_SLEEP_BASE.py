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
:summary: This file implements the sleep base UC
:since: 07/06/2011
:author: vtinelli
"""

import os
from acs_test_scripts.Utilities.PnPUtilities import PnPTargets
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


class SystemSleepBase(UseCaseBase):

    """
    sleep base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call UseCase base constructor
        UseCaseBase.__init__(self, tc_name, global_config)

        # Sleep time
        if self._tc_parameters.get_param_value("DURATION") is not None:
            self._duration = \
                int(self._tc_parameters.get_param_value("DURATION"))

        self._sleep_mode = self._tc_parameters.get_param_value("MODE", "s3")
        audio_file = self._tc_parameters.get_param_value("AUDIO_FILE", None)
        self._audio_file = None

        if audio_file:
            self._audio_file = self._device.get_device_os_path().join(self._device.multimedia_path + audio_file)

        if self._audio_file is None and self._sleep_mode == "lpmp3":
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Audio file is not set in test case parameter")
        self._use_flightmode = self._tc_parameters.get_param_value("USE_FLIGHT_MODE", "false")
        self._settle_time = int(self._tc_parameters.get_param_value("SETTLE_TIME", 0, int))

        # Get UECmdLayer
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")
        self._sleep_mode_api = self._device.get_uecmd("SleepMode")
        self._networking_api = self._device.get_uecmd("Networking")
        self._residency_api = self._device.get_uecmd("Residencies")

        # Retrieve targets
        self._failure_file = os.path.join(self._execution_config_path,
                                          self._device.get_config("FailureFile"))

    def _residency_verdict(self, residency_spent):
        """
        Compute residency verdict
        """
        pnptargets = PnPTargets(self._failure_file)
        target = pnptargets.get(self._dut_config.get("Name"),
                                os.path.basename(self.get_name()),
                                "failure")
        if len(target) == 0:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Residency targets is not set. "
                                     "Device residency is %f" % residency_spent)

        if len(target) == 1 and residency_spent < target[0]:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Residency target not reached: %f (%s requested)" %
                                  (residency_spent, str(target)))

        if len(target) == 2 and (residency_spent <= target[0] or
                                 residency_spent >= target[1]):
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Residency: %f not in range %s" %
                                  (residency_spent, str(target)))

        status = "Residency: %f, FailureResidency: %s" % (residency_spent, str(target))
        return Global.SUCCESS, status

    def set_up(self):
        """
        Set up the test configuration
        """

        # Call use case base Setup function
        UseCaseBase.set_up(self)

        # Enable flight mode if needed
        if self._use_flightmode == "true":
            self._networking_api.set_flight_mode("on")

        # Check that io card is present
        if self._io_card is None:
            # We need to unplug/plug the device with iocard
            error_msg = "This use case requires io card to be executed !"
            raise AcsConfigException(AcsConfigException.INVALID_BENCH_CONFIG, error_msg)

        # Ensure to not have any previous lock
        self._phonesystem_api.clear_pwr_lock()

        self._sleep_mode_api.init(self._sleep_mode, self._settle_time, self._audio_file)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """

        # Call power measurement base tear_down function
        UseCaseBase.tear_down(self)

        # Clear sleep mode
        self._sleep_mode_api.clear()

        # Disable flight mode as it was enabled
        if self._use_flightmode == "true":
            self._networking_api.set_flight_mode("off")

        return Global.SUCCESS, "No errors"
