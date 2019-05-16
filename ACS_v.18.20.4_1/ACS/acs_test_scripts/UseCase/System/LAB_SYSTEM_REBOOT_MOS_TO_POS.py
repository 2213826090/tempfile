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
:summary: This file implements the reboot from MOS to POS UC
:since: 07/09/2012
:author: asebbanx
"""

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.DeviceException import DeviceException


class LabSystemRebootMosToPos(UseCaseBase):

    """
    Lab System Reboot UC Class.
    This Use Case reboots the DUT from I{MOS} (Main OS) to I{POS} (Provisioning OS).
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        self._boot_timeout = None
        self._settledown_duration = None

        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get bootTimeout value either from parameter file or default value from phone catalog
        if self._tc_parameters.get_param_value("BOOT_TIMEOUT") != "":
            self._boot_timeout = \
                int(self._tc_parameters.get_param_value("BOOT_TIMEOUT"))

        # Get settledown duration value either from parameter file or default value from phone catalog
        if self._tc_parameters.get_param_value("SETTLEDOWN_DURATION") != "":
            self._settledown_duration = \
                int(self._tc_parameters.get_param_value("SETTLEDOWN_DURATION"))

        # Instiate the PhoneSystem UE Command category
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Run UC base run_test
        UseCaseBase.run_test(self)

        # Initialize the return code and message of the test
        return_code = Global.FAILURE
        return_message = "An error occurred."

        # Check current state of the phone and act accordingly
        # We expect the phone to be booted before going any further in our test
        if not self._device.is_booted():
            # Phone is not started, just switch it on!
            self._device.switch_on(
                self._boot_timeout,
                self._settledown_duration)

        # Check the state of the phone as returned by adb
        # We do not want to use the 'get_state' method
        # of the device as it adds some post processing
        # on the state value
        state = self._device.get_state()
        if state not in ("device", "alive"):
            message = "Unexpected phone state: %s. expected 'device' or 'alive'." % state
            self._logger.error(message)
            raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, message)

        # Restart the phone in POS
        rebooted = self._device.reboot(
            mode="POS",
            transition_timeout=self._boot_timeout)
        if rebooted:
            return_code = Global.SUCCESS
            return_message = "Board rebooted successfully."
        else:
            return_code = Global.FAILURE
            return_message = "An error occurred when rebooting the board."

        # Return the verdict and the message
        return return_code, return_message
