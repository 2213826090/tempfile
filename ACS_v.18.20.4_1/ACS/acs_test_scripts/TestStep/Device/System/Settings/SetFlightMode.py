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
:summary: This file implements a Test Step to set / reset the flight mode
:since:15/03/2013
:author: fbongiax
"""
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from UtilitiesFWK.Utilities import TestConst
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.TestStep.Utilities.Checks.CheckContextInfoExists import CheckContextInfoExists


class SetFlightMode(DeviceTestStepBase):
    """
    Sets or reset the flight mode
    Attributes:
        FLIGHT_MODE (on|1|true|yes|off|0|false|no|restore): indicate how to set (or reset)
        the flight mode.
        Before the flight mode gets set its current value is saved in the context.
        If FLIGHT_MODE == "restore" the previous status from the context gets set back.
    """

    # Constants
    STR_ARG_FLIGHT_MODE = "FLIGHT_MODE"

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        # get networking cmd.
        self._networking_api = None
        self._factory = factory

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        # get networking cmd.
        self._networking_api = self._device.get_uecmd("Networking")

        # If not set assume it's default value is off (0)
        if self._pars.flight_mode != TestConst.STR_RESTORE:
            # Set flight mode
            old_state = self._set_flight_mode(self._pars.flight_mode == TestConst.STR_ENABLE)
            # Save flight mode in the context
            self._context.set_info(self.STR_ARG_FLIGHT_MODE, old_state)
            self.ts_verdict_msg = "Flight mode set to '{0}'".format(self._pars.flight_mode)
        else:
            # Check flight mode exists in the context
            CheckContextInfoExists(self._conf,
                                   self._global_conf,
                                   {CheckContextInfoExists.STR_KEY: self.STR_ARG_FLIGHT_MODE},
                                   self._factory).run(context)
            # Get the flight mode from the context
            old_state = self._context.get_info(self.STR_ARG_FLIGHT_MODE)
            # Restore the flight mode
            self._set_flight_mode(old_state)
            self.ts_verdict_msg = "Flight mode status restored into '{0}'".format(old_state)

    def _set_flight_mode(self, mode):
        """
        Sets the flight mode to mode (if not already set)

        :type mode: int
        :param mode: the value to set flight mode to
        """

        # Get original flight mode
        original_flight_mode = self._networking_api.get_flight_mode()
        # Flight mode
        if mode != original_flight_mode:
            self._networking_api.set_flight_mode(mode)
            if mode != self._networking_api.get_flight_mode():
                msg = "set flight mode failure"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        return original_flight_mode
