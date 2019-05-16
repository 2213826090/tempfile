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

:since: 01/19/2012
:author: jreynaud
"""
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global


class AndroidUiTestWidiScan(UseCaseBase):

    """
    The new implementation of Android UI test use case for a WiDi Scan.
    """

    RAISE_EXCEPTIONS_POLICY = "RAISE_EXCEPTION"
    """
    The I{raise exception} management policy,
    when an UI action related error occurs,
    an exception is raised.
    """

    LOG_ERROR_MANAGEMENT_POLICY = "LOG_WARNING"
    """
    An alternate error management policy, simply
    logs warnings on errors.
    """

    DEFAULT_ERROR_MANAGEMENT_POLICY = LOG_ERROR_MANAGEMENT_POLICY
    """
    The error management policy to use by default.
    """

    ALLOWED_ERROR_MANAGEMENT_POLICIES = (
        RAISE_EXCEPTIONS_POLICY,
        LOG_ERROR_MANAGEMENT_POLICY)
    """
    The list of all allowed error management policy names.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call Use Case base init
        UseCaseBase.__init__(self, tc_name, global_config)

        self._error_policy = str(self._tc_parameters.get_param_value("ERROR_MANAGEMENT_POLICY"))

        # Get ue command UI instance
        self._ui_api = self._device.get_uecmd("Ui")

        self._networking_api = self._device.get_uecmd("Networking")

    def set_up(self):
        """
        Set up the test configuration
        """

        # Call UseCaseBase Set Up
        UseCaseBase.set_up(self)

        # Initialize UI object
        self._ui_api.init(self._error_policy)

        # Unlock screen
        self._ui_api.run_operation_set("phone_unlock")

        # Go to HOME page
        self._ui_api.run_operation_set("go_home")

        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the test
        """

        # Call UseCaseBase Set Up
        UseCaseBase.run_test(self)

        # Click to Widi to start scan
        self._ui_api.run_operation_set("widi_scan")

        # Retrieve Widi Scan Result
        widi_devices_list = self._networking_api.widi_scan_devices()

        if not widi_devices_list:
            return Global.FAILURE, "No available devices found !"
        else:
            for device in widi_devices_list:
                self._logger.info("WiDi Device found: " + str(device.name) +
                                  " in " + device.adapter + " adapter")

        # Return the result
        return Global.SUCCESS, "No errors"

    def tear_down(self):
        """
        End and dispose the test
        """

        # Call UseCaseBase Tear down
        UseCaseBase.tear_down(self)

        # Click again to Widi to stop scan
        self._ui_api.run_operation_set("widi_scan")

        # Go to HOME page
        self._ui_api.run_operation_set("go_home")

        # Release the Ui instance.
        self._ui_api.release()

        return Global.SUCCESS, "No errors"
