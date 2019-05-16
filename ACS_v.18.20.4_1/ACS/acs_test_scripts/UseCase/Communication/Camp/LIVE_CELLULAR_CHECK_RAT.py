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
:summary: Use Case Cellular Check RAT
:since: 18/04/2013
:author: asebbanx
"""
# Module name follows ACS conventions
# pylint: disable=C0103

from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities import RegistrationUtilities as reg_utils
from UtilitiesFWK.Utilities import Global, str_to_bool
import time


class LiveCellularCheckRat(UseCaseBase):
    """
    Live Cellular Check RAT.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Initialize basic attributes for the Use Case
        self._expected_nw_type_int = None

        self._initial_preferred_nw_type_int = None
        self._preferred_nw_type_int = None
        self._preferred_nw_type_str = None

        self._fail_on_error = False
        self._registration_timeout = None

        # Instantiate generic UECmd for camp
        self._modem_api = self._device.get_uecmd("Modem")
        self._networking_api = self._device.get_uecmd("Networking")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize this test.
        """
        # Call inherited method
        UseCaseBase.set_up(self)

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read network type parameter
        expected_nw_type = self._tc_parameters.get_param_value(
            "EXPECTED_NETWORK_TYPE")

        if expected_nw_type not in ("", None):
            self._expected_nw_type = expected_nw_type
        else:
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                "Invalid parameter value for 'EXPECTED_NETWORK_TYPE': <%s>."
                % str(expected_nw_type))

        # Read preferred network type parameter
        preferred_nw_type = self._tc_parameters.get_param_value(
            "PREFERRED_NETWORK_TYPE")

        if preferred_nw_type not in ("", None):
            self._preferred_nw_type = preferred_nw_type
        else:
            raise AcsConfigException(
                AcsConfigException.INVALID_PARAMETER,
                "Invalid parameter value for 'PREFERRED_NETWORK_TYPE': <%s>."
                % str(preferred_nw_type))

        # Read the fail on error parameter
        fail_on_error = self._tc_parameters.get_param_value("FAIL_ON_ERROR")
        if fail_on_error not in ("", None):
            self._fail_on_error = str_to_bool(fail_on_error)

        # Backup the initial preferred network type value
        self._logger.debug("Backup initial value of preferred network type ...")
        self._initial_preferred_nw_type_int = self._dut_config.get("defaultPreferredNetwork")

        # Force the DUT to use a Network
        self._configure_preferred_network_type(self._preferred_nw_type)

        # Tell ACS that everythin was OK
        return Global.SUCCESS, "No errors."

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test.
        """
        # Call inherited method
        UseCaseBase.run_test(self)

        error_occured = False
        output_message = ""

        # Log the current network preference
        preferred_rat = self._networking_api.get_preferred_network_type()
        log_message = "Current network preference is %s ." % preferred_rat
        self._logger.debug(log_message)

        try:
            self._modem_api.check_rat_with_pref_network(self._preferred_nw_type, self._registration_timeout)
        except DeviceException as de:
            if self._fail_on_error:
                self._logger.error(de.get_error_message())
                raise de
            else:
                error_occured = True
                output_message = de.get_error_message()
                self._logger.warning(output_message)

        if not error_occured:
            nw_type = self._modem_api.get_network_type()
            if self._expected_nw_type == nw_type:
                output_message = "The DUT is camped on network %s as expected with network preference %s" \
                                            % (str(nw_type), self._preferred_nw_type)
            else:
                output_message = "The DUT is camped on network %s instead of expected %s but compatible with network type preference %s" \
                                            % (str(nw_type), self._expected_nw_type, self._preferred_nw_type)

            self._logger.info(output_message)

        # Return the verdict
        return Global.SUCCESS, output_message

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Dispose the test.
        """
        # Call inherited method
        UseCaseBase.tear_down(self)

        # Restore initial value
        if self._initial_preferred_nw_type_int not in ("", None):
            self._configure_preferred_network_type(self._initial_preferred_nw_type_int)

        # Return the verdict
        return Global.SUCCESS, "No errors"

    def _configure_preferred_network_type(self, preferred_nw_type):
        """
        Configure the preferred network type on DUT, then reboot the modem power
        to ensure that the new parameter has been taken into account

        :param preferred_nw_type: The preferred network type value to use
        :type  preferred_nw_type: str

        .. seealso:: UECmd.Imp.Android.Common.Networking.Networking.set_preferred_network_type

        :rtype: None
        """
        self._logger.info(
            "Configuring preferred network type on dut to %s" % str(preferred_nw_type))

        # Configure network type on dut
        self._networking_api.set_preferred_network_type(preferred_nw_type)

        time.sleep(self._wait_btwn_cmd)

        # Power UP modem
        self._modem_api.set_modem_power(1)

        time.sleep(self._wait_btwn_cmd)
