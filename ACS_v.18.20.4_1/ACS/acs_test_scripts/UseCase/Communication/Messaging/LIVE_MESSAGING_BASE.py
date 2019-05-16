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
:summary: This file implements Live SMS Base
:since: 29/03/2010
:author: sfusilie
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException

class LiveMessagingBase(UseCaseBase):

    """
    Live SMS/MMS base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        UseCaseBase.__init__(self, tc_name, global_config)

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Get TC Parameters
        self._destination_number = \
            str(self._tc_parameters.get_param_value("DESTINATION_NUMBER"))
        # Check if the sim phone number is used
        if self._destination_number.upper() == "[PHONE_NUMBER]":
            self._destination_number = self._device.get_phone_number()

        self._message = \
            str(self._tc_parameters.get_param_value("MESSAGE_CONTENT", ""))

        # Read optional parameter PREFERRED_NETWORK_TYPE from xml parameters
        self._network_pref = self._tc_parameters.get_param_value("PREFERRED_NETWORK_TYPE", None)

        if self._network_pref:
            self._network_pref = self._network_pref.upper()

        self._initial_pref_network = None

        # Retrieve ue commands instances
        self._sms_api = self._device.get_uecmd("SmsMessaging")
        self._mms_api = self._device.get_uecmd("MmsMessaging")
        self._networking_api = self._device.get_uecmd("Networking")
        self._modem_api = self._device.get_uecmd("Modem")
#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        time.sleep(self._wait_btwn_cmd)

        # Clear old SMS (Non blocking for this test if function isn't
        # implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._sms_api.delete_all_sms()

        time.sleep(self._wait_btwn_cmd)

        # There is a preferred network to set, backup initial, then set configured one
        if self._network_pref is not None:
            self._initial_pref_network = self._dut_config.get("defaultPreferredNetwork")
            time.sleep(self._wait_btwn_cmd)

            if self._networking_api.is_preferred_network_type_valid(self._network_pref):
                # Setting the DUT preferred network type to the one specified
                # in the TC.
                self._networking_api.set_preferred_network_type(self._network_pref)
                time.sleep(self._wait_btwn_cmd)

                # Check the DUT is camped on a compatible network with the selected
                # preferred network.
                self._modem_api.check_rat_with_pref_network(self._network_pref, self._registration_timeout)
                time.sleep(self._wait_btwn_cmd)
            else:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "Unknown network type: %s" % self._network_pref)

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """

        # Call use case base tear_down function
        UseCaseBase.tear_down(self)

        # Clear old SMS (Non blocking for this test if function isn't
        # implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._sms_api.delete_all_sms()

        # Set initial network state
        if self._initial_pref_network is not None:
            self._networking_api.set_preferred_network_type(self._initial_pref_network)
            time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No errors"
