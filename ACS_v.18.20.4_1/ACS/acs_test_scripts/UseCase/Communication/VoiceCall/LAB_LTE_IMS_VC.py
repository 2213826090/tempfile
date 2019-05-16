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
:summary: Use case to validate VC over LTE IMS
:since: 07/01/2014
:author: gescoffr
"""

import time

from UseCase.Networking.LAB_LTE_BASE import LabLteBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LabLteImsVc(LabLteBase):

    """
    Lab LTE IMS Voice Call class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LTE base Init function
        LabLteBase.__init__(self, tc_name, global_config)

        # Read callSetupTimeout from Device_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        self._phone_number = \
            str(self._tc_parameters.get_param_value("PHONE_NUMBER"))
        if self._phone_number.upper() == "[PHONE_NUMBER]":
            self._phone_number = str(self._device.get_phone_number())

        # Read CALL_DURATION from test case xml file
        self._call_duration = \
            int(self._tc_parameters.get_param_value("CALL_DURATION"))

        # Read VC_TYPE from testcase xml file
        self._vc_type = str(self._tc_parameters.get_param_value("VC_TYPE"))

        # Read RELEASE_VC_SIDE from testcase xml file
        self._release_vc_type = str(self._tc_parameters.get_param_value("RELEASE_VC_TYPE"))

        # Create cellular network simulator and retrieve 4G voice call and data interfaces
        self._ns_voice_call_4g = self._ns.get_cell_4g().get_voice_call()

        # Instantiate generic UECmd for all use cases
        self._voice_call_api = self._device.get_uecmd("VoiceCall")

    def set_up(self):
        """
        Initialize the test
        """
        # Call LAB_LTE_BASE set_up function
        LabLteBase.set_up(self)

        # Set Cell on
        self._cell_4g.set_cell_on(self._mimo)

        # Phone has to see the cell off!
        self._modem_api.check_cdk_no_registration_bfor_timeout(self._registration_timeout)

        # Flight mode deactivation
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._logger.info("Check network registration status is %s on DUT" %
                          (self._wanted_reg_state))

        self._modem_api.check_cdk_state_bfor_timeout(
            self._wanted_reg_state,
            self._registration_timeout)

        # Set APN for LTE and/or IMS depending on protocol IPv4 or IPv6
        self._set_apn_for_lte_and_ims()

        # Enable Data Usage
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.activate_pdp_context()

        # Get RAT from Equipment
        network_type = self._data_4g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        # Check IMS connection state
        status = self._data_4g.check_ims_connection_state("REG", self._registration_timeout)
        if status == False:
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                  "Failed to reach IMS registration")

        # Wait between two commands sending
        time.sleep(self._wait_btwn_cmd)

        return (Global.SUCCESS, "No errors")
#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # pylint: disable=E1101
        # Disable this pylint error due to Enum class VOICE_CALL_STATE

        # Call GSM VoiceCall base run_test function
        LabLteBase.run_test(self)

        # Release any previous call (Robustness)
        self._voice_call_api.release()

        # Perform MT voice call if requested by the test
        if self._vc_type == "MT":
            # Trigger MT call from the call box
            self._ns_voice_call_4g.mt_originate_call()
            # Wait for state "incoming" before callSetupTimeout seconds
            self._voice_call_api.wait_for_state(
                self._uecmd_types.VOICE_CALL_STATE.INCOMING,
                self._call_setup_time)

            # Answer to that call on the DUT
            self._voice_call_api.answerIMS()
        # Otherwise perform a MO call
        else:
            # Dial using PHONE_NUMBER parameter
            self._voice_call_api.dial(self._phone_number)

        # Check call state "CONNECTED" before callSetupTimeout seconds
        self._ns_voice_call_4g.check_call_connected(self._call_setup_time)

        # Wait for state "active" before callSetupTimeout seconds
        self._voice_call_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.ACTIVE,
            self._call_setup_time)

        # Check call is connected for CALL_DURATION seconds
        self._ns_voice_call_4g.is_voice_call_connected(self._call_duration)

        # Perform Network release of the voice call if requested by the test
        if self._vc_type == "NR":
            self._ns_voice_call_4g.voice_call_network_release()
        # Otherwise do a Mobile Release
        else:
            # Mobile Release call
            self._voice_call_api.release()

        # Check voice call state is "released"
        self._logger.info("Wait 5 sec to check VC is released on Callbox")
        time.sleep(5)
        self._ns_voice_call_4g.is_voice_call_idle()

        # Check voice call state is "no_call" on DUT
        time.sleep(self._wait_btwn_cmd)
        self._voice_call_api.wait_for_state(
            self._uecmd_types.VOICE_CALL_STATE.NOCALL,
            self._call_setup_time)
        # pylint: enable=E1101
        return (Global.SUCCESS, "No errors")
