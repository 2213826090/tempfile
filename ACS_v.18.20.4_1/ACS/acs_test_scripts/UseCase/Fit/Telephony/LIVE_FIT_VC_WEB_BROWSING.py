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
:summary: This file implements the loading of a web page while DUT is in voice
call.
:since: 02/10/2013
:author: morchex
"""
import time
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException

class LiveFitVoiceCallWebBrowsing(UseCaseBase):
    """
    Live web browsing and voice call at the same time
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        self._activate_data = None

        # Read the  WAB PAGE LOADINGTIMEOUT from UseCase xml Parameter
        self.__webpage_loading_timeout = self._tc_parameters.get_param_value("WEBPAGE_LOADING_TIMEOUT")

        if isinstance(self.__webpage_loading_timeout, str) and self.__webpage_loading_timeout.isdigit():
            self.__webpage_loading_timeout = int(self.__webpage_loading_timeout)
        else:
            self.__webpage_loading_timeout = None

        # Read REGISTRATION TIMEOUT
        self.__registration_timeout = self._tc_parameters.get_param_value("REGISTRATION_TIMEOUT")
        if isinstance(self.__registration_timeout, str) and self.__registration_timeout.isdigit():
            self.__registration_timeout = int(self.__registration_timeout)
        else:
            self.__registration_timeout = None

        # Read PHONE_NUMBER from testcase xml parameters
        if (self._tc_parameters.get_param_value("PHONE_NUMBER") not in (None, '')) \
            and str(self._tc_parameters.get_param_value("PHONE_NUMBER")).isdigit():
            self.__phone_number = self._tc_parameters.get_param_value("PHONE_NUMBER")
        else:
            self.__phone_number = str(self._device.get_phone_number())

        # Read BROWSER_TYPE from testcase xml parameters
        self.__browser_type = self._tc_parameters.get_param_value("BROWSER_TYPE").lower()

        # Read WEBSITE_URL from testcase xml parameters
        self.__website_url = self._tc_parameters.get_param_value("WEBSITE_URL")

        # Read PREFERRED_NETWORK_TYPE from xml parameters
        self.__network_pref = self._tc_parameters.get_param_value("PREFERRED_NETWORK_TYPE")

        # Get UECmdLayers
        # Get UECmdLayer for VoiceCall
        self.__voicecall_api = self._device.get_uecmd("VoiceCall")

        # Get UECmdLayer for Modem
        self.__modem_api = self._device.get_uecmd("Modem")

        # GET UECmdLayer for networking
        self.__networking_api = self._device.get_uecmd("Networking")

        self.__initial_pref_network = None

        self.__initial_pdp_context_status = None

        self.__initial_flight_mode_state = None

        self.__network_type_updated = False

        #------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Chech initial fligh mode state
        self.__initial_flight_mode_state = self.__networking_api.get_flight_mode()

        # Set flight mode off if it's initially activated
        if self.__initial_flight_mode_state == 1:
            self.__networking_api.set_flight_mode("off")
        time.sleep(self._wait_btwn_cmd)

        # Store the initial PDP context status
        self.__initial_pdp_context_status = self.__networking_api._get_pdp_context_status()
        time.sleep(self._wait_btwn_cmd)

        # Activate PDP context status
        self.__networking_api.activate_pdp_context(check=False)
        time.sleep(self._wait_btwn_cmd)

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml (Non blocking
        # for this test if function isn't implemented on CDK)
        self.__modem_api.check_cdk_registration_bfor_timeout(self.__registration_timeout)

        # Activate preferred network
        self.__initial_pref_network = self._dut_config.get("defaultPreferredNetwork")
        time.sleep(self._wait_btwn_cmd)
        if self.__network_pref is None:
            # If there is no Network preference in the testcase.
            self._logger.warning("No preferred network set in the testcase,"
                                 " will use the one currently set on the"
                                 " phone: %s" % str(self.__initial_pref_network))
        elif self.__networking_api.is_preferred_network_type_valid(self.__network_pref):
            # Setting the DUT preferred network type to the one specified
            # in the TC.
            self.__networking_api.set_preferred_network_type(self.__network_pref)
            time.sleep(self._wait_btwn_cmd)
            # Check the DUT is camped on a compatible network with the selected
            # preferred network.
            self.__modem_api.check_rat_with_pref_network(self.__network_pref, self.__registration_timeout)
            time.sleep(self._wait_btwn_cmd)

            self.__network_type_updated = True
        else:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Unknown network type: %s"
                                     % self.__network_pref)

        # If website url is not specified in testcase.
        if self.__website_url is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "No website url set in the  testcase")

        # Start the call
        self.__voicecall_api.dial(self.__phone_number)
        time.sleep(self._wait_btwn_cmd)

        return (Global.SUCCESS, "No errors")

    #------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        # Browse the web URl
        (self._error.Code, self._error.Msg) = self.__networking_api.open_web_browser(self.__website_url,
                                                                                    self.__browser_type,
                                                                                    self.__webpage_loading_timeout)
        time.sleep(self._wait_btwn_cmd)

        # Check call is active
        self.__voicecall_api.check_state(self._uecmd_types.VOICE_CALL_STATE.ACTIVE)
        time.sleep(self._wait_btwn_cmd)

        return (self._error.Code, self._error.Msg)

        #------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        # Close web browser
        self.__networking_api.close_web_browser(self.__browser_type)
        time.sleep(self._wait_btwn_cmd)

        # Release the call
        self.__voicecall_api.release()
        time.sleep(self._wait_btwn_cmd)

        # Put the PDP_CONTEXT on his original state
        if self.__initial_pdp_context_status in ("0", "2"):
            # Then desactivate PDP context status
            self.__networking_api.deactivate_pdp_context()
            time.sleep(self._wait_btwn_cmd)

        # Set initial network state if it has been updated
        if self.__network_type_updated:
            self.__networking_api.set_preferred_network_type(self.__initial_pref_network)
            time.sleep(self._wait_btwn_cmd)

        # Set the initial flight mode
        self.__networking_api.set_flight_mode(self.__initial_flight_mode_state)
        time.sleep(self._wait_btwn_cmd)

        return (Global.SUCCESS, "No errors")
