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
call on HSPA only.
:since: 14/03/2013
"""

from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.UseCase.Networking.LAB_HSPA_BASE import LabHspaBase
from UtilitiesFWK.Utilities import Global


class LabFitHspaVcWebBrowsing(LabHspaBase):

    """
    Lab Hspa network Ftp and voice call at the same time
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        self._ftp_data_transfer_state_timeout = 10

        # Call LabMobilityBase Init function
        LabHspaBase.__init__(self, tc_name, global_config)

        # Set ps_data_ergch_information_state to OFF
        self._ps_data_ergch_information_state = "OFF"

        # Read agilent network from testcase xml Parameter
        self._rbt_channel_type = \
            self._tc_parameters.get_param_value("RBT_CHANNEL_TYPE")

        if self._cqi_scheme == "FIXED":
            self._cqi = int(self._tc_parameters.get_param_value("CQI"))

        # Read the TIMEOUT from UseCase xml Parameter
        self._timeout = self._tc_parameters.get_param_value("TIMEOUT")
        if isinstance(self._timeout, str) and self._timeout.isdigit():
            self._timeout = int(self._timeout)
        else:
            self._timeout = None

        # Read PHONE_NUMBER from testcase xml parameters
        if (self._tc_parameters.get_param_value("PHONE_NUMBER") not in (None, '')) \
                and str(self._tc_parameters.get_param_value("PHONE_NUMBER")).isdigit():
            self._phone_number = self._tc_parameters.get_param_value("PHONE_NUMBER")
        else:
            self._phone_number = str(self._device.get_phone_number())

        self._call_duration = \
            self._tc_parameters.get_param_value("CALL_DURATION")
        if isinstance(self._call_duration, str) and self._call_duration.isdigit():
            self._call_duration = int(self._call_duration)
        else:
            self._call_duration = None

        self._call_setup_timeout = \
            int(self._dut_config.get("callSetupTimeout"))

        self._voicecall_api = self._device.get_uecmd("VoiceCall")

        # Create cellular network simulator and retrieve 3G API
        self._ns_cell_3g = self._ns.get_cell_3g()
        self._ns_voice_call_3g = self._ns_cell_3g.get_voice_call()

        # Get UECmdLayer for VoiceCall
        self._voicecall_api = self._device.get_uecmd("VoiceCall")

        self._browser_type = self._tc_parameters.\
            get_param_value("BROWSER_TYPE").lower()
        self._web_page = self._tc_parameters.get_param_value("WEBSITE_URL")
        if self._web_page in [None, '']:
            self._web_page = "http://" + str(self._ns_IP_Lan1)

    def set_up(self):
        """
        Initialize the test
        """

        LabHspaBase.set_up(self)

        if self._timeout is None:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "TIMEOUT should be int")

        return Global.SUCCESS, self._error.Msg

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """

        # Call LabHspaBase Run function
        LabHspaBase.run_test(self)

        self._voicecall_api.dial(self._phone_number)

        # Check call state "CONNECTED" before callSetupTimeout seconds
        self._ns_voice_call_3g.check_call_connected(self._call_setup_timeout)

        # load the web page
        (result_code, result_msg) = self._networking_api.\
            open_web_browser(self._web_page, self._browser_type,
                             self._timeout)

        # Check call is still connected
        self._ns_voice_call_3g.is_voice_call_connected()

        # Release the voice call
        self._ns_voice_call_3g.voice_call_network_release()

        return result_code, result_msg
