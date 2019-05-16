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
:summary: This file implements Android UI tests with MT voice call from PHONE2
:since: 19/09/2011
:author: ssavrimoutou
"""

import time

from acs_test_scripts.UseCase.Misc.ANDROID_UI_TEST import AndroidUiTest
from UtilitiesFWK.Utilities import Global
from Device.DeviceManager import DeviceManager


class AndroidUiTestVcMt(AndroidUiTest):

    """
    Android UI test mobile terminated voice call class.
    This use case will permit to launch UI test on DUT.
    The PHONE2 will be used in application framework level to make a voice call.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call Android UI test use case base Init function
        AndroidUiTest.__init__(self, tc_name, global_config)

        #
        self._phone_number = \
            str(self._device.get_phone_number())

        # Read callSetupTimeout from Device_Catalog.xml
        self._call_setup_time = \
            int(self._dut_config.get("callSetupTimeout"))

        # Get UECmdLayer
        self._phone2 = DeviceManager().get_device("PHONE2")
        if self._phone2 is not None:
            self._voice_call_api2 = self._phone2.get_uecmd("VoiceCall")

    def set_up(self):
        """
        Set up the test configuration
        """
        # Call Android UI test use case base Setup function
        verdict, msg = AndroidUiTest.set_up(self)
        if verdict != Global.SUCCESS:
            return verdict, msg

        # Check if we have the second phone available
        if self._phone2 is None:
            # We are using this multi UC with only one phone
            return Global.FAILURE, "Cannot run that use case with only one phone configured."

        # Boot the other phone (the DUT is already booted)

        if self._phone2.get_state() != "alive":
            # We are using this multi UC with an phone2 is not available
            return Global.FAILURE, "Ref phone not available (power off ?), please check your bench."

        if not self._phone2.is_available():
            self._phone2.connect_board()

        return Global.SUCCESS, "No error"

    def run_test(self):
        """
        Execute the test
        """
        self._logger.info("")
        self._logger.info("%s: RunTest", self._name)
        self._logger.info("")

        # Phone 2 : Release any previous call (Robustness)
        time.sleep(self._wait_btwn_cmd)
        self._voice_call_api2.release()

        # Phone2 : Dial without checking call state (Non Blocking)
        time.sleep(self._wait_btwn_cmd)
        self._voice_call_api2.dial(self._phone_number,
                                   False)

        # Phone1 : Call AndroidUiTest run test
        return AndroidUiTest.run_test(self)
