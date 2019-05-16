"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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
:summary: unit test
:since 27/01/2014
:author: fbongiax
"""
import unittest
import mock

from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.BT.BtExpectAVRCPButtons import BtExpectAVRCPButtons


class BtExpectAVRCPButtonsTest(UTestTestStepBase):
    """
    ExpectAvrcpButtons test cases
    """

    def test_api_called_successfully(self):
        sut = self._create_sut({"BUTTONS": "PLAY,PAUSE,STOP", "TIMEOUT": 10, "FILENAME": "myfile.mp3"})
        sut.run(None)
        sut._api.avrcp_expect_buttons.assert_called_with("PLAY,PAUSE,STOP", 10, "/mypath/myfile.mp3")

    def test_invalid_button_parameter(self):
        sut = self._create_sut({"BUTTONS": "PLAY,PAUSE,INVALID", "TIMEOUT":10, "FILENAME": "myfile.mp3"})
        self._assert_run_throw_config_exception(sut, "Error parameter BUTTONS contains invalid value : INVALID")

    def test_float_timeout_parameter(self):
        sut = self._create_sut({"BUTTONS": "PLAY,PAUSE,STOP", "TIMEOUT": 10.25, "FILENAME": "myfile.mp3"})
        self._assert_run_throw_config_exception(sut, "Error parameter TIMEOUT is not integer : <type 'float'>")

    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = BtExpectAVRCPButtons(None, None, test_step_pars, mock.Mock())
        sut._device.multimedia_path = "/mypath"
        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
