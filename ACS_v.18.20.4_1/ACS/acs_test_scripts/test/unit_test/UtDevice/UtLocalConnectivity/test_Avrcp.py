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
:since 27/03/2014
:author: fbongiax
"""
from unit_test.UtDevice.UtLocalConnectivity.BaseTestCase import BaseTestCase
from acs_test_scripts.Device.UECmd.Imp.Android.Common.LocalConnectivity.LocalConnectivity import LocalConnectivity


# Ignore W0212 has mocking / stubbing is needed for testing purpose (hence, accessing protected members)
# Ignore C0111 has methods names should be self describing.
# Ignore W0212 has some times names longer than 30 chars are needed to well express what the method does
# pylint: disable=C0111,C0103,W0212
class AvrcpTest(BaseTestCase):

    def test_api_call_succeeds(self):
        self._sut.avrcp_expect_buttons("play;pause;stop", 10, "myfile.mp3")
        self._assert_internal_exec_v2_was_called(LocalConnectivity._BLUETOOTH_MODULE, "expectButtons",
                                                 "--es buttons play;pause;stop --es timeout 10 --es fileName myfile.mp3")
