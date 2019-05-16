# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
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
:summary: Unit test module
:since: 05/08/14
:author: jfranchx
"""

import os
import mock
from datetime import datetime
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from Core.TestStep.TestStepContext import TestStepContext
from acs_test_scripts.TestStep.Equipment.Sniffer.SnifferStartCapture import SnifferStartCapture


class SnifferStartCaptureTest(UTestTestStepBase):
    # Constants
    REPORT_PATH = "ReportPath"

    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()

    def test_stop_ok(self):
        sut = self._create_sut(
            {"CHANNEL": "6", "SAVE_SNIFF_ASCII": True, "DEVICE_MAC_ADDR": "AA:BB:CC:DD:EE:FF", "SSID": "SSID_TEST",
             "OUTPUT_FILE_SAVE_AS": "OUTPUT_FILE"})
        sut.run(self._context)
        filename = "capture-%s-channel6.cap" % datetime.now().strftime("%Hh%M.%S")
        complete_file_path = os.path.join(self.REPORT_PATH, filename)
        self.assertEqual(complete_file_path, self._context.get_info("OUTPUT_FILE"))
        self._sut._sniffer.start_capture.assert_called_with('6', complete_file_path, True, "AA:BB:CC:DD:EE:FF",
                                                            "SSID_TEST")

    def _create_sut(self, args=None):
        self._sut = SnifferStartCapture(None, mock.Mock(), args, mock.Mock())
        self._sut._pathname = self.REPORT_PATH
        return self._sut
