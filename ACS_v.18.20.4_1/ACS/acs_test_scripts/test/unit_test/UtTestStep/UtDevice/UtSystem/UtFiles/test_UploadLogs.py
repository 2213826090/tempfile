"""
@summary: Unit test module for UploadTeststepLogs
@since: 21 March 2014
@author: Val Peterson
@organization: INTEL PEG-SVE-DSV

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
"""
import os
import shutil
from mock import Mock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.System.Files.UploadLogs import UploadLogs
from Core.TestStep.TestStepContext import TestStepContext


DEFAULT_STORED_PATH = "PATH1"
class UploadLogsTestCase(UTestTestStepBase):
    def setUp(self):
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()

    def _create_sut(self, test_step_pars=None):
        self._sut = UploadLogs(None, None, test_step_pars, Mock())

    def _mock_pull(self, src, dest, timeout):
        src = os.path.join(os.path.dirname(__file__), os.path.normpath(src))
        # if os.path.exists(src):
        shutil.copy(src, dest)

    def test_files_exist(self):
        """
            Check that it uploads files that exist
        """
        self._create_sut({"DESTINATION_STORED_PATH":DEFAULT_STORED_PATH})
        self._sut._pars.file_path = "src_files"
        # Force reloading of os to unmock it.
        reload(os)
        src_path = os.path.join(os.path.dirname(__file__), self._sut._pars.file_path)
        if not os.path.exists(src_path):
            os.makedirs(src_path)
        shutil.copy(__file__, os.path.join(src_path, "pass.log"))
        self._sut._pars.files = "pass.log"
        self._sut._pars.host_subdir = "test_files"
        self._sut._device.get_report_tree.return_value.get_report_path.return_value = os.path.dirname(__file__)
        self._sut.file_api.exist.return_value = (True, "pass.log")
        self._sut._device.pull = Mock()
        self._sut._device.pull.side_effect = self._mock_pull
        self._sut.file_api.find_files.return_value = (True, "pass.log")
        self._sut.run(self._context)
        shutil.rmtree(src_path, True)
        final_path = os.path.join(self._context.get_info(DEFAULT_STORED_PATH), "pass.log")
        assert(os.path.exists(final_path) == True)
        shutil.rmtree(self._context.get_info(DEFAULT_STORED_PATH))

    def test_files_do_not_exist(self):
        """
            Check that it properly reports that files are not available for upload
        """
        self._create_sut({"DESTINATION_STORED_PATH":DEFAULT_STORED_PATH})
        self._sut._pars.file_path = "src_files"
        self._sut._pars.files = "pass.log"
        self._sut._pars.host_subdir = "test_files"
        self._sut._device.get_report_tree.return_value.get_report_path.return_value = os.path.dirname(__file__)
        self._sut.file_api.exist.return_value = (False, "fail.log")
        self._sut.file_api.find_files.return_value = (True, "")
        self._sut._device.pull = Mock()
        self._sut._device.pull.side_effect = self._mock_pull
        self._sut.run(self._context)
        final_path = self._context.get_info(DEFAULT_STORED_PATH)
        assert(final_path == "None")
