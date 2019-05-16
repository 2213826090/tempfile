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
:summary: unit test
:since:30/12/2013
:author: fbongiax
"""

import unittest2 as unittest
import mock

from acs_test_scripts.TestStep.Device.Wireless.BT.Opp.BtOppCheckTransfer import BtOppCheckTransfer
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Wireless.BT.Constants import Constants


class CheckTest(UTestTestStepBase):
    """
    GetAddress test cases
    """

    _BDADDR = "00:00:00:00:00:00"
    _THROUGHPUT_THRESHOLD = 1600.0

    _COMMON_ARGS = {"BDADDR": _BDADDR, "FILES": "1MB.txt", "EXPECTED_FILES_SIZE": "1000"}
    _DOWNLOADING = Constants.OPP_STATE_DOWNLOADING
    _DOWNLOADED = Constants.OPP_STATE_DOWNLOADED
    _CANCELLED = Constants.OPP_STATE_CANCELLED
    _WAITING = Constants.OPP_STATE_WAITING

    # pylint: disable=C0103
    def setUp(self):
        """
        Setup method
        """
        UTestTestStepBase.setUp(self)

        self._test_factory = mock.Mock()

        self._check_service_response = []
        self._checksum_is_called = False
        self._current_time = 0

    # pylint: disable=W0212
    def test_run_time_out_expires_no_transfer(self):
        """
        Test time out expires
        """
        sut = self._create_sut()
        # Force offset timeout in the sut to 0
        sut._time_out_offset = 0
        # force list to be empty
        sut._pars.expected_files_size = 0

        # device will return no data
        self._set_check_service_response({})
        self._assert_run_throw_device_exception(sut, "OPP Check list is empty, Transfer seems not to be started")

    # pylint: disable=W0212
    def test_run_time_out_expires_while_transfer(self):
        """
        Test time out expires
        """
        sut = self._create_sut()
        # Force offset timeout in the sut to 0
        sut._time_out_offset = 0
        # Force timeout while transfer is ongoing
        sut._pars.expected_files_size = 2

        # device will return no data
        self._set_check_service_response({})
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 500))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 1000))
        self._assert_run_throw_device_exception(sut, "File\(s\) haven't been completely downloaded before timeout expired")

    def test_default_timeout(self):
        """
        Test passing nothing about timeout defaults to time_out_offset
        """
        sut = self._create_sut({"BDADDR": self._BDADDR, "FILES": "1MB.txt"}, False)
        self._set_check_service_response(self._create_response(self._DOWNLOADED, 1000))
        sut.run(self._context)

        self.assertEqual(sut._time_out_offset, sut._time_out, "time_out must be defaulted to time_out_offset")

    def test_explicit_timeout(self):
        """
        Test passing nothing about timeout defaults to time_out_offset
        """
        sut = self._create_sut({"BDADDR": self._BDADDR, "FILES": "1MB.txt", "TIMEOUT": 10}, False)
        self._set_check_service_response(self._create_response(self._DOWNLOADED, 1000))
        sut.run(self._context)

        self.assertEqual(10, sut._time_out, "time_out must be equal to TIMEOUT value")

    def test_run_ok_save_file_as(self):
        """
        Test runs ok
        """
        sut = self._create_sut({"SAVE_FILE_SIZE_AS": "size"})

        self._set_check_service_response({})
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 500))
        self._set_check_service_response(self._create_response(self._DOWNLOADED, 1000))
        self._assert_run_succeeded(sut)

    def test_run_time_ok_save_info_as(self):
        """
        Test runs ok
        """
        sut = self._create_sut({"SAVE_INFO_AS": "info"})

        self._set_check_service_response({})
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 500))
        self._set_check_service_response(self._create_response(self._DOWNLOADED, 1000))
        self._assert_run_succeeded(sut)

    def test_run_transfer_failed(self):
        """
        Test runs ok
        """
        sut = self._create_sut()

        self._set_check_service_response({})
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 0))
        self._set_check_service_response(self._create_response(self._CANCELLED, 0))
        self._assert_run_throw_device_exception(sut, "At least one of the transfer failed")

    def test_run_expected_any_started(self):
        """
        Test runs ok with expected state any_started
        """
        sut = self._create_sut({"EXPECTED_STATE": "any_started"})

        self._set_check_service_response({})
        self._set_check_service_response(self._create_response(self._WAITING, 0))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 0))
        self._assert_run_succeeded(sut)

    def test_run_expected_any_cancelled(self):
        """
        Test runs ok with expected state any_cancelled
        """
        sut = self._create_sut({"EXPECTED_STATE": "any_cancelled"})

        self._set_check_service_response({})
        self._set_check_service_response(self._create_response(self._WAITING, 0))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 500))
        self._set_check_service_response(self._create_response(self._CANCELLED, 0))
        self._assert_run_succeeded(sut)

    def test_run_expected_state_invalid(self):
        """
        Test runs ok
        """
        sut = self._create_sut({"EXPECTED_STATE": "invalid_state"})
        self._set_check_service_response({})

        self._assert_run_throw_device_exception(sut, "Invalid value for EXPECTED_STATE parameter: invalid_state")

    def test_throughput_ok(self):
        """
        Test throughput
        """
        self._build_file_transfer_simulation()
        sut = self._create_sut({"THROUGHPUT_MARGIN": "20"})

        sut.run(self._context)
        sut._throughput_targets.ul_target_value = self._THROUGHPUT_THRESHOLD

    def test_throughput_fails(self):
        """
        Test throughput
        """
        self._build_file_transfer_simulation()
        sut = self._create_sut({"THROUGHPUT_MARGIN": 1})

        self._assert_run_throw_device_exception(sut, "Throughput wasn't good enough for 1MB.txt transfer")
        sut._throughput_targets.ul_target_value = self._THROUGHPUT_THRESHOLD

    def test_throughput_ignored(self):
        """
        Test throughput is ignored.
        Today test steps arguments are mandatory, hence a way to say "ignore throughput" is needed.
        Check.py assumes that passing 0 as throughput margin means "ignore throughput"
        """
        self._build_file_transfer_simulation()
        sut = self._create_sut({"THROUGHPUT_MARGIN": 0})

        self._assert_run_succeeded(sut)

    def _build_file_transfer_simulation(self):
        """
        Simulate a real file transfer
        """
        self._set_check_service_response(self._create_response(self._WAITING, 0, 1388746253646))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 327615, 1388746255555))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 655230, 1388746257477))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 1048368, 1388746259455))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 1375983, 1388746261363))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 1703598, 1388746263259))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 2096736, 1388746265151))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 2424351, 1388746267035))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 2817489, 1388746268995))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 3145104, 1388746270934))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 3472719, 1388746272857))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 3865857, 1388746274794))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 4127949, 1388746276747))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 4455564, 1388746278665))
        self._set_check_service_response(self._create_response(self._DOWNLOADING, 4783179, 1388746280619))
        self._set_check_service_response(self._create_response(self._DOWNLOADED, 5120010, 1388746282589))

    def _create_response(self, status, downloaded_size, current_time=0):
        """
        Return data in the form needed by the sut
        """
        return [{"id": 1, "address": self._BDADDR, "filename": "1MB.txt",
                 "status": status, "filesize": "1000", "timestamp": "0",
                 "downloadedsize": str(downloaded_size), "currenttime": str(current_time), "direction": "0"}]

    def _set_check_service_response(self, response):
        """
        Set what bt_opp_check_service stub will return when called
        """
        self._check_service_response.append(response)

    def _stub_bt_opp_check_service(self):
        """
        Stub method
        """
        return self._check_service_response.pop(0)

    def _stub_bt_opp_get_files_checksum(self, folder_name, file_list):
        """
        Stub method
        """
        self._checksum_is_called = True
        self.assertIsNone(folder_name)
        self.assertEqual(["1MB.txt"], file_list)

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None, common_args=True):
        """
        Create the SUT with only test step pars
        """
        if common_args:
            args = self._COMMON_ARGS if not test_step_pars else dict(self._COMMON_ARGS.items() + test_step_pars.items())
        else:
            args = test_step_pars

        sut = BtOppCheckTransfer(None, None, args, self._test_factory)

        # Force poll time in the sut to 0
        sut._poll_sleep = 0

        # Stub device's api methods
        sut._api.bt_opp_check_service_raw = self._stub_bt_opp_check_service
        sut._api.bt_opp_get_files_checksum = self._stub_bt_opp_get_files_checksum
        sut._device.get_phone_model.return_value = 'BLACKBAY-Android-JB'

        sut._api.fail_on_not_implemented(True)

        return sut

if __name__ == "__main__":
    # import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
