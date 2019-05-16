"""
@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: unit test
:since 08/09/2014
:author: jfranchx
"""
import mock

from UtilitiesFWK.Utilities import Global
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.Comms.DataXfer.FTPTransfer import FTPTransfer
from Core.TestStep.TestStepContext import TestStepContext


class FTPTransferTest(UTestTestStepBase):
    """
    FTPTransfer test cases
    """

    PARAM_DEVICE = "DEVICE"
    PARAM_USERNAME = "USERNAME"
    PARAM_PASSWORD = "PASSWORD"
    PARAM_SERVER_ADDRESS = "SERVER_IP"
    PARAM_CLIENT_ADDRESS = "CLIENT_IP"
    PARAM_DIRECTION = "DIRECTION"
    PARAM_TIMEOUT = "TIMEOUT"
    PARAM_FILENAME = "FILENAME"
    PARAM_SAVE_THROUGHPUT_AS = "SAVE_THROUGHPUT_AS"

    FTP_DEVICE = "PHONE1"
    FTP_USERNAME = "username"
    FTP_PASSWORD = "password"
    FTP_SERVER_ADDRESS = "192.168.0.150"
    FPT_CLIENT_ADDRESS = "none"
    FTP_TIMEOUT = "1000"
    FTP_DIRECTION_UPLOAD = "UL"
    FTP_DIRECTION_DOWNLOAD = "DL"
    FTP_FILENAME = "testFile"
    FTP_SAVE_THROUGHPUT_AS = "THROUGHPUT"

    FTP_DIRECTORY = "FTP_DIR"
    FTP_RESULT_SUCCESS = "Transfer file: acs_test/pub/get10M successfully - throughput: 4269kBytes/sec"
    FTP_RESULT_ERROR = "Transfer file: acs_test/pub/get10M error"

    def setUp(self):
        """
        Set up
        """
        UTestTestStepBase.setUp(self)
        self._context = TestStepContext()
        self._ftp_return_value = None

    def test_ftp_transfer_download_ok(self):
        self._ftp_return_value = [Global.SUCCESS, self.FTP_RESULT_SUCCESS]
        sut = self._create_sut({self.PARAM_DEVICE: self.FTP_DEVICE, self.PARAM_SERVER_ADDRESS: self.FTP_SERVER_ADDRESS,
                                self.PARAM_CLIENT_ADDRESS: self.FPT_CLIENT_ADDRESS,
                                self.PARAM_USERNAME: self.FTP_USERNAME, self.PARAM_PASSWORD: self.FTP_PASSWORD,
                                self.PARAM_TIMEOUT: self.FTP_TIMEOUT, self.PARAM_FILENAME: self.FTP_FILENAME,
                                self.PARAM_SAVE_THROUGHPUT_AS: self.FTP_SAVE_THROUGHPUT_AS,
                                self.PARAM_DIRECTION: self.FTP_DIRECTION_DOWNLOAD})

        sut.run(self._context)
        sut._networking_api.ftp_xfer.assert_called_once_with(self.FTP_DIRECTION_DOWNLOAD, self.FTP_SERVER_ADDRESS,
                                                             self.FTP_USERNAME, self.FTP_PASSWORD, self.FTP_FILENAME,
                                                             int(self.FTP_TIMEOUT), self.FTP_DIRECTORY,
                                                             target_throughput=None, client_ip_address=None)

    def test_ftp_transfer_upload_ok(self):
        self._ftp_return_value = [Global.SUCCESS, self.FTP_RESULT_SUCCESS]
        sut = self._create_sut({self.PARAM_DEVICE: self.FTP_DEVICE, self.PARAM_SERVER_ADDRESS: self.FTP_SERVER_ADDRESS,
                                self.PARAM_CLIENT_ADDRESS: self.FPT_CLIENT_ADDRESS,
                                self.PARAM_USERNAME: self.FTP_USERNAME, self.PARAM_PASSWORD: self.FTP_PASSWORD,
                                self.PARAM_TIMEOUT: self.FTP_TIMEOUT, self.PARAM_FILENAME: self.FTP_FILENAME,
                                self.PARAM_SAVE_THROUGHPUT_AS: self.FTP_SAVE_THROUGHPUT_AS,
                                self.PARAM_DIRECTION: self.FTP_DIRECTION_UPLOAD})

        sut.run(self._context)
        sut._networking_api.ftp_xfer.assert_called_once_with(self.FTP_DIRECTION_UPLOAD, self.FTP_SERVER_ADDRESS,
                                                             self.FTP_USERNAME, self.FTP_PASSWORD, self.FTP_FILENAME,
                                                             int(self.FTP_TIMEOUT), self.FTP_DIRECTORY,
                                                             target_throughput=None, client_ip_address=None)

    def test_ftp_transfer_download_fail(self):
        self._ftp_return_value = [Global.FAILURE, self.FTP_RESULT_ERROR]
        sut = self._create_sut({self.PARAM_DEVICE: self.FTP_DEVICE, self.PARAM_SERVER_ADDRESS: self.FTP_SERVER_ADDRESS,
                                self.PARAM_CLIENT_ADDRESS: self.FPT_CLIENT_ADDRESS,
                                self.PARAM_USERNAME: self.FTP_USERNAME, self.PARAM_PASSWORD: self.FTP_PASSWORD,
                                self.PARAM_TIMEOUT: self.FTP_TIMEOUT, self.PARAM_FILENAME: self.FTP_FILENAME,
                                self.PARAM_SAVE_THROUGHPUT_AS: self.FTP_SAVE_THROUGHPUT_AS,
                                self.PARAM_DIRECTION: self.FTP_DIRECTION_UPLOAD})

        self._assert_run_throw_device_exception(sut, "Error with FTP transfer result : %s - %s" % (
        Global.FAILURE, self.FTP_RESULT_ERROR))

    def test_ftp_transfer_download_result_fail(self):
        self._ftp_return_value = [Global.SUCCESS, self.FTP_RESULT_ERROR]
        sut = self._create_sut({self.PARAM_DEVICE: self.FTP_DEVICE, self.PARAM_SERVER_ADDRESS: self.FTP_SERVER_ADDRESS,
                                self.PARAM_CLIENT_ADDRESS: self.FPT_CLIENT_ADDRESS,
                                self.PARAM_USERNAME: self.FTP_USERNAME, self.PARAM_PASSWORD: self.FTP_PASSWORD,
                                self.PARAM_TIMEOUT: self.FTP_TIMEOUT, self.PARAM_FILENAME: self.FTP_FILENAME,
                                self.PARAM_SAVE_THROUGHPUT_AS: self.FTP_SAVE_THROUGHPUT_AS,
                                self.PARAM_DIRECTION: self.FTP_DIRECTION_UPLOAD})

        self._assert_run_throw_device_exception(sut,
                                                "Error invalid throughput during FTP transfer : %s" % self.FTP_RESULT_ERROR)

    def _simulate_get_ftpdir_path(self):
        return self.FTP_DIRECTORY

    # pylint: disable=W0212
    def _create_sut(self, test_step_pars=None):
        """
        Create the SUT with only test step pars
        """
        sut = FTPTransfer(None, mock.Mock(), test_step_pars, mock.Mock())
        sut._networking_api.ftp_xfer.return_value = self._ftp_return_value
        sut._device.multimedia_path = self.FTP_DIRECTORY
        return sut

