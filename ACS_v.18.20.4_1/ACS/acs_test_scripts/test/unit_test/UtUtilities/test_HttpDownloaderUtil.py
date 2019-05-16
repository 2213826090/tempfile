# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
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
:summary: Unit test module
:since: 1/27/14
:author: sfusilie
"""
import os
import unittest

from mock import Mock
from mock import patch

from acs_test_scripts.Utilities.HttpDownloaderUtil import HttpDownloaderUtil
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsToolException import AcsToolException


class fake_request_response(object):
    headers = {'X-Checksum-Md5': '1234567890', 'content-length': '10'}
    status_code = 200

    def iter_content(self, chunk_size):
        return ["a"]


class HttpDownloaderTestCase(unittest.TestCase):

    def setUp(self):
        patch("acs_test_scripts.Utilities.HttpDownloaderUtil.hashlib.md5").start()
        patch("acs_test_scripts.Utilities.HttpDownloaderUtil.shutil.move").start()
        patch("acs_test_scripts.Utilities.HttpDownloaderUtil.os.remove").start()
        patch("acs_test_scripts.Utilities.HttpDownloaderUtil.tempfile.NamedTemporaryFile").start()

    def test_http_downloader_check_wrong_creds_format(self):
        mock_logger = Mock()
        # empty creds
        http_downloader_obj = HttpDownloaderUtil(url="",
                                                 destination="",
                                                 logger=mock_logger,
                                                 creds="")
        self.assertRaises(AcsConfigException, http_downloader_obj.check_creds)
        # wrong creds format
        http_downloader_obj.set_creds("mylogin-mdp")
        self.assertRaises(AcsConfigException, http_downloader_obj.check_creds)
        # missing password
        http_downloader_obj.set_creds("mylogin:")
        self.assertRaises(AcsConfigException, http_downloader_obj.check_creds)
        # missing user
        http_downloader_obj.set_creds(":mypassword")
        self.assertRaises(AcsConfigException, http_downloader_obj.check_creds)

    def test_http_downloader_check_creds_good_format(self):
        mock_logger = Mock()
        # empty creds
        http_downloader_obj = HttpDownloaderUtil(url="",
                                                 destination="",
                                                 logger=mock_logger,
                                                 creds="mylogin:password")
        http_downloader_obj.check_creds()
        encoded_creds = http_downloader_obj._encoded_creds
        self.assertNotEqual(encoded_creds, "")
        # anonymous
        http_downloader_obj.set_creds("anonymous")
        no_encoded_creds = http_downloader_obj._encoded_creds
        self.assertNotEqual(encoded_creds, no_encoded_creds)

    @patch.object(HttpDownloaderUtil, "check_url")
    def test_http_downloader_check_url_arg_ko_url_empty(self, mock_check_url):
        mock_logger = Mock()
        mock_check_url.return_value = None
        # empty url
        http_downloader_obj = HttpDownloaderUtil(url="",
                                                 destination="",
                                                 logger=mock_logger)

        self.assertRaises(AcsConfigException, http_downloader_obj.check_url_arg)

    @patch.object(HttpDownloaderUtil, "check_url")
    def test_http_downloader_check_url_arg_ko_url_bad_pattern(self, mock_check_url):
        mock_logger = Mock()
        mock_check_url.return_value = None
        # wrong http pattern
        http_downloader_obj = HttpDownloaderUtil(url="http:/does_not_exist.com",
                                                 destination="",
                                                 logger=mock_logger)
        # wrong url format
        self.assertRaises(AcsConfigException, http_downloader_obj.check_url_arg)
        # error raised without any call of check_url
        self.assertFalse(mock_check_url.called)

    @patch.object(HttpDownloaderUtil, "check_url")
    def test_http_downloader_check_url_arg_ko_url_could_not_be_reached(self, mock_check_url):
        mock_logger = Mock()
        mock_check_url.return_value = None
        # good url but not cound not be reached
        http_downloader_obj = HttpDownloaderUtil(url="http://ggoggle.com",
                                                 destination="",
                                                 logger=mock_logger)
        self.assertRaises(AcsConfigException, http_downloader_obj.check_url_arg)
        self.assertTrue(mock_check_url.called)

    @patch.object(HttpDownloaderUtil, "check_url")
    def test_http_downloader_check_url_arg_ok_http_url(self, mock_check_url):
        mock_logger = Mock()
        mock_check_url.return_value = fake_request_response()
        # empty url
        http_downloader_obj = HttpDownloaderUtil(url="http://www.google.com",
                                                 destination="",
                                                 logger=mock_logger)
        http_downloader_obj.check_url_arg()

    @patch.object(HttpDownloaderUtil, "check_url")
    def test_http_downloader_check_url_arg_ok_https_url(self, mock_check_url):
        mock_logger = Mock()
        mock_check_url.return_value = fake_request_response()
        # empty url
        http_downloader_obj = HttpDownloaderUtil(url="https://www.google.com",
                                                 destination="",
                                                 logger=mock_logger)
        http_downloader_obj.check_url_arg()

    def test_http_downloader_check_destination_arg_ko_empty_destination(self):
        mock_logger = Mock()
        # good url is according to method comment
        # empty destination
        http_downloader_obj = HttpDownloaderUtil(url="http://test/my_file",
                                                 destination="",
                                                 logger=mock_logger)

        self.assertRaises(AcsConfigException, http_downloader_obj.check_destination_arg)

    def test_http_downloader_check_destination_arg_ko_destination_does_not_exist(self):
        mock_logger = Mock()
        # good url is according to method comment
        # destination does not exist
        http_downloader_obj = HttpDownloaderUtil(url="http://test/my_file",
                                                 destination="doestNotExist/at/all/",
                                                 logger=mock_logger)

        self.assertRaises(AcsConfigException, http_downloader_obj.check_destination_arg)

    def test_http_downloader_check_destination_arg_ok_file_as_dest(self):
        mock_logger = Mock()
        # good url is according to method comment
        # existing file destination
        dest_file = __file__
        http_downloader_obj = HttpDownloaderUtil(url="http://test/my_file",
                                                 destination=dest_file,
                                                 logger=mock_logger)
        http_downloader_obj.check_destination_arg()
        self.assertEqual(http_downloader_obj._destination, dest_file)

    def test_http_downloader_check_destination_arg_ok_dir_as_dest(self):
        mock_logger = Mock()
        # good url is according to method comment
        # existing dir destination
        dest_dir = os.path.dirname(__file__)
        expected_dest = os.path.join(dest_dir, "my_file")
        http_downloader_obj = HttpDownloaderUtil(url="http://test/my_file",
                                                 destination=dest_dir,
                                                 logger=mock_logger)
        http_downloader_obj.check_destination_arg()
        self.assertEqual(http_downloader_obj._destination, expected_dest)

    def test_http_downloader_check_destination_arg_ok_unexisting_file_but_existing_dir_as_dest(self):
        mock_logger = Mock()
        # good url is according to method comment
        # existing dir, unexisting file
        dest_file = os.path.join(os.path.dirname(__file__), "doesnotexistfile")
        http_downloader_obj = HttpDownloaderUtil(url="http://test/my_file",
                                                 destination=dest_file,
                                                 logger=mock_logger)
        http_downloader_obj.check_destination_arg()
        self.assertEqual(http_downloader_obj._destination, dest_file)

    @patch.object(HttpDownloaderUtil, "get")
    def test_http_downloader_check_url_ok(self, mock_get):
        mock_logger = Mock()
        http_downloader_obj = HttpDownloaderUtil(url="",
                                                 destination="",
                                                 logger=mock_logger)
        fake_response = fake_request_response()
        fake_response.status_code = 200
        mock_get.return_value = fake_response
        self.assertTrue(http_downloader_obj.check_url())

    @patch.object(HttpDownloaderUtil, "get")
    def test_http_downloader_check_url_ko(self, mock_get):
        mock_logger = Mock()
        http_downloader_obj = HttpDownloaderUtil(url="",
                                                 destination="",
                                                 logger=mock_logger)
        fake_response = fake_request_response()
        fake_response.status_code = None
        mock_get.return_value = fake_response
        self.assertFalse(http_downloader_obj.check_url())

        mock_get.reset_mock()
        fake_response.status_code = 403
        mock_get.return_value = fake_response
        self.assertFalse(http_downloader_obj.check_url())

        mock_get.reset_mock()
        fake_response.status_code = 401
        mock_get.return_value = fake_response
        self.assertFalse(http_downloader_obj.check_url())

        mock_get.reset_mock()
        fake_response.status_code = 404
        mock_get.return_value = fake_response
        self.assertFalse(http_downloader_obj.check_url())

    def test_http_downloader_download_override_parameter(self):
        # to avoid removing file if unit teest is ko
        mock_logger = Mock()
        # empty header
        http_downloader_obj = HttpDownloaderUtil(url="",
                                                 destination=__file__,
                                                 override=False,
                                                 logger=mock_logger)
        self.assertRaises(AcsConfigException, http_downloader_obj.download)

    @patch.object(HttpDownloaderUtil, "get")
    def test_http_downloader_download_ko_bad_header(self, mock_get):
        mock_logger = Mock()
        # empty header
        http_downloader_obj = HttpDownloaderUtil(url="",
                                                 destination="",
                                                 logger=mock_logger)
        fake_response = fake_request_response()
        fake_response.headers = dict()
        mock_get.return_value = fake_response
        return_value = http_downloader_obj.download()
        self.assertNotEqual(return_value[0], 0)
        self.assertEqual(return_value[2], "")

    @patch.object(HttpDownloaderUtil, "get")
    def test_http_downloader_download_timeout_ko(self, mock_get):
        mock_logger = Mock()
        # empty header
        http_downloader_obj = HttpDownloaderUtil(url="",
                                                 destination="",
                                                 download_timeout=0,
                                                 logger=mock_logger)
        fake_response = fake_request_response()
        mock_get.return_value = fake_response
        self.assertRaises(AcsToolException, http_downloader_obj.download)

    @patch.object(HttpDownloaderUtil, "get")
    def test_http_downloader_download_no_md5_check(self, mock_get):
        patch("acs_test_scripts.Utilities.HttpDownloaderUtil.hashlib.md5")
        mock_logger = Mock()
        # empty header
        http_downloader_obj = HttpDownloaderUtil(url="",
                                                 destination=os.path.dirname(__file__),
                                                 download_timeout=1,
                                                 logger=mock_logger)
        fake_response = fake_request_response()
        fake_response.headers = {'content-length': '10'}
        mock_get.return_value = fake_response
        return_value = http_downloader_obj.download()
        self.assertEqual(return_value[0], 0)
        self.assertNotEqual(return_value[2], "")

    @patch('hashlib.md5')
    @patch.object(HttpDownloaderUtil, "get")
    def test_http_downloader_download_timeout_md5_check_ok(self, mock_get, mock_md5):
        mock_logger = Mock()
        # empty header
        http_downloader_obj = HttpDownloaderUtil(url="",
                                                 destination=os.path.dirname(__file__),
                                                 download_timeout=1,
                                                 logger=mock_logger)
        fake_response = fake_request_response()
        fake_response.headers = {'content-length': '10',
                                 'X-Checksum-Md5': '123456789abc'}
        my_md5 = mock_md5()
        my_md5.hexdigest.return_value = '123456789abc'
        mock_get.return_value = fake_response

        return_value = http_downloader_obj.download()
        self.assertEqual(return_value[0], 0)
        self.assertNotEqual(return_value[2], "")

    @patch('acs_test_scripts.Utilities.HttpDownloaderUtil.hashlib.md5')
    @patch.object(HttpDownloaderUtil, "get")
    def test_http_downloader_download_timeout_md5_check_ko(self, mock_get, mock_md5):
        mock_logger = Mock()
        # empty header
        http_downloader_obj = HttpDownloaderUtil(url="",
                                                 destination=os.path.dirname(__file__),
                                                 download_timeout=1,
                                                 logger=mock_logger)
        fake_response = fake_request_response()
        fake_response.headers = {'content-length': '10',
                                 'X-Checksum-Md5': '123456789abc'}
        my_md5 = mock_md5()
        my_md5.hexdigest.return_value = '123456789abcNotTheSame'
        mock_get.return_value = fake_response

        return_value = http_downloader_obj.download()
        self.assertNotEqual(return_value[0], 0)
        self.assertEqual(return_value[2], "")
