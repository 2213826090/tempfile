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
:summary: This file implements an utility class to download from http/https server
:since 12/02/2014
:author: kturban
"""

import base64
import hashlib
import os
import tempfile
import requests
import shutil
import time

from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global

PROGRESS_INFO_SIZE_MIN = 500000
PRINT_STEP_PROGRESS = 5
HTTP_TIMEOUT = 10
DOWNLOAD_TIMEOUT = 60 * 10
HTTP_PATTERNS = ["http://", "https://"]


def is_http_uri(uri):
    """
    Tells if the given uri is http or https based uri

    :param uri: An URI
    :type uri: str

    :return: the result
    :rtype: bool

    """
    return any([True for pattern in HTTP_PATTERNS if pattern in uri])


class HttpDownloaderUtil(object):

    """
    Http downloader tool
    - init must be called in order to class attributes.
    - AcsConfigException will raised when one attribute is wrong
    - download method must be called to proceed the download
    """

    @staticmethod
    def is_http_uri(uri):
        """
        Wrapping :func:`is_http_uri` as static method for facility purpose

        :param uri: An URI
        :type uri: str

        """
        return is_http_uri(uri)

    def __init__(self, url, destination, proxy='', creds='anonymous', override=True, logger=LOGGER_TEST_SCRIPT,
                 http_timeout=HTTP_TIMEOUT, download_timeout=DOWNLOAD_TIMEOUT):
        """
        Constructor

        :type  url: str
        :param url: http url to the file to download

        :type  destination: str
        :param destination: local directory path

        :type  proxy: str
        :param proxy: proxy address

        :type  override: bool
        :param override: existing file must be removed

        :type  logger: logging
        :param logger: a logger class

        :type  http_timeout: int
        :param http_timeout: http request timeout

        :type  http_timeout: int
        :param http_timeout: download transfer timeout
        """
        self._download_timeout = download_timeout
        self._url = url
        self._logger = logger
        self._destination = destination
        self._proxy = proxy
        self._override_file = override
        self._http_timeout = http_timeout
        self.__creds = creds
        self._encoded_creds = ""

    def set_creds(self, creds):
        """
        Set creds attribute and encode creds as base 64 string

        :type  creds: str
        :param creds: credentials (format - user:password)
        """
        self.__creds = creds
        self._encode_creds()

    def init(self):
        """
        Check that all argument are good
        raise AcsConfigException if some values are wrong

        :rtype: request response object
        :return: the request get response
        """

        self.check_creds()
        self.check_proxy()
        response = self.check_url_arg()
        self.check_destination_arg()
        return response

    def check_proxy(self):
        """
        Check proxy
        """
        if not self._proxy or self._proxy.lower() == 'no_proxy':
            self._proxy = ''

    def check_creds(self):
        """
        Check credentials format
        """
        creds = self.__creds
        error_msg = "Invalid credentials"
        valid_parameter = False

        if not creds:
            error_msg = "Empty credentials!"
        elif not(len(creds.split(':')) == 2 and all(creds.split(":"))) and creds.lower() != "anonymous":
            error_msg = "creds must be : 'anonymous' or formated as 'user:password'"
        else:
            valid_parameter = True
            self.set_creds(creds)

        if not valid_parameter:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

    def check_url_arg(self):
        """
        Check url is valid

        :rtype: request response object
        :return: the request get response
        """
        error_msg = "Invalid parameter!"
        valid_parameter = False
        response = None

        if not self._url:
            error_msg = "Empty url"
        elif not is_http_uri(self._url):
            error_msg = "Url to download must start by {0}".format(" or ".join(HTTP_PATTERNS))

        else:
            response = self.check_url()
            if not response:
                error_msg = "Could not reach {0} : wrong url, proxy settings or credentials".format(self._url)
            else:
                valid_parameter = True

        if not valid_parameter:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
        return response

    def check_destination_arg(self):
        """
        Check destination folder is valid
        """
        destination = self._destination
        error_msg = "Invalid parameter!"
        valid_parameter = False

        if destination:
            destination = os.path.abspath(destination)
            # caution : check_url_arg has to be done first !
            if os.path.isdir(destination):
                filename = self._url.split('/')[-1]
                self._destination = os.path.join(destination, filename)
                valid_parameter = True
            elif not os.path.isdir(os.path.dirname(destination)):
                error_msg = "Destination folder {0} does not exist".format(os.path.dirname(destination))
            else:
                self._destination = destination
                valid_parameter = True
        else:
            error_msg = "No destination specified"

        if not valid_parameter:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

    def _encode_creds(self):
        """
        Encode credentials if necessary
        """
        if len(self.__creds.split(':')) == 2:
            user = self.__creds.split(':')[0]
            pw = self.__creds.split(':', 1)[1]
            self._encoded_creds = base64.b64encode(user + ":" + pw)
        else:
            self.__creds = ""
            self._encoded_creds = ""

    def get_md5(self, response=None):
        """
        Get md5 value if available

        :type  response: request response object
        :param response: previous request get response
        """
        return_value = None
        if not response:
            response = self.get()

        if response is not None:
            return_value = response.headers.get('X-Checksum-Md5')
        return return_value

    def check_url(self):
        """
        Check that url is accessible

        :rtype: request response object
        :return: the request get response
        """
        response = self.get()
        return_result = None
        if response is not None:
            if response.status_code == 200:
                return_result = response
            elif response.status_code == 401:
                return_result = None
                self._logger.error("Unauthorized Access : Wrong or missing cred!")
        return return_result

    def _generate_http_request_header(self):
        """
        Generate http request header
        """
        headers = {'User-Agent': 'python/request/ACS/%s' % (os.path.basename(__file__)),
                   "Accept-Encoding": "gzip,text",
                   "Accept": "application/json,text/html",
                   "X-Result-Detail": "info"}
        if self._encoded_creds:
            headers["Authorization"] = "Basic %s" % self._encoded_creds
        return headers

    def get(self):
        """
        Get a response object from specified url attribute.

        :rtype: request response object
        :return: the request get response
        """
        response = None
        # internal use only, no proxy
        if self._proxy.lower() == 'system':
            proxy_value = os.environ.get("HTTP_PROXY", "")
        else:
            proxy_value = self._proxy

        proxy = {'http': proxy_value,
                 'https': proxy_value}

        headers = self._generate_http_request_header()
        try:
            response = requests.get(self._url, headers=headers, proxies=proxy,
                                    timeout=self._http_timeout, stream=True, verify=False)
        except requests.exceptions.RequestException as e:
            error_msg = "An error occurred during connection with url : ({0})".format(e)
            self._logger.error(error_msg)
        return response

    def download(self, response=None):
        """
        Process the download of the file

        :type  response: request response object
        :param response: previous request get response
        """
        block_sz = 1024
        print_step = PRINT_STEP_PROGRESS
        download_finished = False
        downloaded_file = ""
        downloaded_file_path = None
        output = "Download has failed!"

        if os.path.isfile(self._destination):
            if self._override_file:
                os.remove(self._destination)
            else:
                error_msg = "%s already exist and parameter override file is set to False!" % self._destination
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        if not response:
            response = self.get()

        if response is not None:
            file_size = response.headers.get('content-length')
            if not file_size:
                output = "Cannot get the length of file to download"
            else:
                file_size = float(file_size)
                if response and response.status_code == 200:
                    with tempfile.NamedTemporaryFile(delete=False) as f:
                        self._logger.info("Downloading: {0} - Size : {1} ko ".format(self._url, file_size / (10 ** 3)))

                        file_size_dl = 0
                        md5_obj = hashlib.md5()
                        start_time = time.time()
                        for chunk in response.iter_content(chunk_size=block_sz):
                            # download timeout
                            if time.time() - start_time >= self._download_timeout:
                                error_msg = "{0} download has timeout after {1} seconds".format(self._url,
                                                                                                self._download_timeout)
                                raise AcsToolException(AcsToolException.HOST_OPERATION_TIMEOUT, error_msg)
                            if not chunk:
                                continue
                            file_size_dl += len(chunk)
                            f.write(chunk)
                            f.flush()
                            md5_obj.update(chunk)

                            if file_size >= PROGRESS_INFO_SIZE_MIN:
                                # Print progress status for file greater than 500kB
                                progress = file_size_dl * 100 / file_size

                                if (progress > print_step) | (int(progress) == int(100)):
                                    status = "[%3.1f%%]" % progress
                                    self._logger.info(status)
                                    print_step += PRINT_STEP_PROGRESS

                        local_md5 = md5_obj.hexdigest()
                        original_md5 = response.headers.get('X-Checksum-Md5')
                        downloaded_file_path = f.name
                        if original_md5:
                            original_md5 = str(original_md5)
                            msg = "ORIGINAL MD5 : %s - LOCAL MD5 : %s" % (original_md5, local_md5)
                            self._logger.debug(msg)
                            if local_md5 != original_md5:
                                output = "CheckSum has failed!"
                                self._logger.error(output)
                                download_finished = False
                            else:
                                # no md5 available to check file integrity but download has succeeded
                                download_finished = True
                        else:
                            download_finished = True

        if download_finished and downloaded_file_path:
            shutil.move(downloaded_file_path, self._destination)
            output = "Download has succeed in {0}".format(self._destination)
            self._logger.info(output)
            return_code = Global.SUCCESS
            downloaded_file = self._destination
        else:
            return_code = Global.FAILURE

        return return_code, output, downloaded_file
