# pylint: disable=C0303
"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file expose a generic HTTP request handler
:since: 13/06/2014
:author: dgonza4x
"""
import json

import requests
from requests import exceptions

from acs_test_scripts.Utilities.HTTPRequestHandler.HTTPRequestException import HTTPRequestException


class HTTPRequestHandler:

    """
    Handle GET and POST requests, with common http error codes
    """

    def __init__(self, logger=None, proxies=None):
        """
        Constructor

        :type logger: object
        :param logger: Write logs into a logging instance

        :type proxies: dict
        :param proxies: Dictionary mapping protocol to the URL of the proxy
        (e.g. "http": "foo.bar:3128") to be used on each requests
        """
        self.__proxies = proxies
        self.__logger = logger

    def __log(self, message):
        """
        Write a log or print messages into Console if no logger has been set

        :type message: str
        :param message: Message to be written
        """
        if self.__logger:
            self.__logger.debug(message)
        else:
            print message

    def __process_response(self, response):
        """
        Process the response from a get or post http request

        :type response: object
        :param response: requests.Response instance to be processed

        :rtype: str
        :return: request content if status is OK
        """
        resp_handler = {
            200: self.__on_status_ok,
            400: self.__on_status_bad_request,
            401: self.__on_status_unauthorized,
            404: self.__on_status_not_found,
            501: self.__on_status_not_implemented,
            503: self.__on_status_service_unavailable,
            550: self.__on_status_permission_denied
        }
        # process the response
        return resp_handler[response.status_code](response)

    def get_request(self, url, timeout):
        """
        Process a GET requests.

        :type url: str
        :param url: URL to be sent

        :type timeout: int
        :param timeout: URL response timeout, in seconds

        :rtype: str
        :return: URL response

        """
        self.__log("HTTP GET request : " + str(url))
        try:
            response = requests.get(url=url, timeout=timeout, proxies=self.__proxies)
            return self.__process_response(response)

        except exceptions.Timeout:
            raise HTTPRequestException(HTTPRequestException.REQUEST_TIMEOUT)
        except exceptions.ConnectionError:
            raise HTTPRequestException(HTTPRequestException.CONNECTION_ERROR)
        except exceptions.MissingSchema:
            raise HTTPRequestException(HTTPRequestException.MALFORMED_REQUEST)
        except Exception as e:
            self.__log("Unknown Exception occurred during http GET request: " + str(e))
            raise

    def post_request(self, url, post_data, timeout):
        """
        Process a POST http request.

        :type url: str
        :param url: URL to be sent

        :type timeout: int
        :param timeout: URL response timeout, in seconds

        :rtype: str
        :return: URL response
        """
        self.__log("HTTP POST request : " + str(url))
        headers = {'content-type': 'text/plain'}
        try:
            response = requests.post(url, data=post_data, headers=headers,
                                     timeout=timeout, proxies=self.__proxies)
            return self.__process_response(response)
        except exceptions.Timeout:
            raise HTTPRequestException(HTTPRequestException.REQUEST_TIMEOUT)
        except exceptions.ConnectionError:
            raise HTTPRequestException(HTTPRequestException.CONNECTION_ERROR)
        except exceptions.MissingSchema:
            raise HTTPRequestException(HTTPRequestException.MALFORMED_REQUEST)
        except Exception as e:
            self.__log("Unknown Exception occurred during http POST request: " + str(e))
            raise

    def __on_status_ok(self, response):
        """
        Process a http request response on status 200 (OK).

        :type response: requests.Response
        :param response: Response object coming from a http request

        :rtype: str
        :return: output from the http request
        """
        self.__log("HTTP request result %s: OK" % str(response.status_code))
        return response.text

    def __on_status_bad_request(self, response):
        """
        Process a http request response on status 400 (bad request),
        raising an HTTPRequestException.

        :type response: requests.Response
        :param response: Response object coming from a http request

        :return: None
        """
        self.__log("HTTP request result %s: bad request (%s)" % (str(response.status_code), response.text))
        raise HTTPRequestException("HTTP status %s: bad request" % str(response.status_code))

    def __on_status_unauthorized(self, response):
        """
        Process a http request response on status 401 (unauthorized),
        raising an HTTPRequestException.

        :type response: requests.Response
        :param response: Response object coming from a http request

        :return: None
        """
        self.__log("HTTP request result %s: unauthorized (%s)" % (str(response.status_code), response.text))
        raise HTTPRequestException("HTTP status %s: unauthorized" % str(response.status_code))

    def __on_status_not_found(self, response):
        """
        Process a http request response on status 404 (not found),
        raising an HTTPRequestException.

        :type response: requests.Response
        :param response: Response object coming from a http request

        :return: None
        """
        self.__log("HTTP request result %s: not found (%s)" % (str(response.status_code), response.text))
        raise HTTPRequestException("HTTP status %s: not found" % str(response.status_code))

    def __on_status_not_implemented(self, response):
        """
        Process a http request response on status 501 (not implemented),
        raising an HTTPRequestException.

        :type response: requests.Response
        :param response: Response object coming from a http request

        :return: None
        """
        self.__log("HTTP request result %s: not implemented (%s)" % (str(response.status_code), response.text))
        raise HTTPRequestException("HTTP status %s: not implemented" % str(response.status_code))

    def __on_status_service_unavailable(self, response):
        """
        Process a http request response on status 503 (service not available),
        raising an HTTPRequestException.

        :type response: requests.Response
        :param response: Response object coming from a http request

        :return: None
        """
        self.__log("HTTP request result %s: service unavailable (%s)" % (str(response.status_code), response.text))
        raise HTTPRequestException("HTTP status %s: service unavailable" % str(response.status_code))

    def __on_status_permission_denied(self, response):
        """
        Process a http request response on status 550 (permission denied),
        raising an HTTPRequestException.

        :type response: requests.Response
        :param response: Response object coming from a http request

        :return: None
        """
        self.__log("HTTP request result %s: permission denied (%s)" % (str(response.status_code), response.text))
        raise HTTPRequestException("HTTP status %s : permission denied" % str(response.status_code))

if __name__ == "__main__":
    va = HTTPRequestHandler()
    print va.get_request("http://localhost:8080/Acs/ws/GetVersion", 2)

    error_cmds = [
        "http://www.google.com/toto",  # not found
        "http://192.168.0.0:8080/Acs/ws/GetVersion",  # timeout
        "http://localhost:1234/Acs/ws/GetVersion",  # Connection error
        "malformed/http/request",  # Malformed HTTP request
        "http://status.savanttools.com/?code=501%20Not%20Implemented",  # not implemented
    ]
    for cmd in error_cmds:
        try:
            # bad command
            va.get_request(cmd, 2)
        except Exception as excp:
            print "EXCEPTION : " + str(excp)

    data = {
        "assembly": "Intel.Acs.TestFmk.WifiConnectivity",
        "class": "Intel.Acs.TestFmk.WifiConnectivity.WifiActivity",
        "method": "ListAvailableEAPMethods",
        "debugger": False,
        "args": "",
    }
    json_str = str(json.dumps(data))
    print va.post_request("http://localhost:8080/Acs/ws/LaunchActivity", json_str, 10)
