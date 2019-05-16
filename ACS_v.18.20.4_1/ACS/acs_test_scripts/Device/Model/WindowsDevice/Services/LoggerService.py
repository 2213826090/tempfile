# pylint: disable = C0303
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
:summary: This file define an Exception class handling http request problems
:since: 18/06/2014
:author: dgonza4x
"""
import json
import time

from acs_test_scripts.Utilities.HTTPRequestHandler.HTTPRequestHandler import HTTPRequestHandler
from acs_test_scripts.Utilities.HTTPRequestHandler.HTTPRequestException import HTTPRequestException


from CommandResultException import CommandResultException


class LoggerService:

    """
    Class that handle communication with the Windows 'Logger' Service
    running on the DUT, through http requests.
    """

    __ROOT_URL_PATTERN = "http://%s:%s/Acs/Logger/ws/"
    """
    Base url PATTERN used to call each web service methods
    """

    __EMPTY_LOG_FILENAME = "NULL"
    """
    Define an empty log filename, used in methods enable_kernel_traces and enable_traces
    """

    def __init__(self, device_ip, device_port, proxies, logger, default_timeout):
        """
        Constructor

        :type device_ip: str
        :param device_ip: String representing a valid IPv4 address

        :type device_port: str
        :param device_port: string representing a valid port number

        :type proxies: dict
        :param proxies: Dictionary mapping protocol to the URL of the proxy
        (e.g. "http": "foo.bar:3128") to be used on each requests

        :type logger: object
        :param logger: Instance used to log messages

        :type default_timeout: int
        :param default_timeout: default command response timeout
        """
        self.__http_handler = HTTPRequestHandler(logger=logger, proxies=proxies)
        self.__default_timeout = default_timeout
        self.__device_ip = str(device_ip)
        self.__device_port = str(device_port)
        self.__logger = logger

    def __root_url(self):
        """
        Compute the root URL of this service

        :rtype: str
        :return: base url for every service's method
        """
        return self.__ROOT_URL_PATTERN % (self.__device_ip, self.__device_port)

    def enable_kernel_traces(self, events, log_filename=""):
        """
        Enable kernel traces

        :type events: str
        :param events: String representing a list (comma separated) of event filters

        :type log_filename: str
        :param log_filename: complete file pathname where log will be written on DUT
        """
        filename = log_filename
        if not filename:
            filename = self.__EMPTY_LOG_FILENAME
        events = str(events).replace(' ', '')
        _url = "%s%s/%s/%s" % (self.__root_url(), "EnableKernelTraces", str(events), filename)
        try:
            self.__http_handler.get_request(_url, self.__default_timeout)
        except HTTPRequestException as excp:
            raise CommandResultException("Communication error: " + str(excp))

    def disable_kernel_traces(self):
        """
        Disable kernel traces
        """
        try:
            self.__http_handler.get_request(self.__root_url() + "DisableKernelTraces", self.__default_timeout)
        except HTTPRequestException as excp:
            raise CommandResultException("Communication error: " + str(excp))

    def enable_traces(self, guid, trace_level, log_filename):
        """
        Enable traces of a specific application

        :type guid: str
        :param guid: Globally Unique IDentifier of an application

        :type trace_level: str
        :param trace_level: String representing the windows trace level
        (see TraceView levels for more information)

        :type log_filename: str
        :param log_filename: complete file pathname where log will be written on DUT
        """
        filename = log_filename
        if not filename:
            filename = self.__EMPTY_LOG_FILENAME
        url = "%s%s/%s/%s/%s" % (self.__root_url(), "EnableTraces", guid, trace_level, filename)
        try:
            self.__http_handler.get_request(url, self.__default_timeout)
        except HTTPRequestException as excp:
            raise CommandResultException("Communication error: " + str(excp))

    def disable_traces(self):
        """
        Disable traces
        """
        try:
            self.__http_handler.get_request(self.__root_url() + "DisableTraces", self.__default_timeout)
        except HTTPRequestException as excp:
            raise CommandResultException("Communication error: " + str(excp))

    def enable_debug_traces(self):
        """
        Enable debug traces
        """
        try:
            self.__http_handler.get_request(self.__root_url() + "EnableDebugTraces", self.__default_timeout)
        except HTTPRequestException as excp:
            raise CommandResultException("Communication error: " + str(excp))

    def disable_debug_traces(self):
        """
        Disable debug traces
        """
        try:
            self.__http_handler.get_request(self.__root_url() + "DisableDebugTraces", self.__default_timeout)
        except HTTPRequestException as excp:
            raise CommandResultException("Communication error: " + str(excp))

    def flush(self):
        """
        Flush the unprocessed logs
        """
        try:
            self.__http_handler.get_request(self.__root_url() + "Flush", self.__default_timeout)
        except HTTPRequestException as excp:
            raise CommandResultException("Communication error: " + str(excp))

    def inject_log(self, message, tag, level):
        """
        Inject a custom log, defined by a tag, a level and a message

        :type message: str
        :param message: Message to be written on log.

        :type tag: str
        :param tag: Tag to be used to identify the log onto the logger

        :type level: str
        :param level: log level. Level should be defined by the instance using this method
        """
        try:
            data = {
                "message": message,
                "tag": tag,
                "level": level,
            }
            encoded_data = json.dumps(data)
            self.__http_handler.post_request(self.__root_url() + "InjectLog", encoded_data, self.__default_timeout)
        except HTTPRequestException as excp:
            raise CommandResultException("Communication error: " + str(excp))


def __test_service():
    """
    Function testing the Logger service.
    :attention: the service must be launched before executing this file
    """
    logger = LoggerService("192.168.0.166", "8082", None, 60)

    # test kernel traces
    logger.enable_kernel_traces("DiskIO,ImageLoad")
    time.sleep(2)
    logger.disable_kernel_traces()

    # test debug traces
    logger.enable_debug_traces()
    time.sleep(2)
    logger.disable_debug_traces()

    # test traces
    logger.enable_traces("7652996b-4eb4-495e-a7b2-5e0ca6400713", "Always", "")
    time.sleep(2)
    logger.disable_traces()

    # test log injection
    logger.inject_log("Hello world", "GREETINGS", "VERBOSE")

    # test flush
    logger.flush()

if __name__ == "__main__":
    __test_service()
