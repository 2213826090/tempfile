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

:organization: INTEL MCG AMPS
:summary: Device application analysis classes
:author: cbonnard
:since: 09/09/2014
"""

import os
import re
from UtilitiesFWK.Utilities import Global, internal_shell_exec


class AndroidApplication(object):

    """
        Class to provide misc methods for analysing android application
    """

    def __init__(self, path="", logger=None):

        self._logger = logger
        self._path = path
        self._app_ext = ".apk"
        self._default_timeout = 1
        self._pkg_name = None
        self._version_code = None
        self._version_name = None

    def set_application(self, path):
        """
            Set application path

        :type path: str
        :param path: path of application

        """
        self._path = path

    def _log(self, message):
        """
               Log if logger defined

        :type message: str
        :param message: data to be logged
        """
        if self._logger:
            self._logger.debug(message)

    def get_package_name(self):
        """"
            Retrieve package name from an application file

            :rtype: str
            :return: package name
        """
        if not self._pkg_name:
            self._parse_application_data()
        return self._pkg_name

    def get_package_version_name(self):
        """"
            Retrieve package versionName from an application file

            :rtype: str
            :return: package version name
        """
        if not self._version_name:
            self._parse_application_data()
        return self._version_name

    def get_package_version_code(self):
        """"
            Retrieve package versionName from an application file

            :rtype: str
            :return: package version code
        """
        if not self._version_code:
            self._parse_application_data()
        return self._version_code

    def _parse_application_data(self):
        """
            Parse application data package to collect information on the application
        """
        if self._path.endswith(self._app_ext):
            # TODO: if aapt is missing ,this should be clearly indicated
            aapt_cmd_str = "aapt dump badging \"{0}\"".format(self._path)
            if os.name in ['posix']:
                aapt_cmd_str = "./{0}".format(aapt_cmd_str)
            # remove the silent mode to log the error reasons
            status, status_msg = internal_shell_exec(aapt_cmd_str, self._default_timeout,
                                                         log_stdout=False, silent_mode=False)
            if status == Global.SUCCESS:
                if status_msg:
                    self._extract_application_data(status_msg)
            else:
                self._log("aapt command failed: {0}".format(status_msg))
        else:
            self._log("Android application extensions should be .apk")

    def _extract_application_data(self, data):
        """
            Extract various data from parsed str in application package to format:
            package: name='com.intel.test_apk.app' versionCode='1' versionName='1.0'
        :type data: str
        :param data: data to be parsed
        """
        try:
            pattern = re.compile(r"package: name='(.*)' versionCode='(.*)' versionName='(.*)'")
            mth = re.match(pattern, data)
            self._pkg_name = mth.group(1)
            self._version_code = mth.group(2)
            self._version_name = mth.group(3)
        except Exception as e:
            self._log("Cannot find data in application package: {0}".format(e))
