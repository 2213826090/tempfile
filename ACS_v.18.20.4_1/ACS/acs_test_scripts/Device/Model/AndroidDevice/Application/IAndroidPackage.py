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
:summary: This script implements the interface for android packages
:since: 18/02/2013
:author: pbluniex
"""

import os
import subprocess
from acs_test_scripts.Device.Model.AndroidDevice.Application.IApplication import IApplication
from ErrorHandling.AcsConfigException import AcsConfigException


class IAndroidPackage(IApplication):

    """
    Abstract class that defines basis operations of android package application
    """

    class _ApkMetaReader:

        """
        Subclass to read meta data of android package
        """

        def __init__(self, apk_file):
            """
            Constructeur

            :type apk_file: str
            :param apk_file: Apk path
            """
            self.__apk_infos = {}
            self.__apk_file = apk_file

        def __parse_data(self, string):
            """
            Parse an element of line with format "'datum1', 'datum2'" ...

            :type str: str
            :param str: output of aapt dump badging
            """
            data = string.split(",")
            ret = list()

            for datum in data:
                ret.append(datum.strip("'"))

            return ret

        def __parse_keys(self, args):
            """
            Parse an element of line with format key1="value1" key2="'value2'" ...

            :type args: list
            :param args: list of "key=value" of aapt output
            """
            ret = {}
            for elt in args:
                splitted = elt.strip().split("=")
                if len(splitted) == 1:
                    ret[splitted[0]] = None
                else:
                    ret[splitted[0]] = splitted[1].strip("'")

            return ret

        def __parse_values(self, string):
            """
            Parse an element of line after ":"

            :type str: str
            :param str: element of line after ":" of aapt output
            """
            splitted = string.strip().split(" ")

            if len(splitted) > 1:
                return self.__parse_keys(splitted)
            else:
                return self.__parse_data(splitted[0])

        def __parse_line(self, line):
            """
            Parse a line of "aapt dump badging" output

            :type line: str
            :param line: Line of aapt output
            """
            splitted = line.split(":")
            if len(splitted) > 1:
                return splitted[0], self.__parse_values(splitted[1])
            else:
                return splitted[0], None

        def __assert_file(self):
            """
            Assert that file exists and is not None
            """
            if self.__apk_file is None or not os.path.exists(self.__apk_file):
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "APK file is not set or is not a "
                                         "regular file : '%s'" %
                                         str(self.__apk_file))

        def __retrieve_metadata(self):
            """
            Read metadata from local file
            """
            if os.name in ['posix']:
                cmd = ["./aapt", "dump", "badging", self.__apk_file]
            else:
                cmd = ["aapt", "dump", "badging", self.__apk_file]

            try:
                proc = subprocess.Popen(cmd,
                                        stdout=subprocess.PIPE,
                                        stderr=subprocess.STDOUT)
                metadata = proc.stdout.read()
            except subprocess.CalledProcessError as err:
                raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                         "Failed to read metadata from file '%s' (%s)" %
                                        (self.__apk_file, str(err)))

            infos = metadata.splitlines()
            # line is in format : key[: val1='v1'[ val2='v2' ..]]
            for line in infos:
                key, infos_line = self.__parse_line(line)
                self.__apk_infos[key] = infos_line

        def get_infos(self):
            """
            Return the hashtable of retrived apk informations
            """
            if not self.__apk_infos:
                self.__assert_file()
                self.__retrieve_metadata()

            return self.__apk_infos

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IApplication.__init__(self, device)
        self.__app_name = None
        self.__launcher = None
        self.__version = None

    def _get_application_name(self):
        """
        Get the application name read in package
        """
        return self.__app_name

    def _get_launcher(self):
        """
        Get the launcher
        """
        return self.__launcher

    def _get_version(self):
        """
        get version of the package
        """
        return self.__version

    def _set_launcher(self, launcher):
        """
        Set the launcher

        :type launcher: str
        :param launcher: The new launcher to start application
        """
        self.__launcher = launcher

    def install(self, appuri, additionnals=None, arguments=None, url=None, destination=None):
        """
        Install the application on the device

        :type appuri: String
        :param appuri: The full path to the application file

        :type additionnals: String
        :param additionnals: The full path of additionnals elements to run the
                             application

        :type arguments: String
        :param arguments: The arguments of the application. May be everything
                          the application need to run.

        :type destination: String
        :param destination: The directory where the application will be installed
        """
        IApplication.install(self, appuri, additionnals, arguments, url, destination)

        metadata = self._ApkMetaReader(self._application_uri)
        apk_infos = metadata.get_infos()

        self.__app_name = apk_infos["package"]["name"]
        self.__version = apk_infos["package"]["versionName"]

        if "launchable-activity" in apk_infos:
            self.__launcher = apk_infos["launchable-activity"]["name"]

    def uninstall(self):
        """
        Uninstall the application on the device
        """
        if self.__app_name is not None:
            self.adb_uninstall(self.__app_name, 10)

    def start(self):
        """
        Start the application
        """
        run_apk_cmd = "am start "
        run_apk_cmd += "-a android.intent.action.LAUNCHER "
        run_apk_cmd += "-n " + self.__app_name
        run_apk_cmd += "/" + self.__launcher

        self.adb_shell(run_apk_cmd, 3)

    def stop(self):
        """
        Stop the application
        """
        IApplication.stop(self)

        run_apk_cmd = "am force-stop " + self.__app_name
        self.adb_shell(run_apk_cmd, 5)

    def app_is_alive(self):
        """
        Checks if the application is still alive and can be killed
        """
        output = self.adb_shell("ps|grep %s" % self.__app_name, 2)
        if self.__app_name in output:
            return True
        return False
