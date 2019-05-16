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
:summary: This script implements the interface for applications
:since: 03/12/2013
:author: pbluniex
"""
import os
from acs_test_scripts.Device.Model.AndroidDevice.Application.IApplication import IApplication


class IBinary(IApplication):

    """
    Abstract class that defines the basis operations of binary application file
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IApplication.__init__(self, device)

        self.__binary_file = "/data/%s" % self._benchmark_name
        self.__shell_file = "/data/%s.sh" % self._benchmark_name
        self._result_file = "/data/result_%s.txt" % self._benchmark_name
        self._command = "./%s" % self._benchmark_name

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
        app_name = os.path.split(self._application_uri)[1]
        cmd = "mv /data/%s /data/%s" % (app_name, self._benchmark_name)
        self.adb_shell(cmd, 3)


    def post_install(self):
        """
        Post installation actions
        """
        cmd = "echo '%s' > %s" % (self._command, self.__shell_file)
        self.adb_shell(cmd, 3)

        cmd = "chmod 0777 %s %s" % (self.__binary_file, self.__shell_file)
        self.adb_shell(cmd, 3)

    def wait(self, timeout):
        """
        Run the benchmark

        :type timout: integer
        :param timeout: The timeout beyond the test should have end
        """
        self._run_no += 1
        self.adb_shell("cd /data ; rm -f %s; nohup sh %s > %s" %
                       (self._result_file, self.__shell_file, self._result_file), 15)
        self._wait_for_application(self._benchmark_name, timeout)

    def uninstall(self):
        """
        Uninstall an application
        """
        self.adb_shell("rm %s %s %s" %
                       (self.__binary_file, self.__shell_file, self._result_file), 3)
