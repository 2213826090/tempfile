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

:organization: PSTV-BA
:summary: This script implements Iozone performance benchmark.
:since: 10/07/2014
:author: Jayesh Kumar Tank
:History: Implemented the base version of this benchmark.
"""
import time
from acs_test_scripts.Device.Model.AndroidDevice.Application.IBinary import IBinary
from ErrorHandling.DeviceException import DeviceException


class Iozone(IBinary):
    """
    Iozone benchmark implementation
    """

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IBinary.__init__(self, device)
        self.is_lower_better = False
        self._logger = device.get_logger()
        self.__iozone_test = None
        self._result_file = None
        self.__tests = ("SEQUENCIAL", "RANDOM")
        self._results = {"write": [], "read": []}
        self.__shell_file = None

        self.command = (
        "/data/iozone -azec -+n -e -L64 -S32 -r64k -s1G -i0 -i1 -f /data/iozone.tmp > /data/seq-run.log",
        "/data/iozone -azec -+n -e -I -L64 -S32 -r4k -s1G -O -i0 -i2 -f /data/iozone.tmp > /data/random-run.log")

    def install(self, appuri, additionnals=None, arguments=None, destination=None):
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
        IBinary.install(self, appuri, additionnals, arguments, destination)

    def start(self):
        """ create results files """

        if self.__iozone_test == self.__tests[0]:
            # log file for Sequential read/write
            self._result_file = "/data/seq-run.log"
        else:
            # log file for Random read/write
            self._result_file = "/data/random-run.log"
        self._logger.info("iozone result file is : %s", self._result_file)

    def wait(self, timeout):
        """
        Run the benchmark in parameter

        :type timeout: integer
        :parame timeout: Timeout until the benchmark should end
        """

        if self.__iozone_test == self.__tests[0]:
            # run Sequential read/write
            self.adb_shell(self.command[0], timeout * 10)
        else:
            # run Random read/write
            self.adb_shell(self.command[1], timeout * 10)

    def _fetch_result(self):
        """
        Get score for Iozone
        """
    # get score for read/write
        result_file = self._fetch_file()
        if result_file is None:
            raise AcsConfigException(AcsDeviceException.FILE_NOT_FOUND,
                                "There is no Result file generated, Aborting.")

    #Uncomment to get debug information
    #self._logger.info("iozone result file content is : %s", str(result_file))

        time.sleep(10)
        num_lines = result_file.count('\n')
        if self.__iozone_test == self.__tests[0]:
            self._results["write"].append(int(result_file.splitlines()[num_lines - 3].strip().split()[2]) / 1024)
            self._results["read"].append(int(result_file.splitlines()[num_lines - 3].strip().split()[4]) / 1024)
        else:
            self._results["read"].append(int(result_file.splitlines()[num_lines - 3].strip().split()[4]))
            self._results["write"].append(int(result_file.splitlines()[num_lines - 3].strip().split()[5]))

    def post_install(self):
        """
        Post installation configurations
        """
        if self._arguments not in self.__tests:
            raise AcsConfigException(AcsConfigException.INVALID_TEST_CASE_FILE,
                    "There is no test named %s" % self._arguments)

        self.__iozone_test = self._arguments
        self._logger.info("iozone test is : %s", self.__iozone_test)

        #Do a sequential write on the test partition:
        command = "dd if=/dev/zero of=/data/testfile bs=4096"
        self.adb_shell(command, 10)

        #delete this test file
        command = "rm -r /data/testfile"
        self.adb_shell(command, 3)

        IBinary.post_install(self)

    def uninstall(self):
        """
        Remove files after the test is completed
        """
        self.adb_shell("rm /data/iozone.sh; rm /data/iozone; rm /data/seq-run.log; rm /data/random-run.log", 10)