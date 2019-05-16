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
:summary: This script implements the interface for tarball files
:since: 18/02/2013
:author: pbluniex
"""

import os
import tarfile
from acs_test_scripts.Device.Model.AndroidDevice.Application.IApplication import IApplication
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


class ITarball(IApplication):

    """
    Abstract class that defines basis operation of application embedded in
    tarball file
    """

    _root_dir = None

    def __init__(self, device):
        """
        Initializes this instance.

        :type device: Device
        :param device: The DUT
        """
        IApplication.__init__(self, device)

    def __extract(self, appfile, destination):
        """
        Extract tarball archive on device destination directory.
        This method will detect if tarball has a root directory and will
        create it if no.

        :type appfile: str
        :param appfile: tarball file to extract

        :type destination: str
        :param destination: Path where to extract tarball
        """

        filename = os.path.basename(appfile)

        tar_file = tarfile.open(appfile)
        tar_members = tar_file.getmembers()
        tar_file.close()

        if not tar_members[0].isdir():
            # Create root directory from archive filename
            tarpos = filename.find(".tar")
            if tarpos == -1:
                tarpos = filename.find(".tgz")

            if tarpos == -1:
                raise AcsConfigException(AcsConfigException.OPERATION_FAILED,
                                         "Bad format for benchmark filename : %s" % filename)

            self._root_dir = "%s/%s" % (destination, filename[:tarpos])
            self.adb_shell("rm -r %s" % self._root_dir, 3)
            self.adb_shell("mkdir %s" % self._root_dir, 3)
            self.adb_shell("mv %s/%s %s" % (destination, filename, self._root_dir), 3)
            destination = self._root_dir
        else:
            self._root_dir = "%s/%s" % (destination, tar_members[0].name)

        res = self.adb_shell("cd %s && tar xf %s && echo Ok" %
                             (destination, filename), 300)
        self.adb_shell("rm %s/%s" % (destination, filename), 3)

        if res != "Ok":
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Unable to extract benchmark file : %s" % res[1])

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
        IApplication.install(self, appuri, additionnals, arguments, destination)

        self.__extract(self._application_uri, "/data")

    def uninstall(self):
        """
        Uninstall the application on the device
        """
        self.adb_shell("rm -r %s" % self._root_dir, 10)
