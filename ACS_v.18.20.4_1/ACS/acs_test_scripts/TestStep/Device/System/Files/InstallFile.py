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
:summary: This file implements a Test Step to push a media file/executable on device
:since:12/02/2014
:author: kturban
"""

import os
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global


class InstallFile(DeviceTestStepBase):
    """
    Push file on device
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Initialize test step
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._file_api = self._device.get_uecmd("File")
        self._system_api = self._device.get_uecmd("System")

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        if not os.path.exists(self._pars.file_path):
            error_msg = "File {0} does not exist!".format(self._pars.file_path)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        basename_file = os.path.basename(self._pars.file_path)
        device_destination = self._device.get_device_os_path().join(self._pars.destination, basename_file)

        # First check if the file is already present on the device before trying to push it and uncompress it if needed
        verdict, msg = self._system_api.check_device_file_install(self._pars.file_path, device_destination)

        if verdict == Global.SUCCESS:
            context.set_info(self._pars.destination_stored_path, device_destination)
            msg = "File {0} already installed/pushed on the device".format(basename_file)
            self.ts_verdict_msg = msg
        else:
            if self._pars.type == "bin":
                # Install executable on the device
                verdict, msg = self._system_api.install_device_executable(self._pars.file_path,
                                                                          self._pars.destination,
                                                                          self._pars.timeout)
            else:
                # Push the media file
                verdict, msg = self._device.push(self._pars.file_path, device_destination, self._pars.timeout)

                if verdict == Global.SUCCESS:
                    # File has been installed / pushed on the device
                    # Second check if the file is present on the device after pushing it
                    to_check = [(self._pars.file_path, device_destination)]
                    if os.path.isdir(self._pars.file_path):
                        to_check = []
                        os_path = self._device.get_device_os_path()
                        for root, _, files in os.walk(self._pars.file_path):
                            for file in files:
                                local_path = os.path.normpath(os.path.join(root, file))
                                dir_path = os.path.normpath(self._pars.file_path)
                                relative_path = os.path.relpath(local_path, dir_path)
                                device_path = os_path.join(device_destination, relative_path)
                                device_path = device_path.replace("\\", "/")
                                device_path = os_path.normpath(device_path)
                                to_check.append((local_path, device_path))
                    for local_path, device_path in to_check:
                        verdict, msg = self._system_api.check_device_file_install(local_path, device_path)
                        if verdict != Global.SUCCESS:
                            break
                    if verdict == Global.SUCCESS:
                        context.set_info(self._pars.destination_stored_path, device_destination)
                        msg = "File {0} successfully installed/pushed on the device".format(basename_file)
                        self.ts_verdict_msg = msg

                        # Uncompress the file if necessary on the device
                        if self._pars.type == "zipped_tarball":
                            unzipped_file = self._file_api.unzip(device_destination)
                            if unzipped_file == "":  #should not hit this, but just in case...
                                verdict = Global.FAILURE
                                msg = "InstallFile: Could not unzip " + device_destination
                            self._file_api.untar(unzipped_file)
                        elif self._pars.type == "tarball":
                            self._file_api.untar(device_destination)
                        elif self._pars.type == "zipped":
                            unzipped_file = self._file_api.unzip(device_destination)
                            if unzipped_file == "":  #should not hit this, but just in case...
                                verdict = Global.FAILURE
                                msg = "InstallFile: Could not unzip " + device_destination

            if verdict != Global.SUCCESS:
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
