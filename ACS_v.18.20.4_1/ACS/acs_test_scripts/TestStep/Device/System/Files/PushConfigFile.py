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
:summary: This file implements a Test Step to push a config file on device
:since:31/03/2015
:author: gcharlex
"""

import os
from Core.PathManager import Paths
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global


class PushConfigFile(DeviceTestStepBase):
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

        source = os.path.normpath(str(self._pars.source))
        destination = str(self._pars.destination)

        msg = "Push local file '{0}' on the device '{1}'".format(source, destination)
        self._logger.info(msg)
        source_path = os.path.join(Paths.EXECUTION_CONFIG, source)

        if not os.path.exists(source_path):
            error_msg = "File {0} does not exist!".format(self._pars.file_path)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        dirname_destination, basename_destination = os.path.split(destination)

        if not basename_destination:
            basename_source = os.path.basename(source)
            destination_path = os.path.join(dirname_destination, basename_source)
        else:
            destination_path = destination

        verdict, msg = self._device.push(source_path, destination_path)

        if verdict != Global.SUCCESS:
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
