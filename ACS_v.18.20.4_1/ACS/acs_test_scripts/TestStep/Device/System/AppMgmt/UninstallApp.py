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
:summary: This file implements a Test Step to uninstall a device application
:since:04/03/2014
:author: kturban
"""

import os
from Core.PathManager import Paths
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global


class UninstallApp(DeviceTestStepBase):

    """
    Uninstall a device application
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Initialize test step
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._system_api = self._device.get_uecmd("System")
        self._app_api = self._device.get_uecmd("AppMgmt")

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        # TO DO: add regex check about package name
        backup_file = ""
        if self._pars.backup_app_name != "NO_BACKUP":
            backup_folder = os.path.abspath(os.path.join(Paths.EXECUTION_CONFIG, "Backup"))
            if not os.path.exists(backup_folder):
                os.makedirs(backup_folder)
            backup_file = os.path.join(backup_folder,
                                       os.path.normpath(self._pars.backup_app_name))
            context.set_info(self._pars.backup_file_path, backup_file)

        try:
            verdict, msg = self._app_api.uninstall_device_app(app_name=self._pars.app_name,
                                                              timeout=self._pars.timeout,
                                                              backup_file=backup_file,
                                                              forced=self._pars.forced)
        except AcsConfigException:
            raise

        if verdict != Global.SUCCESS:
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)
        else:
            msg = "App {0} successfully removed from the device".format(os.path.basename(self._pars.app_name))
            self.ts_verdict_msg = msg
            self._logger.info(msg)
