# @PydevCodeAnalysisIgnore
# pylint: disable=E0602,W0212,C0103,C0111
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
:summary: Unit test module
:since: 25/02/14
:author: kturban
"""
import os
from Core.PathManager import Paths
from mock import patch
from mock import MagicMock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.System.AppMgmt.UninstallApp import UninstallApp
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class UninstallAppTestCase(UTestTestStepBase):
    def setUp(self):
        UTestTestStepBase.setUp(self)
        patch("TestStep.Device.System.AppMgmt.UninstallApp.os.makedirs").start()

    def test_uninstall_app_ko_operation_failed_no_backup(self):
        teststep = UninstallApp(None, None, None, MagicMock())
        teststep._pars.app_name = "android.package.myapp"
        teststep._pars.backup_app_name = "NO_BACKUP"
        teststep._pars.timeout = 10
        # uninstall failed
        teststep._system_api.uninstall_device_app.return_value = Global.FAILURE, "error uninstall"

        with self.assertRaises(DeviceException):
            teststep.run(self._context)

    def test_uninstall_app_ok_no_backup(self):
        teststep = UninstallApp(None, None, None, MagicMock())
        teststep._pars.app_name = "android.package.myapp"
        teststep._pars.backup_app_name = "NO_BACKUP"
        teststep._pars.timeout = 10
        # uninstall success
        teststep._system_api.uninstall_device_app.return_value = Global.SUCCESS, "uninstall done"
        teststep.run(self._context)
        # backup_file must be empty because NO_BACKUP specified
        teststep._system_api.uninstall_device_app.assert_called_once_with(app_name="android.package.myapp",
                                                                      backup_file="",
                                                                      timeout=10)

    def test_uninstall_app_ok_operation_failed_backup(self):
        teststep = UninstallApp(None, None, None, MagicMock())
        teststep._pars.app_name = "android.package.myapp"
        teststep._pars.backup_app_name = "my_app.apk"
        teststep._pars.timeout = 10
        # uninstall success
        teststep._system_api.uninstall_device_app.return_value = Global.SUCCESS, "error uninstall"
        # backup location
        expected_backup_file = os.path.abspath(os.path.join(Paths.EXECUTION_CONFIG, "Backup",
                                                            "my_app.apk"))
        teststep.run(self._context)
        teststep._system_api.uninstall_device_app.assert_called_once_with(app_name="android.package.myapp",
                                                                      backup_file=expected_backup_file,
                                                                      timeout=10)
