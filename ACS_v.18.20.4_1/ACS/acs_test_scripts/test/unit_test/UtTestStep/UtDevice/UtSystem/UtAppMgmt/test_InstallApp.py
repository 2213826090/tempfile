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
import tempfile
from Core.PathManager import Paths
from mock import patch
from mock import MagicMock
from unit_test.UtTestStep.UTestTestStepBase import UTestTestStepBase
from acs_test_scripts.TestStep.Device.System.AppMgmt.InstallApp import InstallApp
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class InstallAppTestCase(UTestTestStepBase):
    def setUp(self):
        UTestTestStepBase.setUp(self)
        patch("acs_test_scripts.Utilities.HttpDownloaderUtil.os.makedirs").start()
        self._fake_keys = []


    def tearDown(self):
        for key_file in self._fake_keys:
            if os.path.isfile(key_file):
                os.remove(key_file)

    def test_install_app_unexisting_app_path(self):
        """
            Check that unexisting app path will raise error
        """
        teststep = InstallApp(None, None, None, MagicMock())
        teststep._pars.file_path = os.path.join(os.path.dirname(__file__), "doesnotexist")
        # teststep._device = MagicMock()
        with self.assertRaises(AcsConfigException):
            teststep.run(self._context)

    def test_install_app_ko_no_backup(self):
        teststep = InstallApp(None, None, None, MagicMock())
        teststep._pars.file_path = __file__
        # teststep._device = MagicMock()
        # teststep._device._system_api.install_device_app = MagicMock()
        # install failed
        teststep._system_api.install_device_app.return_value = Global.FAILURE, "error install"

        with self.assertRaises(DeviceException):
            teststep.run(self._context)

    def test_install_app_ok_backup_file(self):
        """
            Check that unexisting app path will raise error
        """
        teststep = InstallApp(None, None, None, MagicMock())
        expected_backup_file = os.path.abspath(os.path.join(Paths.EXECUTION_CONFIG, "Backup",
                                               "app_path"))
        teststep._pars.file_path = app_file = __file__
        teststep._pars.backup = True
        teststep._pars.timeout = 10
        teststep._system_api.backup_app.return_value = Global.SUCCESS, "backup ok"
        teststep._system_api.install_device_app.return_value = Global.SUCCESS, "install ok"
        teststep._system_api.get_path_of_device_app.return_value = "app_path"
        teststep.run(self._context)

        teststep._system_api.install_device_app.assert_called_once_with(app_path=app_file, timeout=10, allow_downgrade=False)
        teststep._system_api.backup_app.assert_called_once_with(expected_backup_file, "app_path", 10)

    def test_install_app_ok_sign_ok(self):
        """
            Check that unexisting app path will raise error
        """
        teststep = InstallApp(None, None, None, MagicMock())
        tempfile.mkdtemp = MagicMock()
        tempfile.mkdtemp.return_value = ""
        teststep._pars.sign_key_path = app_file = __file__
        for ext in [".pk8", ".x509.pem"]:
            fake_key_path = app_file + ext
            self._fake_keys.append(fake_key_path)
            with open(fake_key_path, "w") as f:
                f.write("dummy")
        teststep._pars.file_path = app_file
        teststep._pars.sign = True
        teststep._pars.timeout = 10
        teststep._system_api.install_device_app.return_value = Global.SUCCESS, "install ok"
        teststep._system_api.sign_device_app.return_value = Global.SUCCESS, "sign ok"

        teststep.run(self._context)

        teststep._system_api.sign_device_app.assert_called_once_with(__file__, os.path.dirname(__file__),
                                                                     os.path.basename(__file__),
                                                                     os.path.basename(__file__),
                                                                     5)
