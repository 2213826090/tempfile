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
:summary: This file implements a Test Step to install a device application on device
:since:04/03/2014
:author: kturban
"""

import os
import tempfile

from Core.PathManager import Paths
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsToolException import AcsToolException
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.DeviceApplication import AndroidApplication
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager


class InstallApp(DeviceTestStepBase):

    """
    Install a device application on device
    """

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Initialize test step
        """
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._system_api = self._device.get_uecmd("System")
        self._app_api = self._device.get_uecmd("AppMgmt")
        self._equipment_manager = EquipmentManager()
        self._application = AndroidApplication(logger=self._logger)

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """
        DeviceTestStepBase.run(self, context)

        if not os.path.isfile(self._pars.file_path):
            error_msg = "File {0} does not exists!".format(self._pars.file_path)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
        self._application.set_application(self._pars.file_path)
        installing_same_version = self._compare_application_versions()
        # Backup only if a version is installed and is different from the one to install and user requested backup
        if self._pars.backup:
            if not installing_same_version:
                backup_folder = os.path.abspath(os.path.join(Paths.EXECUTION_CONFIG, "Backup"))
                if not os.path.exists(backup_folder):
                    os.makedirs(backup_folder)
                app_location = self._app_api.get_path_of_device_app(self._application.get_package_name())
                if not app_location:
                    self._logger.warning("No application backup done: application is not installed")
                else:
                    backup_file = os.path.join(backup_folder,
                                               os.path.basename(app_location))
                    verdict, msg = self._app_api.backup_app(backup_file, app_location, self._pars.timeout)
                    if verdict != Global.SUCCESS:
                        self._logger.warning("No application backup done: {0}".format(msg))
                    else:
                        self._logger.info("Backup of application done to {0}".format(backup_file))
                    context.set_info(self._pars.backup_file_path, backup_file)
            else:
                self._logger.warning("No application backup done: same version already installed")

        # Sign only if user requested it and installing application
        if self._pars.sign and not installing_same_version:
            app_path = self._sign_app()
            self._logger.info("Signing application done")
        else:
            app_path = self._pars.file_path

        # Install only if not installed or installed version is different from the one to install
        if not installing_same_version:
            allow_downgrade = False
            # parameter is optional
            if self._pars.allow_downgrade is not None:
                allow_downgrade = self._pars.allow_downgrade
            verdict, msg = self._app_api.install_device_app(app_path=app_path,
                                                            timeout=self._pars.timeout,
                                                            allow_downgrade=allow_downgrade)
            if verdict != Global.SUCCESS:
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)
            else:
                msg = "Application {0} installation done".format(os.path.basename(
                    app_path))
                self.ts_verdict_msg = msg
                self._logger.info(msg)
        else:
            msg = "No application installation done: same version already installed"
            self.ts_verdict_msg = msg
            self._logger.warning(msg)

    def _compare_application_versions(self):
        """
            Compare application given by user with application already installed on device
        :return: bool result of the comparison
        """
        new_version = self._application.get_package_version_name()
        installed_version = self._device.get_apk_version(self._application.get_package_name())
        return new_version == installed_version

    def _sign_app(self):
        """
         Sign application using key pointed by user in benchconfig

        :return: path of the signed application
        """
        if self._pars.sign_key_path:
            signing_key = self._pars.sign_key_path
        else:
            signing_key = self._device.get_config("appSigningKey", "")
        if not signing_key:
            error_msg = "appSigningKey parameter is not present in Device Catalog nor in Bench Config"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
        signing_key_1 = "{0}.{1}".format(signing_key, "pk8")
        signing_key_2 = "{0}.{1}".format(signing_key, "x509.pem")
        # Use Artifact manager for non local key files
        if not os.path.isfile(os.path.abspath(signing_key_1)) and not os.path.isfile(os.path.abspath(signing_key_2)):
            artifact_manager = self._equipment_manager.get_artifact_manager("ARTIFACT_MANAGER")
            if not artifact_manager:
                error_msg = "ArtifactManager equipment is not present in the BenchConfig file"
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
            artifact_manager.get_artifact(artifact_name=signing_key_1, transfer_timeout=10)
            local_artifact = artifact_manager.get_artifact(artifact_name=signing_key_2, transfer_timeout=10)
            signing_key_folder = os.path.dirname(local_artifact)
        else:
            signing_key_folder = os.path.dirname(signing_key)
        signing_key_name = os.path.basename(signing_key)
        signing_tmp_folder = tempfile.mkdtemp()
        signed_app = os.path.join(signing_tmp_folder, os.path.basename(self._pars.file_path))
        if signing_key_folder and signing_key_name and signed_app:
            status, status_msg = self._app_api.sign_device_app(self._pars.file_path,
                                                               signing_key_folder,
                                                               signing_key_name,
                                                               signed_app, 5)
            if status != Global.SUCCESS:
                raise AcsToolException(AcsToolException.OPERATION_FAILED, status_msg)
        else:
            error_msg = "Invalid signing data among key folder:{0}, key name:{1}, key application:{2}".format(
                signing_key_folder, signing_key_name, signed_app)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        return signed_app
