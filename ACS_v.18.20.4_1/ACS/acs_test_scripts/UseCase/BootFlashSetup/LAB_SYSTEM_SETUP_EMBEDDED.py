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
:summary: This file implements the UC to setup ACS Agent(s)
:since: 18/09/2014
:author: nbrissox
"""

import os
import tempfile
import time

from os import path

from Device.DeviceManager import DeviceManager
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsToolException import AcsToolException
from Core.PathManager import Paths


class LabSystemSetupEmbedded(UseCaseBase):

    """
    Lab System Setup embedded UC Class.

    Handles Installation of ACS Agent(s) on the DUT.

    """

    APP_FOLDER = "APPLICATIONS"
    INTEL_FOLDER = "Intel"
    REF_FOLDER = "Ref"
    IGNORED_FILES = [".gitignore", ".gitkeep"]

    def __init__(self, tc_name, global_config):
        """
        Constructor.

        :param tc_name: The Test Case Name
        :type tc_name: str

        :param global_config:
        :type global_config:

        """
        # Call UseCase base "__init__" method
        UseCaseBase.__init__(self, tc_name, global_config)

        self._system_api = None
        self._app_api = None
        self._phone_system_api = None
        self._setup_path = None

        self._current_device = None
        self._devices = DeviceManager().get_all_devices()

    def set_up(self):
        """
        Prepare the test.

        """
        # Call UseCase base "set_up" method
        results = UseCaseBase.set_up(self)
        self._setup_path = path.abspath(Paths.EMBEDDED)

        if not path.isdir(self._setup_path):
            message = "Setup embedded folder(s) {0} not found! Abort!".format(self._setup_path)
            self._logger.error(message)
            results = Global.FAILURE, message

        return results

    def run_test(self):
        """
        Execute the test.

        """
        # Call UseCase base "run_test" method
        UseCaseBase.run_test(self)

        # Setup embedded for all defined devices
        for device in self._devices:

            self._current_device = device
            self._system_api = device.get_uecmd("System")
            self._app_api = device.get_uecmd("AppMgmt")
            self._phone_system_api = device.get_uecmd("PhoneSystem")

            device_name = device.whoami().get("device", "")
            if not device_name:
                error_msg = "Device name cannot be found from device instance!"
                raise AcsConfigException(AcsConfigException.INSTANTIATION_ERROR, error_msg)

            # Boot the device if not booted and connect it
            if not device.is_available():
                DeviceManager().boot_device(device_name)

            # disable app verification
            if device.is_rooted():
                self._phone_system_api.disable_antimalware_request()

            # Install applications
            self.__install_applications()
            # start agent if it exists
            acs_agent = device.get_acs_agent()
            if acs_agent:
                # update agent's version in report, as it may have been overwritten
                acs_agent.update_version()
                if not acs_agent.start() or \
                   not acs_agent.wait_for_agent_started(10):
                    raise DeviceException(DeviceException.OPERATION_FAILED,
                                          "Cannot start the agent on {0}!".format(device_name))

            if device.is_rooted() and device.has_intel_os():
                # disable app verification
                self._phone_system_api.set_verify_application(False)

        return Global.SUCCESS, "SUCCESS"

    def __install_applications(self):
        """
        Installs ACS Agent(s) Application(s) on the DUT.

        """
        signing_key_name = None
        signing_key_folder = None

        device = self._current_device
        signing_key = device.get_config("appSigningKey")

        if signing_key and device.has_intel_os():
            artifact_manager = self._em.get_artifact_manager("ARTIFACT_MANAGER")
            artifact_manager.get_artifact(artifact_name='{0}.pk8'.format(signing_key), transfer_timeout=10)
            local_artifact = artifact_manager.get_artifact(artifact_name='{0}.x509.pem'.format(signing_key),
                                                           transfer_timeout=10)
            signing_key_folder = path.dirname(local_artifact)
            signing_key_name = path.basename(signing_key)

        self._logger.info("Setup the device from {0} folder...".format(self._setup_path))
        self._logger.info("SETTING UP {0}".format(self.APP_FOLDER))

        # install from Version/Ref-Intel
        subpath = path.join(device.OS_TYPE, device.get_os_version_name(), self.APP_FOLDER)
        subpath = path.join(subpath, self.INTEL_FOLDER if device.has_intel_os() else self.REF_FOLDER)

        # Agent folder path
        folder_path = path.join(self._setup_path, subpath)

        self.__install_applications_from_folder(folder_path, signing_key_folder, signing_key_name)

    def __install_applications_from_folder(self, folder_path, signing_key_folder=None, signing_key_name=None):
        """
        Installs applications on the DUT based on folder path containing all needed applications.

        :param folder_path: The folder path containing Applications.
        :type folder_path: str

        :param signing_key_folder: The folder where are stored Signing keys.
        :type signing_key_folder: str

        :param signing_key_name: The key name to look up into passed folder.
        :type signing_key_name: str

        :raise DeviceException: Raised when an error occurs while installing application on DUT.

        """
        if not path.isdir(folder_path):
            error_msg = "Applications folder not found {0} !".format(folder_path)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

        for filename in os.listdir(folder_path):
            if filename not in self.IGNORED_FILES:
                app_path = path.join(folder_path, filename)

                # if app signing is needed, do it now
                if signing_key_folder and signing_key_name:
                    app_path = self.__sign_app(app_path, signing_key_folder, signing_key_name)

                status, status_msg = self._app_api.install_device_app(app_path)
                if status != Global.SUCCESS:
                    error_msg = "Unable to install application from {0} : {1}".format(app_path, status_msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

    def __sign_app(self, app_path, signing_key_folder, signing_key_name):
        """
        Sign the given application with the given Signing key.

        :param app_path: The application path to be signed
        :type app_path: str

        :param signing_key_folder: The folder where are stored Signing keys.
        :type signing_key_folder: str

        :param signing_key_name: The key name to look up into passed folder.
        :type signing_key_name: str

        :raise AcsToolException: Raised when an error occurs while signing the Application.

        :return: The path of the signed application.
        :rtype: str

        """
        signing_tmp_folder = tempfile.mkdtemp()

        signed_app = path.join(signing_tmp_folder, path.basename(app_path))
        status, status_msg = self._app_api.sign_device_app(app_path,
                                                           signing_key_folder, signing_key_name, signed_app, 5)
        if status != Global.SUCCESS:
            raise AcsToolException(AcsToolException.OPERATION_FAILED, status_msg)
        else:
            return signed_app
