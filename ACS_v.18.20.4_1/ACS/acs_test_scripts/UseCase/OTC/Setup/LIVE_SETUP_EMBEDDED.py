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

:organization: INTEL OTC
:summary: This file implements the UC to setup embedded
:since: 05/20/2014
:author: agdobrex
"""

import os
import tempfile
import time

from Device.DeviceManager import DeviceManager
from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsToolException import AcsToolException
from Core.PathManager import Paths


class LiveSetupEmbedded(UseCaseBase):

    """
    Class that implements LIVE SETUP EMBEDDED. This UseCase downloads the Agents needed by OTC to run instrumentation
    tests and signs them with the ABT keys.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)
        self._agent_paths = None
        self._ui_api = self._device.get_uecmd("Ui")
        self._ui_api.set_global_config(global_config)

    def set_up(self):
        """
        Prepare the test
        """
        UseCaseBase.set_up(self)
        _agent_path = self._tc_parameters.get_param_value("AGENT_PATH")
        if not _agent_path:
            error_msg = "No agent path has been specified."
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)
        self._agent_paths = _agent_path.split(";")
        return Global.SUCCESS, ""

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        # Setup embedded for all defined devices
        for device in DeviceManager().get_all_devices():

            # get the signing keys from the device's BENCHCFG
            signing_key = device.get_config("appSigningKey", "")
            if not signing_key:
                error_msg = "appSigningKey parameter is not present in Device Catalog nor in Bench Config"
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

            system_api = device.get_uecmd("System")
            app_api = device.get_uecmd("AppMgmt")
            phone_system_api = device.get_uecmd("PhoneSystem")

            device_name = device.whoami().get("device", "")
            if not device_name:
                error_msg = "Device name cannot be found from device instance!"
                raise AcsConfigException(AcsConfigException.INSTANTIATION_ERROR, error_msg)

            # Boot the device if not booted and connect it
            if not device.is_available():
                DeviceManager().boot_device(device_name)

            signing_key_name = None
            signing_key_folder = None

            # Get key to sign apps if necessary
            if not device.has_intel_os():
                signing_key = None
            if signing_key:
                artifact_manager = self._em.get_artifact_manager("ARTIFACT_MANAGER")
                artifact_manager.get_artifact(artifact_name=signing_key+".pk8", transfer_timeout=10)
                local_artifact = artifact_manager.get_artifact(artifact_name=signing_key+".x509.pem", transfer_timeout=10)
                signing_key_folder = os.path.dirname(local_artifact)
                signing_key_name = os.path.basename(signing_key)

            for agent_path in self._agent_paths:
                agent_path = agent_path.strip()
                if not agent_path:
                    error_msg = "Split parameter error. An application path is empty. Did you forget a \";\" ?"
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

                # check if path to ACS_AGENT exists
                if not os.path.exists(agent_path):
                    error_msg = "Application not found {0} !".format(agent_path)
                    raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, error_msg)

                # if app signing is needed, do it now
                if signing_key_folder and signing_key_name:
                    app_path = self.__sign_app(app_api, agent_path, signing_key_folder, signing_key_name)

                # actually install the app
                status, status_msg = app_api.install_device_app(app_path)
                if status != Global.SUCCESS:
                    error_msg = "Unable to install application from {0} : {1}".format(app_path, status_msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)

                # disable app verification
                if device.is_rooted():
                    phone_system_api.set_verify_application(False)

            # Reboot device in order to "clean" the system after install
            device.reboot()

            # Initialize UI api
            self._ui_api.init()

            #disable SetupWizard
            verdict, msg = self._disable_wizard()
            if verdict != Global.SUCCESS:
                error_msg = "Unable to disable SetupWizard. Error Message: {0}".format(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, error_msg)
        return Global.SUCCESS, ""

    def tear_down(self):
        """
        End and dispose the test

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        # Release UI api
        if self._ui_api is not None and self._ui_api.isInitialized:
            self._ui_api.release()

        return Global.SUCCESS, "No errors"

    def __sign_app(self, app_api, app_path, signing_key_folder, signing_key_name):
        signing_tmp_folder = tempfile.mkdtemp()

        signed_app = os.path.join(signing_tmp_folder, os.path.basename(app_path))
        status, status_msg = app_api.sign_device_app(app_path, signing_key_folder, signing_key_name, signed_app, 5)
        if status != Global.SUCCESS:
            raise AcsToolException(AcsToolException.OPERATION_FAILED, status_msg)
        else:
            return signed_app

    def _disable_wizard(self):
        """
        Disable device wizard on the device (device wizard activated on user/userdebug builds)

        :rtype: tuple
        :return: ACS verdict and msg output.
        """
        verdict = Global.FAILURE
        msg = ""
        verdict, msg = self._ui_api.run_operation_set("unlock_wizard")
        if verdict == Global.SUCCESS:
            self._ui_api.run_operation_set("go_home")
            verdict, msg = Global.SUCCESS, "Wizard deactivated !"
        return verdict, msg
