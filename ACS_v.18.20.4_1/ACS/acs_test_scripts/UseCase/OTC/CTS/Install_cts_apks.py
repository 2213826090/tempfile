"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
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

:organization: INTEL OTC ANDROID QA
:description: This TC sets CTS prerequisites
:since: 15/10/14
:author: rcstamat
"""

import __builtin__
import os
import zipfile

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Utilities.HttpDownloaderUtil import HttpDownloaderUtil

SysErrors = __builtin__.OSError, __builtin__.IOError,
CTS_MEDIA_COPY_SCRIPT = "copy_media.sh"

class InstallCTSApks(UseCaseBase):

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)
        self._system_api = self._device.get_uecmd("System")
        self._app_api = self._device.get_uecmd("AppMgmt")

    def initialize(self):
        """
        Process the **<Initialize>** section of the XML file and execute defined test steps.
        """
        UseCaseBase.initialize(self)
        result, output = Global.SUCCESS, ""

        self._cts_path = self._tc_parameters.get_param_value("CTS_PATH")
        self._cts_media_path = self._tc_parameters.get_param_value("CTS_MEDIA_PATH")
        # Retrieve artifact manager
        self._artifact_manager = self._em.get_artifact_manager("ARTIFACT_MANAGER")
        self._timeout = self._tc_parameters.get_param_value(param="DOWNLOAD_TIMEOUT", default_value=7000.0,
                                                            default_cast_type=float)
        return result, output

    def _extract(self, file_path):
        fileName, fileExtension = os.path.splitext(file_path)
        if fileExtension.lower() == ".zip":
            zip_file = zipfile.ZipFile(file_path, "r")
            zip_file.extractall(fileName)
            zip_file.close()

    def _install_cts_apks(self, path):
        """
        :param path:
        :return:
        """
        apk_to_install = ["CtsDeviceAdmin.apk"]
        error_msg = ""
        verdict = Global.SUCCESS

        for apk in apk_to_install:
            found = False
            for root, _, file_names in os.walk(path):
                for file_ in file_names:
                    if apk in file_:
                        found = True
                        apk_full_path = os.path.join(root, file_)
                        (verdict, error_msg) = self._app_api.install_device_app(apk_full_path)
                        if verdict != Global.SUCCESS:
                            break
                if verdict != Global.SUCCESS:
                    break
            if verdict != Global.SUCCESS:
                break

        if not found:
            verdict = Global.FAILURE
            error_msg = "None of these necessaries APKs has been found: %s" % (" , ".join(apk_to_install))

        return verdict, error_msg

    def _run_copy_media(self):
        """
        Execute copy_media.sh script from CTS media zip
        :return:
        """
        error_msg = ""
        verdict = Global.SUCCESS
        found = False
        sys_path = os.getcwd()
        try:
            for root, _, file_names in os.walk(self._cts_media_path):
                for file_ in file_names:
                    if CTS_MEDIA_COPY_SCRIPT in file_:
                        found = True
                        os.chdir(root)
                        script_full_path = 'bash %s all -s %s' %(file_, self._device._serial_number)
                        self._logger.info("Run CTS copy_media script: %s" % script_full_path)
                        verdict, error_msg = self._device.run_cmd(script_full_path, 1200)
                        if verdict != Global.SUCCESS:
                            break
                if verdict != Global.SUCCESS:
                    break
            if not found:
                verdict = Global.FAILURE
                error_msg = "CTS media copy script: %s not found in: %s" % (CTS_MEDIA_COPY_SCRIPT, self._cts_media_path)
        except Exception, err:
            verdict = Global.FAILURE
            error_msg = err
        finally:
            os.chdir(sys_path)
        return verdict, error_msg

    def _check_media_on_dut(self):
        """
        Check if media is not already pushed on DUT
        :return:
        """
        push_check = "adb shell du -hs /sdcard/test"
        verdict, error_msg = self._device.run_cmd(push_check, 20)
        error_msg = error_msg.strip()
        if verdict == Global.SUCCESS:
            if error_msg and "No such file or directory" not in error_msg:
                size_x = error_msg.split()[0].strip()
                if size_x.endswith("G"):
                    self._logger.info("CTS media files are already on DUT here: /sdcard/test")
                    return False
        return True

    def _push_cts_medias(self):
        """
        Push videos needed by CTS tests
            - retrieve cts media files if specified by the user
            - push them on the phone

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        verdict = Global.SUCCESS
        error_msg = ""
        if not self._cts_media_path:
            self._logger.warning("No media specified, some CTS packages require such files and may be failed")
            verdict = Global.SUCCESS
            error_msg = "Nothing to do, no media specified"
        elif self._check_media_on_dut():
            self._logger.info("Retrieve cts media Path = from %s" % self._cts_media_path)
            self._cts_media_path = self._artifact_manager.get_artifact(artifact_name=self._cts_media_path,
                                                                       transfer_timeout=self._timeout)
            if self._cts_media_path.strip():
                # Set cts directory path
                # can be a path to a dir or a zip file
                # in both case, resulting filename will be a directory
                fileName, fileExtension = os.path.splitext(self._cts_media_path)
                if fileExtension.lower() == ".zip":
                    zip_file = zipfile.ZipFile(self._cts_media_path, "r")
                    zip_file.extractall(fileName)
                    zip_file.close()
                # cts path should be a dir.
                if os.path.isdir(fileName):
                    self._cts_media_path = fileName
                    verdict, error_msg = self._run_copy_media()
                    if verdict == Global.FAILURE:
                        self._logger.warning("Cannot push CTS media because: %s"%error_msg)
                    else:
                        self._logger.info("CTS Media pushed properly")
                        error_msg = "CTS Media pushed properly"
                else:
                    verdict, error_msg = Global.FAILURE, "Cannot find CTS media folder"
            else:
                verdict = Global.FAILURE
                error_msg = "Path not valid: ", self._cts_media_path
        return verdict, error_msg

    def set_up(self):
        """
        Initialize the test

        :rtype: tuple
        :return: ACS verdict and msg output
        """

        UseCaseBase.set_up(self)
        return Global.SUCCESS, "No errors"

    def run_test(self):
        """
        Execute the CTS test
        Compute result based on CTS xml result output

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        UseCaseBase.run_test(self)

        full_path = self._artifact_manager.get_artifact(artifact_name=self._cts_path, transfer_timeout=self._timeout)
        if not full_path:
            verdict = Global.FAILURE
            msg = "Path not correct: %s" % full_path
        else:
            self._extract(full_path)
            fileName, fileExtension = os.path.splitext(full_path)
            (verdict, error_msg) = self._install_cts_apks(fileName)

        if verdict == Global.SUCCESS:
            (verdict, error_msg) = self._push_cts_medias()

        return verdict, error_msg

    def tear_down(self):
        """
        End and dispose the test

        :rtype: tuple
        :return: ACS verdict and msg output
        """
        UseCaseBase.tear_down(self)

        return Global.SUCCESS, "No errors"
