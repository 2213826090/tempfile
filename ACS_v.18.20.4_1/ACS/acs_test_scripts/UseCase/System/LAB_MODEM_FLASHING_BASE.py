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
:summary: This file implements the Modem Flash base Use Case
:since: 06/09/2012
:author: asebbanx
"""
import re
import time
import os.path

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.GTester.PhoneWrapper import AndroidDeviceWrapper
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException


class LabModemFlashingBase(UseCaseBase):

    """
    Lab Modem Flash Use Case base class.
    """
    PLATFORM_CONF_FILE = "config_name"
    """
    Original platform configuration file
    """

    DEFAULT_PLATFORM_SCALABILITY_PATH = "/system/etc/telephony/"
    """
    Default path for scalability configuration files
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Initialize flash attributes
        self._flash_file_path = None
        self._expected_result = None
        self._initial_pref_network = None
        self._fls_pattern = None
        self._modem_fw_initial = None
        self._modem_fw_after_update = None
        self._modem_platform_name = None
        self._modem_swRevision = None
        self._modem_hwRevision = None

        # We will need a phone wrapper
        self.phone_wrapper = AndroidDeviceWrapper.get_instance(self._device)

        # Get the flash file path value from parameter file
        flash_file_path = self._tc_parameters.get_param_value("MODEM_FLASH_FILE_PATH")
        if flash_file_path is not None and flash_file_path != "":
            self._flash_file_path = flash_file_path

        # Get the expected result
        expected_result = self._tc_parameters.get_param_value("EXPECTED_RESULT")
        if expected_result is not None and expected_result != "":
            self._expected_result = expected_result

        # Get the expected result
        self._flash_timeout = None
        flash_timeout = self._tc_parameters.get_param_value("FLASH_TIMEOUT")
        if flash_timeout is not None and flash_timeout != "":
            self._flash_timeout = int(flash_timeout)

        # Get the platform config path
        platform_config_path = self._tc_parameters.get_param_value("TARGET_PLATFORM_CONFIG_PATH")
        if platform_config_path is not None and platform_config_path != "":
            self._platform_config_path = platform_config_path

        # Get the SW scalability config path
        particular_scl_cfg_path = self._device.get_config("platformConfigPath")
        config_scalability_path = self._tc_parameters.get_param_value("TARGET_CONFIG_SCALABILITY_PATH")

        if config_scalability_path is not None and config_scalability_path != "":
            if config_scalability_path == self.DEFAULT_PLATFORM_SCALABILITY_PATH and \
                    particular_scl_cfg_path is not None:
                self._config_scalability_path = particular_scl_cfg_path
            else:
                self._config_scalability_path = config_scalability_path

        # Get the application to use for the flash operation
        flash_app = self._tc_parameters.get_param_value("FLASH_APPLICATION")
        if flash_app is not None and flash_app != "":
            self._flash_app = flash_app

        # Initialize the possible values to look for
        self._possible_values = {
            "FW_FLASHING_MODEM_UPDATE_OK": "modem updated successfully",
            "FW_FLASHING_OPERATION_COMPLETE": "operation has succeed"
        }

        # Instantiate the modem flashing UE Commands
        self._modem_flashing_api = self._device.get_uecmd("ModemFlashing")
        # Instantiate the UEcmds for networking
        self._networking_api = self._device.get_uecmd("Networking")

        # ADB disable-verity
        cmd = "adb disable-verity"
        (return_code, output) = self._device.run_cmd(cmd,
                                                    10,
                                                    force_execution=True)
        # Reboot the device
        self.phone_wrapper.reboot()
        cmd = "adb root"
        (return_code, output) = self._device.run_cmd(cmd,
                                                    10,
                                                    force_execution=True)
        
        time.sleep(5)

        # Remount the device's filesystem
        self.phone_wrapper.remount()

        time.sleep(5)

    def set_up(self):
        """
        Initialize the test.
        """
        # Run UC base set_up
        UseCaseBase.set_up(self)

        # Check the flash file path
        if self._flash_file_path is None:
            message = "Invalid parameter value: %s for parameter '%s'." % \
                (str(self._flash_file_path), "MODEM_FLASH_FILE_PATH")
            self._logger.error(message)
            return Global.FAILURE, message

        # Check the flash timeout before going any further
        if self._flash_timeout is None:
            message = "Invalid parameter value: %s for parameter '%s'." % \
                (str(self._flash_timeout), "FLASH_TIMEOUT")
            self._logger.error(message)
            return Global.FAILURE, message

        # Check the path of the config_name
        # file on the DUT before going any further
        if self._platform_config_path is None:
            message = "Invalid parameter value: %s for parameter '%s'." % (
                str(self._platform_config_path),
                "TARGET_PLATFORM_CONFIG_PATH")
            self._logger.error(message)
            return Global.FAILURE, message

        # Check the path on the DUT for the SW scalability config
        # file before going any further
        if self._config_scalability_path is None:
            message = "Invalid parameter value: %s for parameter '%s'." % (
                str(self._config_scalability_path),
                "TARGET_CONFIG_SCALABILITY_PATH")
            self._logger.error(message)
            return Global.FAILURE, message

        # ADB disable-verity
        cmd = "adb disable-verity"
        (return_code, output) = self._device.run_cmd(cmd,
                                                    10,
                                                    force_execution=True)
        if return_code is Global.FAILURE:
            return_msg = "Command %s has failed" % cmd
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, return_msg)
        elif output in "Now reboot your device":
            # Reboot the device
            self.phone_wrapper.reboot()

            # Remount the device's filesystem
            self.phone_wrapper.remount()

            time.sleep(5)

        # Retrieve the platform scalability file name
        self._logger.debug("Retrieve platform scalability file name.")
        self._platform_scalability_file = self.__get_platform_config_file_name(
                                                        self._platform_config_path,
                                                        self.PLATFORM_CONF_FILE)

        # Extract modem information needed to identify the fls file to be used later
        (self._modem_platform_name, self._modem_swRevision, self._modem_hwRevision) = self.__get_modem_info(
                                                                                    self._config_scalability_path,
                                                                                    self._platform_scalability_file)

        # Identify the modem pattern to be used in order to find the right fls file
        # Since the pattern is different for 7480 adding check for 7480
        if str(self._modem_platform_name) == "7480":
            self._fls_pattern = "XMM_%s_*" % (str(self._modem_platform_name))
        else:

            self._fls_pattern = "XMM_%s_REV%s*V%s*" % (
                str(self._modem_platform_name),
                str(self._modem_hwRevision),
                str(self._modem_swRevision))
        self._logger.debug("modem pattern to be used:%s." % self._fls_pattern)

        # extract correct flash file to be used for modem fw update
        self._flash_file = self.__get_fls(self._fls_pattern)
        self._logger.debug("modem file to be used for modem fw update:%s." % self._flash_file)

        return Global.SUCCESS, "No error."

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        # Initialize the return message and return code
        return_msg = None
        return_code = None

        # Retrieve initial modem version
        self._modem_fw_initial = self.__get_mdm_version()

        # Call the flash sequence with the provided flashing application
        (return_code, stdout) = \
            self._modem_flashing_api.flash_fw(
                self._flash_file,
                self._flash_timeout,
                self._flash_app)

        # Check the error code on the stdout.
        stdout = stdout.translate(None, '\r\n')
        expected_result = self._possible_values[self._expected_result]
        matcher = re.search("%s" % expected_result, stdout)
        if matcher is not None:
            return_code = Global.SUCCESS
            return_msg = "Flash test success. Found expected result: %s (%s)." % \
                (str(self._expected_result), expected_result)
        else:
            message = "Flash test failed. Expected flash result was %s (%s)." % (
                str(self._expected_result),
                str(expected_result))
            raise DeviceException(
                DeviceException.OPERATION_FAILED,
                    message)

        # Wait until the modem is up before
        # going any further in the campaign
        self._logger.debug("Waiting for the modem to be up")
        time.sleep(15)

        # Retrieve modem version after modem fw update
        self._modem_fw_after_update = self.__get_mdm_version()

        # Check that modem version after update is same as the initial one
        if (self._modem_fw_initial != self._modem_fw_after_update):
            return_code = Global.FAILURE
            return_msg = "Modem version after update %s is different from the initial one %s" % (
                str(self._modem_fw_initial),
                str(self._modem_fw_after_update))
        else:
            return_code = Global.SUCCESS
            return_msg = "Modem version after update %s is similar to the initial one %s" % (
                str(self._modem_fw_initial),
                str(self._modem_fw_after_update))

        # Return the verdict of the flash
        return return_code, return_msg

    def tear_down(self):
        """
        Disposes this test.
        """
        # Run the inherited 'tear_down' method
        UseCaseBase.tear_down(self)

        # Delete the local files
        self._logger.debug("Removing local files.")
        os.unlink(self.PLATFORM_CONF_FILE)

        # Return the verdict
        return Global.SUCCESS, "No error."

    def __get_platform_config_file_name(self, pltf_config_path, pltf_config_name):
        """
        Returns an str representation of the platform scalability
        file name (i.e. XMM7160_CONF_6.xml)

        :param pltf_config_path: the absolute path of the platform_config file.
        :type pltf_config_path: str

        :param pltf_config_name: the name of the platform_config file.
        :type pltf_config_name: str

        :rtype: str
        :return: the platform scalability file name as str or raise error
        """
        self.phone_wrapper.pull(pltf_config_path+pltf_config_name, ".")

        content = None
        try:
            # Open the platform config file, and store the platform
            # scalability configuration
            with open(pltf_config_name, 'r') as file_object:
                content = file_object.readlines()

        # Catch possible IO Errors
        except IOError as io_error:
            message = "I/O error({0}): {1}".format(
                io_error.errno,
                io_error.strerror)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, message)
        # Catch other kind of error/exceptions
        # (errors/exceptions possibly raised by 'readlines' are not documented)
        except:
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, "Could not change the configuration file.")

        platform_config_file = ''.join(content)[:-1]+".xml"
        return platform_config_file

    #------------------------------------------------------------------------------

    def __get_modem_info(self, sclb_path, sclb_name):
        """
        Parses the scalability file and returns modem specific information

        :param sclb_path: the absolute path of the scalability file.
        :type sclb_path: str

        :param sclb_name: the name of the scalability file.
        :type sclb_name: str

        :rtype: str
        :return: name & SW revision & HW revision of the modem used or raise error
        """
        # Retrieve the platform scalability file (XMMxxx_conf_x.xml)
        self._logger.debug("Retrieve platform scalability file from the target.")
        self.phone_wrapper.pull(sclb_path + sclb_name, ".")

        try:
            # Open the scalability file, and stores modem info
            # like modem name, SW revision and HW revision
            with open(self._platform_scalability_file, 'r') as file_object:
                # Parse the platform scalability file to identify
                # the corresponding mmgr config file
                for line in file_object.readlines():
                    if 'name' in line:
                        platform = line
                    if 'swRevision' in line:
                        swRevision = line
                    if 'hwRevision' in line:
                        hwRevision = line
                        break

        # Catch possible IO Errord
        except IOError as io_error:
            message = "I/O error({0}): {1}".format(
                io_error.errno,
                io_error.strerror)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, message)
        # Catch other kind of error/exceptions
        # (errors/exceptions possibly raised by 'readlines' are not documented)
        except:
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, "Could not read the configuration file.")
        return self.__get_substring(platform), self.__get_substring(swRevision), self.__get_substring(hwRevision)

    #------------------------------------------------------------------------------

    def __get_substring(self, string_value):
        """
        Returns the C{str} from a given text, representing modem
        relevant data (from scalability file)

        :param string_value: the str extracted from scalability file
        :type string_value: str

        :rtype: str
        :return: the substring representation of modem info
        """
        return string_value[string_value.find("\"")+1:(len(string_value)-2)]

    #------------------------------------------------------------------------------

    def __get_fls(self, pattern):
        """
        Returns the correct modem fw file needed for the tested platform

        :param pattern: the modem information needed to extract the correct fls file
        :type string_value: str

        :rtype: str
        :return: the modem fw file or raise error
        """
        # Get the modem fls file
        command = "adb shell ls -R %s%s" % (
            str(self._flash_file_path),
            str(pattern))
        (return_code, output) = self._device.run_cmd(command,
                                                    10,
                                                    force_execution=True)

        # Check whether previous step completed successfully
        if return_code is Global.FAILURE:
            return_msg = "Command %s has failed" % command
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, return_msg)

        # Return the result
        return output

    #------------------------------------------------------------------------------

    def __get_mdm_version(self):
        """
        Returns the modem version from device system properties

        :rtype: str
        :return: the modem version on the target or raise error
        """
        # Get the modem version from target system properties
        modem_fw_cmd = "adb shell getprop gsm.version.baseband"
        (return_code, output) = self._device.run_cmd(modem_fw_cmd,
                                                    10,
                                                    force_execution=True)
        if return_code is Global.FAILURE:
            return_msg = "Command %s has failed" % modem_fw_cmd
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, return_msg)
        elif output is "":
            return_msg = "Modem version returned is empty"
            raise DeviceException(AcsConfigException.OPERATION_FAILED, return_msg)

        # Return the result
        return output
