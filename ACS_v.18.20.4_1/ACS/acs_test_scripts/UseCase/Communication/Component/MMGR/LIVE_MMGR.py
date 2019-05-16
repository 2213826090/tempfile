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
:summary: This file implements the MMGR testing Use Case
:since: 13/02/2013
:author: lakuex
"""
# Module name follows ACS conventions
# pylint: disable=C0103

import re
import time
import fnmatch
import os

from shutil import copyfile
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.GTester.PhoneWrapper import AndroidDeviceWrapper
from ErrorHandling.AcsConfigException import AcsConfigException


class LiveMMGR(UseCaseBase):

    """
    LIVE MMGR test use case class.
    """

    PLATFORM_CONF_FILE = "config_name"
    """
    Original platform configuration file
    """

    DEFAULT_PLATFORM_SCALABILITY_PATH = "/system/etc/telephony/"
    """
    Default path for scalability configuration files
    """

#------------------------------------------------------------------------------

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Initialize mmgr-test parameters
        self._test_number = None
        self._expected_result = None
        self._test_timeout = None
        self._platform_config_path = None
        self._config_scalability_path = None
        self._config_mmgr_file = None
        self._config_mmgr_file_backup = None
        self._max_modem_cold_reset = None
        self._platform_config_file = None

        # We will need a phone wrapper
        self.phone_wrapper = AndroidDeviceWrapper.get_instance(self._device)

        # Get the test number to execute
        test_number = self._tc_parameters.get_param_value("TEST_NUMBER")
        if test_number is not None and test_number != "":
            self._test_number = test_number

        # Get the expected result
        expected_result = \
            self._tc_parameters.get_param_value("EXPECTED_RESULT")
        if expected_result is not None and expected_result != "":
            self._expected_result = expected_result

        # Get the execution timeout
        test_timeout = self._tc_parameters.get_param_value("EXECUTION_TIMEOUT")
        if test_timeout is not None and test_timeout != "":
            self._test_timeout = int(test_timeout)

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

        # Get the maximum number of cold reset allowed during the test
        max_modem_cold_reset = self._tc_parameters.get_param_value("MAX_MODEM_COLD_RESET")
        if max_modem_cold_reset is not None and max_modem_cold_reset != "":
            self._max_modem_cold_reset = max_modem_cold_reset

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test.
        """
        # Run UC base run_test
        UseCaseBase.set_up(self)

        # Check the test number before going any further
        if self._test_number is None:
            message = "Invalid parameter value: %s for parameter '%s'." % (
                str(self._test_number),
                "TEST_NUMBER")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        # Check the expected result str before going any further
        if self._expected_result is None:
            message = "Invalid parameter value: %s for parameter '%s'." % (
                str(self._expected_result),
                "EXPECTED_RESULT")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        # Check the test execution timeout before going any further
        if self._test_timeout is None:
            message = "Invalid parameter value: %s for parameter '%s'." % (
                str(self._test_timeout),
                "EXECUTION_TIMEOUT")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        # Check the path of the config_name
        # file on the DUT before going any further
        if self._platform_config_path is None:
            message = "Invalid parameter value: %s for parameter '%s'." % (
                str(self._platform_config_path),
                "TARGET_PLATFORM_CONFIG_PATH")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        # Check the path on the DUT for the SW scalability config
        # file before going any further
        if self._config_scalability_path is None:
            message = "Invalid parameter value: %s for parameter '%s'." % (
                str(self._config_scalability_path),
                "TARGET_CONFIG_SCALABILITY_PATH")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        # Check the value of the maximum number of cold reset allowed
        if self._max_modem_cold_reset is None:
            message = "Invalid parameter value: %s for parameter '%s'." % (
                str(self._max_modem_cold_reset),
                "MAX_MODEM_COLD_RESET")
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, message)

        # ADB disable-verity
        cmd = "adb disable-verity"
        (return_code, output) = self._device.run_cmd(cmd,
                                                    10,
                                                    force_execution=True)
        if return_code is Global.FAILURE:
            return_msg = "Command %s has failed" % cmd
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, return_msg)

        # Reboot the device
        self.phone_wrapper.reboot()

        # Remount the device's filesystem
        self.phone_wrapper.remount()

        # Retrieve the platform configuration file
        self._logger.debug("Retrieve platform configuration name.")
        self.phone_wrapper.pull(self._platform_config_path+self.PLATFORM_CONF_FILE, ".")

        content = None
        try:
            # Open the platform config file, and store the platform
            # scalability configuration
            with open(self.PLATFORM_CONF_FILE, 'r') as file_object:
                content = file_object.readlines()

        # Catch possible IO Errord
        except IOError as io_error:
            message = "I/O error({0}): {1}".format(
                io_error.errno,
                io_error.strerror)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, message)
        # Catch other kind of error/exceptions
        # (errors/exceptions possibly raised by 'readlines' are not documented)
        except:
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, "Could not change the configuration file.")

        self._platform_config_file = ''.join(content)[:-1]+".xml"

        # Retrieve the platform config file (XMMxxx_conf_x.xml)
        self._logger.debug("Retrieve platform configuration file.")
        self.phone_wrapper.pull(self._config_scalability_path + self._platform_config_file, ".")

        content = None
        try:
            # Open the platform config file, and store the platform
            # scalability configuration
            with open(self._platform_config_file, 'r') as file_object:
                # Parse the platform scalability file to identify
                # the corresponding mmgr config file
                for x in file_object.readlines():
                    if 'mmgr_xml' in x:
                        content = x
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
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, "Could not change the configuration file.")

        # build the name of  the mmgr file
        self._config_mmgr_file = content[content.find("\"")+1:(len(content)-2)]

        # Retrieve the mmgr config file
        self.phone_wrapper.pull(self._config_scalability_path + self._config_mmgr_file, ".")

        # Create a backup for mmgr_config file to restore initial configuration
        self._config_mmgr_file_backup = "backup_"+self._config_mmgr_file
        copyfile(self._config_mmgr_file, self._config_mmgr_file_backup)

        content = None
        try:
            # Open the file, and change the MaxModemColdReset value
            with open(self._config_mmgr_file, 'r') as file_object:
                # As the mmgr.conf file is small, we can allow
                # ourselves to store all the lines of the file
                # at the same time in a variable without
                # risking memory issues.
                content = [re.sub("[0-9]+", str(self._max_modem_cold_reset), x)
                           if fnmatch.fnmatch(x, "*cold_reset=*")
                           else x
                           for x in file_object.readlines()]

            with open(self._config_mmgr_file, 'w') as file_object:
                # Convert the content to a str
                content = "".join(map(str, content))  # pylint: disable=W0141
                file_object.write(content)

        # Catch possible IO Errord
        except IOError as io_error:
            message = "I/O error({0}): {1}".format(
                io_error.errno,
                io_error.strerror)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, message)
        # Catch other kind of error/exceptions
        # (errors/exceptions possibly raised by 'readlines' are not documented)
        except:
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, "Could not change the configuration file.")

        # Remount the device's filesystem
        self.phone_wrapper.remount()

        # Push the file on the target
        self._device.push(self._config_mmgr_file, self._config_scalability_path, 10)

        # Reboot the DUT to apply the new configuration
        self.phone_wrapper.reboot()

        # Return the status
        return Global.SUCCESS, "No error."

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Run UC base run_test
        UseCaseBase.run_test(self)

        # Create the shell command
        cmd = "adb shell mmgr-test -f -t %s" % (str(self._test_number))

        # Run the shell command
        (return_code, output) = self._device.run_cmd(cmd,
                                                     self._test_timeout,
                                                     force_execution=True)

        # Check the error code on the stdout.
        output = output.translate(None, '\r\n')
        matcher = re.search("%s" % self._expected_result, output)
        if matcher is not None:
            return_code = Global.SUCCESS
            return_msg = "mmgr-test success. Found expected result: %s." % (
                str(self._expected_result))
        else:
            return_code = Global.FAILURE
            return_msg = "mmgr-test test failed. Expected result was %s." % (
                str(self._expected_result))
            self._logger.error(return_msg)

        # Wait until the modem is up before going any further in the campaign
        self._logger.debug("Waiting for the modem to be up")

        # The modem takes up to 10 secs to be declared UP
        time.sleep(15)

        # Return the verdict of the flash
        return return_code, return_msg

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        Disposes this test.
        """
        # Run the inherited 'tear_down' method
        UseCaseBase.tear_down(self)

        #
        # Now configure the platform in order to dispose the test
        #

        # Remount the device's filesystem
        self.phone_wrapper.remount()

        # Push back the original configuration file on the target
        self._device.push(self._config_mmgr_file_backup, self._config_scalability_path+self._config_mmgr_file, 10)

        # Reboot the DUT if needed
        if self.phone_wrapper.is_reboot_advised:
            self.phone_wrapper.reboot()

        # Delete the local files
        self._logger.debug("Removing local files.")
        os.unlink(self.PLATFORM_CONF_FILE)
        os.unlink(self._platform_config_file)
        os.unlink(self._config_mmgr_file)
        os.unlink(self._config_mmgr_file_backup)

        # Return the verdict
        return Global.SUCCESS, "No error."
