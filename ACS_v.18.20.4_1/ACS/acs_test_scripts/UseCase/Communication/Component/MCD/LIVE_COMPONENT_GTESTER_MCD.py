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
:summary: This file implements MCD Use Case via GTester.
:since: 24/07/2013
:author: jreynaux
"""
# Module name follows ACS conventions
# pylint: disable=C0103
import time

from UtilitiesFWK.Utilities import Global
# We locally disable some errors that PyDev may see
# pylint: disable=F0401,E0611
from acs_test_scripts.UseCase.Communication.Component.LIVE_GTESTER_BASE import LiveGTesterBase
from Core.Report.SecondaryTestReport import SecondaryTestReport
# We restore the previously disabled errors.
# pylint: enable=F0401,E0611
from ErrorHandling.AcsConfigException import AcsConfigException

class LiveComponentGTesterMcd(LiveGTesterBase):

    """
    Live GTester MCD Use Case class.
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Call UseCase base __init__ method
        LiveGTesterBase.__init__(self, tc_name, global_config)

        # Specific attributes
        # Some attributes are initialized to empty
        # strings to avoid Pylint warning
        self.__command = None
        self.__test_name = None

        # Enable ACS secondary reports
        self._secondary_report = SecondaryTestReport(
            self._device.get_report_tree().get_report_path())

    def set_up(self):
        """
        Initializes this test.
        """
        # Run the inherited 'set_up' method
        LiveGTesterBase.set_up(self)

        # Retrieve the command from parameters
        self.__command = self._tc_parameters.get_param_value("COMMAND")

        # Retrieve the test name from command
        self.__test_name = self._get_test_name(self.__command)

        # Format the command
        self.__command = self.parser.build_gtester_command(self.__command)
        self.__command = "adb shell %s" % self.__command

        # Return the verdict
        return Global.SUCCESS, "No error."

    def run_test(self):
        """
        Executes the test.
        """
        # Run the inherited 'run_test' method
        LiveGTesterBase.run_test(self)

        time.sleep(240)

        # Run the command
        self._logger.debug("Run command.")
        (return_code, command_response) = self.runner.run_command(self.__command)

        # check if the MCD-test command receives a response; no TIMEOUT error present
        if "PASS: /system/bin/mcd-test" in str(command_response):
            # Retrieve the result
            result_object = self.runner.last_result
            # Parse and compute results
            # Comments regarding the result
            (verdict, output) = self._parse_and_compute_result(result_object, self.__test_name, self._secondary_report)
        else:
            message = "TIMEOUT reached; response not succesfully received"
            self._logger.error(message)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, message)

        # Return the verdict
        return verdict, output

    #------------------------------------------------------------------------------

    def tear_down(self):
        """
        Disposes this test.
        """
        # Run the inherited 'tear_down' method
        LiveGTesterBase.tear_down(self)

        # Reboot the device to return to original state
        self._device.reboot()

        # Return the verdict
        return Global.SUCCESS, "No error."

#------------------------------------------------------------------------------
