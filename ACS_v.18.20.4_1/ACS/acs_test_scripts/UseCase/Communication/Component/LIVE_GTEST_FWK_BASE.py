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
:summary: This file implements base methods for I{GTest Framework} Use Cases.
:since: 07/04/2013
:author: jreynaux
"""
# Module name follows ACS conventions
# pylint: disable=C0103

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.GTestFwk.Parser import GTestFwkResult
from acs_test_scripts.Utilities.GTestFwk.PhoneWrapper import \
    AndroidDeviceWrapper as AndroidDeviceWrapperGTestFwk
from acs_test_scripts.Utilities.GTestFwk.Runner import GTestFwkRunner
from acs_test_scripts.Utilities.GTester.Parser import GCampaignParser
from UtilitiesFWK.Utilities import Global
import os.path
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.DeviceException import DeviceException
# We locally disable some errors that PyDev may see
# pylint: disable=F0401,E0611
# We restore the previously disabled errors.
# pylint: enable=F0401,E0611


class LiveGTestFwkBase(UseCaseBase):

    """
    Live Google Test Framework base Use Case class.
    """

    def __init__(self, tc_name, global_config):
        """
        Initializes this instance.
        """

        # Call UseCase base __init__ method
        UseCaseBase.__init__(self, tc_name, global_config)

        # We will need a phone wrapper
        self.__phone_wrapper = AndroidDeviceWrapperGTestFwk.get_instance(self._device)

        # We may need a runner to execute the commands
        self.__runner = None

        # Specific attributes
        # Some attributes are initilized to None
        # to avoid Pylint warning

        self._command = None
        self._command_timeout = None
        self._gtest_filter = None
        self._gtest_output_folder = None
        self._gtest_output_file = None

    def set_up(self):
        """
        Initializes this test.
        """
        # Run the inherited 'set_up' method
        UseCaseBase.set_up(self)

        self._command_timeout = self._tc_parameters.get_param_value("COMMAND_TIMEOUT")
        self._gtest_filter = self._tc_parameters.get_param_value("GTEST_FILTER")
        self._gtest_output_folder = self._tc_parameters.get_param_value("GTEST_OUTPUT_FOLDER")
        self._gtest_output_file = self._tc_parameters.get_param_value("GTEST_OUTPUT_FILE")

        self.__runner = GTestFwkRunner(self._device)

        # Create the result directory if needed
        # Remount the device's filesystem
        self.phone_wrapper.remount()

        self.phone_wrapper.make_result_directory(self._gtest_output_folder)

        # Return the verdict
        return Global.SUCCESS, "No error."

    def get_runner(self):
        """
        Returns the C{GTesterRunner} instance used by this
        Use Case.

        :rtype: GTesterRunner
        :return: the C{GTesterRunner} instance
        """
        return self.__runner

    def get_phone_wrapper(self):
        """
        Returns the C{AndroidDeviceWrapper} instance used by this
        Use Case.

        :rtype: AndroidDeviceWrapper
        :return: the C{AndroidDeviceWrapper} instance
        """
        return self.__phone_wrapper

    def _count_verdict_in_multiple_result(self, verdict, results):
        """
        Count the number of occurrences of the given verdict in
        the given multiple results.

        :param verdict: The verdict to count
        :type verdict: UtilitiesFWK.Utilities.Verdict

        :param results: The results to use
        :type results: dict

        .. seealso:: Report.SecondaryTestReport.SecondaryTestReport

        :return: The count of verdict in results
        :rtype: int
        """
        count = 0
        for el in results:
            if 'status' in results[el] and results[el]['status'] == verdict:
                count += 1

        return count

    def _build_gtest_fwk_command(self, application):
        """
        Build command to be launched for component testing application.

        :param application: The application to use for test
        :type application: str

        :rtype: str
        :return: The full built command line
        """
        if application in [None, ""]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                   "Empty parameter value for 'application' !")

        if self._gtest_output_folder in [None, ""]:
            self._gtest_output_folder = GTestFwkResult.GTEST_FWK_RESULT_ROOT

        if self._gtest_output_file in [None, ""]:
            self._gtest_output_file = GCampaignParser.DEFAULT_GTESTER_RESULT_FILE

        # Format as Linux path
        path = os.path.normpath(self._gtest_output_folder) + '/' + self._gtest_output_file

        command = "adb shell " + application + " --gtest_filter=\"" + \
            self._gtest_filter + "\"" + \
            " --gtest_output=\"xml:\"" + path
        return command

    def _parse_and_compute_result(self, result_object):
        """
        Parse the given result object and compute verdict.

        :param result_object: The result object to parse
        :type result_object: Utilities.GTestFwk.Parser.GTestFwkResult

        :rtype: None
        """
        if result_object is None:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "There is no result object to compute !")

        # Retrieve the remote file path
        remote_file_path = result_object.remote_file_path

        # Compute the local file path
        local_file_path = result_object.local_file_path
        local_file_path = os.path.normpath(local_file_path)

        # Retrieve the result file
        self._logger.debug("Retrieve result file.")
        self.phone_wrapper.pull(remote_file_path, local_file_path)

        # Parse the result file
        self._logger.debug("Parse the file.")
        result_object.parse()

        # Copy local file to report dir
        self._logger.debug("Copy local file.")
        result_object.copy_local_file(self._device.get_report_tree().get_report_path())

        # Delete the local file
        self._logger.debug("Delete local file.")
        result_object.delete_local_file()

    # Declare properties that will help us
    # to write other Component tests Use Case more easily.
    runner = property(
        get_runner,
        None,
        None,
        "The runner (GTesterRunner) instance.")

    phone_wrapper = property(
        get_phone_wrapper,
        None,
        None,
        "The parser (AndroidDeviceWrapper) instance.")
