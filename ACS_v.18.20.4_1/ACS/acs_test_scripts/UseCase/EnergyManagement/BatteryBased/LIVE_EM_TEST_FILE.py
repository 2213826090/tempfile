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
:summary: Energy Management - test sysfs operation like edit, delete...
:author: vgombert
:since: 14/11/2013
"""
import os

from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from Core.Report.SecondaryTestReport import SecondaryTestReport
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from UtilitiesFWK.Utilities import Global
import time


class LiveEmTestFile(EmUsecaseBase):

    """

    """
    DEDICATED_BENCH = "BATTERY_BENCH"
    __POSSIBLE_ACTION = {"TRY_EDIT": ["SUCCEED", "FAILED"],
                         "TRY_DELETE": ["SUCCEED", "FAILED"],
                         "TRY_REPLACE": ["SUCCEED", "FAILED"],
                         "CHECK_PERMISSION": ["READ_ONLY", "WRITE_ONLY", "READ_WRITE", "NO_READ_WRITE"]}

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)
        #-----------------------------------------------------------------------
        # create a list of file to test
        file_to_test_list = self._tc_parameters.get_param_value("FILE")
        file_to_test_list = file_to_test_list.split(';')
        self.__file_to_test_list = map(str.strip, file_to_test_list)

        # get the list of expected result
        result_list = self._tc_parameters.get_param_value("EXPECTED_RESULT")
        result_list = result_list.split(';')
        self.__expected_result_list = map(str.strip, result_list)

        self.__action_to_perform = str(self._tc_parameters.get_param_value("ACTION")).upper()
        self.__user_privilege = self._tc_parameters.get_param_value("USER_PRIVILEGE")

        self.__file_api = self._device.get_uecmd("File")

        # all this are related to fuel gauging file
        self.__file_secure_copy_list = []
        self.__file_original_permission_list = []
        self.__dangerous_action_performed = False
        # Instantiate the Secondary Report object
        self.__secondary_report = SecondaryTestReport(self._device.get_report_tree().get_report_path())

#-----------------------------------------------------------------------

    def set_up(self):
        """
        create secure copy of file we are going to modify
        """
        # Call the UseCaseBase Setup function
        EmUsecaseBase.set_up(self)

        # Check action first
        if self.__action_to_perform not in self.__POSSIBLE_ACTION.keys():
            txt = "wrong action to perform, can only be %s and not %s" % (self.__POSSIBLE_ACTION.keys(),
                                                                          self.__action_to_perform)
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # check that there is a the equal number of file and expected result
        if len(self.__file_to_test_list) != len(self.__expected_result_list):
            txt = "the number of FILE and EXPECTED_RESULT is different , please correct this on the testcase.xml."
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        txt = ""
        # Check expected result first
        for result in self.__expected_result_list:
            if result not in self.__POSSIBLE_ACTION[self.__action_to_perform]:
                txt = "wrong expected result for action %s, can only be %s and not %s" % (self.__action_to_perform,
                                                                                          str(self.__POSSIBLE_ACTION[self.__action_to_perform]),
                                                                                          result)
        if txt != "":
            self._logger.error(txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, txt)

        # check that all files exist
        txt = ""
        for a_file in self.__file_to_test_list:
            if not self.__file_api.exist(a_file)[0]:
                txt += " %s does not exist!\n" % a_file
        if txt != "":
            self._logger.error(txt)
            raise DeviceException(DeviceException.FILE_SYSTEM_ERROR, txt)

        # do a secure copy only if we are going to play
        if  self.__action_to_perform in ["TRY_DELETE", "TRY_REPLACE", "TRY_EDIT"]:
            for a_file in self.__file_to_test_list:
                file_secure_copy = os.path.join(self._device.get_sdcard_path() + "/" + os.path.basename(a_file))
                if not self.__file_api.exist(file_secure_copy)[0]:
                    self.__file_api.remount()
                    self._logger.info("making a secure copy of file at : %s" % self.__file_secure_copy_list)
                    self.__file_api.copy(a_file, file_secure_copy)
                else:
                    self._logger.info("a secure copy of file %s has been found at : %s" % (file, file_secure_copy))
                self.__file_secure_copy_list.append(file_secure_copy)
                self.__file_original_permission_list.append(self.__file_api.get_file_permissions(a_file, self.__user_privilege)[1])

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------

    def run_test_body(self):
        """
        try to modify FG file and restore it each time a modification works.
        """
        EmUsecaseBase.run_test_body(self)
        result_msg = ""
        # change user privilege first, it may cause connection lost
        self.__file_api.change_user_privilege(self.__user_privilege)
        # wait for a while for getting board connection
        self._logger.info("wait 30s to let the user privilege change be applied")
        time.sleep(30)
        wait_time = 150
        end_time = time.time() + wait_time
        self._logger.info("wait at most %ss to see board connection" % wait_time)
        while time.time() < end_time:
            if self._device.get_state() == "alive":
                break

        if self._device.get_state() != "alive":
            txt = " board connection lost after changing user privilege and waiting %ss" % wait_time
            self._logger.error(txt)
            raise DeviceException(DeviceException.OPERATION_FAILED, txt)

        verdict = Global.SUCCESS
        for index in range(len(self.__file_to_test_list)):
            # load targets in order to measure iteration
            a_file = self.__file_to_test_list[index]
            result = self.__secondary_report.verdict.BLOCKED
            raw_result = None
            output = "test not reached"
            txt = ""

            if self.__action_to_perform == "TRY_DELETE":
                self.__dangerous_action_performed = True
                raw_result, output = self.__file_api.delete(a_file)
                if raw_result:
                    raw_result = "SUCCEED"
                else:
                    raw_result = "FAILED"
                txt = "file %s to be deleted" % raw_result

            elif self.__action_to_perform == "TRY_EDIT":
                self.__dangerous_action_performed = True
                raw_result, output = self.__file_api.write_file(a_file, "modified_by_acs")
                if raw_result:
                    raw_result = "SUCCEED"
                else:
                    raw_result = "FAILED"
                txt = "file %s to be edited" % raw_result

            elif self.__action_to_perform == "TRY_REPLACE":
                self.__dangerous_action_performed = True
                raw_result, output = self.__file_api.copy(self.__file_secure_copy_list[index], a_file)
                if raw_result:
                    raw_result = "SUCCEED"
                else:
                    raw_result = "FAILED"
                txt = "file %s to be replaced" % raw_result

            elif self.__action_to_perform == "CHECK_PERMISSION":
                raw_result, output = self.__file_api.get_file_permissions(a_file, self.__user_privilege)

            output = output.strip()
            if output == "":
                output = txt

            if txt != "":
                self._logger.info(txt)

            # init text
            local_result = "test of file %s is" % a_file

            if raw_result is not None:
                # now check if the result match with expected result
                if raw_result != self.__expected_result_list[index]:
                    result = self.__secondary_report.verdict.FAIL
                    verdict = Global.FAILURE
                else:
                    result = self.__secondary_report.verdict.PASS

                local_result += " %s, reason: read %s, expected %s ;\n" % (result, raw_result,
                                                                    self.__expected_result_list[index])
            # block case
            else:
                local_result = "%s as not test was done\n" % result
            result_msg += local_result
            self.__secondary_report.add_result(a_file, result, output,
                                               self.get_name(), self.tc_order)

        if result_msg == "":
            result_msg = "no test run"
            verdict = Global.BLOCKED

        return verdict, result_msg.rstrip("\n")

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        # call tear down after some operations
        EmUsecaseBase.tear_down(self)
        # get just the aplogs
        if self._device.get_state() == "alive":
            # restart acs connection to have clean rooted device
            self._device.disconnect_board()
            self._device.connect_board()
            self.get_application_logs()

        if len(self.__file_secure_copy_list) > 0:
            self._logger.info("Restoring and deleting generated file")
            # parse all secure copy
            for index in range(len(self.__file_secure_copy_list)):
                secure_copy_file = self.__file_secure_copy_list[index]

                # if a copy exist, compare it if original
                if self.__file_api.exist(secure_copy_file)[0]:
                    original_file = self.__file_to_test_list[index]

                    self.__file_api.remount()
                    if (not self.__file_api.exist(original_file)[0] or not self.__file_api.is_equal(original_file, secure_copy_file)) and self.__dangerous_action_performed:
                        self._logger.info("restoring file")
                        self.__restore_file(index)
                    # delete it the end
                    self.__file_api.delete(secure_copy_file)
            self.__dangerous_action_performed = False

        return Global.SUCCESS, "No errors"

    def __restore_file(self, index):
        self.__file_api.copy(self.__file_secure_copy_list[index], self.__file_to_test_list[index])
        if len(self.__file_original_permission_list) > 0:
            if  self.__file_original_permission_list[index] != self.__file_api.get_file_permissions(self.__file_to_test_list[index], self.__user_privilege)[1]:
                self.__file_api.set_file_permission(self.__file_to_test_list[index],
                                                    self.__file_original_permission_list[index])
