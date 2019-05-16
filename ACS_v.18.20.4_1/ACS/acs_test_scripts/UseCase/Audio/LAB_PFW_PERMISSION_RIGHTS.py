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

:organization: INTEL MCG ST
:summary: This file aims to verify the "parameterFrameworkConfiguration.xml" rights access
:since: 03/04/2013
:author: sjamaoui
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
import time
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException


class LabPfwPermissionRights(UseCaseBase):

    """
    Lab Audio PFW class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get UECmdLayer
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")

        self._following_name = self._tc_parameters.get_param_value("FOLLOWING_NAME")
        self._operation_type = self._tc_parameters.get_param_value("OPERATION_TYPE")
        self._file_name = self._tc_parameters.get_param_value("FILE_NAME")

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Initialize the test
        """

        UseCaseBase.set_up(self)
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Start file name operation...")

        result = self._phonesystem_api.get_property_value("ro.build.type")
        if result != "userdebug":
            msg = "you need to use a Userdebug Build, test will not be run"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
        else:
            msg = "the DUT is flashed with a %s Build" % result
            self._logger.info(msg)

        return Global.SUCCESS, "No errors"
#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        UseCaseBase.run_test(self)
        # time.sleep(self._wait_btwn_cmd)

        if self._operation_type == "RIGHTS":
            self._logger.info("Check the rights permissions of the file is in Read only.")
            if not self._phonesystem_api.get_file_permissions(self._file_name, "-rw-r--r--"):
                msg = "The file %s is not read only" % self._file_name
                self._logger.error(msg)
                raise DeviceException(DeviceException.FILE_SYSTEM_ERROR, msg)
            else:
                msg = "the file %s is Read Only" % self._file_name

        elif self._operation_type == "REMOVE":
            self._logger.info("The operation type is: Remove the file")
            # check read only file system
            try:
                self._phonesystem_api.delete(self._file_name, False)
                msg = "the file %s can't be deleted" % self._file_name
                self._logger.info(msg)
            except AcsBaseException as e:
                msg = "File deletion occurs"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        elif self._operation_type == "WRITE":
            self._logger.info("The operation type is: Modify (write and save) the file")
            # check read only file system
            try:
                self._phonesystem_api.write_file(self._file_name, "hello")
                self._logger.info(msg)
            except AcsBaseException as e:
                if str(e).find("no write permission"):
                    # expected behavior
                    msg = "the file %s can't be modified" % self._file_name
                    self._logger.info(msg)
                else:
                    msg = "File modification occurs"
                    self._logger.error(msg)
                    raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        elif self._operation_type == "RENAME":
            self._logger.info("The operation type is: Rename the file")
            try:
                self._phonesystem_api.rename(self._file_name, self._following_name)
                msg = "the file %s can't be renamed" % self._file_name
                self._logger.info(msg)

            except AcsBaseException as e:
                msg = "File rename occurs"
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        else:
            self._logger.info("please verify the operation's type")

        return Global.SUCCESS, msg

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """

        UseCaseBase.tear_down(self)
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Quit file name operation...")

        return Global.SUCCESS, "No errors"
