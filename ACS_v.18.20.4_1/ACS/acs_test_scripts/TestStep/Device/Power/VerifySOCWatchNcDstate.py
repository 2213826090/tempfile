"""
@summary: Verify NC D state residency from the result which was created by SOCwatch.
PREREQUISITES:
socwatch result should be available on the host

@since 20 April 2015
@author: Jongyoon Choi
@organization: INTEL PEG-SVE-DSV

@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
import re
import os.path

class VerifySOCWatchNcDstate(DeviceTestStepBase):
    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        DeviceTestStepBase.run(self, context)
        self._logger.info(self._pars.id + ": Run")

        self.residency_api = self._device.get_uecmd("Residencies")

        result_file_fullpath = self._pars.csv_stored_path + "/" + self._pars.csv_file_name + ".csv"

        target_block = self._pars.target_block
        target_criteria = self._pars.target_criteria.split(";")
        converted_target_criteria = self.residency_api.convert_criteria_list(target_criteria)
        if not converted_target_criteria:
            msg = "run_socwatch: TARGET_CRITERIA contains invalid value"
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        target_value = self._pars.target_value.split(";")

        if not os.path.isfile(result_file_fullpath):
            msg = "run_socwatch: Could not find a SOCWatch result file {0}".format(result_file_fullpath)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        states_title, states_info = self.residency_api.parse_socwatch_nc_dstates_info(result_file_fullpath, target_block)

        if not len(states_title) is len(converted_target_criteria):
            msg = "run_socwatch: Length of TARGET_CRITERIA is different from expected value {0}".format(len(states_info))
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        if not len(states_title) is len(target_value):
            msg = "run_socwatch: Length of TARGET_VALUE is different from expected value {0}".format(len(states_info))
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        self._logger.info("NC D state residency information")
        self._logger.info("================================")
        for i in xrange(len(states_title)):
            self._logger.info("{0} state: {1}%".format(states_title[i], states_info[i]))

        for index, title in enumerate(states_title):
            ret_val = eval("{0} {1} {2}".format(states_info[index], converted_target_criteria[index], target_value[index]))
            if ret_val:
                self._logger.info("Success: Reported {1} residency of {0} is {2}%. It is {3} {4} ".format(target_block, title, states_info[index], converted_target_criteria[index], target_value[index]))
            else:
                msg = "Reported {1} residency of {0} is {2}%. Different from expectation, it is not {3} {4}".format(target_block, title, states_info[index], converted_target_criteria[index], target_value[index])
                self._logger.error(msg)
                raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        self._logger.info(self._pars.id + ": Done")
