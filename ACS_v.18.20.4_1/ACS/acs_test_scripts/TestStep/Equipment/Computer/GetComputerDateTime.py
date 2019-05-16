"""

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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

@organization: UMG PSI Validation
@summary: This file implements the step to get current date and time of a given computer.

@author: emarchan

"""

from acs_test_scripts.TestStep.Equipment.Computer.ComputerBase import ComputerBase


class GetComputerDateTime(ComputerBase):
    """
    Gets current date and time of a given computer.
    """

    def run(self, context):
        """
        Run the test step
        """
        ComputerBase.run(self, context)
        result_parameters = self._get_current_date_and_time()

        context.set_nested_info([self._pars.computer_current_date_time, "DAY"], result_parameters["DAY"])
        context.set_nested_info([self._pars.computer_current_date_time, "MONTH"], result_parameters["MONTH"])
        context.set_nested_info([self._pars.computer_current_date_time, "YEAR"], result_parameters["YEAR"])
        context.set_nested_info([self._pars.computer_current_date_time, "HOURS"], result_parameters["HOURS"])
        context.set_nested_info([self._pars.computer_current_date_time, "MINUTES"], result_parameters["MINUTES"])
        context.set_nested_info([self._pars.computer_current_date_time, "SECONDS"], result_parameters["SECONDS"])

        self._ts_verdict_msg += "\nVERDICT: %s stored as {0}:DAY".format(
            str(self._pars.computer_current_date_time)) % result_parameters["DAY"]
        self._ts_verdict_msg += "\nVERDICT: %s stored as {0}:MONTH".format(
            str(self._pars.computer_current_date_time)) % result_parameters["MONTH"]
        self._ts_verdict_msg += "\nVERDICT: %s stored as {0}:YEAR".format(
            str(self._pars.computer_current_date_time)) % result_parameters["YEAR"]
        self._ts_verdict_msg += "\nVERDICT: %s stored as {0}:HOURS".format(
            str(self._pars.computer_current_date_time)) % result_parameters["HOURS"]
        self._ts_verdict_msg += "\nVERDICT: %s stored as {0}:MINUTES".format(
            str(self._pars.computer_current_date_time)) % result_parameters["MINUTES"]
        self._ts_verdict_msg += "\nVERDICT: %s stored as {0}:SECONDS".format(
            str(self._pars.computer_current_date_time)) % result_parameters["SECONDS"]
        self._logger.debug(self._ts_verdict_msg)

    def _get_current_date_and_time(self):
        return self._computer.get_date_time()