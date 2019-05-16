"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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
:summary: EM - this UseCase test all Uecmd used by all EM/PUPDR/THERMAL UseCase
:author: vgombert
:since: 15/07/2015
"""
from acs_test_scripts.UseCase.EnergyManagement.EM_USECASE_BASE import EmUsecaseBase
from acs_test_scripts.UseCase.EnergyManagement.UcModule.LoadModule import LoadModule
from UtilitiesFWK.Utilities import Global


class LabEmInitBoard(EmUsecaseBase):

    """
    Lab Energy Management class.
    """
    # This is the default bench type where this test can be run
    DEDICATED_BENCH = "BATTERY_BENCH"

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call the LabEmBaseCommon Init function
        EmUsecaseBase.__init__(self, tc_name, global_config)  # pylint: disable=W0233
        load_off = self._tc_parameters.get_param_value("LOAD_OFF", "")
        self.__load_module = LoadModule()
        self.__load_module.add_load(load_off)

    def set_up(self):
        """
        Initialize the test
        """
        EmUsecaseBase.set_up(self)
        return Global.SUCCESS, "No errors"

    def run_test_body(self):
        """
        Execute the test
        """
        EmUsecaseBase.run_test_body(self)

        # compare values with targets
        self.__load_module.stop_load()
        self.em_api.clean_autolog()
        self.em_api.clean_daemon_files()

        return Global.SUCCESS, "No errors"

    def tear_down(self):
        """
        End and dispose the test
        """
        EmUsecaseBase.tear_down(self)
        return Global.SUCCESS, "No errors"

