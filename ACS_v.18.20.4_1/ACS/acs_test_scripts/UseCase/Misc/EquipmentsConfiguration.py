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
:summary: This file implements an equipments configuration
UC for testing ACS framework
:since: 04/08/2011
:author: dgonzalez
"""

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase


class EquipmentsConfiguration(UseCaseBase):

    """
    Usecase that configure equipment(s) thanks to
    an xml configuration file.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._conf_filename = \
            self._tc_parameters.get_param_value("CONFIGURATION_FILE")

    def run_test(self):
        """
        Execute the test
        """

        UseCaseBase.run_test(self)

        self._em.configure_equipments(self._conf_filename)

        return Global.SUCCESS, "No Errors"
