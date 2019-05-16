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
:summary: This file implements the LIVE WIFI CONNECT UC
:since: 07/08/2013
:author: apairex
"""
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException


class LabCwsEthernetCommutationTool(UseCaseBase):
    """
    Lab Wifi Connect Test class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Retrieve UC parameter
        self._ethernet_network_selection = self._tc_parameters.get_param_value("ETHERNET_NETWORK_SELECTION", "")

        self._ethernet_commutator = None

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Check TC parameter validity
        if self._ethernet_network_selection not in ["BENCH", "CORPORATE"]:
            msg = "Wrong value for ETHERNET_NETWORK_SELECTION TC parameter: " + self._ethernet_network_selection
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        # Get the Ethernet Commutator equipment
        self._ethernet_commutator = self._em.get_ethernet_commutator("ETHERNET_COMMUTATOR")

        self._ethernet_commutator.init()

        return Global.SUCCESS, "no_error"

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        if self._ethernet_network_selection == "BENCH":
            self._ethernet_commutator.activate_bench_network()
        else:
            self._ethernet_commutator.activate_corporate_network()

        return Global.SUCCESS, "no_error"
