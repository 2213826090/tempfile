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

:summary: Use Case EGPRS Base for SMOKE and BAT tests.
:organization: INTEL MCG PSI
:author: ccontreras
:since: 02/09/2010
"""

import time
import acs_test_scripts.Utilities.RegistrationUtilities as RegUtil

from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from acs_test_scripts.Utilities.CommunicationUtilities import TelephonyConfigsParser, throughput_targets_string, \
    ConfigsParser


class TurnOffCell(UseCaseBase):

    """
    Lab EGRPS Data Transfer base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """


        UseCaseBase.__init__(self, tc_name, global_config)

        # Read registrationTimeout from DeviceCatalog.xml


        # Read CELLULAR_NETWORK parameters from BenchConfig.xml
        self._network = \
            global_config.benchConfig.get_parameters("CELLULAR_NETWORK")


        # Id of the NETWORK_SIMULATOR
        self._ns_number = 1

        # Read NETWORK_SIMULATOR1 from BenchConfig.xml
        self._ns_node = \
            global_config.benchConfig.get_parameters("NETWORK_SIMULATOR1")


        # Cell Tech is 2G
        self.ns_cell_tech = "2G"

        # Create cellular network simulator and retrieve 2G data interface
        self._ns = self._em.get_cellular_network_simulator("NETWORK_SIMULATOR1")
        self._ns_cell_2g = self._ns.get_cell_2g()
        self._ns_data_2g = self._ns_cell_2g.get_data()


#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Connect to equipment
        self._ns.init()

        # Set the equipment application format GSM/GPRS
        self._ns.switch_app_format("GSM/GPRS")

        # Perform a full preset
        self._ns.perform_full_preset()

        # Set cell off
        self._ns_cell_2g.set_cell_off()

        # Disconnect Equipment
        self._ns.release()

        return Global.SUCCESS, "No errors"


