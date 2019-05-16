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
:summary: Use Case Live Network Scan
:since: 017/11/2015
:author: nowelchx
"""


from UtilitiesFWK.Utilities import Global
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from ErrorHandling.AcsConfigException import AcsConfigException

class LiveNetworkScan(UseCaseBase):

    """
    Use Case Live Network Scan
    """

    def __init__(self, tc_name, global_config):


        UseCaseBase.__init__(self, tc_name, global_config)
#-------------------------------------------------------------------------
    def set_up(self):
        """
         Initialize the test
        """

        UseCaseBase.set_up(self)
        #To check same ue command is running or not
        self._networking_api = self._device.get_uecmd("Networking")
        #To disable the flight mode
        self._networking_api.set_flight_mode("off")
        return Global.SUCCESS, "No errors"
#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)
        #getting the result from uecmd
        available_networks = self._networking_api.get_available_networks()

        #checking the available_network is present or not
        if available_networks!= None:


            self._logger.info("NETWORK SCAN IS SUCCESSFULL")
            self._logger.info("THE AVAILABLE NETWORKS ARE BELOW")

            for network in iter(available_networks.values()[2].split(',')):
                self._logger.info(network)

            return(Global.SUCCESS,"network scanning  is successful")
        else:
            return(Global.FAILURE,"network scanning is failure")

#---------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """

        UseCaseBase.tear_down(self)

        return Global.SUCCESS, "No errors"
