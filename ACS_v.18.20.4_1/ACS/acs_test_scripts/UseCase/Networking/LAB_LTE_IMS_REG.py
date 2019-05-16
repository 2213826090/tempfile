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
:summary:  Use case to validate LTE IMS registration
:since: 07/01/2014
:author: gescoffr
"""

import time

from LAB_LTE_BASE import LabLteBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LabLteImsReg(LabLteBase):

    """
    Lab LTE registration and mobile originated ping.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_LTE_BASE Init function
        LabLteBase.__init__(self, tc_name, global_config)

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        # Call LAB_LTE_BASE set_up function
        LabLteBase.set_up(self)

        # Set Cell on
        self._ns_cell_4g.set_cell_on(self._mimo)

        return (Global.SUCCESS, "No errors")

#------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        # Call LAB_LTE_BASE run_test function
        LabLteBase.run_test(self)

        # Phone has to see the cell off!
        self._modem_api.check_cdk_no_registration_bfor_timeout(self._registration_timeout)

        # Flight mode deactivation
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._logger.info("Check network registration status is %s on DUT" %
                          (self._wanted_reg_state))

        self._modem_api.check_cdk_state_bfor_timeout(
            self._wanted_reg_state,
            self._registration_timeout)

        # Set APN for LTE and/or IMS depending on protocol IPv4 or IPv6
        self._set_apn_for_lte_and_ims()

        # Enable Data Usage
        time.sleep(self._wait_btwn_cmd)
        self._networking_api.activate_pdp_context()

        # Get RAT from Equipment
        network_type = self._ns_data_4g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        # Check IMS connection state
        status = self._ns_data_4g.check_ims_connection_state("REG", self._registration_timeout)
        if status == False:
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                  "Failed to reach IMS registration")

        # Wait between two commands sending
        time.sleep(self._wait_btwn_cmd)

        # Flight mode activation
        self._networking_api.set_flight_mode("on")
        return (self._error.Code, self._error.Msg)
