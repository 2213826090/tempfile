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
:summary: This file implements TDSCDMA Camp and Reg
:author: gcharlem
:since:17/11/2014
"""

import time
from UtilitiesFWK.Utilities import Global, str_to_bool
from acs_test_scripts.UseCase.Networking.LAB_TDSCDMA_BASE import LabTdscdmaBase
from acs_test_scripts.Utilities.PhoneOnOffUtilities import PhoneOnOff


class LabTdscdmaCamp(LabTdscdmaBase):

    """
    Lab TDSCDMA ftp test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LAB_TDSCDMA_BASE Init function
        LabTdscdmaBase.__init__(self, tc_name, global_config)

        # Read mode from test case xml file (str)
        self._switch_mode = self._tc_parameters.get_param_value("SWITCH_MODE", "softshutdown")

        # Read PDP Activation from test case xml file
        self._pdp_activation = str_to_bool(self._tc_parameters.get_param_value("PDP_ACTIVATION",
                                                                               "false"))
        # Instantiate Phone On/OFF utilities
        self.phoneonoff_util = PhoneOnOff(self._networking_api, self._device, self._logger)

# ------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        if self._switch_mode == "airplane":
            # Switch on according to the mode chosen
            self.phoneonoff_util.switch_on(self._switch_mode)
        elif self._switch_mode in ("hardshutdown", "softshutdown"):
            # Reboot according to the mode chosen
            self.phoneonoff_util.reboot(self._switch_mode)
            self._networking_api.set_flight_mode("off")
        else:
            self._logger.info("No actions required to do a switch On/Off")

        init_time = time.time()

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml (Non blocking
        # for this test if function isn't implemented on CDK)
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # time needed to reach registered state
        reg_time = time.time() - init_time
        self._logger.info("DUT registered in less than %3.2f seconds!", reg_time)

        if self._pdp_activation:
            # Set the APN
            time.sleep(self._wait_btwn_cmd)
            self._logger.info("Setting APN " + str(self._apn) + "...")
            self._networking_api.set_apn(self._ssid, self._apn)

            # Activate PDP context
            time.sleep(self._wait_btwn_cmd)
            self._logger.info("Active PDP Context...")
            self._networking_api.activate_pdp_context(self._ssid, False)

            # Force screen on to avoid end of PDP context due to fast dormancy
            self._phone_system.wake_screen()
            self._phone_system.set_phone_screen_lock_on(1)
            # Check Data Connection State => PDP Active before timeout
            self._ns_data_3g.check_data_connection_state("PDP_ACTIVE",
                                                         self._registration_timeout,
                                                         blocking=False)

            pdp_active_time = time.time() - init_time
            self._logger.info("DUT pdp activated in less than %3.2f seconds!", pdp_active_time)
        else:
            # Check Data Connection State => ATTACHED before timeout
            self._ns_data_3g.check_data_connection_state("ATTACHED",
                                                         self._registration_timeout,
                                                         False)

            attached_time = time.time() - init_time
            self._logger.info("DUT attached in less than %3.2f seconds!", attached_time)

        if self._switch_mode == "airplane":
            # Switch off according to the switch mode chosen
            self.phoneonoff_util.switch_off(self._switch_mode)
            # Check that DUT is no longer camped on Network
            self._modem_api.check_cdk_no_registration_bfor_timeout(self._registration_timeout)

        return Global.SUCCESS, "No error"
