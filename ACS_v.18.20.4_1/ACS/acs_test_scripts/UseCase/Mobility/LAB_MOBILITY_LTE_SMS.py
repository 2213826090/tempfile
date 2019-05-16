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
:summary: Test The Circuit Switched Fallback from LTE to UTRAN
:since: 15/04/2013
:author: lvacheyx
"""
import time
from LAB_MOBILITY_LTE_3GSM_BASE import LabMobilityLte3gsmBase
from acs_test_scripts.Utilities.SmsUtilities import SmsMessage, DataCodingScheme
from UtilitiesFWK.Utilities import Global


class LabMobilityLteSms(LabMobilityLte3gsmBase):

    """
    Usecase base for mobility LTE handover use cases
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LabMobilityLte3gsmBase.__init__(self, tc_name, global_config)

        # Read PHONE_NUMBER from testcase xml parameters
        self._phone_number = \
            str(self._tc_parameters.get_param_value("PHONE_NUMBER"))
        if self._phone_number.upper() == "[PHONE_NUMBER]":
            self._phone_number = str(self._device.get_phone_number())

        # Read SMS_TEXT from testcase xml file
        self._sms_text = self._tc_parameters.get_param_value("SMS_TEXT")

        # Read SMS_DIRECTION from testcase xml file
        self._sms_direction = str(self._tc_parameters.get_param_value("SMS_DIRECTION"))

        # Read SMS_TRANSFER_TIMOUT from testcase xml file
        self._sms_transfer_timeout = \
            int(self._tc_parameters.get_param_value("SMS_TRANSFER_TIMEOUT"))

        # Read DATA_CODING_SCHEME from xml UseCase parameter file
        self._data_coding_sheme = \
            self._tc_parameters.get_param_value("DATA_CODING_SCHEME")

        dcs = DataCodingScheme(self._data_coding_sheme)
        dcs.decode()

        # Get number of bits per character setted in DCS
        self._nb_bits_per_char = dcs.compute_character_size()

        character_set = dcs.get_character_set()

        if character_set == "7BITS":
            self._content_type = "CTEX"
        else:
            self._content_type = "CDAT"

        # Instantiate Messaging UECmd for SMS UseCases
        self._messaging_api = self._device.get_uecmd("SmsMessaging")

        self._sms = SmsMessage(self._sms_text,
                               self._phone_number,
                               "LCSD",
                               self._ns_3gsm_messaging,
                               self._messaging_api,
                               self._data_coding_sheme,
                               self._nb_bits_per_char,
                               self._sms_transfer_timeout,
                               self._content_type,
                               self._sms_direction)

# ------------------------------------------------------------------------------

    def set_up(self):
        """
        Set up the test configuration
        """

        # Call LabMobilityLte3gsmBase set_up function
        LabMobilityLte3gsmBase.set_up(self)

        # Set LTE cell on
        self._ns_lte_cell.set_cell_on(self._ns_lte_mimo)

        # MO or MT SMS instance
        self._sms.configure_sms()

        # Disable flight mode
        self._networking_api.set_flight_mode("off")

        # Check registration state is connected using
        # registrationTimeout from Device_Catalog.xml
        self._modem_api.check_cdk_registration_bfor_timeout(self._registration_timeout)

        # Set the APN
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Setting APN " + str(self._apn) + "...")
        self._networking_api.set_apn(self._ssid, self._apn)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid, check=False)

        # Check Data Connection State => CON before timeout
        self._ns_lte_data.check_data_connection_state("CON",
                                                      self._registration_timeout,
                                                      blocking=False,
                                                      cell_id=self._ns_lte_cell_id)

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(self._ns_lte_data.get_network_type(),
                                                          self._registration_timeout)

        # Set Network Simulator 3GSM cell on
        self._ns_3gsm_cell.set_cell_on()

        #  Set External EPC connection between Network Simulators
        self._ns_3gsm_cell.set_external_epc_connection(self._ns_lte_ip_lan1,
                                                       self._ns_lte_dl_earfcn)

        return Global.SUCCESS, "No errors"
# ------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        # Call LabMobilityLte3gsmBase run_test function
        LabMobilityLte3gsmBase.run_test(self)

        # Clear old SMS on phone
        self._messaging_api.delete_all_sms()

        # Clear old SMS on Network Simulator
        self._ns_3gsm_messaging.clear_message_data()

        self._sms.send_sms()

        self._sms.get_sms()

        return Global.SUCCESS, "No errors"
