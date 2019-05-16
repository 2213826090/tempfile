"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: Use Case LTE SMS over IMS
:since: 28/01/2014
:author: gescoffr
"""

import time

import acs_test_scripts.Utilities.SmsUtilities as SmsUtil
from UseCase.Networking.LAB_LTE_BASE import LabLteBase
from UtilitiesFWK.Utilities import Global
from ErrorHandling.DeviceException import DeviceException


class LabLteImsSms(LabLteBase):

    """
    Use Case WCDMA SMS base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call LTE base Init function
        LabLteBase.__init__(self, tc_name, global_config)
        # SMS direction (MO or MT)
        self._sms_direction = self._tc_parameters.get_param_value("SMS_DIRECTION")
        # SMS type (TGPP or TGP2)
        self._sms_type = self._tc_parameters.get_param_value("SMS_TYPE","TGPP")
        # SMS core text
        self._sms_core_text = self._tc_parameters.get_param_value("SMS_TEXT")
        # Timeout to expect SMS to be received or sent
        self._sms_timeout = \
            int(self._tc_parameters.get_param_value("SMS_TIMEOUT","0"))
        #
        self._destination_number = \
            str(self._tc_parameters.get_param_value("DESTINATION_NUMBER"))
        if self._destination_number.upper() == "[PHONE_NUMBER]":
            self._destination_number = str(self._device.get_phone_number())

        # Instantiate Messaging UECmd for SMS UseCases
        self._messaging_api = self._device.get_uecmd("SmsMessaging")

        # Create cellular network simulator and retrieve 3G messaging API
        self._messaging_4g = self._ns.get_cell_4g().get_messaging()


#------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """
        # Call LAB_LTE_BASE set_up function
        LabLteBase.set_up(self)

        # Set Cell on
        self._cell_4g.set_cell_on(self._mimo)

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
        network_type = self._data_4g.get_network_type()

        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(network_type,
                                                          self._registration_timeout)

        # Check IMS connection state
        status = self._data_4g.check_ims_connection_state("REG", self._registration_timeout)
        if status == False:
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE,
                                  "Failed to reach IMS registration")

        # Wait 15 seconds after registration before sending first SMS
        time.sleep(15)

        # Initialize counter to be append to sms_text in order to
        # differenciate the different SMS sent in B2B mode
        self._counter = 0

        return (Global.SUCCESS, "No errors")

#------------------------------------------------------------------------------
    def run_test(self):
        """
        Execute the test
        """
        # pylint: disable=E1101
        # Disable this pylint error due to Enum

        # Call LAB_LTE_BASE run_test function
        LabLteBase.run_test(self)

        # Increment counter
        self._counter += 1
        # Append counter value to SMS core text to create a different SMS for each loop
        self._sms_text = self._sms_core_text+str(self._counter)

        self._sms = SmsUtil.SmsMessage(self._sms_text, self._destination_number, "")

        # Clear old SMS on phone
        self._messaging_api.delete_all_sms()

        if self._sms_direction == "MO":
            # Send SMS to equipment using SMS parameters :
            # - SMS_TEXT
            # - PHONE_NUMBER
            time.sleep(self._wait_btwn_cmd)
            self._messaging_api.send_sms(self._sms.sender, self._sms.message)

            # Retrieve the MO SMS received On Network Simulator
            self._sms_received = self._messaging_4g.retrieve_sms_over_ims(self._sms_timeout)
            # Force sender number because this info can't be retrieved from Network Simulator
            self._sms_received.sender = self._destination_number

        elif self._sms_direction == "MT":
            # register on intent to receive incoming sms
            self._messaging_api.register_for_sms_reception()

            # Configure the type of the message to send CUSTOM TEXT.
            self._messaging_4g.set_sms_over_ims_type(self._sms_type)
            self._messaging_4g.set_custom_sms_over_ims_text(self._sms_text)

            # Send SMS to CDK using SMS parameters :
            # - SMS_TEXT
            self._messaging_4g.send_sms_over_ims()

            # Check sms acknowledged by network simulator
            if not self._messaging_4g.check_sms_over_ims_delivery(self._sms_timeout):
                msg = "SMS over IMS not acknowledged by NW"
                self._logger.info(msg)
                # Return message and exit test case
                return Global.FAILURE, msg

            # Retrieve the MT SMS received On DUT
            self._sms_received = self._messaging_api.wait_for_incoming_sms(self._sms_timeout)

        else:
            # Log that SMS_DIRECTION is not declared in Xml
            msg = "Test Not available. Please make sure to specify SMS DIRECTION MO/MT "
            self._logger.info(msg)
            # Return message and quit the method
            return Global.FAILURE, msg

        # Compare sent and received SMS (Text, Phone number)
        (result_verdict, result_message) = \
            SmsUtil.compute_sms_equals(self._sms, self._sms_received)
        # Print SMS test status
        self._logger.info(result_message)

        # Perform 10 MO ping of 32 byte if SMS is successful
        if result_verdict == Global.SUCCESS:
            # Compute packet loss value
            packet_loss = self._networking_api.ping(self._server_ip_address, 32, 10)
            # Compute verdict depending on % of packet loss
            if packet_loss.value > 0:
                self.result_verdict = Global.FAILURE
            else:
                self.result_verdict = Global.SUCCESS

            self.result_message = "Measured Packet Loss: %.0f%s (Target: %.0f%s)" \
                              % (packet_loss.value,
                                 packet_loss.units,
                                 0, # no packet loss expected
                                 packet_loss.units)

        return result_verdict, result_message
