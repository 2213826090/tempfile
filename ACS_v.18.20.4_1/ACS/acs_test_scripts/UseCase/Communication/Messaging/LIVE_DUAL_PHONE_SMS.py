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
:summary: This file implements Live Dual Phone SMS
:since: 07/04/2011
:author: ssavrimoutou
"""

import time
from acs_test_scripts.Utilities.SmsUtilities import DataCodingScheme, compute_sms_segments
from UtilitiesFWK.Utilities import Global
from Device.DeviceManager import DeviceManager
from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase


class LiveDualPhoneSms(UseCaseBase):

    """
    Live Dual Phone SMS test.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get TC Parameters
        self._service_center_addr = \
            self._tc_parameters.get_param_value("SERVICE_CENTER_ADDRESS")
        self._destination = \
            str(self._tc_parameters.get_param_value("DESTINATION_NUMBER"))
        self._message = self._tc_parameters.get_param_value("MESSAGE_CONTENT")
        self._data_coding_sheme = \
            self._tc_parameters.get_param_value("DATA_CODING_SCHEME")
        self._sms_transfer_timeout = \
            int(self._tc_parameters.get_param_value("SMS_TRANSFER_TIMEOUT"))

        dcs = DataCodingScheme(self._data_coding_sheme)
        dcs.decode()

        # Get number of bits per character setted in DCS
        self._nb_bits_per_char = dcs.compute_character_size()

        character_set = dcs.get_character_set()

        if character_set == "7BITS":
            self._content_type = "CTEX"
        else:
            self._content_type = "CDAT"

        # Phone1 & 2 : Get ue command for Phone
        self._msg_api = self._device.get_uecmd("SmsMessaging")
        self._networking_api = self._device.get_uecmd("Networking")
        self._phone2 = DeviceManager().get_device("PHONE2")
        if self._phone2 is not None:
            self._msg_api2 = self._phone2.get_uecmd("SmsMessaging")
            self._networking_api2 = self._phone2.get_uecmd("Networking")
            if self._destination.upper() == "[PHONE_NUMBER]":
                self._destination = str(self._phone2.get_phone_number())

#------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        UseCaseBase.set_up(self)

        # Check if we have the second phone available
        if self._phone2 is None:
            # We are using this multi UC with only one phone
            return (Global.FAILURE,
                    "Cannot run that use case with only one phone configured.")

        # Boot the other phone (the DUT is already booted)
        DeviceManager().boot_device("PHONE2")

        # Disable flight mode on Phone 1 & 2
        self._networking_api.set_flight_mode("off")
        self._networking_api2.set_flight_mode("off")

        self._msg_api.set_service_center_address(self._service_center_addr)

        return Global.SUCCESS, "No errors"

#-----------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """
        UseCaseBase.run_test(self)

        # Clear old SMS (Non blocking for this test if function isn't
        # implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Clearing all sms...")
        self._msg_api2.delete_all_sms()

        # Calculate how many SMS will be sent to equipment
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Computing sms segments...")
        nb_segments = \
            compute_sms_segments(self._message,
                                 self._nb_bits_per_char)

        # Phone2 : Wait for the sms
        # retrieve instance in order to get sent messages
        time.sleep(self._wait_btwn_cmd)
        # register on intent to receive incoming sms
        self._msg_api2.register_for_sms_reception()

        # Phone1: Check SMS delivery status OK before timeout using
        # SMS_TRANSFER_TIMEOUT value and number of SMS to be received
        # (Non blocking for this test if function isn't implemented on CDK)
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Sending sms from DUT...")
        self._msg_api.send_sms(self._destination, self._message)

        # Phone2: get sms received
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Assembling received sms...")
        sms_received = self._msg_api2.wait_for_incoming_sms(self._sms_transfer_timeout)

        if sms_received.message == self._message:
            result_code = Global.SUCCESS
            result_message = "Sms text received and sms text sent are equal"
            result_message += " (sent:{%s}, received:{%s})." % (str(self._message), str(sms_received.message))
            self._logger.info(result_message)
        else:
            result_code = Global.FAILURE
            result_message = "Sms text received and sms text sent aren't equal"
            result_message += " (sent:{%s}, received:{%s})." % (str(self._message), str(sms_received.message))
            self._logger.error(result_message)

        return result_code, result_message

#------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        UseCaseBase.tear_down(self)

        # Phone2: Switch off the phone
        self._phone2.switch_off()

        return Global.SUCCESS, "No errors"
