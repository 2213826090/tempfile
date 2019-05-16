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
:summary: This file implements the Messaging UEcmd for Android ICS device
:since: 05/12/2012
:author: rbertolx
"""
import time
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Communication.MmsMessaging import MmsMessaging as MmsMessagingCommon
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.DeviceException import DeviceException


class MmsMessaging(MmsMessagingCommon):

    """
    :summary: MmsMessaging UEcommands operations for Android ICS platforms
    using an C{Intent} based communication to the I{DUT}.
    """

    @need('modem')
    def __init__(self, phone):
        """
        Constructor.
        """
        MmsMessagingCommon.__init__(self, phone)

    def request_remote_send_received_mms_compare(self, mms_type):
        """
        Compares the sent MMS and the last received MMS.

        :type mms_type: str
        :param mms_type: Describes the type of MMS that will be build.
            Should be either text for a MMS with subject and text or picture
            for a MMS with subject, text and a picture attached.

        """
        # Defining keys for input bundle.
        mms_type_key = "type"
        first_msg_id_key = "first_msg_id"
        second_msg_id_key = "second_msg_id"
        # Logging what the function will do.
        self._logger.info("Comparing the sent and received MMS.")
        # Building the command.
        function = "compareMms"

        self._internal_exec_v2(self._mms_module,
                               function,
                               "--es %s %s --es %s %s --es %s %s"
                               % (mms_type_key,
                                  mms_type,
                                  first_msg_id_key,
                                  self.sent_msg_id,
                                  second_msg_id_key,
                                  self.received_msg_id), is_system=True)
        # Reset msg ids
        self.sent_msg_id = 0
        self.received_msg_id = 0

    def wait_for_incoming_mms(self, timeout, sender_number):
        """
        Waits for incoming MMS, will return successfully if before the timeout
        if the DUT receives an MMS in the thread with id thread_id the number
        of MMS in the inbox reaches the value of the expected_number_of_MMS
        parameter

        :type timeout: int
        :param timeout: time in seconds to wait before considering that the
        MMS will never be received

        :type sender_number: str
        :param sender_number: phone number of the sender

        :rtype: str
        :return: sent_time of the received MMS.

        :raise DeviceException:
            - If the output bundle is missing one of the expected keys.
            - If the MMS is not received before timeout.
        """
        # Defining keys for input and output bundles.
        sender_number_key = "sender_number"
        start_date_key = "start_date"
        thread_id_key = "thread_id"
        message_id_key = "msg_id"
        reception_date_key = "reception_date"
        # Defining function name.
        function = "lookForIncomingMms"

        start_time = time.time()
        is_message_received = False
        self.start_date = self._phonesystem_api.get_phone_time()

        while (time.time() - start_time) < timeout and is_message_received is False:

            # Sending the start command
            output = self._internal_exec_multiple_v2(self._mms_module, "assembleReceivedMms", is_system=True)
            self._logger.debug("The address of received MMS list : %s" % output)

            # Check if we have already received Mms on Embedded side, before parse
            # the MMS database to lookForIncomingMessages
            # At least one MMS is received and sender number is also found in received MMS

            if (len(output) <= 0):
                continue

            for mms in output:
                if sender_number in mms["address"]:
                    # Sending the start command
                    output = self._internal_exec_v2(self._mms_module,
                                                    function,
                                                    "--es %s %s --es %s %s"
                                                    % (sender_number_key,
                                                       sender_number,
                                                       start_date_key,
                                                       self.start_date), is_system=True)

                    if thread_id_key not in output:
                        raise AcsToolException(AcsToolException.INVALID_PARAMETER,
                                               "output key %s not found: %s"
                                               % (thread_id_key, output))
                    else:
                        self.received_thread_id = output.get(thread_id_key)
                    if message_id_key not in output:
                        raise AcsToolException(AcsToolException.INVALID_PARAMETER,
                                               "output key %s not found: %s"
                                               % (message_id_key, output))
                    else:
                        self.received_msg_id = output.get(message_id_key)
                    if reception_date_key not in output:
                        raise AcsToolException(AcsToolException.INVALID_PARAMETER,
                                               "output key %s not found: %s"
                                               % (reception_date_key, output))
                    if (output.get(thread_id_key) != "-1"
                            and output.get(message_id_key) != "-1"
                            and output.get(reception_date_key) != "-1"):
                        is_message_received = True

                else:
                    self._logger.debug("Ignoring MMS data from sender %s: %s" % (str(sender_number), str(output)))

        if is_message_received:
            return output.get(reception_date_key)
        else:
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "Did not received the MMS after waiting %s seconds" % timeout)
