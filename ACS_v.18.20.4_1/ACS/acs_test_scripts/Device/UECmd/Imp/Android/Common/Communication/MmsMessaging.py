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
:summary: This file implements the Messaging UEcmd for Android device
:since: 01/04/2011
:author: asebbane
"""
from acs_test_scripts.Device.UECmd.Interface.Communication.IMmsMessaging import IMmsMessaging
from acs_test_scripts.Device.UECmd.UECmdDecorator import need
from acs_test_scripts.Device.UECmd.UECmdTypes import MSG_DB_PATH
import time
import subprocess
import UtilitiesFWK.Utilities as Util
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsToolException import AcsToolException


class MmsMessaging(BaseV2, IMmsMessaging):
    """
    :summary: Messaging UEcommands operations for Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """
    @need('modem')
    def __init__(self, device):
        """
        Constructor.

        """
        BaseV2.__init__(self, device)
        IMmsMessaging.__init__(self, device)
        self._logger = device.get_logger()
        self._mms_module = "acscmd.telephony.messaging.MmsModule"
        #Instanting Phone System API
        self._phonesystem_api = self._device.get_uecmd("PhoneSystem")

        self.phone_time = 0
        self.time_to_send_mms = 0
        self.start_date = 0
        self.sent_thread_id = 0
        self.sent_msg_id = 0
        self.received_thread_id = 0
        self.received_msg_id = 0

    def register_for_mms_reception(self):
        """
        Register on android.provider.Telephony.WAP_PUSH_RECEIVED intent to wait for incoming MMS
        """
        self._logger.info("register on android.provider.Telephony.WAP_PUSH_RECEIVED \
                           intent for MMS Reception")
        self._internal_exec_v2(self._mms_module, "registerForMmsReception", is_system=True)

    def build_mms(self, mms_type, address, subject, mms_text, content_path, repeat_count=0):
        """
        Construct a MMS. Using the parameters passed by the TestCase.

        :type mms_type: str
        :param mms_type: Describing the type of MMS that will be build.
        Should be either text for a MMS with subject and text or picture for a
        MMS with subject, text and a picture attached.

        :type address: str
        :param address: Contains the destination phone number.

        :type subject: str
        :param subject: Contains the text used as subject for the build MMS.

        :type mms_text: str
        :param mms_text: Contains the text used as the main message text for the
        build MMS.

        :type content_path: str
        :param content_path: Contains the path of the image that will be attached
        to the build MMS. Can be null if the mms_type is "text".

        :type repeat_count: int
        :param repeat_count: [optional] the number of times we want to repeat the message
        content in the MMS body.
        """
        self._logger.info("Building Mms")

        function = "buildMms"

        self._internal_exec_v2(self._mms_module, function,
                               "--es type %s --es address %s --es mms_subject \"%s\" "
                               "--es mms_text \"%s\" --es file_path \"%s\""
                               % (mms_type, address, subject, mms_text, content_path),
                               is_system=True)

    def delete_all_messages(self):
        """
        Deletes all the SMS and MMS present on the phone.
        """

        self._logger.info("Deleting all SMS and MMS.")

        function = "deleteAllMmsAndSms"

        self._internal_exec_v2(self._mms_module, function, is_system=True)

    def kill_mms_app(self):
        """
        Kills the MMS application.
        """
        self._logger.info("Killing the MMS application")
        cmd = "adb shell am force-stop com.android.mms"
        self._exec(cmd)
        # Give 2 seconds to the App to exit properly
        time.sleep(2)
        # Check the application was killed and raise an exception if it is not the case
        if self.__is_stock_msg_app_running():
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "The stock messaging application should not be running.")
        # else log that stock messaging application is stopped
        else:
            self._logger.info("The stock messaging application is not running...")

    def send_mms(self, mms_type, mms_address, mms_subject, mms_text, mms_content_path, repeat_count=0):
        """
        Send MO MMS

        :type mms_type: str
        :param mms_type: Describing the type of MMS that will be build.
        Should be either text for a MMS with subject and text or picture for a
        MMS with subject, text and a picture attached.

        :type mms_address: str
        :param mms_address: Contains the destination phone number.

        :type mms_subject: str
        :param mms_subject: Contains the text used as subject for the build MMS.

        :type mms_text: str
        :param mms_text: Contains the text used as the main message text for the
        build MMS.

        :type mms_content_path: str
        :param mms_content_path: Contains the path of the image that will be attached
        to the build MMS. Can be null if the mms_type is "text".

        :type repeat_count: int
        :param repeat_count: [optional] the number of times we want to repeat the message
        content in the MMS body.
        """
        # Wake up the phone screen.
        self._logger.info("Turning the screen on.")
        self._phonesystem_api.wake_screen()
        self._phonesystem_api.set_phone_lock("off")

        # Build the MMS on the DUT
        time.sleep(2)
        self.build_mms(mms_type, mms_address, mms_subject, mms_text, mms_content_path, repeat_count)

        # Wait for MMS to be present in the MMS app
        time.sleep(6)

        self.phone_time = self._phonesystem_api.get_phone_time()

        # Send the MMS.
        if not self._phonesystem_api.get_screen_status():
            self._logger.warning("The screen should not be off when trying"
                                  " to send the MMS.")
        # Send the MO MMS
        self.acknowledge_mms_sending()

        # Wait for MMS to be sent
        time.sleep(3)

    def check_mms_sent(self, mms_address, timeout):
        """
        Check sending of MMS

        :type mms_address: str
        :param mms_address: Contains the destination phone number.

        :rtype: str
        :return: the time when the message has been sent.
        """
        # Checking id of send command to know if previous command was successful.
        is_sent = self.__was_mms_send_cmd_successfull(mms_address, self.phone_time)
        # Treating the result of the previous command.
        # Check if a thread has been created after MMS sending
        if is_sent:
            self._logger.info("Found sending message: Id=%s ThreadId=%s"
                              % (self.sent_msg_id, self.sent_thread_id))
            # Verify that a thread_id and a msg_id have been generated
            if self.sent_thread_id and self.sent_msg_id:
                self._logger.info("The send command was successful.")
                # Quit MMS application
                self.__quit_mms_app()
            # If there are no thread_id and msg_id for the MMS sending,
            # raise an exception
            else:
                # Quit MMS application
                self.__quit_mms_app()
                raise DeviceException(DeviceException.OPERATION_FAILED,
                                      "Invalid thread id: %s or message id: %s"
                                      % (self.sent_msg_id, self.sent_thread_id))
        # If MMS was not sent, raise an exception
        else:
            # Quit MMS application
            self.__quit_mms_app()
            raise DeviceException(DeviceException.OPERATION_FAILED, "The send command was not successful.")
        sent_time = self.__is_mms_sent(timeout)
        self.time_to_send_mms = float(sent_time) - float(self.phone_time)
        self._logger.info("The MMS has been sent in %s seconds." % self.time_to_send_mms)
        # Waiting for the MMS to be sent.
        return sent_time

    def acknowledge_mms_sending(self):
        """
        Sends an MMS previously constructed by the build_mms function
        """
        cmd = "adb shell input keyevent 66"
        self._exec(cmd)
        self._logger.info("Sending of built MMS acknowledged.")

    def wait_for_mms_notification(self, mms_timeout):
        """
        Wait for MMS notification sent from the server to the DUT

        :type mms_timeout: int
        :param mms_timeout: MMS reception timeout
        """
        # Wait for MMS notification (MMS pdu count has change)
        self._logger.info("Waiting for the MMS notification...")
        self._precount = self.__get_pdu_precount()
        count = self._precount
        timeout = 0
        while count <= self._precount:
            # Force screen to on
            self._phonesystem_api.display_on()
            try:
                count = self.__get_mms_pdu_count()
            except Exception as error:
                self._logger.warning("SMS/MMS database not accessible (%s)." % str(error))
            time.sleep(1)
            timeout += 1
            if timeout >= mms_timeout:
                self._logger.error("timeout of %s seconds has been reached for MMS notification" % timeout)
                raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                      "Receive MMS notification Timeout has been reached")
        self._logger.info("MMS notification has been successfully received")

    def wait_for_mms_download(self, mms_timeout):
        """
        Wait for download the MMS content sent from the server to the DUT

        :type mms_timeout: int
        :param mms_timeout: MMS reception timeout
        """
        # Wait for MMS download to finish (MMS part count has change)
        self._logger.info("Waiting for the MMS Download...")
        self._precount = self.__get_part_precount()
        count = self._precount
        timeout = 0
        while count <= self._precount:
            # Force screen to on
            self._phonesystem_api.display_on()
            try:
                count = self.__get_mms_part_count()
            except Exception as error:
                self._logger.warning("SMS/MMS database not accessible (%s)." % str(error))
            time.sleep(1)
            timeout += 1
            if timeout == mms_timeout:
                self._logger.info("timeout of %s seconds has been reached for MMS download" % timeout)
                raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                      "MMS Download Timeout has been reached")
        self._logger.info("MMS has been successfully received (Notification and Download)")

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
                                  self.received_msg_id),
                               is_system=True)

    def wait_for_incoming_mms(self, timeout, sender_number):
        """
        Waits for incoming MMS, will return successfully if before the timeout
        if the DUT receives an MMS in the thread with id thread_id the number
        of MMS in the inbox reaches the value of the expcepted_number_of_MMS
        parameter

        :type timeout: int
        :param timeout: time in seconds to wait before considering that the
        MMS will never be received

        :type sender_number: str
        :param sender_number: phone number of the sender

        :rtype: int
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

        while (time.time() - start_time) < timeout and is_message_received == False:

            # Sending the start command
            output = self._internal_exec_multiple_v2(self._mms_module, "assembleReceivedMms")
            self._logger.debug("The address of received MMS list : %s" % output)

            # Check if we have already received Mms on Embedded side, before parse
            # the MMS database to lookForIncomingMessages
            # At least one MMS is received and sender number is also found in received MMS
            if len(output) > 0 and sender_number in [mms["address"] for mms in output]:
                # Sending the start command
                output = self._internal_exec_v2(self._mms_module,
                                                function,
                                                "--es %s %s --es %s %s"
                                                % (sender_number_key,
                                                   sender_number,
                                                   start_date_key,
                                                   self.start_date),
                                                is_system=True)

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
                if output.get(thread_id_key) != "-1" \
                    and output.get(message_id_key) != "-1" \
                    and output.get(reception_date_key) != "-1":
                    is_message_received = True

        if is_message_received:
            return output.get(reception_date_key)
        else:
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "Did not received the MMS after waiting %s seconds" % timeout)

    def __get_mms_box(self):
        """
        Returns the message box number of the message described by the thread
        id and message id passed as parameters.

        :rtype: str
        :return: the name of the box the message is in.

        :raise AcsToolException: If the output is missing one of the expected
            keys.
        """
        # Defining keys for input and output of the embedded UE command.
        thread_id_key = "thread_id"
        message_id_key = "msg_id"
        message_box_key = "msg_box_number"
        # Defined the function name used on the embedded side.
        function = "getMmsBoxFromId"
        # Building the command to send to the embedded part.

        # Launch the command and retreive the result.
        output = self._internal_exec_v2(self._mms_module,
                                        function,
                                        "--es %s %s --es %s %s"
                                        % (thread_id_key,
                                           self.sent_thread_id,
                                           message_id_key,
                                           self.sent_msg_id),
                                        is_system=True)

        # Check the output contains the expected keys.
        if message_box_key not in output:
            raise AcsToolException(AcsToolException.INVALID_PARAMETER,
                                   "output key %s not found: %s" % (message_box_key, output))
            # Return the output.
        return output.get(message_box_key)

    def __was_mms_send_cmd_successfull(self, phone_number, send_time):
        """
        Check if the send_mms function executed successfully.
        If the execution was successfull return the message and thread id of
        the MMS.
        If the execution fails returns 0 for both message id and thread id.

        :type phone_number: str
        :param phone_number: destination phone number used to find the mms in
            the database.

        :type send_time: int
        :param send_time: time on embedded side when the send command was sent.

        :rtype: bool
        :return: verdict of send_mms command (True|False)

        :raise AcsToolException:
            - If the output is missing one of the expected keys.
        """
        # Defining keys for input and output parameters of the embedded part.
        phone_number_key = "phone_number"
        sent_time_key = "send_time"
        message_id_key = "msg_id"
        thread_id_key = "thread_id"
        verdict_key = "verdict"
        # Defining and initializing output variables.
        verdict = None
        # Defining function name of the UE command on embedded side.
        function = "wasMmsSendCommandSuccessfull"

        # Sending the command.
        output = self._internal_exec_v2(self._mms_module,
                                        function,
                                        "--es %s %s --es %s %s"
                                        % (phone_number_key,
                                           phone_number,
                                           sent_time_key,
                                           send_time),
                                        is_system=True)
        # If one of the result key is not present raise an exception
        # Checking if the thread id key is present in the output bundle.
        if message_id_key not in output:
            raise AcsToolException(AcsToolException.INVALID_PARAMETER,
                                   "output key %s not found: %s" % (message_id_key, output))
        else:
            # If the output message id is different than -1, save it
            if not output.get(message_id_key) == -1:
                self.sent_msg_id = output.get(message_id_key)
                # Checking if the thread id key is present in the output bundle.
        if thread_id_key not in output:
            raise AcsToolException(AcsToolException.INVALID_PARAMETER,
                                   "output key %s not found: %s" % (thread_id_key, output))
        else:
            # If the output thread id is different than -1, save it
            if not output.get(thread_id_key) == -1:
                self.sent_thread_id = output.get(thread_id_key)
        # Checking if the verdict key is present in the output bundle.
        if verdict_key not in output:
            raise AcsToolException(AcsToolException.INVALID_PARAMETER,
                                   "output key %s not found: %s" % (verdict_key, output))
        else:
            # Converting output to python boolean.
            if output.get(verdict_key) == "true":
                verdict = True
            else:
                verdict = False
                # If UE command is successfull return verdict, thread id and id of the
            # MMS.
        return verdict

    def __is_mms_sent(self, time_out):
        """
        Method that will wait for the MMS to be send.
        It will wait for the MMS to be send until the timeout has been reached.

        :type time_out: int
        :param time_out: time to wait for the MMS to be send.

        :rtype: str
        :return: the time when the message has been sent.

        :raise AcsToolException: if the output is missing one of the expected keys.
        :raise DeviceException: if the message takes more time than the timeout to be sent.
        """
        # Defining task id key for passing parameters to embedded.
        message_id_key = "msg_id"
        thread_id_key = "thread_id"
        sent_time_key = "sent_time"
        # Defining functions name.
        function = "isMmsSent"
        is_mms_sent = False
        start_time = time.time()

        while (time.time() - start_time) < time_out and is_mms_sent == False:
            output = self._internal_exec_v2(self._mms_module,
                                            function,
                                            "--es %s %s --es %s %s"
                                            % (message_id_key,
                                               self.sent_msg_id,
                                               thread_id_key,
                                               self.sent_thread_id),
                                            is_system=True)

            if sent_time_key not in output:
                raise AcsToolException(AcsToolException.INVALID_PARAMETER,
                                       "output key %s not found: %s" % (message_id_key, output))
            if output.get(sent_time_key) != "-1":
                is_mms_sent = True
            else:
                time.sleep(5)
        if is_mms_sent:
            msg = "Send Command was succesfull. Message found with id: %s in conversation: %s" \
                  % (self.sent_msg_id, self.sent_thread_id)
            self._logger.info(msg)
            return output.get(sent_time_key)
        else:
            message_box_name = self.__get_mms_box()
            raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                  "The Mms was not send in time (waited %s seconds). It is in the %s box "
                                  % (time_out, message_box_name))

    def __is_stock_msg_app_running(self):
        """
        Checks id the stock messaging application is running.
        :rtype: boolean
        :return: True if the application is running False otherwise.
        """
        result_key = "isMessagingAppRunning"

        self._logger.info("Check if stock messaging is running.")

        function = "checkIfMmsStockAppIsRunning"

        output = self._internal_exec_v2(self._mms_module, function, is_system=True)

        if result_key not in output:
            raise AcsToolException(AcsToolException.INVALID_PARAMETER,
                                   "output key %s not found : %s" % (result_key, str(output)))
        # Convert result to bool.
        result = Util.str_to_bool(output[result_key])
        return result

    def __get_pdu_precount(self, precount_try=60):
        """
        Try to get pdu count 10 times (by default)
        :rtype: int
        :return: pdu count
        """
        count = precount_try
        pdu_precount = 0
        while count:
            try:
                pdu_precount = self.__get_mms_pdu_count()
                self._logger.debug("PDU precount %s" % str(pdu_precount))
                if pdu_precount != -1:
                    break
            except Exception as error:
                self._logger.warning(str(error))
            count -= 1
            if not count:
                raise DeviceException(DeviceException.CRITICAL_FAILURE,
                                      "Failed to access MMS data base (pdu count) after %d tries" % precount_try)
            time.sleep(1)
        return pdu_precount

    def __get_part_precount(self, precount_try=60):
        """
        Try to get part count 10 times (by default)
        :rtype: int
        :return: part count
        """
        count = precount_try
        part_precount = 0
        while count:
            try:
                part_precount = self.__get_mms_part_count()
                self._logger.debug("PART precount %s" % str(part_precount))
                if part_precount != -1:
                    break
            except Exception as error:
                self._logger.warning(str(error))
            count -= 1
            if not count:
                raise DeviceException(DeviceException.CRITICAL_FAILURE,
                                      "Failed to access MMS data base (part count) after %d tries" % precount_try)
            time.sleep(1)
        return part_precount

    def __get_mms_part_count(self):
        """
        When a part is inserted, if it is not text/plain or application/smil
        (which both can exist with text-only MMSes), then there is an attachment.
        Set has_attachment=1 in the threads table for the thread in question.
        """
        ret = -1
        cmd = "adb shell sqlite3 " + MSG_DB_PATH + " \"select count(1) from part\""
        output = str(subprocess.check_output(cmd)).strip()
        self._logger.debug("output: (%s)" % str(output))
        if output.isdigit():
            ret = output
        return ret

    def __get_mms_pdu_count(self):
        """
        When looking in the pdu table for unread messages, only count messages that
        are displayed to the user. The constants are defined in PduHeaders and could be used
        here, but the str "(m_type=132 OR m_type=130 OR m_type=128)" is used throughout this
        file and so it is used here to be consistent.

            m_type=128   = MESSAGE_TYPE_SEND_REQ
            m_type=130   = MESSAGE_TYPE_NOTIFICATION_IND
            m_type=132   = MESSAGE_TYPE_RETRIEVE_CONF
        """
        ret = -1
        cmd = "adb shell sqlite3 " + MSG_DB_PATH + " \"select count(1) from pdu\""
        output = str(subprocess.check_output(cmd)).strip()
        self._logger.debug("output: (%s)" % str(output))
        if output.isdigit():
            ret = output
        return ret

    def __quit_mms_app(self):
        """
        Press home button to quit MMS application
        """
        KEYCODE_HOME = "3"

        self._logger.info("Return from MMS to Home.")

        cmd = "adb shell input keyevent " + KEYCODE_HOME

        self._exec(cmd)
