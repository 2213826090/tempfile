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
:summary: Use Case RAT SELECTION (Base)
:since: 12/05/2013
:author: jreynaux
:since: 19/02/2014
:author: galagnox
"""
import time

from acs_test_scripts.UseCase.Mobility.LAB_MOBILITY_3GSM_BASE import LabMobility3gsmBase
from UtilitiesFWK.Utilities import Global, str_to_bool
import acs_test_scripts.Device.UECmd.UECmdTypes as UECmdTypes
from acs_test_scripts.Utilities.SmsUtilities import \
    compute_sms_equals, compute_sms_segments, SmsMessage, DataCodingScheme
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsBaseException import AcsBaseException


class LabRatSelectionBase(LabMobility3gsmBase):
    """
    Lab Rat Selection class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """

        # Call LabMobility3gsmBase Init function
        LabMobility3gsmBase.__init__(self, tc_name, global_config)

        # Read registrationTimeout from Device_Catalog.xml
        self._registration_timeout = \
            int(self._dut_config.get("registrationTimeout"))

        # Read isSpeechCallSupported from Device_Catalog.xml, this will drive the tests on UC
        self._speechcall_supported = \
            str_to_bool(self._dut_config.get("isSpeechCallSupported", "false"))
        # Read isSmsSupported from Device_Catalog.xml, this will drive the tests on UC
        self._sms_supported = \
            str_to_bool(self._dut_config.get("isSmsSupported", "false"))

        self._default_rat = self._dut_config.get("defaultPreferredNetwork", "3G_PREF")

        # Read the first network to set from testcase xml parameters
        self._wanted_network1 = \
            self._tc_parameters.get_param_value("WANTED_NETWORK1")
        # Read the second network to set from testcase xml parameters
        self._wanted_network2 = \
            self._tc_parameters.get_param_value("WANTED_NETWORK2")

        # Read NS1 parameters from test case xml file
        self._ns1_cell_service = \
            str(self._tc_parameters.get_param_value("NS1_CELL_SERVICE"))

        self._ns1_cell_tech = \
            str(self._tc_parameters.get_param_value("NS1_CELL_TECH"))

        self._ns1_arfcn = \
            int(self._tc_parameters.get_param_value("NS1_ARFCN"))

        self._ns1_cell_power = \
            float(self._tc_parameters.get_param_value("NS1_CELL_POWER"))

        self._ns1_registration_timeout = \
            int(self._tc_parameters.get_param_value("NETWORK1_REG_TIMEOUT", "30"))

        # Read NS2 parameters from test case xml file
        self._ns2_cell_service = \
            str(self._tc_parameters.get_param_value("NS2_CELL_SERVICE"))

        self._ns2_cell_tech = \
            str(self._tc_parameters.get_param_value("NS2_CELL_TECH"))

        self._ns2_arfcn = \
            int(self._tc_parameters.get_param_value("NS2_ARFCN"))

        self._ns2_cell_power = \
            float(self._tc_parameters.get_param_value("NS2_CELL_POWER"))

        self._ns2_registration_timeout = \
            int(self._tc_parameters.get_param_value("NETWORK2_REG_TIMEOUT", "30"))

        self._ns2_delay_cell_activation = \
            int(self._tc_parameters.get_param_value("NS2_DELAY_CELL_ACTIVATION", "0"))

        self._cresel_power = \
            float(self._tc_parameters.get_param_value("CRESEL_POWER", (-60)))

        # Read DECREMENTATION_STEP_POWER from testcase xml parameters
        self._decrementation_step_power = \
            float(self._tc_parameters.get_param_value("DECREMENTATION_STEP_POWER", "4"))

        # Read DECREMENTATION_STEP_TIMER from testcase xml parameters
        self._decrementation_step_timer = \
            float(self._tc_parameters.get_param_value("DECREMENTATION_STEP_TIMER", "5"))

        # Read CRESEL_LIMIT_POWER from testcase xml parameters
        self._cresel_limit_power = \
            float(self._tc_parameters.get_param_value("CRESEL_LIMIT_POWER", "-110"))

        # Read VC_TYPE from test case xml file
        self._vc_type = str(self._tc_parameters.get_param_value("VC_TYPE", "MT"))

        # Read MMS_TYPE from test case xml file
        self._sms_type = str(self._tc_parameters.get_param_value("MMS_TYPE", "MT"))

        self._resel = \
            str(self._tc_parameters.get_param_value("RESEL", "FALSE"))

        self._retry_count = \
            int(self._tc_parameters.get_param_value("RETRY_COUNT", "1"))

        """
        SMS Configuration
        """
        # Read DATA_CODING_SCHEME from xml UseCase parameter file
        self._data_coding_scheme = \
            self._tc_parameters.get_param_value("DATA_CODING_SCHEME", "00")

        # Read SMS_TEXT from xml UseCase parameter file
        self._sms_text = self._tc_parameters.get_param_value("SMS_TEXT", "This is my SMS text !")

        # Read SMS_TRANSFER_TIMEOUT from xml UseCase parameter file
        self._sms_transfer_timeout = \
            int(self._tc_parameters.get_param_value("SMS_TRANSFER_TIMEOUT", "10"))

        self._sms_sender = "8960"

        dcs = DataCodingScheme(self._data_coding_scheme)
        dcs.decode()

        # Get number of bits per character set in DCS
        self._nb_bits_per_char = dcs.compute_character_size()

        character_set = dcs.get_character_set()

        if character_set == "7BITS":
            self._content_type = "CTEX"
        else:
            self._content_type = "CDAT"

        # Add Messaging API as LabMobility3gsmBase and LabMobilityBase does not use it
        self._messaging_api = self._device.get_uecmd("SmsMessaging")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")

        # UECmd type
        self._uecmd_types = UECmdTypes

#------------------------------------------------------------------------------
    def set_up(self):
        """
        Set up the test configuration
        """

        # Call LabMobility3gsmBase Setup function so Cell 1 and Cell 2 are OFF.
        LabMobility3gsmBase.set_up(self)

        # Set Cell Band and ARFCN using NS1_CELL_BAND
        # and NS1_ARFCN parameters
        # Set cell service using NS1_CELL_SERVICE parameter
        # Set Cell Power using NS1_CELL_POWER parameter
        self._ns1_cell.configure_basic_cell_parameters(
            self._ns1_cell_service, self._ns1_cell_band,
            self._ns1_arfcn, self._ns1_cell_power)

        if "3G" in self._ns1_cell_tech:
            self._logger.info("Set the RRC Idle With SCRI State on for NS1")
            self._ns1_data.set_fast_dormancy_support("enable", 10)

        # Set Cell Band and ARFCN using NS2_CELL_BAND
        # and NS2_ARFCN parameters
        # Set cell service using NS2_CELL_SERVICE parameter
        # Set Cell Power using NS2_CELL_POWER parameter
        self._ns2_cell.configure_basic_cell_parameters(
            self._ns2_cell_service, self._ns2_cell_band,
            self._ns2_arfcn, self._ns2_cell_power)

        if "3G" in self._ns2_cell_tech:
            self._logger.info("Set the RRC Idle With SCRI State on for NS2")
            self._ns2_data.set_fast_dormancy_support("enable", 10)

        # If SMS supported, need to configure others things
        if self._sms_supported:
            # Set SMS parameters using NS1 messaging
            # Set the Data coding scheme using DATA_CODING_SCHEME value
            self._ns1_messaging.set_sms_data_coding_scheme(
                int(self._data_coding_scheme, 16))
            # Set sender address using DESTINATION_NUMBER on Network simulator
            self._ns1_messaging.set_sms_sender_address(self._sms_sender)
            # Clear old SMS on 8960
            self._ns1_messaging.clear_message_data()

            # Set SMS parameters using NS2 messaging
            # Set the Data coding scheme using DATA_CODING_SCHEME value
            self._ns2_messaging.set_sms_data_coding_scheme(
                int(self._data_coding_scheme, 16))
            # Set sender address using DESTINATION_NUMBER on Network simulator
            self._ns2_messaging.set_sms_sender_address(self._sms_sender)
            # Clear old SMS on 8960
            self._ns2_messaging.clear_message_data()

            # Clear old SMS
            time.sleep(self._wait_btwn_cmd)
            self._messaging_api.delete_all_sms()

        return Global.SUCCESS, "No errors"

#------------------------------------------------------------------------------
    def tear_down(self):
        """
        End and dispose the test
        """
        # Reset Default Preferred RAT
        if self._default_rat is not None:
            self._logger.debug("Setting of the initial default preferred network type (%s)." % self._default_rat)
            self.configure_preferred_rat(self._default_rat)
        else:
            self._logger.warning("No default rat parameter found on device catalog, unable to reset rat")

        if self._sms_supported:
            # Clear old SMS
            time.sleep(self._wait_btwn_cmd)
            self._messaging_api.delete_all_sms()

        # Enable Data output traffic
        self._networking_api.enable_output_traffic()

        # Call LAB_MOBILITY_3GSM_BASE
        LabMobility3gsmBase.tear_down(self)

        return Global.SUCCESS, "No errors"

# -----------------------------------------------------------------------------
    def check_registration_on_ns(self, ns_data, timeout=None):
        """
        Perform a standard registration check according RATSEL_xx specification:
        - Attachment on network
        - Network type
        - PDP context active

        :type ns_data: str
        :param ns_data: The network simulator to be used for check NS1|NS2

        :type timeout:
        """
        if timeout is None:
            timeout = self._registration_timeout

        if ns_data not in ["NS1", "NS2"]:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "Wrong Network simulator param for check, should be NS1 or NS2")
        if "NS1" in ns_data:
            ns_data = self._ns1_data
        else:
            ns_data = self._ns2_data

        # Check that DUT is attached on NS => ATTACHED before timeout
        ns_data.check_data_connection_state("ATTACHED",
                                            timeout,
                                            blocking=True)
        # Check that DUT is registered on the good RAT
        self._modem_api.check_network_type_before_timeout(ns_data.get_network_type(), self._registration_timeout)

        # Set the APN
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Setting APN " + str(self._apn) + "...")
        self._networking_api.set_apn(self._ssid, self._apn)

        # Activate PDP context
        time.sleep(self._wait_btwn_cmd)
        self._logger.info("Active PDP Context...")
        self._networking_api.activate_pdp_context(self._ssid, check=False)

        # Check Data Connection State => PDP_ACTIVE before timeout
        ns_data.check_data_connection_state("PDP_ACTIVE",
                                            self._registration_timeout,
                                            blocking=True)

#------------------------------------------------------------------------------
    def configure_preferred_rat(self, rat, check=False):
        """
        Configure given rat on device then check if requested.

        :type rat: int
        :param rat: The rat to be set on device

        :type check: bool
        :param check: Define whether the check is performed or not

        :rtype: None
        """
        self._phone_system_api.sleep_screen()

        try:
            self._networking_api.set_preferred_network_type(rat)
        except:
            raise

        time.sleep(5)
        if check:
            # need time to get dut register to network, then to get_preferred_network_type
            # Check registration state is connected using registrationTimeout from Device_Catalog.xml
            self._modem_api.check_cdk_registration_bfor_timeout(
                self._registration_timeout)

            current_rat = self._networking_api.get_preferred_network_type()

            if current_rat != rat:
                raise DeviceException(DeviceException.CRITICAL_FAILURE,
                                      "The preferred network is not correct (expected: %s, found: %s)" \
                                      % (str(rat), str(current_rat)))

#------------------------------------------------------------------------------
    def check_networking_status_bfor_timeout(self, status, blocking, timeout=None):
        """
        Check modem status before given timeout

        :type status: int
        :param status: 0 for Network status disconnected,
                       1 for Network status connected.
        :type blocking: bool
        :param blocking: True, Failed test is blocking and exception is raise.
                        False, Failed test is not blocking.
        :type timeout: int
        :param timeout: Time to wait setting preferred network type
        :rtype: bool
        :return: True if status reached before timeout, else False (if blocking is set to False).
        """
        # modem_state: 0 for OFF or 1 for ON
        modem_state = 0

        if timeout is None:
            timeout = self._registration_timeout

        if status == 0:
            network_status = "disconnected"
        else:
            network_status = "connected"

        msg = "Check Network state is %s before %s seconds..." % (network_status, str(timeout))
        self._logger.info(msg)

        start_time = time.time()
        while (time.time() - start_time) <= timeout:
            self._phone_system_api.wake_screen()
            modem_state = self._modem_api.get_modem_online_status()
            if modem_state == status:
                break
            time.sleep(1)

        return_msg = "Network status %s " % network_status
        if modem_state == status:
            return_msg += " has been reached !"
            self._logger.info(return_msg)
            return True
        else:
            return_msg += " has not been reached on time !"
            self._logger.error(return_msg)
            if blocking:
                raise DeviceException(DeviceException.TIMEOUT_REACHED,
                                      "Failed to reach %s Network state before timeout!" % network_status)
            else:
                return False

#------------------------------------------------------------------------------
    def do_ratsel_main_test(self, ns):
        """
        RAT selection Main test: depending on device capability (speech, sms),
        perform a voice call or an SMS send.

        :type ns: str
        :param ns: NS1 or NS2
        :rtype: Verdict, message
        """
        verdict = Global.SUCCESS
        output = "No Error"

        if ns not in ["NS1", "NS2"]:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "Wrong Network simulator param, should be NS1 or NS2")

        # Category a: CS speech call is supported
        if self._speechcall_supported:
            # Wake up screen
            self._phone_system_api.wake_screen()
            self._logger.info("")
            self._logger.info("*** Speech call is supported by device, test %s VC" % self._vc_type)
            # wait for 5 second
            self._logger.info("")
            self._logger.info("*** Waiting for 5 seconds ...")
            time.sleep(5)
            (verdict, output) = self.do_vc_test(ns)

        # Category b: CS speech call is not supported but SMS is supported
        elif not self._speechcall_supported and self._sms_supported:
            # Wake up screen
            self._phone_system_api.wake_screen()
            self._logger.info("")
            self._logger.info("*** Speech call is not is supported by device, but SMS is supported, test %s SMS" % self._sms_type)
            # wait for 5 second
            self._logger.info("")
            self._logger.info("*** Waiting for 5 seconds ...")
            time.sleep(5)
            (verdict, output) = self.do_sms_test(ns)

        # Category c: CS speech call and SMS are not supported
        else:
            self._logger.info("")
            self._logger.info("*** Device doesn't support speech call nor SMS, do nothing else")

        return verdict, output

#------------------------------------------------------------------------------
    def do_vc_test(self, ns):
        """
        Test for category a devices: perform a Voice call

        :type ns: str
        :param ns: NS1 or NS2
        :rtype: Verdict, message
        """
        if ns not in ["NS1", "NS2"]:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "Wrong Network simulator param, should be NS1 or NS2")

        msg = "No error on VC test (%s). " % ns
        verdict = Global.SUCCESS

        if "NS1" in ns:
            ns_cell_tech = self._ns1_cell_tech
            ns_vc_registered = self._ns1_vc
        else:
            ns_cell_tech = self._ns2_cell_tech
            ns_vc_registered = self._ns2_vc

        try:
            # Wake up screen
            self._phone_system_api.wake_screen()

            if "MT" in self._vc_type:
                # Mobile Terminated call
                # Initiate VoiceCall to CDK
                ns_vc_registered.mt_originate_call()

                # Check call status is incoming before callSetupTimeout
                self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.INCOMING, 30)
                self._logger.info("*** INCOMING call state reached on DUT")

                if "3G" in ns_cell_tech:
                    ns_vc_registered.check_call_state("CALL", 1, blocking=True)
                    self._logger.info("*** CALL call state reached on 3G equipment")
                else:
                    ns_vc_registered.check_call_state("ALER", 1, blocking=True)
                    self._logger.info("*** ALER call state reached on 2G equipment")

            else:
                # Mobile Originated call
                # Dial using PHONE_NUMBER parameter
                self._voicecall_api.dial("8960")

                # Wait for state "active" before callSetupTimeout seconds
                self._voicecall_api.wait_for_state(self._uecmd_types.VOICE_CALL_STATE.ALERTING, 30)

                self._logger.info("*** ALERTING call state reached on DUT")

                # Check call state "CONNECTED" before callSetupTimeout seconds
                ns_vc_registered.check_call_connected(self._call_setup_time)

        except Exception as exception:
            msg = "%s. " % str(exception)
            verdict = Global.FAILURE
            self._logger.error(msg)

        finally:
            # Mobile Release call
            self._voicecall_api.release()

        return verdict, msg

    def do_sms_test(self, ns):
        """
        Test for category b devices: Send an SMS

        :type ns: str
        :param ns: NS1 or NS2

        :rtype: Verdict, message
        """
        if ns not in ["NS1", "NS2"]:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "Wrong Network simulator param, should be NS1 or NS2")
        if "NS1" in ns:
            ns_messaging = self._ns1_messaging
            ns_data = self._ns1_data
        else:
            ns_messaging = self._ns2_messaging
            ns_data = self._ns2_data

        # Wake up screen
        self._phone_system_api.wake_screen()

        # Check Data Connection State => PDP_ACTIVE before timeout
        ns_data.check_data_connection_state("PDP_ACTIVE", self._registration_timeout, False)

        # Mobile Terminated / Mobile Originated SMS
        (result_verdict, result_message) = self._send_sms(ns_messaging)
        result_message += " (%s). " % ns
        self._logger.info(result_message)

        return result_verdict, result_message

    def _send_sms(self, ns_messaging):
        """
        Send sms from equipment or DUT

        :type ns_messaging: IMessaging3G
        :param ns_messaging: network simulator messaging

        :rtype: Verdict, message
        """
        nb_segments = \
            compute_sms_segments(self._sms_text, self._nb_bits_per_char)

        if "MT" in self._sms_type:
            # [SEND MT SMS DIRECTLY BY EQUIPMENT]

            # register on intent to receive incoming sms
            self._messaging_api.register_for_sms_reception()

            # Construct SMS
            sms_sent = SmsMessage(self._sms_text, self._sms_sender, "GSM")

            # Configure the type of the message to send CUSTOM TEXT.
            ns_messaging.select_sms_content(self._content_type)

            # Set the custom text message to send, using SMS_TEXT parameter
            if self._content_type == "CTEX":
                ns_messaging.set_custom_sms_text(self._sms_text)
            elif self._content_type == "CDAT":
                ns_messaging.set_custom_sms_data(self._sms_text)

            # Send MT SMS to CDK using SMS parameters :
            # - SMS_TEXT
            ns_messaging.send_sms()

            # Check sms acknowledged by network simulator
            ns_messaging.check_sms_state(
                'ACK', self._sms_transfer_timeout)
        else:
            # [SEND MO SMS BY DUT]

            # Construct SMS
            sms_sent = SmsMessage(self._sms_text, self._sms_sender, "GSM")

            self._messaging_api.send_sms(self._sms_sender, self._sms_text)

        # Get received sms
        sms_received = self._messaging_api.wait_for_incoming_sms(self._sms_transfer_timeout)

        # Compare sent and received SMS (Text,
        # Destination number)
        (result_verdict, result_message) = \
            compute_sms_equals(sms_sent, sms_received)

        return result_verdict, result_message

    def _go_to_idle_state(self, ns, timeout=None):
        """
        :type ns_messaging: IMessaging3G
        :param ns_messaging: network simulator messaging
        :type timeout: int
        :param timeout: timeout to reach idle state
        :rtype: Verdict, message
        """
        if ns not in ["NS1", "NS2"]:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER,
                                   "Wrong Network simulator param, should be NS1 or NS2")

        if timeout is None:
            timeout = self._registration_timeout

        if "NS1" in ns:
            ns_data = self._ns1_data
            ns_cell_tech = self._ns1_cell_tech
        else:
            ns_data = self._ns2_data
            ns_cell_tech = self._ns2_cell_tech

        msg = "Go to Idle state before %s seconds..." % str(timeout)
        self._logger.info(msg)

        start_time = time.time()
        while (time.time() - start_time) <= timeout:
            if "3G" in ns_cell_tech:
                if "PDP_ACTIVE" in str(ns_data.get_data_connection_status()) and "idle" in str(ns_data.get_rrc_states()).lower():
                    msg = "DUT reach IDLE mode (%s) in %ds. " % (ns, time.time()-start_time)
                    self._logger.info(msg)
                    return Global.SUCCESS, msg

            elif "PDP_ACTIVE" in str(ns_data.get_data_connection_status()) or "ATTACHED" in str(ns_data.get_data_connection_status()):
                    msg = "DUT reach IDLE mode (%s) in %ds. " % (ns, time.time() - start_time)
                    self._logger.info(msg)
                    return Global.SUCCESS, msg

            # Wait for 1 s
            time.sleep(1)

        # Timeout as been reach, fail to reach Idle state.
        msg = "DUT Failed to reach IDLE mode (%s) in %ds. " % (ns, timeout)
        self._logger.error(msg)
        return Global.FAILURE, msg
