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
:summary: Implementation of Microbot Equipment
:since:18/01/2013
:author: lpastor
"""

import logging

from acs_test_scripts.Equipment.IEquipment import ExeRunner
from acs_test_scripts.Equipment.NFCTools.Interface.INFCTools import INFCTools
from ErrorHandling.TestEquipmentException import TestEquipmentException
from Core.Report.ACSLogging import ACS_LOGGER_NAME, EQT_LOGGER_NAME


class NfcEmulator(ExeRunner, INFCTools):

    """
    Implementation of NfcEmulator Equipment
    """

    CONTACT_SENT_BY_EMULATOR = "Nfc"
    CONTACT_EXPECTED_BY_EMULATOR = "Nfc"

    def __init__(self, name, model, eqt_params):
        """
        Constructor

        :type name: str
        :param name: the bench name of the equipment

        :type model: str
        :param model: the model of the equipment

        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment catalog parameters
        """
        INFCTools.__init__(self)
        ExeRunner.__init__(self, name, model, eqt_params)

        # The logger instance to use
        self.__logger = logging.getLogger("%s.%s.%s" % (ACS_LOGGER_NAME, EQT_LOGGER_NAME, self.__class__.__name__,))

    def exchange_apdu(self, protocol_type, bitrate, data_size, apdu_case, loop):
        """
        Exchange APDU with test applet installed on secure element

        :type protocol_type: str
        :param protocol_type: choose std 14443 type A or B

        :type bitrate : str
        :param bitrate: select communication bitrate

        :type data_size : str
        :param data_size: choose command and response data size

        :type apdu_case : str
        :param apdu_case : select the apdu case to send

        :type loop : int
        :param loop : number of apdu to send

        :rtype: bool
        :return: result: True if test passes
        """

        cmd_line = "%s %s %s %s %s %s" % ("apduExchange", protocol_type, bitrate, data_size, apdu_case, loop)
        output = ExeRunner.start_exe(self, cmd_line)

        self._logger.debug("NfcEmulation returnCode : " + str(output[1]))
        if output[1] is None or output[1] != 0:
            self._logger.error("Error during NfcEmulation execution")
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, "Error during NfcEmulation execution")

        result = self._get_apdu_exchange_result(output[0])

        return result

    def _get_apdu_exchange_result(self, response):
        """
        Extract apdu response from the output strings returned by NfcEmulation

        :type response: str
        :param response: str array that contains NfcEmulation output

        :rtype: bool
        :return: final_result: True if APDU responses are OK
        """
        response_split = response.split("result :")
        final_result = True

        if len(response_split) > 1:
            for i in range(1, len(response_split)):
                result = response_split[i].split()
                if result[0] != "9000":
                    final_result = False
                    self.__logger.error("Error during APDU exchange")
                    raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, "Error during APDU exchange")

        else:
            final_result = False
            self.__logger.error("APDU exchange sequence has not been executed")
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, "APDU exchange sequence has not been executed")

        return final_result

    def exchange_contact(self, mode, role, bitrate, direction, result_queue=None):
        """
        Exchange contact between emulator and DUT

        :type mode : str
        :param mode: select communication mode (PASSIVE or ACTIVE)

        :type role: str
        :param role: select dut's role (INITIATOR or TARGET)

        :type bitrate : str
        :param bitrate: select communication bitrate

        :type direction : str
        :param direction: select communication direction (SEND or RECEIVE)

        :type result_queue : queue
        :param result_queue: queue used to store result
        """
        if direction == "SEND" and result_queue is None:
            self.__logger.error("Queue parameter cannot be None if direction is SEND")
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, "Queue parameter cannot be None if direction is SEND")

        cmd_line = "%s %s %s %s %s" % ("p2p", mode, role, bitrate, direction)
        output = ExeRunner.start_exe(self, cmd_line)

        if direction == "SEND":
            is_contact_received = "Nfc contact correctly received" in output[0]
            result_queue.put(is_contact_received)
