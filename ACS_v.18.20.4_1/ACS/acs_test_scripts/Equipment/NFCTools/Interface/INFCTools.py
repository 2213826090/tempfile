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
:summary: virtual interface for NFC tools
:since:17/01/2013
:author: lpastor
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class INFCTools(object):

    """
    Virtual interface for NFC tools
    """

    CONTACT_SENT_BY_EMULATOR = None

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def check_reader_connection(self):
        """
        Check APDU response returned to see if card is removed

        :rtype: boolean
        :return: True if APDU responses are OK, False if no connection possible
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def run_script(self, script_name):
        """
        Run script using MP300 TCL2 from micropross

        :type script_name: str
        :param script_name: name of the script to run
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def init(self):
        """
        Initializes the equipment and establishes the connection.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def release(self):
        """
        Release the equipment and all associated resources and kill child process
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
