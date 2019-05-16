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
:summary: Implementation of Gpshell Equipment
:since:18/01/2013
:author: lpastor
"""

import logging
import os
import shutil
import tempfile

from acs_test_scripts.Equipment.IEquipment import ExeRunner
from acs_test_scripts.Equipment.NFCTools.Interface.INFCTools import INFCTools
from ErrorHandling.TestEquipmentException import TestEquipmentException
from Core.Report.ACSLogging import ACS_LOGGER_NAME, EQT_LOGGER_NAME


class Gpshell(ExeRunner, INFCTools):

    """
    Implementation of Gpshell Equipment
    """

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

    def _add_send_apdu_command(self, apdu_case, data_length, loop, script_file):
        """
        Add send_APDU command in script file

        :type apdu_case: str
        :param apdu_case: apdu case number. It may be 1, 2, 3, 4 or ALL

        :type data_length: int
        :param data_length: data length used in apdu command

        :type loop: int
        :param loop: number of apdu to send. If apdu_case is "ALL", each type of apdu is sent "loop" times

        :type script_file: str
        :param script_file: script file to edit before launching GPShell
        """
        if data_length == 256:
            le = 0
            lc = 255
        else:
            le = data_length
            lc = data_length

        script = open(script_file, 'a')

        for i in range(1, loop + 1):
            if apdu_case == "ALL":
                script.write('\nsend_apdu -sc 0 -APDU ')
                script.write('00010000')
                script.write('\nsend_apdu -sc 0 -APDU ')
                script.write('00020000')
                script.write(("%X" % le).zfill(2))
                script.write('\nsend_apdu -sc 0 -APDU ')
                script.write('00030000')
                script.write(("%X" % lc).zfill(2))
                for i in range(0, data_length - 1):
                    script.write(("%X" % i).zfill(2))
                script.write('\nsend_apdu -sc 0 -APDU ')
                script.write('00040000')
                script.write(("%X" % lc).zfill(2))
                for i in range(0, data_length - 1):
                    script.write(("%X" % i).zfill(2))
                script.write(("%X" % le).zfill(2))
            else:
                script.write('\nsend_apdu -sc 0 -APDU ')
                if apdu_case == "1":
                    script.write('00010000')
                elif apdu_case == "2":
                    script.write('00020000')
                    script.write(("%X" % le).zfill(2))
                elif apdu_case == "3":
                    script.write('00030000')
                    script.write(("%X" % lc).zfill(2))
                    for i in range(0, data_length - 1):
                        script.write(("%X" % i).zfill(2))
                elif apdu_case == "4":
                    script.write('00040000')
                    script.write(("%X" % lc).zfill(2))
                    for i in range(0, data_length - 1):
                        script.write(("%X" % i).zfill(2))
                    script.write(("%X" % le).zfill(2))

        script.close()

    def _build_apdu_exchange_script(self, apdu_case, data_length, loop, script_file):
        """
        Build script to be used as parameter of GPShell to test APDU exchange

        :type apdu_case: str
        :param apdu_case: apdu case number. It may be 1, 2, 3, 4 or ALL

        :type data_length: int
        :param data_length: data length used in apdu command

        :type loop: int
        :param loop: number of apdu to send. If apdu_case is "ALL", each type of apdu is sent "loop" times

        :type script_file: str
        :param script_file: script file to edit before launching GPShell
        """
        script = open(script_file, 'w')
        script.write('establish_context\nenable_trace\ncard_connect\nselect -AID D27600011801')
        script.close()

        self._add_send_apdu_command(apdu_case, data_length, loop, script_file)

        script = open(script_file, 'a')
        script.write('\ncard_disconnect\nrelease_context')
        script.close()

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

        self._logger.info("Protocol type and bitrate parameters cannot be configured with this tool")

        # create temporary folder to store script
        script_folder = tempfile.mkdtemp()
        script_file = str(os.path.join(script_folder, "exchangeApduScript.txt"))
        self._logger.info("Script file location is " + script_file)

        self._build_apdu_exchange_script(apdu_case, data_size, loop, script_file)

        cmd_line = "%s" % script_file
        output = ExeRunner.start_exe(self, cmd_line)

        self._logger.debug("GPShell returnCode : " + str(output[1]))
        if output[1] is None or output[1] != 0:
            self._logger.error("Error during GPShell execution")
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, "Error during GPShell execution")

        result = self._get_apdu_exchange_result(output[0])

        # remove temporary folder where script is stored
        self._logger.info("Remove temporary script folder")
        shutil.rmtree(script_folder, ignore_errors=False)

        return result

    def _get_apdu_exchange_result(self, response):
        """
        Extract apdu response from the output strings returned by GPShell

        :type response: str
        :param response: str array that contains gpshell output

        :rtype: boolean
        :return: final_result: True if APDU responses are OK
        """
        response_split = response.split("send_APDU() returns ")
        final_result = True

        if len(response_split) > 1:
            for i in range(1, len(response_split)):
                self.__logger.info("Send APDU case " + str(((i - 1) % 4) + 1))
                apdu_result = response_split[i].split()
                result = int(apdu_result[0], 16) & 0x0000FFFF
                self.__logger.info("Command response is : " + hex(result))
                if result != 0x9000:
                    final_result = False
                    self.__logger.error("Apdu case " + str(i) + "error")
                    raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, "Apdu case " + str(i) + "error")

        else:
            final_result = False
            self.__logger.error("send_APDU() has not been sent")
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, "send_APDU() has not been sent")

        return final_result

    def check_reader_connection(self):
        """
        Check APDU response returned to see if card is removed

        :rtype: boolean
        :return: True if APDU responses are OK, False if no
                                connection possible
        """
        # Create temporary folder to store script
        script_folder = tempfile.mkdtemp()
        script_file = str(os.path.join(script_folder, "checkConnectionApduScript.txt"))

        output = ""
        # Build script file
        script = open(script_file, 'w')
        try:
            script.write('establish_context\nenable_trace\ncard_connect\ncard_disconnect\nrelease_context')

            # Run program that sends APDU
            cmd_line = "%s" % script_file
            output = ExeRunner.start_exe(self, cmd_line)

        finally:
            script.close()

        # remove temporary folder where script is stored
        self._logger.debug("Remove temporary script folder")
        try:
            shutil.rmtree(script_folder, ignore_errors=False)
        except Exception as ex:  # pylint: disable=W0703
            self.__logger.error(" Error during removal of temp directory (" + str(ex) + ")")

        if "card_connect() returns " in output[0]:
            return False

        return True
