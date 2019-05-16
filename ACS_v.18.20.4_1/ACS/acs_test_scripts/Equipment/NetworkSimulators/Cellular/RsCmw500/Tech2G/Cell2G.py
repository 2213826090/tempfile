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
:summary: Cell 2G implementation for R&S CMW500
:since: 09/09/2014
:author: fbelvezx
"""
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ICell2G import ICell2G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Tech2G.VoiceCall2G import VoiceCall2G
from acs_test_scripts.TestStep.Utilities.Visa import VisaObject
from ErrorHandling.TestEquipmentException import TestEquipmentException
import time


class Cell2G(ICell2G, VisaObject):
    """
    Cell GSM implementation for Rohde and Schwarz CMW500 (CMW)
    """

    def __init__(self, visa):
        """
        Constructor
        :type visa: visaInterface
        :param visa: the PyVisa connection
        """
        VisaObject.__init__(self, visa)
        self.__voicecall = VoiceCall2G(visa)
        self.__is_cell_off = True
        self.__rrc_conn_state = "ON"
        self.__antennas_number = None

    def __del__(self):
        """
        Destructor
        """
        del self.__voicecall

    def get_voice_call(self):
        """
        Access to 2G voice call interface.
        :rtype: IVoiceCall2G
        :return: the 2G voice call object.
        """
        return self.__voicecall

    def set_cell_on(self, mimo=None):
        """
        Turns on the cell.

        :raise TestEquipmentException: In case or timeout on cell activation
        """
        # timer value (300 secs) to allow for the cell in the CMW500 to initialize and turn on...
        timer = 300
        cell_enabled = False
        cell_status = self._visa.query_command("SOUR:GSM:SIGN:CELL:STAT?")
        if cell_status != "OFF":
            self.get_logger().info("Cell is currently enabled... disabling")
            self._visa.send_command("SOUR:GSM:SIGN:CELL:STAT OFF")

        # Now enable the cell emulator
        self.get_logger().info("Enabling cell and RF output...")
        self._visa.send_command("SOUR:GSM:SIGN:CELL:STAT ON")
        # Now wait until the cell is up and running before returning control...
        while timer > 0:
            if self._visa.query_command("SOUR:GSM:SIGN:CELL:STAT?") == "ON":
                cell_enabled = True
                break

            # time.sleep(1)
            time.sleep(5)  # There is no need to query so many times for a time consuming operation like this...
            timer -= 5
        if not cell_enabled:
            err_msg = "Error while activating the GSM cell!!!"
            self.get_logger().error(err_msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, err_msg)
        else:
            self.__is_cell_off = False

    def set_cell_off(self):
        """
        Turns off the cell.

        Just disable cell generation in the CMW500
        """
        # Turn off both RF
        self.get_logger().info("Disabling cell and RF output...")
        self._visa.send_command("SOUR:GSM:SIGN:CELL:STAT OFF")
        self.__is_cell_off = True

    def set_cell_service(self, service):
        """
        Sets the cell service
        :type service: str
        :return: the cell service already set. Possible values:
            - "GSM"
            - "GPRS"
            - "EGPRs" (GPIB command formatting)
        """
        if service in "GSM":
            self._visa.send_command("CONFigure:GSM:SIGN:CELL:PSDomain OFF")
        elif service in {"GPRS", "EGPRS"}:
            self._visa.send_command("CONFigure:GSM:SIGN:CELL:PSDomain ON")
            self._visa.send_command("CONFigure:GSM:SIGN:CONNection:PSWitched:TLEVel %s" % service)
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "Invalid cell service type %s" % service)

    def set_band(self, band):
        """
        Sets the band to use for 2G signalling
        :type band: str
        :param band: the band to use. Possible values:
            - "TGSM380" => "UNSUPPORTED"
            - "TGSM410" => "UNSUPPORTED"
            - "GSM450" => "UNSUPPORTED"
            - "GSM480" => "UNSUPPORTED"
            - "GSM710" => "UNSUPPORTED"
            - "GSM750" => "UNSUPPORTED"
            - "TGSM810" => "UNSUPPORTED"
            - "GSM850" => "G085"
            - "EGSM" => "G09"
            - "RGSM" => "G09"
            - "TGSM" => "G09"
            - "DCS" => "G18"
            - "PCS" => "P19"
        """
        if band in "PGSM" or band in "RGSM" or band in "EGSM":
            band = "G09"
        elif band in "GSM850":
            band = "G085"
        elif band in "DCS":
            band = "G18"
        elif band in "PCS":
            band = "G19"
        else:
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                         "Band %s is not supported by RS CMW500" % band)
        self._visa.send_command("CONFigure:GSM:SIGN:BAND:BCCH %s" % band)

    def is_dut_registered(self, dut_imsi):
        """
        Test if DUT is synchronized with equipment
        :type dut_imsi: str
        :param dut_imsi: IMSI retrieved from CDK.
        :rtype: boolean
        :return: true if the DUT is synchronized with the equipment, false otherwise
        """
        dut_imsi = dut_imsi.replace('\n', '')
        dut_imsi = dut_imsi.replace('"', '')

        imsi = self._visa.query_command("SENSe:GSM:SIGN:MSSinfo:IMSI?")

        imsi = imsi.replace('"', '')  # pylint: disable=E1101

        return imsi == dut_imsi
