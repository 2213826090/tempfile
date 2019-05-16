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

:organization: INTEL QCTV
:summary: Cell TDSCDMA implementation for R&S CMW500 using visa interface
:since: may 07th 2015
:author: Martin Brisbarre
"""
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ICell3G import ICell3G
from acs_test_scripts.TestStep.Utilities.Visa import VisaObject
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Tech3G.VoiceCallTDSCDMA import VoiceCallTDSCDMA
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Tech3G.DataTDSCDMA import DataTDSCDMA
from ErrorHandling.TestEquipmentException import TestEquipmentException
import time


class CellTDSCDMA(ICell3G, VisaObject):
    """
    Cell LTE implementation for Rohde and Schwarz CMW500 (CMW)
    """

    def __init__(self, visa):
        """
        Constructor
        :type visa: visaInterface
        :param visa: the PyVisa connection
        """
        VisaObject.__init__(self, visa)
        self.__data = DataTDSCDMA(visa)
        self.__is_cell_off = True
        self.__rrc_conn_state = "ON"
        self.__antennas_number = None

    def __del__(self):
        """
        Destructor
        """
        VisaObject.__del__(self)

    def get_data(self):
        """
        Access to LTE data interface.
        :rtype: IData4G
        :return: the LTE data object.
        """
        return self.__data

    def set_cell_on(self, mimo=None):
        """
        Turns on the cell.

        :raise TestEquipmentException: In case or timeout on cell activation
        """
        # timer value (300 secs) to allow for the cell in the CMW500 to initialize and turn on...
        timer = 300
        cell_enabled = False
        cell_status = self._visa.query("SOURce:TDSCdma:SIGN:CELL:STATe:ALL ?")
        if cell_status != "OFF":
            self.get_logger().info("Cell is currently enabled... disabling")
            self._visa.write("SOURce:TDSCdma:SIGN:CELL:STATe OFF")

        # Now enable the cell emulator
        self.get_logger().info("Enabling cell and RF output...")
        self._visa.write("SOURce:TDSCdma:SIGN:CELL:STATe ON")
        # Now wait until the cell is up and running before returning control...
        while timer > 0:
            if self._visa.query("SOURce:TDSCdma:SIGN:CELL:STATe:ALL ?") == "ON":
                cell_enabled = True
                break

            # time.sleep(1)
            time.sleep(5)  # There is no need to query so many times for a time consuming operation like this...
            timer -= 5
        if not cell_enabled:
            err_msg = "Error while activating the TDSCDMA cell!!!"
            self.get_logger().error(err_msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, err_msg)
        else:
            self.__is_cell_off = False

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

        ritype = self._visa.query("SENSe:TDSCdma:SIGN:UESinfo:RITYpe?")
        if ritype not in ["IMSI", "'IMSI'"]:
            self.get_logger().warning("Callbox didn't receive IMSI from phone, %s received instead" % ritype)
        imsi = self._visa.query("SENSe:TDSCdma:SIGN:UESinfo:RIDentity?")

        imsi = imsi.replace('"', '')  # pylint: disable=E1101

        return imsi == dut_imsi

    def get_cell_status(self):
        """
        Turns on the cell.

        :rtype:str
        :return: Cell state (ON|OFF)
        """
        return self._visa.query("SOURce:TDSCdma:SIGN:CELL:STATe:ALL ?")

    def set_cell_off(self):
        """
        Turns off the cell.
        """
        # Turn off both RF
        self.get_logger().info("Disabling cell and RF output...")
        self._visa.write("SOURce:TDSCdma:SIGN:CELL:STATe OFF")
        self.__is_cell_off = True

    def set_cell_power(self, power):
        """
        Sets the cell power amplitude

        :type power: double
        :param power: cell power to set
        """
        self._visa.write("CONFigure:TDSCdma:SIGN:DL:LEVel:PCCPch %s" % power)
        self.get_logger().info("Configured total cell power : %s dBm" % power)

    def _set_general_settings(self):
        """
        Specifying General and Common Measurement Settings on CMW500:
        - Set duplex mode FDD
        - Enable E2E data transfers
        - Enable a connection to the DAU.
        - Select KRRC mode

        .. seealso:: Chapter 5.5.1.1 Specifying General Settings
                of CMW500 LTE UE Firmware Applications User Manual

        :rtype: None
        """
        self.get_logger().debug("#### Configure general setting ####")

        # Enable E2E data transfers
        self.get_logger().info("Enabling E2E data transfers on this cell...")
        self._visa.write("CONFigure:TDSCdma:SIGN:ETOE ON")
        # Enable a connection to the DAU.
        self.get_logger().info("Configuring LTE signaling unit to use DAU as data source/sink")
        self._visa.write("CONF:LTE:SIGN:CONN:CTYP DAPP")
