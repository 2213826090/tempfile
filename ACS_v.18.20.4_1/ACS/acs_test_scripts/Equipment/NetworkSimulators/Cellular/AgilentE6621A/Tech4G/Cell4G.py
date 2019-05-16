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
:summary: Cell 4G implementation for Agilent E6621A
:since: 27/04/2012
:author: lvacheyx

.. note:: BZ3071 - PING MO over LTE network

"""

from acs_test_scripts.Equipment.NetworkSimulators.Cellular.AgilentE6621A.Tech4G.Data4G import Data4G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ICell4G import ICell4G
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.TestEquipmentException import TestEquipmentException
import time


class Cell4G(ICell4G):

    """
    Cell LTE implementation for Agilent E6621A (PXT)
    """

    def __init__(self, root, name="A"):  # pylint: disable=W0231
        """
        Constructor
        :type root: weakref
        :param root: a weak reference on the root class (Agilent8960)
        """
        self.__root = root
        self.__data = Data4G(root, name)
        self.__cell_name = name

    def __del__(self):
        """
        Destructor
        """
        del self.__data

    def __error_check(self, err, msg):
        """
        Error checking and warning reporting
        :raise TestEquipmentException: if err < 0
        """
        if err < 0:
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)
        elif err > 0:
            self.get_logger().warning(msg)

    def get_root(self):
        """
        Gets the root of the equipment
        :rtype: Agilent8960
        :return: the root of the equipment
        """
        return self.__root

    def get_logger(self):
        """
        Gets the logger
        """
        return self.get_root().get_logger()

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

        :type mimo: boolean
        @ param mimo: If mimo is not used, only RFO1 / MOD1 are on
        If mimo is used, RFO1 / MOD1 and RF02 / MOD2 are on
        """
        if mimo == False:
            # Turn on RF1
            self.get_logger().info("Set RFO1 to on")
            self.__root.send_command("RFO1:STAT ON")
            self.get_logger().info("Set MOD1 to on")
            self.__root.send_command("MOD1:STAT ON")
            # Turn off RF2
            self.get_logger().info("Set RFO2 to off")
            self.__root.send_command("RFO2:STAT OFF")
            self.get_logger().info("Set MOD2 to off")
            self.__root.send_command("MOD2:STAT OFF")

        elif mimo:
            # Turn on all RF
            self.get_logger().info("Set RFO1 to on")
            self.__root.send_command("RFO1:STAT ON")
            self.get_logger().info("Set MOD1 to on")
            self.__root.send_command("MOD1:STAT ON")
            self.get_logger().info("Set RFO2 to on")
            self.__root.send_command("RFO2:STAT ON")
            self.get_logger().info("Set MOD2 to on")
            self.__root.send_command("MOD2:STAT ON")

    def set_cell_off(self):
        """
        Turns off the cell for RFO1 and RFO2.
        """
        # Turn off both RF
        self.get_logger().info("Set RFO1 to off")
        self.__root.send_command("RFO1:STAT OFF")
        self.get_logger().info("Set MOD1 to off")
        self.__root.send_command("MOD1:STAT OFF")
        self.get_logger().info("Set RFO2 to off")
        self.__root.send_command("RFO2:STAT OFF")
        self.get_logger().info("Set MOD2 to off")
        self.__root.send_command("MOD2:STAT OFF")

    def set_cell_band(self, band):
        """
        Set the cell band

        :type band: int
        :param band: Cell Band to be set
        """
        self.get_logger().info("Set cell band : %s" % band)
        self.__root.send_command("FREQ:BAND %s" % band)

    def set_mcc(self, code):
        """
        Sets the mobile country code
        :type code: integer
        :param code: the Mobile Country Code to set.
        An integer from 0 to 999.
        """
        self.get_logger().info("Set MCC : %s" % code)
        self.__root.send_command("BSE:CONF:NAS:MCC %s" % code)

    def set_mnc(self, code):
        """
        Sets the mobile network code
        :type code: integer
        :param code: the Mobile Network Code to set.
        An integer from 0 to 99 for other bands.
        """
        self.get_logger().info("Set MNC : %s" % code)
        # Retrieve mnc length
        mnc_len = len(str(code))
        if mnc_len > 1:
            # Set MNC length
            self.__root.send_command("BSE:CONFig:NAS:MNCDigits %s" % mnc_len)
        # Set MNC code
        self.__root.send_command("BSE:CONF:NAS:MNC %s" % code)

    def set_cell_setup(self, cell_id):
        """
        Set cell setup

        :param cell_id : A-cell or B-cell
        :type cell_id: str ("A" or "B" are expected values)
        """
        self.__cell_name = cell_id
        self.get_logger().info("Set cell setup : %s" % self.__cell_name)
        self.__root.send_command("BSE:CELL:SEL %s" % self.__cell_name)

    def set_signal_mode(self, mode):
        """
        Sets the PXT operation Mode

        :type mode : str
        :param mode : Operation mode to set BSE or SA
        """
        self.get_logger().info("Set signal mode : %s" % mode)
        self.__root.send_command("SIGN:MODE %s" % mode)

    def set_cell_power(self, power_rf1, power_rf2=None):
        """
        Sets the cell power amplitude

        :type power_rf1, power_rf2 : double
        :param power_rf1, power_rf2 : cell power to set on RF1 and RF2
        """

        if power_rf2 == "" or power_rf2 is None:
            self.get_logger().info("Set cell power on RF1 : %s" % power_rf1)
            self.__root.send_command("AMPL:RF1 %s" % power_rf1)

        else:
            self.get_logger().info("Set cell power on RF1 : %s" % power_rf1)
            self.__root.send_command("AMPL:RF1 %s" % power_rf1)
            self.get_logger().info("Set cell power on RF2 : %s" % power_rf2)
            self.__root.send_command("AMPL:RF2 %s" % power_rf2)

    def set_frequency_setting_method(self):
        """
        Sets the frequency setting method

        .. warning:: EARFCN [hard coded] or FREQ [not used for now]
        """
        self.get_logger().info("Set frequency setting method : EARFCN")
        self.__root.send_command("FREQ:SMET EARFCN")

    def set_downlink_earfcn(self, channel):
        """
        Sets the downlink E-ARFCN

        Terms:
        - E-ARFCN: E-UTRA Absolute Radio Frequency Channel Number
        - E-UTRA: Evolved Universal Terrestrial Radio Access

        :type: int
        :param channel: downlink channel to be set
        """
        self.get_logger().info("Set downlink E-ARFCN : %i" % channel)
        self.__root.send_command("FREQ:EARF:DL %i" % channel)

    def set_random_lac(self):
        """
        Sets a random local area code (LAC)

        .. warning:: the lac list to avoid, but empty for Agilent E6621A
        It is not supported by this equipment but this function is needed
        to be registered on network simulator so code is empty.
        """
        self.get_logger().info("set_random_lac function is stubbed for AgilentE6621A")
        pass

    def set_cell_id(self, cellid):
        """
        Set the cell ID

        :type cellid: int
        :param cellid: Cell ID to be set
        """
        self.get_logger().info("Set cell ID : %s" % cellid)
        self.__root.send_command("BSE:CONF:PHY:CELL:ID %s" % cellid)

    def get_cell_channel_bandwidth(self):
        """
        Get the cell  channel bandwidth
        :return: bandwidth: Cell Band to get
        :type bandwidth: str
        """
        self.get_logger().info("Getting the Cell Channel Bandwidth")
        return self.__root.query_command("BSE:CONF:PROF?")

    def set_cell_channel_bandwidth(self, bandwidth):
        """
        Set the cell channel bandwidth
        :param bandwidth: Cell Band to be set
        :type bandwidth: str(20MHz for example)
        """
        self.get_logger().info("Set Cell Channel Bandwidth : %sMHz" % bandwidth)
        self.__root.send_command("BSE:CONF:PROF %sMHz" % bandwidth)

    def get_rrc_state(self):
        """
        Returns the rrc state
        Passed for AgilentE6621A as not sure if needed or supported.
        :rtype: str
        :return: the rrc state
        .. warning:: TODO, to check if it's supported by AgilentE6621A
        """
        self.get_logger().info("Get RRC STATE")
        return self.__root.query_command("BSE:STATus:ACELL?")

    def check_rrc_state_before_timeout(self, state, timeout):
        """
        Check for the given rrc state before given timeout.

        :param state: The state to obtain IDLE|CONNECTED
        :type state: str

        :param timeout: The timeout to wait in seconds
        :type timeout: int

        :raise AcsConfigException: If given state parameter is incorrect
        :raise TestEquipmentException: If unable to reach <state> on time
        """
        timeout = int(timeout)
        # Note: Possible state in PXT OFF | IDLE | CON | REG | LOOP | REL | UNAV
        if state not in ["IDLE", "CONNECTED"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                "Given state parameter (%s) is not valid" % str(state))

        self.__root.get_logger().error("Check for %s RRC state before %d seconds" % (state, timeout))

        while timeout > 0:
            rrc_state = str(self.get_rrc_state())
            self.__root.get_logger().debug("Current RRC state is: %s seconds" % rrc_state)
            if rrc_state in state:
                state_reached = True
                break
            time.sleep(1)
            timeout -= 1

        if not state_reached:
            err_msg = "Error while attempting to reach %s RRC state on DAU unit !" % state
            self.__root.get_logger().error(err_msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, err_msg)
        else:
            msg = "State %s reached on time !" % state
            self.__root.get_logger().info(msg)

    def get_antennas_number(self):
        """
        Returns the number of antennas
        The number of antennas dictates the Transmission mode availablility.
        :rtype: int
        :return: the number of antennas
        """
        output = self.__root.query_command("BSE:CONF:PHY:CELL:NCT?")
        self.get_logger().info("Get the number of antennas: %s" % output)
        return output

    def set_antennas_number(self, antennas_number):
        """
        sets the number of antennas wanted
        The number of antennas dictates the Transmission mode availability.
        :param antennas_number: the number of antennas to set
        """
        self.__root.send_command("BSE:CONF:PHY:CELL:NCT %s" % antennas_number)
        self.get_logger().info("%s antennas are activated" % antennas_number)

    def get_transmission_mode(self):
        """
        Returns the transmission mode
        :rtype: str
        :return: the transmission mode
        """

        output = self.__root.query_command("BSE:CONFig:RRC:TMode?")
        self.get_logger().info("Get the transmission mode: %s" % output)
        return output

    def set_transmission_mode(self, transmission_mode):
        """
        sets the number of antennas wanted
        :param transmission_mode: the value of transmission mode to set
        :type transmission_mode: str (ex: TM1, TM2, TM3, TM4 ...)
        """
        self.__root.send_command("BSE:CONF:RRC:TM %s" % transmission_mode)
        self.get_logger().info("Set the transmission mode to: %s" % transmission_mode)

    def get_dl_i_mcs(self):
        """
        Returns the value of DL I_MCS
        :rtype: int
        :return: the value of DL I_MCS
        """
        output = self.__root.query_command("BSE:CONF:PHY:DL:RESOU:ALLO:IMCS?")
        self.get_logger().info("Get the value of DL I_MCS: %s" % output)
        return output

    def set_dl_i_mcs(self, dl_i_mcs):
        """
        sets the DL I_MCS
        :param dl_i_mcs: the value of DL I_MCS to set,
                    this parameter can be adjusted in order to reach the max throughput
        :type dl_i_mcs: int (ex: 22)
        """
        self.__root.send_command("BSE:CONF:PHY:DL:RESOU:ALLO:IMCS %s" % dl_i_mcs)
        self.get_logger().info("Set the DL I_MCS to: %s " % dl_i_mcs)

    def get_ul_i_mcs(self):
        """
        Returns the value of UL I_MCS
        :rtype: int
        :return: the value of UL I_MCS
        """
        output = self.__root.query_command("BSE:CONF:PHY:UL:GRAN:IMCS?")
        self.get_logger().info("Get the value of UL I_MCS: %s" % output)
        return output

    def set_ul_i_mcs(self, ul_i_mcs):
        """
        sets the UL I_MCS
        :param ul_i_mcs: the value of UL I_MCS to set,
                    this parameter can be adjusted in order to reach the max throughput
        :type ul_i_mcs: int (ex: 22)
        """
        self.__root.send_command("BSE:CONF:PHY:UL:GRAN:IMCS %s" % ul_i_mcs)
        self.get_logger().info("Set the I_MCS to: %s " % ul_i_mcs)

    def get_dl_rb_size(self):
        """
        Returns DL RB Size value
        :rtype: int
        :return: the DL resource Alloc: RB Size, the values returned can be:
                1.4 MHz = 6
                3 MHz = 15
                5 MHz = 25
                10 MHz = 50
                15 MHz = 75
                20 MHz = 100
        """
        output = self.__root.query_command("BSE:CONF:PHY:DL:RESOU:ALLO:RB:SIZE?")
        self.get_logger().info("Get DL resource Alloc: DL RB Size value is %s" % output)
        return output

    def set_dl_rb_size(self, rb_size):
        """
        sets DL RB Size value
        :param trs_mode: the value of DL RB Size
        :type trs_mode: int, the possible values are:
                1.4 MHz = 6
                3 MHz = 15
                5 MHz = 25
                10 MHz = 50
                15 MHz = 75
                20 MHz = 100
        """
        self.__root.send_command("BSE:CONF:PHY:DL:RESOU:ALLO:RB:SIZE %s" % rb_size)
        self.get_logger().info("the DL resource Alloc: set DL RB Size value to %s " % rb_size)

    def get_ul_rb_size(self):
        """
        Returns DL RB Size value
        :rtype: int
        :return: the DL resource Alloc: RB Size, the values returned can be:
                1.4 MHz = 6
                3 MHz = 15
                5 MHz = 25
                10 MHz = 50
                15 MHz = 75
                20 MHz = 100
        """
        output = self.__root.query_command("BSE:CONF:PHY:UL:GRAN:RB:SIZE?")
        self.get_logger().info("Get UL resource Alloc: UL RB Size value is %s" % output)
        return output

    def set_ul_rb_size(self, rb_size):
        """
        sets DL RB Size value
        :param trs_mode: the value of DL RB Size
        :type trs_mode: int, the possible values are:
                1.4 MHz = 6
                3 MHz = 15
                5 MHz = 25
                10 MHz = 50
                15 MHz = 75
                20 MHz = 100
        """
        self.__root.send_command("BSE:CONF:PHY:UL:GRAN:RB:SIZE %s" % rb_size)
        self.get_logger().info("the UL resource Alloc: set UL RB Size value to %s " % rb_size)

    def get_type0_bitmap(self):
        """
        Returns Type 0 Bitmap
        :rtype: int
        :return: theType0 Bitmap(value for channel Bandwidth), the values returned can be:
                1.4 MHz = 63 (0x3F)
                3 MHz = 255 (0xFF)
                5 MHz =8191 (0x1FFF)
                10 MHz =131071 (0x1FFFF)
                15 MHz = 524287 (0x7FFFF)
                20 MHz =33554431 (0x1FFFFFF)
        """
        output = self.__root.query_command("BSE:CONF:PHY:DL:RESOU:ALLO:TYPE0:BITM?")
        self.get_logger().info("Get DL resource Alloc: Type0 bitmap value is %s" % output)

    def set_type0_bitmap(self, type0_bitmap):
        """
        sets DL RB Size value
        :param trs_mode: the value of DL RB Size
        :type trs_mode: int, the possible values are:
                1.4 MHz = 63 (0x3F)
                3 MHz = 255 (0xFF)
                5 MHz =8191 (0x1FFF)
                10 MHz =131071 (0x1FFFF)
                15 MHz = 524287 (0x7FFFF)
                20 MHz =33554431 (0x1FFFFFF)
        """
        self.__root.send_command("BSE:CONF:PHY:DL:RESOU:ALLO:TYPE0:BITM %s" % type0_bitmap)
        self.get_logger().info("the DL resource Alloc: set Type0 bitmap value to %s " % type0_bitmap)

    def get_ul_grant_mode(self):
        """
        Returns the UL GRANT MODE
        :rtype: str
        :return: the ul grant mode (AUTO or FIXEDMAC)
        """
        output = self.__root.query_command("BSE:CONF:PHY:UL:GRAN:MODE?")
        self.get_logger().info("Get the UL Grant Mode: %s" % output)
        return output

    def set_ul_grant_mode(self, ul_grant_mode):
        """
        sets the number of transmission mode wanted
        :param trs_mode: the value of UL Grant mode to set
        :type trs_mode: str (the ul grant mode: AUTO or FIXEDMAC)
        """
        self.get_logger().info("Set the UL Grant mode to: %s" % ul_grant_mode)
        self.__root.send_command("BSE:CONF:PHY:UL:GRAN:MODE %s" % ul_grant_mode)

    def configure_basic_lte_cell_parameters(self,
                                        band,
                                        earfcn,
                                        mimo,
                                        power1,
                                        power2,
                                        mcc=1,
                                        mnc=1):
        """
        Configure basic LTE cell parameters like cell, band,
        earfcn, cell power.
        :type band: str
        :param band: : the current band.
        :type earfcn: int
        :param earfcn: : the current E-ARFCN corresponding to the band.
        :type mimo: boolean
        @ param mimo: If mimo is not used, only RFO1 / MOD1 are on
        If mimo is used, RFO1 / MOD1 and RF02 / MOD2 are on
        :type power1: integer
        :param power1: : the current cell RF1 power in dBm.
        :type power2: integer
        :param power2: : the current cell RF2 power in dBm.
        :type mcc: integer
        :param mcc: the Mobile Network Code to set.
        An integer from 0 to 99 for other bands.
        :type mnc: integer
        :param mnc: the Mobile Network Code to set.
        An integer from 0 to 99 for other bands.
        """
        # Set cell band and BCCH ARFCN
        self.set_cell_band(band)
        self.set_downlink_earfcn(earfcn)
        # Set cell power
        if (mimo and
           (power2 != "") and
           (power2 is not None)):

            self.set_cell_power(power1, power2)

        else:
            self.set_cell_power(power1)
        # Set Mobile Country Code
        self.set_mcc(mcc)
        # Set Mobile Network Code
        self.set_mnc(mnc)

    def set_lte_configuration(self,
                              antennas_number,
                              bandwidth,
                              transmission_mode,
                              type0_bitmap,
                              ul_grant_mode,
                              dl_rb_size,
                              dl_i_mcs,
                              ul_rb_size,
                              ul_i_mcs,
                              mimo,
                              carrier_aggregation=False):
        """
        Sets the main LTE parameters
        :type antennas_number: str
        :param antennas_number: number of antennas used on the base station and the DUT
        :type bandwidth: str
        :param bandwidth: LTE bandwidth
        :type transmission_mode: str
        :param transmission_mode: LTE transmission mode
        :type type0_bitmap: integer
        :param type0_bitmap: DL Resource Alloc - Type#0 Bitmap
        :type ul_grant_mode: str
        :param ul_grant_mode: the ul grant mode (AUTO or FIXEDMAC)
        :type dl_rb_size: str
        :param dl_rb_size: the value of DL RB Size
        :type dl_i_mcs: integer
        :param dl_i_mcs: the value of DL I_MCS to set,
                    this parameter can be adjusted in order to reach the max throughput
        :type ul_rb_size: integer
        :param ul_rb_size: the value of UL RB Size
        :type ul_i_mcs: integer
        :param ul_i_mcs: the value of UL I_MCS to set,
                    this parameter can be adjusted in order to reach the max throughput
        :type mimo: bool
        :param mimo: mimo, not used for AgilentE6621A
        :type carrier_aggregation: bool
        :param carrier_aggregation: Use of Carrier Aggregation or not
        """
        # Set antennas number
        self.set_antennas_number(antennas_number)
        # Set the Bandwidth value
        self.set_cell_channel_bandwidth(bandwidth)
        # Set the Transmission mode
        self.set_transmission_mode(transmission_mode)

        if int(transmission_mode.split("TM")[1]) > 2:
            # Set the DL TYPE0_BITMAP for TM3 and TM4 transmission mode
            self.set_type0_bitmap(type0_bitmap)
        else:
            # Set the DL RB size
            self.set_dl_rb_size(dl_rb_size)

        # Set the DL I_MCS
        self.set_dl_i_mcs(dl_i_mcs)
        # Set Subframe #5 to Max Throughput
        if antennas_number in "2" and bandwidth in "20":
            self.__root.send_command("BSE:CONFig:PHY:DL:RESOUrce:ALLOc:CONTrol:SFRame5 MAXTh")
            self.get_logger().info("Subframe 5 is used for signalling in order to reach max throughput")
        # Set the UL RB Size
        self.set_ul_rb_size(ul_rb_size)
        # Set the UL I_MCS
        self.set_ul_i_mcs(ul_i_mcs)
        self.set_ul_grant_mode(ul_grant_mode)

        # Carrier Aggregation not yet supported
        if carrier_aggregation:
            err_msg = "Carrier Aggregation will not be implemented on PXT (buy an Agilent UXM)"
            raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED, err_msg)

    def keep_rrc_connection(self, enabled):
        """
        Selects whether the RRC connection shall be kept or released after attach.

        :param enabled: keep connection if this parameter is set to 'True'.
                        Otherwise the RRC state will be back to 'IDLE'
                        after cell attach process.
        :type enabled: bool
        """
        mode = "release"
        if enabled:
            mode = "keep"
        self.get_logger().warning("This equipment has not been designed to "
                                + mode + " RRC connection after attach "\
                                "process automatically")

    def set_cell_ratio(self, ratio):
        """
        Set the cell ratio for the active cell

        :type ratio: int
        :param ratio: the cell ratio is an int in the range [0,101]
        """
        if not (type(ratio) == int and ratio in range(101)):
            raise TestEquipmentException("ratio value should be between 0 and 1")
        self.get_logger().info("Set cell %s ratio to %d%%" % (self.__cell_name, ratio))
        self.__root.send_command("CELLSetup:%sCELl:RATio %d" % (self.__cell_name, ratio))

        data = self.__root.query_command("CELLSetup:%sCELl:RATio?" % self.__cell_name)
        if str(data) != str(ratio):
            self.get_logger().warning("set_cell_ratio command has not worked")

    def send_custom_message(self, message_number):
        """
        send custom message

        :type message_number: either the message as str or an int:
        :param message_number: the custom message number to send
            this message should be one of
            - Measurement Report
            - Meas Report B13
        """
        CUSTOM_MESSAGES = {"MEASUREMENT REPORT": 1, "MEAS REPORT B13": 2}
        if message_number.upper() in CUSTOM_MESSAGES:
            message_number = CUSTOM_MESSAGES[message_number.upper()]
        elif type(message_number) == int:
            pass
        else:
            raise TestEquipmentException("custom_message should be a valid message string or an integer")
        self.__root.send_command("BSE:FUNCtion:CUSTom:MESSAge%d:SEND" % message_number)

    def send_handover_message(self, message_number):
        """
        send handover message

        :type message_number:
        :param message_number:
            - DL Info CS Notify
            - B1 RRC Handover A to B
            - B3 RRC Handover A to B
            - B13 RRC Handover A to B
            - B17 RRC Handover A to B
            - PS HO To WCDMA
            - Blind HO
            - SRVCC To WCDMA
        """
        HANDOVER_MESSAGES = {"DL INFO CS NOTIFY": 1,
                           "B1 RRC HANDOVER A TO B": 2,
                           "B1 RRC HANDOVER B TO A": 2,
                           "B3 RRC HANDOVER A TO B": 3,
                           "B3 RRC HANDOVER B TO 1": 3,
                           "B13 RRC HANDOVER A TO B": 4,
                           "B13 RRC HANDOVER B TO A": 4,
                           "B17 RRC HANDOVER A TO B": 5,
                           "B17 RRC HANDOVER B TO A": 5,
                           "PS HO TO WCDMA": 6,
                           "BLIND HO": 7,
                           "SRVCC TO WCDMA": 8}
        if message_number.upper() in HANDOVER_MESSAGES:
            message_number = HANDOVER_MESSAGES[message_number.upper()]
        elif type(message_number) == int:
            pass
        else:
            raise TestEquipmentException("message number should be a valid message string or an integer")
        self.__root.send_command("BSE:FUNCtion:HANDOver:MESSAge%d:SEND" % message_number)


    def set_rrc_connection_inact_timer(self, timer_len):
        '''
        Set the inactivity timer after which an RRC Connection Release will be sent to the DUT

        :type: int
        :param: timer duration in seconds

        :rtype: None
        '''
        if timer_len  >= 1 and timer_len <= 2000:
            # Set inactivity status monitor state to ON
            self.__root.send_command("BSE:CONF:RRC:ITIM:STAT ON")
            # Set the inactivity status monitor period (in sec) before an RRC Connection Release is sent to the DUT
            self.__root.send_command("BSE:CONF:RRC:ITIM:LENG %s" % timer_len)
        else:
            self.get_logger.info("Inactive Timer must be integer between 1 and 2000")
            raise TestEquipmentException("message number should be a valid message string or an integer")

    def set_rrc_connection_timer(self, timer_len):
        '''
        Set the check interval for connection status monitoring

        :type: int
        :param: timer duration in seconds

        :rtype: None
        '''
        if timer_len  >= 2 and timer_len <= 60:
            # Set cell connection status monitor state to ON
            self.__root.send_command("BSE:CONF:RRC:CTIM:STAT ON")
            #  Set the cell connection status monitor check interval (in sec)
            self.__root.send_command("BSE:CONF:RRC:CTIM:LENG %s" % timer_len)
        else:
            self.get_logger.info("Connection Timer must be integer between 2 and 60")
            raise TestEquipmentException("message number should be a valid message string or an integer")

