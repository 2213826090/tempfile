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
:summary: Cell 4G implementation for R&S CMW500 using visa interface
:since: 16/09/2014
:author: jduran4x
"""
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ICell4G import ICell4G
from acs_test_scripts.TestStep.Utilities.Visa import VisaObject
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Tech4G.Data4G import Data4G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Tech4G.Messaging4G import Messaging4G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmw500.Tech4G.VoiceCall4G import VoiceCall4G
from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.TestEquipmentException import TestEquipmentException
import math
import random
import time


class Cell4G(ICell4G, VisaObject):

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
        self.__data = Data4G(visa)
        self.__messaging = Messaging4G(visa)
        self.__voicecall = VoiceCall4G(visa)
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

    def get_messaging(self):
        """
        Access to LTE data interface.
        :rtype: IData4G
        :return: the LTE data object.
        """
        return self.__messaging

    def get_voice_call(self):
        """
        Access to 4G voice call interface.
        :rtype: IVoiceCall4G
        :return: the 4G voice call object.
        """
        return self.__voicecall

    def set_cell_on(self, mimo=None):
        """
        Turns on the cell.

        :type mimo: boolean
        :param mimo: not used for CMW500 as SISO/SIMO/MIMO parameters should be configured BEFORE setting cell on

        :raise TestEquipmentException: In case or timeout on cell activation
        """
        # timer value (300 secs) to allow for the cell in the CMW500 to initialize and turn on...
        timer = 300
        cell_enabled = False
        cell_status = self._visa.query_command("SOUR:LTE:SIGN:CELL:STAT?")
        if cell_status != "OFF":
            self.get_logger().info("Cell is currently enabled... disabling")
            self._visa.send_command("SOUR:LTE:SIGN:CELL:STAT OFF")

        # Now enable the cell emulator
        self.get_logger().info("Enabling cell and RF output...")
        self._visa.send_command("SOUR:LTE:SIGN:CELL:STAT ON")
        # Now wait until the cell is up and running before returning control...
        while timer > 0:
            if self._visa.query_command("SOUR:LTE:SIGN:CELL:STAT?") == "ON":
                cell_enabled = True
                break

            # time.sleep(1)
            time.sleep(5)  # There is no need to query so many times for a time consuming operation like this...
            timer -= 5
        if not cell_enabled:
            err_msg = "Error while activating the LTE cell!!!"
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
        imsi = self._visa.query_command("SENSe:LTE:SIGN:UESinfo:IMSI?")
        imsi = imsi.replace('"', '')

        return imsi == dut_imsi

    def get_cell_status(self):
        """
        Turns on the cell.

        :rtype:str
        :return: Cell state (ON|OFF)
        """
        return self._visa.query_command("SOUR:LTE:SIGN:CELL:STAT?")

    def set_cell_off(self):
        """
        Turns off the cell.

        [JPHC 27/07/2012]
        Just disable cell generation in the CMW500, no difference between SISO and MIMO configurations...
        """
        # Turn off both RF
        self.get_logger().info("Disabling cell and RF output...")
        self._visa.send_command("SOUR:LTE:SIGN:CELL:STAT OFF")
        self.__is_cell_off = True

    def set_cell_band(self, band):
        """
        Set the cell band

        :param band: Cell Band to be set
        :type band: int
        """
        self.get_logger().info("Set cell band : %s" % band)
        self._visa.send_command("CONF:LTE:SIGN:BAND OB%s" % band)

    def set_secondary_carrier_band(self, band):
        """
        Set the cell band on Secondary Component Carrier

        :param band: Cell Band to be set
        :type band: int
        """
        self.get_logger().info("Set cell band on Secondary Component Carrier: %s" % band)
        self._visa.send_command("CONF:LTE:SIGN:SCC:BAND OB%s" % band)

    def set_mcc(self, code):
        """
        Sets the mobile country code
        :type code: integer
        :param code: the Mobile Country Code to set.
        An integer from 0 to 999.
        """
        self.get_logger().info("Set MCC : %s" % code)
        self._visa.send_command("CONF:LTE:SIGN:CELL:MCC %s" % code)

    def set_mnc(self, code):
        """
        Sets the mobile network code
        :type code: integer
        :param code: the Mobile Network Code to set.
        An integer from 0 to 99 for other bands.
        """
        self.get_logger().info("Set MNC : %s" % code)
        self._visa.send_command("CONF:LTE:SIGN:CELL:MNC %s" % code)

    def set_cell_setup(self, cell_id):
        """
        Set cell setup

        :param cell_id : A-cell or B-cell
        :type cell_id: str ("A" or "B" are expected values) THIS ONE HAS NO TRANSLATION TO CMW500 WORLD!!!
        """
        self.get_logger().info("set_cell_setup function is stubbed for RsCmw500")

    def set_signal_mode(self, mode):
        """
        Sets the PXT operation Mode

        :type mode : str
        :param mode : Operation mode to set BSE or SA THIS ONE HAS NO TRANSLATION IN CMW500 WORLD!!!
        """
        self.get_logger().info("set_signal_mode function is stubbed for RsCmw500")

    def set_cell_power(self, power_rf1, power_rf2=None):
        """
        Sets the cell power amplitude

        :type power_rf1, power_rf2 : double
        :param power_rf1, power_rf2 : cell power to set on RF1 and RF2

        [JPHC 27/07/2012]
        This is quite different in CMW500... only power_rf1 value will be taken into account (for LTE it makes no sense to configure both antenna independently)
        However, it is possible to emulate PXT behavior if we know the used channel bandwidth to the CMW500 and computing the needed power density value
        Unfortunately, the CMW500 provides no SCPI command to query the currently configured ChBW...10MHz will be assumed by the function, but we definitely need
        to come back to this...
        """

        # First, we need to know the currently configured Sys Ch BW...assuming 10 MHz (50 PRBs)
        system_ch_bw = 50
        # Then we compute EPRE
        cell_epre = float(power_rf1) - 10.0 * math.log10(12.0 * float(system_ch_bw))

        self.get_logger().info("Set cell power : %s dBm/15kHz" % cell_epre)
        self._visa.send_command("CONF:LTE:SIGN:DL:RSEP:LEV %s" % cell_epre)
        total_ch_power_configured = self._visa.query_command("SENS:LTE:SIGN:DL:FCP?")
        self.get_logger().info("Configured total cell power : %s dBm" % total_ch_power_configured)

    def set_secondary_carrier_cell_power(self, cell_power):
        """
        Set the cell power on Secondary Component Carrier

        :type cell_power: float
        :param cell_power: Cell Power to be set
        """
        # First, we need to know the currently configured Sys Ch BW...assuming 10 MHz (50 PRBs)
        system_ch_bw = 50
        # Then we compute EPRE
        cell_epre = float(cell_power) - 10.0 * math.log10(12.0 * float(system_ch_bw))

        self.get_logger().info("Set cell power : %s dBm/15kHz" % cell_epre)
        self._visa.send_command("CONF:LTE:SIGN:DL:SCC:RSEP:LEV %s" % cell_epre)
        total_ch_power_configured = self._visa.query_command("SENS:LTE:SIGN:DL:SCC:FCP?")
        self.get_logger().info("Configured total cell power : %s dBm" % total_ch_power_configured)

    def set_frequency_setting_method(self):
        """
        Sets the frequency setting method

        .. warning:: EARFCN [hard coded] or FREQ [not used for now]
        CMW500 allows both EARFCN based and FREQ based configuration of the DL/UL channel without the need to specify operation mode (just adding units to the value used with the SCPI command)
        Therefore, modifications to support configuration of FREQ values are to be done in the corresponding setting function...
        """
        self.get_logger().info("set_frequency_setting_method function is stubbed for RsCmw500")

    def set_downlink_earfcn(self, channel):
        """
        Sets the downlink E-ARFCN

        Terms:
        - E-ARFCN: E-UTRA Absolute Radio Frequency Channel Number
        - E-UTRA: Evolved Universal Terrestrial Radio Access

        :param channel: downlink channel to be set
        :type: int
        """
        self.get_logger().info("Set downlink E-ARFCN : %i" % channel)
        self._visa.send_command("CONF:LTE:SIGN:RFS:CHAN:DL %i" % channel)

    def set_secondary_carrier_earfcn(self, channel):
        """
        Sets the downlink E-ARFCN of Secondary Component Carrier

        Terms:
        - E-ARFCN: E-UTRA Absolute Radio Frequency Channel Number
        - E-UTRA: Evolved Universal Terrestrial Radio Access

        :param channel: downlink channel to be set
        :type: int
        """
        self.get_logger().info("Set downlink E-ARFCN on Secondary Component Carrier: %i" % channel)
        self._visa.send_command("CONF:LTE:SIGN:RFS:SCC:CHAN:DL %i" % channel)

    def set_random_lac(self):
        """
        Sets a random local area code (LAC)

        .. warning:: the lac list to avoid, but empty for Agilent E6621A
        It is not supported by this equipment but this function is needed
        to be registered on network simulator so code is empty.
        Comment...
        [JPHC, 24/07/2012]
        R&S CMW500 supports configuration of the TAC... it is easy to randomize... but will need to create Random type object
        """
        tac_rnd = random.randint(0, 65535)
        self.get_logger().info("Set TAC to : %i" % tac_rnd)
        self._visa.send_command("CONF:LTE:SIGN:CELL:TAC %i" % tac_rnd)

    def set_cell_id(self, cellid):
        """
        Set the cell ID

        :param cellid: Cell ID to be set
        :type cellid: int
        """
        self.get_logger().info("Set cell ID : %s" % cellid)
        self._visa.send_command("CONFigure:LTE:SIGN:CELL:PCID %s" % cellid)

    def set_channel_bandwidth(self, carrier, bandwidth):
        """
        Set channel bandwidth of the specified carrier

        :type carrier: str
        :param carrier: Component Carrier to set
            - PCC : Principal Component Carrier
            - SCC : Secondary Component Carrier
        :type bandwidth: str
        :param bandwidth: Cell bandwidth to be set
        """

        # Primary or Secondary Carrier Component
        supported_carrier_list = ["PCC", "SCC"]
        if carrier not in supported_carrier_list:
            err_msg = "The carrier %s isn't in the supported_carrier_list : %s " % \
                (carrier, supported_carrier_list)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)
        elif carrier == "PCC":
            gpib_command = "CONF:LTE:SIGN:CELL:BAND:DL"
        elif carrier == "SCC":
            gpib_command = "CONF:LTE:SIGN:CELL:BAND:SCC:DL"

        # Configure System Channel Bandwidth
        supported_bandwidth_list = ["1.4", "3", "5", "10", "15", "20"]

        if bandwidth not in supported_bandwidth_list:
            err_msg = "The bandwidth %s isn't in the supported_bandwidth_list : %s " % \
                (bandwidth, supported_bandwidth_list)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)
        elif bandwidth == "1.4":
            self.get_logger().info("Configuring %s channel bandwidth to %s" % (carrier, bandwidth))
            self._visa.send_command("%s B014" % gpib_command)
        elif int(bandwidth) < 10:
            self.get_logger().info("Configuring %s channel bandwidth to %d" % (carrier, int(bandwidth)))
            # The bandwidth is multiplied by 10 to convert it from MHz to
            # a code understand by the equipment. See the doc of the function.
            self._visa.send_command("%s B0%d" % (gpib_command, (int(bandwidth) * 10)))
        elif int(bandwidth) >= 10:
            self.get_logger().info("Configuring %s channel bandwidth to %d" % (carrier, int(bandwidth)))
            # The bandwidth is multiplied by 10 to convert it from MHz to
            # a code understand by the equipment. See the doc of the function.
            self._visa.send_command("%s B%d" % (gpib_command, (int(bandwidth) * 10)))

    def set_cell_channel_bandwidth(self, bandwidth):
        """
        Set the cell channel bandwidth

        :type bandwidth: str
        :param bandwidth: Cell bandwidth to be set
        """
        self.set_channel_bandwidth("PCC", bandwidth)

    def set_secondary_carrier_channel_bandwidth(self, bandwidth):
        """
        Set the Secondary Componnent Carrier channel bandwidth

        :type bandwidth: str
        :param bandwidth: Cell bandwidth to be set
        """
        self.set_channel_bandwidth("SCC", bandwidth)

    def get_rrc_state(self):
        """
        Returns the rrc state

        :rtype: str
        :return: the rrc state
        """
        self.get_logger().info("Get RRC STATE")
        return self._visa.query_command("SENS:LTE:SIGN:RRCS?")

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
        # Note: Possible state in CMW500 IDLE | CONNected
        if state not in ["IDLE", "CONNECTED"]:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                     "Given state parameter (%s) is not valid" % str(state))

        self.get_logger().error("Check for %s RRC state before %d seconds" % (state, timeout))
        state_reached = False
        while timeout > 0:
            rrc_state = str(self.get_rrc_state())
            self.get_logger().debug("Current RRC state is: %s seconds" % rrc_state)
            if rrc_state in state:
                state_reached = True
                break
            time.sleep(1)
            timeout -= 1

        if not state_reached:
            err_msg = "Error while attempting to reach %s RRC state on DAU unit !" % state
            self.get_logger().error(err_msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, err_msg)
        else:
            msg = "State %s reached on time !" % state
            self.get_logger().info(msg)

    def configure_basic_lte_cell_parameters(self,
                                            band,
                                            earfcn,
                                            mimo,
                                            power1,
                                            power2,
                                            mcc=1,
                                            mnc=1):
        """
        Configure basic LTE cell parameters like cell service, band,
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
        if ((mimo) and
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
        :param mimo: Use mimo or not, used for scenario and antenna numbers settings
        :type carrier_aggregation: bool
        :param carrier_aggregation: Use of Carrier Aggregation or not, used for scenario settings
        """
        # Specifying General Settings (previously done on set_cell_on (!)) (CMW500 specific)
        self._set_general_settings()

        # 1- Set scenario, depends on mimo and transmission mode (CMW500 specific)
        self._set_scenario(mimo, transmission_mode, carrier_aggregation)

        # store locally for next usage
        self.__antennas_number = int(antennas_number)
        # 2- Set transmission mode, need antennas number
        self.set_transmission_mode(transmission_mode)

        # check mimo and antennas number
        if mimo and self.__antennas_number < 2:
            err_msg = "Incompatibility between MIMO and antennas number (%s)" % self.__antennas_number
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)
        else:
            # 3- Set antennas number, use mimo to avoid misbehavior
            self.set_antennas_number(self.__antennas_number)

        # Set the Bandwidth value
        self.set_cell_channel_bandwidth(bandwidth)
        # Set the connection setup (same GPIB command for CMW500)
        self._set_lte_connection_setup(dl_rb_size, dl_i_mcs, ul_rb_size, ul_i_mcs)

    def set_lte_tdd_configuration(self,
                                  antennas_number,
                                  mimo):
        """
        Sets the main LTE parameters
        :type antennas_number: str
        :param antennas_number: number of antennas used on the base station and the DUT
        :type mimo: bool
        :param mimo: Use mimo or not, used for scenario and antenna numbers settings
        """
        if int(antennas_number) == 1:
            transmission_mode = "TM1"
        else:
            transmission_mode = "TM3"
        # 1- Set scenario, depends on mimo and transmission mode (CMW500 specific)
        self._set_scenario(mimo, transmission_mode)

        # store locally for next usage
        self.__antennas_number = int(antennas_number)
        # 2- Set transmission mode, need antennas number
        self.set_transmission_mode(transmission_mode)

        # check mimo and antennas number
        if mimo and self.__antennas_number < 2:
            err_msg = "Incompatibility between MIMO and antennas number (%s)" % self.__antennas_number
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)
        else:
            # 3- Set antennas number, use mimo to avoid misbehavior
            self.set_antennas_number(self.__antennas_number)

    def set_secondary_carrier_configuration(self,
                                            band,
                                            earfcn,
                                            bandwidth,
                                            cell_power,
                                            dl_rb_size,
                                            dl_i_mcs):
        """
        Sets the Secondary Component Carrier parameters
        :type band: str
        :param band: : the current band.
        :type earfcn: int
        :param earfcn: : the current E-ARFCN corresponding to the band.
        :type bandwidth: int
        :param bandwidth: the current bandwidth.
        :type cell_power: float
        :param cell_power: the SCC cell power
        :type dl_rb_size: str
        :param dl_rb_size: the value of DL RB Size
        :type dl_i_mcs: integer
        :param dl_i_mcs: the value of DL I_MCS to set,
                    this parameter can be adjusted in order to reach the max throughput
        """
        self.set_secondary_carrier_band(band)
        self.set_secondary_carrier_earfcn(earfcn)
        self.set_secondary_carrier_channel_bandwidth(bandwidth)
        self._set_secondary_carrier_connection_setup(dl_rb_size, dl_i_mcs)
        self.set_secondary_carrier_cell_power(cell_power)

    def swap_primary_and_secondary_carrier_settings(self):
        """
        Initiates a swap of PCC and SCC settings
        """
        self.get_logger().info("Swap PCC and SCC configuration")
        self._visa.send_command("CONFigure:LTE:SIGN:CONNection:SCC:SEXecute")

    def set_secondary_carrier_state(self, state):
        """
        Controls the state of the Secondary Component Carrier

        :type state: str
        :param state: State to set :
            OFF: Switch SCC off
            ON: Switch SCC on
            RRCadd: Add SCC RRC connection
            MACactivate: Activate MAC for the SCC
            MACDeactivat: Deactivate MAC for the SCC
            RRCDelete: Delete SCC RRC connection
        """
        supported_state_list = ["OFF", "ON", "RRCadd", "MACactivate", "MACDeactivat", "RRCDelete"]
        if state not in supported_state_list:
            err_msg = "The state %s isn't in the supported_state_list : %s " % \
                (state, supported_state_list)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)
        else:
            self.get_logger().info("Set Secondary Carrier Component state to : %s" % state)
            self._visa.send_command("CALL:LTE:SIGN:SCC:ACTion %s" % state)

    def get_secondary_carrier_state(self):
        """
        Gets the state of the Secondary Component Carrier

        :rtype: str
        :return: SCC state :
            OFF: Switch SCC off
            ON: Switch SCC on
            RRCadd: Add SCC RRC connection
            MACactivate: Activate MAC for the SCC
            MACDeactivat: Deactivate MAC for the SCC
            RRCDelete: Delete SCC RRC connection
        """
        translation_dict = {"RRC": "RRCadd", "MAC": "MACactivate", "MACD": "MACDeactivat", "RRCD": "RRCDelete"}
        scc_state = self._visa.query_command("FETCh:LTE:SIGN:SCC:STATe?")
        if scc_state in translation_dict:
            scc_state = translation_dict[scc_state]
        return scc_state

    def check_secondary_carrier_state_before_timeout(self, state, timeout):
        """
        Checks the state of the Secondary COmponent Carrier

        :type state: str
        :param state: State to reach :
            OFF: Switch SCC off
            ON: Switch SCC on
            RRCadd: Add SCC RRC connection
            MACactivate: Activate MAC for the SCC
            MACDeactivat: Deactivate MAC for the SCC
            RRCDelete: Delete SCC RRC connection
        :type timeout: int
        :param timeout: The timeout to wait in seconds
        """
        supported_state_list = ["OFF", "ON", "RRCadd", "MACactivate", "MACDeactivat", "RRCDelete"]
        if state not in supported_state_list:
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, "Given SCC state parameter (%s) is not valid" % str(state))

        self.get_logger().info("Check for %s SCC state before %d seconds" % (state, timeout))
        state_reached = False
        while timeout > 0:
            scc_state = str(self.get_secondary_carrier_state())
            self.get_logger().debug("Current SCC state is: %s seconds" % scc_state)
            if scc_state in state:
                state_reached = True
                break
            time.sleep(1)
            timeout -= 1

        if not state_reached:
            err_msg = "%s SCC state not reached in time!" % state
            self.get_logger().error(err_msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, err_msg)
        else:
            msg = "SCC state %s reached on time !" % state
            self.get_logger().info(msg)

    def keep_rrc_connection(self, enabled):
        """
        Selects whether the RRC connection shall be kept or released after attach.

        :param enabled: keep connection if this parameter is set to 'True'.
                        Otherwise the RRC state will be back to 'IDLE'
                        after cell attach process.
        :type enabled: bool
        """
        if not self.__is_cell_off:
            # cell is ON : we are not able to keep the RCC connection.
            raise TestEquipmentException(TestEquipmentException.PROHIBITIVE_BEHAVIOR,
                                         "Cannot change equipment behavior when cell is ON "
                                         "(RRC connection state)")
        rrc_state = "kept"
        if enabled:
            self.__rrc_conn_state = "ON"
        else:
            rrc_state = "released"
            self.__rrc_conn_state = "OFF"
        self.get_logger().info("RCC connection will be %s after attach process"
                               % rrc_state)

    def set_antennas_number(self, antennas_number):
        """
        Sets the number of reception antennas wanted
        The number of antennas dictates the Transmission mode availability.
        :param antennas_number: the number of antennas to set
        :type antennas_number: int

        .. note::  For MIMO, RF1C connector is to be used for main DUT antenna
                and RF3C connector is to be used for aux UE antenna

        """
        self.get_logger().debug("#### Set Antennas Number ####")
        transmit_antennas = "ONE"
        if int(antennas_number) < 2:
            self.get_logger().info("No need to change antenna configuration (single antenna)")
            return
        else:
            if antennas_number == 2:
                transmit_antennas = "TWO"
            elif antennas_number == 4:
                transmit_antennas = "FOUR"
            else:
                err_msg = "The antennas number %d isn't supported" % antennas_number
                raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)

        # Use x transmit antennas (MIMO 2x2 or 4x2)
        self.get_logger().info("Set antennas number to " + transmit_antennas)
        self._visa.send_command("CONFigure:LTE:SIGN:CONNection:NENBantennas " + transmit_antennas)

        # Enable and specify static channel model for MIMO 2x2.
        self.get_logger().debug("Enable static channel model for MIMO 2x2.")
        self._visa.send_command("CONFigure:LTE:SIGN:CONNection:SCHModel:ENABle ON")
        self.get_logger().debug("Specify static channel model for MIMO 2x2.")
        self._visa.send_command("CONFigure:LTE:SIGN:CONNection:SCHModel 0.9,0,45,0.1,45,0")

        if transmit_antennas == "FOUR":
            # Specify static channel model for MIMO 4x2.
            self._visa.send_command("CONF:LTE:SIGN:CONN:SCHM:MIMO42 0.1,0,0.2,0,0.3,0,0.4,270,0.4,270,"
                                    "0.3,90,0.2,90,0.1,180")

    def set_transmission_mode(self, transmission_mode):
        """
        sets the number of antennas wanted
        :param transmission_mode: the value of transmission mode to set
        :type transmission_mode: str (ex: TM1, TM2, TM3, TM4 ...)

        .. warning:: This method use antennas number for CMW500

        :rtype: None
        """
        self.get_logger().debug("#### Configure Transmission Mode ####")
        if transmission_mode == "TM1":
            if self.__antennas_number == 1:
                # Select Single Input Single Output as transmission scheme.
                self.get_logger().info("Select SISO as transmission scheme.")
                self._visa.send_command("CONF:LTE:SIGN:CONN:TSCH SISO")
            else:
                # Select Single Input Multiple Output (Rx diversity) as transmission scheme.
                self.get_logger().info("Select SIMO as transmission scheme.")
                self._visa.send_command("CONF:LTE:SIGN:CONN:TSCH SIMO")
        elif transmission_mode == "TM3":
            # Select open loop spatial multiplexing as transmission scheme.
            self.get_logger().info("Select OLSMultiplex as transmission scheme.")
            self._visa.send_command("CONF:LTE:SIGN:CONN:TSCH OLSM")
        elif transmission_mode == "TM4":
            # Select closed loop spatial multiplexing as transmission scheme.
            self.get_logger().info("Select CLSMultiplex as transmission scheme.")
            self._visa.send_command("CONF:LTE:SIGN:CONN:TSCH CLSM")

            # Select precoding matrix. (only for TM4)
            self.get_logger().info("Select precoding matrix PM1")
            self._visa.send_command("CONF:LTE:SIGN:CONN:PMAT PMI1")
        else:
            err_msg = "%s isn't supported by CMW500 (use TM1, TM3, TM4 instead)" % transmission_mode
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)

    def _set_scenario(self, mimo, transmission_mode, carrier_aggregation=False):
        """
        Configure the scenario to used on CWM500, depend on MIMO and transmission mode on CMW500

        .. warning:: R&S CMW500 specific method

        :param mimo: Whether MIMO or not
        :type mimo: bool

        :param transmission_mode: the value of transmission mode to set
        :type transmission_mode: str (ex: TM1, TM2, TM3, TM4 ...)

        :rtype: None
        """
        self.get_logger().debug("#### Configure Scenario ####")
        if not carrier_aggregation:
            if not mimo:
                # Only TM1 to be used for SISO and MIMO
                if transmission_mode != "TM1":
                    err_msg = "%s isn't supported if no MIMO (use TM1 instead)" % transmission_mode
                    raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)
                # SISO or SIMO
                # Configure Standard Cell scenario for SISO or SIMO operations and RF port routing
                self.get_logger().info("Configuring SISO or SIMO operation and RF port routing: RF1C used for Tx/Rx")
                self._visa.send_command("ROUT:LTE:SIGN:SCEN:SCEL RF1C,RX1,RF1C,TX1")
            else:
                # Only TM1 to be used for SISO and MIMO
                if transmission_mode not in ["TM3", "TM4"]:
                    err_msg = "%s isn't supported if no MIMO (use TM3 or TM4 instead)" % transmission_mode
                    raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)
                # In case of multiple antenna, configure "Two RF Out Ports" scenario and RF port routing otherwise
                self.get_logger().info("Configuring MIMO operation and RF port routing: RF1C used for Tx1/Rx and RF3C used for Tx2")
                try:
                    self._visa.send_command("ROUT:LTE:SIGN:SCEN:TRO RF1C,RX1,RF1C,TX1,RF3C,TX2")
                except:
                    self._visa.send_command("ROUT:LTE:SIGN:SCEN:TRO RF1C,RX1,RF1C,TX1,RF3C,TX3")
        else:
            if not mimo:
                self.get_logger().info("Configuring SISO or SIMO operation and RF port routing: \n"
                                       "PCC - RF1C used for Tx1/Rx \n"
                                       "SCC - RF1C used for Tx3")
                self._visa.send_command("ROUT:LTE:SIGN:SCEN:CATR RF1C,RX1,RF1C,TX1,RF1C,TX3")
            else:  # MIMO
                self.get_logger().info("Configuring MIMO operation and RF port routing with Carrier Aggregation: \n "
                                       "PCC - RF1C used for Tx1/Rx and RF3C used for Tx2 \n"
                                       "SCC - RF1C used for Tx3 and RF3C used for Tx4")
                self._visa.send_command("ROUT:LTE:SIGN:SCEN:CAFR RF1C,RX1,RF1C,TX1,RF3C,TX2,RF1C,TX3,RF3C,TX4")

    def _set_lte_connection_setup(self, dl_rb_size, dl_i_mcs, ul_rb_size, ul_i_mcs):
        """
        Configure the B{connection setup} block as on CMW500

        .. warning:: R&S CMW500 specific method

        :type dl_rb_size: int
        :param dl_rb_size: DownLink Ressource Block size

        :type dl_i_mcs: int
        :param dl_i_mcs: DownLink MCS index, used to define (DL) modulation and (DL) TBS Index

        :type ul_rb_size: int
        :param ul_rb_size: UpLink Ressource Block size

        :type ul_i_mcs: int
        :param ul_i_mcs: UpLink MCS index, used to define (UL) modulation and (UL) TBS Index

        :rtype: None
        """
        self.get_logger().debug("#### Set Connection Setup ####")
        self.get_logger().info("Select user defined channels as scheduling type.")
        self._visa.send_command("CONFigure:LTE:SIGN:CONNection:STYPe UDCH")
        self._visa.send_command("CONFigure:LTE:SIGN:CONNection:DLEQual ON")
        # Configure DL parameters
        self._set_rb_size_i_mcs("PCC", dl_rb_size, dl_i_mcs, "DL")
        # Configure UL parameters
        self._set_rb_size_i_mcs("PCC", ul_rb_size, ul_i_mcs, "UL")

    def _set_secondary_carrier_connection_setup(self, dl_rb_size, dl_i_mcs):
        """
        Configure the B{connection setup} block as on CMW500 on Secondary Component Carrier

        .. warning:: R&S CMW500 specific method

        :type dl_rb_size: int
        :param dl_rb_size: DownLink Ressource Block size

        :type dl_i_mcs: int
        :param dl_i_mcs: DownLink MCS index, used to define (DL) modulation and (DL) TBS Index

        :type ul_rb_size: int
        :param ul_rb_size: UpLink Ressource Block size

        :type ul_i_mcs: int
        :param ul_i_mcs: UpLink MCS index, used to define (UL) modulation and (UL) TBS Index
        """
        self.get_logger().debug("#### Set Connection Setup on Secondary Component Carrier ####")
        self.get_logger().info("Select user defined channels as scheduling type.")
        self._visa.send_command("CONFigure:LTE:SIGN:CONNection:SCC:STYPe UDCH")
        self._visa.send_command("CONFigure:LTE:SIGN:CONNection:SCC:DLEQual ON")
        # Configure DL parameters
        self._set_rb_size_i_mcs("SCC", dl_rb_size, dl_i_mcs, "DL")

    def _set_rb_size_i_mcs(self, carrier, rb_size, i_mcs, direction):
        """
        sets UL or DL RB Size value

        :type carrier: str
        :param carrier: Component Carrier to set
            - PCC : Principal Component Carrier
            - SCC : Secondary Component Carrier
        :param rb_size: the value of UL/DL RB Size
        :type rb_size: str, the possible values are:
                0,1,2,3,4,5,6,8,9,10,12,15,16,17,18,20,24,25,27,30,36,40,48,50,54,75,80,83,100

        :param i_mcs: the value of UL/DL MCS index (used to compute modulation and TBS index)
        :type i_mcs: str

        :param direction: the direction to set UL or DL
        :type: str

        :rtype: None
        """
        # Primary or Secondary Carrier Component
        supported_carrier_list = ["PCC", "SCC"]
        if carrier not in supported_carrier_list:
            err_msg = "The carrier %s isn't in the supported_carrier_list : %s " % \
                (carrier, supported_carrier_list)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)
        elif carrier == "PCC":
            gpib_command = "CONF:LTE:SIGN:CONN:UDCH:"
        elif carrier == "SCC":
            gpib_command = "CONF:LTE:SIGN:CONN:SCC:UDCH:"

        rb_size = int(rb_size)
        i_mcs = int(i_mcs)
        # set default values (same as CMW500)
        start_rb = 0
        modulation = "QPSK"
        transport_block_size = 5

        wrong_i_mcs = False
        supported_rb_list = [0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 12, 15, 16, 17, 18, 20, 24, 25, 27, 30, 36, 40, 48, 50, 54, 75, 80, 83, 100]
        if rb_size not in supported_rb_list:
            err_msg = "The RB SIZE %d isn't in the rb list on CMW500 : %s " % \
                (rb_size, supported_rb_list)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)

        if direction not in ["UL", "DL"]:
            raise AcsBaseException(AcsBaseException.INVALID_PARAMETER, "Wrong internal param %s " % str(direction))

        # modulation and transport block size according i_mcs (3GPP as reference)
        if direction == "DL":
            if 0 <= i_mcs <= 9:
                # mean modulation order 2 (QPSK)
                modulation = "QPSK"
                transport_block_size = i_mcs
            elif 10 <= i_mcs <= 16:
                # mean modulation order 4 (16QAM)
                modulation = "Q16"
                transport_block_size = i_mcs - 1
            elif 17 <= i_mcs <= 28:
                # mean modulation order 6 (64QAM)
                modulation = "Q64"
                transport_block_size = i_mcs - 2
            else:
                wrong_i_mcs = True

        elif direction == "UL":
            if 0 <= i_mcs <= 10:
                # mean modulation order 2 (QPSK)
                modulation = "QPSK"
                transport_block_size = i_mcs
            elif 11 <= i_mcs <= 20:
                # mean modulation order 4 (16QAM)
                modulation = "Q16"
                transport_block_size = i_mcs - 1
            elif 21 <= i_mcs <= 24:
                # mean modulation order 6 (64QAM) according 3GPP but 16QAM for CMW500
                modulation = "Q16"
                transport_block_size = i_mcs - 2
            else:
                wrong_i_mcs = True

        if wrong_i_mcs:
            err_msg = "The %s I MCS %d isn't supported by CMW500" % (direction, i_mcs)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, err_msg)

        self.get_logger().info("%s resource Alloc: set %s RB Size value to %d modulation: %s trans block size: %d"
                               % (direction, direction, rb_size, modulation, transport_block_size))
        self._visa.send_command("%s%s %d,%d,%s,%d" % (gpib_command, direction, rb_size, start_rb, modulation, transport_block_size))

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

        # Select duplex mode FDD if needed (warning: unlicenced CMW500 will crash on this GPIB command !)
        try:
            current_dmode = self._visa.send_command("CONF:LTE:SIGN:DMOD?")
            if "FFD" not in current_dmode:
                self.get_logger().info("Setting duplex mode to FDD...")
                self._visa.send_command("CONF:LTE:SIGN:DMOD FDD")
        except TestEquipmentException as e:
            self.get_logger().warning("Unable to query/set Duplex mode, default used (%s)" % e.get_error_message())

        # Enable E2E data transfers
        self.get_logger().info("Enabling E2E data transfers on this cell...")
        self._visa.send_command("CONF:LTE:SIGN:ETOE ON")
        # Enable a connection to the DAU.
        self.get_logger().info("Configuring LTE signaling unit to use DAU as data source/sink")
        self._visa.send_command("CONF:LTE:SIGN:CONN:CTYP DAPP")
        # Selects whether the RRC connection shall be kept (ON) or released (OFF) after attach.
        self.get_logger().info("Enabling RRC connection after ATTACH complete")
        self._visa.send_command("CONF:LTE:SIGN:CONN:KRRC " + self.__rrc_conn_state)

    def set_ims_on(self):
        """
        Enables IMS service

        :rtype: None
        """
        self.get_logger().info("Enabling IMS service...")
        self._visa.send_command("SOURce:DATA:CONTrol:IMS2:STATe ON")

    def set_ims_off(self):
        """
        Disables IMS service

        :rtype: None
        """
        self.get_logger().info("Disabling IMS service...")
        self._visa.send_command("SOURce:DATA:CONTrol:IMS2:STATe OFF")
