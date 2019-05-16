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
:summary: cell 3G implementation of Agilent 8960 cellular network simulator
:since: 22/09/2014
:author: jduran4x
"""
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Common.MessagingVisa import \
    MessagingVisa
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ICell3G import ICell3G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Common.CellCommon import CellCommon
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Agilent8960.Tech3G.Data3GVisa import Data3G


class Cell3G(CellCommon, ICell3G):

    """
    Cell 3G implementation for Agilent 8960
    """

    def __init__(self, visa):
        """
        Constructor
        :type visa: VisaInterface
        :param visa: the PyVisa interface
        """
        CellCommon.__init__(self, visa)
        self._data = Data3G(visa)
        self.__messaging = MessagingVisa(visa)

    def __del__(self):
        """
        Destructor
        """
        CellCommon.__del__(self)

    def get_data(self):
        """
        Access to 3G data interface.
        :rtype: IData3G
        :return: the 3G data object.
        """
        return self._data

    def get_messaging(self):
        """
        Access to Visa messaging interface.
        :rtype: IMessagingVisa
        :return: the Visa messaging object.
        """
        return self.__messaging

    def set_paging_service(self, service):
        """
        Sets the paging service
        :type service: str
        :param service: the paging service to set. Possible values:
            - "AMR"
            - "GPRS"
            - "RBT"
        """
        # (err, msg) = W.SetPagingService(self.get_root(), service)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_ps_domain_information(self, domain):
        """
        Sets the PS domain information present or not
        :type domain: str
        :param domain: the PS domain information to set. Possible values:
            - "ABSENT"
            - "PRESENT"
        """
        # (err, msg) = W.SetPSDomainInformation(self.get_root(), domain)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_downlink_arfcn(self, arfcn):
        """
        Sets the downlink ARFCN
        :type arfcn: integer
        :param arfcn: the downlink ARFCN to set
        """
        # (err, msg) = W.SetDownlinkArfcn(self.get_root(), arfcn)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_uplink_arfcn(self, arfcn):
        """
        Sets the uplink ARFCN
        :type arfcn: integer
        :param arfcn: the uplink ARFCN
        :raise TestEquipmentException: failed to call SetUplinkArfcn
        driver function
        :rtype: integer
        :return: the error code of the driver function
        """
        # (err, msg) = W.SetUplinkArfcn(self.get_root(), arfcn)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_uplink_channel_mode(self, state):
        """
        Sets the uplnik channel mode
        :type state: str
        :param state: the state to set:
            - "ON"
            - "OFF"
        """
        # (err, msg) = W.SetUplinkChannelMode(self.get_root(), state)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_gsm_neighbor_cell_states(self, states):
        """
        Sets GSM neighbor cell states
        :type states: array
        :param states: contains the states of the 8 cells to set (see doc1)
        """
        # (err, msg) = W.SetGsmNeighborCellStates(self.get_root(), states)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_inter_freq_neighbor_cell_states(self, states):
        """
        Sets inter frequency neighbor cell states
        :type states: array
        :param states: contains inter frequency neighbor states
        of the 8 cells to set
        """
        # (err, msg) = W.SetInterFreqNeighborCellStates(self.get_root(), states)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_intra_freq_neighbor_cell_states(self, states):
        """
        Sets intra frequency neighbor cell states
        :type states: array
        :param states: contains intra frequency neighbor states
        of the 8 cells to set
        """
        # (err, msg) = W.SetIntraFreqNeighborCellStates(self.get_root(), states)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_drx_cycle(self, drx_cycle_length):
        """
        Sets DRX cycle
        :type drx_cycle_length: str
        :param drx_cycle_length: DRX cycle length to set. Possible values:
            - "FRAM64"
            - "FRAM128"
            - "FRAM256"
            - "FRAM512"
        """
        # (err, msg) = W.SetDrxCycle(self.get_root(), drx_cycle_length)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_periodic_location_update_timer(self, period):
        """
        Sets periodic location update timer
        :type period: integer
        :param period: T312 link establishment timer to set
        (in decihours 0 to 255)
        """
        # (err, msg) = W.SetPeriodicLocationUpdateTimer(self.get_root(), period)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_srb_config_control(self, state):
        """
        Sets SRB configuration control state
        :type state: str
        :param state: the desired state:
            - "ON"
            - "OFF"
        """
        # (err, msg) = W.SetSrbConfigControl(self.get_root(), state)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_scrambling_code(self, code):
        """
        Sets the scrambling code
        :type code: integer
        :param code: the scrambling code to set
        """
        # (err, msg) = W.SetScramblingCode(self.get_root(), code)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_dpch_level(self, level):
        """
        Sets DPCH level
        :type level: double
        :param level: the DPCH level to set
        """
        # (err, msg) = W.SetDpchLevel(self.get_root(), level)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_dpch_state(self, state):
        """
        Sets DPCH state
        :type state: str
        :param state: str representation of the desired state:
            - "ON"
            - "OFF"
        """
        # (err, msg) = W.SetDpchState(self.get_root(), state)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_ms_power(self, mspower):
        """
        Sets the mobile station power
        :type mspower: integer
        :param mspower: the expected power level from the UE to set
        """
        # (err, msg) = W.SetMsPower(self.get_root(), mspower)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_rbt_channel_type(self, channel_type):
        """
        Sets the RBT channel type
        :type channel_type: str
        :param channel_type: the RBT channel type to set:
            - "HSDParmc12" : (12.2k RMC + HSDPA)
            - "HSParmc12"  : (12.2k RMC + HSPA
            - "HSPA"
            - "RMC12"      : (12.2k RMC)
            - "RMC64"      : (64k RMC)
            - "RMC144"     : (144k RMC)
            - "RMC384"     : (384k RMC)
            - "RMC33NC"    : (33k No Coding RMC)
            - "RMCAM1264"  : (12.2k UL/64k DL AM RMC) (active cell operating mode only)
            - "RMCAM12144" : (12.2k UL/144k DL AM RMC) (active cell operating mode only)
            - "RMCAM12384" : (12.2k UL/384k DL AM RMC) (active cell operating mode only)
            - "RMCAM64384" : (64k UL/384k DL AM RMC) (active cell operating mode only)
        """
        # (err, msg) = W.SetRbtChannelType(self.get_root(), channel_type)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_mnc(self, code):
        """
        Sets the mobile network code
        :type code: str
        :param code: the Mobile Network Code to set.
        An integer from 0 to 999.
        """
        if len(code) == 3:
            self._visa.send_command("CALL:MNCode:LENGth DIGits3")
        if len(code) == 2:
            self._visa.send_command("CALL:MNCode:LENGth AUTO")
        self._visa.send_command("CALL:MNC %s" % str(code))

    def clear_ue_info(self):
        """
        Clears UE information
        """
        # (err, msg) = W.ClearUeInfo(self.get_root())
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def get_channel_power(self):
        """
        Gets the channel power
        :rtype: str
        :return: the str representation of channel power or "NA" if the value
        wasn't available when queried
        """
        # (err, power, msg) = W.GetChannelPower(self.get_root())
        # self.__error_check(err, msg)
        # return power
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def init_wcp_measurement(self):
        """
        Initializes WCP measurements
        """
        # (err, msg) = W.InitWCPMeasurement(self.get_root())
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_band_and_dl_arfcn(self, band, arfcn):
        """
        Sets the band and the ARFCN. Checks if the ARFCN is valid for the specified band.
        :type band: str
        :param band: the band to set.
        :type arfcn: integer
        :param arfcn: the downlink ARFCN to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")
        # self.set_downlink_arfcn(arfcn)
        # if str(band).isdigit():
        #     band = "BAND" + str(band)
        # if band in ["BAND5", "BAND6"]:
        #     (err, msg) = W.SetBchBandIndicatorState(self.get_root(), "ON")
        #     self.__error_check(err, msg)
        #     (err, msg) = W.SetBandArbitrator(self.get_root(), band)
        #     self.__error_check(err, msg)
        # elif band == "BAND4":
        #     (err, msg) = W.SetTransmitSIB5bis(self.get_root(), band)
        #     self.__error_check(err, msg)

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
        imsi = self._visa.query_command("CALL:PAG:IMSI?")
        imsi = imsi.replace('"', '')  # pylint: disable=E1101

        return imsi == dut_imsi

    def set_cell_service(self, cell_service):
        """
        Sets the cell service.
        This function will set the paging service and the PS domain
        depending on the value of the cell service.
        :type cell_service: str
        :param cell_service: cell service to set. Possible values:
            - "CIRCUIT"
            - "PACKET"
            - "CIRCUIT_PACKET"
            - "RBTEST"
        """
        self.get_logger().info("Set cell service to %s", cell_service)

        if cell_service == "CIRCUIT":
            self.set_paging_service("AMR")
            self.set_ps_domain_information("ABSENT")
        elif cell_service == "PACKET":
            self.set_paging_service("GPRS")
            self.set_ps_domain_information("PRESENT")
        elif cell_service == "CIRCUIT_PACKET":
            self.set_paging_service("AMR")
            self.set_ps_domain_information("PRESENT")
        elif cell_service == "RBTEST":
            self.set_paging_service("RBT")
        else:
            msg = "Unknown cell service !"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER, msg)

    def configure_basic_cell_parameters(self,
                                        cell_service,
                                        cell_band,
                                        cell_arfcn,
                                        cell_power,
                                        lac=1,
                                        rac=1,
                                        mcc=1,
                                        mnc=1):
        """
        Configure basic cell parameters like cell service, band,
        arfcn (BCH for 2G, Uarfcn for 3G), cell power.
        :type cell_service: str
        :param cell_service: : cell service
        :type cell_band: str
        :param cell_band: : the current band.
        :type cell_arfcn: int
        :param cell_arfcn: : the current Arfcn corresponding to the band.
        :type cell_power: integer
        :param cell_power: the current cell power in dBm.
        :type lac: long
        :param lac: local area code to set.
        :type rac: int
        :param rac: routing area code to set.
        :type mcc: integer
        :param mcc: the mobile network code to set.
        An integer from 0 to 999 when band is set to PCS or GSM850.
        An integer from 0 to 99 for other bands.
        :type mnc: integer
        :param mnc: the mobile network code to set.
        An integer from 0 to 999 when band is set to PCS or GSM850.
        An integer from 0 to 99 for other bands.
        """
        # Set Cell Service
        self.set_cell_service(cell_service)

        # Set Cell Band  and Bch Arfcn
        self.set_band_and_dl_arfcn(cell_band, cell_arfcn)

        # Set Cell Power using INITIAL_CELL_POWER parameter
        self.set_cell_power(cell_power)

        # Set cell LAC
        self.set_lac(lac)
        # Set cell RAC
        self.set_rac(rac)
        # Set cell MCC
        self.set_mcc(mcc)
        # Set cell MNC
        self.set_mnc(mnc)

    def set_bch_update_page_state(self, state):
        """
        This function sets the BCCH Update Page state
        :return: the str representation of channel power or "NA" if the value
        :type state: str
        :param state: The BCCH Update Page state:
                        - AUTO
                        - INHIBIT
        wasn't available when queried
        """

        # if state == "INHIBIT":
        #     state = "INH"

        # (err, msg) = W.SetBchUpdatePageState(self.get_root(), state)
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def get_bch_update_page_state(self):
        """
        This function gets the BCCH Update Page state
        :rtype: str
        :return: The BCCH Update Page state
                    - AUTO
                    - INHIBIT
        """
        # (err, state, msg) = W.GetBchUpdatePageState(self.get_root())

        # if state == "INH":
        #     state = "INHIBIT"
        #
        # self.__error_check(err, msg)
        # return state
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def execute_external_handover(self):
        """
        This function performs an external handover.
        :raise TestEquipmentException: call to ExecuteExternalHandover3G driver function failed
        """
        # (err, msg) = W.ExecuteExternalHandover(self.get_root())
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def get_downlink_arfcn(self):
        """
        Gets the downlink ARFCN
        :raise TestEquipmentException: call to GetDownlinkArfcn3G driver function failed
        :rtype: integer
        :return: The actual downlink ARFCN
        """
        # (err, arfcn, msg) = W.GetDownlinkArfcn(self.get_root())
        # self.__error_check(err, msg)
        # return int(arfcn)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def execute_hard_handover(self):
        """
        This function performs a hard handover.
        :raise TestEquipmentException: call to ExecuteHardHandover3G driver function failed
        """
        # (err, msg) = W.ExecuteHardHandover(self.get_root())
        # self.__error_check(err, msg)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def get_pcr_downlink_arfcn(self):
        """
        Gets the PCR downlink ARFCN
        :raise TestEquipmentException: call to GetPcrDownlinkArfcn3G driver function failed
        :rtype: integer
        :return: The actual PCR downlink ARFCN
        """
        # (err, pcr_arfcn, msg) = W.GetPcrDownlinkArfcn(self.get_root())
        # self.__error_check(err, msg)
        # return int(pcr_arfcn)
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_AVAILABLE, "Not implemented!")

    def set_ps_data_ergch_information_state(self, state):
        """
        This function set Ps Data E-RGCH Information state
        :type state: str
        :param state: Possible values: ON | OFF
        """
        self._visa.send_command("CALL:HSUP:SERV:PSD:ERGC:INF %s" % state)

    def set_drx_cycle_utran(self, drx_cycle_length):
        """
        Sets DRX cycle on utran
        :type drx_cycle_length: str
        :param drx_cycle_length: DRX cycle length to set. Possible values:
            - "FRAM8", "FRAM16", "FRAM32"
            - "FRAM64", "FRAM128", "FRAM256", "FRAM512"
        """
        self._visa.send_command("CALL:DRX:CLENgth:UTRan %s" % drx_cycle_length)

    def set_ctch_allocation_period(self, allocation_period):
        """
        Sets ctch allocation_period
        :type allocation_period: str
        :param allocation_period: The maximum number of radio frames.
                             Possible values: 1-4096
        """
        self._visa.send_command("CALL:CTCHannel:APERiod %d" % allocation_period)

    def set_rac(self, rac):
        """
        Sets the routing area code.
        :type rac: int
        :param rac: routing area code to set.
        """
        self.get_logger().info("Set Routing Area Code to %i" % rac)
        self._visa.send_command("CALL:CELL:RACode %i" % rac)

    def get_rac(self):
        """
        Sets the routing area code.
        :rtype: int
        :return: rac: the routing area code.
        """
        rac = int(self._visa.query_command("CALL:CELL:RACode?"))
        return rac

    def set_page_response(self, pr):
        """
        Sets the Unexpected Page Response parameter
        :type pr: str
        :param pr: the Unexpected Page Response parameter to set
            Possible values:
            - "RESPOND"
            - "IGNORE"
        """
        self.get_logger().info("Set unexpected Page Response parameter to %s" % pr)
        self._visa.send_command("CALL:DCH:UPR %s" % pr)

    def get_mcc(self):
        """
        Gets the Mobile Country code.
        :rtype: int
        :return: mcc: the mobile country code.
        """
        mcc = int(self._visa.query_command("CALL:CELL:MCCode?"))
        return mcc

    def get_mnc(self):
        """
        Gets the Mobile Network code.
        :rtype: int
        :return: mnc: the mobile network code.
        """
        mnc = int(self._visa.query_command("CALL:CELL:MNCode?"))
        return mnc

    def set_red_ie_inclusion_state(self, red):
        """
        Sets the Redirection IE Inclusion State
        :type red: str
        :param red : the Unexpected Page Response parameter to set
            Possible values:
            - "ON"
            - "OFF"
        """
        self.get_logger().info("Set Redirection IE Inclusion State to %s" % red)
        self._visa.send_command("CALL:HAND:RRC:CREL:RED %s" % red)

    def set_eutra_earfcn(self, eutra_earfcn):
        """
        Sets the E-UTRA EARFCN to be used in the RRC-Release redirection information.
        :type eutra_earfcn: str
        :param eutra_earfcn: the E-UTRA EARFCN value to set (0 - 65535)
        """
        self.get_logger().info("Set E-UTRA EARFCN to %s" % eutra_earfcn)
        self._visa.send_command("CALL:HAND:RRC:CREL:RED:EUTR:EARF %s" % eutra_earfcn)

    def set_external_epc_connection(self, ns_lte_ip_lan1, ns_lte_dl_earfcn):
        """
        Set External EPC connection, only working with a PXT

        :type ns_lte_ip_lan1: str
        :param ns_lte_ip_lan1: the IP address to set for External EPC connection.
        15 characters formatted as follows: A.B.C.D.
        The range of values for A = 0 to 126 and 128 to 223.
        The range of values for B,C,D = 0 to 255
        (no embedded spaces).
        :type ns_lte_dl_earfcn: str
        :param ns_lte_dl_earfcn: the E-UTRA EARFCN value to set (0 - 65535)
        """

        # Prepare connection to the LTE network simulator
        self.set_page_response("RESPOND")
        self.set_red_ie_inclusion_state("ON")
        self.set_eutra_earfcn(ns_lte_dl_earfcn)

        # Connect to external PXT
        self._visa.connect_to_external_epc(ns_lte_ip_lan1)

        # Verify ethernet connection before timeout
        # 5 seconds using the PXT active IP address
        self._visa.check_external_epc_connection(ns_lte_ip_lan1, 5)

    def set_equivalent_plmn_list_points(self, mcc, mnc):
        """
        Set equivalent PLMN list points (1 point to add)
        Each entry in the list is specified as a triplet of MCC, MNC and MNC Length parameters
        (MNC Length parameters = 0 if AUTO).
        The first triplet defines the first entry in the E-PLMN list
        this function will set 1 triplets (mcc, mnc, 0);
        Sets MCC and MNC entries for PLMN 1 and sets all other MCC and MNC states to Off.
        :type mcc: int
        :param mcc: MCC of neighbor cell
        :type mnc: int
        :param mnc: MNC of neighbor cell
        """

        self.get_logger().info("setting of the equivalent PLMN list entries")
        self._visa.send_command("CALL:PLMNetwork %d, %d, 0" % (mcc, mnc))

    def get_equivalent_plmn_list_points(self):
        """
        Get the number of points existing on the equivalent PLMN list (4 points to add)
        Each entry in the list is specified as a triplet of MCC, MNC and MNC Length parameters.
        The first triplet defines the first entry in the E-PLMN list
        """

        output = self._visa.query_command("CALL:PLMNetwork:POINts?")
        self.get_logger().info("the number of points existing on the equivalent PLMN list is: %s" % output)

    def set_rrc_state_transition(self, state):
        """
        Sets the RCC state transition to state

        :type state: str
        :param:IDLE, DCH, FACH
        """
        self.get_logger().info("setting the state of SIB19 E_UTRAN entries to %s " % state)
        self._visa.send_command("CALL:SERVice:PSData:RRC:TRANsition:%s" % state)

    def set_geran_neighbor_cells(self,
                                 nb_cells,
                                 arfcn,
                                 band,
                                 ncc,
                                 bcc):
        """
        This command sets the list of entries in the GERAN neighbour cell list.
        :type nb_cells: int
        :param nb_cells: nunber of neighbour cell to define
        :type arfcn: list of 8 int
        :param arfcn: The list of 8 arfcn of the 2G neighbour cells
        :type band: list of 8 str
        :param band: The list of 8 band of the 2G neighbour cells
        :type ncc: list of 8 int
        :param ncc: The list of 8 NCC of the 2G neighbour cells
        :type bcc: list of 8 int
        :param bcc: The list of 8 BCC of the 2G neighbour cells
        """
        self.get_logger().info("Set 2G Neighbour cell")
        cell_list = [1] * nb_cells
        cell_list.extend([0] * (8 - nb_cells))
        cell_list_str = str(cell_list).replace("[", "").replace("]", "")
        arfcn_list_str = str(arfcn).replace("[", "").replace("]", "")
        ncc_list_str = str(ncc).replace("[", "").replace("]", "")
        bcc_list_str = str(bcc).replace("[", "").replace("]", "")
        band_list_str = str(band).replace("[", "").replace("]", "").replace("'", "")
        band_list_str = band_list_str.replace("DCS", "DCS1800").replace("EGSM", "DCS1800")
        band_list_str = band_list_str.replace("GSM850", "DCS1800").replace("PCS", "PCS1900")

        self._visa.send_command("CALL:BCCHannel:GSMSystem:STATe %s" % cell_list_str)
        self._visa.send_command("CALL:BCCHannel:GSMSystem:NCC %s" % ncc_list_str)
        self._visa.send_command("CALL:BCCHannel:GSMSystem:BCC %s" % bcc_list_str)
        self._visa.send_command("CALL:BCCHannel:GSMSystem:BAND %s" % band_list_str)
        self._visa.send_command("CALL:BCCHannel:GSMSystem:BCHannel %s" % arfcn_list_str)

    def set_compress_mode_gap_config(self,
                                     gap_number,
                                     tgmpurpose,
                                     tgsnumber,
                                     tgplength,
                                     tglength,
                                     tglength2,
                                     tgcfn,
                                     tgprc,
                                     dsir1,
                                     dsir1_after,
                                     tgd):
        """
        This command sets the parameters for one gap of compress mode
        :type gap_number: int
        :param gap_number: number of the CM gap to set
        :type tgmpurpose: str
        :param tgmpurpose: gap purpose
        :type tgsnumber: int
        :param tgsnumber: gap slot number
        :type tgplength: int
        :param tgplength: gap pattern length
        :type tglength: int
        :param tglength: gap length
        :type tglength2: int
        :param tglength2: gap length2
        :type tgcfn: int
        :param tgcfn: gap relative CFN
        :type tgprc: int
        :param tgprc: gap pattern repetition count
        :type dsir1: int
        :param dsir1: gap delta SIR1
        :type dsir1_after: int
        :param dsir1_after: gap delta SIR1 after
        :type tgd: int
        :param tgd: gap start distance
        """
        self.get_logger().info("Set gap %d compress mode parameters to purpose %s ,slot number %d ,pattern length %d,"
                               "gap length 1&2 %d %d, relative CFN %d, pattern repetition count %d, delta SIR1 %d, "
                               "delta SIR1 after %d" % (gap_number, tgmpurpose, tgsnumber, tgplength,
                                                        tglength, tglength2, tgcfn, tgprc, dsir1, dsir1_after))
        self._visa.send_command("CALL:COMP:TGPS%d:STATE ON" % gap_number)
        self._visa.send_command("CALL:COMP:TGPS%d:TGMP %s" % (gap_number, tgmpurpose))
        self._visa.send_command("CALL:COMP:TGPS%d:TGSN %d" % (gap_number, tgsnumber))
        self._visa.send_command("CALL:COMP:TGPS%d:TGPL %d" % (gap_number, tgplength))
        self._visa.send_command("CALL:COMP:TGPS%d:TGL %d" % (gap_number, tglength))
        self._visa.send_command("CALL:COMP:TGPS%d:TGL2 %d" % (gap_number, tglength2))
        self._visa.send_command("CALL:COMP:TGPS%d:TGCF:REL %d" % (gap_number, tgcfn))
        self._visa.send_command("CALL:COMP:TGPS%d:TGPRC %d" % (gap_number, tgprc))
        self._visa.send_command("CALL:COMP:TGPS%d:DSIR1 %d" % (gap_number, dsir1))
        self._visa.send_command("CALL:COMP:TGPS%d:DSIR1:AFT %d" % (gap_number, dsir1_after))
        self._visa.send_command("CALL:COMPressed:TGPSequence%d:TGDistance %d" % (gap_number, tgd))
        self._visa.send_command("CALL:COMPressed:TGPSequence:DFSType ATYPe")

    def set_compress_mode_channelization_codes_preset_for_384k_dl_dpch(self):
        """
        This command loads DL Channel code preset configuration for compressed mode
        when 384k DL DPCH is active (non HSDPA).
        See wireless.agilent.com/rfcomms/refdocs/wcdma/wcdma_gen_bse_dl_codes_lvls.html
        table Compressed Mode with 384k DL DPCH(Non-HSDPA)
        """
        self.get_logger().info("Set DL Channel code preset configuration for compressed mode "
                               "when 384k DL DPCH is active (non HSDPA)")
        # Set S-CCPCH channelization code
        self._visa.send_command("CALL:CCPChannel:SECondary:CCODe:CODE 7")
        # Set PICH channelization code
        self._visa.send_command("CALL:PICHannel:CCODe:CODE 16")
        # Set AICH channelization code
        self._visa.send_command("CALL:AICHannel:CCODe:CODE 10")
        # Sets the WCDMA channel code OVSF256
        self._visa.send_command("CALL:DPCHannel:KSPS15:CODE 12")
        # Sets the WCDMA channel code OVSF128
        self._visa.send_command("CALL:DPCHannel:KSPS30:CODE 9")
        # Sets the WCDMA channel code OVSF8
        self._visa.send_command("CALL:DPCHannel:KSPS480:CODE 6")
        # Sets the WCDMA channel code OVSF32
        self._visa.send_command("CALL:DPCHannel:KSPS120:CODE 25")
        # Sets the Channelization codes for WCDMA orthogonal channel noise simulator (OCNS) 1 to 16.
        self._visa.send_command("CALL:OCNSource:CCODe:CODE 2,11,17,23,31,38,47,55,62,69,78,85,94,92,88,90")

    def set_compress_mode_state(self, state):
        """
        This command enable/disable compressed mode.
        :type state: int
        :param state: 1 to enable compress mode, 0 to disable it
        """
        self.get_logger().info("Set compress mode state to %s" % state)
        self._visa.send_command("CALL:COMP:ENABLE %d" % state)

    def get_gsm_neighbor_cells_rssi_measurements(self):
        """
        Gets the GSM neighbor cells RSSI measurements during compress mode.

        :rtype: dict
        :return: dictionary with 2G arfcn found during compress mode
            as keys and 2G rssi measured during compress mode.

        """
        self.get_logger().info("Get geran cell measurements")
        arfcn = self._visa.query_command("CALL:MS:REPorted:GSMSystem:BCHannel?").split(",")
        # return of this GPIB command is the following format:
        # ['+3.50000000E+001', '9.91E37', '9.91E37', '9.91E37', '9.91E37', '9.91E37', '9.91E37', '9.91E37']
        # convert it to integer values
        arfcn_list = [int(float(arfcn[i])) for i in range(len(arfcn))]
        self.get_logger().info("Cell ARFCN found %s" % arfcn_list)
        rssi = self._visa.query_command("CALL:MS:REPorted:GSMSystem:RSSI?").split(",")
        # return of this GPIB command is the following format:
        # ['+3.10000000E+001', '9.91E37', '9.91E37', '9.91E37', '9.91E37', '9.91E37', '9.91E37', '9.91E37']
        # convert it to integer values
        rssi_list = [int(float(rssi[i])) for i in range(len(rssi))]
        self.get_logger().info("Cell RSSI found %s" % rssi_list)
        return dict(zip(arfcn_list, rssi_list))

    def set_compress_mode_measurement_config(self,
                                             meas_type):
        """
        This command sets the measurement config for compressed mode
        :type meas_type: str
        :param meas_type: type of measurement to perform during CM (possible values: ITRFreq, ITRRat, ITREutra
        """
        self.get_logger().info("Set Compressed mode measuremnt config")
        self._visa.send_command("CALL:COMPressed:MEASurement:STATe ON")
        self._visa.send_command("CALL:COMPressed:MEASurement:CONFig %s" % meas_type)

    def set_rau_fop_control(self, fop_control, fop_accept_mode):
        """
        This command sets the Routing area update accept Follow on proceed (FOP) parameters
        This setting controls whether "Follow-On Proceed" IE will be present in the Routing Area Update Request Message.
        This setting applies when Routing Area Update Accept FOP Control is set to Manual
        See http://wireless.agilent.com/rfcomms/refdocs/wcdma/wcdma_gen_bse_message_reply.html#CHDDHCGG
        :type fop_control: int
        :param fop_control: Does FOP has to be auto (1) or manual (0)
        :type fop_accept_mode: int
        :param fop_accept_mode: Does FOP has to be activated
        """
        self.get_logger().info("Set the Routing area update accept Follow on proceed (FOP) parameters")
        self._visa.send_command("CALL:SERVice:GPRS:RAUAccept:FOProceed:CONTROL:AUTO %d" % fop_control)
        self._visa.send_command("CALL:SERVice:GPRS:RAUAccept:FOProceed:MANual %d" % fop_accept_mode)

    def set_gsm_cell_reselection_min_rx_level(self, minrxlevel):
        """
        This command sets GSM Cell Reselection Min Rx Level for each of the 8 GSM neighbor cells.

        :type minrxlevel: int
        :param minrxlevel: the minimum RX level to set
        """
        self.get_logger().info("Set GSM Cell Reselection Min Rx Level %d" % minrxlevel)
        min_rx_level_list = [minrxlevel] * 8
        min_rx_level_list_str = str(min_rx_level_list).replace("[", "").replace("]", "")
        self._visa.send_command("CALL:BCCHannel:GSMSystem:CRESelection:RLMinimum %s" % min_rx_level_list_str)
