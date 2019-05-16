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
:summary: virtual interface of 3G functionalities for cellular network
simulators
:since: 10/02/2011
:author: ymorel
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class ICell3G(object):

    """
    ICell3G class: virtual interface of 3G functionalities for cellular
    network simulators.
    """

    def get_data(self):
        """
        Access to 3G data interface.
        :rtype: IData3G
        :return: the 3G data object.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_messaging(self):
        """
        Access to 3G messaging interface.
        :rtype: IMessaging3G
        :return: the 3G messaging object.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_voice_call(self):
        """
        Access to 3G voice call interface.
        :rtype: IVoiceCall3G
        :return: the 3G voice call object.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_test_mode(self):
        """
        Access to 3G test mode interface.
        :rtype: ITestMode3G
        :return: the 3G test mode object.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_off(self):
        """
        Turns off the cell.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_on(self, mimo=None):
        """
        Turns on the cell.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_lac(self, lac):
        """
        Sets the local area code.
        :type lac: long
        :param lac: local area code to set.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_lac(self):
        """
        Gets the local area code.
        :rtype: long
        :return: the local area code.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_band(self, band):
        """
        Sets the band.
        :type band: str
        :param band: the band to set. Possible values:
            - "BAND1"
            - "BAND2"
            - "BAND3"
            - "BAND4"
            - "BAND5"
            - "BAND6"
            - "BAND7"
            - "BAND8"
            - "BAND9"
            - "BAND10"
            - "BAND11"
            - "BAND12"
            - "BAND13"
            - "BAND14"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_physical_channel_power(self, level_ref, scpich_state, power):
        """
        Sets physical channels power
        :type level_ref: str
        :param level_ref: the reference level to use:
            - "PCPICH" : PCPICH mode
            - "OPOW" : total channel power mode
        :type scpich_state: str
        :param scpich_state: desired state for the S-CPICH channel
            - "ON"
            - "OFF"
        :type power: str
        :param power: ten powers separated by a ','. The order of powers in the
        str is:
            - 1) P-CPICH :
                - -147 dBm to -20 dBm : PCPICH mode
                - -30 dBm to 0 dBm : total channel power mode (OPOW)
            - 2) P-SCH :
                - -35 dB to +15 dB level reference : PCPICH mode
                - -30 dB to 0 dB level reference : total channel power mode (OPOW)
            - 3) S-SCH :
                - -35 dB to +15 dB level reference : PCPICH mode
                - -30 dB to 0 dB level reference = total channel power mode (OPOW)
            - 4) P-CCPCH :
                - -35 dB to +15 dB level reference : PCPICH mode
                - -30 dB to 0 dB level reference = total channel power mode (OPOW)
            - 5) S-CCPCH :
                - -35 dB to +15 dB level reference : PCPICH mode
                - -30 dB to 0 dB level reference = total channel power mode (OPOW)
            - 6) PICH :
                - -35 dB to +15 dB level reference : PCPICH mode
                - -30 dB to 0 dB level reference = total channel power mode (OPOW)
            - 7) AICH :
                - -35 dB to +15 dB level reference : PCPICH mode
                - -30 dB to 0 dB level reference = total channel power mode (OPOW)
            - 8) DPDCH :
                - -35 dB to +15 dB level reference : PCPICH mode
                - -30 dB to 0 dB level reference = total channel power mode (OPOW)
            - 9) Power Offset :
                - 0 dB to +6 dB level reference : PCPICH mode
            - 10) S-CPICH :
                - -35 dB to +15 dB level reference : PCPICH mode
                - -30 dB to 0 dB level reference = total channel power mode (OPOW)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_paging_service(self, service):
        """
        Sets the paging service
        :type service: str
        :param service: the paging service to set. Possible values:
            - "AMR"
            - "GPRS"
            - "RBT"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ps_domain_information(self, domain):
        """
        Sets the PS domain information present or not
        :type domain: str
        :param domain: the PS domain information to set. Possible values:
            - "ABSENT"
            - "PRESENT"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_power(self, bch_power):
        """
        Sets the cell power
        :type bch_power: double
        :param bch_power: cell power to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_downlink_arfcn(self, arfcn):
        """
        Sets the downlink ARFCN
        :type arfcn: integer
        :param arfcn: the downlink ARFCN to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_uplink_arfcn(self, arfcn):
        """
        Sets the uplink ARFCN
        :type arfcn: integer
        :param arfcn: the uplink ARFCN
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_uplink_channel_mode(self, state):
        """
        Sets the uplink channel mode
        :type state: str
        :param state: the state to set:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_gsm_neighbor_cell_states(self, states):
        """
        Sets GSM neighbor cells states
        :type states: array
        :param states: contains the states of the 8 cells to set (see doc1)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_inter_freq_neighbor_cell_states(self, states):
        """
        Sets inter frequency neighbor cells states
        :type states: array
        :param states: contains inter frequency neighbor states
        of the 8 cells to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_intra_freq_neighbor_cell_states(self, states):
        """
        Sets intra frequency neighbor cells states
        :type states: array
        :param states: contains intra frequency neighbor states
        of the 8 cells to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_periodic_location_update_timer(self, period):
        """
        Sets periodic location update timer
        :type period: integer
        :param period: T312 link establishment timer to set
        (in decihours 0 to 255)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_srb_config_control(self, state):
        """
        Sets SRB configuration control state
        :type state: str
        :param state: the desired state:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_scrambling_code(self, code):
        """
        Sets the scrambling code
        :type code: integer
        :param code: the scrambling code to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dpch_level(self, level):
        """
        Sets DPCH level
        :type level: double
        :param level: the DPCH level to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dpch_state(self, state):
        """
        Sets DPCH state
        :type state: str
        :param state: str representation of the desired state:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ms_power(self, mspower):
        """
        Sets the mobile station power
        :type mspower: integer
        :param mspower: the expected power level from the UE to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_mcc(self, code):
        """
        Sets the mobile country code
        :type code: integer
        :param code: the Mobile Country Code to set.
        An integer from 0 to 999.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_mnc(self, code):
        """
        Sets the mobile network code
        :type code: integer
        :param code: the Mobile Network Code to set.
        An integer from 0 to 99 for other bands.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def clear_ue_info(self):
        """
        Clears UE information
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_channel_power(self):
        """
        Gets the channel power
        :rtype: str
        :return: the str representation of channel power or "NA" if the value
        wasn't available when queried
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def init_wcp_measurement(self):
        """
        Initializes WCP measurements
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_band_and_dl_arfcn(self, band, arfcn):
        """
        Sets the band and the ARFCN. Checks if the ARFCN is valid for the specified band.
        :type band: str
        :param band: the band to set.
        :type arfcn: integer
        :param arfcn: the downlink ARFCN to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def is_dut_registered(self, dut_imsi):
        """
        Test if DUT is synchronized with equipment
        :type dut_imsi: str
        :param dut_imsi: IMSI retrieved from CDK.
        :rtype: boolean
        :return: true if the DUT is synchronized with the equipment, false otherwise
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        :type service: str
        :param service: : cell service
        :type band: str
        :param band: : the current band.
        :type arfcn: int
        :param arfcn: : the current Arfcn corresponding to the band.
        :type power: integer
        :param power: the current cell power in dBm.
        :type lac: long
        :param lac: local area code to set.
        :type rac: int
        :param rac: routing area code to set.
        :type mcc: integer
        :param mcc: the mobile network code to set.
        An integer from 0 to 999 when band is set to PCS or GSM850.
        An integer from 0 to 99 for other bands.
        :type code: integer
        :param code: the mobile network code to set.
        An integer from 0 to 999 when band is set to PCS or GSM850.
        An integer from 0 to 99 for other bands.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_random_lac(self, lac_list=None):
        """
        Sets a random local area code (LAC)
        :type lac_list: list
        :param lac_list: the lac list to avoid
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_bch_update_page_state(self, state):
        """
        This function sets the BCCH Update Page state
        :type state: str
        :param state: The BCCH Update Page state:
                        - AUTO
                        - INHIBIT
        :raise TestEquipmentException: call to SetBchUpdatePageState driver function failed
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_bch_update_page_state(self):
        """
        This function gets the BCCH Update Page state
        :rtype: str
        :return: The BCCH Update Page state
                    - AUTO
                    - INHIBIT
        :raise TestEquipmentException: call to GetBchUpdatePageState driver function failed
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def execute_external_handover(self):
        """
        This function performs an external handover.
        :raise TestEquipmentException: call to ExecuteExternalHandover3G driver function failed
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_downlink_arfcn(self):
        """
        Gets the downlink ARFCN
        :raise TestEquipmentException: call to GetDownlinkArfcn3G driver function failed
        :rtype: integer
        :return: The actual downlink ARFCN
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def execute_hard_handover(self):
        """
        This function performs a hard handover.
        :raise TestEquipmentException: call to ExecuteHardHandover3G driver function failed
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_pcr_downlink_arfcn(self):
        """
        Gets the PCR downlink ARFCN
        :raise TestEquipmentException: call to GetPcrDownlinkArfcn3G driver function failed
        :rtype: integer
        :return: The actual PCR downlink ARFCN
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ps_data_ergch_information_state(self, state):
        """
        This function set Ps Data E-RGCH Information state
        :type state: str
        :param state: Possible values: ON | OFF
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_rac(self, rac):
        """
        Sets the routing area code.
        :type rac: int
        :param rac: routing area code to set.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_rac(self):
        """
        Sets the routing area code.
        :rtype: int
        :return: rac: the routing area code.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_drx_cycle_utran(self, frame_number):
        """
        Sets DRX cycle on utran
        :type frame_number: str
        :param frame_number: DRX cycle length to set. Possible values:
            - "FRAM8", "FRAM16", "FRAM32"
            - "FRAM64", "FRAM128", "FRAM256", "FRAM512"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ctch_allocation_period(self, allocation_period):
        """
        Sets ctch allocation_period
        :type allocation_period: str
        :param allocation_period: DRX cycle length to set. Possible values: 1-4096
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_mcc(self):
        """
        Gets the Mobile Country code.
        :rtype: int
        :return: mcc: the mobile country code.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_mnc(self):
        """
        Gets the Mobile Network code.
        :rtype: int
        :return: mnc: the mobile network code.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_page_response(self, pr):
        """
        Sets the Unexpected Page Response parameter
        :type pr: str
        :param pr: the Unexpected Page Response parameter to set
            Possible values:
            - "RESPOND"
            - "IGNORE"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_red_ie_inclusion_state(self, red):
        """
        Sets the Redirection IE Inclusion State
        :type red: str
        :param red : the Unexpected Page Response parameter to set
            Possible values:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_eutra_earfcn(self, eutra_earfcn):
        """
        Sets the E-UTRA EARFCN to be used in the RRC-Release redirection information.
        :type eutra_earfcn: int
        :param eutra_earfcn: the E-UTRA EARFCN value to set (0 - 65535)

        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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

        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_equivalent_plmn_list_points(self, mcc, mnc):
        """
        Set equivalent PLMN list points (1 point to add)
        Each entry in the list is specified as a triplet of MCC, MNC and MNC Length parameters (MNC Length parameters = 0 if AUTO).
        The first triplet defines the first entry in the E-PLMN list
        this function will set 1 triplets (mcc, mnc, 0);
        Sets MCC and MNC entries for PLMN 1 and sets all other MCC and MNC states to Off.
        :type mcc: int
        :param mcc: MCC of neighbor cell
        :type mnc: int
        :param mnc: MNC of neighbor cell
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_equivalent_plmn_list_points(self):
        """
        Get the number of points existing on the equivalent PLMN list (4 points to add)
        Each entry in the list is specified as a triplet of MCC, MNC and MNC Length parameters.
        The first triplet defines the first entry in the E-PLMN list
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_rrc_state_transition(self, state):
        """
        Sets the RCC state transition to state

        :type state: str
        :param:IDLE, CELL_DCH, CELL_FACH
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_rrc_state_transition(self, state):
        """
        Sets the RCC state transition to state

        :type state: str
        :param:IDLE, DCH, FACH
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_geran_neighbor_cell(self,
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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def enable_compress_mode(self,
                             enable):
        """
        This command enable/disable compressed mode.
        :type enable: int
        :param enable: 1 to enable compress mode, 0 to disable it
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        :type tgplength: int
        :param tgplength: gap length
        :type tgplength2: int
        :param tgplength2: gap length2
        :type tgcfn: int
        :param tgcfn: gap relative CFN
        :type tgprc: int
        :param tgprc: gap pattern repetition count
        :type dsir1: int
        :param dsir1: gap delta SIR1
        :type dsir1_after: int
        :param dsir1_after: gap delta SIR1 after
        :type tgd: int
        :param tgd: gap Start Distance
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_gsm_neighbor_cells_rssi_measurements(self):
        """
        Gets the GSM neighbor cells RSSI measurements during compress mode.

        :rtype: dict
        :return: dictionary with 2G arfcn found during compress mode as
            keys and 2G rssi measured during compress mode.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_compress_mode_channelization_codes_preset_for_384k_dl_dpch(self):
        """
        This command loads DL Channel code preset configuration for compressed mode when 384k DL DPCH is active (non HSDPA).
        see wireless.agilent.com/rfcomms/refdocs/wcdma/wcdma_gen_bse_dl_codes_lvls.html
        table Compressed Mode with 384k DL DPCH(Non-HSDPA)

        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_compress_mode_measurement_config(self,
                                             meas_type):
        """
        This command sets the measurement config for compressed mode
        :type meas_type: str
        :param meas_type: type of measurement to perform during CM (possible values: ITRFreq, ITRRat, ITREutra
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_gsm_cell_reselection_min_rx_level(self, minrxlevel):
        """
        This command sets GSM Cell Reselection Min Rx Level for each of the 8 GSM neighbor cells.

        :type minrxlevel: int
        :param minrxlevel: the minimum RX level to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_rau_fop_control(self, fop_control, fop_accept_mode):
        """
        This command sets the Routing area update accept Follow on proceed (FOP) parameters
        This setting controls whether "Follow-On Proceed" IE will be present in the Routing Area Update Request Message.
        This setting applies when Routing Area Update Accept FOP Control is set to Manual
        :type fop_control: int
        :param fop_control: Does FOP has to be auto (1) or manual (0)
        :type fop_accept_mode: int
        :param fop_accept_mode: Does FOP has to be activated
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_barred_state(self, barred):
        """
        This command sets the cell as barred or not barred.
        :type barred: int or string
        :param barred: int (0,1) or ('ON','OFF')
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_barred_state(self):
        """
        This command returns the cell barred state.
        :rtype: str
        :return: the bar state. Possible returned values:
            0, 1
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)