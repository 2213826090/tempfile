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
:summary: virtual interface of 2G functionalities for cellular network
simulators
:since: 10/02/2011
:author: ymorel
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class ICell2G(object):

    """
    ICell2G class: virtual interface of 2G functionalities for cellular
    network simulators.
    """

    def get_data(self):
        """
        Access to 2G data interface.
        :rtype: IData2G
        :return: the 2G data object.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_messaging(self):
        """
        Access to 2G messaging interface.
        :rtype: IMessaging2G
        :return: the 2G messaging object.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_voice_call(self):
        """
        Access to 2G voice call interface.
        :rtype: IVoiceCall2G
        :return: the 2G voice call object.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_test_mode(self):
        """
        Access to 2G test mode interface.
        :rtype: ITestMode2G
        :return: the 2G test mode object.
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

    def set_mcc(self, code):
        """
        Sets the mobile country code.
        :type code: integer
        :param code: the mobile country code to set. An integer from 0 to 999.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_mnc(self, code):
        """
        Sets the mobile network code.
        :type code: integer
        :param code: the mobile network code to set.
        An integer from 0 to 999 when band is set to PCS or GSM850.
        An integer from 0 to 99 for other bands.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_band(self, band):
        """
        Sets the band.
        :type band: str
        :param band: the band to use. Possible values:
            - "TGSM380"
            - "TGSM410"
            - "GSM450"
            - "GSM480"
            - "GSM710"
            - "GSM750"
            - "TGSM810"
            - "GSM850"
            - "PGSM"
            - "EGSM"
            - "RGSM"
            - "TGSM"
            - "DCS"
            - "PCS"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_bcch_arfcn(self, arfcn):
        """
        Sets broadcast control channel ARFCN.
        :type arfcn: integer
        :param arfcn: the ARFCN value to set.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_service(self, service):
        """
        Sets the cell service.
        :type service: str
        :param service: the cell service to set. Possible values:
            - "GSM"
            - "GPRS"
            - "EGPRS"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_power(self, power):
        """
        Sets the power level of the broadcast channel of the cell in dBm
        :type power: double
        :param power: the cell power to set.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def configure_basic_cell_parameters(self,
                                        service,
                                        band,
                                        arfcn,
                                        power,
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
        :type mnc: integer
        :param mnc: the mobile network code to set.
        An integer from 0 to 999 when band is set to PCS or GSM850.
        An integer from 0 to 99 for other bands.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dtx(self, state):
        """
        Sets DTX features state.
        :type state: str
        :param state: the desired state. Possible values:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_paging_multiframe(self, mrf):
        """
        Sets paging multiframe
        :type mrf: integer
        :param mrf: number of multiframes paging subchannels (2 to 9)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_frequency_hopping_state(self, state):
        """
        Sets frequency hopping state
        :type state: str
        :param state: the desired state. Possible values:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ms_tx_level(self, tx_level):
        """
        Sets the mobile station Tx level
        :type tx_level: integer
        :param tx_level: value of the Tx level to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_channel_mode(self, chan_mode):
        """
        Sets the channel mode
        :type chan_mode: str
        :param chan_mode: channel mode to use. Possible values:
            - "FRSP"
            - "EFRS"
            - "HRSP"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_periodic_location_update_timer(self, period):
        """
        Sets the periodic location update timer
        :type period: integer
        :param period: timer value in decihours (0 to 255)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ba_table(self, ba_table, bch_band):
        """
        Sets the CA table
        :type ba_table: array
        :param ba_table: the BA table to set
        :type bch_band: str
        :param bch_band: str representation of the BCH band. Possible values:
            - "DCS"
            - "EGSM"
            - "GSM450"
            - "GSM480"
            - "GSM750"
            - "GSM850"
            - "PCS"
            - "PGSM"
            - "DCS"
            - "RGSM"
            - "TGSM810" => BECAREFUL (not supported for the moment)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ca_table(self, ca_table):
        """
        Sets the CA table
        :type ca_table: array
        :param ca_table: the CA table to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_tch_arfcn(self, arfcn):
        """
        Sets the traffic channel ARFCN
        :type arfcn: integer
        :param arfcn: the TCH arfcn to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_pdtch_arfcn(self, arfcn):
        """
        Sets the packet data traffic channel ARFCN
        :type arfcn: integer
        :param arfcn: the ARFCN of the downlink and uplink PDTCH to set.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_tx_power(self):
        """
        Gets Tx power
        :rtype: str
        :return: the str representation of TX power or "NA" if the value
        wasn't available when queried
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def init_orfs_measurement(self):
        """
        Inits ORFS measurement
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

    def set_random_lac(self, lac_list=None):
        """
        Sets a random local area code (LAC)
        :type lac_list: list
        :param lac_list: the lac list to avoid
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_tch_power(self, power):
        """
        Sets traffic channel power.
        :type power: double
        :param double: the traffic channel power to set.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dtm_state(self, state):
        """
        Sets DTM state
        :raise TestEquipmentException: call to SetDtmState driver function failed
        :type state: str
        :param state: the desired state:
            - "ON"
            - "OFF"
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def execute_external_handover(self):
        """
        This function performs an external handover.
        :raise TestEquipmentException: call to ExecuteExternalHandover2G driver function failed
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_rac(self, rac):
        """
        Sets the routing area code.
        :type rac: int
        :param rac: routing area code to set.
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
        :type eutra_earfcn: str
        :param eutra_earfcn: the E-UTRA EARFCN value to set (0 - 65535)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_neighbour_parameters(self, earfcn, bd_incld, msbd, prio_incld, prio, thrsh_high, thrsh_low_incld, thrsh_low, qrx_incld, qrx):
        """
        Specify parameters for neighbour
        :type pr: int
        :param :Possible values:
            earfcn     ->  earfcn (0 to 65535)
            bd_incld   -> Bandwidth inclusion state.: 0 (exclude), 1 (include)
            msbd       -> Measurement Bandwidth :0 (6RBs), 1 (15RBs), 2 (25RBs), 3 (50 RBs), 4 (75RBs), 5 (100RBs)
            prio_incld -> Priority inclusion state: 0 (exclude), 1 (include)
            prio       -> Priority (0 to 7)
            thrsh_high -> THRESH High (0 to 62)
            thrsh_low_incld-> THRESH Low inclusion state: 0 (exclude), 1 (include)
            thrsh_low  -> THRESH Low :  (0 to 62)
            qrx_incld  -> QRXLEVMIN inclusion state: 0 (exclude), 1 (include)
            qrx        -> QRXLEVMIN (-140 to -78)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_rac(self):
        """
        Sets the routing area code.
        :rtype: int
        :return: rac: the routing area code.
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
        Each entry in the list is specified as a triplet of MCC, MNC.
        The first couple defines the first entry in the E-PLMN list
        this function will set 1 couple (mcc, mnc);
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

    def set_mobile_dtx(self, state):
        """
        Sets Mobile DTX state
        :type state: str
        :param state: the Mobile DTX state to set (ON/OFF)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_bcch_network_channel_order(self,
                                       ncorder):
        """
        This command sets the Network Control Order parameter.

        The Network Control Order parameter sets the level of control the network has to control measurement reporting and cell reselection.
        There are three settings:

            NC0 - allows the mobile station (MS) to perform autonomous cell reselections, which is normal mobile station control.
                NOTE: The MS does not send measurement reports to the network.
            NC1 - The MS performs autonomous cell reselection and sends measurement reports to the network.
            NC2 - The MS sends measurement reports to the network and does not perform autonomous cell reselection unless
                triggered downlink signaling failure or random access failure.
        Network Control Order determines how the MS should perform cell reselections.
        If the Network Control Order has been set to NC0 or NC1 at the test set to which the MS is GPRS attached has the Network Control Order
        set to NC2, autonomous cell reselection may not be performed by the MS as a result of invalid BCH configuration.
        :type ncorder: int
        :param ncorder: the Network Channel Order (0,1,2)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_utran_neighbor_cell_list(self):
        """
        This command gets the list of entries in the UTRAN neighbor cell list.
        :rtype:  cell_list: list
        :return:  cell_list: the list of cells BA list (list of UARFCN, SCI, SC, Div)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_utran_neighbor_cell_list(self, cell_list):
        """
        This command sets the list of entries in the UTRAN neighbor cell list.
        There are four parameters that are required for each entry, these include:
            UARFCN,
            Scrambling Code Indication (SCI),
            Scrambling Code (SC),
            Diversity (Div) indicator.
            If no UTRAN FDD neighbor cells (null list) are sent the test set will not broadcast it.
        :type cell_list: list
        :param cell_list: the list of cells to set in BA list (list of UARFCN, SCI, SC, Div)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_utran_neighbor_cell(self,
                                 utran_cell_id,
                                 uarfcn,
                                 sci,
                                 sc,
                                 div):
        """
        This command sets the list of entries in the UTRAN neighbor cell list.
        There are four parameters that are required for each entry, these include:
            UARFCN,
            Scrambling Code Indication (SCI),
            Scrambling Code (SC),
            Diversity (Div) indicator.
            If no UTRAN FDD neighbor cells (null list) are sent the test set will not broadcast it.
        :type utran_cell_id: int
        :param utran_cell_id: The number id of the 3G neighbor in the 8960 BA list
        :type uarfcn: int
        :param uarfcn: The uarfcn of the 3G neighbor cell
        :type sci: int
        :param sci: The Scrambling Code Indication of the 3G neighbor cell
        :type sc: int
        :param sc: The Scrambling Code of the 3G neighbor cell
        :type div: int
        :param div: The Diversity of the 3G neighbor cell
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def remove_all_gsm_pbcch_neighbor_cells(self):
        """
        Removes all the GSM PBCCH neighbor list from the current GERAN cell.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def remove_all_gsm_bch_neighbor_cells(self):
        """
        Removes all the GSM BCH neighbor list from the current GERAN cell.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def remove_all_eutran_neighbor_cells(self):
        """
        Removes all the EUTRAN neighbor list from the current GERAN cell.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def remove_all_utran_neighbor_cells(self):
        """
        Removes all the UTRAN neighbor list from the current GERAN cell.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_eutran_neighbor_cell(self, earfcn, include_bandwidth, bandwidth,
                                 include_priority, priority, thresh_hight,
                                 include_thresh_low, thresh_low,
                                 include_qrxlevmin, qrxlevmin):
        """
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_network_mode_of_operation(self, mode):
        """
        Sets the Network Operation Mode.
        This can only be set when the Cell Operating Mode is set to Cell Off.
        :type mode: int
        :param mode: network mode of operation. Range: 1 to 3.
        1 - Network operation mode I
        2 - Network operation mode II
        3 - Network operation mode III
        See 3GPP TS 23.060 section 6.3.3.1 for more information about the
        network operation mode.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_lau_reject_state(self, state):
        """
        This command sets Location Area Update Reject State. When the parameter
        is set to ON, the test set will reject the LAU request from the mobile
        station.
        :type state: str
        :param state: state wanted for the LAU reject state.
        Range is ON | OFF.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_lau_reject_gmm_cause(self, cause_id):
        """
        This command sets the Location Area Update Reject cause.
        :type cause_id: int
        :param cause_id: Id of the GMM Cause for Rejecting LAU.
        Range 0 to 255. Possible Values:
        2     00000010     IMSI Unknown
        3     00000011     Illegal MS
        4     00000100     IMSI UKNOWN IN VLR
        5     00000101     IMEI NOT ACCEPTED
        6     00000110     Illegal ME
        7     00000111     GPRS Services Not Allowed
        8     00001000     GPRS/Non-GPRS Services Not Allowed
        9     00001001     MS Identity Cannot Be Derived
        10     00001010     Implicitly Detached
        11     00001011     PLMN Not Allowed
        12     00001100     Location Area Not Allowed
        13     00001101     Roaming Not Allowed In This LA
        16     00010000     MSC Temporarily Not Reachable
        17     00010001     Network Failure
        22     00010110     Congestion
        32     00100000     Service Option Not Supported
        33     00100001     Requested Service Option Not Subscribed
        34     00100010     Service Option Temporarily Out of Order
        48     00110000     Retry Upon Entry Into A New Cell
        95     01011111     Semantically Incorrect Message
        96     01100000     Invalid Mandatory Information
        97     01100001     Message Type Nonexistent
        98     01100010     Msg Type Incompatible With Prot State
        99     01100011     Information Element Nonexistent
        100     01100100     Conditional IE Error
        101     01100101     Msg Incompatible With Protocol State
        111     01101111     Protocol Error, Unspecified
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ps_handover_state(self, ps_state):
        """
        This command sets/quires the packet-switched handover state.
        When the PS handover state is set to ON, the handover of a DUT in packet transfer state will maintain packet resources during the handover procedure.
        otherwise, the DUT must abort the packet resources during the handover procedure.
        :rtype:  ps_state: int
        :return:  ps_state: 0 to set the PS handover state to OFF 1 to ON
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
