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
:summary: virtual interface of LTE functionalities for cellular network
simulators
:since: 27/04/2012
:author: lvacheyx
.. note:: BZ3071 - PING MO over LTE network
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class ICell4G(object):

    """
    ICell4G class: virtual interface of LTE functionalities for LTE cellular
    network simulators.
    """

    def get_data(self):
        """
        Access to LTE data interface.
        :rtype: IData4G
        :return: the LTE data object.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_messaging(self):
        """
        Access to LTE messaging interface.
        :rtype: IMessaging4G
        :return: the LTE messaging object.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_on(self, mimo=None):
        """
        Set cell on

        :type mimo: boolean
        @ param mimo: If mimo is not used, only RFO1 / MOD1 are on
        If mimo is used, RFO1 / MOD1 and RF02 / MOD2 are on
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_cell_status(self):
        """
        Turns on the cell.

        :rtype:str
        :return: Cell state (ON|OFF)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_off(self):
        """
        Turns off the cell for RFO1 and RFO2.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_band(self, band):
        """
        Set the cell band

        :param band: Cell Band to be set
        :type: int
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_secondary_carrier_band(self, band):
        """
        Set the cell band on Secondary Component Carrier

        :param band: Cell Band to be set
        :type band: int
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

    def set_cell_setup(self, cell_id):
        """
        Set cell setup

        :param cell_id : A-cell or B-cell
        :type cell_id: str ("A" or "B" are expected values)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_signal_mode(self, mode):
        """
        Sets the operation Mode

        :type mode: str
        :param mode: BSE or SA
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_power(self, power_rf1, power_rf2=None):
        """
        Sets the cell power amplitude

        :type power_rf1, power_rf2 : double
        :param power_rf1, power_rf2 : cell power to set on RF1 and RF2
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_secondary_carrier_cell_power(self, cell_power):
        """
        Set the cell power on Secondary Component Carrier

        :type cell_power: float
        :param cell_power: Cell Power to be set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_frequency_setting_method(self):
        """
        Sets the frequency setting method

        .. warning:: EARFCN [hard coded] or FREQ [not used for now]
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_downlink_earfcn(self, channel):
        """
        Set the downlink E-ARFCN

        Terms:
        - E-ARFCN: E-UTRA Absolute Radio Frequency Channel Number
        - E-UTRA: Evolved Universal Terrestrial Radio Access

        :param channel: downlink channel to be set
        :type: int
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_secondary_carrier_earfcn(self, channel):
        """
        Sets the downlink E-ARFCN of Secondary Component Carrier

        Terms:
        - E-ARFCN: E-UTRA Absolute Radio Frequency Channel Number
        - E-UTRA: Evolved Universal Terrestrial Radio Access

        :param channel: downlink channel to be set
        :type: int
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_random_lac(self):
        """
        Sets a random local area code (LAC)

        .. warning:: the lac list to avoid, but empty for Agilent E6621A
        It is not supported by this equipment but this function is needed
        to be registered on network simulator so code is empty.
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_id(self, cellid):
        """
        Set the cell ID

        :param cellid: Cell ID to be set
        :type cellid: int
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_cell_channel_bandwidth(self):
        """
        Get the cell  channel bandwidth
        :return: bandwidth: Cell Band to get
        :type bandwidth: str
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_channel_bandwidth(self, bandwidth):
        """
        Set the cell  channel bandwidth

        :type bandwidth: str
        :param bandwidth: Cell bandwidth to be set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_secondary_carrier_channel_bandwidth(self, bandwidth):
        """
        Set the Secondary Componnent Carrier channel bandwidth

        :type bandwidth: str
        :param bandwidth: Cell bandwidth to be set
        """

    def swap_primary_and_secondary_carrier_settings(self):
        """
        Initiates a swap of PCC and SCC settings
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_secondary_carrier_state(self, state):
        """
        Controls the state of the Secondary Component Carrier

        :type state: str
        :param state: State to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_rrc_state(self):
        """
        Returns the rrc state
        Passed for AgilentE6621A as not sure if needed or supported.
        :rtype: str
        :return: the rrc state
        .. warning:: TODO, to check if it's supported by AgilentE6621A
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_antennas_number(self):
        """
        Returns the number of antennas
        The number of antennas dictates the Transmission mode availablility.
        :rtype: int
        :return: the number of antennas
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_antennas_number(self, antennas_number):
        """
        sets the number of antennas wanted
        The number of antennas dictates the Transmission mode availability.
        :param antennas_number: the number of antennas to set
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_transmission_mode(self):
        """
        Returns the transmission mode
        :rtype: str
        :return: the transmission mode
        """

        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_transmission_mode(self, transmission_mode):
        """
        sets the number of antennas wanted
        :param transmission_mode: the value of transmission mode to set
        :type transmission_mode: str (ex: TM1, TM2, TM3, TM4 ...)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_dl_i_mcs(self):
        """
        Returns the value of DL I_MCS
        :rtype: int
        :return: the value of DL I_MCS
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_dl_i_mcs(self, dl_i_mcs):
        """
        sets the DL I_MCS
        :param dl_i_mcs: the value of DL I_MCS to set,
                    this parameter can be adjusted in order to reach the max throughput
        :type dl_i_mcs: int (ex: 22)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_ul_i_mcs(self):
        """
        Returns the value of UL I_MCS
        :rtype: int
        :return: the value of UL I_MCS
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ul_i_mcs(self, ul_i_mcs):
        """
        sets the UL I_MCS
        :param ul_i_mcs: the value of UL I_MCS to set,
                    this parameter can be adjusted in order to reach the max throughput
        :type ul_i_mcs: int (ex: 22)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_ul_grant_mode(self):
        """
        Returns the UL GRANT MODE
        :rtype: str
        :return: the ul grant mode (AUTO or FIXEDMAC)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        :param mimo: mimo, not used for AgilentE6621A, only for CMW500
        :type carrier_aggregation: bool
        :param carrier_aggregation: Use of Carrier Aggregation or not, only for CMW500
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

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
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)


    def keep_rrc_connection(self, enabled):
        """
        Selects whether the RRC connection shall be kept or released after attach.

        :param enabled: keep connection if this parameter is set to 'True'.
                        Otherwise the RRC state will be back to 'IDLE'
                        after cell attach process.
        :type enabled: bool
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_cell_ratio(self, ratio):
        """
        Set the cell ratio for the active cell

        :type ratio: int
        :param ratio: the cell ratio is an int in the range [0,101]

        :param cell_id : A-cell or B-cell
        :type cell_id: str ("A" or "B" are expected values)
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def send_custom_message(self, message_number):
        """
        send custom message

        :type message_number:
        :param message_number:
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def send_handover_message(self, message_number):
        """
        send handover message

        :type message_number:
        :param message_number:
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ims_on(self):
        """
        Enables IMS service

        :rtype: None
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_ims_off(self):
        """
        Disables IMS service

        :rtype: None
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_rrc_connection_inact_timer(self, timer_len):
        '''
        Set the inactivity timer after which an RRC Connection Release will be sent to the DUT

        :type: int
        :param: timer duration in seconds

        :rtype: None
        '''
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_rrc_connection_timer(self, timer_len):
        '''
        Set the check interval for connection status monitoring

        :type: int
        :param: timer duration in seconds

        :rtype: None
        '''
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
