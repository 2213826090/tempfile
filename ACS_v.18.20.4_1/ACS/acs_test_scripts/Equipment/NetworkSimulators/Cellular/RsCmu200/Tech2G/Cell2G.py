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
:summary: cell 2G implementation for RS CMU200 cellular network simulator
:since: 05/04/2011
:author: ymorel
"""

import random
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ICell2G import ICell2G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Tech2G.Data2G import Data2G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Tech2G.Messaging2G import Messaging2G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Tech2G.VoiceCall2G import VoiceCall2G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Tech2G.TestMode2G import TestMode2G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Wrapper.Tech2G import WCell2G as W
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Wrapper.Tech2G import WVoiceCall2G as WVC


class Cell2G(ICell2G):

    """
    Cell 2G implementation for RS CMU200
    """

    MAX_LAC_VALUE = 65535

    RS_CMU200_BANDS = {
        "TGSM380": "UNSUPPORTED",
        "TGSM410": "UNSUPPORTED",
        "GSM450": "GSM450",
        "GSM480": "GSM480",
        "GSM710": "UNSUPPORTED",
        "GSM750": "UNSUPPORTED",
        "TGSM810": "UNSUPPORTED",
        "GSM850": "GSM850",
        "PGSM": "PGSM",
        "EGSM": "EGSM",
        "RGSM": "RGSM",
        "TGSM": "UNSUPPORTED",
        "DCS": "DCS",
        "PCS": "PCS"}

    def __init__(self, root):
        """
        Constructor
        :type root: weakref
        :param root: a weak reference on the root class (Agilent8960)
        """
        ICell2G.__init__(self)
        self.__root = root
        self.__data = Data2G(root)
        self.__messaging = Messaging2G(root)
        self.__voicecall = VoiceCall2G(root)
        self.__testmode = TestMode2G(root)

    def __del__(self):
        """
        Destructor
        """
        del self.__data
        del self.__messaging
        del self.__voicecall
        del self.__testmode

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
        Get the root object of the equipment
        :rtype: RsCmu200
        :return: the root object of the equipment
        """
        return self.__root

    def get_logger(self):
        """
        Gets the logger
        """
        return self.get_root().get_logger()

    def get_data(self):
        """
        Access to 2G data interface.
        :rtype: IData2G
        :return: the 2G data object.
        """
        return self.__data

    def get_messaging(self):
        """
        Access to 2G messaging interface.
        :rtype: IMessaging2G
        :return: the 2G messaging object.
        """
        return self.__messaging

    def get_voice_call(self):
        """
        Access to 2G voice call interface.
        :rtype: IVoiceCall2G
        :return: the 2G voice call object.
        """
        return self.__voicecall

    def get_test_mode(self):
        """
        Access to 2G test mode interface.
        :rtype: ITestMode2G
        :return: the 2G test mode object.
        """
        return self.__testmode

    def set_cell_off(self):
        """
        Wraps to SetCellOff 2G function
        :raise TestEquipmentException: call to SetCellOff
        driver function failed
        :rtype: integer
        :return: error code of the driver function
        """
        (err, msg) = W.SetCellOff(self.get_root())
        self.__error_check(err, msg)

    def set_cell_on(self, mimo=None):
        """
        Wraps to 2G SetCellActive function
        :raise TestEquipmentException: call to SetCellActive
        driver function failed
        :rtype: integer
        :return: error code of the driver function
        """
        (err, msg) = W.SetCellActive(self.get_root())
        self.__error_check(err, msg)

    def set_lac(self, lac):
        """
        Wraps to SetLACode function
        :raise TestEquipmentException: call to SetLACode
        driver function failed
        :type lac: long
        :param lac: local area code to set
        :rtype: integer
        :return: error code of the driver function
        """
        (err, msg) = W.SetLAC(self.get_root(), lac)
        self.__error_check(err, msg)

    def get_lac(self):
        """
        Wraps to GetLACode  function
        :raise TestEquipmentException: call to GetLACode
        driver function failed
        :rtype: long
        :return: the LAC
        """
        (err, lac, msg) = W.GetLAC(self.get_root())
        self.__error_check(err, msg)
        return lac

    def set_band(self, band):
        """
        Sets the band to use
        :type band: str
        :param band: the band to use. Possible values:
            - "TGSM380" => "UNSUPPORTED"
            - "TGSM410" => "UNSUPPORTED"
            - "GSM450" => "GSM450"
            - "GSM480" => "UNSUPPORTED"
            - "GSM710" => "UNSUPPORTED"
            - "GSM750" => "GSM750"
            - "TGSM810" => "TGSM810"
            - "GSM850" => "GSM850"
            - "PGSM" => "PGSM"
            - "EGSM" => "EGSM"
            - "RGSM" => "RGSM"
            - "TGSM" => "UNSUPPORTED"
            - "DCS" => "DCS"
            - "PCS" => "PCS"
        """
        cmu_band = self.RS_CMU200_BANDS.get(band)
        if (cmu_band is None) or (cmu_band == "UNSUPPORTED"):
            raise TestEquipmentException(TestEquipmentException.INVALID_PARAMETER,
                                "Band %s is not supported by RS CMU200" % band)
        (err, msg) = W.SetBand(self.get_root(), cmu_band)
        self.__error_check(err, msg)

    def set_cell_power(self, power):
        """
        Sets the power level of the Broadcast Channel of the cell in dBm
        :type power: double
        :param power: the cell power to set. Possible values:
            - "GSM"
            - "GPRS"
            - "EGPRS"
        """
        (err, msg) = W.SetCellPower(self.get_root(), power)
        self.__error_check(err, msg)

    def set_bcch_arfcn(self, arfcn):
        """
        Sets broadcast control channel ARFCN
        :type arfcn: integer
        :param arfcn: the ARFCN value to set
        """
        (err, msg) = W.SetBchArfcn(self.get_root(), arfcn)
        self.__error_check(err, msg)

    def set_cell_service(self, service):
        """
        Sets the cell service
        :type service: str
        :return: the cell service already set. Possible values:
            - "GSM"
            - "GPRS"
            - "EGPRS"
        """
        (err, msg) = W.SetServingCell(self.get_root(), service)
        self.__error_check(err, msg)

    def is_dut_registered(self, dut_imsi):
        """
        Test if DUT is synchronized with equipment
        :type dut_imsi: str
        :param dut_imsi: IMSI retrieved from CDK.
        :rtype: boolean
        :return: true if the DUT is synchronized with the equipment, false otherwise
        """

        (err, state, msg) = WVC.GetCallControlStatus(self.get_root())
        self.__error_check(err, msg)
        state = state.replace('"', '')  # pylint: disable=E1101

        return state == "SYNC"

    def set_pdtch_arfcn(self, arfcn):
        """
        Sets the packet data traffic channel ARFCN
        :type arfcn: integer
        :param arfcn: the ARFCN of the downlink and uplink PDTCH to set.
        """
        (err, msg) = W.SetPdtchArfcn(self.get_root(), arfcn)
        self.__error_check(err, msg)

    def set_ms_tx_level(self, tx_level):
        """
        Sets the mobile station Tx level
        :type tx_level: integer
        :param tx_level: value of the Tx level to set
        """
        (err, msg) = W.SetMSTxLevel(self.get_root(), tx_level)
        self.__error_check(err, msg)

    def set_tch_arfcn(self, arfcn):
        """
        Sets the traffic channel ARFCN
        :type arfcn: integer
        :param arfcn: the TCH arfcn to set
        """
        (err, msg) = W.SetTchArfcn(self.get_root(), arfcn)
        self.__error_check(err, msg)

    def set_tch_power(self, power):
        """
        Sets traffic channel power.
        :type power: double
        :param double: the traffic channel power to set.
        """
        (err, msg) = W.SetTchPower(self.get_root(), power)
        self.__error_check(err, msg)

    def set_random_lac(self, lac_list=None):
        """
        Sets a random local area code (LAC)
        :type lac_list: list
        :param lac_list: the lac list to avoid
        """
        if not isinstance(lac_list, list):
            # Set default value to empty list here
            # because of pylint warning W0102
            lac_list = []

        current_lac = self.get_lac()

        lac_list.append(current_lac)

        lac = lac_list[0]

        while lac in lac_list:
            lac = random.randint(0, self.MAX_LAC_VALUE)
        self.set_lac(lac)

    def set_mobile_dtx(self, state):
        """
        Sets Mobile DTX state
        :type state: str
        :param state: the Mobile DTX state to set (ON/OFF)
        """
        (err, msg) = W.SetMobileDtx(self.get_root(), state)
        self.__error_check(err, msg)

    def set_mcc(self, code):
        """
        Sets the mobile country code.
        :type code: integer
        :param code: the mobile country code to set. An integer from 0 to 999.
        """
        pass

    def set_mnc(self, code):
        """
        Sets the mobile network code.
        :type code: integer
        :param code: the mobile network code to set.
        An integer from 0 to 999 when band is set to PCS or GSM850.
        An integer from 0 to 99 for other bands.
        """
        pass

