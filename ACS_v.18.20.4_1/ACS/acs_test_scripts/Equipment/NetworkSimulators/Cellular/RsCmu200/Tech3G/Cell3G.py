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
:summary: cell 3G implementation for RS CMU200
:author: ymorel
:since: 06/04/2011
"""

import random
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.Interface.ICell3G import ICell3G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Tech3G.Data3G import Data3G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Tech3G.Messaging3G import Messaging3G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Tech3G.VoiceCall3G import VoiceCall3G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Tech3G.TestMode3G import TestMode3G
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Wrapper.Tech3G import WCell3G as W
from acs_test_scripts.Equipment.NetworkSimulators.Cellular.RsCmu200.Wrapper.Tech3G import WVoiceCall3G as WVC


class Cell3G(ICell3G):

    """
    Cell 3G implementation for RS CMU200
    """

    MAX_LAC_VALUE = 65535

    def __init__(self, root):
        """
        Constructor
        :type root: weakref
        :param root: a weak reference on the root class (Agilent8960)
        """
        ICell3G.__init__(self)
        self.__root = root
        self.__data = Data3G(root)
        self.__messaging = Messaging3G(root)
        self.__voicecall = VoiceCall3G(root)
        self.__testmode = TestMode3G(root)

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

    def get_logger(self):
        """
        Gets the logger
        """
        return self.get_root().get_logger()

    def get_root(self):
        """
        Get the root object of the equipment
        :rtype: RsCmu200
        :return: the root object of the equipment
        """
        return self.__root

    def get_data(self):
        """
        Access to 3G data interface.
        :rtype: IData3G
        :return: the 3G data object.
        """
        return self.__data

    def get_messaging(self):
        """
        Access to 3G messaging interface.
        :rtype: IMessaging3G
        :return: the 3G messaging object.
        """
        return self.__messaging

    def get_voice_call(self):
        """
        Access to 3G voice call interface.
        :rtype: IVoiceCall3G
        :return: the 3G voice call object.
        """
        return self.__voicecall

    def get_test_mode(self):
        """
        Access to 3G test mode interface.
        :rtype: ITestMode3G
        :return: the 3G test mode object.
        """
        return self.__testmode

    def set_cell_off(self):
        """
        Turns off the cell.
        """
        (err, msg) = W.SetCellOff(self.get_root())
        self.__error_check(err, msg)

    def set_cell_on(self, mimo=None):
        """
        Turns on the cell.
        """
        (err, msg) = W.SetCellActive(self.get_root())
        self.__error_check(err, msg)

    def set_paging_service(self, service):
        """
        Sets the paging service
        :type service: str
        :param service: the paging service to set. Possible values:
            - "AMR"
            - "GPRS"
            - "RBT"
        """
        (err, msg) = W.SetPagingService(self.get_root(), service)
        self.__error_check(err, msg)

    def set_uplink_arfcn(self, arfcn):
        """
        Sets the uplink ARFCN
        :type arfcn: integer
        :param arfcn: the uplink ARFCN
        """
        (err, msg) = W.SetUplinkArfcn(self.get_root(), arfcn)
        self.__error_check(err, msg)

    def set_cell_power(self, bch_power):
        """
        Sets the cell power
        :type bch_power: double
        :param bch_power: cell power to set
        """
        (err, msg) = W.SetCellPower(self.get_root(), bch_power)
        self.__error_check(err, msg)

    def set_ms_power(self, mspower):
        """
        Sets the mobile station power
        :type mspower: integer
        :param mspower: the expected power level from the UE to set
        """
        (err, msg) = W.SetMsPower(self.get_root(), mspower)
        self.__error_check(err, msg)

    def set_lac(self, lac):
        """
        Sets the local area code.
        :type lac: long
        :param lac: local area code to set.
        """
        (err, msg) = W.SetLAC(self.get_root(), lac)
        self.__error_check(err, msg)

    def get_lac(self):
        """
        Gets the local area code.
        :rtype: long
        :return: the local area code.
        """
        (err, lac, msg) = W.GetLAC(self.get_root())
        self.__error_check(err, msg)
        return lac

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
        (err, msg) = W.SetBand(self.get_root(), band)
        self.__error_check(err, msg)

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
        :rtype: integer
        :return: error code of the driver function
        """
        (err, msg) = W.SetPhysicalChannelPower(
            self.get_root(),
            level_ref,
            scpich_state,
            power)
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

    def set_cell_service(self, service):
        """
        Sets the cell service
        :type service: str
        :param service: the cell service to set. Possible values:
            - "CIRCUIT"
            - "PACKET"
            - "CIRCUIT_PACKET"
            - "RBTEST"
        """
        self.get_logger().info("set_cell_service function is stubbed for RsCmu200")
        pass

    def set_band_and_dl_arfcn(self, band, arfcn):
        """
        Sets the band and the ARFCN. Checks if the ARFCN is valid for the specified band.
        :type band: str
        :param band: the band to set.
        :type arfcn: integer
        :param arfcn: the downlink ARFCN to set
        """
        self.get_logger().info("set_band_and_dl_arfcn function is stubbed for RsCmu200")
        pass
