"""
:copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL OTC Android SSG
:summary: This file implements the BLE Test Step base class
:since: 7/30/15
:author: mmaraci
"""

from TestStep.Device.Wireless.BT.Base import BtBase


class BleBase(BtBase):
    """
    Implements the base test step for BLE
    """

    #####BLE test steps' constant#######

    ADVERTISE_MODE = {'ADVERTISE_MODE_LOW_POWER': 0,
                      'ADVERTISE_MODE_BALANCED': 1,
                      'ADVERTISE_MODE_LOW_LATENCY': 2}

    TX_POWER_LEVELS = {'ADVERTISE_TX_POWER_ULTRA_LOW': 0,
                       'ADVERTISE_TX_POWER_LOW': 1,
                       'ADVERTISE_TX_POWER_MEDIUM': 2,
                       'ADVERTISE_TX_POWER_HIGH': 3}

    ADVERTISE_CODE = {'ADVERTISE_POWER_LEVEL': 10,
                      'ADVERTISE_SCANNABLE': 11,
                      'ADVERTISE_UNSCANNABLE': 12,
                      'BROADCAST_BEACON': 13,
                      'ADVERTISE_BASIC_NAME': 14}

    SCAN_MODE = {'SCAN_MODE_LOW_POWER': 0,
                 'SCAN_MODE_BALANCED': 1,
                 'SCAN_MODE_LOW_LATENCY': 2}

    GATT_SERVER_TYPE = {'CREATE_BARE_GATT_SERVER': 'BARE_GATT_SERVER',
                        'CREATE_FULL_CUSTOM_GATT_SERVER': 'FULL_CUSTOM_GATT_SERVER',
                        'CREATE_NOTIFICATION_GATT_SERVER': 'NOTIFICATION_GATT_SERVER'}

    NOTIFICATION_CHARACTERISTIC = {'PRIVATE_NOTIFICATION': 1,
                                   'HR_NOTIFICATION': 2}

    GATT_SERVER_INTERACTION = {'ADD_SERVICE': 0, 'ADD_CHARACTERISTIC': 1, 'ADD_DESCRIPTOR': 2,
                               'WRITE_CHARACT_CALLBACK': 3, 'WRITE_DESCRIPTOR_CALLBACK': 4,
                                'RELIABLE_WRITE_CALLBACK': 5, 'READ_CHARACT_CALLBACK': 6, 'READ_DESCRIPTOR_CALLBACK': 7}


    def check_scan_mode_valid(self, scan_mode):
        """
        This method verifies whether the provided SCAN_MODE parameter is valid or not.
        Parameter can be provided either as the String name or as the int value
        :param scan_mode: String or int
        :return: scan_mode parameter as int or False if not valid
        """
        if scan_mode in self.SCAN_MODE.values():
            return scan_mode
        elif scan_mode in self.SCAN_MODE:
            return self.SCAN_MODE.get(scan_mode)
        else:
            return False

    def check_advertise_mode_valid(self, advertise_mode):
        """
        This method verifies whether the provided ADVERTISE_MODE parameter is valid or not.
        Parameter can be provided either as the String name or as the int value
        :param advertise_mode: String or int
        :return: advertise_mode parameter as int or False if not valid
        """
        if advertise_mode in self.ADVERTISE_MODE.values():
            return advertise_mode
        elif advertise_mode in self.ADVERTISE_MODE:
            return self.ADVERTISE_MODE.get(advertise_mode)
        else:
            return False

    def check_advertise_code_valid(self, advertise_code):
        """
        This method verifies whether the provided ADVERTISE_CODE parameter is valid or not.
        Parameter can be provided either as the String name or as the int value
        :param advertise_code: String or int
        :return: advertise_code parameter as int or False if not valid
        """
        if advertise_code in self.ADVERTISE_CODE.values():
            return advertise_code
        elif advertise_code in self.ADVERTISE_CODE:
            return self.ADVERTISE_CODE.get(advertise_code)
        else:
            return False

    def check_tx_power_level_valid(self, tx_power_level):
        """
        This method verifies whether the provided TX_POWER_LEVEL parameter is valid or not.
        Parameter can be provided either as the String name or as the int value
        :param tx_power_level: String or int
        :return: tx_power_level parameter as int or False if not valid
        """
        if tx_power_level in self.TX_POWER_LEVELS.values():
            return tx_power_level
        elif tx_power_level in self.TX_POWER_LEVELS:
            return self.TX_POWER_LEVELS.get(tx_power_level)
        else:
            return False

    def check_gatt_server_type(self, gatt_server_type):
        """
        This method check whether the value for GATT_SERVER_TYPE is in the list of acceptable parameters

        :param gatt_server_type: String or int
        :return:
        """
        if gatt_server_type in self.GATT_SERVER_TYPE.values():
            return gatt_server_type
        elif gatt_server_type in self.GATT_SERVER_TYPE:
            return self.GATT_SERVER_TYPE.get(gatt_server_type)
        else:
            return False

    def check_notification_characteristic(self, which_characteristic):
        """

        :param which_characteristic:
        :return:
        """
        if which_characteristic in self.NOTIFICATION_CHARACTERISTIC.values():
            return which_characteristic
        elif which_characteristic in self.NOTIFICATION_CHARACTERISTIC:
            return self.NOTIFICATION_CHARACTERISTIC.get(which_characteristic)
        else:
            return False

    def check_gatt_server_operation(self, gatt_server_operation):
        """
        :param operation:
        :return:
        """
        if gatt_server_operation in self.GATT_SERVER_INTERACTION.values():
            return gatt_server_operation
        elif gatt_server_operation in self.GATT_SERVER_INTERACTION:
            return self.GATT_SERVER_INTERACTION.get(gatt_server_operation)
        else:
            return False