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
:summary: This file implements the Localconnectivity UEcmd for Android ICS device
:since: 02/12/2011
:author: ssavrimoutou
"""
from ErrorHandling.AcsConfigException import AcsConfigException
from ErrorHandling.AcsToolException import AcsToolException
from ErrorHandling.DeviceException import DeviceException

from acs_test_scripts.Device.UECmd.Imp.Android.Common.LocalConnectivity.LocalConnectivity import LocalConnectivity as LocalConnectivityCommon
from acs_test_scripts.Device.UECmd.UECmdDecorator import need

class ScanMode(object):

    """
    Class enumerating Scan mode value :
    cf. http://developer.android.com/reference/android/bluetooth/BluetoothAdapter.html
    """
    SCAN_MODE_NONE = 20
    SCAN_MODE_CONNECTABLE = 21
    SCAN_MODE_CONNECTABLE_DISCOVERABLE = 23


class LocalConnectivity(LocalConnectivityCommon):

    """
    :summary: Local Connectivity UEcommands operations for NexusS ICS Android platform
    that handle all BT operations
    """

    def __init__(self, phone):
        """
        Constructor
        """
        LocalConnectivityCommon.__init__(self, phone)

    @need('bluetooth')
    def set_bt_tethering_power(self, state):
        """
        Enable / Disable the Bluetooth Tethering feature.

        :type state: str or int
        :param state: can be ('on', '1', 1) to enable
                            ('off', '0', 0) to disable
        """
        state = str(state).upper()
        self._logger.info("Set BT Tethering to %s" % state)
        if state in ("ON", "1"):
            state = 1
        elif state in ("OFF", "0"):
            state = 0
        else:
            msg = "set_bt_tethering_power : " + \
                "Parameter state [%s] not valid" % state
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        method = "setBluetoothTethering"
        args = "--ei Pan_state %d" % state
        self._internal_exec_v2(self._BLUETOOTH_MODULE, method, args, is_system=True)

    @need('bluetooth')
    def get_bt_tethering_power(self):
        """
        Get Bluetooth Tethering feature state (Enable/disable).

        :rtype: boolean
        :return: True = enable
                 False = disable
        """
        self._logger.info("Get BT Tethering state")

        method = "getBluetoothTethering"

        pan_state = (self._internal_exec_v2(self._BLUETOOTH_MODULE, method, is_system=True)).get("Pan_state")
        if pan_state is None:
            return_msg = "pan_state parameter not found"
            self._logger.error(return_msg)
            raise AcsToolException(AcsToolException.INVALID_PARAMETER, return_msg)

        self._logger.info("BT tethering state: " + pan_state)
        # pylint: disable=E1101
        if pan_state == "ON":
            pan_state = True
        elif pan_state == "OFF":
            pan_state = False
        else:
            return_msg = "Invalid BT tethering state: " + pan_state
            self._logger.error(return_msg)
            raise DeviceException(DeviceException.INVALID_DEVICE_STATE, return_msg)

        return pan_state
