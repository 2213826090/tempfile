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
:summary: This file implements WPA P2P Supplicant
:since: 04/06/2013
:author: smaurel
"""

from acs_test_scripts.Equipment.WifiP2p.Interface.IP2pSupplicant import IP2pSupplicant
from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2


class WifiP2PSupplicantCLI(BaseV2, IP2pSupplicant):
    """
    Implementation of P2P Supplicant interface
    """

    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)
        self._logger = device.get_logger()

        self._p2psupplicant_ongoing = False
        self._wpa_supp_checked = False

    def start(self, lan_interface):
        """
        Start the P2P Supplicant
        :type lan_interface: str
        :param lan_interface: UNUSED
        """
        if self._p2psupplicant_ongoing:
            msg = "Cannot start 2 P2P Supplicant at the same time"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        self._logger.debug("start_P2P supplicant")

        # Check WPA_Cli existence
        self.__check_wpa_suppl()

        # bring up wifi on device
        self._exec("adb shell ifconfig wlan0 down")
        self._exec("adb shell stop p2p_supplicant")
        self._exec("adb shell insmod /lib/modules/wl12xx.ko")
        self._exec("adb shell insmod /lib/modules/wl12xx_sdio.ko")
        self._exec("adb shell ifconfig wlan0 up")
        self._exec("adb shell setprop ctl.start p2p_supplicant")
        output = self._exec("adb shell ps | grep p2p_supplicant")
        self._logger.debug("check p2p supplicant running: " + output)

        self._p2psupplicant_ongoing = True

    def stop(self):
        """
        Stop the P2P Supplicant
        """
        if not self._p2psupplicant_ongoing:
            self._logger.debug("P2P Supplicant not started\n")
            return

        self._logger.debug("Stop P2P Supplicant")

        self._exec("adb shell stop p2p_supplicant")

        self._p2psupplicant_ongoing = False

    def __check_wpa_suppl(self):
        """
        Check that P2P Supplicant binary tool is installed on the DUT
        """
        if not self._wpa_supp_checked:
            out = self._exec("adb shell which wpa_supplicant")
            if "/wpa_supplicant" not in out:
                msg = "wpa_supplicant is not available on the PHONE"
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

            self._wpa_supp_checked = True
