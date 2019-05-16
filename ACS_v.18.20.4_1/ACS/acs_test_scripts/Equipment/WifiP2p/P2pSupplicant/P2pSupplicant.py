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
:summary: This file implements WPA P2P Supplicant UC
:since: 04/06/2013
:author: smaurel
"""

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.WifiP2p.Interface.IP2pSupplicant import IP2pSupplicant
from ErrorHandling.TestEquipmentException import TestEquipmentException
import posixpath


class P2pSupplicant(EquipmentBase, IP2pSupplicant):
    """
    Implementation of P2P Client interface
    """

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        """
        IP2pSupplicant.__init__(self)
        EquipmentBase.__init__(self, name, model, eqt_params)
        computer = str(bench_params.get_param_value("Computer"))
        # NOTE: import here to avoid circular dependency on
        # EquipmentManager if imported at top level
        from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager
        self._em = EquipmentManager()
        self._computer = self._em.get_computer(computer)
        self._p2psupplicant_ongoing = False

        # Get binary and cofiguration file path
        self._wpa_supplicant_bin = posixpath.realpath(eqt_params[model]["Binary"])
        self._wpa_p2p_conf = posixpath.realpath(eqt_params[model]["Configuration"])

    def start(self, lan_interface):
        """
        Start the P2P Supplicant
        :type lan_interface: str
        :param lan_interface: Lan interface to use
        """
        if self._p2psupplicant_ongoing:
            msg = "Cannot start 2 P2P Supplicant in the same time"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        self._logger.debug("start_P2P supplicant")

        self._computer.init()
        self._computer.run_cmd("killall wpa_supplicant", 1)

        # Check WPA_Cli existence
        self._computer.check_command("wpa_supplicant")

        # remove the old temporary files
        self._computer.run_cmd("rm -rf /var/run/wpa_supplicant/wlan2")

        self._logger.debug("start wpa_supplicant")

        wpaoutput = self._computer.run_cmd("%s -Dnl80211 -c%s -i %s -dt &> /dev/null &" \
                                           % (self._wpa_supplicant_bin, self._wpa_p2p_conf, lan_interface), 2)
        wpaoutput = wpaoutput["std"]

        if "Failed to initialize wpa_supplicant" in wpaoutput:
            msg = "Failed to initialize "
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

        self._p2psupplicant_ongoing = True

    def stop(self):
        """
        Stop the P2P Supplicant
        """
        if not self._p2psupplicant_ongoing:
            self._logger.warning("P2P Supplicant not started")
            return

        self._logger.debug("Stop P2P Supplicant")

        # clean the supplicant environment
        self._computer.run_cmd("killall wpa_supplicant")
        self._computer.run_cmd("rm -rf /var/run/wpa_supplicant/wlan2")

        self._logger.info("P2P release")
        self._computer.release()

        self._p2psupplicant_ongoing = False
