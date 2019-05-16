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
:summary: This file implements the Localconnectivity UEcmd for Android JB_MR1 device
:since: 2013/06/12
:author: cmichelx
"""

from acs_test_scripts.Device.UECmd.Imp.Android.ICS.LocalConnectivity.LocalConnectivity \
    import LocalConnectivity as LocalConnectivityCommonICS
import acs_test_scripts.Utilities.NetworkingUtilities as NetworkingUtil
from Device.DeviceManager import DeviceManager


class LocalConnectivity(LocalConnectivityCommonICS):

    """
    :summary: Local Connectivity UEcommands operations for JB_MR1 Android platform
    that handle all BT operations
    """

    def _remove_bt_config_file(self):
        """
        Remove bt_conf.xml file
        """
        file_list = ["/data/misc/bluedroid/bt_config.xml",
                     "/data/misc/bluedroid/bt_config.old"]
        cmd = "adb shell rm "
        for file_to_rem in file_list:
            self._logger.info("Remove %s file" % file_to_rem)
            cmd += "%s " % file_to_rem

        self._exec(cmd)

    def get_default_addr(self):
        """
        Get the default BD address (persistant address)

        :rtype: str
        :return: default BD address. format: 12:34:56:78:9A:BC
                 otherwise return empty str
        """

        BDADDR_PATH_PROPERTY = "ro.bt.bdaddr_path"
        PERSIST_BDROID_ADDR = "persist.service.bdroid.bdaddr"

        device_properties = DeviceManager().get_device_properties(self._device.get_name())

        # 1. Default from bd_addr.conf file if it exists and not null
        if BDADDR_PATH_PROPERTY in device_properties:
            bdconf_path = device_properties.get(BDADDR_PATH_PROPERTY)
            cmd = "adb shell cat %s" % bdconf_path
            bdconf = self._exec(cmd)
            bdconf = str(bdconf).upper().strip()
            if bdconf.find("NO SUCH FILE OR DIRECTORY"):
                bdconf = ""
            if bdconf not in ["", "00:00:00:00:00:00"]:
                self._logger.debug("Default BD Address from %s is <%s>"
                                   % (bdconf_path, bdconf))
                if NetworkingUtil.is_valid_mac_address(bdconf):
                    return bdconf
                else:
                    return ""

        # 2. Default from persist.service.bdroid.bdaddr property
        if PERSIST_BDROID_ADDR in device_properties:
            bdaddr = device_properties.get(PERSIST_BDROID_ADDR)
        else:
            bdaddr = ""
        bdaddr = str(bdaddr).upper().strip()
        self._logger.debug("Default BD Address from persist.service.bdroid.bdaddr is <%s>" % bdaddr)
        if NetworkingUtil.is_valid_mac_address(bdaddr):
            return bdaddr
        else:
            return ""
