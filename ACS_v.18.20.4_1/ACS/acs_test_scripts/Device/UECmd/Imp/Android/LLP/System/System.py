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

:organization: INTEL MCG PSI
:summary: This file implements the System UEcmd for Android device
:since: 2015-09-14
:author: emarchan
"""

from acs_test_scripts.Device.UECmd.Imp.Android.KK.System.System import System as SystemKK
from ErrorHandling.DeviceException import DeviceException
import UtilitiesFWK.Utilities as Util
import time

class System(SystemKK):

    def set_filesystem_rw(self):
        """
        Set the file system in read/write mode.
        """
        self._logger.info("Set filesystem to read-write (all partitions)")
        max_retry = 3
        tries = 0
        return_code = Util.Global.FAILURE

        self._device.enable_adb_root()
        # For 1and builds, we'll have to execute the full sequence. For the others, the reboot won't be needed
        result, msg = self._device.run_cmd("adb disable-verity", timeout=10, silent_mode=True)
        if result == 0:
            if "Verity already disabled" not in msg:
                # System
                self._logger.info("Please wait while we reboot the device to enable RW in /...")
                time.sleep (2)
                self._device.reboot()
            else:
                self._logger.info(msg)
        else:
            # Display an error but no exception for non 1and builds.
            # Or in the case adb
            self._logger.error("Please use an ADB or a device that allows disable-verity. Error from ADB is:")
            self._logger.error(msg)

        while tries < max_retry and return_code != Util.Global.SUCCESS:
            return_code, return_msg = self._device.run_cmd("adb remount", 5, True)
            tries += 1

        if return_code == Util.Global.FAILURE:
            error_msg = "Failed to set filesystem to read-write: {0}".format(return_msg)
            raise DeviceException(DeviceException.FILE_SYSTEM_ERROR, error_msg)

