"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
:summary: This file implements the Miscellaneous UEcmd for Android phone
:since: 20/03/2014
:author: vdechefd
"""

import re
import os
import time
import tempfile

from acs_test_scripts.Device.UECmd.Imp.Android.JB.Misc.PhoneSystem \
    import PhoneSystem as PhoneSystemJB
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsToolException import AcsToolException
from UtilitiesFWK.Utilities import Global
from acs_test_scripts.Utilities.TextFileUtilities import modify_text_file, add_line_before_pattern


class PhoneSystem(PhoneSystemJB):
    def __init__(self, phone):
        """
        Constructor.
        """
        PhoneSystemJB.__init__(self, phone)

    def set_verify_application(self, value):
        """
        Set the security value for android to verify application

        :type value: bool
        :param value: the value to set (True or False)
        """
        if not value:
            val = 0
        else:
            val = 1


        sql_command = "insert into global (name, value) values ('package_verifier_enable', %d)" % (val)

        cmd = ["adb", "shell", "sqlite3", "/data/data/com.android.providers.settings/databases/settings.db",
                '"{}"'.format(sql_command)]

        output = self._exec(" ".join(cmd))
        if output:
            self.manage_set_verify_application_failure(val, cmd, output)
        # No output - step is successful
        else:
            pass

    def disable_antimalware_request(self):
        """
        Disable request for antimalware check
        """
        package_name = "com.android.vending"
        sharedprefs = "finsky.xml"
        path_on_device = "/data/data/" + package_name + "/shared_prefs/" + sharedprefs
        timeout = 10

        # retrieve com.android.vending path on device
        result, message = self._device.run_cmd("adb shell pm path %s" % package_name, timeout)
        if result == Global.SUCCESS:
            regex_package_path = ".*package:.*"
            matches_path = re.compile(regex_package_path).search(message)
            if matches_path is None:
                # app is not installed: no need to disable it
                return

            cmd = "adb shell ls %s 1>/dev/null 2>&1 && echo Ok || echo NOk" % path_on_device
            output = self._exec(cmd)
            if output == "NOk":
                self._logger.warning("File %s does not exist, skipping" % path_on_device)
                return

            # retrieve com.android.vending shared preferences from device
            tmpdest = os.path.abspath(os.path.join(tempfile.gettempdir(), sharedprefs))
            result, message = self._device.pull(path_on_device, tmpdest, timeout)

            if result == Global.SUCCESS:
                # add a value in shared prefs to disable popup
                is_modified = modify_text_file(tmpdest, add_line_before_pattern, '</map>',
                                               '<boolean name="accepted-anti-malware-consent" value="true" />')
                if is_modified:
                    # push the modified shared prefs on the device
                    result, message = self._device.push(tmpdest, path_on_device, timeout)

                    if result == Global.SUCCESS:
                        # stop com.android.vending process, thus it will re-load shared prefs at next start
                        result, message = self._device.run_cmd("adb shell am force-stop %s" % package_name, timeout)
                        if result != Global.SUCCESS:
                            raise DeviceException("Unable to stop %s application. Error: %s" % (package_name, message))
                        else:
                            adb_force_stop_timeout = self._device.get_config("AdbAmForceStopTimeout", 2, int)
                            time.sleep(adb_force_stop_timeout)
                    else:
                        raise DeviceException("Unable to push %s to %s on device. "
                                              "Error: %s" % (tmpdest, path_on_device, message))
                else:
                    raise AcsToolException("Unable to modify shared prefs for %s" % package_name)
            else:
                raise DeviceException("Unable to pull %s from device. Error: %s" % (path_on_device, message))
        else:
            raise DeviceException("Unable to search path for %s application. Error: %s" % (package_name, message))

    def manage_set_verify_application_failure(self, val, cmd, output):
        # This ugly code is supposed to be removed once Pltf Build will be modified to meet ACS prereqs
        # Is the database locked ?
        # Yes => we need some retry then -|<----------defaultTimeout-------->|
        #                                 |<1s+exec_time>|....|<1s+exec_time>|
        if re.search('locked', output) is not None:
            end_time = int(time.time()) + self._device.get_uecmd_timeout()
            attempt = 1

            while int(time.time()) < end_time and (re.search('locked', output) is not None):
                time.sleep(1)
                self._logger.warning(
                    "set_verify_application : Database is locked -  retry: %d" % attempt)
                output = self._exec(cmd)
                attempt += 1
                if output:
                    if int(time.time()) >= end_time:
                        self._logger.error(
                            "Unable to set verify application setting to {0} - Timeout {1} has been reached" \
                            .format(val, self._device.get_uecmd_timeout()))
                        raise DeviceException(DeviceException.OPERATION_FAILED,
                                              "Unable to set verify application setting to {0} - Timeout {1} has been reached" \
                                              .format(val, self._device.get_uecmd_timeout()))
                # No output - step is successful
                else:
                    break
        # No => database is not locked but another issue occurs
        else:
            self._logger.error(
                "Unexpected error - Unable to set verify application setting to %s" % str(val))
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "Unexpected error - Unable to set verify application setting to %s" % str(val))
