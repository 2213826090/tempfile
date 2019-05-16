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
:summary: This file implements the Miscellaneous UEcmd for Android phone
:since: 11/04/2011
:author: dgonzalez
"""

from acs_test_scripts.Device.UECmd.Imp.Android.JB_MR1.Misc.PhoneSystem import PhoneSystem as PhoneSystemJB_MR1
from ErrorHandling.DeviceException import DeviceException


class PhoneSystem(PhoneSystemJB_MR1):

    def close_video_popup(self):
        """
        close the welcome screen
        function must be implemented for LLP
        """
        self._logger.debug("Closing Video pop-up")
        filename = "/data/system/users/0/settings_secure.xml"
        self.update_xml_setting(filename, "immersive_mode_confirmations", "confirmed", "setting")

    def close_welcome_screen(self):
        """
        close the welcome screen
        function must be implemented for LLP
        """
        self._logger.debug("Closing Welcome pop-up")
        filename = "/data/data/com.google.android.googlequicksearchbox/shared_prefs/com.android.launcher3.prefs.xml"
        self.update_xml_setting(filename, "cling_gel.workspace.dismissed", "true", "boolean")
        self.update_xml_setting(filename, "launcher.first_load_complete", "true", "boolean")

    def allow_install_non_market_apps(self, value, use_agent=False):
        """
        Allow to install non market applications or not

        :type value: bool
        :param value: the value to set (True or False)
        :type use_agent: bool
        :param use_agent: tell if command will use android api or call adb
        """
        if not value:
            val = 0
        else:
            val = 1
        if use_agent:
            raise DeviceException(DeviceException.FEATURE_NOT_IMPLEMENTED, "Implementation using API is not done")

        else:
            # if the xml file exists, we update it and exit
            xmlsecure = "/data/system/users/0/settings_secure.xml"
            if self.update_xml_setting(xmlsecure, "install_non_market_apps", val, "setting"):
                return

            # else update the database
            cmd = "adb shell sqlite3 /data/data/com.android.providers.settings/databases/settings.db " \
                  "\"insert into secure (name,value) values ('install_non_market_apps', %d)\"" % val
            output = self._exec(cmd)
            if output:
                raise DeviceException(DeviceException.OPERATION_FAILED, "Unable to set install non market application"
                                                                        " setting to %s" % str(val))

    def set_verify_application(self, value):
        """
        Set the security value for android to verify application

        :type value: bool
        :param value: the value to set (True or False)
        """
        if not value:
            val = False
        else:
            val = True

        # Set the verify application
        function = "setVerifyApplication"
        cmd_args = "--ei value %d" % val
        self._internal_exec_v2(self._setting_module, function, cmd_args, is_system=True)
