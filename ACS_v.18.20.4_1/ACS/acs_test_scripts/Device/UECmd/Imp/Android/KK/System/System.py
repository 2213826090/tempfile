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
:summary: This file implements the System UEcmd for Android device
:since: 10/29/2014
:author: kturban
"""

import os
from struct import pack

from acs_test_scripts.Device.UECmd.Imp.Android.Common.System.System import System as SystemCommon
from UtilitiesFWK.Utilities import Global

class System(SystemCommon):

    """
    :summary: System UEcommands operations for Android platforms using an C{Intent} based communication to the I{DUT}.
    """
    def disable_google_voice_hotword(self):
        """
        Disables the Google voice hotword feature.

        @return: True if the service is disabled, False otherwise.
        @rtype: Boolean
        """
        self.stop_app("com.google.android.googlequicksearchbox")
        xml_file_path = "/data/data/com.google.android.googlequicksearchbox/shared_prefs/GEL.GSAPrefs.xml"
        ret = self.update_xml_setting(xml_file_path, "GSAPrefs.hotword_enabled", "false", "boolean")
        if ret:
            self._logger.debug("Google Voice Search disabled.")
        else:
            self._logger.error("Cannot set 'hotword_enabled' to false in %s!" % xml_file_path)

        # Pull, edit and push back binary setting file
        bin_path = "/data/data/com.google.android.googlequicksearchbox/app_shared_prefs/SearchSettings.bin"
        localpath = self._device.get_report_tree().get_report_path()
        localfile_in = os.path.join(localpath, "SearchSettings.bin")
        localfile_out = os.path.join(localpath, "SearchSettingsOut.bin")
        output = self._exec("adb pull %s %s" % (bin_path, localfile_in))
        if "does not exist" in output.lower():
            return ret
        bin_in = open(localfile_in, 'r')
        bin_out = open(localfile_out, 'wb')

        edited = False
        for line in bin_in.readlines():
            try:
                if "hotwordDetector" in line:
                    for i in line:
                        if i == chr(1):
                            binval = pack('c', chr(0))
                            edited = True
                        else:
                            binval = pack('c', i)
                        bin_out.write(binval)
                else:
                    for i in line:
                        binval = pack('c', i)
                        bin_out.write(binval)
            except:
                pass

        bin_in.close()
        bin_out.close()

        if edited:
            output = self._exec("adb push %s %s" % (localfile_out, bin_path))

        return ret

    def disable_voiceinteraction(self):
        """
        Disables the Google voiceinteraction service (related to hotword feature)

        @return: True if the service is disabled, False otherwise.
        @rtype: Boolean
        """
        pkg = "com.google.android.googlequicksearchbox/com.google.android.voiceinteraction.GsaVoiceInteractionService"
        (ret_code, _) = self._device.run_cmd("adb shell pm disable %s" % pkg, timeout=10)
        if ret_code != Global.SUCCESS:
            self._logger.error("Cannot disable %s service !" % pkg)
            return False
        return True

    def disable_voicerecognition(self):
        """
        Disables the Google voicerecognition service (related to hotword feature)

        @return: True if the service is disabled, False otherwise.
        @rtype: Boolean
        """
        pkg = "com.google.android.googlequicksearchbox/com.google.android.voicesearch.serviceapi.GoogleRecognitionService"
        (ret_code, _) = self._device.run_cmd("adb shell pm disable %s" % pkg, timeout=10)
        if ret_code != Global.SUCCESS:
            self._logger.error("Cannot disable %s service !" % pkg)
            return False
        return True
