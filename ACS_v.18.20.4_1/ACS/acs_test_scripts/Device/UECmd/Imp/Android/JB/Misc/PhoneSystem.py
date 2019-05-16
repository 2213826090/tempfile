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
:summary: This file implements the Miscellaneous UEcmd for Android JB device
:since: 12 sep 2012
:author: sfusilie
"""
from acs_test_scripts.Device.UECmd.Imp.Android.ICS.Misc.PhoneSystem import PhoneSystem as PhoneSystemICS
from UtilitiesFWK.Utilities import Global
from ErrorHandling.AcsConfigException import AcsConfigException


class PhoneSystem(PhoneSystemICS):

    """
    :summary: PhoneSystem UEcommands operations for JB Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """

    def __init__(self, phone):
        """
        Constructor.

        """
        PhoneSystemICS.__init__(self, phone)
        # tag for uiautomator
        self.__ui_menu_tag = {"POWER_OFF_MENU": "Power off"}

        # JB settings for disabling lockscreen
        self._locksettingdb = "/data/system/locksettings.db"
        self._locksetting_table = "locksettings"
        self._required_records = {'lockscreen.disabled': "1",
                                  'lock_pattern_autolock': "0",
                                  'lockscreen.password_type': "0",
                                  'lockscreen.password_type_alternate': "0"}

        self._screenup_keyevent = 26

    def get_boot_wake_source(self):
        """
        get the boot wake source from adb

        :rtype: tuple of str str
        :return: (wake source number, wake source reason)
        """
        wake_source = {"01": "WAKE_BATT_INSERT",
                       "02": "WAKE_PWR_BUTTON_PRESS",
                       "03": "WAKE_USB_CHRG_INSERT",
                       "04": "reserved",
                       "05": "WAKE_REAL_RESET",
                       "06": "WAKE_COLD_BOOT",
                       "07": "WAKE_UNKNOWN",
                       "08": "WAKE_KERNEL_WATCHDOG_RESET",
                       "09": "WAKE_SECURITY_WATDCHDOG_RESET",
                       "0A": "WAKE_WATDCHDOG_COUNTER_EXCEEDED",
                       "0B": "WAKE_POWER_SUPPLY_DETECTED",
                       "0C": "WAKE_FASTBOOT_BUTTONS_COMBO",
                       "0D": "WAKE_NO_MATCHING_OSIP_ENTRY",
                       "0E": "WAKE_CRITICAL_BATTERY",
                       "0F": "WAKE_INVALID_CHECKSUM",
                       "10": "WAKE_FORCED_RESET"}
        try:
            retry = 2
            # Implement a retry mechanism as on some benches as on first
            # run of adb (if shutdown previously), it always return unknown
            while retry > 0:
                output = self._exec("adb shell getprop ro.boot.wakesrc",
                                    10, force_execution=True)
                if output[0] != Global.FAILURE:
                    wakesrc = (output[1].strip()).lower()
                    for key in wake_source:
                        if wakesrc in key:
                            self._logger.debug("wake source is %s" % wake_source[key])
                            return key, wake_source[key]
                retry -= 1
        except Exception as error:  # pylint: disable=W0703
            self._logger.warning("get_boot_wake_source: error happened: %s" % str(error))

        self._logger.debug("Board wake source is UNKNOWN")

        return "UNKNOWN", "UNKNOWN"

    def is_ui_element_viewable(self, tag):
        """
        Check if given UI element is viewable from current view.

        :type tag: str
        :param tag: the tag that represent an ui element
                    these tag come from a declared dict of known tag in self.__ui_menu_tag

        :rtype: boolean
        :return: True if element is viewable, False otherwise
        """
        if tag not in self.__ui_menu_tag:
            tmp_txt = "is_ui_element_viewable: bad tag parameter : %s, please only use tag in below list %s" % (
                tag, self.__ui_menu_tag.keys())
            self._logger.error(tmp_txt)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, tmp_txt)

        result = False
        output = self._dump_screen_to_text()
        # compare with the same case
        if output.lower().find(self.__ui_menu_tag[tag].lower()) != -1:
            result = True
        return result
