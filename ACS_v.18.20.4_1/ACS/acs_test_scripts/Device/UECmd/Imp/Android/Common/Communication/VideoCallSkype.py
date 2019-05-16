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

:organization: INTEL PEG SVE DSV
:summary: This file implements the IVideoCallSkype UEcmd for Android phone
:since: 07/01/2014
:author: jongyoon
"""
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Misc.PhoneSystem import PhoneSystem
from acs_test_scripts.Device.UECmd.Interface.Communication.IVideoCallSkype import IVideoCallSkype #@UnresolvedImport

class VideoCallSkype(BaseV2, IVideoCallSkype):
    """
    :summary: VoiceCall UEcommands operations for Android platforms
    using an C{Intent} based communication to the I{DUT}.
    """
    touchCmds_720P = {
        'select_contact_1st':'adb shell input tap 350 450',
        'select_contact_2nd':'adb shell input tap 350 320',
        'start_call':'adb shell input tap 60 1150',
        'accept_call':'adb shell input tap 125 1100',
        'reveal_buttons':'adb shell input tap 400 650',
        'disconnect_call':'adb shell input tap 330 1100'
    }

    touchCmds_1080P = {
        'select_contact_1st':'adb shell input tap 525 675',
        'select_contact_2nd':'adb shell input tap 525 480',
        'start_call':'adb shell input tap 90 1725',
        'accept_call':'adb shell input tap 188 1650',
        'reveal_buttons':'adb shell input tap 600 975',
        'disconnect_call':'adb shell input tap 495 1650'
    }

    touchCmds_25X16 = {
        'select_contact_1st':'adb shell input tap 778 900',
        'select_contact_2nd':'adb shell input tap 778 640',
        'start_call':'adb shell input tap 134 2300',
        'accept_call':'adb shell input tap 278 2200',
        'reveal_buttons':'adb shell input tap 889 1300',
        'disconnect_call':'adb shell input tap 734 2200'
    }

    trigger_messages = {
        'Start':'Displayed com.skype.raider/com.skype.android.app.main.HubActivity',
        'Chat':'Displayed com.skype.raider/com.skype.android.app.chat.ChatActivity',
        'Precall':'Displayed com.skype.raider/com.skype.android.app.calling.PreCallActivity',
        'Call':'Displayed com.skype.raider/com.skype.android.app.calling.CallActivity',
        'Release':'abandonAudioFocus() from android.media.AudioManager'
    }

    def __init__(self, phone):
        """
        Constructor.

        """
        BaseV2.__init__(self, phone)
        IVideoCallSkype.__init__(self, phone)
        self._logger = phone.get_logger()
        self._phone_system = PhoneSystem(phone)

        phone_resolution = self._phone_system.get_screen_resolution()
        y_resolution = phone_resolution.split('x', 1)[0]

        if y_resolution == '720':
            # Skype vertical_resolution command dictionary for the 720p" display
            self.touchCmds = VideoCallSkype.touchCmds_720P
        elif y_resolution == '1080':
            # Skype touch command dictionary for the 1080p display
            self.touchCmds = VideoCallSkype.touchCmds_1080P
        elif y_resolution == '1600':
            # Skype vertical_resolution command dictionary for the 720p" display
            self.touchCmds = VideoCallSkype.touchCmds_25x16
        else:
            self._logger.error('No supported display resolution {0}'.format(y_resolution))

    def stop_skype_app(self):
        """
        stop Skype app

        :return: None
        """
        cmd = "adb shell am force-stop com.skype.raider"
        self._exec(cmd)
        self._logger.debug( "Stop Skype on %s "%self._device )

    def start_skype_app(self):
        """
        start Skype app

        :return: None
        """
        # grant runtime permissions to skype
        self._device.grant_runtime_permissions("com.skype.raider")

        cmd = "adb shell am start com.skype.raider/.Main"
        self._exec(cmd)
        self._logger.debug( "Start Skype on %s "%self._device )

    def select_1st_contact_banner(self):
        """
        Select 1st entry on contact when banner is on screen.

        :return: None
        """
        # Start Skype on a phone
        cmd = self.touchCmds['select_contact_1st']
        self._exec(cmd)
        self._logger.debug( "Select Skype contact_1st on %s "%self._device )

    def select_1st_contact_nobanner(self):
        """
        Select 1st entry on contact when no banner is on screen.

        :return: None
        """
        # Start Skype on a phone
        cmd = self.touchCmds['select_contact_2nd']
        self._exec(cmd)
        self._logger.debug( "Select Skype contact_2nd on %s "%self._device )

    def start_call(self):
        """
        Dials a video call.

        :return: None
        """
        # Start Skype on a phone
        cmd = self.touchCmds['start_call']
        self._exec(cmd)
        self._logger.debug( "Start Skype call on %s "%self._device )

    def accept_call(self):
        """
        Accepts a video call.

        :return: None
        """
        # Start Skype on a phone
        cmd = self.touchCmds['accept_call']
        self._exec(cmd)
        self._logger.debug( "Accept Skype call on %s "%self._device )

    def reveal_buttons(self):
        """
        Reveal buttons on screen.

        :return: None
        """
        # Start Skype on a phone
        cmd = self.touchCmds['reveal_buttons']
        self._exec(cmd)
        self._logger.debug( "Reveal Skype buttons on %s "%self._device )

    def disconnect_call(self):
        """
        Disconnects a video call.

        :return: None
        """
        # Start Skype on a phone
        cmd = "adb shell am start com.skype.raider/.Main"
        self._exec(cmd)
        self._logger.debug( "Start Skype on %s "%self._device )

    def get_logcat_trigger_message(self, keyword):
        """
        Get a logcat trigger message corresponding to given operation.

        :return :None
        """
        if VideoCallSkype.trigger_messages.has_key(keyword):
            return VideoCallSkype.trigger_messages[keyword]
        else:
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, "VideoCallSkype does not support {0} operation".format(keyword))
