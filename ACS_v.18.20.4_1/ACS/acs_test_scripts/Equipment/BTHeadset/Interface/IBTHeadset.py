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
:summary: virtual interface for Bluetooth headset used with IO card RLY8USB
:author: cmichelx
:since: 19/02/2013
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException

# pylint: disable=W0613


class IBTHeadset(object):

    """
    Class IBTHeadset: virtual interface for bluetooth headset equipment
    """

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def get_bdaddress(self):
        """
        Returns bd address parameter of the equipment
        :rtype: str
        :return: BD address of the equipment. format 00:00:00:00:00:00
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_power(self, state):
        """
        Set BT headset power ON or OFF
        Control the on/off line
        :type state: boolean
        :param state: true to power ON, false to power OFF
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_discoverable(self):
        """
        Set BT headset in discoverable mode (able to be paired)
        Control the on/off line to switch to pairing mode
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def set_volume(self, state):
        """
        Set Volume up/down depend on state value
        Control the volume up line
        :type state: boolean
        :param state: true to volume UP, false to volume DOWN
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def pickup_call(self):
        """
        Control the call button line
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def redial_call(self):
        """
        Control the call button line with double short press
        to enable redial
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def voice_dial_call(self):
        """
        Start voice recognition with a long press on call button line
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def hangup_call(self):
        """
        Control the call button line
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def reject_call(self):
        """
        Control the call button line with double short press
        to reject an incoming call
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def play_pause_music_toggle(self):
        """
        Control the play/pause line to start/pause playing audio
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def stop_music(self):
        """
        Control the play/pause line to stop playing audio
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def next_audio_track(self):
        """
        Control the fwd line to jump to next track
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def previous_audio_track(self):
        """
        Control the rwd line to jump to previous track
        Press Rwd button twice
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def backward_audio_track(self):
        """
        Control the rwd line to backward track
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def fforward_audio(self, duration):
        """
        Control the fwd line to go ahead in the audio track
        :type duration: integer
        :param duration: the time we want to go ahead in the audio track
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)

    def frewind_audio(self, duration):
        """
        Control the rewind line to go back in the audio track
        :type duration: integer
        :param duration: the time we want to go back in the audio track
        """
        raise TestEquipmentException(TestEquipmentException.FEATURE_NOT_IMPLEMENTED)
