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
:summary: Perform a MO Action depending of supported capabilities. Actions are VoiceCall, Sms or ping.
:since: 11/03/2015
:author: gcharlex
"""
from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
import time

from ErrorHandling.AcsBaseException import AcsBaseException
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import str_to_bool, Global
from acs_test_scripts.Utilities.SmsUtilities import SmsMessage


class PerformSupportedMoAction(DeviceTestStepBase):

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor
        """

        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)

        self._voice_call_api = self._device.get_uecmd("VoiceCall")
        self._networking_api = self._device.get_uecmd("Networking")
        self._equipment_manager = self._factory.create_equipment_manager()

    def run(self, context):
        """
        Runs the test step

        :type context: TestStepContext
        :param context: test case context
        """

        DeviceTestStepBase.run(self, context)

        is_speech_call_supported = str_to_bool(self._device.config["isSpeechCallSupported"])
        is_sms_supported = str_to_bool(self._device.config["isSmsSupported"])

        if is_speech_call_supported:
            self._voice_call_api.dial(self._pars.destination_number, False)
            time.sleep(15)
            self._voice_call_api.release()
        else:
            if is_sms_supported:
                sms = context.get_info(self._pars.sms)
                sms.send_sms()
                (status, msg) = sms.get_sms()
                self._logger.info(msg)
                if status == Global.FAILURE:
                    raise DeviceException(DeviceException.SMS_EXCEPTION, msg)
            else:
                packet_loss = self._networking_api.ping(self._pars.destination_ip, 32, 1)
                if packet_loss.value > 0:
                    raise AcsBaseException(AcsBaseException.OPERATION_FAILED,
                                           "Packet lost greater than zero (value=%.0f%%)"
                                           % packet_loss.value)
