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
:summary: This module implements check of keytone frequancy for each number
:since: 21/01/14
:author: jcoutox
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
from UtilitiesFWK.Utilities import Global
from UtilitiesFWK.Utilities import str_to_dict
import acs_test_scripts.Lib.ImageCheck.Imagecheck as Imagecheck
import acs_test_scripts.Lib.AudioCheck.AudioCheck as AudioCheck
import os
import time


class LiveAudioKeytone(UseCaseBase):
    """
    Class audio check Keytone.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        # Call UseCase base Init function
        UseCaseBase.__init__(self, tc_name, global_config)

        self._image_filename = "DUTScreenshot"
        self._screenshot_state = False
        self._screenshot_path = ""
        self._host_record_file = None
        self._host_record_device = None
        self._number_dict = {}
        self._flag_record = False

        #Read the path of image library from testcase
        self._library_image_path = self._tc_parameters.get_param_value("LIBRARY_IMAGE_PATH")

        #Read value of different image from testcase
        self._keypad_icon = self._tc_parameters.get_param_value("KEYPAD")
        self._number_dict = str_to_dict(self._tc_parameters.get_param_value("NUMBER_DICTIONARY"))

        #Read value of sequence from testcase
        self._sequence = self._tc_parameters.get_param_value("SEQUENCE")
        self._sequence_list = self._sequence.split(" ")

        #Image library
        self._dict_image = Imagecheck.generate_dictionary_image_path(self._library_image_path)

        #Generate the audio record name for each playback, self._host_record_file
        self._host_record_file = self.generate_audio_record_name()

        #Get UECmdLayer
        self._image_api = self._device.get_uecmd("Image")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")
        self._keyevent_api = self._device.get_uecmd("KeyEvent")
        self._audio_record_api = self._device.get_uecmd("AudioRecorderHost")
        self._system_api = self._device.get_uecmd("System")

        #Store initial Sleep Timeout value
        self._initial_sleep_timeout_value = self._phone_system_api.get_screen_timeout()
        time.sleep(self._wait_btwn_cmd)

        #Store initial audio output
        self._original_audio_output = self._phone_system_api.get_audio_output()
        time.sleep(self._wait_btwn_cmd)

    def set_up(self):
        """
        set up
        """
        UseCaseBase.set_up(self)

        #Wake the screen
        self._phone_system_api.display_on()
        time.sleep(self._wait_btwn_cmd)

        #Unlock the screen
        self._phone_system_api.set_phone_lock(0)
        time.sleep(self._wait_btwn_cmd)

        #Set sleep timeout to it's maximum value
        self._phone_system_api.set_screen_timeout(1800)
        time.sleep(self._wait_btwn_cmd)

        #Switch audio output on headset
        self._phone_system_api.switch_audio_output("headset")
        time.sleep(self._wait_btwn_cmd)

        #Adjust ringtones volume at maximum
        self._system_api.adjust_specified_stream_volume("Ringtone", 100)
        time.sleep(self._wait_btwn_cmd)

        #Check if Dialer application is already launch and close it in this case.
        self._system_api.kill_Dialer_app_if_already_launch()
        time.sleep(self._wait_btwn_cmd)

        #Launch Dialer
        self._system_api.open_Dialer()
        time.sleep(self._wait_btwn_cmd)

        #Take screenshot and pull on host.
        self._screenshot_path = self._image_api.take_screenshot_and_pull_on_host(self._image_filename, os.getcwd())
        self._screenshot_state = True

        #Open Keypad
        self._image_api.touch_template_on_screen(self._screenshot_path, self._dict_image[self._keypad_icon])
        time.sleep(self._wait_btwn_cmd)

        return Global.SUCCESS, "No Errors"

    def run_test(self):
        """
        Execute the test
        """

        #Call UseCase base Run function.
        UseCaseBase.run_test(self)

        verdict = Global.SUCCESS
        msg = ""

        #Launch record on host
        self._audio_record_api.pc_audio_record_start(self._host_record_file, self._host_record_device)
        self._flag_record = True

        #Take screenshot and pull on host.
        self._screenshot_path = self._image_api.take_screenshot_and_pull_on_host(self._image_filename, os.getcwd())
        self._screenshot_state = True

        #Touch every number
        self._logger.info("Touch every number...")
        for num in self._sequence_list:
            self._image_api.touch_template_on_screen(self._screenshot_path, self._dict_image[self._number_dict[num]])

        #Stop record
        self._audio_record_api.pc_audio_record_stop()
        self._logger.info("Audio record saved at " + self._host_record_file)
        self._flag_record = False

        #Apply DTMFdetector on record file
        #Create DTMFdetector object
        dtmf = AudioCheck.DTMFdetector()

        self._logger.info("Extract keytone sequence from audio record file...")
        #Get dtmf sequence
        sequence = dtmf.getDTMFfromWAV(self._host_record_file)

        if sequence == self._sequence_list:
            verdict = Global.SUCCESS
            msg = "No errors"
        else:
            if len(sequence) > 0:
                for num in self._sequence_list:
                    if num not in sequence:
                        verdict = Global.FAILURE
                        msg += "Doesn't found DTMF code of " + num +", "
            else:
                verdict = Global.FAILURE
                msg = "No DTMF code found."

        return verdict, msg

    def tear_down(self):
        """
        tear down
        """
        UseCaseBase.tear_down(self)

        if self._flag_record:
            #Stop record
            self._audio_record_api.pc_audio_record_stop()
            self._logger.info("Audio record saved at " + self._host_record_file)

        if self._screenshot_state:
            #Delete screenshot on the host
            self._logger.info("Delete the screenshot on host.")
            os.remove(self._screenshot_path)


        #Set initial sleep timeout
        self._phone_system_api.set_screen_timeout(self._initial_sleep_timeout_value)
        time.sleep(self._wait_btwn_cmd)

        #Switch audio output on headset
        self._phone_system_api.switch_audio_output(self._original_audio_output)
        time.sleep(self._wait_btwn_cmd)

        #Adjust ringtones volume at 50%
        self._system_api.adjust_specified_stream_volume("Ringtone", 50)
        time.sleep(self._wait_btwn_cmd)

        #Force home screen
        self._system_api.kill_Dialer_app()

        #Lock the screen
        self._phone_system_api.set_phone_lock(1)
        time.sleep(self._wait_btwn_cmd)
        self._keyevent_api.power_button()

        return Global.SUCCESS, "No errors"

    def generate_audio_record_name(self):
        """
        Generate the audio record file path
        """
        uc_directory = os.path.join("AudioRecord", self._name + "_" + time.strftime("%Y-%m-%d_%Hh%M.%S"))
        record_directory = self._device.get_report_tree().create_subfolder(uc_directory)
        #Generate the path
        return os.path.join(record_directory, "audiorc.wav")

