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
:summary: This file implements the Live Wifi Audio Stream Quality UC
:since: 17/02/2014
:author: jcoutox
"""
import time
from acs_test_scripts.UseCase.Networking.LIVE_WIFI_WEB_BROWSING import LiveWifiWebBrowsing
from acs_test_scripts.UseCase.Networking.LIVE_WIFI_BASE import LiveWifiBase
from UtilitiesFWK.Utilities import Global
import acs_test_scripts.Lib.ImageCheck.Imagecheck as Imagecheck
import acs_test_scripts.Lib.AudioCheck.AudioCheck as AudioCheck
import os


class LiveWifiAudioStreamQuality(LiveWifiWebBrowsing):
    """
    Live wifi audio streaming quality class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveWifiWebBrowsing.__init__(self, tc_name, global_config)

        self._image_filename = "DUTScreenshot"
        self._screenshot_path = ""
        self._screenshot_state = False
        self._host_record_file = None
        self._host_record_device = None
        self._flag_record = False

        #Read different value from testcase.
        self._library_image_path = self._tc_parameters.get_param_value("LIBRARY_IMAGE_PATH")
        self._volume = int(self._tc_parameters.get_param_value("VOLUME"))
        self._length = int(self._tc_parameters.get_param_value("LENGTH"))
        self._sequence = self._tc_parameters.get_param_value("SEQUENCE")
        self._play_picture = self._tc_parameters.get_param_value("PLAY_PICTURE")
        self._screen_orientation = self._tc_parameters.get_param_value("SCREEN_ORIENTATION")

        #Image library
        self._dic_image = Imagecheck.generate_dictionary_image_path(self._library_image_path)

        # Generate the audio record name for each playback, self._host_record_file
        self._host_record_file = self.generate_audio_record_name()

        # Get UECmdLayer
        self._image_api = self._device.get_uecmd("Image")
        self._display_api = self._device.get_uecmd("Display")
        self._audio_recorder_api = self._device.get_uecmd("AudioRecorderHost")
        self._system_api = self._device.get_uecmd("System")
        self._phone_system_api = self._device.get_uecmd("PhoneSystem")

        #Store initial audio output
        self._original_audio_output = self._phone_system_api.get_audio_output()
        time.sleep(self._wait_btwn_cmd)

    #------------------------------------------------------------------------------

    def set_up(self):
        """
        Initialize the test
        """
        LiveWifiWebBrowsing.set_up(self)

        #Force screen orientation
        self._display_api.set_display_orientation(self._screen_orientation)
        time.sleep(self._wait_btwn_cmd)

        #Switch audio output on headset
        self._phone_system_api.switch_audio_output("headset")
        time.sleep(self._wait_btwn_cmd)

        #Adjust media volume
        self._system_api.adjust_specified_stream_volume("Media", self._volume)
        time.sleep(self._wait_btwn_cmd)

        # Open the browser and load the url before timeout
        (self._error.Code, self._error.Msg) = self._networking_api.open_web_browser(self._website_url,
                                                                                    self._browser_type,
                                                                                    self._webpage_loading_timeout)

        return self._error.Code, self._error.Msg

    #------------------------------------------------------------------------------

    def run_test(self):
        """
        Execute the test
        """

        LiveWifiBase.run_test(self)

        seq_string = ""

        #Take a screenshot and pull on host
        self._screenshot_path = self._image_api.take_screenshot_and_pull_on_host(self._image_filename, os.getcwd())
        self._screenshot_state = True

        #Launch record on host
        self._audio_recorder_api.pc_audio_record_start(self._host_record_file, self._host_record_device)
        self._flag_record = True

        #Tap on play button
        self._logger.info("Play the video...")
        self._image_api.touch_template_on_screen(self._screenshot_path, self._dic_image[self._play_picture])

        self._logger.info("Waiting for the audio to complete...")
        time.sleep(self._length + self._wait_btwn_cmd)

        #Stop record
        self._audio_recorder_api.pc_audio_record_stop()
        self._logger.info("Audio record saved at " + self._host_record_file)
        self._flag_record = False

        self._logger.info("Apply FFT on WAV file for extract frequency.")
        freq = AudioCheck.fft_on_wav_file(self._host_record_file)

        self._logger.info("Replace remarkable frequency by note and extract sequence.")
        seq = AudioCheck.extract_sequence(freq)

        #Create str with sequence
        for i in seq:
            seq_string = seq_string + i + " "

        if self._sequence in seq_string:
            verdict = Global.SUCCESS
            msg = "No errors"
        else:
            verdict = Global.FAILURE
            msg = "Play and record sound are not identical."

        return verdict, msg

    #------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        LiveWifiWebBrowsing.tear_down(self)

        # Close the browser
        self._networking_api.close_web_browser(self._browser_type)
        time.sleep(self._wait_btwn_cmd)

        #Adjust media volume at 50%
        self._system_api.adjust_specified_stream_volume("Media", 50)
        time.sleep(self._wait_btwn_cmd)

        #Switch audio output on headset
        self._phone_system_api.switch_audio_output(self._original_audio_output)
        time.sleep(self._wait_btwn_cmd)

        #Set screen orientation in auto mode
        self._display_api.set_display_orientation("auto")
        time.sleep(self._wait_btwn_cmd)

        if self._flag_record:
            #Stop record
            self._audio_recorder_api.pc_audio_record_stop()
            self._logger.info("Audio record saved at " + self._host_record_file)

        if self._screenshot_state:
            #Delete screenshot on the host
            self._logger.info("Delete the screenshot on host.")
            os.remove(self._screenshot_path)

        return Global.SUCCESS, "No errors"

    def generate_audio_record_name(self):
        """
        generate the audio record file path
        """
        uc_directory = os.path.join("AudioRecord", self._name + "_" + time.strftime("%Y-%m-%d_%Hh%M.%S"))
        record_directory = self._device.get_report_tree().create_subfolder(uc_directory)
        # generate the path
        return os.path.join(record_directory, "audiorc.wav")
