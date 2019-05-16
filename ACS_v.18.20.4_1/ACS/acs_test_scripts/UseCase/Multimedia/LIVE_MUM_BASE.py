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

property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: This file implements the live multimedia base

:since: 28/06/2011
:author: fhu2
"""

from acs_test_scripts.UseCase.UseCaseBase import UseCaseBase
import time
import os
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsToolException import AcsToolException


class LiveMuMBase(UseCaseBase):

    """
    classdocs
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        UseCaseBase.__init__(self, tc_name, global_config)

        # Get UEcmd variable
        self._multimedia_api = None
        self._phone_system_api = None

        # Get common parameters here
        self._file_name = None
        self._volume = None
        self._length = None
        self._operate_sequence = None
        self._play_mode = None
        self._media_type = None

        # other video unique parameters add here
        self._disp_brightness = None

        # other audio unique parameters add here
        self._deviation = None
        self._original_ringtone_mode = None
        self._original_vibrate_mode = None
        self._silent_mode = None

        self._host_record = False
        self._host_record_file = None
        self._host_record_device = None

        self._host_play = False
        self._host_play_file = None
        self._host_play_device = None

        self._host_play = False
        self._host_play_file = None
        self._host_play_device = None

        # Get path to multimedia files
        self._multimedia_path = self._device.multimedia_path

        # Set BT prerequisite (set ON, set OFF or nothing) for FIT BT/WIFI tests
        self._bt_fit_used, self._bt_wished_value = self._get_bt_fit_config()
        # BT initial state
        self._bt_initial_state = 'STATE_ON'
        # Instantiate generic UECmd
        self._localconnectivity_api = self._device.get_uecmd("LocalConnectivity")

    def generate_audio_record_name(self):
        """
        generate the audio record file path
        """
        uc_directory = \
            os.path.join(
                "AudioRecord",
                self._name + "_" + time.strftime("%Y-%m-%d_%Hh%M.%S"))
        record_directory = \
            self._device.get_report_tree().create_subfolder(uc_directory)

        # generate the path
        return os.path.join(record_directory, "audiorc.wav")

    def generate_play_file(self):
        """
        if a local file, return file_path+filename
        if a remote file, just return filename
        """
        if self._file_name.find("://") > -1:
            return str(self._file_name)
        else:
            return str(self._multimedia_path + self._file_name)

    def set_up(self):
        UseCaseBase.set_up(self)

        # If Bt to be (de-)activated for FIT tests
        if self._bt_fit_used:

            self._bt_initial_state = self._localconnectivity_api.get_bt_power_status()

            # if Bt is not at the wished value, set it to the correct one.
            if self._bt_initial_state != self._bt_wished_value:
                self._localconnectivity_api.set_bt_power(self._bt_wished_value)
                time.sleep(self._wait_btwn_cmd)

    def run_test(self):
        UseCaseBase.run_test(self)

    def tear_down(self):
        UseCaseBase.tear_down(self)

        # If Bt to be (de-)activated for FIT tests
        if self._bt_fit_used:

            bt_final_status = self._localconnectivity_api.get_bt_power_status()

            # if Bt is not at the initial value, set it to the correct one.
            if bt_final_status != self._bt_initial_state:
                # restore Bt in initial state
                self._localconnectivity_api.set_bt_power(self._bt_initial_state)
                time.sleep(self._wait_btwn_cmd)

#------------------------------------------------------------------------------

    def _get_bt_fit_config(self):
        """
        Get BT configuration for FIT BT/WIFI tests

        :rtype: list of 2 elements (boolean, str)
        :return: true if BT/WIFI FIT is used, Bluetooth state ON or OFF for the test to be run
        """

        # Read WHILE_BLUETOOTH_ON parameter (named for retro-compatibility) from test case xml file for FIT tests
        param_while_bt_on = \
            str(self._tc_parameters.get_param_value("WHILE_BLUETOOTH_ON"))

        if param_while_bt_on.lower() in ["1", "on", "true", "yes"]:
            bt_fit_used = True
            bt_wished_value = 'STATE_ON'
        elif param_while_bt_on.lower() in ["0", "off", "false", "no"]:
            bt_fit_used = True
            bt_wished_value = 'STATE_OFF'
        else:
            bt_fit_used = False
            bt_wished_value = 'STATE_OFF'

        return bt_fit_used, bt_wished_value
#------------------------------------------------------------------------------
