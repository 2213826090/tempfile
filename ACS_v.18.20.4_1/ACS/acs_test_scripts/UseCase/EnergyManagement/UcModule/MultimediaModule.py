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
:summary: This module represent multimedia module allowing you to interact video/audio of your DUT.
:author: vgomberx
:since: 25/11/2013
"""
from time import sleep
from ErrorHandling.AcsBaseException import AcsBaseException
from acs_test_scripts.UseCase.EnergyManagement.UcModule.OverMind import OverMind
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT


class MultimediaModule():
    """
    init.
    """
    __LOG_TAG = "[MULTIMEDIA_MODULE]\t"

    def __init__(self, mode=None):
        """
        parameter to initialize this module.
        currently consider that you cant have video and music playing at the same time
        that why testcase parameters for both are the same.

        :type mode: str
        :param mode: choose the mode to use with this module, 'audio' or 'video'
        """
        # DECLARE OPTIONAL GLOBAL PARAMETER HERE
        overmind = OverMind()
        self.__logger = LOGGER_TEST_SCRIPT
        self.__logger.info(self.__LOG_TAG + "INIT")
        self.__device = overmind.get_instance(overmind.DEVICE)
        tc_parameters = overmind.get_instance(overmind.TC_PARAMETERS)

        #-----------------------------------------------------------------------
        # Audio and video
        self.__multimedia_file = tc_parameters.get_param_value("MULTIMEDIA_FILE")
        self.__volume = tc_parameters.get_param_value("VOLUME", 100, default_cast_type=int)

        if self.__multimedia_file is not None:
            self.__multimedia_file = self.__device.multimedia_path + self.__multimedia_file

        # get multimedia uecmd
        self.multimedia_api = None
        if mode is not None:
            self.multimedia_api = self.__device.get_uecmd(str(mode).capitalize(), True)
        self.__system_api = self.__device.get_uecmd("System", True)
        self.__phonesystem_api = self.__device.get_uecmd("PhoneSystem", True)

    def is_media_defined(self):
        """
        return if a media has been well defined for this module.
        this does not imply that the media exist on the DUT.
        Used by load system.

        :rtype: boolean
        :return: True if a medio has been defined , False otherwise
        """
        result = False
        if not self.__multimedia_file in [None, "", "NONE"]:
            result = True

        return result

    def configure_media_file(self, media_path, media_volume=100):
        """
        configure the media to use.
        """
        self.__logger.debug(self.__LOG_TAG + "Configure local media to use : %s, at volume %s" % (media_path, media_volume))
        if not media_path in [None, "", "NONE"]:
            self.__multimedia_file = self.__device.multimedia_path + media_path
            self.__volume = int(media_volume)

    def set_media_to_use(self, mode):
        """
        define the media to be used

        :type mode: str
        :param mode: choose the mode to use with this module, 'audio' or 'video'
        """
        self.multimedia_api = self.__device.get_uecmd(str(mode).capitalize(), True)

    def start_media(self, infinite_loop=True):
        """
        start a media
        """
        self.__logger.info(self.__LOG_TAG + "Starting media")
        self.__phonesystem_api.wake_screen()
        sleep(0.1)
        self.__system_api .adjust_specified_stream_volume("Media", self.__volume)
        self.multimedia_api.play(self.__multimedia_file, loop=infinite_loop)

    def stop_media(self):
        """
        stop the running media
        """
        self.__logger.info(self.__LOG_TAG + "Stopping media")
        self.multimedia_api.stop()

    def is_media_running(self, restart_in_case_of_stop=False):
        """
        check the running status

        :rtype: boolean
        :return: True if running, false else.
        """
        self.__logger.info(self.__LOG_TAG + "Get media state")
        final_status = False
        try:
            final_status, _ = self.multimedia_api.is_playing()
        except AcsBaseException as e:
            self.__logger.error(self.__LOG_TAG + "error when reading multimedia playing state :%s" % str(e))

        # restart multimedia----------------------------------------------------
        if not final_status:
            # workaround to force the view to be reset
            if restart_in_case_of_stop:
                self.__phonesystem_api.wake_screen()
                # play music by restarting the player
                self.__system_api .adjust_specified_stream_volume("Media", self.__volume)
                self.multimedia_api.play(self.__multimedia_file, True)
                final_status = self.multimedia_api.is_playing()[0]

        return final_status
