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
:summary: This script implements the angry birds application for power measurement
:since: 18/02/2013
:author: pbluniex
"""
import os
import time
import tempfile
import shutil
import tarfile

from acs_test_scripts.Device.Model.AndroidDevice.Application.IAndroidPackage import IAndroidPackage
from ErrorHandling.AcsConfigException import AcsConfigException


class AngryBirds(IAndroidPackage):

    """
    Implementation of Angry Birds to be driven
    """
    __events = {"play": "play.evt",
                "select_game": "game.evt",
                "select_game_level": "level.evt",
                "validate": "validate.evt",
                "shoot": "shoot.evt"}

    def __init__(self, device):
        """
        Initializes this instance.

        Nothing to be done in abstract class.
        """
        IAndroidPackage.__init__(self, device)

    def __load_event(self, event, rawevent):
        """
        Load a single event in the dictionary
        """
        cmd = ""
        for line in rawevent.splitlines():
            data = line.split(" ")
            if len(data) == 0:
                continue

            if len(data) != 3:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "Bad format of action : '%s'" % str(data))
            try:
                key = int(data[0], 16)
                action = int(data[1], 16)
                value = int(data[2], 16)
            except ValueError as exp:
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER,
                                         "Bad format of action : '%s' (%s)" %
                                        (str(data), str(exp)))

            cmd = "%ssendevent %s %d %d %d\n" % (cmd, self._touch_screen_event_file,
                                                 key, action, value)

        self.__events[event] = cmd

    def __load_events_table(self, eventfile):
        """
        Load event table used in the game
        This method replaces the content of the dictionary
        """
        if not "events_loaded" in self.__events.keys():
            events_dir = tempfile.mkdtemp()
            tar_file = tarfile.open(eventfile)
            tar_file.extractall(events_dir)

            for event in self.__events:
                filename = os.path.join(events_dir, self.__events[event])
                f_events = open(filename)
                filecontent = f_events.read()

                self.__load_event(event, filecontent)

            self.__events["events_loaded"] = 1
            shutil.rmtree(events_dir)

    def __play_event(self, event, sleep=5, timeout=60):
        """
        Play event in the dictionary
        """
        self.adb_shell(self.__events[event], timeout)
        time.sleep(sleep)

    def post_install(self):
        """
        Post install configuration for the application
        """
        self.__load_events_table(self._additionnals)

    def drive(self):
        """
        Actions to drive the application
        """
        # Wait until angry birds has loaded
        time.sleep(5)

        self.__play_event("play")
        self.__play_event("select_game")
        self.__play_event("select_game_level", 20)
        self.__play_event("validate")

        embd_script = "#!/system/bin/sh\n"\
                      "function shoot()\n"\
                      "{\n"\
                      "%s\n"\
                      "sleep 8\n"\
                      "}\n"\
                      "sleep 10\n"\
                      "shoot\n"\
                      "shoot\n"\
                      "shoot\n" % self.__events["shoot"]

        self.adb_shell("echo '%s' > /data/angrybirds.sh" % embd_script, 5)
        self.adb_shell("chmod 755 /data/angrybirds.sh", 5)
        self.adb_shell("(/data/angrybirds.sh &) &", 2)

    def uninstall(self):
        """
        Uninstall application
        """
        IAndroidPackage.uninstall(self)

        self.adb_shell("rm /data/angrybirds.sh", 5)
