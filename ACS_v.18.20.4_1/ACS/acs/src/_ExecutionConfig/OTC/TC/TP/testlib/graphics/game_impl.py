# Copyright (C) 2015  Jin, YingjunX <yingjunx.jin@intel.com>
# Intel Corporation All Rights Reserved.

# The source code contained or described herein and
# all documents related to the source code ("Material") are owned by
# Intel Corporation or its suppliers or licensors.

# Title to the Material remains with Intel Corporation or
# its suppliers and licensors.
# The Material contains trade secrets and proprietary and
# confidential information of Intel or its suppliers and licensors.
# The Material is protected by worldwide copyright and
# trade secret laws and treaty provisions.
# No part of the Material may be used, copied, reproduced, modified,
#published, uploaded, posted, transmitted, distributed
# or disclosed in any way without Intel's prior express written permission.
# No license under any patent, copyright, trade secret or
# other intellectual property right is granted to
# or conferred upon you by disclosure or delivery of the Materials,
# either expressly, by implication, inducement, estoppel or otherwise.

# Any license under such intellectual property rights must be express
# and approved by Intel in writing.

'''
@summary: Class for ChromeCast operation
@since: 04/09/2015
@author: Yingjun Jin
'''


import time
from testlib.util.common import g_common_obj
from testlib.util.config import TestConfig
from testlib.util.repo import Artifactory
from testlib.graphics.tools import ConfigHandle
from testlib.util.log import Logger
from testlib.graphics.common import get_current_focus_window, windows_info
from testlib.graphics.compare_pic_impl import compare_pic


LOG = Logger.getlogger(__name__)
SPINTOPS_PACKAGE_NAME = "fi.Miksumortti.SpinTops"
SPINTOPS_ACTIVITY_NAME = ".Activity"

class Locator(object):

    """
    locator
    """

    def __init__(self, device):
        self.d = device

    @property
    def wait_exist(self):
        """ wait until the ui object exist """
        def _wait(uiobj, timeout=5):
            return uiobj.wait("exists", timeout * 500)
        return _wait

    @property
    def performance_tests(self):
        return self.d(text="Performance Tests")


class GameImpl:

    def __init__(self):
        self.d = g_common_obj.get_device()
        self._locator = Locator(self.d)

    def install_apk(self, apk_name):
        """ Install the apk from artifactory
        """
        config = TestConfig()
        cfg_file = 'tests.tablet.game.conf'
        cfg_arti = config.read(cfg_file, 'artifactory')
        config_handle = ConfigHandle()
        cfg_arti["location"] = config_handle.read_configuration('artifactory', 'location', '/etc/oat/', 'sys.conf')
        arti = Artifactory(cfg_arti.get('location'))
        cfg_apk = config.read(cfg_file, apk_name)
        apk_name = cfg_apk.get("name")
        file_path = arti.get(apk_name)
        g_common_obj.adb_cmd_common('install ' + file_path)

    def launch_airattack_am(self):
        """ Launch airattack via adb am command
        """
        print "Launch airattack by adb am"
        g_common_obj.launch_app_am(
            "dk.logisoft.airattack", "dk.logisoft.airattack.AirAttackActivity")
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)

    @staticmethod
    def stop_airattack_am():
        """ Stop airattack via adb am command
        """
        print "Stop airattack by adb am"
        g_common_obj.stop_app_am("dk.logisoft.airattack")

    @staticmethod
    def uninstall_airattack():
        """ uninstall the airattack
        """
        print "Uninstall the airattack"
        cmd = 'uninstall dk.logisoft.airattack'
        g_common_obj.adb_cmd_common(cmd)

    def play_airattack(self):
        """ play airattack
        """
        print "Playing airattack"
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        self.d.click(x * 1 / 4, y * 6 / 7)
        time.sleep(1)
        self.d(text="Maybe later").click()
        time.sleep(1)
        self.d.click(x * 1 / 4, y * 6 / 7)
        time.sleep(2)
        self.d.click(x * 3 / 10, y * 6 / 7)
        for _ in range(0, 10):
            import random
            pre_x = random.randint(0, x)
            pre_y = random.randint(0, y)
            self.d.click(pre_x, pre_y)
            time.sleep(3)

    def return_normal_screen(self):
        """ change back to normal screen
        """
        self.d.orientation = "n"

    def launch_candycrushsaga_am(self):
        """ Launch candycrushsaga via adb am command
        """
        print "Launch candycrushsaga by adb am"
        g_common_obj.launch_app_am(
            "com.king.candycrushsaga", "com.king.candycrushsaga.CandyCrushSagaActivity")
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(10)

    @staticmethod
    def stop_candycrushsaga_am():
        """ Stop candycrushsaga via adb am command
        """
        print "Stop candycrushsaga by adb am"
        g_common_obj.stop_app_am("com.king.candycrushsaga")

    @staticmethod
    def uninstall_candycrushsaga():
        """ uninstall the candycrushsaga
        """
        print "Uninstall the candycrushsaga"
        cmd = 'uninstall com.king.candycrushsaga'
        g_common_obj.adb_cmd_common(cmd)

    def play_candycrushsaga(self):
        """ play candycrushsaga
        """
        print "Playing candycrushsaga"
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        self.d.click(x * 5 / 9, y / 2)
        time.sleep(2)
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        self.d.click(x * 4 / 7, y * 7 / 9)
        time.sleep(2)
        self.d.click(x * 5 / 19, y * 4 / 5)
        time.sleep(2)
        self.d.click(x * 5 / 19, y * 4 / 5)
        time.sleep(2)
        self.d.click(x / 2, y * 3 / 5)
        time.sleep(5)
        self.d.click(x / 6, y * 9 / 10)
        time.sleep(2)
        for _ in range(0, 10):
            import random
            pre_x = random.randint(0, x)
            pre_y = random.randint(y / 4, y / 2)
            self.d.swipe(pre_x, pre_y, pre_x + 50, pre_y)
            time.sleep(3)

    def launch_doodlejump_am(self):
        """ Launch doodlejump via adb am command
        """
        print "Launch doodlejump by adb am"
        g_common_obj.launch_app_am(
            "com.lima.doodlejump", "com.limasky.doodlejumpandroid.MainActivity")
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(10)

    @staticmethod
    def stop_doodlejump_am():
        """ Stop doodlejump via adb am command
        """
        print "Stop doodlejump by adb am"
        g_common_obj.stop_app_am("com.lima.doodlejump")

    @staticmethod
    def uninstall_doodlejump():
        """ uninstall the doodlejump
        """
        print "Uninstall the doodlejump"
        cmd = 'uninstall com.lima.doodlejump'
        g_common_obj.adb_cmd_common(cmd)

    def play_doodlejump(self):
        """ play doodlejump
        """
        print "Playing doodlejump"
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        self.d.click(x * 2 / 3, y * 2 / 3)
        time.sleep(10)
        self.d.press.back()

    def launch_hillclimb_am(self):
        """ Launch hillclimb via adb am command
        """
        print "Launch hillclimb by adb am"
        g_common_obj.launch_app_am(
            "com.fingersoft.hillclimb", "com.fingersoft.game.MainActivity")
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)

    @staticmethod
    def stop_hillclimb_am():
        """ Stop hillclimb via adb am command
        """
        print "Stop hillclimb by adb am"
        g_common_obj.stop_app_am("com.fingersoft.hillclimb")

    @staticmethod
    def uninstall_hillclimb():
        """ uninstall the hillclimb
        """
        print "Uninstall the hillclimb"
        cmd = 'uninstall com.fingersoft.hillclimb'
        g_common_obj.adb_cmd_common(cmd)

    def play_hillclimb(self):
        """ play hillclimb
        """
        print "Playing hillclimb"
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        self.d.click(x * 4 / 5, y * 4 / 5)
        time.sleep(2)
        self.d.click(x * 4 / 5, y * 4 / 5)
        time.sleep(2)
        self.d.click(x * 4 / 5, y * 4 / 5)
        time.sleep(2)
        for _ in range(0, 10):
            import random
            pre_x = random.randint(0, x)
            pre_y = random.randint(y * 2 / 3, y)
            self.d.long_click(pre_x, pre_y)
            time.sleep(1)

    def launch_plumber_am(self):
        """ Launch Plumber via adb am command
        """
        print "Launch Plumber by adb am"
        g_common_obj.launch_app_am(
            "com.magmamobile.game.Plumber", "com.magmamobile.game.Plumber.ActivityPlumber")
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)
        if self.d(text="No").exists:
            self.d(text="No").click()
            time.sleep(2)

    @staticmethod
    def stop_plumber_am():
        """ Stop Plumber via adb am command
        """
        print "Stop Plumber by adb am"
        g_common_obj.stop_app_am("com.magmamobile.game.Plumber")

    @staticmethod
    def uninstall_plumber():
        """ uninstall the Plumber
        """
        print "Uninstall the Plumber"
        cmd = 'uninstall com.magmamobile.game.Plumber'
        g_common_obj.adb_cmd_common(cmd)

    def play_plumber(self):
        """ play Plumber
        """
        print "Playing Plumber"
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        self.d.click(x / 2, y * 4 / 9)
        time.sleep(2)
        self.d.click(x / 2, y * 4 / 9)
        time.sleep(5)
        self.d.click(x / 5, y * 8 / 17)
        time.sleep(5)
        self.d.click(x / 2, y * 3 / 4)
        time.sleep(2)
        for _ in range(0, 20):
            import random
            pre_x = random.randint(x / 5, x * 4 / 5)
            pre_y = random.randint(y / 4, y * 3 / 4)
            self.d.click(pre_x, pre_y)
            time.sleep(1)

    def launch_tower_am(self):
        """ Launch tower via adb am command
        """
        print "Launch tower by adb am"
        g_common_obj.launch_app_am(
            "com.tower3d", "com.unity3d.player.UnityPlayerNativeActivity")
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)

    @staticmethod
    def stop_tower_am():
        """ Stop tower via adb am command
        """
        print "Stop tower by adb am"
        g_common_obj.stop_app_am("com.tower3d")

    @staticmethod
    def uninstall_tower():
        """ uninstall the tower
        """
        print "Uninstall the tower"
        cmd = 'uninstall com.tower3d'
        g_common_obj.adb_cmd_common(cmd)

    def play_tower(self):
        """ play tower
        """
        print "Playing tower"
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        self.d.click(x / 2, y * 3 / 5)
        time.sleep(2)
        for _ in range(0, 10):
            import random
            pre_x = random.randint(x / 5, x * 4 / 5)
            pre_y = random.randint(y / 4, y * 3 / 4)
            self.d.click(pre_x, pre_y)
            time.sleep(1)

    def launch_osmosdemo_am(self):
        """ Launch osmosdemo via adb am command
        """
        print "Launch osmosdemo by adb am"
        g_common_obj.launch_app_am(
            "com.hemispheregames.osmosdemo", "com.apportable.activity.VerdeActivity")
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(10)

    @staticmethod
    def stop_osmosdemo_am():
        """ Stop osmosdemo via adb am command
        """
        print "Stop osmosdemo by adb am"
        g_common_obj.stop_app_am("com.hemispheregames.osmosdemo")

    @staticmethod
    def uninstall_osmosdemo():
        """ uninstall the osmosdemo
        """
        print "Uninstall the osmosdemo"
        cmd = 'uninstall com.hemispheregames.osmosdemo'
        g_common_obj.adb_cmd_common(cmd)

    def play_osmosdemo(self):
        """ play osmosdemo
        """
        print "Playing osmosdemo"
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        self.d.click(x / 5, y / 2)
        time.sleep(5)
        for _ in range(0, 15):
            import random
            pre_x = random.randint(x / 5, x * 4 / 5)
            pre_y = random.randint(y / 4, y * 3 / 4)
            self.d.click(pre_x, pre_y)
            time.sleep(3)

    def launch_spacewar_am(self):
        """ Launch SpaceWar via adb am command
        """
        print "Launch SpaceWar by adb am"
        g_common_obj.launch_app_am(
            "virtualgs.space", "virtualgs.space.SpaceWar")
        self._locator.wait_exist(self._locator.performance_tests)
        time.sleep(5)

    @staticmethod
    def stop_spacewar_am():
        """ Stop SpaceWar via adb am command
        """
        print "Stop SpaceWar by adb am"
        g_common_obj.stop_app_am("virtualgs.space")

    @staticmethod
    def uninstall_spacewar():
        """ uninstall the SpaceWar
        """
        print "Uninstall the SpaceWar"
        cmd = 'uninstall virtualgs.space'
        g_common_obj.adb_cmd_common(cmd)

    def play_spacewar(self):
        """ play SpaceWar
        """
        print "Playing SpaceWar"
        x = self.d.info["displayWidth"]
        y = self.d.info["displayHeight"]
        self.d.click(x / 2, y / 2)
        time.sleep(2)
        for _ in range(0, 8):
            import random
            pre_x = random.randint(x / 5, x * 4 / 5)
            pre_y = random.randint(y / 4, y * 3 / 4)
            self.d.click(pre_x, pre_y)
            time.sleep(1)

    def launch_spintops_am(self, wait_time=10):
        LOG.info("Launching Spin-Tops app.")
        g_common_obj.launch_app_am(SPINTOPS_PACKAGE_NAME, SPINTOPS_ACTIVITY_NAME)
        while wait_time > 0:
            windows_info.disable_fullscreen_hint()
            if SPINTOPS_PACKAGE_NAME in get_current_focus_window(): return True
            wait_time -= 1
            time.sleep(1)
        raise Exception("Launch Spin-Tops time out.")

    def init_spintops(self, retry=5):
        time.sleep(5)
        if self.d.click(1, 1): time.sleep(2)  # Touch to continue
        mid_x = self.d().bounds['right'] / 2
        mid_y = self.d().bounds['bottom'] / 2
        target_strs = ['and', 'able', 'Games', 'NO', 'YES']
        for r in range(1, retry + 1):
            strings = compare_pic.extract_strings_from_croped_screen_shot(0,
                                                                          mid_x - 200,
                                                                          mid_y - 200,
                                                                          mid_x + 200,
                                                                          mid_y + 200)
            if any(i in strings for i in target_strs):
                time.sleep(.5)
                for x in range(10):
                    self.d.click(mid_x * 0.9,
                                 mid_y + 10 * x) # Click NO button
                time.sleep(1)
                strings = compare_pic.extract_strings_from_croped_screen_shot(0,
                                                                              mid_x - 200,
                                                                              mid_y - 200,
                                                                              mid_x + 200,
                                                                              mid_y + 200)
                if not any(i in strings for i in target_strs):
                    LOG.info("Init Spin-Tops done.")
                    return True
            else:
                LOG.info("Not detect Google Play Game pop-up.")

    def play_spintops(self, stadium=1, retry=5):
        main_menu_number = 9 # Count main menu list, manual add one for empty area.
        menu_y = self.d.info['displayHeight'] / main_menu_number
        menu_x = self.d.info['displayWidth'] / 4
        if stadium == 1:
            k = 1.2
        elif stadium in [2, 3, 4]:
            k = 1.1
        elif stadium in [5, 6]:
            k = 0.7
        else:
            k = 0
        choose_stadium_y = int(menu_y * (stadium + k))
        play_y = self.d.info['displayHeight'] / main_menu_number
        play_x = self.d.info['displayWidth'] / 4 * 3
        swipe_y = self.d.info['displayHeight'] / 3
        swipe_x = self.d.info['displayWidth'] / 2
        if self.d.click(menu_x, menu_y * 2): time.sleep(3)
        LOG.debug("tap in %s, %s." % (menu_x, choose_stadium_y))
        if self.d.click(menu_x, choose_stadium_y): time.sleep(3) # Max is 7
        LOG.info("Choose stadium %s." % stadium )
        if self.d.click(play_x, play_y): time.sleep(2)
        for i in range(10):
            self.d.swipe(swipe_x, swipe_y, self.d.info['displayWidth'], swipe_y, steps=10)
            swipe_y += 3
        while retry > 0:
            time.sleep(1)
            retry -= 1
            strings = compare_pic.extract_strings_from_croped_screen_shot(0,0,0,120,120)
            try:
                rpm = int(strings.splitlines()[0][0])
                if rpm > 0:
                    LOG.info("Game is playing.")
                    return True
            except:
                LOG.debug("Did not detect RPM string, try again")
        raise Exception("Fail to play spintops.")

    def stop_spintops_am(self):
        LOG.info("Stop Spin-Tops app.")
        g_common_obj.stop_app_am(SPINTOPS_PACKAGE_NAME)


game_impl = GameImpl()