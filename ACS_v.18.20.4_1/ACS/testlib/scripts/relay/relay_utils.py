#!/usr/bin/env python

########################################################################
#
# @filename:    relay_utils.py
# @description: Relay tests helper functions
# @author:      aurel.constantin@intel.com
#
#######################################################################

##### imports #####
import time

def select_menu_item(relay, option):

    """ description:


        tags:
            utils, crashmode, fastboot, power, ros, android, relay, reboot
    """
    count = 0
    while count < int(option):
        relay.press_volume_down()
        time.sleep(0.5)
        count += 1


def select_ros_menu_item(relay, mode = "android"):

    """ description:


        tags:
            utils, recovery, android, relay, reboot
    """
    select_menu_item(relay, mode)


def select_fastboot_menu_item(relay, option):

    """ description:


        tags:
            utils, fastboot, android, relay, reboot
    """
    select_menu_item(relay, option)


def select_crashmode_menu_item(relay, option):

    """ description:


        tags:
            utils, crashmode, android, relay, reboot
    """
    select_menu_item(relay, option)
