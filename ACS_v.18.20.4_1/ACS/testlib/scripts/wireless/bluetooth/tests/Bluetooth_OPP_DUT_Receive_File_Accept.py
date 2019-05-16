#!/usr/bin/env python

#######################################################################
#
# @filename:    Bluetooth_OPP_DUT_Receive_File_Accept.py
# @description: DUT receives file and accepts the request
# @author:      costin.carabas@intel.com
#
#######################################################################

import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.ui import ui_utils
from testlib.scripts.wireless.bluetooth import bluetooth_steps_old
from testlib.scripts.android.ui.notification import steps
from testlib.scripts.connections.local import local_steps
from testlib.utils.ui.uiandroid import UIDevice as ui_device
from testlib.utils.connections import adb

# Connect to device
from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))

PAIRING_DEVICE = script_args[0]
DUT_NAME = 'DUT'
PAIRING_DEV_NAME = 'Smartphone'
BLUETOOTH_PATH = '/storage/emulated/0/bluetooth/'
PHOTO_URL = 'http://www.mobileworldmag.com/wp-content/uploads/' \
            '2013/11/intel-logo.jpg'
PHOTO_NAME = 'intel-logo.jpg'
PHOTO_PATH = 'storage/emulated/0/DCIM/'
REFRESH_MEDIA_COMMAND = 'am broadcast -a android.intent.action.MEDIA_MOUNTED '\
                        '-d file:///'


########### Preconditions ###############
#########################################

adb_steps.connect_device(
    serial = serial,
    local_port = adb_server_port,
)()

adb_steps.connect_device(
    serial = PAIRING_DEVICE,
    local_port = adb_server_port,
)()

bluetooth_steps_old.bt_change_device_name(serial = serial,
                                      port = adb_server_port,
                                      name = DUT_NAME
                                     )()
bluetooth_steps_old.bt_change_device_name(serial = PAIRING_DEVICE,
                                      port = adb_server_port,
                                      name = PAIRING_DEV_NAME
                                     )()

#push photo
download_command = 'wget ' + PHOTO_URL
local_steps.command(command = download_command)()
adb_steps.push_file(serial = PAIRING_DEVICE,
                    port = adb_server_port,
                    local = PHOTO_NAME,
                    remote = PHOTO_PATH + PHOTO_NAME
                   )()
adb_steps.command(serial = PAIRING_DEVICE,
                  port = adb_server_port,
                  command = REFRESH_MEDIA_COMMAND + PHOTO_PATH + PHOTO_NAME
                 )()


############### Test ####################
#########################################

ui_steps.open_picture_from_gallery(serial = PAIRING_DEVICE,
                                   port = adb_server_port
                                  )()

while not ui_device(serial = PAIRING_DEVICE,
                             port = adb_server_port
                            )(descriptionContains = 'Share with').exists:
    ui_steps.click_button(serial = PAIRING_DEVICE,
                          port = adb_server_port,
                          view_to_find = {
                                'className': 'android.widget.FrameLayout'
                                         }
                         )()

bluetooth_steps_old.bt_share_picture(serial = PAIRING_DEVICE,
                                 port = adb_server_port
                                )()

bluetooth_steps_old.open_bluetooth_settings(serial = serial,
                                        port = adb_server_port
                                       )()
ui_steps.click_switch(serial = serial,
                      port = adb_server_port,
                      print_error = "Error - Settings page was not displayed",
                      blocking = True,
                      view_to_find = {'className': "android.widget.Switch"},
                      view_to_check = {'text': DUT_NAME},
                      state = "ON"
                     )()

ui_steps.click_button(serial = PAIRING_DEVICE,
                      port = adb_server_port,
                      view_to_find = {'text': DUT_NAME}
                     )()
bluetooth_steps_old.bt_accept_picture(serial = serial,
                                  port = adb_server_port,
                                  path = BLUETOOTH_PATH
                                 )()

########### Postconditions ##############
#########################################

#clear photos
local_steps.command(command = 'rm ' + PHOTO_NAME)()
adb_steps.command(serial = PAIRING_DEVICE,
                  port = adb_server_port,
                  command = 'rm ' + PHOTO_PATH + PHOTO_NAME
                 )()
adb_steps.command(serial = serial,
                  port = adb_server_port,
                  command = 'rm ' + BLUETOOTH_PATH + PHOTO_NAME
                 )()

#refresh media
adb_steps.command(serial = serial,
                  port = adb_server_port,
                  command = REFRESH_MEDIA_COMMAND + PHOTO_PATH + PHOTO_NAME
                 )()
adb_steps.command(serial = PAIRING_DEVICE,
                  port = adb_server_port,
                  command = REFRESH_MEDIA_COMMAND + PHOTO_PATH + PHOTO_NAME
                 )()

adb_steps.disconnect_device(
    serial = serial,
    local_port = adb_server_port
)()

adb_steps.disconnect_device(
    serial = PAIRING_DEVICE,
    local_port = adb_server_port
)()
