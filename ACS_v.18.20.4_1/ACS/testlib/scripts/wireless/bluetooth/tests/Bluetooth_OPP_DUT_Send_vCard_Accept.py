#!/usr/bin/env python

#######################################################################
#
# @filename:    Bluetooth_OPP_serial_Send_vCard_Accept.py
# @description: serial sends vCard and smartphone accepts the request
# @author:      costin.carabas@intel.com
#
#######################################################################

import sys

from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.wireless.bluetooth import bluetooth_steps_old
from testlib.utils.ui.uiandroid import UIDevice as ui_device

# Connect to device
from testlib.base.base_utils import get_args

globals().update(vars(get_args(sys.argv)))


PAIRING_DEVICE = script_args[0]
DUT_NAME = 'DUT'
PAIRING_DEVICE_NAME = 'Smartphone'
BLUETOOTH_PATH = '/storage/emulated/0/bluetooth/'
PHOTO_URL = 'http://www.mobileworldmag.com/wp-content/uploads/2013/11/intel-logo.jpg'
PHOTO_NAME = 'intel-logo.jpg'
PHOTO_PATH = 'storage/emulated/0/DCIM/'
REFRESH_MEDIA_COMMAND = 'am broadcast -a android.intent.action.MEDIA_MOUNTED -d file:///'
CONTACT_NAME = 'Contact Test 1'

########### Preconditions ###############
#########################################
uidut = ui_device(serial = serial, port = adb_server_port)
uidevice = ui_device(serial = PAIRING_DEVICE, port = adb_server_port)

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
bluetooth_steps_old.bt_make_discoverable(serial = serial,
                                     port = adb_server_port,
                                     name = DUT_NAME
                                    )()

bluetooth_steps_old.bt_change_device_name(serial = PAIRING_DEVICE,
                                      port = adb_server_port,
                                      name = PAIRING_DEVICE_NAME
                                     )()
bluetooth_steps_old.bt_make_discoverable(serial = PAIRING_DEVICE,
                                     port = adb_server_port,
                                     name = PAIRING_DEVICE_NAME
                                    )()

#remove existing contact with the same name
ui_steps.press_home(serial = PAIRING_DEVICE)()
ui_steps.open_app_from_allapps(serial = PAIRING_DEVICE,
                               view_to_find = {'text': 'People'}
                              )()

while uidevice(text = CONTACT_NAME).exists:
    ui_steps.click_button(serial = PAIRING_DEVICE,
                          port = adb_server_port,
                          view_to_find = {
                                'descriptionContains': 'More options'
                                         },
                          view_to_check = {'text': 'Delete'}
                         )()
    ui_steps.click_button(serial = PAIRING_DEVICE,
                          port = adb_server_port,
                          view_to_find = {'text': 'Delete'},
                          view_to_check = {'text': 'OK'}
                         )()
    ui_steps.click_button(serial = PAIRING_DEVICE,
                          port = adb_server_port,
                          view_to_find = {'text': 'OK'}
                         )()

############### Test ####################
#########################################

ui_steps.press_home(serial = serial)()
ui_steps.open_app_from_allapps(serial = serial,
                               view_to_find = {'text': 'People'}
                              )()

#select a contact
if not uidut(text = CONTACT_NAME).exists:
    ui_steps.click_button(serial = serial,
                          port = adb_server_port,
                          view_to_find = {'descriptionContains': 'Add Contact'},
                          view_to_check = {'text': 'Name'}
                        )()
    ui_steps.click_button(serial = serial,
                          port = adb_server_port,
                          view_to_find = {'text': 'Name'},
                          view_to_check = {'text': 'Name'}
                         )()
    ui_steps.edit_text(serial = serial,
                       port = adb_server_port,
                       view_to_find = {'text' : 'Name'},
                       value = CONTACT_NAME
                      )()
    ui_steps.click_button(serial = serial,
                          port = adb_server_port,
                          view_to_find = {'text': 'Done'},
                          view_to_check = {'descriptionContains': 'Add Contact'}
                          )()

bluetooth_steps_old.bt_share_contact(serial = serial,
                                 port = adb_server_port,
                                 contact_name = CONTACT_NAME
                                )()

bluetooth_steps_old.open_bluetooth_settings(serial = PAIRING_DEVICE,
                                        port = adb_server_port
                                       )()
ui_steps.click_switch(serial = PAIRING_DEVICE,
                      port = adb_server_port,
                      print_error = "Error - Settings page was not displayed",
                      blocking = True,
                      view_to_find = {'className': "android.widget.Switch"},
                      state = "ON"
                     )()

ui_steps.click_button(serial = serial,
                      port = adb_server_port,
                      view_to_find = {'text': PAIRING_DEVICE_NAME}
                     )()
bluetooth_steps_old.bt_accept_contact(serial = PAIRING_DEVICE,
                                  port = adb_server_port,
                                  contact_name = CONTACT_NAME
                                 )()


########### Postconditions ##############
#########################################

adb_steps.disconnect_device(
    serial = serial,
    local_port = adb_server_port
)()

adb_steps.disconnect_device(
    serial = PAIRING_DEVICE,
    local_port = adb_server_port
)()
