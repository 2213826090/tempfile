#!/usr/bin/env python

##### imports #####
import sys
from testlib.base import base_utils
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui.security import security_steps

##### initialization #####
globals().update(vars(base_utils.get_args(sys.argv)))
adb_steps.connect_device(serial = serial, port = adb_server_port)()

#### test start ####

security_steps.reboot_system(serial = serial)()
ui_steps.open_security_settings(serial = serial)()
security_steps.set_pin_screen_lock(serial = serial)()
security_steps.enter_incorrect_pin(serial = serial, dut_info = "bxt",
                                   dut_pin = "1234", tries = 10)()
security_steps.enter_correct_pin(serial = serial, dut_info = "bxt", dut_pin = "1234")()
security_steps.remove_pin_screen_lock(serial = serial)()

security_steps.reboot_system(serial = serial)()

ui_steps.open_security_settings(serial = serial)()
security_steps.set_password_screen_lock(serial = serial)()
security_steps.enter_incorrect_password(serial = serial, dut_info = "bxt",
										dut_pin = "test1234", tries = 10)()
security_steps.enter_correct_password(serial = serial, dut_info = "bxt", dut_pin = "test1234")()
security_steps.remove_password_screen_lock(serial = serial)()

security_steps.reboot_system(serial = serial)()

ui_steps.open_security_settings(serial = serial)()
security_steps.set_pattern_screen_lock(serial = serial, the_first_point = [780, 350],
									   the_second_point = [960, 350], the_third_point = [1135, 350],
									   the_fourth_point = [1135, 530])()
security_steps.enter_incorrect_pattern(serial = serial, the_first_point = [1150, 525],
									   the_second_point = [1150, 335], the_third_point = [960, 335],
									   the_fourth_point = [770, 335], dut_info = "bxt", tries = 10)()
security_steps.enter_correct_pattern(serial = serial, the_first_point = [770, 335],
									 the_second_point = [960, 335], the_third_point = [1150, 335],
									 the_fourth_point = [1150, 525], dut_info = "bxt")()
security_steps.remove_pattern_screen_lock(serial = serial, the_first_point = [805, 450],
										  the_second_point = [960, 450], the_third_point = [1115, 450],
										  the_fourth_point = [1115, 605])()

##### test end #####