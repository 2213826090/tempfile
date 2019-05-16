'''
    Copyright 2015 Android Open Source Project

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
'''

import os
from display_util import set_brightness
from display_util import user_validation
from display_util import display_on_screen
from util import validate_result
from util import menu
from util import print_to_console_and_logcat
from util import exit_test

VERDICT = FAILURE
OUTPUT = ""
BRIGHTNESS_LEVEL = TC_PARAMETERS('BRIGHTNESS')
check_brightness_message = "Please check the brightness of the display.\n"
white_color = 4
white_timeout = 10

if (BRIGHTNESS_LEVEL == "MAX"):
    brightness_value = 255
    check_brightness_message += "Is it very luminous? (y/n)"
elif (BRIGHTNESS_LEVEL == "MIN"):
    brightness_value = 0
    check_brightness_message += "Is it very dark? (y/n)"
else:
    OUTPUT = "Wrong parameter. BRIGHTNESS value should be MAX or MIN"
    print_to_console_and_logcat(OUTPUT)
    exit_test()

def check_brightness(arguments):
    global VERDICT, OUTPUT
    set_brightness(brightness_value)
    #set display color to be white for 10 seconds
    proc = display_on_screen(white_color, white_timeout)
    if user_validation():
        OUTPUT = "Success"
        print_to_console_and_logcat(OUTPUT)
        VERDICT = SUCCESS
    else:
        print_to_console_and_logcat(OUTPUT)
        OUTPUT = "Failure. Display brightness was not appropriate when " + \
                                    "configured to value: " + str(brightness_value)
        VERDICT = FAILURE
    if proc:
        proc.kill()

menu_items = ["Press enter when you are ready to begin the test",
              {check_brightness_message: check_brightness},
              {"Do you want repeat the test ? (y/n)": validate_result}
              ]
menu(menu_items, menu_items[1:])
