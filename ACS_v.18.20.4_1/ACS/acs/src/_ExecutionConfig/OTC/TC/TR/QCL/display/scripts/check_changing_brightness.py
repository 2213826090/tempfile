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

from time import sleep
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
BRIGHTNESS_CHANGE = TC_PARAMETERS('BRIGHTNESS')
check_brightness_message = "Please check the brightness of the display."
if (BRIGHTNESS_CHANGE == "INCREASE"):
    order = 1
elif (BRIGHTNESS_CHANGE == "DECREASE"):
    order = -1
else:
    OUTPUT = "Wrong parameter. BRIGHTNESS value should be MAX or MIN"
    print_to_console_and_logcat(OUTPUT)
    exit_test()

brightness_max = 255
brightness_min = 0
step = 15
white_color = 4
white_timeout = 10
validation_message = "Did you observe a gradually " + BRIGHTNESS_CHANGE.lower() + \
                     " of the display brightness? (y/n)"

def check_brightness(arguments):
    global VERDICT, OUTPUT
    # add step to brightness_max in order to have a closed range
    values_range = xrange(brightness_min, brightness_max + step, step)
    if order == -1:
        values_range = reversed(values_range)
    values_range = list(values_range)
    set_brightness(values_range[0])
    #set display color to be white for 10 seconds
    proc = display_on_screen(white_color, white_timeout)
    for value in values_range[1:]:
        sleep(0.3)
        set_brightness(value)
    print validation_message
    if user_validation():
        OUTPUT = "Success"
        print_to_console_and_logcat(OUTPUT)
        VERDICT = SUCCESS
    else:
        OUTPUT = "Failure. Display brightness should " + BRIGHTNESS_CHANGE.lower()
        print_to_console_and_logcat(OUTPUT)
        VERDICT = FAILURE
    if proc:
        proc.kill()

menu_items = ["Press enter when you are ready to begin the test",
              {check_brightness_message: check_brightness},
              {"Do you want repeat the test ? (y/n)": validate_result}
              ]
menu(menu_items, menu_items[1:])
