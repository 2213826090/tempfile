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

from random import shuffle
from time import sleep
from display_util import display_on_screen
from display_util import validate_response
from util import validate_result
from util import menu
from util import print_to_console_and_logcat

VERDICT = FAILURE
OUTPUT = ""
check_color_message = "The display will show different colors."
insert_color_msg = "Please insert which color you have seen by inserting the " + \
                   "corresponding number followed by ENTER:\n1 for red\n2 for green\n3 for blue\n" + \
                   "4 for white\n5 for other color\nPress 0 if you want the color to be displayed again\n"

def check_color(arguments):
    global VERDICT, OUTPUT
    colors = range(1, 5)
    shuffle(colors)
    passed = True
    for color in colors:
        print insert_color_msg
        sleep(1)
        proc = display_on_screen(color)
        passed, OUTPUT = validate_response(color)
        if not passed:
            print_to_console_and_logcat(OUTPUT)
            VERDICT = FAILURE
            if proc:
                proc.kill()
            break
        if proc:
            proc.kill()

    if passed:
        OUTPUT = "Success"
        print_to_console_and_logcat(OUTPUT)
        VERDICT = SUCCESS

menu_items = ["Press enter when you are ready to begin the test",
              {check_color_message: check_color},
              {"Do you want repeat the test ? (y/n)": validate_result}
              ]
menu(menu_items, menu_items[1:])
