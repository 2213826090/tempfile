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

from random import randint
from time import sleep
from display_util import display_on_screen
from display_util import validate_response
from display_util import suspend_and_resume
from util import validate_result
from util import menu
from util import print_to_console_and_logcat

VERDICT = FAILURE
OUTPUT = ""
insert_color_msg = "Please insert which color you have seen on the display by inserting the " + \
                   "corresponding number followed by ENTER:\n1 for red\n2 for green\n3 for blue\n" + \
                   "4 for white\n5 for other color\nPress 0 if you want the color to be displayed again\n"

def check_sleep(color):
    proc = display_on_screen(color)
    passed, message = validate_response(color)
    if not passed:
        if proc:
            proc.kill()
        return False, message
    if proc:
        proc.kill()
    if not suspend_and_resume():
        return False, "Failure"
    sleep(1)
    print insert_color_msg
    proc = display_on_screen(color)
    passed, message = validate_response(color)
    if not passed:
        if proc:
            proc.kill()
        return False, message
    if proc:
        proc.kill()
    return True, "Success"

def check_color(arguments):
    global VERDICT, OUTPUT
    color = randint(1, 4)
    passed, message = check_sleep(color)
    if passed:
        OUTPUT = message
        print_to_console_and_logcat(OUTPUT)
        VERDICT = SUCCESS
    else:
        OUTPUT = message + ". Device couldn't support suspend/resume power cycle"
        print_to_console_and_logcat(OUTPUT)
        VERDICT = FAILURE

menu_items = ["Press enter when you are ready to begin the test",
              {insert_color_msg: check_color},
              {"Do you want repeat the test ? (y/n)": validate_result}
              ]
menu(menu_items, menu_items[1:])
