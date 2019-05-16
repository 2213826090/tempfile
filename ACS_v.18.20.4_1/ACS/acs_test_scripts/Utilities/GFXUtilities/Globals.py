#!/usr/bin/env python
'''
    Copyright 2012 Android Open Source Project

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

#
# Globals definition
#
COMMAND_DELAY  = 1    # used for interact function delay
INTERNAL_DELAY = 3    # used for MoneyRunner.sleep()
SHELL_DELAY    = 2    # used for execute_command delay
EXCLUDE_FROM_RECENTS = 0x00800000
EXIT_SUCCESS  = 0x0
EXIT_MULTIPLE = 0x1

TOUCH_TEXT = 0x000
TOUCH_ID   = 0x001
TYPE_TEXT  = 0x010
TYPE_ID    = 0x011
SWIPE      = 0x100

# People Application globals
PEOPLE_ADD_CONTACT_BUTTON    = 'id/no_id/13'
PEOPLE_CONTACT_EXPAND_BUTTON = 'id/no_id/32'
PEOPLE_DONE_BUTTON           = 'id/no_id/5'

# OxBenchmark Application globals
OxBENCH_MENU_ID = {'0xMath'  : 'id/no_id/36',
                   '0x2D'    : 'id/no_id/37',
                   '0x3D'    : 'id/no_id/38',
                   '0xVM'    : 'id/no_id/39',
                   '0xNative': 'id/no_id/40',
                   '0xMisc'  : 'id/no_id/41'}

# AndroBench Application globals
ANDRO_MEASUREMENT_BUTTON = 'id/no_id/8'
ANDRO_ALL_BUTTON = 'id/no_id/24'

# Temperature index constants
COLD_BOOT = 35000
HOT_BOOT = 40000
THERMAL_COMPONENT_DELAY = 120 #seconds /120