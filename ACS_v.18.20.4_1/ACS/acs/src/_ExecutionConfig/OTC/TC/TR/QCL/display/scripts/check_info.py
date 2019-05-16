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

import sys
import os
from display_util import get_display_info
from display_util import get_desc_info
from util import print_test_result
from util import exit_test

arg = sys.argv
accepted_error = 0.001
try:
    desc_info = None
    display_info = None
    info_string = arg[1]
    info_name = arg[2]

    desc_info = get_desc_info(info_string, info_name)
    if desc_info is None:
        print_test_result(False, "Information could not be found in the desc file")
    else:
        display_info = get_display_info()
        if display_info is None or info_name not in display_info.keys():
            print_test_result(False, "Information could not be found by the driver")
        else:
            display_info = display_info[info_name]
            message = "Expected " + info_name + ": " + str(desc_info) + \
                      ", obtained " + info_name + ": " + str(display_info)
            print_test_result(abs(float(display_info) - float(desc_info)) < accepted_error, message)

except Exception as e:
    print_test_result(False, "Some information could not be found")
    exit_test()
