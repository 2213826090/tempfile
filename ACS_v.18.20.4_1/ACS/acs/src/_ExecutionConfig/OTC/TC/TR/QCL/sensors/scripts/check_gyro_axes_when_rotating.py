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
from sensors_util import get_sensor_hal_info
from sensors_util import PollThread
from util import print_test_result
from util import exit_test
from util import validate_result
from util import menu
from util import are_values_ordered
from util import eliminate_succesive_duplicates

# class definition
class ExecutePoll(PollThread):
    def stop(self):
        self.proc.terminate()

    def check_values(self):
        maxim = max(self.values)
        max_position = self.values.index(maxim)
        if max_position > 3 and are_values_ordered(self.values[:max_position + 1]) and \
            maxim - self.values[0] > 1:
            message = "Gyroscope values on {} axis were increasing".format(self.axes)
            return True, message
        else:
            values = eliminate_succesive_duplicates(self.values)
            message = "It was expected an increasing of the values on the {} axis with at least 1.".format(self.axes)
            message += " Returned values were: {}".format(values)
            return False, message

# function definition
def check_gyro_when_rotating(args):
    global VERDICT, OUTPUT
    print "Press ENTER to stop the record"
    executorPoll = ExecutePoll()
    executorPoll.id = sensor_id
    executorPoll.type = sensor_type
    executorPoll.axes = AXIS
    executorPoll.start()
    while (True):
        input_line = raw_input("")
        if input_line == "":
            executorPoll.stop()
            ver, OUTPUT = executorPoll.check_values()
            print_test_result(ver, OUTPUT)
            if ver:
                VERDICT = SUCCESS
            else:
                VERDICT = FAILURE
            break

# test execution
VERDICT = FAILURE
OUTPUT = ""
AXIS = TC_PARAMETERS('PARAMETERS')
COMP_CONF = TC_PARAMETERS('COMP_CONF')
COMP_CONF = COMP_CONF[1:-1]

try:
    sensor_info = get_sensor_hal_info(COMP_CONF)
    sensor_id = sensor_info["index"]
    sensor_type = sensor_info["type"]

except:
    OUTPUT =  "Sensor was not found"
    print_test_result(False, OUTPUT)
    exit_test()

menu_items = ["Press enter when you are ready to begin the test",
              {"Rotate the device counter clockwise along the {} axis".format(AXIS): check_gyro_when_rotating},
              {"Do you want repeat the test (y/n)": validate_result}]

menu(menu_items, menu_items[1:])
