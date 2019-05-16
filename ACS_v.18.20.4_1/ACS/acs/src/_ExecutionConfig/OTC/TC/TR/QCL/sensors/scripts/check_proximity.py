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
import sys
from sensors_util import get_sensor_hal_info
from sensors_util import PollThread
from util import menu_next_item
from util import validate_result
from util import menu
from util import print_test_result
from util import exit_test

arg = sys.argv

expected_values_covered = []
expected_values_uncovered = []

class ExecutePoll(PollThread):
    def stop(self):
        self.aux_proc.terminate()

    def check_values_covered(self):
        print "EXPECTED VALUES: {} ".format(expected_values_covered)
        print "GOT VALUES: ".format(self.values)

        if len(self.values) != len(expected_values_covered):
            return False, "The number of expected events did not happen"
        else:
            for index in range(len(self.values)):
                if self.values[index] != expected_values_covered[index]:
                    return False,"Recived values did not match the expected values"
        return True, "Success"

    def check_values_uncovered(self):
        print "EXPECTED VALUES: {} ".format(expected_values_uncovered)
        print "GOT VALUES: ".format(self.values)

        if len(self.values) != len(expected_values_uncovered):
            return False, "The number of expected events did not happen"
        else:
            for index in range(len(self.values)):
                if self.values[index] != expected_values_uncovered[index]:
                    return False, "Recived values did not match the expected values"
        return True, "Success"

def check_proximity_covered_sensor():
    global VERDICT, OUTPUT
    print "Enter 'END' to stop the record"
    executorPoll = ExecutePoll()
    executorPoll.id = sensor_id
    executorPoll.type = sensor_type
    executorPoll.axes = "distance"
    executorPoll.start()
    index = 0
    while (True):
        input_line = raw_input("")
        if input_line == "":
            if (index % 2) == 0:
                expected_values_covered.append(float(1))
            else:
                expected_values_covered.append(float(0))
            index = index + 1
        elif input_line == "END":
            executorPoll.stop()
            ver, OUTPUT = executorPoll.check_values_covered()
            print_test_result(ver, OUTPUT)
            if ver:
                VERDICT = SUCCESS
            else:
                VERDICT = FAILURE
            break
        else:
            print "Unknown command " + input_line + "list of recongised commands : 'ENTER', END"

def check_proximity_uncovered_sensor():
    global VERDICT, OUTPUT
    print "Enter 'END' to stop the record"
    executorPoll = ExecutePoll()
    executorPoll.id = sensor_id
    executorPoll.type = sensor_type
    executorPoll.axes = "distance"
    executorPoll.start()
    index = 0
    while (True):
        input_line = raw_input("")
        if input_line == "":
            if (index % 2) == 0:
                expected_values_uncovered.append(float(0))
            else:
                expected_values_uncovered.append(float(1))
            index = index + 1
        elif input_line == "END":
            executorPoll.stop()
            ver, OUTPUT = executorPoll.check_values_uncovered()
            print_test_result(ver, OUTPUT)
            if ver:
                VERDICT = SUCCESS
            else:
                VERDICT = FAILURE
            break
        else:
            print "Unknown command " + input_line + "list of recongised commands : 'ENTER', END"

# test execution
VERDICT = FAILURE
OUTPUT = ""
COMP_CONF = TC_PARAMETERS('COMP_CONF')
COMP_CONF = COMP_CONF[1:-1]

try:
    sensor_info = get_sensor_hal_info(COMP_CONF)
    sensor_id = sensor_info["index"]
    sensor_type = sensor_info["type"]

except:
    OUTPUT = "Sensor was not found"
    print_test_result(False, OUTPUT)
    exit_test()

menu_items = ["Press enter when you are ready to begin the test",
              "The first test consists in determining that the sensor readings are correct when it starts with an object in front of it (Press ENTER to continue)",
              "Put an object in front of the sensor (Press ENTER to continue)",
              {
                  "Sensor readings start now! In order to proceed press ENTER before any action (remove/put object in fornt of the sensor) is performed ": check_proximity_covered_sensor},
              "The second test consists in determining that the sensor readings are correct when it starts with no object in front of it (Press ENTER to continue)",
              "Remove any object that might be in front of the sensor (Press ENTER to continue)",
              {
                  "Sensor readings start now!  In order to proceed press ENTER before any action (remove/put object in fornt of the sensor) is performed ": check_proximity_uncovered_sensor},
              {"Do you want repeat the test (y/n)": validate_result}]

menu(menu_items)
