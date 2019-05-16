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
from sensors_util import get_sensor_hal_info
from util import print_test_result
from util import exit_test

arg = sys.argv
try:
    sensor_info = get_sensor_hal_info(arg[1])
    sensor_id = sensor_info["index"]
    sensor_type = sensor_info["type"]
    resolution = sensor_info["resolution"]
except:
    print_test_result(False, "Sensor was not found")
    exit_test()

reference = arg[2]
if resolution >= float(reference):
    print_test_result(True, "Resolution is greater than " + reference)
else:
    print_test_result(False, "Resolution is less than " + reference)
