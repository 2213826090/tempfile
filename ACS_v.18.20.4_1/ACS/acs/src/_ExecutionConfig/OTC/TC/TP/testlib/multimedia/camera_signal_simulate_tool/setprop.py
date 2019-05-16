#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2017 Google Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

"""
    This module tests the Vehicle HAL using adb socket.

    Protocol Buffer:
        This module relies on VehicleHalProto_pb2.py being in sync with the protobuf in the VHAL.
        If the VehicleHalProto.proto file has changed, re-generate the python version using:

            protoc -I=<proto_dir> --python_out=<out_dir> <proto_dir>/VehicleHalProto.proto
            protoc -I=proto --python_out=proto proto/VehicleHalProto.proto
"""

# Suppress .pyc files
import sys

sys.dont_write_bytecode = True

import VehicleHalProto_pb2
import vhal_consts_2_0
import vhal_emulator
import logging


class VhalTest:
    _configs = 0  # List of configs from DUT
    _vhal = 0  # Handle to VHAL object that communicates over socket to DUT


    def onAction(self, prop, area, value):
        self._vhal.setProperty(prop, area, value)
        reply = self._vhal.rxMsg()
        print reply

    def convertPropId(self, prop):
        if prop == 1:
            return vhal_consts_2_0.VEHICLE_PROPERTY_HVAC_RECIRC_ON
        elif prop == 2:
            return vhal_consts_2_0.VEHICLE_PROPERTY_HVAC_FAN_DIRECTION
        elif prop == 3:
            return vhal_consts_2_0.VEHICLE_PROPERTY_HVAC_AC_ON
        elif prop == 4:
            return vhal_consts_2_0.VEHICLE_PROPERTY_HVAC_DEFROSTER
        elif prop == 5:
            return vhal_consts_2_0.VEHICLE_PROPERTY_HVAC_AUTO_ON
        elif prop == 6:
            return vhal_consts_2_0.VEHICLE_PROPERTY_HVAC_TEMPERATURE_SET
        elif prop == 7:
            return vhal_consts_2_0.VEHICLE_PROPERTY_HVAC_FAN_SPEED
        elif prop == 8:
            return vhal_consts_2_0.VEHICLE_PROPERTY_NIGHT_MODE
        elif prop == 9:
            return vhal_consts_2_0.VEHICLE_PROPERTY_FUEL_LEVEL_LOW
        elif prop == 10:
            return vhal_consts_2_0.VEHICLE_PROPERTY_ENGINE_RPM
        elif prop == 11:
            return vhal_consts_2_0.VEHICLE_PROPERTY_PERF_ODOMETER
        elif prop == 12:
            return vhal_consts_2_0.VEHICLE_PROPERTY_ENGINE_COOLANT_TEMP
        elif prop == 13:
            return vhal_consts_2_0.VEHICLE_PROPERTY_ENGINE_OIL_TEMP
        elif prop == 14:
            return vhal_consts_2_0.VEHICLE_PROPERTY_GEAR_SELECTION
        print "Parameters propId Error: " + str(prop)
        return 0

    def convertAreaId(self, area):
        if area == 1:
            return vhal_consts_2_0.VEHICLE_ZONE_ROW_1_LEFT
        elif area == 2:
            return vhal_consts_2_0.VEHICLE_ZONE_ROW_1_CENTER
        elif area == 4:
            return vhal_consts_2_0.VEHICLE_ZONE_ROW_1_RIGHT
        else:
            return 0

    def parseParameters(self, argv):
        prop = 0
        value = 0
        areas = 0
        if len(argv) == 4:
            prop = self.convertPropId(int(argv[1]))
            areas = self.convertAreaId(int(argv[2]))
            value = int(argv[3])
        else:
            print "Parameters Error, e.g.: python setprop.py propId areaId value"
            return 0, 0, 0
        if prop != 0:
            return prop, areas, value
        else:
            self.printPropId()
            return 0, 0, 0

    def printPropId(self):
        print "All propId:"
        print "1. hvac recirc on/off"
        print "2. hvac fan direction"
        print "3. hvac ac on/off"
        print "4. hvac defroster"
        print "5. hvac auto on/off"
        print "6. hvac temperature"
        print "7. hvac fan speed"
        print "8. hvac night mode on/off"
        print "9. hvac fuel level low"
        print "10. hvac engine rpm"
        print "11. hvac odometer"
        print "12. hvac engine coolant temperature"
        print "13. hvac engine oil temperature"
        print "14. hvac property gear selection"

    def __init__(self, types, argv):
        self._types = types
        # Start the VHAL Emulator
        self._vhal = vhal_emulator.Vhal(types)
        # Get the list of configs
        #self._vhal.getConfigAll()
        #self._configs = self._vhal.rxMsg().config
        #print self._configs
        prop, area, value = self.parseParameters(argv)
        if prop != 0:
            self.onAction(prop, area, value)


if __name__ == '__main__':
    v = VhalTest(vhal_consts_2_0.vhal_types_2_0, sys.argv)
