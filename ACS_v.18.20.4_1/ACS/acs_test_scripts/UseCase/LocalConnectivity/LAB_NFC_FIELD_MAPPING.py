"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described herein and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL MCG PSI
:summary: Making a NFC field mapping
:since: 22/07/2014
:author: jortetx
"""
import os
import time
import threading

from acs_test_scripts.UseCase.LocalConnectivity.LAB_NFC_BASE import LabNfcBase
from UtilitiesFWK.Utilities import Global, str_to_bool, is_number
from ErrorHandling.DeviceException import DeviceException


class LabNfcFieldMapping(LabNfcBase):
    def __init__(self, tc_name, global_config):

        LabNfcBase.__init__(self, tc_name, global_config)

        # starting height
        self.__starting_Z = self._tc_parameters.get_param_value("STARTING_Z")
        if self.__starting_Z is None:
            self.__starting_Z = 0
        else:
            self.__starting_Z = int(self.__starting_Z)

        # ending height
        self.__max_Z = self._tc_parameters.get_param_value("MAX_Z")
        if self.__max_Z is None:
            self.__max_Z = 0
        else:
            self.__max_Z = int(self.__max_Z)

        # test max Z or not
        self.__is_max_z = str_to_bool(self._tc_parameters.get_param_value("FOUND_MAX_Z"))

        # position of the testing square
        self.__right_up_corner_position = str(self._tc_parameters.get_param_value("RIGHT_UP_CORNER")).split("x")

        self.__starting_X = int(self.__right_up_corner_position[0])
        self.__starting_Y = int(self.__right_up_corner_position[1])

        # get the resolution
        self.__resolution = int(self._tc_parameters.get_param_value("RESOLUTION"))

        # output file
        self.__result_file_name = self._tc_parameters.get_param_value("RESULT_FILE_NAME")
        self.__result_file_handler = open(os.path.join(self._saving_directory, self.__result_file_name), 'w')

        # get the antenna position
        nfc_antenna_position = self._device.get_config("NfcAntennaPosition", "").split(',')
        if len(nfc_antenna_position) == 2 and is_number(nfc_antenna_position[0]) and is_number(nfc_antenna_position[1]):
            self._x_antenna = int(nfc_antenna_position[0])
            self._y_antenna = int(nfc_antenna_position[1])
        else:
            self._x_antenna = 0
            self._y_antenna = 0

# ----------------------------------------------------------------------------------------------------------------

    def set_up(self):
        LabNfcBase.set_up(self)
        self._robot_positioning('0', '0', '0', "null")
        return Global.SUCCESS, "No errors"

# ----------------------------------------------------------------------------------------------------------------

    def run_test(self):
        LabNfcBase.run_test(self)

        if self.__is_max_z:

            # try to find the higher field point in the center of device

            higher_z = self.__found_max_Z()

            if higher_z is None:
                # something is wrong
                msg = "No field found on Z axe, test ending"
                self.__result_file_handler.write(msg)
            else:
                msg = "Higher field point on Z axe is %f\n" % float(abs(self.__max_Z - higher_z))
                self.__result_file_handler.write(msg)

        # Build the field

        for y in range(-self.__starting_Y, self.__starting_Y, self.__resolution):
            for x in range(-self.__starting_X, self.__starting_X, self.__resolution):
                is_field_on = self.__is_read_ok(x, y, self.__max_Z)
                self.__add_point_to_file(x, y, self.__max_Z, is_field_on)

        return Global.SUCCESS, "No errors"

# ----------------------------------------------------------------------------------------------------------------

    def tear_down(self):
        """
        End and dispose the test
        """
        self.__result_file_handler.close()
        LabNfcBase.tear_down(self)

        return Global.SUCCESS, "No errors"

# ----------------------------------------------------------------------------------------------------------------

    def __found_max_Z(self):
        """
        Found first value of Z where tags can be read

        :return: if z is found return z, else None
        """
        z = self.__starting_Z

        found = False
        while not found:
            z -= self.__resolution
            if z <= self.__max_Z:
                return None
            found = self.__is_read_ok(self._x_antenna, self._y_antenna, z)

        return z

# ----------------------------------------------------------------------------------------------------------------

    def __is_read_ok(self, x, y, z):
        """
        check if tag is read or not
        :return: Boolean
        """

        # just try to read tag
        # Initialize  the read threads

        reading_thread = threading.Thread(target=self._robot_positioning,
                                          args=("null", "null", str(z), "null"))

        self._robot_positioning(str(x), str(y), self.__starting_Z, "null")
        time.sleep(1)

        try:
            reading_thread.start()
            self._nfc_api.read_nfc_tag()
            reading_thread.join()

        except:
            return False
            raise

        return True

# ----------------------------------------------------------------------------------------------------------------

    def __add_point_to_file(self, x, y, z, is_field_on):
        """
        add a point to the result file
        """
        # before adding data to file we have to translate in mm
        # 1000 robot step = 1mm
        if is_field_on:
            fo = 1
        else:
            fo = 0

        line = "%f;%f;%f;%d\n" % (float(x)/1000, float(y)/1000, float(abs(self.__max_Z - z)), fo)
        self.__result_file_handler.write(line)

