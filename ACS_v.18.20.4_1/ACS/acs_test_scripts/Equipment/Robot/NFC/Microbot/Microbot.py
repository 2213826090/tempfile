"""
:copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
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
:summary: Implementation of Microbot Equipment
:since:09/10/2012
:author: lpastor
"""
import time

from acs_test_scripts.Equipment.IEquipment import ExeRunner
from acs_test_scripts.Equipment.Robot.NFC.Interface.INFCRobot import INFCRobot
from acs_test_scripts.Equipment.IOCards.ACB.Common.SerialComms import SerialComms
from UtilitiesFWK.Utilities import is_number
from ErrorHandling.TestEquipmentException import TestEquipmentException


class Microbot(ExeRunner, INFCRobot):
    """
    Implementation of Microbot Equipment
    """

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment catalog parameters
        :type bench_params: dict
        :param bench_params: the dictionary containing bench parameters of the equipment
        """
        INFCRobot.__init__(self)
        ExeRunner.__init__(self, name, model, eqt_params)
        self.__bench_params = bench_params
        # Get Serial parameters for connection
        self.serial_com_port = str(self.__bench_params.get_param_value("ComPort"))

        self.__retrieve = 3

        # current position (unfortunatly it's impossible to retrieve robot position)
        # so we move the robot to zero position in the init
        self.__current_position = {'x': 0, 'y': 0, 'z': 0}

        self.positioning(str(self.__current_position['x']), str(self.__current_position['y']), str(self.__current_position['z']), "null")

    def positioning(self, xCoordinate, yCoordinate, zCoordinate, timer):
        """
        Change robot position
        :type xCoordinate: str
        :param xCoordinate: X-axis coordinate. Use "null" to move along Z axis
        :type yCoordinate : str
        :param yCoordinate: Y-axis coordinate. Use "null" to move along Z axis
        :type zCoordinate : str
        :param zCoordinate: Z-axis coordinate. Use "null" to move in XY plan
        :type timer : str
        :param timer: time in seconds to stay in position before coming back at origin. Use "null" to keep the position
        """

        # make sure that the com interface is released
        self._serialtesting()

        cmd_line = "%s %s %s %s %s" % (xCoordinate, yCoordinate, zCoordinate, timer, self.serial_com_port)

        # try several time in case of unhandled exceptions
        for i in range(self.__retrieve):
            try:
                ExeRunner.start_exe(self, cmd_line)
                break
            except TestEquipmentException:
                # if we are in the last try, raise the test equipment exception
                if i == self.__retrieve-1:
                    raise (TestEquipmentException.CRITICAL_FAILURE, " Robot failed to move see log")
                # if robot failed, just wait and try again
                time.sleep(5)
                pass

        # save new position
        if is_number(xCoordinate) and is_number(yCoordinate) and is_number(zCoordinate):
            self.__current_position['x'] = int(xCoordinate)
            self.__current_position['y'] = int(yCoordinate)
            self.__current_position['z'] = int(zCoordinate)

    def retrieve_position(self):
        """
        return a dictionnary with the current robot position
        :return: position : dictionnary
        """
        return self.__current_position

    def _serialtesting(self):
        """
        testing COM port before launching a positionning command to avoid conflict
        we use serialComms acs utility to open and close the COM connection
        """

        #initialize serialcomms
        acsSerial = SerialComms()
        acsSerial.init(self.serial_com_port, 115200, 3)
        if acsSerial.is_communication_up():
            acsSerial.release()
