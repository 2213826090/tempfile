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
:summary: This file implements the LAB NFC BASE class
:since: 18/07/2013
:author: apairex
"""

from acs_test_scripts.UseCase.LocalConnectivity.LIVE_NFC_BASE import LiveNfcBase
from UtilitiesFWK.Utilities import is_number, Global, str_to_bool_ex
from ErrorHandling.AcsConfigException import AcsConfigException


class LabNfcBase(LiveNfcBase):
    """
    Live NFC Test base class.
    """

    def __init__(self, tc_name, global_config):
        """
        Constructor
        """
        LiveNfcBase.__init__(self, tc_name, global_config)

        # Create NFC robot
        self._nfc_robot = self._em.get_nfc_robot("NFC_ROBOT1")

        # Get the TC parameter: Should we use offset for robot moves (considering antenna position on DUT)
        self._use_antenna_offset = str_to_bool_ex(self._tc_parameters.get_param_value("USE_ANTENNA_OFFSET", ""))
        self._x_offset = None
        self._y_offset = None
        if self._use_antenna_offset:
            # Get the NFC antenna position from Device_Catalog.xml
            nfc_antenna_position = self._device.get_config("NfcAntennaPosition", "").split(',')
            if len(nfc_antenna_position) == 2 and \
                    is_number(nfc_antenna_position[0]) and is_number(nfc_antenna_position[1]):
                self._x_offset = int(nfc_antenna_position[0])
                self._y_offset = int(nfc_antenna_position[1])

    def set_up(self):
        """
        Initialize the test
        """
        LiveNfcBase.set_up(self)

        if self._use_antenna_offset:
            if self._x_offset is None or self._y_offset is None:
                msg = "You cannot use Antenna Offset. "
                msg += "The antenna position on DUT is not present in the Device_Catalog XML file"
                self._logger.error(msg)
                raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

            self._logger.debug("DUT Antenna offset : x-> %d y-> %d" % (self._x_offset, self._y_offset))

        return Global.SUCCESS, "No errors"

    def _robot_positioning(self, x_coordinate, y_coordinate, z_coordinate, timer):
        """
        Change robot position considering offset if required
        :type x_coordinate: str
        :param x_coordinate: X-axis coordinate. Use "null" to move along Z axis
        :type y_coordinate : str
        :param y_coordinate: Y-axis coordinate. Use "null" to move along Z axis
        :type z_coordinate : str
        :param z_coordinate: Z-axis coordinate. Use "null" to move in XY plan
        :type timer : str
        :param timer: time in seconds to stay in position before coming back at origin. Use "null" to keep the position
        """
        if self._use_antenna_offset:
            if is_number(x_coordinate):
                self._logger.debug("X position before offset: " + x_coordinate)
                x_coordinate = str(int(x_coordinate) + self._x_offset)
            if is_number(y_coordinate):
                self._logger.debug("Y position before offset: " + y_coordinate)
                y_coordinate = str(int(y_coordinate) + self._y_offset)

        self._logger.info("Start moving robot")
        self._nfc_robot.positioning(x_coordinate, y_coordinate, z_coordinate, timer)
