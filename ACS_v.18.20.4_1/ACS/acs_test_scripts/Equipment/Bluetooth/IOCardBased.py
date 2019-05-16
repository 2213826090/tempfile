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
:summary: Base class for bluetooth equipment based on IO Cards (e.g. headset, mouse)
:since: 20/02/2014
:author: fbongiax
"""

import time
from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from ErrorHandling.TestEquipmentException import TestEquipmentException


class IOCardBased(EquipmentBase):

    """
    Class that implements Base IOCard equipment
    """

    def __init__(self, name, model, eqt_params, bench_params, factory=None):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        :type bench_params: dict
        :param bench_params: the dictionary containing equipment bench parameters
        """
        EquipmentBase.__init__(self, name, model, eqt_params, factory)

        # instantiate IOCard
        iocard = str(bench_params.get_param_value("IOCard"))
        self._em = self._factory.create_equipment_manager()
        self._iocard = self._em.get_io_card(iocard)

        self._bench_params = bench_params
        self._power_scard = None
        self._bdaddress = None
        self._name = None

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use
        """
        self.get_logger().info("IOCardBased equipment initialization")

        # Verify IOCard model is USBRLy08 or USBRELAY32
        if self._iocard.get_bench_params().get_param_value("Model") \
                != "USB_RLY08" and self._iocard.get_bench_params().get_param_value("Model") != "USBRELAY32":
            msg = "IOCard Model shall be USB_RLY08 or USBRELAY32"
            self.get_logger().error(msg)
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)

        if ((self._bench_params.has_parameter("BD_Address")) and
           (self._bench_params.get_param_value("BD_Address") != "")):
            self._bdaddress = \
                str(self._bench_params.get_param_value("BD_Address")).upper()
            self.get_logger().info("Set BD address to %s", self._bdaddress)
        else:
            self._bdaddress = "00:00:00:00:00:00"

        if ((self._bench_params.has_parameter("Name")) and
                (self._bench_params.get_param_value("Name") != "")):
            self._name = str(self._bench_params.get_param_value("Name"))
            self.get_logger().info("Set Name to %s", self._name)

    def get_bdaddress(self):
        """
        Returns bd address parameter of the equipment
        :rtype: str
        :return: BD address of the equipment. format 00:00:00:00:00:00
        """
        assert self._bdaddress, "self._bdaddress cannot be empty or None at this stage"
        return str(self._bdaddress)

    def _configure_button(self, name):
        """
        If information for the button exist in the bench config, returns it,
        otherwise it returns None.
        :type name: str
        :param name: name of the bench config item
        :rtype: str
        :return: the value from the bench config or None
        """
        if ((self._bench_params.has_parameter(name)) and
           (self._bench_params.get_param_value(name) != "")):
            return self._bench_params.get_param_value(name)
        return None

    def _set_button_line(self, name):
        """
        Return the relay line for the button with the given name if defined in the bench config or None.
        :type name: str
        :param name: name of the bench config item
        :rtype: str
        :return: the value from the bench config or None
        """
        value = self._configure_button(name)
        return int(value) if value else None

    def _set_button_timer(self, name):
        """
        Return the time to wait for the button with the given name between closing / opening the relay
        (to simulate the button click) if defined in the bench config or None.
        :type name: str
        :param name: name of the bench config item
        :rtype: str
        :return: the value from the bench config or None
        """
        value = self._configure_button(name)
        return float(value) if value else None

    def _wait_for_secs(self, secs):
        """
        Waits for "secs" seconds
        """
        time.sleep(secs)

    def _press_relay(self, line, duration):
        """
        Press slave IOcard relay during duration (in second)
        :type line: integer
        :param line: ID of the relay to press
        :type duration: integer
        :param duration: duration between press and release relay, in second
        """
        self._iocard.press_relay_button(duration, line)

    def _raise_error_if_any_is_none(self, values, msg):
        """
        If any of the passed values is None it raises an exception with the given message
        :type values: array of objects
        :param values: list of values which can be None
        """
        for item in values:
            if item is None:
                self.get_logger().error(msg)
                raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR, msg)

    def get_device_name(self):
        """
        Returns bt device name parameter of the equipment
        :rtype: str
        :return: BT device name of the equipment. as shown in logs and on the screen
        """
        assert self._name, "self._name cannot be empty or None at this stage"
        return str(self._name)