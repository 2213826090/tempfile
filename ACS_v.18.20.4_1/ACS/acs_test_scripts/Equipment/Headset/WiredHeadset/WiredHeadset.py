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

:organization: FT AudioComms
:summary: Wired Headset implementation
:since: 28/10/2013
:author: nprecigx
"""

from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.Headset.Interface.IHeadset import IHeadset


class WiredHeadset(EquipmentBase, IHeadset):

    """
    Class that implements WiredHeadset equipment
    """

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        :type name: str
        :param name: the bench configuration name of the equipment
        :type model: str
        :param model: the model of the equipment
        :type eqt_params: dict
        :param eqt_params: the dictionary containing equipment parameters
        """
        EquipmentBase.__init__(self, name, model, eqt_params)
        IHeadset.__init__(self)
        self.__wired_headset = None
        self.__headphone = None
        self._bench_params = bench_params

        # NOTE: import here to avoid circular dependency on
        # EquipmentManager if imported at top level
        from acs_test_scripts.Equipment.EquipmentManager import EquipmentManager
        self._em = EquipmentManager()
        self._io_card = self._em.get_io_card("IO_CARD")

        if ((self._bench_params.has_parameter("wired_headset")) and
           (self._bench_params.get_param_value("wired_headset") != "")):
            self.__wired_headset = \
                int(self._bench_params.get_param_value("wired_headset"))

        if ((self._bench_params.has_parameter("headphone")) and
           (self._bench_params.get_param_value("headphone") != "")):
            self.__headphone = \
                int(self._bench_params.get_param_value("headphone"))

        if self._io_card is None:
            self._logger.info("No IO card instance")

    def plug_whs(self):
        """
        Plug the wired headset
        :rtype: None
        """
        if self._io_card is None:
            self._logger.info("No IO card instance")
        else:
            self._logger.info("Plug wired headset")
            self._io_card.enable_line(self.__wired_headset)

    def unplug_whs(self):
        """
        Unplug the wired headset
        :rtype: None
        """
        if self._io_card is None:
            self._logger.info("No IO card instance")
        else:
            self._logger.info("Unplug wired headset")
            self._io_card.disable_line(self.__wired_headset)


    def plug_headphone(self):
        """
        Plug the headphone
        :rtype: None
        """
        if self._io_card is None:
            self._logger.info("No IO card instance")
        else:
            self._logger.info("Plug headphone")
            self._io_card.enable_line(self.__headphone)

    def unplug_headphone(self):
        """
        Unplug the headphone
        :rtype: None
        """
        if self._io_card is None:
            self._logger.info("No IO card instance")
        else:
            self._logger.info("Unplug headphone")
            self._io_card.disable_line(self.__headphone)
