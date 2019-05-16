"""

:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
"""

from Device.Model.AndroidDevice.IntelDeviceBase import IntelDeviceBase
from UtilitiesFWK.Utilities import Global


class CohoDevice(IntelDeviceBase):

    """
        Sofia Lte platform implementation
    """

    # overrides boot modes, defined in parent class
    BOOT_STATES = {"MOS": "unknown",
                   "ROS": "recovery",
                   "COS": "charger"}

    def __init__(self, config, logger):
        """
        Constructor

        :type  phone_name: string
        :param phone_name: Name of the current phone(e.g. PHONE1)
        """

        IntelDeviceBase.__init__(self, config, logger)
        # Kept for c
        self._cellular_network_interface = self.get_config("cellularNetworkInterface", "rmnet0")

    def switch_on(self, boot_timeout=None, settledown_duration=None,
                  simple_switch_mode=False):
        """
        Switch ON the device
        This can be done either via the power supply
        or via IO card

        :param boot_timeout: Total time to wait for booting
        :type boot_timeout: int

        :param settledown_duration: Time to wait until start to count \
                                    for timeout,
                                    Period during which the device must \
                                    have started.
        :type settledown_duration: int

        :param simple_switch_mode: a C{boolean} indicating whether we want
        to perform a simple switch on.
        :type simple_switch_mode: bool

        :rtype: list
        :return: Output status and output log
        """
        (return_code, return_message) = IntelDeviceBase.switch_on(self, boot_timeout,
                                                                  settledown_duration,
                                                                  simple_switch_mode)
        if return_code == Global.SUCCESS:
            if self._embedded_log:
                # Initialize embedded log mechanism for MODEM if required
                self._embedded_log.start("MODEM")

        return return_code, return_message

    def get_cellular_network_interface(self):
        """
        Return the ip interface of celluar network

        this interface can be obtain from the command "busybox ifconfig"
        :rtype: str
        :return: telephony ip interface
        """
        return self._cellular_network_interface


    def get_boot_mode(self):
        """
        get the boot mode from adb

        :rtype: string
        :return: device state : MOS, ROS, POS, DNX, COS or UNKNOWN depending of what your board support
        """
        current_state = "UNKNOWN"
        retry = 2
        # Implement a retry mechanism as on some benches as on first
        # run of adb (if shutdown previously), it always return unknown
        #
        while retry > 0 and current_state.lower() == "unknown":
            retry -= 1
            current_state = self._get_device_boot_mode()
            if current_state.lower() == "unknown":
                # Fastboot command help find if device is in DnX or POS
                # For now we do not differentiate POS and DNX:
                # if DNX or POS -> POS
                # because DNX is tricky to detect and no req has been ask to do so
                current_state = self._check_pos_mode()

        self._logger.debug("device boot mode is %s" % current_state)

        return current_state
