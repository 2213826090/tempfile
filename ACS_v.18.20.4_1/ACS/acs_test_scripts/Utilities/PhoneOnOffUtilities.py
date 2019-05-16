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
:summary: Utilities class for Web Browsing implementation
:since: 31/07/2013
:author: razzix
"""
import time
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException


class PhoneOnOff:
    """
    This class contains all the options related to resetting the phone
    """
    def __init__(self, networking_api, device, logger=None):
        """
        Constructor
        """
        # Get UECmdLayer for Data Use Cases
        self.__networking_api = networking_api
        self.__device = device
        self.__logger = logger
        self.__device_off = None
        self.__boot_mode = None

    def switch_off(self, mode):
        """
        Switch off the modem either by PowerOff the DUT or airplane mode on

        :type mode: str
        :param mode: Choose the mode to perform the swith off
                     Values : -hardshutdown(power off using hard Shutdown)
                              -softshutdown(power off using Soft Shutdown)
                              -airplane (set Airplane  mode on)
                              -off(default value: no actions needed)
        :raise DeviceException: if shutdown failed
                                 if mobile is is unknown boot mode
                                 if mode value is an unknow value
        """
        if mode == "airplane":
            self.__logger.info("switch off using airplane mode")

            # Flight mode activation
            self.__networking_api.set_flight_mode("on")

        elif mode == "hardshutdown":
            self.__logger.info("switch off using hard shutdown mode")
            # Hard Shutdown
            self.__device_off = self.__device.hard_shutdown(True)

            if self.__device_off:
                self.__logger.info("DUT is power off and ready to be switched on")
            else:
                msg = "Hard Shutdown failed"
                raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, msg)
        elif mode == "softshutdown":
            self.__logger.info("switching off using soft shutdown mode")
            self.__device_off = self.__device.soft_shutdown(True)
            # waiting for phone to boot in COS
            time.sleep(40)
            # Check if device is power off
            if self.__device_off:
                # Retrieve the boot mode
                self.__boot_mode = self.__device.get_boot_mode()
                if self.__boot_mode in ("COS", "UNKNOWN"):
                    self.__logger.info("Mobile is in %s Mode and ready to be switched on" % self.__boot_mode)
                else:
                    msg = "Mobile is in unknown state"
                    raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, msg)
            else:
                msg = "Soft Shutdown Failed"
                raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, msg)
        else:
            msg = "Received an unknown parameter value"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

    def switch_on(self, mode):
        """
        Switch on the modem either by PowerOn the DUT or airplane mode off

        :type mode: str
        :param mode: Choose the mode to perform the swith on
                     Values : -hardshutdown(after hard Shutdown)
                              -softshutdown(after Soft Shutdown)
                              -airplane(after Airplane on)
        :raise DeviceException: if DUT is still not switched off
                                 if the mode value is an unknown value

        """
        if mode == "airplane":
            self.__logger.info("switch on using airplane mode")

            # Flight mode desactivation
            self.__networking_api.set_flight_mode("off")

        elif mode in ("hardshutdown", "softshutdown"):

            self.__logger.info("switching on the dut after shutdown")

            if self.__device_off:
                # Switch on the Device
                self.__device.switch_on(None, 0)
            else:
                msg = "Mobile is still not switched off"
                raise DeviceException(DeviceException.PROHIBITIVE_BEHAVIOR, msg)
        else:
            msg = "Received an unknown parameter value"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

    def reboot(self, mode):
        """
        Reboot the modem by Power OFF/On the DUT

        :type mode: str
        :param mode: Choose the mode to perform the swith on
                     Values : -hardshutdown
                              -softshutdown
        :raise DeviceException: if the mode value is an unknown value

        """
        if mode == "hardshutdown":
            # Power OFF the UE
            self.switch_off("hardshutdown")
            # Power ON the UE
            self.switch_on("hardshutdown")
        elif mode == "softshutdown":
            self.__logger.info("Rebooting Device")
            self.__device.reboot(["COS", "MOS"])
        else:
            msg = "Received an unknown parameter value"
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)
