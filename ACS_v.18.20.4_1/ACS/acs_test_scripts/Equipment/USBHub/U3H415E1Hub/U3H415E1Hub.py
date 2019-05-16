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

:organization: INTEL MCG PSI
:summary: :summary: Library to handles usb actions based on libusb1
to be enriched
:author: cbonnard
:since: 13/01/2014
"""

from ErrorHandling.TestEquipmentException import TestEquipmentException
from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.USBHub.Interface.IUSBHub import IUSBHub

import libusb1
import usb1

REQUEST_TYPE = libusb1.LIBUSB_TYPE_CLASS | libusb1.LIBUSB_RECIPIENT_OTHER
USB_PORT_FEATURE_POWER = 8
DEFAULT_VENDOR_ID = 0x8043
DEFAULT_PRODUCT_ID = 0x0451
DEFAULT_BUS_ID = 1


class U3H415E1Hub(EquipmentBase, IUSBHub):

    """
    Class USBLib: library based on libusb1
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
        :type bench_params: dict
        :param bench_params: the dictionary containing equipment bench parameters
        """

        EquipmentBase.__init__(self, name, model, eqt_params)
        IUSBHub.__init__(self)
        self.__bench_params = bench_params
        if (self.__bench_params.has_parameter(IUSBHub.PRODUCT_ID_PARAM) and
                self.__bench_params.get_param_value(IUSBHub.PRODUCT_ID_PARAM) != ""):
            self.__product_id = int(self.__bench_params.get_param_value(IUSBHub.PRODUCT_ID_PARAM), 16)
        else:
            self.__product_id = DEFAULT_PRODUCT_ID
        if (self.__bench_params.has_parameter(IUSBHub.VENDOR_ID_PARAM) and
                self.__bench_params.get_param_value(IUSBHub.VENDOR_ID_PARAM) != ""):
            self.__vendor_id = int(self.__bench_params.get_param_value(IUSBHub.VENDOR_ID_PARAM), 16)
        else:
            self.__vendor_id = DEFAULT_VENDOR_ID
        if (self.__bench_params.has_parameter(IUSBHub.USB_BUS_PARAM) and
                self.__bench_params.get_param_value(IUSBHub.USB_BUS_PARAM) != ""):
            self.__vendor_id = int(self.__bench_params.get_param_value(IUSBHub.USB_BUS_PARAM), 16)
        else:
            self.__bus_id = DEFAULT_BUS_ID
        if ((self.__bench_params.has_parameter(IUSBHub.USB_HOST_DUT_PORT_PARAM)) and
                (self.__bench_params.get_param_value(IUSBHub.USB_HOST_DUT_PORT_PARAM) != "")):
            self.__usb_host_dut_port = int(self.__bench_params.get_param_value(IUSBHub.USB_HOST_DUT_PORT_PARAM))
        else:
            self.__usb_host_dut_port = 1
        # USB context hold all data necessary to handles USB devices from usb1
        self.__context_usb = None
        # The handle retreived on USB device used from USB context at usb1 level
        self.__usb_component = None

    def init(self):
        """
        Initializes the equipment. The equipment is ready to use
        """
        self.__context_usb = usb1.USBContext()
        self.__usb_component = self.__find_usb_component()
        if self.__usb_component is None:
            self._logger.error("No USB device found on host, please check the device connections (power and USB) and the following benchconfig file parameters"
                               ": %s, %s, %s"
                               % (IUSBHub.PRODUCT_ID_PARAM, IUSBHub.VENDOR_ID_PARAM, IUSBHub.USB_BUS_PARAM))

    def __find_usb_component(self):
        """
         Search for usb device connected
        :return: object
        """
        result = None
        for device in self.__context_usb.getDeviceList(skip_on_access_error=False, skip_on_error=False):
            if device.getVendorID() == self.__vendor_id and device.getProductID() == self.__product_id and device.getBusNumber() == self.__bus_id:
                result = device
                if result is not None:
                    # return USBDeviceHandle object from USBDevice
                    result = result.open()
                break

        return result

    def enable_port(self, port_number):
        """
        Activate selected port

        :param port_number:
        :rtype: bool
        :return: status of operation
        """
        status = False
        # No data required for this operation
        data_length = 0
        result = self.__usb_component.controlWrite(REQUEST_TYPE, libusb1.LIBUSB_REQUEST_SET_FEATURE,
                                                   USB_PORT_FEATURE_POWER, port_number, data_length)
        if result < 0:
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR,
                                         libusb1.libusb_error.get(result, 'Unknown error'))
        else:
            status = True

        return status

    def disable_port(self, port_number):
        """
        Deactivate selected port

        :param port_number:
        :rtype: bool
        :return: status of operation
        """
        status = False
        # No data required for this operation
        data_length = 0
        result = self.__usb_component.controlWrite(
            REQUEST_TYPE, libusb1.LIBUSB_REQUEST_CLEAR_FEATURE, USB_PORT_FEATURE_POWER,
            port_number, data_length)
        if result < 0:
            raise TestEquipmentException(TestEquipmentException.SPECIFIC_EQT_ERROR,
                                         libusb1.libusb_error.get(result, 'Unknown error'))
        else:
            status = True

        return status

    def enable_port_usb_host_device(self):
        """
        Activate port that controls host-dut usb line
        :rtype: bool
        :return: status of operation
        """
        return self.enable_port(self.__usb_host_dut_port)

    def disable_port_usb_host_device(self):
        """
        Deactivate port that controls host-dut usb line
        :rtype: bool
        :return: status of operation
        """
        return self.disable_port(self.__usb_host_dut_port)

    def release(self):
        """
            Release resources if any
        """
        pass
