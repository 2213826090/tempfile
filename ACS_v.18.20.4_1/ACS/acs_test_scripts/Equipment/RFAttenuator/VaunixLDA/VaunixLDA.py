"""
:copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related to the source code ("Material") are owned by
Intel Corporation or its suppliers or licensors. Title to the Material remains with Intel Corporation or its suppliers
and licensors. The Material contains trade secrets and proprietary and confidential information of Intel or its
suppliers and licensors.

The Material is protected by worldwide copyright and trade secret laws and treaty provisions. No part of the Material
may be used, copied, reproduced, modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual property right is granted to or conferred
upon you by disclosure or delivery of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express and approved by Intel in writing.

:organization: INTEL MCG
:summary: This file implements the driver for the Vaunix Lab Brick Digital Attenuator
:since: 10/02/2015
:author: jfranchx

"""

from ErrorHandling.TestEquipmentException import TestEquipmentException
from ErrorHandling.AcsConfigException import AcsConfigException
from acs_test_scripts.Equipment.IEquipment import EquipmentBase
from acs_test_scripts.Equipment.RFAttenuator.Interface.IRFAttenuator import IRFAttenuator
from acs_test_scripts.Equipment.IEquipment import DllLoader
import ctypes


class VaunixLDA(DllLoader, EquipmentBase, IRFAttenuator):
    """
    Implementation of Vaunix Lab Brick Digital Attenuator
    """

    TEST_MODE = ctypes.c_bool(False)
    SUPPORTED_DEVICES_MODELS = ["LDA-602"]
    # db unit output of dll is 0.25db
    DB_UNIT_COEFFICIENT = 4
    STATUS_OK = 0
    DEVID = ctypes.c_uint32

    def __init__(self, name, model, eqt_params, bench_params):
        """
        Constructor
        """
        # Initialize class parent
        EquipmentBase.__init__(self, name, model, eqt_params)
        DllLoader.__init__(self, name, model, eqt_params)

        self._serial_number = str(bench_params.get_param_value("SerialNumber"))
        self._model_name = str(bench_params.get_param_value("EqtModel"))

        self._dll = None
        self._device_id = None
        self._max_attenuation = None
        self._min_attenuation = None
        self._test_mode = self.TEST_MODE

    def init(self):
        """
        Initialize and connect to Vaunix LDA.
        """
        self.load_driver("STANDARD")
        self._dll = self.get_dll()
        self._configure_dll_functions()

        self._logger.debug("Vaunix LDA DLL set test mode to %s" % str(self.TEST_MODE.value))
        self._dll.fnLDA_SetTestMode(self.TEST_MODE)

        # Check if devices are detected
        lda_devices = self._dll.fnLDA_GetNumDevices()
        if lda_devices <= 0:
            msg = "No Vaunix LDA device detected"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)
        self._logger.debug("%s Vaunix LDA devices detected" % lda_devices)

        # Get devices IDs and search our device
        lda_dev_info = ctypes.POINTER(self.DEVID)(self.DEVID(0))
        lda_dev_info_nb = self._dll.fnLDA_GetDevInfo(lda_dev_info)

        index = 0
        while index < lda_dev_info_nb:
            current_dev = lda_dev_info[index]
            current_dev_model = ctypes.c_char_p("----------")
            current_dev_model_length = self._dll.fnLDA_GetModelName(self.DEVID(current_dev), current_dev_model)
            if current_dev_model.value == self._model_name:
                # Correct model, check serial number
                current_dev_serial_number = self._dll.fnLDA_GetSerialNumber(self.DEVID(current_dev))
                if str(current_dev_serial_number) == str(self._serial_number):
                    self._logger.debug("Vaunix LDA %s found - DEVID %s" % (self._serial_number, current_dev))
                    self._device_id = current_dev
                    index = lda_dev_info_nb
            index += 1

        if self._device_id is None:
            msg = "Vaunix LDA %s can't be found"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)

        result = self._dll.fnLDA_InitDevice(self.DEVID(self._device_id))
        if result != self.STATUS_OK:
            msg = "Fail to init connection to Vaunix LDA device"
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.CONNECTION_ERROR, msg)

        # Get max/min attenuation
        max_attenuation = self._dll.fnLDA_GetMaxAttenuation(self.DEVID(self._device_id))
        min_attenuation = self._dll.fnLDA_GetMinAttenuation(self.DEVID(self._device_id))
        self._max_attenuation = max_attenuation / self.DB_UNIT_COEFFICIENT
        self._min_attenuation = min_attenuation / self.DB_UNIT_COEFFICIENT

    def release(self):
        """
        Release connection to the Vaunix LDA.
        """
        self._check_device_id()
        try:
            result = self._dll.fnLDA_CloseDevice(self.DEVID(self._device_id))
            if result != self.STATUS_OK:
                msg = "Error on Vaunix LDA %1 release" % self._device_id
                self._logger.error(msg)
                raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
        finally:
            self.unload_driver()

    def set_attenuation(self, attenuation):
        """
        Sets the attenuation. This method will round the value to the closest step supported by the attenuator.

        :type attenuation: float
        :param attenuation: Attenuation value (between MIN_ATTN_VALUE and MAX_ATTN_VALUE)
        """
        self._check_device_id()
        if attenuation not in range(self._min_attenuation, self._max_attenuation) and attenuation != self._max_attenuation:
            msg = "Error set_attenuation, attenuation parameter is out of range(%s, %s) : %s" % (self._min_attenuation, self._max_attenuation, attenuation)
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.INVALID_PARAMETER, msg)

        current_attenuation = ctypes.c_uint32(int(attenuation) * self.DB_UNIT_COEFFICIENT)
        self._logger.debug("Vaunix LDA set attenuation to %s" %  attenuation)
        result = self._dll.fnLDA_SetAttenuation(self.DEVID(self._device_id), current_attenuation)
        if result != self.STATUS_OK:
            msg = "Error fnLDA_SetAttenuation : %s" % result
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)

    def get_attenuation(self):
        """
        Gets the current attenuation.

        :rtype: float
        :return: current attenuation value
        """
        self._check_device_id()
        current_attenuation = self._dll.fnLDA_GetAttenuation(self.DEVID(self._device_id)) / self.DB_UNIT_COEFFICIENT
        if current_attenuation not in range(self._min_attenuation, self._max_attenuation) and current_attenuation != self._max_attenuation:
            msg = "Error get_attenuation, result is out of range : %s" % current_attenuation
            self._logger.error(msg)
            raise TestEquipmentException(TestEquipmentException.OPERATION_FAILED, msg)
        return current_attenuation

    def get_max_attn(self):
        """
        Parses the attenuator model number and computes the maximal attenuation from it.
        :rtype: float
        :return: The maximum attenuator for the current attenuator.
        """
        return float(self._max_attenuation)

    def get_min_attn(self):
        """
        Parses the attenuator model number and computes the minimal attenuation from it.
        :rtype: float
        :return: The minimal attenuator for the current attenuator.
        """
        return float(self._min_attenuation)

    # ------------------------------------------------------------------------------------------------------------------

    def _check_device_id(self):
        """
        Check if the device is correctly identified
        """
        if self._device_id is None:
            msg = "Error Vaunix LDA is not identified - execute init function to identify it"
            self._logger.error(msg)
            raise AcsConfigException(AcsConfigException.OPERATION_FAILED, msg)

    def _configure_dll_functions(self):
        """
        Configure arguments types of dll functions.
        Check PDF documentation for more precisions.
        """
        # Selecting device functions
        self._dll.fnLDA_SetTestMode.argtypes = [ctypes.c_bool]
        self._dll.fnLDA_GetDevInfo.argtypes = [ctypes.POINTER(self.DEVID)]
        self._dll.fnLDA_GetModelName.argtypes = [self.DEVID, ctypes.c_char_p]
        self._dll.fnLDA_GetSerialNumber.argtypes = [self.DEVID]
        self._dll.fnLDA_InitDevice.argtypes = [self.DEVID]
        self._dll.fnLDA_CloseDevice.argtypes = [self.DEVID]

        # Setting parameters functions
        self._dll.fnLDA_SetAttenuation.argtypes = [self.DEVID, ctypes.c_uint32]
        self._dll.fnLDA_SetRampStart.argtypes = [self.DEVID, ctypes.c_uint32]
        self._dll.fnLDA_SetRampEnd.argtypes = [self.DEVID, ctypes.c_uint32]
        self._dll.fnLDA_SetAttenuationStep.argtypes = [self.DEVID, ctypes.c_uint32]
        self._dll.fnLDA_SetDwellTime.argtypes = [self.DEVID, ctypes.c_uint32]
        self._dll.fnLDA_SetIdleTime.argtypes = [self.DEVID, ctypes.c_uint32]
        self._dll.fnLDA_SetRFOn.argtypes = [self.DEVID, ctypes.c_bool]
        self._dll.fnLDA_SetRampDirection.argtypes = [self.DEVID, ctypes.c_bool]
        self._dll.fnLDA_SetRampMode.argtypes = [self.DEVID, ctypes.c_bool]
        self._dll.fnLDA_StartRamp.argtypes = [self.DEVID, ctypes.c_bool]
        self._dll.fnLDA_SaveSettings.argtypes = [self.DEVID]

        # Reading parameters functions
        self._dll.fnLDA_GetAttenuation.argtypes = [self.DEVID]
        self._dll.fnLDA_GetRampStart.argtypes = [self.DEVID]
        self._dll.fnLDA_GetRampEnd.argtypes = [self.DEVID]
        self._dll.fnLDA_GetAttenuationStep.argtypes = [self.DEVID]
        self._dll.fnLDA_GetDwellTime.argtypes = [self.DEVID]
        self._dll.fnLDA_GetIdleTime.argtypes = [self.DEVID]
        self._dll.fnLDA_GetRF_On.argtypes = [self.DEVID]
        self._dll.fnLDA_GetMaxAttenuation.argtypes = [self.DEVID]
        self._dll.fnLDA_GetMinAttenuation.argtypes = [self.DEVID]
