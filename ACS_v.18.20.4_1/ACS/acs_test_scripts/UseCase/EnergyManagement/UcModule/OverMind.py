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
:summary: This class is a singleton that keep a reference on any useful object use by em usecase and em module
:author: vgomberx
:since: 17/12/2014
"""
from ErrorHandling.AcsBaseException import AcsBaseException
from Core.Report.ACSLogging import LOGGER_TEST_SCRIPT
from UtilitiesFWK.Utilities import Singleton
import weakref


class OverMind(object):
    """
    """
    __metaclass__ = Singleton
    __LOG_TAG = "[OVERMIND]\t"
    # common usecase object
    DEVICE = "DEVICE"
    TC_PARAMETERS = "TC_PARAMETERS"
    LOGGER = "LOGGER"
    EQUIPMENT_MANAGER = "EQUIPMENT_MANAGER"
    BENCH_CONFIG = "BENCH_CONFIG"
    DUT_CONFIGURATION = "DUT_CONFIGURATION"
    # em module object

    def __init__(self):
        """
        uecmd got already a way to retrieve existing object.
        this module will store all necessary object need for module
        module should call it and try to get what they need
        """
        self.__logger = LOGGER_TEST_SCRIPT
        self.__logger.info(self.__LOG_TAG + "Created")
        self.__ref_dico = {}

    def __del__(self):
        """
        class destructor
        """
        self.__ref_dico.clear()
        del self.__ref_dico

    def init(self, device=None, tc_parameters=None, equipment_manager=None, bench_config=None, dut_config=None):
        """
        init the overmind with given value and delete others
        """
        # delete the old red dico
        self.__logger.info(self.__LOG_TAG + "Init")
        self.__ref_dico.clear()
        del self.__ref_dico
        # create a new one with his entry
        self.__ref_dico = {}
        if device is not None:
            self.store_instance(OverMind.DEVICE, device)
        if tc_parameters is not None:
            self.store_instance(OverMind.TC_PARAMETERS, tc_parameters)
        if equipment_manager is not None:
            self.store_instance(OverMind.EQUIPMENT_MANAGER, equipment_manager)
        if bench_config is not None:
            self.store_instance(OverMind.BENCH_CONFIG, bench_config)
        if dut_config is not None:
            self.store_instance(OverMind.DUT_CONFIGURATION, dut_config)

    def store_instance(self, name, obj):
        """
        store an instance with a given name as key
        """
        self.__logger.warning("OVERMIND store OBJECT %s" % (name))
        if self.__ref_dico.has_key(name):
            msg = self.__LOG_TAG + "key %s already exist in known object" % name
            self.__logger.error(msg)
            raise AcsBaseException(AcsBaseException.OPERATION_FAILED, msg)
        # store a weak reference to avoid locking object
        self.__ref_dico[name] = weakref.proxy(obj)

    def get_instance(self, name, raise_error=True):
        """
        get an instance by its stored name
        """
        if raise_error:
            if  name not in self.__ref_dico.keys():
                msg = self.__LOG_TAG + "key %s does not exist" % name
                self.__logger.error(msg)
                raise AcsBaseException(AcsBaseException.OPERATION_FAILED, msg)

        a = self.__ref_dico.get(name)
        self.__logger.debug("OVERMIND get OBJECT %s" % (name))
        return a

