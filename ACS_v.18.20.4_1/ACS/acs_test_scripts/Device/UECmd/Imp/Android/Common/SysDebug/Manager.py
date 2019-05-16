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
:summary: Sysdebug manager
:since: 13/03/2013
:author: pbluniex
"""
import os
import imp
import time
from lxml import etree
from acs_test_scripts.Device.UECmd.Imp.Android.Common.Base import Base
from acs_test_scripts.Device.UECmd.Interface.SysDebug.ISysDebug import ISysDebug


from acs_test_scripts.Utilities.ModuleLoaderUtilities import ModuleLoader
class Manager(Base, ISysDebug):

    """
    Manager for Sysdebug plugins
    """

    def __init__(self, device):
        """
        Constructor

        :type device: Device
        :param device: The Device under test
        """
        Base.__init__(self, device)
        ISysDebug.__init__(self, device)
        self.__sys_dbg_module = {}
        self.__start_timestamp = None
        self.__stop_timestamp = None
        self.__host_offset = None

    def __get_module(self, name, config):
        """
        Get the module for the SysDebug class defined by name

        :type name: str
        :param name: The name of SysDebug module

        :type config: dictionnary
        :param config: The configuration of the sysdebug module.
                       The keys of the dictionary are the parameters of the module.
                       Values are the parameter value if any, None otherwise

                       e.g. :
                       The configuration value :

                           "param1=val1, param2=val2, param3"

                       will be represented by the dictionary:

                           config = {"param1": val1,
                                     "param2": val2,
                                     "param3": None}

        :rtype: dictionary
        :return: The configuration compiled in a dictionary
        """
        uecmd = None
        path = os.path.dirname(os.path.abspath(__file__))
        module_file = os.path.join(path, "Modules", name)
        if os.path.isfile(module_file + ".py"):
            uecmd = imp.load_source(name, module_file + ".py")
        elif os.path.isfile(module_file + ".pyc"):
            uecmd = imp.load_compiled(name, module_file + ".pyc")

        module = None
        if uecmd:
            module = getattr(uecmd, name)(self._device, config)

        if module:
            self._logger.info("SysDebug module '%s' successfuly loaded" % name)
        else:
            self._logger.error("SysDebug module '%s' not loaded" % name)

        return module

    def __parse_module_configuration(self, module):
        """
        Configuration of one module. This will enabled the module

        :type module: str
        :param module: The configuration item.
                       Each module can be activated in the test case parameter.
                       The configuration must follows the syntax (regular expression):
                           (?P<ModuleName>[^:]+)[\s*][:[\s*]config];

                       e.g.:
                           * Activate a module without configuration
                           ModuleName;

                           * Activate a module with configuration
                           ModuleName: config;
        """
        if not module:
            return

        conf = module.split(":")
        mod_name = conf[0].strip()
        config = None
        if len(conf) == 2:
            parameters = conf[1].strip()
            config = self.__parse_module_parameters(parameters)
        elif len(conf) > 2:
            self._logger.error("Can not configure sysdebug module : "
                               "invalid syntax (%s)" % module)
            return

        sysdebug_module = self.__get_module(mod_name, config)
        if sysdebug_module:
            self.__sys_dbg_module[mod_name] = sysdebug_module

    def __parse_module_parameters(self, parameters):
        """
        Parse parameters of a module

        :type parameters: str
        :param parameters: str representation of the parameters of the module
                           The configuration must follow the syntax (regular expression)
                           (?P<param1>\w+)[\s*=\s*(?P<val1>\w+)]?[\s,\s*(?P<paramX>\w+[=(?P<valX>\w+)]]*

                           e.g. :
                               * Parameter with empty value
                               param1

                               * Parameter with value
                               param1 = val1

                               * Parameters
                               param1, param2 = val2, param3
        :rtype: dictionary
        :return: The parameters of modules compiled in a dictionary
        """

        config = {}

        for param in parameters.split(","):
            paramconf = param.split("=")
            prm_name = paramconf[0].strip()
            config[prm_name] = None
            if len(paramconf) == 2:
                config[prm_name] = paramconf[1].strip()
            elif len(paramconf) > 2:
                self._logger.error("Can not configure sysdebug module "
                                   "parameters (%s)" % param)
                return

        return config

    def __date_offset(self):
        """
        Get the offset of host and device dates
        """
        # Compute host threshold timestamp
        hosttime = time.time()
        res = self._exec("adb shell date +%s",
                         force_execution=True)
        date = int(res)
        self.__host_offset = date - hosttime

    def synchronize(self):
        """
        Call synchronization method of sysdebug modules
        :type config: Device_config
        :param config: Configuration for modules.
                       Each configurable module must have an entry in the dictionary.

        :rtype: boolean
        :return: True if the module is synchronized, False otherwise
        """
        for module in self.__sys_dbg_module.values():
            if not module.synchronize():
                return False

        return True

    def reset(self, delay_s=0):
        """
        Call reset method of sysdebug modules
        """
        for module in self.__sys_dbg_module.values():
            module.reset(delay_s)

    def init(self, config=None):
        """
        Initialization of the sysdebug modules.
        At this step, the board must be connected

        :type config: str
        :param config: The configuration of sysdebug.
                       The configuration follows the syntax:
                           Module1[: config1];
                           Module2[: config2];

        """
        self.__date_offset()

        dbgfs = self._exec("adb shell mount|grep debugfs || echo nok", 1,
                           force_execution=True)

        if dbgfs == "nok":
            self._logger.debug("Mount debugfs filesystem")
            self._exec("adb shell mount -t debugfs none /sys/kernel/debug", 1,
                       force_execution=True)
        else:
            self._logger.debug("Filesystem debugfs already mounted")

        path = os.path.dirname(os.path.abspath(__file__))
        module_path = os.path.join(path, "Modules")
        modloader = ModuleLoader(self._device, module_path)

        self.__sys_dbg_module = modloader.load(config)
        for modname, module in self.__sys_dbg_module.items():
            if not module.init():
                self._logger.error("Module '%s' could not initialize so we remove it." % modname)
                self.__sys_dbg_module.pop(modname)

    def start(self):
        """
        Set trigger time start
        """
        if self.__host_offset is not None:
            self.__start_timestamp = time.time() + self.__host_offset
            self._logger.debug("Start timer at %d (%s) - offset : %d" %
                       (self.__start_timestamp,
                        time.ctime(self.__start_timestamp),
                        self.__host_offset))

    def stop(self):
        """
        Set trigger time stop and fill wakelocks messages in list
        """
        if self.__host_offset is not None:
            self.__stop_timestamp = time.time() + self.__host_offset
            self._logger.debug("Stop timer at %d (%s) - offset : %d" %
                               (self.__stop_timestamp,
                                time.ctime(self.__stop_timestamp),
                                self.__host_offset))

        # Wait a second to prevent sysdebug events occur in same stop second
        time.sleep(1)

    def fetch(self):
        """
        Fetch  board. Some sysdebug informations need to have access
        to the board to be fetched.

        :type wait: Integer
        :param wait: Tempo to wait before fetching information on the device.
        """
        for module in self.__sys_dbg_module.values():
            module.fetch()

    def report(self):
        """
        Return statistic vue for results of sysdebug modules

        :rtype: etree.Element
        :return: The Xml tree of Sysdebug modules
        """
        xmltree = etree.Element("SysDebug")
        aplogs = []

        for module in self.__sys_dbg_module.values():
            # provide aplogs to the module (if already retrieved)
            if aplogs:
                module.set_aplog_messages(aplogs)

            xmlelement = module.stats(self.__start_timestamp, self.__stop_timestamp)
            xmltree.append(xmlelement)

            # retrieve aplogs for the next module
            if not aplogs:
                aplogs = module.get_aplog_messages()

        return xmltree

    def report_current_data(self):
        """
        Return statistic view for results of sysdebug modules at the current time (without having called stop)

        :rtype: etree.Element
        :return: The Xml tree of Sysdebug modules
        """
        xmltree = etree.Element("SysDebug")
        cur_time = time.time() + self.__host_offset + 1
        self._logger.debug("Reporting current sysdebug data between [%s - %s]" % (time.ctime(self.__start_timestamp), time.ctime(cur_time)))
        aplogs = []

        for module in self.__sys_dbg_module.values():

            # provide aplogs to the module (if already retrieved)
            if aplogs:
                module.set_aplog_messages(aplogs)

            xmlelement = module.stats(self.__start_timestamp, cur_time)
            xmltree.append(xmlelement)

            # retrieve aplogs for the next module
            if not aplogs:
                aplogs = module.get_aplog_messages()

        return xmltree
