"""
@copyright: (c)Copyright 2013, Intel Corporation All Rights Reserved.
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

:organization: INTEL MCG QCTV SystemTest
:summary: Script for probing and comparing signals provided by the LPE
:since 04-17-2014
:author: J. Bonnal <julienX.bonnal@intel.com>, fbelvezx
"""

import time
import subprocess
import xml.etree.ElementTree as ET
from Core.PathManager import Paths
from acs_test_scripts.Device.UECmd.Imp.Android.Common.BaseV2 import BaseV2
from ErrorHandling.DeviceException import DeviceException
from ErrorHandling.AcsConfigException import AcsConfigException

NBR_MAX_MODULE = 20

## PFW commands for SCALPE platforms
PFW_ENABLE = "adb shell \"sed -i 's/TuningAllowed=\\\"false\\\"/TuningAllowed=\\\"true\\\"/' /etc/parameter-framework/ParameterFrameworkConfiguration.xml\""
PFW_SET = 'adb shell parameter setParameter'
PFW_GET = 'adb shell parameter getParameter'
PFW_TUNE = 'adb shell parameter setTuningMode on'

## LPEEXPLORER for non-SCALPE platforms
LPE_CMD_SET = 'adb shell lpeexplorer -i -w '
LPE_CMD_SUFFIX = ' -k'

# AudioAlgoController xml path
ALGOCONTROL_PATH = config_folder = Paths.CONFIGS + '\\AudioAlgoController_config.xml'


class AudioAlgoController(BaseV2):
    """
    Class that handles audio algorithms used during voice call in the DUT

    This includes the following algorithms:
     . Rx_NoiseReduction
     . Tx_NoiseReduction
     . Tx_EchoReduction
     . Tx_Equalizer
     . Rx_Equalizer
     . Sidetone_mute
     . wsHS_Phones_mute
     . wsHS_Mic_mute
    """

    def __init__(self, device):
        """
        Constructor
        """
        BaseV2.__init__(self, device)

        self.audio_arch = 'VOID'
        self.version = 'VOID'
        self.product = 'VOID'
        self.device = 'VOID'
        self.algoID = 'VOID'
        self._logger = device.get_logger()
        self.module_nbr = 0
        self.module_name = []
        for each in range(NBR_MAX_MODULE):
            self.module_name.append(str)
        self.module_path = []
        for each in range(NBR_MAX_MODULE):
            self.module_path.append(str)
            self.module_path[each] = 'VOID'
        self.module_enable_value = []
        for each in range(NBR_MAX_MODULE):
            self.module_enable_value.append(str)
        self.module_disable_value = []
        for each in range(NBR_MAX_MODULE):
            self.module_disable_value.append(str)
        self.is_scalpe_platform = False
        self.module_cmd_enable = []
        for each in range(NBR_MAX_MODULE):
            self.module_cmd_enable.append(str)
            self.module_cmd_enable[each] = 'VOID'
        self.module_cmd_disable = []
        for each in range(NBR_MAX_MODULE):
            self.module_cmd_disable.append(str)
            self.module_cmd_disable[each] = 'VOID'
        self.is_loaded_algo = False

    def pfw_setParam(self, path, value):
        """
        Launches the setParam function of Parameter Framework

        :type path: str
        :param path: path of the audio component targeted
        :type value: str
        :param value: action to perform on the component

        :return: None
        """
        self._exec('adb remount')
        if "PFW remote-processor is not running" in subprocess.check_output("adb shell parameter version", shell=True):
            try:
                self._exec(PFW_ENABLE)
                self._exec('adb shell stop media')
                self._exec('adb shell start media')
            except DeviceException as e:
                self._logger.error("Could not set tuning mode to ON - " + str(e))

        time.sleep(1)
        self._exec(PFW_TUNE)
        cmd = PFW_SET + ' ' + path + ' ' + value
        time.sleep(1)
        self._exec(cmd)

    def pfw_getParam(self, path):
        """
        Launches the getParam function of Parameter Framework

        :type path: str
        :param path: path of the audio component to get status from

        :return: None
        """
        cmd = PFW_GET + ' ' + path
        self._exec(cmd)

    def lpe_setParam(self, command):
        """
        Launches the lpeexplorer function

        :type command: str
        :param command: command to be executed by lpeexplorer

        :return: None
        """
        cmd = LPE_CMD_SET + command + LPE_CMD_SUFFIX
        self._exec(cmd)

    def find_product(self, PLATFORM, DEVICE, config_file=ALGOCONTROL_PATH):
        """
        Updates class attributes with the platform and device ID of the DUT

        :type PLATFORM: str
        :param PLATFORM: platform ID of the DUT
        :type DEVICE: str
        :param DEVICE: device ID of the DUT
        :type config_file: str
        :param config_file: path to the xml config file for audio algorithms control

        :return: None
        """
        tree = ET.parse(config_file)
        root = tree.getroot()

        for platform_root in root.findall('platform_description'):
            for platform in platform_root.findall('platform'):
                if platform.get('name') == PLATFORM:
                    for device in platform.findall('device'):
                        if device.get('name') == DEVICE:
                            self.device = device
                            self.audio_arch = device.find('audio_arch').text
                            if self.audio_arch == "scalpe_audio_arch":
                                self.is_scalpe_platform = True
                            self.version = device.find('version').text
        for arch in root.findall('audio_arch'):
            if arch.get('name') == self.audio_arch:
                for version in arch.findall('version'):
                    if version.get('name') == self.version:
                        self.product = version
                        return 1

        return 0

    def find_algo(self, ALGO_ID):
        """
        Check that an input audio algorithm is valid, and update the corresponding fields

        :type ALGO_ID: str
        :param ALGO_ID: Name of the audio algorithm

        :rtype: bool
        :return: A boolean corresponding to the outcome of the verification of the input audio algorithm
        """
        product = self.product
        for algo in product.findall('algo'):
            name_algo = algo.get('name')
            if name_algo == ALGO_ID:
                self.algoID = name_algo
                self.module_nbr = int(algo.find('nb_modules').text)
                ii = 0
                for module in algo.findall('module'):
                    self.module_name[ii] = module.get('name')
                    self.module_enable_value[ii] = module.find('enable_value').text
                    self.module_disable_value[ii] = module.find('disable_value').text
                    if self.is_scalpe_platform:
                        self.module_path[ii] = module.find('path').text
                    else:
                        self.module_cmd_enable[ii] = module.find('cmd_enable').text
                        self.module_cmd_disable[ii] = module.find('cmd_disable').text
                    ii += 1
                if self.is_scalpe_platform:
                    if (self.module_path[self.module_nbr - 1] != 'VOID') & (
                            self.module_path[self.module_nbr] == 'VOID'):
                        self.is_loaded_algo = True
                else:
                    if (self.module_cmd_disable[self.module_nbr - 1] != 'VOID') & (
                            self.module_cmd_disable[self.module_nbr] == 'VOID'):
                        self.is_loaded_algo = True
                return 1
        return 0

    def enable_algo(self):
        """
        Enables an audio algorithm

        :return: None
        """
        if self.is_loaded_algo == False:
            self._logger.warning('no algo previously loaded.')

        for ii in range(self.module_nbr):
            self._logger.info(self.module_name[ii] + ' set to ON (' + self.module_enable_value[ii] + ')')

            if self.is_scalpe_platform:
                self.pfw_setParam(self.module_path[ii], self.module_enable_value[ii])
            else:
                self.lpe_setParam(self.module_cmd_enable[ii])

    def disable_algo(self):
        """
        Disables an audio algorithm

        :return: None
        """
        if not self.is_loaded_algo:
            self._logger.warning('no algo previously loaded.')

        for ii in range(self.module_nbr):
            self._logger.info(self.module_name[ii] + ' set to OFF (' + self.module_disable_value[ii] + ')')

            if self.is_scalpe_platform:
                self.pfw_setParam(self.module_path[ii], self.module_disable_value[ii])
            else:
                self.lpe_setParam(self.module_cmd_disable[ii])

    def manage_audio_algo(self, algo, action):
        """
        Enables or disables an audio algorithm

        :type algo: str
        :param algo: algorithm on which to perform the action
        :type action: str
        :param action: acrion to perform on the algorithm

        :return: None
        """
        [PLATFORM, PRODUCT] = self.list_platform_details()
        if self.find_product(PLATFORM, PRODUCT, ALGOCONTROL_PATH):

            if self.find_algo(algo):
                self._logger.info(action + ' ' + algo)
            else:
                self._logger.error(algo + '_' + 'algorithm not found')
                raise AcsConfigException(
                    AcsConfigException.INVALID_PARAMETER,
                    algo + '_' + 'algorithm not found')

            if action == 'ENABLE':
                self.enable_algo()
            elif action == 'DISABLE':
                self.disable_algo()
            else:
                self._logger.warning('Invalid action parameter. Will do nothing.')

        else:
            self._logger.warning('Unknown platform/product. Will not %s %s.' % (action.lower(), algo))

    def list_platform_details(self):
        """
        Lists platform details (platform type, and product type for a given platform)

        :return: None
        """
        cmd = 'adb shell getprop ro.board.platform'
        platform = self._exec(cmd)
        platform = platform.strip(' ')
        cmd = 'adb shell getprop ro.product.device'
        product = self._exec(cmd)
        product = product.strip(' ')

        return [platform, product]
