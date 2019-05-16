"""
@summary: Get sensor values using Cittest embedded application.
The 3 components of the sensor value (one for each axis) are extracted
and stored in context variables specified in parameters.
Assume that Cittest is already launched at test step beginning
@since 12 August 2014
@author: Souyris Matthieu
@organization: INTEL PEG-SVE-DSV

@copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
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
otherwise. Any license under such intellectual property rights must be expressed
and approved by Intel in writing.
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
from ErrorHandling.DeviceException import DeviceException
from UtilitiesFWK.Utilities import Global


class GetSensorValuesWithCittest(DeviceTestStepBase):
    """
    Get sensor components values using cittest
    """

    # UI operation set of each sensor to launch to do sensor measurement (operation set are in UI dictionary,
    # whose name is defined by in device_catalog by uiDictionnaryName attribute. Dictionaries are in
    # acs_test_scripts/Device/UECmd/Imp/Android/Common/Ui/OpDictionary directory
    __UIOpSet = {'ACC': 'cittest_get_acc_values',
                 'GYR': 'cittest_get_gyr_values'}

    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self._api = self._device.get_uecmd("PhoneSystem")

        # Get ue command UI instance
        self._ui_api = self._device.get_uecmd("Ui")
        self._ui_api.set_global_config(global_conf)
        self._ui_api.init()

    def __exec_cmd(self, cmd_to_exec):
        """
        Exec an adb command and take verdict into account
        :type cmd_to_exec: str
        :param cmd_to_exec: adb command to exec
        :return: content printed by the command on stdout
        """
        verdict, res_cmd = self._device.run_cmd(cmd_to_exec, timeout=10)
        if verdict == Global.FAILURE:
            raise DeviceException(DeviceException.OPERATION_FAILED,
                                  "GetSensorValuesWithCittest: unable to run_cmd: "+cmd_to_exec)
        return res_cmd

    def __get_component_from_log(self, component_num):
        """
        Extract one sensor component from logs
        :type component_num: int
        :param component_num: component number (0 for x, 1 for y and 2 for z)
        :return: string representing component value
        """
        if component_num not in [0, 1, 2]:
            msg = "GetSensorValuesWithCittest: unexpected component number '{0}'".format(component_num)
            self._logger.error(msg)
            raise DeviceException(DeviceException.OPERATION_FAILED, msg)

        # Old command line for previous cittest version
        #cmd = "adb logcat -d | grep 'SensorTestActivity.*values\[{0}\]' | cut -f2 -d'=' | tr -d ' ' | tail -1"\
        #    .format(component_num)

        # 1st part of the extraction command: get and format all components output
        cmd = "adb logcat -d |grep 'HAL:.*data.*:' |cut -f4 -d: |sed -e 's/ *--.*//' -e 's/^ *//'"
        # 2nd part of the extraction command: select only the values of the desired component then get the last value
        cmd += " |cut -f{0} -d' ' |tail -n1".format(component_num+1)
        self._logger.debug("GetSensorValuesWithCittest: extraction command: '{0}'".format(cmd))

        return self.__exec_cmd(cmd)

    def __check_component(self, component_value, error_message):
        """
        Check that the component value given in parameter is acceptable (not None nor "")
        :type component_value: string
        :param component_value: component value to check
        :type error_message: string
        :param error_message: message to log if component value is incorrect
        """
        if component_value in ["", None]:
            self._logger.error("GetSensorValuesWithCittest: "+error_message)
            raise DeviceException(DeviceException.OPERATION_FAILED, "GetSensorValuesWithCittest: "+error_message)

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """

        DeviceTestStepBase.run(self, context)
        self._logger.debug("GetSensorValuesWithCittest: used sensor: "+self._pars.sensor_name)

        # Flush logs
        self.__exec_cmd("adb shell logcat -c")
        self._logger.debug("GetSensorValuesWithCittest: logs flushed")

        # Launch sensor components logging and extract their values from log.
        # Up to 3  attempts are done if one or more component can't be extracted
        x_compo_extracted, y_compo_extracted, z_compo_extracted = "", "", ""
        nb_retry = 0
        while nb_retry < 3:
            self._logger.debug("GetSensorValuesWithCittest: Component extraction attempt No {0}.".format(nb_retry+1))
            self._logger.debug("GetSensorValuesWithCittest: Launch {0} UI scenario"
                               .format(self.__UIOpSet[self._pars.sensor_name]))
            # Launch right UI scenario to check / uncheck right sensor checkbox
            self._ui_api.run_operation_set(self.__UIOpSet[self._pars.sensor_name])

            # Parse log in order to extract the 3 components and save them into the context
            x_compo_extracted = self.__get_component_from_log(0)
            y_compo_extracted = self.__get_component_from_log(1)
            z_compo_extracted = self.__get_component_from_log(2)

            # If all sensor components have been correctly extracted, terminate test step
            if x_compo_extracted not in [None, ""] and y_compo_extracted not in [None, ""] \
                    and z_compo_extracted not in [None, ""]:
                break

            # One or more component have not been extracted, try another acquisition
            nb_retry += 1

        self.__check_component(x_compo_extracted, "extraction of x sensor component failed")
        self.__check_component(y_compo_extracted, "extraction of y sensor component failed")
        self.__check_component(z_compo_extracted, "extraction of z sensor component failed")

        # Sensor components correctly extracted, save them into the context
        context.set_info(self._pars.x_comp, x_compo_extracted)
        context.set_info(self._pars.y_comp, y_compo_extracted)
        context.set_info(self._pars.z_comp, z_compo_extracted)

        ts_verdict_msg = "VERDICT: sensor components ({0},{1},{2}) stored in ({3},{4},{5})".format(
            x_compo_extracted, y_compo_extracted, z_compo_extracted,
            self._pars.x_comp, self._pars.y_comp, self._pars.z_comp)
        self._logger.debug(ts_verdict_msg)
