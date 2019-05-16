"""
@copyright: (c)Copyright 2015, Intel Corporation All Rights Reserved.
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

:organization: INTEL NDG SW DEV
:summary: This file implements the get em info test step
:since 01/22/2015
:author: jreynaux
"""
from TestStep.Device.EnergyManagement.EmBase import EmBase


class GetEmInfo(EmBase):
    """
    Implements the get em info test step

    This step retrieve data retrieved from get_msic_registers() and store them as is in the context !
    """

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """
        EmBase.run(self, context)

        self._logger.info("Getting the current msic register infos ...")

        # Where the information will be stored into the context?
        variable_to_set = self._pars.save_info_as

        behavior = self._pars.behavior
        behavior_value = self._pars.behavior_value

        # Get msic register values
        result = self._api.get_msic_registers(behavior=behavior, behavior_value=behavior_value)

        if result is not None and isinstance(result, dict):

            for info in result.keys():
                # Should have TIME_STAMP = (str, str), 'BATTERY' = dict, 'CHARGER'= dict ...
                # if not dict store as is
                if not isinstance(result[info], dict):
                    key = "{0}:{1}".format(variable_to_set, info)
                    value = result[info][0]
                    context.set_info(key, value)
                    self._logger.debug("{0} = {1}".format(key, value))
                else:
                    # should be a dict
                    # ex {'BATTERY': {'CAPACITY': (89, 'none'), 'VOLTAGE': (3.904, 'V'), 'TEMP': (8, 'DegreeCelsius')}
                    for sub_info in result[info].keys():
                        key = "{0}:{1}:{2}".format(variable_to_set, info, sub_info)
                        value = result[info][sub_info][0]
                        context.set_info(key, value)
                        self._logger.debug("{0} = {1}".format(key, value))
        elif result is not None and isinstance(result, int):
            # in case of scheduled action, just store the pid
            context.set_info(variable_to_set, result)