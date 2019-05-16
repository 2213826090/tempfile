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

:organization: PSI SI
:summary: UECmd Factory base
connectivity features
:since: 23 oct 2012
:author: sfusilie
"""
from acs_test_scripts.Device.UECmd.UECmdTypes import UECmdCat
import imp
import os
from ErrorHandling.AcsToolException import AcsToolException


class FactoryBase():

    def __init__(self, device):
        self._device = device

    def get_uecmd(self, name):
        raise AcsToolException(AcsToolException.INSTANTIATION_ERROR, "Cannot find/instanciate uecmd %s" % name)

    def _get_local_uecmd(self, path, name):
        uecmd = None
        module_path = UECmdCat.get(name, None)
        if module_path:
            module_full_path = path
            for module in module_path.split("."):
                module_full_path = os.path.join(module_full_path, module)

            if os.path.isfile(module_full_path + ".py"):
                # pylint: disable=W0631
                uecmd = imp.load_source(module, module_full_path + ".py")
            elif os.path.isfile(module_full_path + ".pyc"):
                # pylint: disable=W0631
                uecmd = imp.load_compiled(module, module_full_path + ".pyc")

            if uecmd:
                uecmd = getattr(uecmd, module)(self._device)

        return uecmd
