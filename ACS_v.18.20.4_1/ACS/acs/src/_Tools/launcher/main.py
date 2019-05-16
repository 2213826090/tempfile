#!/usr/bin/env python
# -*- coding: utf-8 -*-

#-------------------------------------------------------------------------------
# @copyright: (c)Copyright 2014, Intel Corporation All Rights Reserved.
# The source code contained or described here in and all documents related
# to the source code ("Material") are owned by Intel Corporation or its
# suppliers or licensors. Title to the Material remains with Intel Corporation
# or its suppliers and licensors. The Material contains trade secrets and
# proprietary and confidential information of Intel or its suppliers and
# licensors.

# The Material is protected by worldwide copyright and trade secret laws and
# treaty provisions. No part of the Material may be used, copied, reproduced,
# modified, published, uploaded, posted, transmitted, distributed, or disclosed
# in any way without Intel's prior express written permission.

# No license under any patent, copyright, trade secret or other intellectual
# property right is granted to or conferred upon you by disclosure or delivery
# of the Materials, either expressly, by implication, inducement, estoppel or
# otherwise. Any license under such intellectual property rights must be express
# and approved by Intel in writing.

# @organization: INTEL MCG PSI
# @summary: "ACS launcher entry point: QtGui.QApplication()"
# @since: 5/13/14
# @author: nbrissox
#-------------------------------------------------------------------------------
import sys
import ctypes

from qt_compat import QtGui, Css, WIN32
from views.main import ACSLauncher


def main():
    """
    GUI ACS Launcher entry point

    """
    if WIN32:
        # this is mandatory, to display icon in task bar in Windows
        app_id = 'intel.acs.launcher.1.0'  # arbitrary string
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(app_id)

    app = QtGui.QApplication(sys.argv)
    app.setStyleSheet(Css.content('app'))

    icon = QtGui.QIcon(Css.image('logo'))

    app.setWindowIcon(icon)

    window = ACSLauncher(ui_filename='main')
    window.setWindowIcon(icon)
    window.show()

    return app.exec_()


if __name__ == '__main__':
    sys.exit(main())
