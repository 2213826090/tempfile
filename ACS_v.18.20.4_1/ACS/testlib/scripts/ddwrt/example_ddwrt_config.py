#!/usr/bin/env python

##############################################################################
#
# @filename:    steps.py
# @description: Configure ddwrt router example
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################

from testlib.scripts.ddwrt import ddwrt_steps
from testlib.scripts.connections.local import local_steps


DDWRT_IP = "10.237.100.219"
DDWRT_USER = "root"

DDWRT_SCRIPTS_PATH = "../../resources/ddwrt/"


ddwrt_steps.set_cfg(config = "mode",
                    value = "n",
                    ssh_host = DDWRT_IP,
                    ssh_user = DDWRT_USER,
                    media_path = DDWRT_SCRIPTS_PATH)()
ddwrt_steps.set_cfg(config = "security",
                    value = "wep_128",
                    ssh_host = DDWRT_IP,
                    ssh_user = DDWRT_USER,
                    media_path = DDWRT_SCRIPTS_PATH)()
local_steps.ssh_command(command = "reboot",
                        ssh_host = DDWRT_IP,
                        ssh_user = DDWRT_USER)()
