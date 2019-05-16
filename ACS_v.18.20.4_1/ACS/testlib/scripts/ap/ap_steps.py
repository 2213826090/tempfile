#!/usr/bin/env python

##############################################################################
#
# @filename:    ap_steps.py
# @description: Defines the interface for the different AP modules implementation.
# @author:      aurel.constantin@intel.com
#
##############################################################################


from testlib.base.abstract.abstract_step import abstract_step
from testlib.base.abstract import abstract_utils

USE_MODULE = abstract_utils.get_module("ap_module")
if USE_MODULE == None:
    # default 'ddwrt'
    USE_MODULE = "ddwrt_steps"


@abstract_step(use_module = USE_MODULE)
class setup():
    """ description:
            sets a new configuration on the AP.

        required parameters:
            mode
            security

               Valid values for "mode" = "b",
                                           "bg",
                                           "g",
                                           "mixed",
                                           "n",
                                           "ng"
                            for "security" = "none",
                                               "wpa_psk_mixed",
                                               "wpa_psk",
                                               "wpa2",
                                               "wpa_enterprise",
                                               "wpa2_enterprise",
                                               "wep64",
                                               "wep128"
        optional parameters:
            ssh_user (default is 'root')
            encryption
            wifi_password
            radius_ip
            radius_secret
            ssh_pwd
    """
    pass


@abstract_step(use_module = USE_MODULE)
class reboot():
    pass


@abstract_step(use_module = USE_MODULE)
class reload_ap():
    pass


@abstract_step(use_module = USE_MODULE)
class set_ap_wireless():
    pass


@abstract_step(use_module = USE_MODULE)
class setup_virtual_interface():
    pass


@abstract_step(use_module = USE_MODULE)
class radvd_enable():
    pass


@abstract_step(use_module = USE_MODULE)
class set_ipv6():
    pass
