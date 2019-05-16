"""
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
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:organization: INTEL NDG SW
:summary: This file implements a Test Step for setting wifi configuration
:since 12/06/2014
:author: floeselx
"""
from acs_test_scripts.TestStep.Device.Wireless.Wifi.WifiBase import WifiBase
from UtilitiesFWK.Utilities import split_and_strip


class WifiAddNetwork(WifiBase):
    """
    Set the Wifi network configuration
    """

    def run(self, context):
        WifiBase.run(self, context)

        # Common parameters (mandatory)
        ssid = str(self._pars.ssid)
        security = str(self._pars.security)
        pass_phrase = str(self._pars.pass_phrase)

        # Optional parameters
        eap_method = str(self._pars.eap_method)
        eap_user = str(self._pars.eap_user)
        eap_password = str(self._pars.eap_password)
        if str(self._pars.phase2_auth) == "NONE":
            phase2_auth = "None"
            self._logger.info("phase2_auth :  %s" % (phase2_auth))
        else:
            phase2_auth = str(self._pars.phase2_auth);

        use_certificate = self._pars.use_certificate
        certificat_name = str(self._pars.certificat_name)

        if self._pars.static_ip is None:
            static_ip = "dhcp"
        else:
            static_ip = str(self._pars.static_ip).lower()
        netmask = str(self._pars.ip_netmask)
        ip_address = str(self._pars.ip_address)
        gateway = str(self._pars.ip_gateway)
        dns1 = str(self._pars.ip_dns1)
        dns2 = str(self._pars.ip_dns2)

        if self._pars.proxy_config is None:
            proxy_config = "NONE"
        else:
            proxy_config = str(self._pars.proxy_config)
        proxy_address = str(self._pars.proxy_address)
        proxy_port = str(self._pars.proxy_port)
        proxy_bypass = str(self._pars.proxy_bypass)

        if security in ["EAP-WPA", "EAP-WPA2"]:
            # Compute passphrase for WPA-ENT (will be parsed by ACS Embedded)
            pass_phrase = eap_method + "-" + phase2_auth \
                + "_" + eap_user + "_" + eap_password \
                + "_" + certificat_name + "_" + str(use_certificate)

        self._logger.info("WifiAddNetwork : ssid = %s, passkey = %s, security = %s" % (ssid, pass_phrase, security))

        self._api.set_wificonfiguration(ssid, pass_phrase, security, static_ip,
                                        ip_address, netmask, gateway, dns1, dns2,
                                        proxy_config, proxy_address, proxy_port, proxy_bypass)
