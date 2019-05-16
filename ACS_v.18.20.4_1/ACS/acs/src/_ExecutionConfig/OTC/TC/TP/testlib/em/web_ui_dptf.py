#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import time
from tools import *
from testlib.em.apps import Chrome

class WebUI_DPTF(UIBase):

    def __init__(self, cfg = None):
        UIBase.__init__(self)

    def get_web_dptf(self):
        version = self.get_build_version()
        if version.startswith("8"):
            return WebUI_DPTF_OMR1("web_dptf_omr1", "/vendor/etc/dptf")
        elif version.startswith("6"):
            return WebUI_DPTF_M("web_dptf_m", "/etc/dptf")
        else:
            assert False, "Version not supported"

class WebUI_DPTF_Base(UIBase):

    def __init__(self, arti_conf, dptf_dir):
        UIBase.__init__(self)
        self.arti_conf = arti_conf
        self.dptf_dir = dptf_dir

    def push_web_dptf_files(self):
        cmd_dir = self.dptf_dir + "/cmd"
        ui_dir = self.dptf_dir + "/ui"
        bin_dir = self.dptf_dir + "/bin"
        start_name = "start"
        index_name = "index.html"
        combined_name = "combined.xsl"
        msg = self.testDevice.adb_cmd_capture_msg("'ls %s %s %s 2>/dev/null'" % (cmd_dir, ui_dir, bin_dir))
        if start_name in msg and index_name in msg and combined_name in msg:
            return
        web_dptf_zip = download_artifactory_content(self.arti_conf)
        file_dir = os.path.dirname(web_dptf_zip)
        cmd = "unzip -oq %s -d %s" % (web_dptf_zip, file_dir)
        shell_cmd(cmd)
        self.testDevice.adb_cmd("mkdir -p %s %s %s" % (cmd_dir, ui_dir, bin_dir))
        start_file = os.path.join(file_dir, start_name)
        self.testDevice.adb_cmd_common("push %s %s" % (start_file, cmd_dir))
        index_file = os.path.join(file_dir, index_name)
        self.testDevice.adb_cmd_common("push %s %s" % (index_file, ui_dir))
        combined_file = os.path.join(file_dir, combined_name)
        self.testDevice.adb_cmd_common("push %s %s" % (combined_file, bin_dir))

    def start_dptf_web_server(self):
        process = "esif_ufd"
        self.testDevice.adb_cmd("stop %s" % process)
        self.testDevice.adb_cmd("%s -s" % process)

    def open_web_dptf(self):
        chrome = Chrome()
        chrome.install()
        chrome.close_tabs()
        chrome.launch()
        chrome.open_url("0.0.0.0:8888")
        time.sleep(10)

    def open_policy_tab(self, policy_name):
        assert False, "Must be overwrite"

    @property
    def main_content(self):
        assert False, "Must be overwrite"

    def get_passive_policy2_cpu_power(self, freq_type = "Max"):
        self.open_policy_tab("Passive Policy 2")
        if freq_type == "Max":
            ind = 3
        elif freq_type == "Requested Value":
            ind = 4
        elif freq_type == "Min":
            ind = 5
        elif freq_type == "Granted Value":
            ind = 6
        else:
            assert False, "Invalid argument"
        power = self.main_content.child(index=2).child(index=2).child(index=ind).description
        return int(power)

    def get_psvt_step_size(self, trip_point):
        self.open_policy_tab("Passive Policy 2")
        if trip_point == 90:
            ind = 2
        elif trip_point == 105:
            ind = 3
        elif trip_point == 110:
            ind = 4
        else:
            assert False, "Invalid argument"
        size = self.main_content.child(index=5).child(index=ind).child(index=8).description
        return int(size)

    def check_psvt_solution(self):
        self.open_policy_tab("Adaptive Performance Policy")
        # check psvt_norm
        text_norm = self.main_content.child(index=0).child(index=2).child(index=0).description
        assert text_norm == "psvt_norm"
        comp = self.main_content.child(index=0).child(index=2).child(resourceId="comp").description
        assert comp == "<="
        desc = self.main_content.child(index=0).child(index=2).child(resourceId="arg").description
        assert desc == "59.9"
        # check psvt_high
        text_high = self.main_content.child(index=0).child(index=3).child(index=0).description
        assert text_high == "psvt_high"
        comp = self.main_content.child(index=0).child(index=3).child(resourceId="comp").description
        assert comp == ">="
        desc = self.main_content.child(index=0).child(index=3).child(resourceId="arg").description
        assert desc == "60.0"

    def check_critical_policy(self):
        self.open_policy_tab("Critical Policy")
        trip_dict = {}
        for i in [2, 3, 4]:
            name = self.main_content.child(index=2).child(index=i).child(index=1).description
            value = self.main_content.child(index=2).child(index=i).child(index=5).description
            trip_dict[name] = value
        assert trip_dict["TCPU"] == "109.0"
        assert trip_dict["TSKN"] == "105.0"
        assert trip_dict["TAMB"] == "89.0"

    def get_art_status(self, tcpu, tskn, tamb):
        self.open_policy_tab("Active Policy")
        ac_dict = {}
        art_dict = {}
        ac_dict["TCPU"] = 10 - int(tcpu / 10)
        ac_dict["TSKN"] = 10 - int(tskn / 10)
        ac_dict["TAMB"] = 10 - int(tamb / 10)
        for i in [2, 3, 4]:
            desc = self.main_content.child(index=4).child(index=i).child(index=1).description
            key = desc[-4:]
            idx = ac_dict[key] + 5
            art_dict[key] = self.main_content.child(index=4).child(index=i).child(index=idx).description
        print art_dict
        fan = 0
        for art in art_dict.values():
            if art != 'X':
                art_int = int(art)
                if art_int > fan:
                    fan = art_int
        return fan


class WebUI_DPTF_M(WebUI_DPTF_Base):

    def __init__(self, arti_conf, dptf_dir):
        WebUI_DPTF_Base.__init__(self, arti_conf, dptf_dir)

    def open_web_dptf(self):
        WebUI_DPTF_Base.open_web_dptf(self)
        self.d.click(800, 725)
        time.sleep(5)

    def open_policy_tab(self, policy_name):
        self.d(description=policy_name).click()

    @property
    def main_content(self):
        return self.d(resourceId="dvPageDisplay")


class WebUI_DPTF_OMR1(WebUI_DPTF_Base):

    def __init__(self, arti_conf, dptf_dir):
        WebUI_DPTF_Base.__init__(self, arti_conf, dptf_dir)

    def open_web_dptf(self):
        WebUI_DPTF_Base.open_web_dptf(self)
        self.d(description="Policies").click()
        ui_obj = self.d(description="Active Policy").sibling(index=0)
        if ui_obj.description == "":
            ui_obj.click()
        ui_obj = self.d(description="Adaptive Performance Policy").sibling(index=0)
        if ui_obj.description == "":
            ui_obj.click()
        ui_obj = self.d(description="Critical Policy").sibling(index=0)
        if ui_obj.description == "":
            ui_obj.click()
        ui_obj = self.d(description="Passive Policy 2").sibling(index=0)
        if ui_obj.description == "":
            ui_obj.click()
        self.d.click(800, 725)

    def open_policy_tab(self, policy_name):
        desc = self.d(descriptionStartsWith=policy_name).description
        if desc == policy_name:
            self.d(descriptionStartsWith=policy_name).click()

    @property
    def main_content(self):
        return self.d(resourceId="main-content-window-container").child(index=0)

