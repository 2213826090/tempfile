#!/usr/bin/env python

# #############################################################################
#
# @filename:
#
# @description: Android Flash steps
#
# @author:      ion-horia.petrisor@intel.com
#
##############################################################################

from testlib.base.base_step import step as base_step
from testlib.scripts.android.adb import adb_steps
from testlib.scripts.android.ui import ui_steps
from testlib.scripts.android.adb.adb_step import step as adb_step
from testlib.scripts.android.ui.ui_step import step as ui_step
from testlib.scripts.connections.local import local_steps
from testlib.scripts.connections.local.local_step import step as local_step
from testlib.scripts.connections.local import local_utils
from testlib.utils.relay import Relayed_device
from testlib.scripts.kb import kb_steps
from testlib.base.base_utils import TimeoutError
import time
import datetime
import os
import json

class pft_flash(adb_step):

    """ description:
            Performs device flash using PFT cli. The image flashed is
            described by flash.xml <flash_xml_path>. PFT is called in a
            screen described by <screen_name>.
            After flash, the step waits for adb and checks if the
            ro.build.version.incremental property is the same as
            <build_no>

        usage:
            flash_steps.pft_flash(flash_xml_path = path_to_flash_xml,
                                  build_no = new_image_build_no,
                                  screen_name = some_name_of screen)()

        tags:
            android, flash, PFT, flash.xml, screen
    """

    def __init__(self,
                 device,
                 flash_xml_path,
                 build_no,
                 user_build = True,
                 user_signed = False,
                 screen_name = "flashing",
                 usb_debugging = True,
                 platform = 'ECS-Trekstor',
                 version = 'L',
                 screen_log = None,
                 timeout = 900,
                 update = False,
                 **kwargs):
        adb_step.__init__(self, platform = platform, dessert = version, **kwargs)
        self.device = device
        self.flash_xml_path = flash_xml_path
        self.build_no = build_no
        self.user_build = user_build
        self.user_signed = user_signed
        self.screen_name = "{0}_{1}".format(screen_name, self.serial)
        self.usb_debugging = usb_debugging
        self.platform = platform
        self.version = version
        self.dessert = version
        self.timeout = timeout
        self.update = update
        self.set_passm("Image {0} flashed on {1} device.".format(build_no, self.serial))
        self.set_errorm("", "Image {0} flashed on {1} device.".format(build_no, self.serial))

    def do(self):

        screen_log = local_steps.create_screen(screen_name = self.screen_name,
                                               with_log = True)()
        grep_for = ["Phone Flash Tool exited with code ", "Platform Flash Tool exited with code "]
        pft_screen_command(serial = self.serial,
                           device = self.device,
                           screen_name = self.screen_name,
                           screen_log = screen_log,
                           flash_xml_path = self.flash_xml_path,
                           user_build = self.user_build,
                           user_signed = self.user_signed,
                           timeout = self.timeout,
                           grep_for = grep_for,
                           update = self.update)()

        if self.usb_debugging:
            iterations = 2
            for i in range(iterations):
                try:
                    local_steps.wait_for_adb(serial = self.serial,
                                         timeout = 300)()
                    break
                except Exception, e:
                    print "{0}/{1} interation({2}): adb is not enabled on "\
                          "{3} after 300 seconds.".format(i + 1, iterations,
                                                          e.message, self.serial)
                    try:
                        my_relay = Relayed_device(relay_port = self.device["relay"]["tty"],
                                      power_port = self.device["relay"]["power_port"],
                                      v_up_port = self.device["relay"]["v_up_port"],
                                      v_down_port = self.device["relay"]["v_down_port"])
                        my_relay.relay_reboot()
                    except Exception, e:
                        print "Serial {0} has no relay connection: {1}".format(self.serial, e.message)
                        raise e
            else:
                local_steps.wait_for_adb(serial = self.serial,
                                 timeout = 120)()

            if self.update and not self.user_signed:
                for i in range(iterations):
                    try:
                        adb_steps.wait_for_ui(serial = self.serial,
                                        timeout = 900)()
                        print "{0}: Perform factory reset".format(self.serial)
                        factory_reset(serial = self.serial)()
                        time.sleep(120)
                        local_steps.wait_for_adb(serial = self.serial,
                                                 timeout = 120)()
                        print "{0}: Factory reset - Done!".format(self.serial)
                        break
                    except Exception, e:
                        print e.message
                        try:
                            my_relay = Relayed_device(relay_port = self.device["relay"]["tty"],
                                          power_port = self.device["relay"]["power_port"],
                                          v_up_port = self.device["relay"]["v_up_port"],
                                          v_down_port = self.device["relay"]["v_down_port"])
                            my_relay.relay_reboot()
                        except Exception, e:
                            print e.message
                            raise e
                else:
                    adb_steps.wait_for_ui(serial = self.serial,
                                    timeout = 900)()
                    print "{0}: Perform factory reset".format(self.serial)
                    factory_reset(serial = self.serial)()
                    time.sleep(120)
                    local_steps.wait_for_adb(serial = self.serial,
                                             timeout = 120)()
                    print "{0}: Factory reset - Done!".format(self.serial)
            time.sleep(60)
        else:
            # wait for ui
            print "Waiting 15 minutes for the UI.. ", datetime.datetime.now()
            time.sleep(900)
            print "Done waiting for UI"
            print "Serial execution start"
            # serial code
            while True:
                try:
                    os.mkdir("/tmp/lock")
                    break
                except OSError: pass
            try:
                time.sleep(1)
                kb_steps.perform_startup_wizard(device = self.device,
                                                platform = self.platform,
                                                version = self.version)()
                kb_steps.enable_usb_debugging(device = self.device,
                                                platform = self.platform)()
            finally:
                os.rmdir("/tmp/lock")
            print "Serial execution end"

    def check_condition(self):
        build_no_flashed = self.adb_connection.get_prop(prop = "ro.build.version.incremental").strip()
        local_steps.remove_screen(screen_name = self.screen_name)()
        return self.build_no in build_no_flashed


class pft_screen_command(adb_step, local_step):
    """ description:
            Calls the PFT command to flash the image from <flash_xml_path>
            inside the <screen_name> screen.
            Unlock oem and Lock oem need relay connection and the logic
            is implemented here.
            The step checks if the command ends with "Phone Flash Tool
            exited with code 0" in the screen log.

        usage:
            flash_steps.pft_screen_command(flash_xml_path = path_to_flash_xml,
                                           screen_name = some_name_of screen,
                                           timeout = flash)()

        tags:
            android, flash, PFT, flash.xml, screen, oem_lock, oem_unlock, relay
    """
    def __init__(self,
                 device,
                 screen_name,
                 screen_log,
                 flash_xml_path,
                 user_build,
                 user_signed,
                 timeout = 450,
                 grep_for = ["Phone Flash Tool exited with code "],
                 update = False,
                 **kwargs):
        adb_step.__init__(self, **kwargs)
        local_step.__init__(self, **kwargs)
        self.device = device
        self.screen_name = screen_name
        self.screen_log = screen_log
        if "xml" in flash_xml_path:
            self.screen_command = "phoneflashtool --cli --os-sn {0} --flash-file {1}".format(self.serial, flash_xml_path)
        else:
            if self.device_info.has_multiple_flashfiles:
                flash_conf_id = self.device_info.flash_configuration_id
                if "$platform" in flash_conf_id:
                    flash_conf_id = flash_conf_id.replace("$platform", os.environ["PLATFORM"])
                self.screen_command = "phoneflashtool --cli --os-sn {0} --flash-file {1} --configuration {2}".format(self.serial,
                                                                                                                     flash_xml_path,
                                                                                                                     flash_conf_id,)
            else:
                self.screen_command = "phoneflashtool --cli --os-sn {0} --flash-file {1}".format(self.serial, flash_xml_path)
        self.user_build = user_build
        self.user_signed = user_signed
        self.timeout = timeout
        self.grep_for = grep_for
        self.update = update

    def do(self):
        cmd_header = "screen -x {0} -p0 -X stuff '".format(self.screen_name)
        enter = "\r'"
        local_steps.command(command = "{0}{1}{2}".format(cmd_header, self.screen_command, enter))()
        if not self.user_build or self.user_signed or self.update:
            return
        waiting = 0
        unlock_grep = "\"oem\" \"unlock\"` command"
        print "{0}: Waiting unlock command".format(self.serial)
        while waiting < self.timeout / 8:
            cat_grep_cmd = "tail -n 10 {0} | grep '{1}'".format(self.screen_log, unlock_grep)
            stdout, stderr = self.local_connection.run_cmd(command = cat_grep_cmd)
            if unlock_grep in stdout:
                time.sleep(30)
                my_relay = Relayed_device(relay_port = self.device["relay"]["tty"],
                                        power_port = self.device["relay"]["power_port"],
                                        v_up_port = self.device["relay"]["v_up_port"],
                                        v_down_port = self.device["relay"]["v_down_port"])
                my_relay.press_volume_up()
                my_relay.press_power()
                my_relay.close()
                break
            waiting += 2
            time.sleep(2)
        unlock_grep = "\"oem\" \"unlock\"` failed"
        while waiting < 30:
            cat_grep_cmd = "tail -n 10 {0} | grep '{1}'".format(self.screen_log, unlock_grep)
            stdout, stderr = self.local_connection.run_cmd(command = cat_grep_cmd)
            if unlock_grep in stdout:
                print "{0}: Unlock command failed".format(self.serial)
                print "{0}: The device is not OEM enabled!!!!!".format(self.serial)
                sys.exit(0)
            waiting += 2
            time.sleep(2)
        waiting = 0
        lock_grep = "\"oem\" \"lock\"` command"
        print "{0}: Waiting lock command".format(self.serial)
        while waiting < self.timeout * 6 / 8:
            cat_grep_cmd = "tail -n 10 {0} | grep '{1}'".format(self.screen_log, lock_grep)
            stdout, stderr = self.local_connection.run_cmd(command = cat_grep_cmd)
            if lock_grep in stdout:
                time.sleep(30)
                my_relay = Relayed_device(relay_port = self.device["relay"]["tty"],
                                        power_port = self.device["relay"]["power_port"],
                                        v_up_port = self.device["relay"]["v_up_port"],
                                        v_down_port = self.device["relay"]["v_down_port"])
                my_relay.press_volume_up()
                my_relay.press_power()
                my_relay.close()
                break
            waiting += 2
            time.sleep(2)

    def check_condition(self):

        waiting = 0
        while waiting < self.timeout:
            for item in self.grep_for:
                cat_grep_cmd = "tail -n 10 {0} | grep '{1}'".format(self.screen_log, item)
                stdout, stderr = self.local_connection.run_cmd(command = cat_grep_cmd)
                if item in stdout:
                    discovered_returned_code = stdout.rsplit()[-1]
                    if discovered_returned_code == '0':
                        self.set_passm("Flashed succesfully!")
                        return True
                    else:
                        self.set_errorm("", "Flashing failed with code {0}".format(discovered_returned_code))
                        return False
            waiting += 2
            time.sleep(2)
        raise TimeoutError(" Timeout reached while flashing device {0}".format(self.serial))


class fastboot_flash(adb_step):

    """ description:
            Performs device flash using flash_all.sh. The image flashed
            is described by <image_path>.
            After flash, the step waits for adb and checks if the
            ro.build.version.incremental property is the same as
            <build_no>

        usage:
            flash_steps.fastboot_flash(image_path = path_to_image,
                                       build_no = new_image_build_no)()

        tags:
            android, flash, flash_all, image, relay
    """
    def __init__(self, image_path, build_no, **kwargs):
        self.img_path = image_path
        self.build_no = build_no
        adb_step.__init__(self, **kwargs)

    def do(self):
        print "{0}: {1}".format(self.serial, self.img_path)
        ################################################################################
        # Device has to be in fastboot
        #   - if already in fastboot, do nothing
        #   - if it has adb, reboot in fastboot via adb
        #   - for any other state:
        #       * force poweroff via relay
        #       * boot in fastboot via relay (poweron & volume down)
        ################################################################################
        print "{0}: reboot to fastboot".format(self.serial)
        if local_utils.has_fastboot_serial(self.serial):
            pass
        elif local_utils.has_adb_serial(self.serial):
            adb_steps.reboot(serial = self.serial,
                             command = "fastboot")()
        else:
            ############################################################################
            # device file should be made dynamic to support multiple relays
            ############################################################################
            print "{0}: Relay needed!!!!!!!!".format(self.serial)
            my_relay = Relay(port = self.device["relay"]["tty"])
            my_relay.power_on(self.device["relay"]["v_up_port"])
            my_relay.power_on(self.device["relay"]["power_port"])
            my_relay.close()
        print "{0}: Done rebooting for fastboot!".format(self.serial)

        print "{0}: Wait for fastboot".format(self.serial)
        local_steps.wait_for_fastboot(serial = self.serial,
                                      timeout = 100)()
        print "{0}: Done waiting for fastboot!".format(self.serial)
        old_folder, err = local_steps.command("pwd")()

        local_steps.change_dir(new_folder = self.img_path)()
        print "{0}: Wait for flash-all.sh".format(self.serial)
        #TODO: add stdout_gprep for flash-all.sh
        sout, serr = local_steps.command(command = "./flash-all.sh -s {0}".format(self.serial))()
        print "{0}: Done! Image flashed".format(self.serial)
        print "{0}: Wait for adb".format(self.serial)

        local_steps.wait_for_adb(serial = self.serial,
                                 timeout = 720)()
        print "{0}: Done! adb connected".format(self.serial)
        local_steps.change_dir(new_folder = old_folder.strip())()

    def check_condition(self):
        version = self.adb_connection.get_prop(prop = "ro.build.version.incremental")
        print "{0} == {1}".format(version, self.build_no)
        return self.build_no in version.strip()


class fastboot_flash_to_ui(base_step):

    """ description:
            Performs device flash using flash_all.sh. The image flashed
            is described by <image_path>. It wats for the DUT to get to
            UI.

        usage:
            flash_steps.fastboot_flash_to_ui(image_path = path_to_image,
                                             build_no = new_image_build_no,
                                             timeout = flash_timeout)()

        tags:
            android, flash, flash_all, image, ui
    """
    def __init__(self, serial, image_path, build_no, timeout, **kwargs):
        self.serial = serial
        self.image_path = image_path
        self.build_no = build_no
        self.timeout = timeout
        base_step.__init__(self, **kwargs)

    def do(self):

        fastboot_flash(serial = self.serial,
                       image_path = self.image_path,
                       build_no = self.build_no)()
        adb_steps.wait_for_ui(serial = self.serial,
                              timeout = self.timeout)()

    def check_condition(self):
        #check is done by the last step
        return True


class prepare_image_for_flash(local_step):

    """ description:
            Prepares a new image for flashing. Gets the build_no from
            build.prop and unzips the archive

        usage:
            flash_steps.prepare_image_for_flash(path_to_image = path_to_image,
                                                platform = 'ECS_E7')()

        tags:
            android, flash, flash_all, image, ui
    """

    def __init__(self,
                 path_to_image,
                 platform = "ECS27B",
                 prop_file = 'build.prop',
                 device_prop = "ro.build.version.incremental",
                 user_signed = False,
                 cts_abi = None,
                 resources_subfolder = None,
                 flash_xml_path = None,
                 edit_flash_file = True,
                 repackage_userdata = False,
                 fls_tool_location = None,
                 fake_userdata_size = "4569694208",
                 **kwargs):
        self.path_to_image = path_to_image
        self.platform = platform
        self.prop_file = prop_file
        self.device_prop = device_prop
        self.user_signed = user_signed
        self.cts_abi = cts_abi
        self.flash_xml_path = flash_xml_path
        self.edit_flash_file = edit_flash_file
        self.repackage_userdata = repackage_userdata
        self.fls_tool_location = fls_tool_location
        self.fake_userdata_size = fake_userdata_size
        if "RESOURCES_FOLDER" in os.environ:
            self.resources_folder = os.environ["RESOURCES_FOLDER"]
        else:
            self.resources_folder = os.path.join(os.getenv("HOME"), "acas/resources/")
        if resources_subfolder:
            self.resources_folder = os.path.join(self.resources_folder, resources_subfolder)
        self.repackage_build_location = 'build'
        self.repackage_origdata_location = 'origdata'
        self.repackage_fakedata_location = 'fakedata'
        self.new_userdata_name = 'my_new_userdata.fls'
        local_step.__init__(self, **kwargs)

    def do(self):

        command = "cat {0}".format(os.path.join(self.path_to_image, self.prop_file))
        print command
        self.step_data = self.local_connection.parse_cmd_output(cmd = command,
                                                                grep_for = self.device_prop).split("=")[1]
        self.set_passm("Preparing image for build {0}".format(self.step_data))
        self.set_errorm("", "Preparing image for build {0}".format(self.step_data))

        # delete the unzip path folder if exists
        if self.cts_abi:
            self.unzip_path = os.path.join(self.path_to_image, "{0}{1}".format(self.platform, self.cts_abi))
        else:
            self.unzip_path = os.path.join(self.path_to_image, self.platform)
        if local_utils.folder_exists(path = self.unzip_path):
            local_steps.delete_folder(folder = self.unzip_path)()

        # get the archive name
        ls_out = self.local_connection.parse_cmd_output(cmd = "ls {0}".format(self.path_to_image),
                                                        grep_for = self.platform)
        if "zip" not in ls_out:
            ls_out = self.local_connection.parse_cmd_output(cmd = "ls {0}".format(self.path_to_image),
                                                        grep_for = self.step_data)
        archive_name = None
        for entry in ls_out.split():
            if (self.user_signed and "signed.zip" in entry) or \
               (not self.user_signed and "automation.zip" in entry) or \
               (not self.user_signed and "signed.zip" not in entry):
                archive_name = entry
                break
        if archive_name is None:
            archive_name = [entry for entry in ls_out.split() if ".zip" in entry][0]
        print archive_name
        # create the unzip path folder
        if self.cts_abi:
            local_steps.create_folder(folder = "{0}{1}".format(self.platform, self.cts_abi),
                                      path = self.path_to_image)()
        else:
            local_steps.create_folder(folder = self.platform,
                                  path = self.path_to_image)()

        # unzip the archive
        local_steps.unzip_archive(zip_path = os.path.join(self.path_to_image, archive_name),
                                  unzip_destination = self.unzip_path)()

        #repackage userdata if needed
        if self.repackage_userdata:
            local_steps.copy_file(file = os.path.join(self.path_to_image, archive_name),
                                  destination = self.fls_tool_location,
                                  with_rename = False)()

            if local_utils.folder_exists(path = os.path.join(self.fls_tool_location, self.repackage_build_location)):
                local_steps.delete_folder(folder = os.path.join(self.fls_tool_location, self.repackage_build_location))()
            if local_utils.folder_exists(path = os.path.join(self.fls_tool_location, self.repackage_origdata_location)):
                local_steps.delete_folder(folder = os.path.join(self.fls_tool_location, self.repackage_origdata_location))()

            local_steps.create_folder(folder = self.repackage_build_location,
                                      path = self.fls_tool_location)()
            local_steps.create_folder(folder = self.repackage_origdata_location,
                                      path = self.fls_tool_location)()

            local_steps.unzip_archive(zip_path = os.path.join(self.fls_tool_location, archive_name),
                                  unzip_destination = os.path.join(self.fls_tool_location, self.repackage_build_location))()

            userdata_name = self.local_connection.parse_cmd_output(cmd = "ls {0}".format(os.path.join(self.fls_tool_location, self.repackage_build_location)),
                                                                    grep_for = "userdata")
            if ".fls" not in userdata_name:
                self.set_errorm("", "There is no userdata in the image".format(self.step_data))
            else:
                command = "cd {0}; make_ext4fs -s -C fsconfig.txt -l {1} -a data fakedata.img {2}".format(self.fls_tool_location, self.fake_userdata_size, self.repackage_fakedata_location)
                local_steps.command(command = command,
                                    live_print = False,
                                    mode = "sync")()
                command = "cd {0};./FlsTool -x {1}/{2} -o {3}".format(self.fls_tool_location, self.repackage_build_location, userdata_name, self.repackage_origdata_location)
                local_steps.command(command = command,
                                    live_print = False,
                                    mode = "sync")()
                command = "cd {0};./FlsTool --extract-prg -r {1}/{2} -o {3}/prg.bin".format(self.fls_tool_location, self.repackage_build_location, userdata_name, self.repackage_origdata_location)
                local_steps.command(command = command,
                                    live_print = False,
                                    mode = "sync")()
                command = "cd {0};./FlsTool --to-fls2 --prg {2}/prg.bin --psi {2}/userdata.fls_inj_PSI.bin --ebl {2}/userdata.fls_inj_EBL.bin \
                          --tag USERDATA -r -o {1} fakedata.img".format(self.fls_tool_location, self.new_userdata_name, self.repackage_origdata_location)
                local_steps.command(command = command,
                                    live_print = False,
                                    mode = "sync")()
                local_steps.copy_file(file = os.path.join(self.fls_tool_location, self.new_userdata_name),
                                      destination = os.path.join(self.resources_folder, userdata_name),
                                      with_rename = True)()

        # copy resources folder
        local_steps.copy_folder_content(folder = self.resources_folder,
                                        destination = self.unzip_path)()

        # edit flash.json to flash userdata and/or remove oem/flashing lock
        print "Edit {0} file".format(self.flash_xml_path)
        if ('json' in self.flash_xml_path) and (self.edit_flash_file):
            js_data = None
            with open(self.flash_xml_path, "r") as flash_file:
                js_data = json.load(flash_file)
                # do not lock bootloader. Write userdata instead
                for cmd in js_data['flash']['commands']:
                    if 'blank' in cmd['restrict'][0] and cmd['tool'] == "fastboot":
                        if cmd['args'] == "oem lock" or cmd['args'] == "flashing lock" :
                            cmd[u'description'] = u'Flash userdata partition'
                            cmd[u'args'] = u'flash userdata ${userdata}'
                            cmd[u'timeout'] = 180000
                for cmd in js_data['flash']['commands']:
                    if 'blank' in cmd['restrict'][0] and cmd['tool'] == "fastboot":
                        if "flash oemvars" in cmd['args']:
                            js_data['flash']['commands'].remove(cmd)
                            break

                js_data['flash']['parameters']['userdata'] = {u'type': u'file',
                        u'description': u'userdata.img', u'value': u'userdata.img', u'name': u'userdata'}
            if js_data is not None:
                with open(self.flash_xml_path, "w") as flash_file:
                    json.dump(js_data, flash_file, sort_keys=True, indent=4)

    def check_condition(self):
        ls_out = self.local_connection.parse_cmd_output(cmd = "ls {0}".format(self.unzip_path),
                                                        grep_for = "flash")
        return "flash.json" in ls_out or "flash.xml" in ls_out


class factory_reset(ui_step):

    """ description:
            Performs factory reset

        usage:
            flash_steps.factory_reset()()

        tags:
            android, factory reset
    """

    def __init__(self,
                 reset_button_text = "Reset tablet",
                 reboot_timeout = 120,
                 **kwargs):
        self.reset_button_text = reset_button_text
        self.reboot_timeout = reboot_timeout
        ui_step.__init__(self, **kwargs)

    def do(self):
        adb_steps.am_start_command(serial = self.serial,
                                   component = "com.android.settings/.Settings")()
        ui_steps.click_button_with_scroll(serial = self.serial,
                                          view_to_find = {"textContains": "Backup"},
                                          view_to_check = {"text": "Factory data reset"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "Factory data reset"},
                              view_to_check = {"text": self.reset_button_text})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": self.reset_button_text},
                              view_to_check = {"text": "Erase everything"})()
        ui_steps.click_button(serial = self.serial,
                              view_to_find = {"text": "Erase everything"})()
        adb_steps.check_device_reboots(serial = self.serial,
                                       reboot_timeout = self.reboot_timeout)()


    def check_condition(self):
        # check performed in do()
        return True
