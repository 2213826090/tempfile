from PyUiApi.common.storage_usb_utils import *
from PyUiApi.common.system_utils import *
from PyUiApi.app_utils.settings_utils import Settings


class StorageUSBTests(unittest.TestCase):
    TAG = "StorageUSBTests"
    instrumentation_pass_msg = InstrumentationInterface.instrumentation_one_test_pass_output

    def setUp(self):
        UiAutomatorUtils.unlock_screen()

    def tearDown(self):
        UiAutomatorUtils.close_all_tasks()

    valid_timing_specs = ["HS200", "HS400"]

    def test_timing_spec_parameter(self):
        output = AdbUtils.run_adb_cmd("cat /d/mmc0/ios")
        print output
        lines = output.split("\n")

        def validate_line(line):
            if "timing spec:" not in line:
                return False
            is_valid_line = False
            for valid_spec in StorageUSBTests.valid_timing_specs:
                if valid_spec in line:
                    LOG.info("found valid timing spec % s in line %s" % (valid_spec, line))
                    is_valid_line = True
            self.assertTrue(is_valid_line, "line %s did not contain any valid timing spec from %s"
                            % (line, str(StorageUSBTests.valid_timing_specs)))
            return True

        for line in lines:
            if validate_line(line):
                return
        print "timing spec not found"
        self.assertTrue(False, "timing spec line was not found in cat /d/mmc0/ios")

    def test_adb_bugreport(self):
        output = AdbUtils.run_adb_cmd("bugreport", printout=False)
        print len(output)
        self.assertTrue(len(output) > 3000000)  # magic number empirically determined
        bugreport_sections = ["MEMORY INFO", "CPU INFO", "ZONEINFO", "SYSTEM LOG"]
        for section in bugreport_sections:
            self.assertTrue(section in output)

    def test_memory(self):
        totalRAMkb = AdbUtils.run_adb_cmd("dumpsys meminfo").split("Total RAM:")[1].split(" ")[1]
        LOG.info("total RAM: " + totalRAMkb)
        totalRAMkb_value = totalRAMkb.replace(",","").replace("k","").replace("K","")\
            .replace("b","").replace("B","")
        totalRAM = int(int(totalRAMkb_value) / 1024)
        LOG.info("RAM in MB: " + str(totalRAM))
        api_tests_command = "am instrument -e class com.intel.test.apitests.tests." \
                            "DisplayMetricsTestDriver#testLowRamDevice -e totalRAM " + \
                            str(totalRAM) + \
                            " -w com.intel.test.apitests/com.intel.test.apitests.runners." \
                            "DisplayMetricsTestRunner"
        api_tests_output = AdbUtils.run_adb_cmd(api_tests_command)
        if "Failure" in api_tests_output:
            self.assertFalse("Failure in instrumentation", api_tests_output)
            return
        print "instrumentation returned OK"
        self.assertTrue(True, api_tests_output)

    def test_copy_emulated_from_physical(self):
        if StorageUsbUtils.check_sdcard_exists() and StorageUsbUtils.check_sdcard_mounted():
            file_name = "test.txt"
            file_path = Environment.sd_card_path + file_name
            LOG.info("File path in physical storage: " + file_path)
            if StorageUsbUtils.create_file_on_dut(file_path, 100):
                file_in_emulated_path = Environment.emulated_storage_path + file_name
                AdbUtils.copy_file(file_path, file_in_emulated_path)
                if StorageUsbUtils.test_file_exists_on_dut(file_in_emulated_path):
                    LOG.info("File exists in emulated storage")
                else:
                    LOG.info("Failure, file was not copied to emulated storage")
                    self.assertFalse("Failure in copying file from SD card to emulated storage",
                                     Environment.emulated_storage_path)
                    return
            else:
                LOG.info("Failure, file was not created on SD card")
                self.assertFalse("Failure in creating file on SD card",
                                 Environment.sd_card_path)
                return
        else:
            LOG.info("Failure, SD card is not inserted or mounted")
            self.assertFalse("Failure SD card is not inserted or mounted")
            return

        AdbUtils.delete_files(file_path, file_in_emulated_path)
        if (StorageUsbUtils.test_file_exists_on_dut(file_path) is False and
           StorageUsbUtils.test_file_exists_on_dut(file_in_emulated_path) is False):
            LOG.info("Success")
            return
        else:
            LOG.info("One of the files has not been deleted")
            self.assertFalse("Failure in deleting one of the files")
            return

    def test_copy_to_sdcard_from_emulated(self):
        if StorageUsbUtils.check_sdcard_exists() and StorageUsbUtils.check_sdcard_mounted():
            file_name = "test.txt"
            file_path = Environment.emulated_storage_path + file_name
            LOG.info("File path in emulated storage: " + file_path)
            if StorageUsbUtils.create_file_on_dut(file_path, 100):
                file_in_sdcard_path = Environment.sd_card_path + file_name
                AdbUtils.copy_file(file_path, file_in_sdcard_path)
                if StorageUsbUtils.test_file_exists_on_dut(file_in_sdcard_path):
                    LOG.info("File exists on SD card")
                else:
                    LOG.info("Failure, file was not copied to SD card")
                    self.assertFalse("Failure in copying file from emulated storage to SD card",
                                     Environment.sd_card_path)
                    return
            else:
                LOG.info("Failure, file was not created on emulated storage")
                self.assertFalse("Failure in creating file on emulated storage",
                                 Environment.emulated_storage_path)
                return
        else:
            LOG.info("Failure, SD card is not inserted or mounted")
            self.assertFalse("Failure SD card is not inserted or mounted")
            return

        AdbUtils.delete_files(file_path, file_in_sdcard_path)
        if (StorageUsbUtils.test_file_exists_on_dut(file_path) is False and
           StorageUsbUtils.test_file_exists_on_dut(file_in_sdcard_path) is False):
            LOG.info("Success")
            return
        else:
            LOG.info("One of the files has not been deleted")
            self.assertFalse("Failure in deleting one of the files")
            return

    def test_format_as_portable(self):
        # Check Sdcard is plugged in device
        self.assertTrue(StorageUsbUtils.check_sdcard_exists(),
                        "Test requires SD Card to be inserted in device")
        # partition disk as public
        self.assertTrue(StorageUsbUtils.forget_all_and_make_public(),
                        "Fail to partition disk as public")
        # check sdcard is mounted
        if StorageUsbUtils.check_sdcard_mounted():
                StorageUsbUtils.open_portable_sdcard_settings_menu()
                self.assertTrue(UiAutomatorUtils.click_view_with_text(STORAGE_USB_FORMAT_TXT, 25000),
                                "Fail to press Format")

                view_found = d(text=STORAGE_USB_ERASE_AND_FORMAT_TXT).wait.exists(timeout=3000)
                if view_found:
                    self.assertTrue(d(text=STORAGE_USB_ERASE_AND_FORMAT_TXT).click.topleft())
                view_found = d(text=STORAGE_USB_DONE_TXT).wait.exists(timeout=300000)
                if view_found:
                    self.assertTrue(d(text=STORAGE_USB_DONE_TXT).click.topleft())
                self.assertTrue(UiAutomatorUtils.close_all_tasks(),
                                "Fail to close recent applications")
                self.assertTrue(StorageUsbUtils.check_sdcard_mounted(),
                                "SDcard is not mounted as portable after format")
        else:
                self.assertFalse("SDcard is not mounted as portable")

    def test_format_as_adoptable_without_migrate(self):
        # Check Sdcard is plugged in device
        self.assertTrue(StorageUsbUtils.check_sdcard_exists(),
                        "Test requires SD Card to be inserted in device")
        # partition disk as public
        self.assertTrue(StorageUsbUtils.forget_all_and_make_public(),
                        "Fail to partition disk as public")
        # check sdcard is mounted
        if StorageUsbUtils.check_sdcard_mounted():
                StorageUsbUtils.open_portable_sdcard_settings_menu()
                self.assertTrue(UiAutomatorUtils
                                .click_view_with_text(STORAGE_USB_FORMAT_AS_INTERNAL_TXT, 25000),
                                "Fail to press Format as internal")
                self.assertTrue(UiAutomatorUtils.click_view_with_text(STORAGE_USB_ERASE_AND_FORMAT_TXT),
                                "Fail to press button ERASE & FORMAT")
                # wait for 300 seconds for the erase and format procedure to complete
                move_later_pressed = False
                for i in range(100):
                    if d(textContains=STORAGE_USB_MOVE_LATER_TXT).wait.exists(timeout=1000):
                        if UiAutomatorUtils.click_view_with_text(STORAGE_USB_MOVE_LATER_TXT, 1000):
                            move_later_pressed = True
                    if d(textContains=STORAGE_USB_OK_TXT).wait.exists(timeout=3000):
                        UiAutomatorUtils.click_view_with_text(STORAGE_USB_OK_TXT)
                    if move_later_pressed:
                        break

                self.assertTrue(move_later_pressed, "Fail to press radio button Move later")
                self.assertTrue(UiAutomatorUtils.click_view_with_text(STORAGE_USB_NEXT_TXT),
                                "Fail to press button Next")
                self.assertTrue(UiAutomatorUtils.click_view_with_text(STORAGE_USB_DONE_TXT),
                                "Fail to press button DONE")
                self.assertTrue(UiAutomatorUtils.close_all_tasks(),
                                "Fail to close recent applications")
                self.assertTrue(StorageUsbUtils.is_external_storage_adopted(),
                                "SDcard is not mounted as adopted after format")
        else:
                self.assertFalse("SDcard is not mounted as portable")

    def test_format_as_adoptable_migrate(self):
        # Check Sdcard is plugged in device
        self.assertTrue(StorageUsbUtils.check_sdcard_exists(),
                        "Test requires SD Card to be inserted in device")
        # partition disk as public
        self.assertTrue(StorageUsbUtils.forget_all_and_make_public(),
                        "Fail to partition disk as public")
        # check sdcard is mounted
        if StorageUsbUtils.check_sdcard_mounted():
                StorageUsbUtils.open_portable_sdcard_settings_menu()
                self.assertTrue(UiAutomatorUtils
                                .click_view_with_text(STORAGE_USB_FORMAT_AS_INTERNAL_TXT, 25000),
                                "Fail to press Format as internal")
                self.assertTrue(UiAutomatorUtils.click_view_with_text(STORAGE_USB_ERASE_AND_FORMAT_TXT),
                                "Fail to press button ERASE $ FORMAT")

                # wait for 300 seconds for the erase and format procedure to complete
                move_later_pressed = False
                for i in range(100):
                    if d(textContains=STORAGE_USB_MOVE_NOW_TXT).wait.exists(timeout=1000):
                        if UiAutomatorUtils.click_view_with_text(STORAGE_USB_MOVE_NOW_TXT, 1000):
                            move_later_pressed = True
                    if d(textContains=STORAGE_USB_OK_TXT).wait.exists(timeout=3000):
                        UiAutomatorUtils.click_view_with_text(STORAGE_USB_OK_TXT)
                    if move_later_pressed:
                        break

                self.assertTrue(move_later_pressed, "Fail to press radio button Move later")
                self.assertTrue(UiAutomatorUtils.click_view_with_text(STORAGE_USB_NEXT_TXT),
                                "Fail to press button Next")
                self.assertTrue(UiAutomatorUtils.click_view_with_text(STORAGE_USB_MOVE_TXT),
                                "Fail to press button Move")
                self.assertTrue(UiAutomatorUtils.click_view_with_text(STORAGE_USB_DONE_TXT, 300000),
                                "Fail to press button DONE")
                self.assertTrue(UiAutomatorUtils.close_all_tasks(),
                                "Fail to close recent applications")
                self.assertTrue(StorageUsbUtils.is_external_storage_adopted(),
                                "SDcard is not mounted as adopted after format")
        else:
                self.assertFalse("SDcard is not mounted as portable")

    def test_check_mountpoint_internal_storage(self):
        # verify if the internal mountpoint is mounted at /data with fstype ext4
        self.assertTrue(StorageUsbUtils
                        .search_mountpoint_and_fstype(Environment.internal_mountpoint,
                                                      Environment.internal_fstype),
                        "Fail! Internal Mountpoint is not the right one or the fstype is wrong!")

    def test_check_mountpoint_emulated_external(self):
        # verify if the emulated external storage is mounted with the fstype fuse
        self.assertTrue(StorageUsbUtils
                        .search_mountpoint_and_fstype(EnvironmentUtils.get_emulated_mountpoint(),
                                                      Environment.fuse_fstype),
                        "Fail! Emulated External Mountpoint is not the right one or the fstype is wrong!")
        self.assertTrue(StorageUsbUtils
                        .verify_symlink("/", "sdcard", EnvironmentUtils.get_sdcard_folder_symlink()),
                        "Fail! /sdcard is not the right symlink for emulated external storage!")

    def test_check_mountpoint_sdcard_portable(self):
        sdcard = EnvironmentUtils.get_sd_card_path()
        self.assertTrue(StorageUsbUtils
                        .search_mountpoint_and_fstype(sdcard[:-1], Environment.fuse_fstype),
                        "Fail! Storage Portable SDcard Mountpoint is not the right one or the fstype is wrong!")
        media_rw = EnvironmentUtils.get_portable_sdcard_mountpoint()
        self.assertTrue(StorageUsbUtils
                        .search_mountpoint_and_fstype(media_rw, Environment.fat_fstype),
                        "Fail! Media_rw Portable SDcard Mountpoint is not the right one or the fstype is wrong!")

    def test_benchmark(self):
        # Check Sdcard is plugged in device
        self.assertTrue(StorageUsbUtils.check_sdcard_exists(),
                        "Test requires SD Card to be inserted in device")

        result = SystemApiTestsInterface\
            .run_instrumentation(class_name="storage.StorageManagerTestsDriver",
                                 method_name="testBenchmark",
                                 instrumentation_args=None,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)

        if(StorageUSBTests.instrumentation_pass_msg in result):
            LOG.info("Benchmark passed")
            benchmark_result = True
        else:
            LOG.info("Benchmark failed")
            benchmark_result = False
        self.assertTrue(StorageUsbUtils.open_portable_sdcard_settings_menu(),
                        "could not open portable sdcard settings menu")
        self.assertTrue(UiAutomatorUtils
                        .click_view_with_text(STORAGE_USB_FORMAT_AS_INTERNAL_TXT, 25000),
                        "Fail to press Format as internal")
        self.assertTrue(UiAutomatorUtils.click_view_with_text(STORAGE_USB_ERASE_AND_FORMAT_TXT),
                        "Fail to press button ERASE $ FORMAT")
        if benchmark_result:
            self.assertTrue(UiAutomatorUtils.click_view_with_text(STORAGE_USB_MOVE_LATER_TXT, 300000),
                            "Fail to press radio button Move later")
        else:
            self.assertTrue(UiAutomatorUtils.click_view_with_text(STORAGE_USB_OK_TXT, 300000),
                            "Benchmark response was negative but the notification didn't appear or couldn't be pressed")
            self.assertTrue(UiAutomatorUtils.click_view_with_text(STORAGE_USB_MOVE_LATER_TXT, 15000),
                            "Fail to press radio button Move later")
        self.assertTrue(UiAutomatorUtils.click_view_with_text(STORAGE_USB_NEXT_TXT),
                        "Fail to press button Next")
        self.assertTrue(UiAutomatorUtils.click_view_with_text(STORAGE_USB_DONE_TXT),
                        "Fail to press button DONE")
        self.assertTrue(UiAutomatorUtils.close_all_tasks(),
                        "Fail to close recent applications")
        self.assertTrue(StorageUsbUtils.is_external_storage_adopted(),
                        "SDcard is not mounted as adopted after format")

    def test_adopted_mountpoint(self):
        Settings.open_storage_usb_options()
        sdcard_view = d(textContains="SD card")
        self.assertTrue(sdcard_view.wait.exists(timeout=3000), "sdcard not visible in Storage & USB window")
        sdcard_status_view = sdcard_view.down(resourceId="android:id/summary")
        self.assertTrue("Ejected" not in Info.get_text(sdcard_status_view))
        mount_info = AdbUtils.run_adb_cmd("cat /proc/self/mountstats | grep mnt/expand", add_ticks=False)
        self.assertTrue("fstype ext4" in mount_info, "something wrong with adopted sdcard in mountstats: " +
                        str(mount_info))

    def test_internal_storage_capacity(self):
        df_status = AdbUtils.run_adb_cmd("df | grep /data", add_ticks=False)
        partition_size = df_status.split()[1]
        LOG.info("total space for /data: " + partition_size)
        value = float(re.findall('([\.\d]+)', partition_size)[0])
        if "G" in partition_size:
            value *= 1024 * 1024
        elif "M" in partition_size:
            value *= 1024
        self.assertTrue(value > 3 * 1024 * 1024,
                        "/data partition must be at least 3GB in total size")


if __name__ == "__main__":
    test_result = SingleMethodRunner.run_single_test(StorageUSBTests, "test_adb_bugreport")
    print test_result.wasSuccessful()
