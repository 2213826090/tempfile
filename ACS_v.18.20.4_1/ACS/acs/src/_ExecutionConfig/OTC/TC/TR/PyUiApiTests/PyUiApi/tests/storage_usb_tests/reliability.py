from PyUiApi.common.storage_usb_utils import *
from PyUiApi.common.system_utils import *
from PyUiApi.adb_helper.logcat_messaging_gateway import *


class StorageUSBTestsReliability(unittest.TestCase):

    def setUp(self):
        LOG.info("Test start")
        self.do_clean_up = True

    def tearDown(self):
        if self.do_clean_up:
            self.cleanUp()
        LOG.info("Test finished")

    def cleanUp(self, *dirs_to_delete):
        for folder in dirs_to_delete:
            AdbUtils.delete_files(folder)

    def test_adb_copy_3level_folder_10000_times(self):
        A_path = os.path.join(Environment.emulated_storage_path, "A")
        B_path = os.path.join(Environment.emulated_storage_path, "B")
        C_path = os.path.join(Environment.emulated_storage_path, "C")
        try:
            A_3_level_path = os.path.join(A_path, "level1/level2/level3")
            B_3_level_path = os.path.join(A_path, "level1/level2/level3")
            C_3_level_path = os.path.join(A_path, "level1/level2/level3")
            file_name_1 = "file1.file"
            file_name_2 = "file2.file"
            file_content = "this is some epic file content"
            AdbUtils.create_dir(A_3_level_path)
            AdbUtils.create_file(file_name_1, A_3_level_path + "/", file_content)
            AdbUtils.create_file(file_name_2, A_3_level_path + "/", file_content)
            AdbUtils.copy(A_path, B_path)
            ''' this runs very fast
            multi_copy_script = Template("cd $testdir ; for i in `seq 1 $iterations`; do cp -r ./B/* ./C/ ; "
                                         "cp -r  ./C/* ./B/ ; done")
            multi_copy_cmd = multi_copy_script.substitute(iterations=10000,
                                                          testdir=Environment.emulated_storage_path)
            AdbUtils.run_adb_cmd(multi_copy_cmd)
            '''
            for i in range(10000):
                if i % 500 == 0:
                    LOG.info("got to iteration nr: " + str(i))
                AdbUtils.run_adb_cmd("cp -r " + B_path + "/* " + C_path, verbose=False)
                AdbUtils.run_adb_cmd("cp -r " + C_path + "/* " + B_path, verbose=False)

            B_file1_content = AdbUtils.cat_file(os.path.join(B_3_level_path, file_name_1))
            LOG.info("B file 1 content: " + str(B_file1_content))
            B_file2_content = AdbUtils.cat_file(os.path.join(B_3_level_path, file_name_2))
            LOG.info("B file 2 content: " + str(B_file2_content))
            C_file1_content = AdbUtils.cat_file(os.path.join(C_3_level_path, file_name_1))
            LOG.info("C file 1 content: " + str(C_file1_content))
            C_file2_content = AdbUtils.cat_file(os.path.join(C_3_level_path, file_name_2))
            LOG.info("C file 2 content: " + str(C_file2_content))
            self.assertTrue(file_content in B_file1_content and file_content in B_file2_content and
                            file_content in C_file1_content and file_content in C_file2_content,
                            "content of the files changed during sequential copying")
        finally:
            AdbUtils.delete_files(A_path, B_path, C_path)

    def test_full_memory_3_times(self):
        self.assertFalse(StorageUsbUtils.is_external_storage_adopted(),
                         "SD Card must not be adopted")
        free_space = StorageUsbUtils.get_free_space_in_MB(Environment.storage_emulated_root)
        fill_storage_script_template = Template("cd $testdir ; for i in `seq 1 $nroffiles`; do dd if=/dev/zero "
                                                "of=testfile$$i bs=1m count=1 ; done")
        test_dir_name = "full_memory_test"
        test_dir_path = os.path.join(Environment.emulated_storage_path, test_dir_name)
        try:
            nr_of_files = int(free_space) - 2  # just in case
            fill_storage_cmd = fill_storage_script_template.substitute(testdir=test_dir_path,
                                                                       nroffiles=nr_of_files)
            for i in range(3):
                AdbUtils.create_dir(test_dir_path)
                AdbUtils.run_adb_cmd(fill_storage_cmd)
                final_free_space = StorageUsbUtils.get_free_space_in_MB(Environment.storage_emulated_root)
                self.assertTrue(final_free_space < free_space, "something went wrong when filling up memory")
                before_keyevent = datetime.datetime.now()
                AdbUtils.send_key_event("KEYCODE_MENU")
                after_keyevent = datetime.datetime.now()
                time_delta = after_keyevent - before_keyevent
                self.assertTrue(time_delta.total_seconds() < 10, "device is no longer responsive after memory fill")
                AdbUtils.delete_files(test_dir_path)
        finally:
            AdbUtils.delete_files(test_dir_path)

    def test_adb_copy_10000_times_emulated(self):
        A_path = os.path.join(Environment.emulated_storage_path, "A.file")
        B_path = os.path.join(Environment.emulated_storage_path, "B.file")
        C_path = os.path.join(Environment.emulated_storage_path, "C.file")
        try:
            StorageUsbUtils.create_file_on_dut(A_path, 1024)  # 1MB file
            AdbUtils.copy_file(A_path, B_path)
            AdbUtils.verbose = False
            for i in range(10000):
                AdbUtils.copy_file(B_path, C_path)
                AdbUtils.copy_file(C_path, B_path)
            AdbUtils.verbose = True
            b_c_compare_cmd = "comm -23 " + B_path + " " + C_path
            b_c_compare = AdbUtils.run_adb_cmd(b_c_compare_cmd)
            LOG.info("comparison between B and C yielded: " + b_c_compare)
            a_b_compare_cmd = "comm -23 " + B_path + " " + A_path
            a_b_compare = AdbUtils.run_adb_cmd(a_b_compare_cmd)
            LOG.info("comparison between B and A yielded: " + a_b_compare)
            self.assertTrue(len(b_c_compare) == 0, "files B and C are not equal")
            self.assertTrue(len(a_b_compare) == 0, "files B and A are not equal")
        finally:
            AdbUtils.delete_files(A_path, B_path, C_path)
