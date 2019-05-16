from PyUiApi.common.storage_usb_utils import *
from PyUiApi.common.system_utils import *
from PyUiApi.adb_helper.logcat_messaging_gateway import *


class StorageUSBTestsWithAdbMisc(unittest.TestCase):

    dut_emulated_storage_path = "/storage/emulated"

    def setUp(self):
        LOG.info("Test start")
        self.test_dir_base_path = EnvironmentUtils.get_emulated_storage_path()
        self.do_clean_up = True

    def tearDown(self):
        if self.do_clean_up:
            self.cleanUp()
        LOG.info("Test finished")

    def cleanUp(self, *dirs_to_delete):
        for folder in dirs_to_delete:
            AdbUtils.delete_files(folder)

    def test_internal_storage_fuse_support(self):
        # step 1 check fuse mount parameters
        mount_info_cmd = "mount | grep fuse | grep " + StorageUSBTestsWithAdbMisc.dut_emulated_storage_path
        mount_info = AdbUtils.run_adb_cmd(mount_info_cmd, add_ticks=False)
        LOG.info("got mount info: " + str(mount_info))
        self.assertTrue("/dev/fuse" in mount_info, "/storage/emulated/ is not mounted as fuse")
        mount_info_atoms = mount_info.split()
        config_params = mount_info_atoms[3].split(',')
        reference_params = ['rw', 'nosuid', 'nodev', 'noexec', 'noatime', 'user_id', 'group_id', 'default_permissions',
                            'allow_other']
        for param in reference_params:
            self.assertTrue(param in mount_info_atoms[3], "parameter %s not found in mount string" % str(param))

        # step 2 create file
        file_name = "test_fuse.txt"
        ref_content = "this is a test file guys"
        dut_dir = StorageUSBTestsWithAdbMisc.dut_emulated_storage_path + os.path.sep
        AdbUtils.create_file(file_name, dut_dir, ref_content)
        file_path_on_dut = os.path.join(dut_dir, file_name)
        content = AdbUtils.cat_file(file_path_on_dut)
        LOG.info("dut file content is: " + str(content))
        self.assertTrue(content == ref_content, "created file content does not match reference content")

        # step 3 check that the file permissions are described in fuse parameters
        get_permissions_cmd = "ls -l " + dut_dir + " | grep " + file_name
        file_ls = AdbUtils.run_adb_cmd(get_permissions_cmd, add_ticks=False)
        LOG.info("got following file permissions: " + str(file_ls))
        # file should have rw permissions for owner and group and nothing else
        self.assertTrue("-rw-rw----" in file_ls, "created file permissions differ from required")

        # step 4 try changing file permissions
        chmod_cmd = "chmod 777 " + file_path_on_dut
        AdbUtils.run_adb_cmd(chmod_cmd, add_ticks=False)
        file_ls = AdbUtils.run_adb_cmd(get_permissions_cmd, add_ticks=False)
        LOG.info("after chmod got following file permissions: " + str(file_ls))
        self.assertTrue("-rw-rw----" in file_ls, "created file permissions should not be affected by chmod")

        # step 5 try changing the owner
        chown_cmd = "chown system " + file_path_on_dut
        AdbUtils.run_adb_cmd(chown_cmd, add_ticks=False)
        file_ls = AdbUtils.run_adb_cmd(get_permissions_cmd, add_ticks=False)
        LOG.info("after chown got following file permissions: " + str(file_ls))
        self.assertTrue("root" in file_ls, "created file owner should not be affected by chown")

        # step 6 try creating a soft link
        soft_link = os.path.join(Environment.emulated_storage_path, file_name)
        link_cmd = "ln -s " + file_path_on_dut + " " + soft_link
        AdbUtils.run_adb_cmd(link_cmd)
        soft_link_content = AdbUtils.cat_file(soft_link)
        LOG.info("soft link content is: " + str(soft_link_content))
        self.assertTrue(Environment.file_not_found_cmd_output in soft_link_content, "soft link command should not work")

        # step 7 try creating a hard link
        hard_link = soft_link
        link_cmd = "ln " + file_path_on_dut + " " + soft_link
        AdbUtils.run_adb_cmd(link_cmd)
        hard_link_ls_cmd = "ls -i " + os.path.dirname(hard_link)
        hard_link_ls = AdbUtils.run_adb_cmd(hard_link_ls_cmd)
        LOG.info("hard link ls output: " + str(hard_link_ls))
        self.assertTrue(file_name not in hard_link_ls, "hard link command should not work")
