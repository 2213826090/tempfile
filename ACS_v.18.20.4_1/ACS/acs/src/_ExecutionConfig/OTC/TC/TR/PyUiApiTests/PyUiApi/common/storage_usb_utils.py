from PyUiApi.common.environment_utils import *
from PyUiApi.adb_helper.instrumentation_utils import *
from PyUiApi.common.process import shell_command_ext
import time

@staticmethod
def run_adb_cmd(cmd_string, adb_shell=True, add_ticks=True, dut_serial=None, verbose=True, timeout=None, printout=True):

    exit_code, output = AdbUtils.run_adb_cmd_ext(cmd_string, adb_shell=adb_shell,\
                            add_ticks=add_ticks, dut_serial=dut_serial, verbose=verbose, timeout=timeout)
    assert exit_code == 0, output
    if printout:
        print output

    return output

AdbUtils.run_adb_cmd = run_adb_cmd

def run_cmd(cmd_string, verbose=True, timeout=None):
    if verbose:
        print cmd_string
    exit_code, stdout, stderr = shell_command_ext(cmd_string, timeout=timeout)
    output = stdout.strip('\r\n') + stderr.strip('\r\n')

    return exit_code, output

class StorageUsbUtils(object):
    @staticmethod
    def check_sdcard_exists():
        blkid = AdbUtils.run_adb_cmd("blkid")
        if "mmcblk0p1" in blkid:
            LOG.info("SD card is inserted")
            return True
        else:
            LOG.info("SD card is not inserted")
            return False

    @staticmethod
    def check_sdcard_mounted():
        mount = AdbUtils.run_adb_cmd("mount")
        if EnvironmentUtils.get_sd_card_path_for_android_m()[:-1] in mount:
            LOG.info("SD card is mounted")
            return True
        else:
            LOG.info("SD card is not mounted")
            return False

    @staticmethod
    def test_file_exists_on_dut(path):
        instrumentation_check_command = "am instrument -e class " \
                    "com.intel.test.systemapitests.tests.FileSystemTestsDriver#testCheckFile " \
                    "-e args 'fileName:" + str(path) + "' -w com.intel.test.systemapitests/" \
                    "com.intel.test.systemapitests.runners.GenericArgumentPassingTestRunner"
        instrumentation_output = AdbUtils.run_adb_cmd(instrumentation_check_command)
        if "Failure" in instrumentation_output:
            LOG.info("Failure in instrumentation for path: " + path)
            return False
        LOG.info("instrumentation returned OK")

        ls_check_cmd = "ls " + path
        ls_check = AdbUtils.run_adb_cmd(ls_check_cmd)
        LOG.info("ls check " + path + " output: " + ls_check)
        if ls_check.strip() != path.strip():
            LOG.info("Failure in ls check")
            return False
        LOG.info("ls check for " + path + " returned OK")
        return True

    @staticmethod
    def create_file_on_dut(path, size):
        create_cmd = "dd if=/dev/urandom of=%s bs=1024 count=%d" % (path, size)
        create_file_output = AdbUtils.run_adb_cmd(create_cmd)
        LOG.info("Create file command output: " + create_file_output)
        if StorageUsbUtils.test_file_exists_on_dut(path):
            LOG.info("File created successfully in: " + path)
            return True
        else:
            LOG.info("There was a problem creating the file in: " + path)
            return False

    @staticmethod
    def get_free_space_in_MB(volume):
        df_info = AdbUtils.run_adb_cmd("df | grep " + volume)
        free_space = df_info.split()[3]
        free_space_value = re.findall("([\d.]+)", free_space)[0]
        free_space_value = float(free_space_value)
        if "g" in free_space.lower():
            free_space_value *= 1024
        elif "t" in free_space.lower():
            free_space_value *= 1024 * 1024
        elif "k" in free_space.lower():
            free_space_value /= 1024.0
        LOG.info("found free space amount of %s megabytes" % str(free_space_value))
        return free_space_value

    @staticmethod
    def fill_emulated_memory(fill_file_size_in_MB, fill_dir_path, leave_free_space_files=1):
        free_space = StorageUsbUtils.get_free_space_in_MB(Environment.storage_emulated_root)
        fill_storage_script_template = Template("cd $testdir ; for i in `seq 1 $nroffiles`; do dd if=/dev/zero "
                                                "of=testfile$$i bs=${size}m count=1 ; done")
        nr_of_files = int(free_space/fill_file_size_in_MB) - leave_free_space_files
        fill_storage_cmd = fill_storage_script_template.substitute(testdir=fill_dir_path,
                                                                   nroffiles=nr_of_files,
                                                                   size=fill_file_size_in_MB)
        AdbUtils.create_dir(fill_dir_path)
        AdbUtils.run_adb_cmd(fill_storage_cmd)
        StorageUsbUtils.get_free_space_in_MB(Environment.storage_emulated_root)

    @staticmethod
    def delete_file_from_dut(file_path):
        AdbUtils.delete_files(file_path)

    @staticmethod
    def is_external_storage_adopted():
        # check that device has adoptable storage
        cmd = "sm has-adoptable"
        output = AdbUtils.run_adb_cmd(cmd)
        if "true" not in output:
            return False
        sdcard_path = Environment.sd_card_path
        sdcard_not_adopted_check_cmd = "cat %s 2>&1; exit 0" % str(sdcard_path)
        output = AdbUtils.run_adb_cmd(sdcard_not_adopted_check_cmd)
        # if the sdcard is mounted separatly, then it is not adopted
        if Environment.file_not_found_cmd_output not in output and sdcard_path is not None:
            return False
        # check sdcard is adopted with api - THIS IS AFFECTED BY OAM-4333 - comment until the issue is clarified
        """result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testSdcardIsAdopted",
                                 instrumentation_args="",
                                 runner_name="GenericArgumentPassingTestRunner")
        if not InstrumentationInterface.was_instrumentation_test_successful(result):
            return False"""
        return True

    @staticmethod
    def open_portable_sdcard_settings_menu():
        sdcard_uuid = EnvironmentUtils.get_sd_card_uuid()
        if sdcard_uuid is not None:
            AdbUtils.run_adb_cmd("am start -a android.provider.action.DOCUMENT_ROOT_SETTINGS -d "
                                 "content://com.android.externalstorage.documents/root/"
                                 + str(sdcard_uuid)
                                 + " -t vnd.android.document/root")
        else:
            LOG.info("could not find sdcard_uuid")
        return sdcard_uuid is not None

    @staticmethod
    def get_installed_packages():
        packages = []
        packages_output = AdbUtils.run_adb_cmd('pm list packages')
        for package_line in packages_output.splitlines():
            # remove the string "package:" from the line string to get only the package name
            packages.append(package_line.replace('package:', '').strip())
        return packages

    @staticmethod
    def forget_all_and_make_public():
        # check that device has adoptable storage
        cmd = "sm has-adoptable"
        output = AdbUtils.run_adb_cmd(cmd)
        if "true" not in output:
            return False
        # Find out disk id for adoptable location
        DISK = AdbUtils.run_adb_cmd("sm list-disks adoptable").rstrip()
        # Partition disk as public
        AdbUtils.run_adb_cmd("sm partition " + DISK + " public")
        # Forget all - in case it was any sdcard previous adopted
        AdbUtils.run_adb_cmd("sm forget all")
        # wait 10 sec for sdcard to be ready
        time.sleep(10)
        # if we have an adopted sdcard and we make it portable => refresh environment
        Environment.refresh_paths_after_sdcard_operations()
        return True

    @staticmethod
    def search_mountpoint_and_fstype(mount, fstype):
        mount_output = AdbUtils.run_adb_cmd("mount | grep " + mount)
        if mount in mount_output.split() and fstype in mount_output.split():
            LOG.info(mount + " was mounted with " + fstype + " fstype!")
            return True
        else:
            LOG.info(mount + " was not mounted or doesn't have " + fstype + " fstype!")
            return False

    @staticmethod
    def verify_symlink(parent_folder, search_folder, symlink):
        list_output = AdbUtils.run_adb_cmd("ls -l " + parent_folder + " | grep " + search_folder)
        if symlink in list_output.split():
            LOG.info(search_folder + " is a symlink for " + symlink)
            return True
        else:
            LOG.info(search_folder + " is not a symlink for " + symlink)
            return False

    @staticmethod
    def search_for_file_with_api(search_dir, file_name_fragment):
        instrumentation_args = ApiTestsGenericExtraArgs()
        test_args = instrumentation_args\
            .get_args_string(searchDir=search_dir,
                             fileName=file_name_fragment)
        LOG.info("test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testSearchFile",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        return result


class ContinuousOrientationChanger(object):
    stop_operation = False

    class Orientation(object):
        def __init__(self, orientation_type, next_orientation):
            self.type = orientation_type
            self.next_orientation = next_orientation

    def __init__(self, change_wait=3):
        natural_orientation = ContinuousOrientationChanger.Orientation("n", None)
        left_orientation = ContinuousOrientationChanger.Orientation("l", natural_orientation)
        right_orientation = ContinuousOrientationChanger.Orientation("n", left_orientation)
        natural_orientation.next_orientation = right_orientation
        self.orientation_start = left_orientation
        self.change_wait = change_wait

    def start_running(self):
        orientation = self.orientation_start
        while orientation.next_orientation is not None:
            OrientationChanger.change_orientation(orientation.type)
            time.sleep(self.change_wait)
            orientation = orientation.next_orientation
            if ContinuousOrientationChanger.stop_operation:
                break


class FileCopier(object):
    def __init__(self, create_dir, copy_dir, nr_of_files_to_copy=1, file_size_kb=500000):
        self.nr_of_files_to_copy = nr_of_files_to_copy
        self.file_size_kb = file_size_kb
        self.instrumentation_args = ApiTestsGenericExtraArgs(deleteCreatedFiles="true")
        self.create_dir = create_dir
        self.copy_dir = copy_dir
        self.copy_successful = False

    def copy_file(self):
        test_args = self.instrumentation_args\
            .get_args_string(createDir=self.create_dir,
                             copyDir=self.copy_dir,
                             nrOfFiles=self.nr_of_files_to_copy,
                             fileSizeKB=self.file_size_kb)
        LOG.info("File Copier test args: " + test_args)
        result = ApiTestsInterface\
            .run_instrumentation(class_name="SystemStorageUSBTestsDriver",
                                 method_name="testCopyFile",
                                 instrumentation_args=test_args,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        if InstrumentationInterface.instrumentation_one_test_pass_output in result:
            self.copy_successful = True


class SettingsCLI(object):

    def set_key_value(self, namespace, key, value):
        assert namespace in ['system', 'secure', 'global'],\
                    "namespace is one of {system, secure, global}"

        cmd = 'settings get %s %s' % (namespace, key)
        output = AdbUtils.run_adb_cmd(cmd)
        assert output != 'null', 'Invalid key %s' % key

        cmd = 'settings put %s %s %s' % (namespace, key, value)
        output = AdbUtils.run_adb_cmd(cmd)

    def get_key_value(self, namespace, key):
        cmd = 'settings get %s %s' % (namespace, key)
        output = AdbUtils.run_adb_cmd(cmd)
        assert output != 'null', 'Invalid key %s' % key
        return output


class UIGuideSkipper(object):

    def __init__(self):
        self._regist_actions()

    def _regist_actions(self):
        d.watcher('googlequicksearchbox').when(packageNameMatches=".*googlequicksearchbox.*", text="GOT IT").click(text="GOT IT")
        d.watcher('packageinstaller').when(packageNameMatches=".*packageinstaller.*", text="Allow").click(text="Allow")

    def start_watching(self):
        print d.watchers.run()

    def stop_watching(self):
        d.watcher("googlequicksearchbox").remove()
        d.watcher("packageinstaller").remove()

uiskipper = UIGuideSkipper()
