from os import path
from PyUiApi.common.uiautomator_utils import *
from PyUiApi.adb_helper.adb_utils import *
from PyUiApi.common.shell_utils import *
from PyUiApi.adb_helper.instrumentation_utils import *
import re


class EnvironmentUtils(object):
    sd_card_uuid_regex = 'mmcblk1p1.*UUID="([^"]*)"'
    usb_uuid_regex = 'sda\d.*UUID="([^"]*)"'

    @staticmethod
    def get_usb_storage_path():
        if ANDROID_VERSION in ["M", "L", "N"]:
            return EnvironmentUtils.get_usb_storage_path_for_android_M()
        else:
            LOG.info("Android version not recognized")
            return None

    @staticmethod
    def get_sd_card_path(refresh=False):
        try:
            if refresh is False and Environment.sd_card_path is not None:
                return Environment.sd_card_path
        except:
            LOG.info("proceding with sdcard initializing")
        if ANDROID_VERSION is "L":
            return "/storage/sdcard1/"
        elif ANDROID_VERSION is "M":
            sd_card_path = EnvironmentUtils.get_sd_card_path_for_android_m()
            LOG.info("Android M SDCARD path is: " + str(sd_card_path))
            return sd_card_path
        elif ANDROID_VERSION is "N":
            sd_card_path = EnvironmentUtils.get_sd_card_path_for_android_n()
            LOG.info("Android N SDCARD path is: " + str(sd_card_path))
            return sd_card_path
        else:
            LOG.info("Android version not recognized")
            return None

    @staticmethod
    def get_emulated_storage_path():
        if ANDROID_VERSION is "L":
            return "/storage/sdcard0/"
        elif ANDROID_VERSION is "M":
            return "/storage/emulated/0/"
        elif ANDROID_VERSION is "N":
            return "/storage/emulated/0/"
        else:
            LOG.info("Android version not recognized")
            return None

    @staticmethod
    def quick_adb_check_sdcard_mounted():
        cmd = "ls /storage"
        out = AdbUtils.run_adb_cmd(cmd)
        if len(out.splitlines()) > 2 or "-" in out:
            return True
        return False

    @staticmethod
    def get_sd_card_uuid(force_blkid_search=False):
        try:
            if not EnvironmentUtils.quick_adb_check_sdcard_mounted() and not force_blkid_search:
                LOG.info("SDCARD is not mounted")
                return None
            block_device_attributes = AdbUtils.run_adb_cmd("blkid")
            sd_card_uuid = re.findall(EnvironmentUtils.sd_card_uuid_regex, block_device_attributes)[0]
            if sd_card_uuid == None or len(sd_card_uuid) > 15:
                EnvironmentUtils.sd_card_uuid_regex = 'mmcblk0p1.*UUID="([^"]*)"'
                sd_card_uuid = re.findall(EnvironmentUtils.sd_card_uuid_regex, block_device_attributes)[0]
            LOG.info("SD card UUID: " + sd_card_uuid)
            return sd_card_uuid
        except:
            LOG.info("Exception in getting sdcard UUID")
            return None

    @staticmethod
    def get_usb_uuids():
        try:
            block_device_attributes = AdbUtils.run_adb_cmd("blkid")
            usb_uuids = re.findall(EnvironmentUtils.usb_uuid_regex, block_device_attributes)
            LOG.info("USB Storage UUID: " + str(usb_uuids))
            return usb_uuids
        except:
            return None

    @staticmethod
    def get_sd_card_path_for_android_m():
        try:
            sd_card_path = "/storage/" + EnvironmentUtils.get_sd_card_uuid() + "/"
            return sd_card_path
        except:
            LOG.info("Exception in trying to find the SD card UUID")
            return None

    @staticmethod
    def get_sd_card_path_for_android_n():
        # it seems the sdcard path is the same for N and M
        return EnvironmentUtils.get_sd_card_path_for_android_m()

    @staticmethod
    def get_usb_storage_path_for_android_M():
        try:
            usb_uuids = EnvironmentUtils.get_usb_uuids()
            usb_path = "/storage/" + usb_uuids[0] + "/"
            return usb_path
        except:
            LOG.info("Can't find USB storage UUID")
            return None

    @staticmethod
    def get_emulated_mountpoint():
        if ANDROID_VERSION is "L":
            return "/mnt/shell/emulated"
        elif ANDROID_VERSION is "M" or ANDROID_VERSION is "N":
            return "/storage/emulated"
        else:
            LOG.info("Android version is not recognized")

    @staticmethod
    def get_sdcard_folder_symlink():
        if ANDROID_VERSION is "L":
            return "/storage/emulated/legacy"
        elif ANDROID_VERSION is "M" or ANDROID_VERSION is "N":
            return "/storage/self/primary"
        else:
            LOG.info("Android version is not recognized")

    @staticmethod
    def get_portable_sdcard_mountpoint():
        if ANDROID_VERSION is "L":
            return "/mnt/media_rw/sdcard1"
        elif ANDROID_VERSION is "M" or ANDROID_VERSION is "N":
            sd_card_uuid = EnvironmentUtils.get_sd_card_uuid()
            return "/mnt/media_rw/" + sd_card_uuid
        else:
            LOG.info("Android version is not recognized")

    @staticmethod
    def is_sdcard_adopted():
        result = SystemApiTestsInterface\
            .run_instrumentation(class_name="storage.StorageManagerTestsDriver",
                                 method_name="testSDCardIsAdopted",
                                 instrumentation_args=None,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        return SystemApiTestsInterface.instrumentation_one_test_pass_output in result

    @staticmethod
    def is_sdcard_portable():
        result = SystemApiTestsInterface\
            .run_instrumentation(class_name="storage.StorageManagerTestsDriver",
                                 method_name="testSDCardIsMounted",
                                 instrumentation_args=None,
                                 runner_name="GenericArgumentPassingTestRunner")
        LOG.info(result)
        return SystemApiTestsInterface.instrumentation_one_test_pass_output in result

    @staticmethod
    def get_gvfs_folder_path():
        # gvfs_mount_info should be something like:
        # "gvfsd-fuse on /run/user/1000/gvfs type fuse.gvfsd-fuse (rw,nosuid,nodev,user=bogdan)"
        gvfs_mount_info = ShellUtils.run_shell_cmd('mount | grep gvfs')
        gvfs_mounts_folder = os.path.normpath(gvfs_mount_info.split()[2])
        return gvfs_mounts_folder

    @staticmethod
    def get_dut_MTP_host_path():
        '''
        # Legacy code
        systemd_user_folder_var_name = "XDG_RUNTIME_DIR"
        if systemd_user_folder_var_name not in os.environ:
            systemd_user_folder = "/run/user/1000"
        else:
            systemd_user_folder = os.environ[systemd_user_folder_var_name]
        mtp_mounts_folder = os.path.join(systemd_user_folder, "gvfs")
        '''
        mtp_mounts_folder = EnvironmentUtils.get_gvfs_folder_path()
        dut_mtp_folder_candidates = [folder for folder in os.listdir(mtp_mounts_folder) if "mtp:" in folder]
        if len(dut_mtp_folder_candidates) == 0:
            return None
        else:
            return os.path.join(mtp_mounts_folder, dut_mtp_folder_candidates[0])

    @staticmethod
    def get_dut_PTP_host_path():
        ptp_mounts_folder = EnvironmentUtils.get_gvfs_folder_path()
        dut_ptp_folder_candidates = [folder for folder in os.listdir(ptp_mounts_folder) if "host" in folder and
                                     "mtp:" not in folder]
        if len(dut_ptp_folder_candidates) == 0:
            return None
        else:
            return os.path.join(ptp_mounts_folder, dut_ptp_folder_candidates[0])

    @staticmethod
    def get_dut_MTP_internal_storage_host_path():
        host_mtp_path = EnvironmentUtils.get_dut_MTP_host_path()
        if host_mtp_path is None:
            return None
        dut_storage_mtp_host_paths = os.listdir(host_mtp_path)
        if ANDROID_VERSION in ["M", "L", "N"]:
            for path in dut_storage_mtp_host_paths:
                if "internal" in path.lower():
                    return os.path.join(host_mtp_path, os.path.normpath(path))
        else:
            LOG.info("Android version is not recognized")

    @staticmethod
    def get_dut_MTP_sd_card_host_path():
        host_mtp_path = EnvironmentUtils.get_dut_MTP_host_path()
        if host_mtp_path is None:
            return None
        dut_storage_mtp_host_paths = os.listdir(host_mtp_path)
        if ANDROID_VERSION in ["M", "L", "N"]:
            for path in dut_storage_mtp_host_paths:
                if "sd" in path.lower() and "card" in path.lower():
                    return os.path.join(host_mtp_path, os.path.normpath(path))
        else:
            LOG.info("Android version is not recognized")

    @staticmethod
    def get_dut_MTP_adopted_host_path():
        return EnvironmentUtils.get_dut_MTP_sd_card_host_path()


class Environment(object):
    api_tests_data_cache_dir = "/data/data/com.intel.test.apitests/cache/"
    check_permissions_external_cache_dir = "Android/data/com.intel.test.checkpermissions/cache/"
    api_tests_data_cache_dir_sdcard = "Android/data/com.intel.test.apitests/cache/"
    context_tests_data_cache_dir = "Android/data/com.intel.test.apitests/cache"
    context_tests_data_external_dir = "Android/data/com.intel.test.apitests/files/Documents"
    context_tests_obb_dir = "Android/obb/com.intel.test.apitests"
    file_not_found_cmd_output = "No such file or directory"
    sd_card_path = EnvironmentUtils.get_sd_card_path()
    emulated_storage_path = EnvironmentUtils.get_emulated_storage_path()
    dcim_folder_path = emulated_storage_path + "DCIM/"
    tmp_dir_path = path.join(path.dirname(path.dirname(path.abspath(__file__))), "tmp")
    internal_mountpoint = "/data"
    internal_fstype = "ext4"
    fuse_fstype = "fuse"
    fat_fstype = "vfat"
    mtp_internal_storage_path = None
    mtp_sdcard_path = None
    mtp_adopted_path = None
    ptp_path = None
    dut_tmp_dir = "/data/local/tmp/"
    storage_emulated_root = "/storage/emulated"

    @staticmethod
    def initialize_mtp_paths():
        Environment.mtp_internal_storage_path = EnvironmentUtils.get_dut_MTP_internal_storage_host_path()
        LOG.info("MTP internal storage path: " + str(Environment.mtp_internal_storage_path))
        Environment.mtp_sdcard_path = EnvironmentUtils.get_dut_MTP_sd_card_host_path()
        LOG.info("MTP sdcard host path: " + str(Environment.mtp_sdcard_path))
        Environment.mtp_adopted_path = EnvironmentUtils.get_dut_MTP_adopted_host_path()
        LOG.info("MTP adopted host path: " + str(Environment.mtp_adopted_path))

    @staticmethod
    def initialize_ptp_paths():
        Environment.ptp_path = EnvironmentUtils.get_dut_PTP_host_path()

    @staticmethod
    def clean_tmp_dir():
        ShellUtils.clean_local_dir(Environment.tmp_dir_path)

    @staticmethod
    # in case sdcard is adopted or made portable
    def refresh_paths_after_sdcard_operations():
        Environment.sd_card_path = EnvironmentUtils.get_sd_card_path(refresh=True)
