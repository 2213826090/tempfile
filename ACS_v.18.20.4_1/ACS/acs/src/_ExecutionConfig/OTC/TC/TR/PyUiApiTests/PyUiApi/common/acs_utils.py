from PyUiApi.common.shell_utils import *
from PyUiApi.adb_helper.adb_utils import *


class AcsUtils(object):
    acs_download_dir = os.path.join(os.path.expanduser("~/"), ".acs")

    @staticmethod
    def find_file_path_in_acs_downloaded_artifacts(file_name):
        return ShellUtils.find_files_matching_pattern(AcsUtils.acs_download_dir, file_name)

    @staticmethod
    def copy_acs_artifact_to_dut(artifact_name, dut_absolute_file_path):
        artifact_abs_path = AcsUtils.find_file_path_in_acs_downloaded_artifacts(artifact_name)
        AdbUtils.push(artifact_abs_path, dut_absolute_file_path)

    @staticmethod
    def copy_acs_artifact_to_host(artifact_name, host_absolute_file_path):
        artifact_abs_path = AcsUtils.find_file_path_in_acs_downloaded_artifacts(artifact_name)
        try:
            shutil.copy(artifact_abs_path, host_absolute_file_path)
        except:
            LOG.info("could not copy -- %s -- to -- %s --" % (artifact_abs_path, host_absolute_file_path))
