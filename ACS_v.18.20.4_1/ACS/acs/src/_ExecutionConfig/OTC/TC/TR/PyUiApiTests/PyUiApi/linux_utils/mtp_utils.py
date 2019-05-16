from PyUiApi.linux_utils.common_utils import *


class MTPUtils(object):
    mtp_gvfs_monitor_path = os.path.normpath("/usr/lib/gvfs/gvfs-mtp-volume-monitor")
    mtp_gvfs_monitor_name = "gvfs-mtp-volume-monitor"

    @staticmethod
    def restart_mtp_monitor():
        if CommonLinuxUtils.is_proc_running(MTPUtils.mtp_gvfs_monitor_name):
            CommonLinuxUtils.disable_running_proc(MTPUtils.mtp_gvfs_monitor_name)
        CommonLinuxUtils.enable_proc(MTPUtils.mtp_gvfs_monitor_path, MTPUtils.mtp_gvfs_monitor_name)
