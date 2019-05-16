import os

class GlobalContext:
    """
    GlobalContext class is used to hold global settings & configurations
    for test cases in single place.

    The purposes of this class:
    Decouple with nose context object;
    Provide global contexts even not run in nose runner.

    """

    def __init__(self):
        self.device_serial = None
        self.devicetype = "default"
        self.language = "en.US"
        oat_logdir_root = os.path.expanduser('~/.oat')
        if not os.path.exists(oat_logdir_root):
            os.makedirs(oat_logdir_root)
        self.user_log_dir = os.path.normpath(oat_logdir_root + '/logs/')
        if not os.path.exists(self.user_log_dir):
            os.makedirs(self.user_log_dir)
        self.sc_tmp_dir = os.path.normpath(oat_logdir_root + '/sc/')
        if not os.path.exists(self.sc_tmp_dir):
            os.makedirs(self.sc_tmp_dir)
        self.anr_captured = False
        self.crash_captured = False
