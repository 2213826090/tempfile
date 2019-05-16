"""
@summary:
Test step to create symbolic links to BusyBox for commands that BusyBox
implements, so that our shell commands will continue to work. It will
attempt to do this only if no /system/xbin/sed node exists yet.

The Busybox binary is included in artifactory at the following location:
    acs_test_artifacts/busybox/busybox

It came from a KitKat build and is known to also work on Lollipop. The
TestStep will create all the symbolic links for the shell commands that
are expected in /system/xbin.  These commands are included in
busybox_commands list.

Here is the right TestStep sequence to install BusyBox utils :

    <TestStepSet Id="BB_Initialize">
        <TestStep Id="MAKE_SYSTEM_PARTITION_RW" DEVICE="PHONE1"/>
        <TestStep Id="GET_ARTIFACT" ARTIFACT="busybox/busybox" ARTIFACT_SOURCE="FROM_BENCH:ARTIFACT_MANAGER:URI" TRANSFER_TIMEOUT="DEFAULT" STORED_FILE_PATH="FILE_SRC_PATH" EQT="DEFAULT"/>
        <TestStep Id="INSTALL_FILE" DEVICE="PHONE1" FILE_PATH="FROM_CTX:FILE_SRC_PATH" TYPE="bin" DESTINATION="/system/xbin" TIMEOUT="DEFAULT" DESTINATION_STORED_PATH="FILE_DEST_PATH"/>
        <TestStep Id="FINALIZE_BUSYBOX_INSTALLATION" DEVICE="PHONE1"/>
    </TestStepSet>

@since 25 May 2015
@author: Aamir Khowaja
@organization: INTEL TMT-TTD-AN
"""

from Core.TestStep.DeviceTestStepBase import DeviceTestStepBase
import os

class FinalizeBusyBoxInstallation(DeviceTestStepBase):
    def __init__(self, tc_conf, global_conf, ts_conf, factory):
        """
        Constructor: instantiates busybox commands list
        """

        DeviceTestStepBase.__init__(self, tc_conf, global_conf, ts_conf, factory)
        self.busybox_commands = ["acpid", "addgroup", "adduser", "adjtimex", "ar", "arp", "arping", "ash", "awk", "basename", "bbconfig", "beep", "blkid", "brctl", "bunzip2", "bzcat", "bzip2", "cal", "cat", "catv", "chat", "chattr", "chgrp", "chmod", "chown", "chpasswd", "chpst", "chroot", "chrt", "chvt", "cksum", "clear", "cmp", "comm", "cp", "pio", "crond", "crontab", "cryptpw", "cttyhack", "cut", "date", "dc", "dd", "deallocvt", "delgroup", "deluser", "depmod", "devmem", "df", "dhcprelay", "diff", "dirname", "dmesg", "dnsd", "dnsdomainname", "dos2unix", "du", "dumpkmap", "dumpleases", "echo", "ed", "egrep", "eject", "env", "envdir", "envuidgid", "ether-wake", "expand", "expr", "fakeidentd", "false", "fbset", "fbsplash", "fdflush", "fdformat", "fdisk", "fgrep", "find", "findfs", "flash_eraseall", "flash_lock", "flash_unlock", "flashcp", "fold", "free", "freeramdisk", "fsck", "fsck.minix", "fsync", "ftpd", "ftpget", "ftpput", "fuser", "getopt", "getty", "grep", "gunzip", "gzip", "halt", "hd", "hdparm", "head", "hexdump", "hostid", "hostname", "httpd", "hush", "hwclock", "id", "ifconfig", "ifdown", "ifenslave", "ifplugd", "ifup", "inetd", "init", "inotifyd", "insmod", "install", "ionice", "ip", "ipaddr", "ipcalc", "ipcrm", "ipcs", "iplink", "iproute", "iprule", "iptunnel", "kbd_mode", "kill", "killall", "killall5", "klogd", "last", "length", "less", "linux32", "linux64", "linuxrc", "ln", "loadfont", "loadkmap", "logger", "login", "logname", "logread", "losetup", "lpd", "lpq", "lpr", "ls", "lsattr", "lsmod", "lspci", "lsusb", "lzmacat", "lzop", "lzopcat", "makedevs", "makemime", "man", "md5sum", "mdev", "mesg", "microcom", "mkdir", "mkdosfs", "mke2fs", "mkfifo", "mkfs.ext2", "mkfs.minix", "mkfs.vfat", "mknod", "mkpasswd", "mkswap",  "mktemp", "modprobe", "more", "mount", "mountpoint", "msh", "mt", "mv", "nameif", "nc", "netstat", "nice", "nmeter", "nohup", "nslookup", "ntpd", "od", "openvt", "passwd", "patch", "pgrep", "pidof", "ping", "ping6", "pipe_progress", "pivot_root", "pkill", "popmaildir", "poweroff", "printenv", "printf", "ps", "pscan", "pwd", "raidautorun", "rdate", "rdev", "readahead", "readlink", "readprofile", "realpath", "reboot", "reformime", "renice", "reset", "resize", "rm", "rmdir", "rmmod", "route", "rtcwake", "run-parts", "runlevel", "runsv", "runsvdir", "rx", "script", "scriptreplay", "sed", "sendmail", "seq", "setarch", "setconsole", "setfont", "setkeycodes", "setlogcons", "setsid", "setuidgid", "sh", "sha1sum", "sha256sum", "sha512sum", "showkey", "slattach", "sleep", "softlimit", "sort", "split", "start-stop-daemon", "stat", "strings", "stty", "su", "sulogin", "sum", "sv", "svlogd", "swapoff", "swapon", "switch_root", "sync", "sysctl", "syslogd", "tac", "tail", "tar", "taskset", "tcpsvd", "tee", "telnet", "telnetd", "test", "tftp", "tftpd", "time", "timeout", "top", "touch", "tr", "traceroute", "traceroute6", "true", "tty", "ttysize", "tunctl", "tune2fs", "udhcpc", "udhcpd", "udpsvd", "umount", "uname", "uncompress", "unexpand", "uniq", "unix2dos", "unlzma", "unlzop", "unzip", "uptime", "usleep", "uudecode", "uuencode", "vconfig", "vi", "vlock", "volname", "wall", "watch", "watchdog", "wc", "wget", "which", "who", "whoami", "xargs", "yes", "zcat", "zcip"]

    def run(self, context):
        """
        Runs the test step

        @type context: TestStepContext
        @param context: test case context
        """

        DeviceTestStepBase.run(self, context)
        self._logger.info("FinalizeBusyBoxInstallation: Run")

        self.file_api = self._device.get_uecmd("File")
        self._logger.info("Checking for existing busybox installation")
        status, err_msg = self.file_api.exist("/system/xbin/busybox")
        if status:
            self._logger.info("busybox already deployed.")
        else:
            self._logger.info("busybox is missing. Please run TestStep to deploy the binary from artifactory and setup filesystem to rw.")
            return
        self._logger.info("Checking for sed as a sanity check that busybox paths were set up")
        status, err_msg = self.file_api.exist("/system/xbin/sed")
        if status:
            self._logger.info("It looks like at least sed was set up too, so the existing installation appears sane.")
            return
        else:
            self._logger.info("Setting up shell command symbolic links")
            self._logger.warning("You may see lots of errors from ADB that these files exist")
            for command in self.busybox_commands:
                self._logger.debug("Setting up %s" % command)
                status, err_msg = self._device.run_cmd("adb shell ln -s /system/xbin/busybox /system/xbin/%s" % command, 3, True)
                if not err_msg:
                    self._logger.info("%s setup successfully" %command)
                else:
                    self._logger.warning("%s" %err_msg)
                    self._logger.warning("continuing with other commands ..")
