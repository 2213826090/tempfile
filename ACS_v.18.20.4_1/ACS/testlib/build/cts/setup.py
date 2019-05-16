import os
import shutil
import sys
from distutils.core import setup
from distutils.command.bdist_dumb import bdist_dumb

class custom_bdist_dumb(bdist_dumb):

    def reinitialize_command(self, name, **kw):
        cmd = bdist_dumb.reinitialize_command(self, name, **kw)
        if name == 'install':
            cmd.install_lib = '/'
        return cmd

def create_binary_file(file_name = "run_suite", zip_file_name = "dist/run_suite-0.1.linux-x86_64.zip"):
    header = '''#!/bin/bash

export TMPDIR=`mktemp -d /tmp/run_suite.XXXXXX`

ARCHIVE=`awk '/^__ARCHIVE_BELOW__/ {print NR + 1; exit 0; }' $0`

tail -n+$ARCHIVE $0 > $TMPDIR/run_suite.zip
export PYTHONPATH=`pwd`
python $TMPDIR/run_suite.zip $@

#rm -rf $TMPDIR

exit 0

__ARCHIVE_BELOW__
'''
    with open(file_name, "wb") as bin_file:
        bin_file.write(header)
        with open(zip_file_name, "rb") as zip_file:
            bin_file.write(zip_file.read())

def clean_up():
    os.chmod("run_suite", 0755)
    os.remove("../../../__main__.py")
    shutil.rmtree("dist")
    shutil.rmtree("build")


if __name__ == '__main__':
    # Work-around to default to bdist
    sys.argv.append('bdist')
    sys.argv.append('--format=zip')

    shutil.copy("__main__.py", "../../..")
    setup(
        # our custom class override
        version = "0.1",
        cmdclass = {'bdist_dumb': custom_bdist_dumb},
        name='run_suite',
        py_modules = ['__main__',
                        'testlib.external.uiautomator',
                        'testlib.external.daemon',
                        'testlib.base.base_step',
                        'testlib.base.base_utils',
                        'testlib.base.abstract.abstract_step',
                        'testlib.base.abstract.abstract_utils',
                        'testlib.utils.relay',
                        'testlib.utils.connections.connection',
                        'testlib.utils.connections.adb',
                        'testlib.utils.connections.ssh',
                        'testlib.utils.connections.local',
                        'testlib.utils.ui.uiandroid',
                        'testlib.utils.serial_to_usb',
                        'testlib.utils.logger',
                        'testlib.utils.statics.android.statics',
                        'testlib.utils.defaults.wifi_defaults',
                        'testlib.scripts.connections.local.local_steps',
                        'testlib.scripts.connections.local.local_step',
                        'testlib.scripts.connections.local.local_utils',
                        'testlib.scripts.connections.ssh.ssh_step',
                        'testlib.scripts.connections.ssh.ssh_steps',
                        'testlib.scripts.connections.ssh.ssh_utils',
                        'testlib.scripts.android.ui.browser.browser_steps',
                        'testlib.scripts.android.ui.browser.browser_utils',
                        'testlib.scripts.file.file_utils',
                        'testlib.scripts.file.file_steps',
                        'testlib.scripts.mail.mail_steps',
                        'testlib.scripts.android.flash.flash_steps',
                        'testlib.scripts.android.flash.flash_utils',
                        'testlib.scripts.android.adb.adb_step',
                        'testlib.scripts.android.adb.adb_utils',
                        'testlib.scripts.android.adb.adb_steps',
                        'testlib.scripts.android.ui.ui_step',
                        'testlib.scripts.android.ui.ui_utils',
                        'testlib.scripts.android.ui.ui_steps',
                        'testlib.scripts.android.ui.ui_statics',
                        'testlib.scripts.android.cts.cts_steps',
                        'testlib.scripts.android.gts.gts_steps',
                        'testlib.scripts.android.cts.suite_utils',
                        'testlib.scripts.android.android_step',
                        'testlib.scripts.android.android_utils',
                        'testlib.scripts.android.ui.gms.gms_utils',
                        'testlib.scripts.android.logcat.logcat_steps',
                        'testlib.scripts.android.fastboot.fastboot_utils',
                        'testlib.scripts.wireless.wifi.wifi_generic_steps',
                        'testlib.scripts.wireless.wifi.wifi_steps',
                        'testlib.scripts.wireless.wifi.wifi_utils',
                        'testlib.scripts.kb.kb_steps',
                        'testlib.external.pyftpdlib.authorizers',
                        'testlib.external.pyftpdlib.handlers',
                        'testlib.external.pyftpdlib.servers'
                        ],
        packages = ['testlib',
                    'testlib.base',
                    'testlib.base.abstract',
                    'testlib.scripts',
                    'testlib.utils',
                    'testlib.utils.statics',
                    'testlib.scripts.connections',
                    'testlib.scripts.wireless',
                    'testlib.scripts.wireless.wifi',
                    'testlib.scripts.wireless.wifi_generic',
                    'testlib.external.pyftpdlib',
                   ],
        package_dir = {'': '../../..'}
    )

    create_binary_file()
    clean_up()
