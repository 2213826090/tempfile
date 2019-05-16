"""
:copyright: (c)Copyright Intel Corporation All Rights Reserved.
The source code contained or described here in and all documents related
to the source code ("Material") are owned by Intel Corporation or its
suppliers or licensors. Title to the Material remains with Intel Corporation
or its suppliers and licensors. The Material contains trade secrets and
proprietary and confidential information of Intel or its suppliers and
licensors.

The Material is protected by worldwide copyright and trade secret laws and
treaty provisions. No part of the Material may be used, copied, reproduced,
modified, published, uploaded, posted, transmitted, distributed, or disclosed
in any way without Intel's prior express written permission.

No license under any patent, copyright, trade secret or other intellectual
property right is granted to or conferred upon you by disclosure or delivery
of the Materials, either expressly, by implication, inducement, estoppel or
otherwise. Any license under such intellectual property rights must be express
and approved by Intel in writing.

:summary: Contains classes, functions, and other objects that make it easier
to interact with the command line.
"""

# BEGIN Code copied from PythonSV
# https://subversion.jf.intel.com/deg/pve/csv/pythonsv/trunk/tangier/user/cgsapp/exe_runner.py

import os as _os
import time as _time
import threading as _threading
import subprocess as _subprocess
import sys as _sys

class ExeCommand(object):
    def __init__(self):
        self.stdout = ""
        self.stdout_lock = _threading.Lock()
        self.stderr = ""
        self.stderr_lock = _threading.Lock()
        self.timeout = 60*60*24*365 # 1 Year
        self.exe_and_args = None
        self.blocking = True
        self.return_code = None
        self.stdout_print = False
        self.stderr_print = False
        self.exe_dir = None
    def execute(self):
        '''This function takes care of kicking off the exe and monitoring the output.'''
        self.return_code = None
        self.stdout = ""
        self.stderr = ""
        exe_cwd = self.exe_dir
        if exe_cwd == None:
            exe_cwd = _os.getcwd()
        start_time = _time.time()
        process = _subprocess.Popen(self.exe_and_args, shell=False, stdout=_subprocess.PIPE,
            stderr=_subprocess.PIPE, cwd=exe_cwd)
        stdout_thread = _threading.Thread(target=self._copy_file_to_stdout, args=(process.stdout,))
        stdout_thread.start()
        stderr_thread = _threading.Thread(target=self._copy_file_to_stderr, args=(process.stderr,))
        stderr_thread.start()
        if self.blocking:
            self._monitor(process)
            # Wait a half second to gather the stdout
            _time.sleep(0.5)
            return (self.get_return_code(), self.get_stdout(), self.get_stderr())
        else:
            monitor_thread = _threading.Thread(target=self._monitor, args=(process,))
            monitor_thread.start()
    def _copy_file_to_stdout(self, file_object, read_chunk_size=1 ):
        '''This function is used to spawn the copying of stdout to self.stdout onto
        another thread so its not blocking other tasks.'''
        read_data = file_object.read(read_chunk_size)
        while read_data != "":
            if self.stdout_print:
                _sys.stdout.write(read_data)
            self.stdout_lock.acquire()
            self.stdout += read_data
            self.stdout_lock.release()
            read_data = file_object.read(read_chunk_size)
    def _copy_file_to_stderr(self, file_object, read_chunk_size=1 ):
        '''This function is used to spawn the copying of stdout to self.stdout onto
        another thread so its not blocking other tasks.'''
        read_data = file_object.read(read_chunk_size)
        while read_data != "":
            if self.stderr_print:
                _sys.stderr.write(read_data)
            self.stderr_lock.acquire()
            self.stderr += read_data
            self.stderr_lock.release()
            read_data = file_object.read(read_chunk_size)
    def _monitor(self, process):
        start_time = _time.time()
        while _time.time()<start_time+self.timeout:
            self.return_code = process.poll()
            if self.return_code!=None:
                return self.return_code
        # Time has expired
        process.terminate()
        raise Exception("Timeout waiting for process to complete.")
    def set_blocking(self, blocking_mode=True):
        self.blocking = blocking_mode
    def set_exe_and_args(self, exe_and_args):
        self.exe_and_args = exe_and_args
    def set_execution_directory(self, exe_dir):
        self.exe_dir = exe_dir
    def set_timeout(self, seconds):
        self.timeout = seconds
    def set_print(self, stdout=False, stderr=False):
        self.stdout_print = stdout
        self.stderr_print = stderr
    def get_return_code(self):
        return self.return_code
    def get_stdout(self, erase_on_read=False):
        if erase_on_read:
            self.stdout_lock.acquire()
        return_data = self.stdout
        if erase_on_read:
            self.stdout = ""
            self.stdout_lock.release()
        return return_data
    def get_stderr(self, erase_on_read=False):
        if erase_on_read:
            self.stderr_lock.acquire()
        return_data = self.stderr
        if erase_on_read:
            self.stderr = ""
            self.stderr_lock.release()
        return return_data

def exe_command(exe_and_args, background_process=False, timeout=None, print_output=False, exe_dir=None):
    '''Returns a tuple with (Return Code, Standard Output, Standard Error) if the process wasn't backgrounded.'''
    cmd = ExeCommand()
    cmd.set_exe_and_args(exe_and_args)
    cmd.set_blocking(not background_process)
    if timeout:
        cmd.set_timeout(timeout)
    cmd.set_print(print_output, print_output)
    cmd.set_execution_directory(exe_dir)
    return cmd.execute()

# END Code copied from PythonSV
