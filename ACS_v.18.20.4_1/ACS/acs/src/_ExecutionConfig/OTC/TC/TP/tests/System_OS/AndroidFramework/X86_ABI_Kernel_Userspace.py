# -*- coding:utf-8 -*-

'''
@summary: Android X86 ABI Kernel userspace test.
@since: 07/07/2016
@author: Lijin Xiong
'''

from testlib.util.uiatestbase import UIATestBase
from testlib.androidframework.adb_utils import AdbUtils
from testlib.util.log import Logger

LOG = Logger.getlogger(__name__)

class Kernel_Userspace(UIATestBase):

    def setUp(self):
        super(Kernel_Userspace, self).setUp()
        self.__test_name = __name__
        print "Setup: %s" % self.__test_name

    def test_verify_x86_abi_kernel_64_userspace_32(self):
        cmd = "getprop | grep -i abi"
        output = AdbUtils.run_adb_cmd(cmd)
        print output
        expected_results = ["[ro.product.cpu.abi]: [x86]",
                            "[ro.product.cpu.abilist32]: [x86,armeabi-v7a,armeabi]",
                            "[ro.product.cpu.abilist64]: []",
                            "[ro.product.cpu.abilist]: [x86,armeabi-v7a,armeabi]"]
        for result in expected_results:
            self.assertTrue(result in output, "reference string " + str(result) + " not found in abi props")
        cmd = "echo 1 > /proc/sys/kernel/kptr_restrict"
        AdbUtils.run_adb_cmd(cmd)
        cmd = "cat /proc/kallsyms | head -5"
        output = AdbUtils._run_adb_cmd(cmd, add_ticks=False)
        print output
        for line in output.splitlines():
            strings = line.split()
            self.assertTrue(len(strings[0]) == 16, "cat /proc/kallsyms address is not 16 bits: " + str(line))
        cmd = "cat /proc/1/maps | head -10"
        output = AdbUtils._run_adb_cmd(cmd, add_ticks=False)
        print output
        self.assertTrue("/init" in output, "/init not in cat /proc/1/maps output")
        self.assertTrue("[anon:libc_malloc]" in output, "[anon:libc_malloc] not in cat /proc/1/maps output")
        output_lines = output.splitlines()
        for line in output_lines:
            addr = line.split()[2]
            print addr
            # 32 bit address = 8 hex digits
            self.assertTrue(len(addr) == 8, "cat /proc/1/maps address is not 8 hex digits long: " + str(line))

    def test_verify_x86_abi_kernel_32_userspace_32(self):
        cmd = "getprop | grep -i abi"
        output = AdbUtils.run_adb_cmd(cmd)
        print output
        expected_results = ["[ro.product.cpu.abi]: [x86]",
                            "[ro.product.cpu.abilist32]: [x86,armeabi-v7a,armeabi]",
                            "[ro.product.cpu.abilist64]: []",
                            "[ro.product.cpu.abilist]: [x86,armeabi-v7a,armeabi]"]
        for result in expected_results:
            self.assertTrue(result in output, "reference string: " + str(result) + " not in getprop abi output")
        cmd = "echo 1 > /proc/sys/kernel/kptr_restrict"
        AdbUtils.run_adb_cmd(cmd)
        cmd = "cat /proc/kallsyms | head -5"
        output = AdbUtils._run_adb_cmd(cmd, add_ticks=False)
        print output
        for line in output.splitlines():
            strings = line.split()
            self.assertTrue(len(strings[0]) == 8, "cat /proc/kallsyms address is not 8 bits: " + str(line))
        cmd = "cat /proc/1/maps | head -10"
        output = AdbUtils._run_adb_cmd(cmd, add_ticks=False)
        print output
        self.assertTrue("/init" in output, "/init not in cat /proc/1/maps output")
        self.assertTrue("[anon:libc_malloc]" in output, "[anon:libc_malloc] not in cat /proc/1/maps output")
        output_lines = output.splitlines()
        for line in output_lines:
            addr = line.split()[2]
            print addr
            # 32 bit address = 8 hex digits
            self.assertTrue(len(addr) == 8, "cat /proc/1/maps address is not 8 bit long: " + str(line))
            if "[anon:libc_malloc]" in line:
                strings = line.split()
                addr = strings[0]
                self.assertTrue(len(addr) == 17, "libc_malloc address is not 17 char long")  # observe large address

    def test_verify_x86_abi_kernel_64_userspace_64(self):
        cmd = "getprop | grep -i abi"
        output = AdbUtils.run_adb_cmd(cmd)
        print output
        expected_results = ["[ro.product.cpu.abi]: [x86_64]",
                            "[ro.product.cpu.abilist32]: [x86,armeabi-v7a,armeabi]",
                            "[ro.product.cpu.abilist64]: [x86_64,arm64-v8a]",
                            "[ro.product.cpu.abilist]: [x86_64,x86,armeabi-v7a,armeabi,arm64-v8a]"]
#                             "[sys.chaabi.version]: [2.0.1.2091]"]
        for result in expected_results:
            self.assertTrue(result in output, "reference string: " + str(result) + " not in getprop abi output")
        cmd = "echo 1 > /proc/sys/kernel/kptr_restrict"
        AdbUtils.run_adb_cmd(cmd)
        cmd = "cat /proc/kallsyms | head -5"
        output = AdbUtils._run_adb_cmd(cmd, add_ticks=False)
        print output
        for line in output.splitlines():
            strings = line.split()
            self.assertTrue(len(strings[0]) == 16, "cat /proc/kallsyms address is not 16 bits: " + str(line))
        cmd = "cat /proc/1/maps | head -7"
        output = AdbUtils._run_adb_cmd(cmd, add_ticks=False)
        print output
        self.assertTrue("/init" in output, "/init not in cat /proc/1/maps output")
        self.assertTrue("[anon:libc_malloc]" in output, "[anon:libc_malloc] not in cat /proc/1/maps output")
        output_lines = output.splitlines()
        for line in output_lines:
            if "[anon:libc_malloc]" in line:
                strings = line.split()
                addr = strings[0]
                # observe large address
                self.assertTrue(len(addr) == 25, "cat /proc/1/maps address is not 25 chars long: " + str(line))