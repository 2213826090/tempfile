# -*- coding: utf-8 -*-
'''
@since: 01/07/2016
@author: Zhao Xiangyi
'''

from testlib.util.common import g_common_obj
from testlib.graphics.test_template.render_app_testbase import RenderAppTestBase
from testlib.graphics.renderscript_impl_N import RenderScriptImpl


class RenderscriptTest(RenderAppTestBase):

    @classmethod
    def setUpClass(cls):
        print "[setUpClass]: %s" % cls.__name__
        super(RenderscriptTest, cls).setUpClass()
        cls.d = g_common_obj.get_device()
        if g_common_obj.adb_cmd_capture_msg("ps | grep adbd")[0:4] != "root":
            g_common_obj.root_on_device()
#         g_common_obj.remount_device()

    def setUp(self):
        print "[Setup]:%s" % self.__class__.__name__
        super(RenderscriptTest, self).setUp()
        self._renderscript = RenderScriptImpl()
        self._renderscript.setup()

    def tearDown(self):
        print "[tearDown]:%s" % self.__class__.__name__
        super(RenderscriptTest, self).tearDown()

    @classmethod
    def tearDownClass(cls):
        print "[tearDownClass]: %s" % cls.__name__
        super(RenderscriptTest, cls).tearDownClass()

    def test_RenderScript_Allocation(self):
        """
        test_RenderScript_Allocation
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("AllocationCopy2DRangeTest")
        self._renderscript.run_case("AllocationCopyPaddedTest")
        self._renderscript.run_case("AllocationCopyToTest")
        self._renderscript.run_case("AllocationResize")
        self._renderscript.run_case("AllocationTest")
        self._renderscript.run_case("GetAllocationTest")
        self._renderscript.run_case("rsAllocationCopyTest")
        self._renderscript.run_case("AllocationCreateAllocationsTest")
        self._renderscript.run_case("AllocationByteBufferTest")

    def test_RenderScript_Atomic(self):
        """
        test_RenderScript_Atomic
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("AtomicTest")

    def test_RenderScript_BaseObjTest(self):
        """
        test_RenderScript_BaseObjTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("BaseObjTest")

    #def test_RenderScript_BLASData(self):
    #    """
    #    test_RenderScript_BLASData
    #    """
    #    print "[RunTest]: %s" % self.__str__()
    #    self._renderscript.run_case("BLASData")

    #def test_RenderScript_BNNMTest(self):
    #    """
    #    test_RenderScript_BNNMTest
    #    """
    #    print "[RunTest]: %s" % self.__str__()
    #    self._renderscript.run_case("BNNMTest")

    def test_RenderScript_CompilerTest(self):
        """
        test_RenderScript_CompilerTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("CompilerTest")

    def test_RenderScript_ComputeTest(self):
        """
        test_RenderScript_ComputeTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("ComputeTest")

    def test_RenderScript_DebugContext(self):
        """
        test_RenderScript_DebugContext
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("DebugContext")

    def test_RenderScript_DoubleTest(self):
        """
        test_RenderScript_DoubleTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("DoubleTest")

    def test_RenderScript_Element(self):
        """
        test_RenderScript_Element
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("ElementTest")
        self._renderscript.run_case("GetElementAt")

    def test_RenderScript_ExceptionTest(self):
        """
        test_RenderScript_ExceptionTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("ExceptionTest")

    def test_RenderScript_FieldPackerTest(self):
        """
        test_RenderScript_FieldPackerTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("FieldPackerTest")

    def test_RenderScript_Float16ArithmeticTest(self):
        """
        test_RenderScript_Float16ArithmeticTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("Float16ArithmeticTest")

    def test_RenderScript_ForEachTest(self):
        """
        test_RenderScript_ForEachTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("ForEachTest")

    def test_RenderScript_GetSetTest(self):
        """
        test_RenderScript_GetSetTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("GetSetTest")

    def test_RenderScript_Global(self):
        """
        test_RenderScript_Global
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("GlobalTest")
        self._renderscript.run_case("GlobalSync")

    def test_RenderScript_ImageProcessing(self):
        """
        test_RenderScript_ImageProcessing
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("ImageProcessingTest")

    def test_RenderScript_InitTest(self):
        """
        test_RenderScript_InitTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("InitTest")

    def test_RenderScript_Intrinsic(self):
        """
        test_RenderScript_Intrinsic
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("Intrinsic3DLut")
#        self._renderscript.run_case_sublist("IntrinsicBLAS", "intrinsicblas")
        self._renderscript.run_case("IntrinsicBlur")
        self._renderscript.run_case("IntrinsicColorMatrix")
        self._renderscript.run_case("IntrinsicConvolve3x3")
        self._renderscript.run_case("IntrinsicConvolve5x5")
        self._renderscript.run_case("IntrinsicHistogram")
        self._renderscript.run_case("IntrinsicLut")
        self._renderscript.run_case("IntrinsicResize")

    def test_RenderScript_Kernel(self):
        """
        test_RenderScript_Kernel
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("KernelInputTest")
        self._renderscript.run_case("KernelTest")

    def test_RenderScript_LaunchClip(self):
        """
        test_RenderScript_LaunchClip
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("LaunchClip")

    def test_RenderScript_Matrix(self):
        """
        test_RenderScript_Matrix
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("Matrix2fTest")
        self._renderscript.run_case("Matrix3fTest")
        self._renderscript.run_case("Matrix4fTest")

    def test_RenderScript_ObjectTest(self):
        """
        test_RenderScript_ObjectTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("ClearObjectTest")
        self._renderscript.run_case("IsObjectTest")
        self._renderscript.run_case("SetObjectTest")

    def test_RenderScript_RenderScriptTest(self):
        """
        test_RenderScript_RenderScriptTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("RenderScriptTest")

    #def test_RenderScript_RSBase(self):
    #    """
    #    test_RenderScript_RSBase
    #    """
    #    print "[RunTest]: %s" % self.__str__()
    #    self._renderscript.run_case("RSBase")
    #    self._renderscript.run_case("RSBaseCompute")

    def test_RenderScript_RsPackUnpackColorTo8888Test(self):
        """
        test_RenderScript_RsPackUnpackColorTo8888Test
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("RsPackColorTo8888Test")
        self._renderscript.run_case("RsUnpackColor8888Test")

    def test_RenderScript_SampleTest(self):
        """
        test_RenderScript_SampleTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("SampleTest")
        self._renderscript.run_case("SamplerTest")

    def test_RenderScript_ScriptTest(self):
        """
        test_RenderScript_ScriptTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("ScriptTest")
        self._renderscript.run_case("ScriptGroupTest")

    def test_RenderScript_SendToClient(self):
        """
        test_RenderScript_SendToClient
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("SendToClient")
        self._renderscript.run_case("SendToClientBlockingTest")

    def test_RenderScript_StructArrayPadTest(self):
        """
        test_RenderScript_StructArrayPadTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("StructArrayTest")
        self._renderscript.run_case("StructPadTest")

    def test_RenderScript_TestMathMisc(self):
        """
        test_RenderScript_TestMathMisc
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case_list("mathmisc")

    def test_RenderScript_TestNative(self):
        """
        test_RenderScript_TestNative
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case_list("native")

    def test_RenderScript_TestNextafter(self):
        """
        test_RenderScript_TestNextafter
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("TestNextafter")

    def test_RenderScript_TestNormalize(self):
        """
        test_RenderScript_TestNormalize
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("TestNormalize")

    def test_RenderScript_ThunkerCreateTest(self):
        """
        test_RenderScript_ThunkerCreateTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("ThunkerCreateTest")

    def test_RenderScript_TypeTest(self):
        """
        test_RenderScript_ThunkerCreateTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("TypeTest")

    def test_RenderScript_VLoadTest(self):
        """
        test_RenderScript_VLoadTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("VLoadTest")

    def test_RenderScript_VoidPtr(self):
        """
        test_RenderScript_VoidPtr
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("VoidPtr")

    def test_RenderScript_YuvTest(self):
        """
        test_RenderScript_YuvTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("YuvTest")

    def test_RenderScript_SingleSourceAllocationTest(self):
        """
        test_RenderScript_SingleSourceAllocationTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("SingleSourceAllocationTest")

    def test_RenderScript_SingleSourceForEachTest(self):
        """
        test_RenderScript_SingleSourceForEachTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("SingleSourceForEachTest")

    def test_RenderScript_SmallStructsTest(self):
        """
        test_RenderScript_SmallStructsTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("SmallStructsTest")

    def test_RenderScript_ReduceTest(self):
        """
        test_RenderScript_ReduceTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("ReduceTest")

    def test_RenderScript_refocus_RefocusTest(self):
        """
        test_RenderScript_refocus_RefocusTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("refocus.RefocusTest")

    def test_RenderScript_FloatyUnitTest(self):
        """
        test_RenderScript_FloatyUnitTest
        """
        print "[RunTest]: %s" % self.__str__()
        self._renderscript.run_case("FloatyUnitTest")
