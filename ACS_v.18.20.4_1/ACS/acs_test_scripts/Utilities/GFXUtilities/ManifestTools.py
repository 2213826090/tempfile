#!/usr/bin/python

# Copyright @ 2012 Intel Corp .
# All rights reserved
#
# File: conformance_tool_lib.py
# Author: Thuan T. Tran
# Date: 12-20-2012
#
# Description:
#     This script generates test result summary for
# OGL ES conformance test 1.1

from xml.dom.minidom import parse, Document


lib_path = os.path.abspath('./')
sys.path.append(lib_path)

from GFXFunctions import *

szBeginTag = "Default State test"
szEndTag = "Conform"

# Function write_to_csv_file
#     This function will check the value field.
#     if the value field is positive it will PASSED else FAILED
# Parameters:
#     List: The list of to output with new name.
#     csvFname: This file has the list of all test names
# Return: none
def write_to_csv_file(List, csvFname):
    pfile = open(csvFname, "wb")
    print "Write to file: %s" % csvFname
    for szVar, x in List:
        if (x > 0):
            szVar = "%s; PASSED;  %d\n" % (szVar, x)
            pfile.write(szVar)
        else:
            szVar = "%s; FAILED;  %d\n" % (szVar, x)
            pfile.write(szVar)
    pfile.close()
    return


# Function gen_map_variables_name
#     This function will check List variables name
#     and replace with new name.
# Parameters:
#     List: The list of to output with new name.
#     inFname: This file has the list of all test names old and new
# Return: List  ( Updated List)

def gen_map_variables_name(List, inFname):
    pfile = open(inFname, "r")
    strText = pfile.readlines()
    pfile.close()
    mapText = []
    for line in strText:
        line = line.strip('\n')
        words = line.split(':')
        oldName = words[0].strip()
        if (len(oldName) > 0):
            newName = words[1]
            newList = [oldName, newName]
            mapText.append(newList)
    for i, nameList in enumerate(List):
        for j, szName in enumerate(mapText):
            if (nameList[0] == szName[0]):
                nameList[0] = szName[1]  # replace with new name
    return List


# Function get_selection_list
#     This function will generate unique List variables name
#     with not duplicate the test names.
# Parameters:
#     List: The list of to output with new name.
#     Filename: This file has the list of all tests, all passes and all levels
# Return: List  ( Updated List)

def get_selection_list(List, Filename):
    iStart = 0
    iSet = 0
    log = open(Filename, "r")
    lines = log.readlines()
    log.close()
    if (len(List) == 0):
        iCreate = 1  # create List
    else:
        iCreate = 0
    for line in lines:
        line = line.strip('\n')
        words = line.split(':')
        strLine = words[0].strip()
        if (len(strLine) > 0):
            if ( strLine == szBeginTag ):
                iStart = 1
                iSet += 1
            if (iStart == 1):
                temp = words[1]
                if (len(temp) > 0 ):
                    result = (temp.strip()).strip('.')
                if (iSet == 1 and iCreate == 1):  ## Set up the List
                    newlist = [strLine, 1]
                    List.append(newlist)
                else:
                    for i, indexList in enumerate(List):
                        if (indexList[0] == strLine):
                            if (result == "passed"):
                                indexList[1] += 1
                            else:
                                indexList[1] = -9999

        else:
            iStart = 0

    return List


def get_other_list(List, Filename):
    iSet = 0
    log = open(Filename, "r")
    lines = log.readlines()
    log.close()
    if (len(List) == 0):
        iCreate = 1  # create List
    else:
        iCreate = 0

    for line in lines:
        line = line.strip('\n')
        if (line.find(':') > 0):
            words = line.split(':')
            strLine = words[0].strip()
            if (len(strLine) > 0):
                print ("%s") % strLine
                newlist = [strLine, 1]
                List.append(newlist)
                iSet = iSet + 1
                break;
    if (iSet == 0):
        szMsg = "%s : 1" % Filename
        newlist = [szMsg, 1]
        List.append(newlist)
    return List


# Function gen_ogl_es_1_result
#     This function will collect all tests, all passes and all levels.
#     This will control the execution of the program
# Parameters:
#     List: The list of to output with new name.
#     log_fixed_p0_fname:
#     log_fixed_p1_fname:
#     log_fixed_p2_fname:
#     log_fixed_p3_fname:
#     log_float_p0_fname:
#     log_float_p1_fname:
#     log_float_p2_fname:
#     log_float_p3_fname:
#
#     test_name_file_name, output_file_name: This file has the list of all tests, all passes and all levels
# Return: None
def gen_ogl_es_1_result(log_fixed_p0_fname, log_fixed_p1_fname, log_fixed_p2_fname, log_fixed_p3_fname, \
                        log_float_p0_fname, log_float_p1_fname, log_float_p2_fname, log_float_p3_fname, \
                        log_covgl_fname, log_covegl_fname, log_fixed_primtest_fname, \
                        test_name_file_name, output_file_name):
    #log_float_primtest_fname is hanging so disable for now
    L = []
    print "Filename: %s" % log_fixed_p0_fname
    L = get_selection_list(L, log_fixed_p0_fname)
    L = get_selection_list(L, log_fixed_p1_fname)
    L = get_selection_list(L, log_fixed_p2_fname)
    L = get_selection_list(L, log_fixed_p3_fname)
    L = get_selection_list(L, log_float_p0_fname)
    L = get_selection_list(L, log_float_p1_fname)
    L = get_selection_list(L, log_float_p2_fname)
    L = get_selection_list(L, log_fixed_p3_fname)
    print("Total List item = %d" % len(L))
    L = get_other_list(L, log_covgl_fname)
    L = get_other_list(L, log_covegl_fname)
    L = get_other_list(L, log_fixed_primtest_fname)
    #    L= get_other_list(L,  log_float_primtest_fname)
    L = gen_map_variables_name(L, test_name_file_name)
    write_to_csv_file(L, output_file_name)

    return


# Function get_selection_list_es3
#     This function will generate unique List variables name
#     with not duplicate the test names.
# Parameters:
#     List: The list of to output with new name.
#     Filename: This file has the list of all tests, all passes and all levels
# Return: List  ( Updated List)

def get_selection_list_es3(List, Filename):
    iStart = 0
    iSet = 0
    log = open(Filename, "r")
    lines = log.readlines()
    log.close()
    if (len(List) == 0):
        iCreate = 1  # create List
    else:
        iCreate = 0
    for line in lines:
        line = line.strip('\r\n')
        if (line == "ConformanceGLESConfigEnd"):
            iSet = 1
        if (line.find("ConformanceGLES_Passed") > 0):
            words = line.split(':')
            testcase = words[1].strip()
            if (testcase != "GTF"):
                if (iCreate == 1 and iSet == 0 ):  ## Set up the List
                    # for debug only
                    #                    print("SUCCESS Testcase: %s" %  testcase )
                    newlist = [testcase, 1]
                    List.append(newlist)
                else:
                    for i, indexList in enumerate(List):
                        if (indexList[0] == testcase):
                            indexList[1] += 1
        if (line.find("ConformanceGLES_Failed") > 0):
            line = line.strip('\n')
            words = line.split(':')
            testcase = words[1].strip()
            if (testcase != "GTF"):
                if (iCreate == 1 and iSet == 0):  ## Set up the List
                    # for debug only
                    #                    print("FAILED Testcase: %s" %  testcase )
                    newlist = [testcase, 1]
                    List.append(newlist)
                else:
                    for i, indexList in enumerate(List):
                        if (indexList[0] == testcase):
                            indexList[1] = -9999
    return List


# Function gen_ogl_es_3_result
#     This function will collect all tests, all passes and all levels for OGLES3.
#     This will control the execution of the program
# Parameters:
#     List: The list of to output with new name.
#     log_fname1:
#     log_fname2:
#     log_fname3:
#     log_fname4:
#     test_name_file_name:
#     output_file_name:
#
#     test_name_file_name, output_file_name: This file has the list of all tests, all passes and all levels
# Return: None

def gen_ogl_es_3_result(log_fname1, log_fname2, log_fname3, log_fname4, \
                        test_name_file_name, output_file_name):
    L = []
    L = get_selection_list_es3(L, log_fname1)
    L = get_selection_list_es3(L, log_fname2)
    L = get_selection_list_es3(L, log_fname3)
    L = get_selection_list_es3(L, log_fname4)
    print("Total List item = %d" % len(L))
    L = gen_map_variables_name(L, test_name_file_name)
    write_to_csv_file(L, output_file_name)
    # debug
    #    print "List :", L
    return


# Copyright @ 2012, 2013
# Intel Corp.
# All rights reserved

# File: create_ogl_es_manifest.py
# Author: Gunawan Ali-Santosa
# Date: 06-20-2013
#
# This tool is used to automatically generate OGL ES test manifest file.
# Note:
#     This tool is intended to be generic tool, but for now,
# its primary function is for OGL ES 3.0
#
# Usage:
#     create_ogl_es_manifest.py test_list_filename manifest_filename
#
# Where:
#     test_list_filename: Name of file that contains all tests
#     manifest_filename: Name of test manifest file, in .xml format


def generate_oglconform_manifest(test_list_filename):
    if os.path.isfile(test_list_filename):
        print ("\n Generating OGLConform test manifest file")
    else:
        print ("\n **** ERROR **** Test list file name does not exist: %s" % test_list_filename)
        sys.exit(1)

    test_list_file = open(test_list_filename, "r")
    test_list = test_list_file.readlines()
    test_list_file.close()

    manifest_file = 'oglconform_auto.xml'

    #defining the manifest file structure

    manifest_document = Document()

    root = manifest_document.createElement('oglconform_auto')
    manifest_document.appendChild(root)

    id = 000000
    for test in test_list:
        id = id + 1
        test_name = test.strip('\n')
        test_name = test_name.strip('\r')
        push_binary = 'oglconform;/data/app'
        adb_cmd = 'adb shell chmod -R 777 /data/app'
        #get the test-suite name
        test_suite_name = test_name.split(' ')[0]
        subtestcase_name = test_name.split(' ')[1]

        test_case = manifest_document.createElement('test_case')
        # test_case.setAttribute('name', 'oglconform_%s' % test_suite_name)
        test_case.setAttribute('name', 'oglconform_%s' % test_name)
        test_case.setAttribute('id', str(id))
        root.appendChild(test_case)

        test_item = manifest_document.createElement('test_item')
        test_item.setAttribute('name', '')
        test_case.appendChild(test_item)

        test_step = manifest_document.createElement('test_step')
        test_step.setAttribute('type', 'fun')
        test_step.setAttribute('fun_name', 'test_sequence_handler')
        # parameter = manifest_document.createElement('parameter')
        # parameter.setAttribute('name', 'push_native_app')
        # parameter_content = manifest_document.createTextNode('%s' % push_binary)
        # parameter.appendChild(parameter_content)
        # test_step.appendChild(parameter)
        parameter = manifest_document.createElement('parameter')
        parameter.setAttribute('name', 'adb_cmd')
        parameter_content = manifest_document.createTextNode('%s' % adb_cmd)
        parameter.appendChild(parameter_content)
        test_step.appendChild(parameter)
        test_item.appendChild(test_step)

        test_step = manifest_document.createElement('test_step')
        test_step.setAttribute('type', 'fun')
        test_step.setAttribute('fun_name', 'ogles_conformance_test_run')
        parameter = manifest_document.createElement('parameter')
        parameter.setAttribute('name', 'test_file_name')
        parameter_content = manifest_document.createTextNode('%s' % test_name)
        parameter.appendChild(parameter_content)
        test_step.appendChild(parameter)

        parameter = manifest_document.createElement('parameter')
        parameter.setAttribute('name', 'test_bin_name')
        parameter_content = manifest_document.createTextNode('%s' % push_binary)
        parameter.appendChild(parameter_content)
        test_step.appendChild(parameter)

        parameter = manifest_document.createElement('parameter')
        parameter.setAttribute('name', 'run_ext_es2')
        test_step.appendChild(parameter)

        parameter = manifest_document.createElement('parameter')
        parameter.setAttribute('name', 'result_check_string')
        parameter_content = manifest_document.createTextNode('%s subcase' % subtestcase_name)
        parameter.appendChild(parameter_content)
        test_step.appendChild(parameter)
        test_item.appendChild(test_step)

    file_handler = open(manifest_file, 'w')
    file_handler.write(manifest_document.toprettyxml(indent='\t'))
    file_handler.close()


def oglconform_make_list_folder(dir):
    if not os.path.exists(dir):
        os.makedirs(dir)
    return dir


def oglconform_generate_list(suite_name):
    generate_command = "adb shell /data/app/oglconform -generateTestList /data/app/" + suite_name + " -test " + suite_name
    temp = oglconform_make_list_folder('temp')

    pull_command = "adb pull /data/app/" + suite_name + " ."
    #print generate_command
    os.system(generate_command)
    time.sleep(5)
    os.system(pull_command)


TEST_SUITES = [
    "egl-basic",
    "egl-config",
    "egl-context",
    "egl-image",
    "egl-android",
    "egl-surface",
    "egl-initialization",
    "egl-sync",
    "egl-android-sync",
    "egl-gpa_es1",
    "egl-gpa_es2",
    "egl-android-fb",
    "egl-reusable-sync",
]


def oglconform_generate_request():
    while True:
        answer = raw_input("Generate test lists? [Y/N]")
        if answer in ["Y", "y", "Yes", "YES"]:
            with open('test_list.txt', 'w') as outfile:
                for suite_name in TEST_SUITES:
                    oglconform_generate_list(suite_name)
                    with open(suite_name) as infile:
                        for line in infile:
                            outfile.write(line)
                break
        elif answer in ["n", "N", "No", "no"]:
            break
    else:
        print "Please enter a valid answer"


def khronos_generate_manifest(test_list_filename, manifest_file):
    if os.path.isfile(test_list_filename):
        print ("\n Generating OGLConform test manifest file")
    else:
        print ("\n **** ERROR **** Test list file name does not exist: %s" % test_list_filename)
        sys.exit(1)

    test_list_file = open(test_list_filename, "r")
    test_list = test_list_file.readlines()
    test_list_file.close()

    test_directory = test_list_filename.split(".")[0]
    print "\n\n Test Directory " + test_directory
    #manifest_file = 'oglconform_auto.xml'

    #defining the manifest file structure

    manifest_document = Document()

    root = manifest_document.createElement('ogl_es_2.0_conformance_auto')
    manifest_document.appendChild(root)

    id = 000000
    for test in test_list:
        id = id + 1
        test_name = test.strip('\n')
        test_name = test_name.strip('\r')
        push_binary = 'GTF;/data/app'
        adb_cmd = 'adb shell chmod -R 777 /data/app'
        pass_string = 'PASSED all'
        fail_string = 'PASSED all 0'
        rm_files = 'adb shell rm -rf /data/app/*.'
        #get the test-suite name
        # test_suite_name = test_name.split(' ')[0]
        # subtestcase_name = test_name.split(' ')[1]

        test_case = manifest_document.createElement('test_case')
        test_case.setAttribute('name', 'ogles2.0_conf_%s_%s' % (test_directory, test_name))
        test_case.setAttribute('id', str(id))
        root.appendChild(test_case)

        test_item = manifest_document.createElement('test_item')
        test_item.setAttribute('name', '')
        test_case.appendChild(test_item)

        test_step = manifest_document.createElement('test_step')
        test_step.setAttribute('type', 'fun')
        test_step.setAttribute('fun_name', 'test_sequence_handler')
        parameter = manifest_document.createElement('parameter')
        parameter.setAttribute('name', 'adb_cmd')
        parameter_content = manifest_document.createTextNode('%s' % adb_cmd)
        parameter.appendChild(parameter_content)
        test_step.appendChild(parameter)
        test_item.appendChild(test_step)

        test_step = manifest_document.createElement('test_step')
        test_step.setAttribute('type', 'fun')
        test_step.setAttribute('fun_name', 'ogles_conformance_test_run')
        parameter = manifest_document.createElement('parameter')
        parameter.setAttribute('name', 'test_file_name')
        parameter_content = manifest_document.createTextNode('%s/%s/input.run' % (test_directory, test_name))
        parameter.appendChild(parameter_content)
        test_step.appendChild(parameter)

        parameter = manifest_document.createElement('parameter')
        parameter.setAttribute('name', 'test_bin_name')
        parameter_content = manifest_document.createTextNode('%s' % push_binary)
        parameter.appendChild(parameter_content)
        test_step.appendChild(parameter)

        parameter = manifest_document.createElement('parameter')
        parameter.setAttribute('name', 'log_param')
        parameter_content = manifest_document.createTextNode('/data/app/%s' % test_name)
        parameter.appendChild(parameter_content)
        test_step.appendChild(parameter)
        test_item.appendChild(test_step)

        test_step = manifest_document.createElement('test_step')
        test_step.setAttribute('type', 'fun')
        test_step.setAttribute('fun_name', 'check_result')
        parameter = manifest_document.createElement('parameter')
        parameter.setAttribute('name', 'result_file')
        parameter_content = manifest_document.createTextNode('/data/app/%s.txt' % test_name)
        parameter.appendChild(parameter_content)
        test_step.appendChild(parameter)

        parameter = manifest_document.createElement('parameter')
        parameter.setAttribute('name', 'pass_string')
        parameter_content = manifest_document.createTextNode('%s' % pass_string)
        parameter.appendChild(parameter_content)
        test_step.appendChild(parameter)

        parameter = manifest_document.createElement('parameter')
        parameter.setAttribute('name', 'fail_string')
        parameter_content = manifest_document.createTextNode('%s' % fail_string)
        parameter.appendChild(parameter_content)
        test_step.appendChild(parameter)
        test_item.appendChild(test_step)

        test_step = manifest_document.createElement('test_step')
        test_step.setAttribute('type', 'fun')
        test_step.setAttribute('fun_name', 'test_sequence_handler')
        parameter = manifest_document.createElement('parameter')
        parameter.setAttribute('name', 'adb_cmd')
        parameter_content = manifest_document.createTextNode('%stxt' % rm_files)
        parameter.appendChild(parameter_content)
        test_step.appendChild(parameter)

        parameter = manifest_document.createElement('parameter')
        parameter.setAttribute('name', 'adb_cmd')
        parameter_content = manifest_document.createTextNode('%sxml' % rm_files)
        parameter.appendChild(parameter_content)
        test_step.appendChild(parameter)
        test_item.appendChild(test_step)

    file_handler = open(manifest_file, 'w')
    file_handler.write(manifest_document.toprettyxml(indent='\t'))
    file_handler.close()
