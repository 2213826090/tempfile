#!/usr/bin/python
import os
import sys
from xml.dom import minidom
import argparse
import traceback
from subprocess import Popen, PIPE
import time

PATH = os.path.abspath(__file__)
PATH = os.path.dirname(PATH)

def get_arguments():
    parser = argparse.ArgumentParser(prog='ACS to TRC result convertor')
    parser.add_argument("-if", dest="acs_input_folder", help="input ACS folder")
    parser.add_argument("-ef", dest="et_input_folder", help="input ET folder")
    parser.add_argument("-of", dest="acs_output_folder", help="output TRC result folder")
    parser.add_argument("-r", dest="format_results", help="folder where the final results are")
    parser.add_argument("-id", dest="build_id", help="build id to be added to all the output documents")
    parser.add_argument("-exec_set", dest="exec_set", help="the execution set to be converted: BAT, AFT, MTBF ...")
    args = parser.parse_args()
    return args, parser

def validate_file(io_file):
    if not os.path.isfile(io_file):
        io_file = os.path.join(PATH, io_file)
    if not os.path.isfile(io_file):
        raise Exception("File path is not valid: %s" % io_file)
    return io_file

class AcsToTrc(object):
    '''
    Convert ACS XML result into TRC CSV file format
    '''

    def __init__(self, acs_r_folder, trc_r_path, et_input_folder, format_results, build_id, exec_set):
        '''
        '''
        self.acs_r_folder = acs_r_folder
        self.trc_r_path = trc_r_path
        self.et_input_folder = et_input_folder
        self.format_results = format_results
        self.build_id = build_id
        self.exec_set = exec_set
        self.results_csv_file = None

    def extract_results(self,acs_r_path):
        '''
        This function is used to extract test results from ACS XML results
        files into TRC required form
        '''
        #print "Input ACS xml results file= ", acs_r_path
        xmldoc = minidom.parse(acs_r_path)
        ltest_case = xmldoc.getElementsByTagName('TestCase')
        for item in ltest_case:
            name = item.getAttribute('id')
            relative_path = item.getAttribute('relative_path')
            if relative_path:
                relative_path = relative_path.rsplit(os.path.sep, 1)[1]
            test_name = os.path.join(relative_path, name)

            try:
                    usecasename = item.getElementsByTagName('UseCase')[0].firstChild.data
                    if ('PY_UNIT' == usecasename):
                        parameters = item.getElementsByTagName('Parameters')[0].firstChild.data
                        full_tc_name = parameters.split(';')[-1]
                        methodname = full_tc_name.split('.')[-1]
                        if (name == methodname):
                            # The method name was used as test case for reducing the whole path length.
                            # Then convertes it back to the full test case name defined in ET
                            origin_tc_name = '.'.join(full_tc_name.split('.')[-3:])
                            test_name = os.path.join(relative_path, origin_tc_name)
            except:
                print "! Usecase not available, skipping py_unit verification for test case:",item.getAttribute('id')

            status = item.getElementsByTagName('Verdict')[0]
            status = status.firstChild.data
            failure_details = item.getElementsByTagName('Comment')[0]
            failure_detail = ""
            if failure_details.hasChildNodes():
                for child_n in failure_details.childNodes:
                    if child_n.hasChildNodes() and child_n.nodeType != child_n.TEXT_NODE:
                        failure_detail += child_n.firstChild.data
            description = item.getElementsByTagName('Description')[0]
            description = description.firstChild.data
            self.report_results_qa_reports(test_name, status, failure_detail, description)

    def report_results_qa_reports(self, test_name, status, failure_details, description):
        '''
        This function will generate the Results Report file formatted
        as valid input for TRC
        '''
        name = test_name.replace(',', '_')
        status = status.strip()
        comment = failure_details
        measure_name = ""
        value = ""
        unit = ""
        target = ""
        failure = ""
        if status == "BLOCKED":
            status = "block"
        if status == "NOT EXECUTED":
            status = "block"
        status = status.replace(',', '_').lower()
        if comment:
            comment = comment.replace(',', '_')
            comment = comment.replace('\n', '_').replace('\r', '_')
            comment = comment.replace('"', '_').replace("'", '_')
        try:
            qa_result = ""
            if not os.path.exists(os.path.dirname(self.trc_r_path)):
                os.makedirs(os.path.dirname(self.trc_r_path))
            if not os.path.isfile(self.trc_r_path):
                qa_result = open(self.trc_r_path, 'a')
                qa_result.write('Name,Status,Component,Comment,')
                qa_result.write('Measurement Name,Value,Unit,Target,Failure\n')
            else:
                qa_result = open(self.trc_r_path, 'a')
            qa_result.write('{0},{1},{2},{3},{4},{5},{6},{7},{8}\n'
                            .format(name, status, "", comment, measure_name, value, unit, target, failure))
            qa_result.close()
        except IOError, err:
            print 'IOError when trying to open file= %s' % self.trc_r_path
            raise

    def run_shell_cmd(self, cmd):
        '''
        This function is used to execute some adb commands to extract DUT properties
        '''
        process = Popen(cmd, shell=True, stdout=PIPE, stderr=PIPE)
        while process.poll() is None:
            time.sleep(1)
        (child_stdout, child_stderr) =  process.communicate()
        return (child_stdout, child_stderr)

    def write_dut_prop_in_file(self, dut_prop):
        '''
        This function will write specific data in the Results CSV
        '''
        command = "adb shell getprop | grep " + dut_prop
        (child_stdout, child_stderr) =  self.run_shell_cmd(command)
        if not child_stderr:
            self.results_csv_file.write('\n' + child_stdout)

    def transform_csv_to_trc_format(self, trc_r_path):
        '''
            This function will transform the csv input to a csv following this
            standards:
                arg1 = ACS csv
                arg2 = ET csv
                arg3 = out csv
                ex: python change_csv_test_name.py GMS_Apps.csv GMS_ET.csv GMS_TRC.csv
        '''
        if self.exec_set == "BAT":
            script_library_name = "Script Library - BAT|"
        else:
            script_library_name = "Script Library - System Functional Test|"
        if not os.path.exists(os.path.abspath(self.format_results)):
            os.makedirs(os.path.abspath(self.format_results))
        self.results_csv_file = open(os.path.join(os.path.abspath(self.format_results),os.path.basename(trc_r_path)),'a')
        index=0
        line_init=0
        line_final=1
        error_list = []
        header_line = "Package,Status,Name,StepActualResult,StepNotes\n"
        prereq_number = 0
        component = "None"
        for line in open(trc_r_path):
            try:
                if line_init!=0:
                    test_name = line.split(",")[0].split("/")[1].strip()
                    prefix_line = line.split(",")[1:2]
                    sufix_line = line.split(",")[3:]
                    found = False
                    for line2 in open("./ET/ET_DB.csv"):
                        if script_library_name in line2:
                            if index==0:
                                self.results_csv_file.write(header_line)
                                index=2
                            if test_name == line2.split(",")[1].strip():
                                #print [test_name] + prefix_line + [line2.split(",")[0].split(script_library_name)[1]] + sufix_line
                                if prefix_line[0] == "pass":
                                    prefix_line[0] = "Passed"
                                elif prefix_line[0] == "fail":
                                    prefix_line[0] = "Failed"
                                elif prefix_line[0] == "block":
                                    prefix_line[0] = "Blocked"
                                line_out = ['|'.join(line2.split(",")[0].split(script_library_name)[1].split("|")[0:2])] + prefix_line + [test_name] + [""] + [self.build_id]
                                line_final = line_final + 1
                                self.results_csv_file.write(','.join(line_out)+ '\n')
                                found = True
                                component = line2.split(",")[0].split(script_library_name)[1].split("|")[0].replace("/","_")
                                break
                       # else:
                            # component = "None"
                    if (not found):
                        if (not test_name.startswith("prereq_") and (not test_name.startswith("postreq_"))):
                            error_list.append(test_name)
                        else:
                            prereq_number = prereq_number + 1
                line_init = line_init + 1
            except Exception, err:
                traceback.print_exc()
        print "Component:                ",component
        print "Input file lines :        ",line_init
        print "Output file lines:        ",line_final
        if (prereq_number != 0):
            print "Prerequisites ignored:    ",prereq_number
        print "------------------------------"
        if (line_init - line_final - prereq_number != 0):
            print "Tests that were not found:", abs(line_init - line_final - prereq_number)
            for test_err in error_list:
                print "***", test_err
        print ""
        if (self.exec_set == "BAT") and (line_final > 1):
            #start adb
            command = "adb shell devices"
            self.run_shell_cmd(command)
            #get bios version
            self.write_dut_prop_in_file("ro.bootloader")
            #get build id
            self.write_dut_prop_in_file("ro.build.display.id")
            #get product name
            self.write_dut_prop_in_file("ro.product.name")
            #get build fingerprint
            self.write_dut_prop_in_file("ro.build.fingerprint")
            #get build version release
            self.write_dut_prop_in_file("ro.build.version.release")
            #get kernel version
            command = "adb shell cat /proc/version"
            (child_stdout, child_stderr) =  self.run_shell_cmd(command)
            if not child_stderr:
                self.results_csv_file.write('\n' + child_stdout)
        self.results_csv_file.close()
        #rename output files same as components
        old_name = os.path.join(os.path.abspath(self.format_results),os.path.basename(trc_r_path))
        print os.path.abspath(self.format_results) +"/"+ component
        if self.exec_set != "BAT":
            new_name = os.path.join(os.path.abspath(self.format_results),component) + ".csv"
        else:
            new_name = os.path.join(os.path.abspath(self.format_results),"BAT") + ".csv"
        if not os.path.isfile(new_name):
            os.rename(old_name, new_name)
        else:
            file_input = open(old_name,"r")
            file_output = open(new_name, "a")
            index_line = 1
            for line in file_input:
                if index_line != 1:
                    file_output.write(line)
                index_line = index_line + 1
            file_input.close()
            file_output.close()
            os.remove(old_name)

def main():
    args, parser = get_arguments()
    if len(sys.argv) < 8:
        parser.print_help()
        exit()
    et_input_folder = args.et_input_folder
    acs_r_folder = args.acs_input_folder
    format_results = args.format_results
    build_id = args.build_id
    exec_set = args.exec_set
    for file in os.listdir(acs_r_folder):
        if file.endswith(".xml"):
            acs_r_path = validate_file(os.path.join(os.path.abspath(acs_r_folder), file))
            trc_r_path = os.path.join(os.path.abspath(args.acs_output_folder), os.path.basename(acs_r_path).replace(".xml",".csv"))
            convertor = AcsToTrc(file, trc_r_path, et_input_folder, format_results, build_id, exec_set)
            convertor.extract_results(acs_r_path)
            convertor.transform_csv_to_trc_format(trc_r_path)
            #print "\nTRC formatted file can be found here:",trc_r_path

if __name__ == "__main__":
    main()
