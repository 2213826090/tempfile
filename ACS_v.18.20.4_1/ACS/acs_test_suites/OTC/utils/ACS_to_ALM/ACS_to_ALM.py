#!/usr/bin/python
import os
from xml.dom import minidom
import argparse
import traceback
from subprocess import Popen, PIPE
import time
import sys

PATH = os.path.abspath(__file__)
PATH = os.path.dirname(PATH)


def get_arguments():
    parser = argparse.ArgumentParser(prog='ACS to TRC result convertor')
    parser.add_argument("-xf", dest="acs_input_folder", help="input ACS xml folder", required=True)
    parser.add_argument("-df", dest="alm_input_folder", help="input ALM database folder", required=True)
    parser.add_argument("-r", dest="folder_results", help="results folder", required=True)

    args = parser.parse_args()

    return args, parser


def validate_file(io_file):
    if not os.path.isfile(io_file):
        io_file = os.path.join(PATH, io_file)
    if not os.path.isfile(io_file):
        raise Exception("File path is not valid: %s" % io_file)
    return io_file


def create_hpalm_db(csv_folder, csv_db_file):
    if os.path.isfile(os.path.join(os.getcwd(), csv_db_file)):
        os.remove(os.path.join(os.getcwd(), csv_db_file))

    with open(csv_db_file, 'a') as fd:
        for dirName, subdirList, fileList in os.walk(csv_folder):
            for csv_file in fileList:
                for line in open(dirName + "/" + csv_file, "r"):
                    if not "Package,Name" in line:
                        fd.write(line)


class AcsToAlm(object):
    '''
    Convert ACS XML result into TRC CSV file format
    '''

    def __init__(self, acs_r_folder, alm_r_path, alm_input_folder, folder_results, csv_db_file):

        self.acs_r_folder = acs_r_folder
        self.alm_r_path = alm_r_path
        self.alm_input_folder = alm_input_folder
        self.folder_results = folder_results
        self.csv_db_file = csv_db_file
        self.results_csv_file = None
        if os.path.isfile(self.alm_r_path):
            os.remove(self.alm_r_path)

    def extract_results(self, acs_r_path):
        '''
        This function is used to extract test results from ACS XML results
        files into TRC required form
        '''
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
                print "! Usecase not available, skipping py_unit verification for test case:", item.getAttribute('id')

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
            header_line = "Contour, Plan path, Name, Folder, Test set, Status, Bug, Comment, Tester name, Duration, Execution date, Time, Site, Value, Unit, Target, Type\n"
            if not os.path.exists(os.path.dirname(self.alm_r_path)):
                os.makedirs(os.path.dirname(self.alm_r_path))
            if not os.path.isfile(self.alm_r_path):
                qa_result = open(self.alm_r_path, 'a')
                qa_result.write(header_line)
            else:
                qa_result = open(self.alm_r_path, 'a')
            qa_result.write(
                    "{0},{1},{2},{3},{4},{5},{6},{7},{8},{9},{10},{11},{12},{13},{14},{15}\n".format("", "", name, "",
                                                                                                     "",
                                                                                                     status, "",
                                                                                                     comment,
                                                                                                     "", "", "", "", "",
                                                                                                     "",
                                                                                                     "", ""))
            qa_result.close()

        except IOError, err:
            print 'IOError when trying to open file= %s' % self.alm_r_path
            raise

    def run_shell_cmd(self, cmd):
        '''
        This function is used to execute some adb commands to extract DUT properties
        '''
        process = Popen(cmd, shell=True, stdout=PIPE, stderr=PIPE)
        while process.poll() is None:
            time.sleep(1)
        (child_stdout, child_stderr) = process.communicate()
        return (child_stdout, child_stderr)

    def write_dut_prop_in_file(self, dut_prop):
        '''
        This function will write specific data in the Results CSV
        '''
        command = "adb shell getprop | grep " + dut_prop
        (child_stdout, child_stderr) = self.run_shell_cmd(command)
        if not child_stderr:
            self.results_csv_file.write('\n' + child_stdout)

    def transform_csv_to_alm_format(self, alm_r_path):
        '''
            This function will transform the csv input to a csv file
        '''
        script_library_name = "System Functional Tests/"
        if not os.path.exists(os.path.abspath(self.folder_results)):
            os.makedirs(os.path.abspath(self.folder_results))
        self.results_csv_file = open(os.path.join(os.path.abspath(self.folder_results), os.path.basename(alm_r_path)),
                                     'a')
        index = 0
        line_init = 0
        line_final = 1
        error_list = []
        header_line = "Feature,Case Id,Test Case,Pass,Fail,N/A,Measured,Comment,Measurement Name,Value,Unit,Target,Failure,Build Id,Duration,Bug\n"
        prereq_number = 0
        component = "None"
        for line in open(alm_r_path):
            try:
                if line_init != 0:
                    test_name = line.split(",")[2].split("/")[1].strip()
                    status = line.split(",")[5]

                    found = False
                    for line2 in open(self.csv_db_file):
                        if test_name not in line2:
                            continue
                        if script_library_name in line2:
                            if index == 0:
                                self.results_csv_file.write(header_line)
                                index = 2
                            if line2.strip().strip("\n").endswith(test_name):
                                component = line2.split(",")[0].split(script_library_name)[1].split("/")[0].replace("/","_")
                                if status == "pass":
                                    out_status = ["1", "", ""]
                                elif status == "fail":
                                    out_status = ["", "1", ""]
                                else:
                                    out_status = ["", "", "1"]
                                line_out = [component] + [test_name] + [""] + out_status + [""] * 10
                                line_final = line_final + 1
                                t_r = ','.join(line_out) + '\n'
                                self.results_csv_file.write(t_r.replace('/', "\\"))
                                found = True
                                break
                        else:
                            component = "None"
                    if (not found):
                        if (not test_name.startswith("prereq_") and (not test_name.startswith("postreq_"))):
                            error_list.append(test_name)
                        else:
                            prereq_number = prereq_number + 1
                line_init = line_init + 1
            except Exception, err:
                traceback.print_exc()
        print "Component:                ", component
        print "Input file lines :        ", line_init - 1
        print "Output file lines:        ", line_final - 1
        if (prereq_number != 0):
            print "Prerequisites ignored:    ", prereq_number
        print "------------------------------"
        if (line_init - line_final - prereq_number != 0):
            print "Tests that were not found:", abs(line_init - line_final - prereq_number)
            for test_err in error_list:
                print "***", test_err
        print ""
        self.results_csv_file.close()
        # rename output files same as components
        old_name = os.path.join(os.path.abspath(self.folder_results), os.path.basename(alm_r_path))
        # print os.path.abspath(self.folder_results) + "/" + component
        new_name = os.path.join(os.path.abspath(self.folder_results), component) + ".csv"
        if not os.path.isfile(new_name):
            os.rename(old_name, new_name)
        else:
            file_input = open(old_name, "r")
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

    alm_input_folder = args.alm_input_folder
    acs_r_folder = args.acs_input_folder
    folder_results = args.folder_results
    csv_db_file = "testcases.txt"
    xml_to_csv_folder = "tmp"
    create_hpalm_db(alm_input_folder, csv_db_file)

    for file in os.listdir(acs_r_folder):
        if file.endswith(".xml"):
            acs_r_path = validate_file(os.path.join(os.path.abspath(acs_r_folder), file))
            alm_r_path = os.path.join(os.path.abspath(xml_to_csv_folder),
                                      os.path.basename(acs_r_path).replace(".xml", ".csv"))
            convertor = AcsToAlm(file, alm_r_path, alm_input_folder, folder_results, csv_db_file)
            convertor.extract_results(acs_r_path)
            convertor.transform_csv_to_alm_format(alm_r_path)


if __name__ == "__main__":
    main()
