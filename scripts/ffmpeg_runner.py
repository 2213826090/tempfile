#coding=utf-8
"""
Create at 08/3/23 by Zeng Hui
for running ffmpeg
"""

import re
import os
import sys
import csv
import time
import shutil

def printHelp(scriptName):
    print """Error: No specified cases or suites was given
  Command line examples:

  a) single testSuite execution:
    %s testSuite

  b) multi testSuites execution:
    %s testSuite1 testSuite2 ...

  c) single testCase from single testSuite execution:
    %s testSuite testCase

  d) multi testCases from single testSuite execution:
    %s testSuite testCase1 testCase2 ...

    """ % (scriptName, scriptName, scriptName, scriptName)

def checkArgv(argv):
    argvList = argv
    if len(argvList) == 1 or not argvList[1].endswith(".csv"):
        printHelp(argvList[0])
        sys.exit(1)
    else:
        testSuites = []
        Cases = []
        for i in argvList[1:] :
            if i.endswith(".csv"):
                testSuites.append(i)
            else:
                Cases.append(i)
        return [testSuites, Cases]

def generateCmdLine(parameter):
    cmd = FFMPEG
    para = parameter
    if not os.path.isdir("temp" + os.sep + para['Test ID']):
        os.makedirs("temp" + os.sep + para['Test ID'])
    else:
        pass
    for i in config.readlines():
        for j in dict.keys(para):
            if j and re.match(j, i) and para[j]:
                cmd = cmd + " " + i.split()[2] + " " + para[j]
    cmd += " temp" + os.sep  + para['Test ID'] + os.sep + "out.264"
    return cmd

def runCases(testSuite, Cases):
    scenario = testSuite
    cases = Cases
    with open(scenario, 'r+') as csvfile:
        reader = csv.DictReader(csvfile)
        parameters = [i for i in reader]
    if not cases :
        for parameter in parameters:
            if  parameter['Test ID'].startswith("#") or  parameter['State'] == "b" :
                pass
            else:
                commandLine = generateCmdLine(parameter)
                print "about to run command line: %s" % commandLine
                os.system(commandLine)
    else:
#        print cases
        for case in cases :
            for parameter in parameters:
                if parameter['Test ID'] == case:
                    commandLine = generateCmdLine(parameter)
                    print "about to run command line: %s" % commandLine
                    os.system(commandLine)
def checkResult():
    pass


if __name__ == '__main__':
    testSuites = checkArgv(sys.argv)[0]
    Cases = checkArgv(sys.argv)[1]
    config = open("defalt.cfg", 'r+')
    FFMPEG = "ffmpeg -y"
    if Cases :  # example c) d)
        runCases(testSuites[0], Cases)
    else :      # example a) b)
        for testSuite in testSuites :
            runCases(testSuite, [])

    config.close()
