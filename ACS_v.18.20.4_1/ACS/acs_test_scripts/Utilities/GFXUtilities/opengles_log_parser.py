# -*- coding: utf-8 -*-

import shlex
import xml.dom.minidom


class StatusCode:
    PASS = 'Pass'
    PASS_GLES = 'passed'
    PASSED = "PASSED"
    FAIL = 'Fail'
    FAIL_GLES = 'failed'
    FAILED = 'FAILED'
    QUALITY_WARNING = 'QualityWarning'
    COMPATIBILITY_WARNING = 'CompatibilityWarning'
    PENDING = 'Pending'
    NOT_SUPPORTED = 'NotSupported'
    RESOURCE_ERROR = 'ResourceError'
    INTERNAL_ERROR = 'InternalError'
    CRASH = 'Crash'
    TIMEOUT = 'Timeout'
    SUPPORTED_REPORTED = "SUPPORTED AND REPORTED"

    STATUS_CODES = [
        PASS,
        PASSED,
        PASS_GLES,
        FAIL,
        FAILED,
        FAIL_GLES,
        QUALITY_WARNING,
        COMPATIBILITY_WARNING,
        PENDING,
        NOT_SUPPORTED,
        RESOURCE_ERROR,
        INTERNAL_ERROR,
        CRASH,
        TIMEOUT,
        SUPPORTED_REPORTED
    ]
    STATUS_CODE_SET = set(STATUS_CODES)

    @staticmethod
    def isValid(code):
        return code in StatusCode.STATUS_CODE_SET


class TestCaseResult:
    def __init__(self, name, statusCode, statusDetails, log):
        self.name = name
        self.statusCode = statusCode
        self.statusDetails = statusDetails
        self.log = log

    def __str__(self):
        return "%s: %s (%s)" % (self.name, self.statusCode, self.statusDetails)


class ParseError(Exception):
    def __init__(self, filename, line, message):
        self.filename = filename
        self.line = line
        self.message = message

    def __str__(self):
        return "%s:%d: %s" % (self.filename, self.line, self.message)


def splitContainerLine(line):
    return shlex.split(line)


def getNodeText(node):
    rc = []
    for node in node.childNodes:
        if node.nodeType == node.TEXT_NODE:
            rc.append(node.data)
    return ''.join(rc)


class BatchResultParser:
    def __init__(self):
        pass

    def parseFile(self, filename):
        self.init(filename)

        f = open(filename, 'rb')
        for line in f:
            self.parseLine(line)
            self.curLine += 1
        f.close()
        if self.teststarted is True:
            self.testCaseResults.append(TestCaseResult(self.curCaseName, "SIGSEGV", None, None))

        return self.testCaseResults

    def init(self, filename):
        # Results
        self.sessionInfo = []
        self.testCaseResults = []

        # State
        self.curResultText = None
        self.curCaseName = None
        self.teststarted = False

        # Error context
        self.curLine = 1
        self.filename = filename

    def parseLine(self, line):
        if len(line) > 0 and line[0] == '#':
            self.parseContainerLine(line)
        elif self.curResultText != None:
            self.curResultText += line
        # else: just ignored

    def parseContainerLine(self, line):
        args = splitContainerLine(line)
        if args[0] == "#sessionInfo":
            if len(args) < 3:
                print args
                self.parseError("Invalid #sessionInfo")
            self.sessionInfo.append((args[1], ' '.join(args[2:])))
        elif args[0] == "#beginSession" or args[0] == "#endSession":
            pass  # \todo [pyry] Validate
        elif args[0] == "#beginTestCaseResult":
            self.teststarted = True
            if len(args) != 2 or self.curCaseName != None:
                self.parseError("Invalid #beginTestCaseResult")
            self.curCaseName = args[1]
            self.curResultText = ""
        elif args[0] == "#endTestCaseResult":
            if len(args) != 1 or self.curCaseName == None:
                self.parseError("Invalid #endTestCaseResult")
            self.parseTestCaseResult(self.curCaseName, self.curResultText)
            self.curCaseName = None
            self.curResultText = None
            self.teststarted = False
        elif args[0] == "#terminateTestCaseResult":
            if len(args) < 2 or self.curCaseName == None:
                self.parseError("Invalid #terminateTestCaseResult")
            statusCode = ' '.join(args[1:])
            statusDetails = statusCode

            if not StatusCode.isValid(statusCode):
                # Legacy format
                if statusCode == "Watchdog timeout occurred.":
                    statusCode = StatusCode.TIMEOUT
                else:
                    statusCode = StatusCode.CRASH

            # Do not try to parse at all since XML is likely broken
            self.testCaseResults.append(TestCaseResult(self.curCaseName, statusCode, statusDetails, self.curResultText))

            self.curCaseName = None
            self.curResultText = None
        else:
            # Assume this is result text
            if self.curResultText != None:
                self.curResultText += line

    def parseTestCaseResult(self, name, log):
        try:
            doc = xml.dom.minidom.parseString(log)
            resultItems = doc.getElementsByTagName('Result')
            if len(resultItems) != 1:
                self.parseError("Expected 1 <Result>, found %d" % len(resultItems))

            statusCode = resultItems[0].getAttributeNode('StatusCode').nodeValue
            statusDetails = getNodeText(resultItems[0])
        except Exception, e:
            statusCode = StatusCode.INTERNAL_ERROR
            statusDetails = "XML parsing failed: %s" % str(e)

        self.testCaseResults.append(TestCaseResult(name, statusCode, statusDetails, log))

    def parseError(self, message):
        raise ParseError(self.filename, self.curLine, message)
