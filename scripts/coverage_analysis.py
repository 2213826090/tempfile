import re
import os
import sys
import csv
import xlwt
import time
import shutil

resultXls = 'C:\\BULLSEYE\\doc\\TF_status_xls\\cmp_result_9w11_withoutddi.xls'
resultTxt = 'C:\\BULLSEYE\\doc\\TF_status_txt\\TF_status_withoutddi_codec.txt'
inputDir = sys.argv[1]
testSuites = os.listdir(inputDir)
scenariosDir = 'C:\\BULLSEYE\\doc\\test_suites'

def cleanDir():
    try:
        os.remove("run_result.txt")
        for file in os.listdir(os.getcwd()):
            if file.endswith(".cov"):
                os.remove(file)
    except:
	    pass

def resultAnalysis(testSuite):
    suiteName = testSuite
    scenariosfile = os.path.join(scenariosDir, suiteName + '.csv')
    f_caseSummaryCsv = open(scenariosfile, 'r')
    f_caseSummaryTxt = open(resultTxt, 'r')
    if os.path.exists(resultXls):
        os.remove(resultXls)
    f_caseCmp = open(resultXls, 'a')

    lineListTxt = f_caseSummaryTxt.readlines()
    lineListCsv = csv.reader(f_caseSummaryCsv)

    column_num = 1
    num = 1
    caseList_len = 0

    f = xlwt.Workbook()
    sheet1 = f.add_sheet('case_dif', cell_overwrite_ok=True)
    pattern = xlwt.Pattern()
    pattern.pattern = xlwt.Pattern.SOLID_PATTERN
    pattern.pattern_fore_colour = 5
    style = xlwt.XFStyle()
    style.pattern = pattern

    if os.path.exists(resultXls):
        for line in lineListCsv:
            if line[0] == 'Test ID':
                for i in range(0, len(line)):
                    sheet1.write(0, i, line[i])

    for eachLine in lineListTxt:
        if re.search('\*', eachLine):
            caseColumnList = [] 
            caseParameterList = []
            caseList = re.split('\]|=> \[|\* |\=>  |\n|=PASSED|=FAILED|=NOTRUN', eachLine)
            while '' in caseList:
                caseList.remove('')
            caseList_len =len(caseList)
            sheet1.write(column_num, 0, '#' + str(num))
            num+=1
            for i in caseList:
                for each in lineListCsv: 
                    if i == each[0] + '.cov':
                        caseParameterList.append(each)
                        break
						
            for i in range(0, len(caseParameterList[0])):
                caseColumn = []
                for j in range(0, caseList_len):
                    caseColumn.append(caseParameterList[j][i])
                caseColumnList.append(caseColumn)
            print caseColumnList
            for i in range(0, len(caseParameterList[0])):
                count = 1
                for k in range(0, caseList_len - 1):
                    if caseColumnList[i][k] == caseColumnList[i][k + 1]:
                        count+=1
                if caseList_len == count:
                    for j in range(0, caseList_len):
                        sheet1.write(j + column_num + 1, i, caseParameterList[j][i])
                else:
                    if (i == 0) | (case_parameter_h[i] == 'ReferenceFile') | (case_parameter_h[i] == 'ReferenceWidth') | (case_parameter_h[i] == 'ReferenceHeight'):
                        for j in range(0, caseList_len):
                            sheet1.write(j + column_num + 1, i, caseParameterList[j][i])
                    else:
                        for j in range(0, caseList_len):
                            sheet1.write(j + column_num + 1, i, caseParameterList[j][i], style)
                            
            column_num = column_num + caseList_len + 1
        f.save(resultXls)

    f_caseSummaryCsv.close()
    f_caseSummaryTxt.close()
    f_caseCmp.close()
		
for testSuite in testSuites:
    cleanDir()
    testSuiteDir = os.path.join(os.getcwd(), inputDir, testSuite)
    for file in os.listdir(testSuiteDir):
        if os.path.isfile(os.path.join(testSuiteDir, file)):
            print os.path.join(testSuiteDir, file)
            shutil.copy(os.path.join(testSuiteDir, file), os.getcwd())
#    os.system("perl codec_coverage_info_many_witoutddi.pl")
    finishTime = time.strftime('%Y_%m_%d_%H_%M_%S',time.localtime(time.time()))
    dstDir = os.path.join(testSuiteDir, 'result' + '_' + finishTime)
    os.mkdir(dstDir)
    try:
        resultAnalysis(testSuite)
    except:
	    pass
    try:
        shutil.copy(resultTxt, dstDir)
        shutil.copy(resultXls, dstDir)
#        shutil.copytree(resultDir, dstDir, ignore=None)
    except:
        pass
    cleanDir() 
