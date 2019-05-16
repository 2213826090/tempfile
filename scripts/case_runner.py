inputUrl = [["gfx-media-assets-fm/PROD/content/AVC", "Internal", "2.4"], ["gfx-media-assets-fm/PROD/content/HEVC_8bit", "Internal", "1.6"],
    ["gfx-media-assets-fm/PROD/content/MPEG2", "Internal", "2"], ["gfx-media-assets-fm/PROD/content/RAW", "AYUV", "6"],
    ["gfx-media-assets-fm/PROD/content/RAW", "UYVY", "3"], ["gfx-media-assets-fm/PROD/content/RAW", "Y410", "6"],
    ["gfx-media-assets-fm/PROD/content/RAW", "YU12", "1.3"], ["gfx-media-assets-fm/PROD/content/RAW", "YUV", "56"],
    ["gfx-media-assets-fm/PROD/content/RAW", "YUY2", "8"], ["gfx-media-assets-fm/PROD/content/RAW", "ARGB", "9"],
    ["gfx-media-assets-fm/PROD/content/RAW", "AYUV", "4"], ["gfx-media-assets-fm/PROD/content/RAW", "Y410", "4"],
    ["gfx-media-assets-fm/PROD/content/RAW", "YUV", "81"], ["gfx-sandbox-fm/bmunshi", "AVBR_RR", "1"], ["gfx-media-assets-fm/PROD/content/RAW", "YUV", "107"]]
def downloadInput(File):
    inputFile = File
    for url in inputUrl:
        state = os.system('gta-asset.exe pull ' + url[0] + ' ' + url[1] + ' ' + url[2] + ' --pattern ' + inputFile + ' --dest-dir ./')
        if state == 0:
            return 0
    return 1

def searchFile(File):
    File = File
    inputFile = "//10.239.141.231//genxfsim-share//ExtDisk//EncoderContent//" + File
    print File, inputFile
    try:
        shutil.copyfile(inputFile, File)
    except:
        print "fail to find input file %s on sever." % inputFile
        
	
for testSuite in testSuites:
    outDir = os.path.join(summaryDir, os.path.splitext(testSuite)[0].split(os.path.sep)[-1])
    os.mkdir(outDir)
    run_result = []
#    with open(os.path.join(scenariosPath, testSuite), 'r') as fl:
    with open(testSuite, 'r') as fl:
        title = fl.readline()
        indexInput = title.split(',').index("File")
        indexID = title.split(',').index("Test ID")
        for line in fl:
            if re.search('\.tpl', line) and re.search('\.par', line):
                if not re.search('\,b\,', line):
                    caseID = line.split(',')[indexID]
                    inputFile = line.split(',')[indexInput]
#                    downloadInput(inputFile)
                    searchFile(inputFile)
                    shutil.copyfile(igd9with11_test_cov_path, eachCovPath)
                    try:
                        os.system('lucas.exe -o -r all -l info --scenario-safe-mode --logfile 1.txt -s' + ' ' + testSuite + ' ' + caseID)
                    except:
                        pass
                    with open('1.txt', 'r') as fc:
                        content = fc.readlines()
                        for each_con in content:
                            if re.match(r'.*(END).*(PASSED)', each_con):
                                run_result.append(caseID + '.cov' + '=PASSED' + ',')
                                break
                            elif re.match(r'.*(END).*(FAILED)', each_con):
                                run_result.append(caseID + '.cov' + '=FAILED' + ',')
                                break
                            elif re.match(r'.*(END).*(NOT.*RUN)', each_con):
                                run_result.append(caseID + '.cov' + '=NOTRUN' + ',')
                                break
                    savedCovPath = testSuite.split("\\")[1] + '_' + caseID + '.cov'
                    print outDir, savedCovPath
                    shutil.move(eachCovPath, os.path.join(outDir, savedCovPath))
                    try:
                        os.remove(inputFile)
                    except:
                        pass
                    os.remove('1.txt')
    with open(os.path.join(outDir, 'run_result.txt'), 'w') as fr:
        fr.write(str(run_result) + '\n')
