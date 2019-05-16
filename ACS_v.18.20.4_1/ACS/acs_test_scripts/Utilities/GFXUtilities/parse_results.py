#!/usr/bin/python
import os
import time
import sys
from subprocess import *

def exec_command_str(cmd):
    my_proc = Popen(cmd, shell=True, stdin=None, stdout=PIPE, stderr=PIPE)
    (stdout, stderr) = my_proc.communicate()
    return stdout


def parse_file_gles1(file_name, test_suite):
    fisier=open(file_name).read()

    if fisier.find('failed')!=-1:
        failed.write('Tests failed from the '+ test_suite+' suite with the setup:\n')
        fisier=open(file_name,'r')
        linii=fisier.readlines()
        setup=0
        i=0
        for line in linii:
            if line.find('Setup Report.')!=-1 and setup==0:
                failed.write(linii[i])
                j=i+1
                while linii[j]!='\n': 
                    failed.write(linii[j])
                    j=j+1
                setup=setup+1
    
            if line.find('Config Report.')!=-1:
                linii_left=linii[i+2:len(linii)]
                left=linii[i+1]
                a=0
                for rows in linii_left:
                    if rows.find('Config Report.')!=-1 or rows.find('**************')!=-1:
                        break
                    left=left+rows
                if left.find('failed')!=-1:
                    failed.write('\n')            
                    failed.write(linii[i])
                    j=i+1
                    while linii[j]!='\n': 
                        failed.write(linii[j])
                        j=j+1
                    failed.write('\n')
                
            if line.find('failed')!=-1 and line.find('#')==-1 and line.find('(')==-1 and line.find(':')==-1:
                failed.write('\t'+line)
            i=i+1
        failed.write('\n')
        failed.write('****************************************************\n')
        failed.write('\n')

    fisier=open(file_name,'r')
    lines=fisier.read()
    setups=lines.split('Setup Report.\n')
    feature_ind=0
    for i in range(1,len(setups)):
        parts=setups[i].split('\n\n')
        if feature_ind==0:
            feature=parts[0].replace('    ','')
            feature=feature.replace('.\n','_')
            feature=feature.replace('.',',')
            feature=feature.replace('Verbose level = 1_','')
            feature=feature.replace('Random number seed','Random seed')
            feature=feature.replace(' = ','=')
            feature='"Khronos_Gles1\\'+test_suite+'('+feature[0:-1]+')_ConfigID_'
            feature_ind=1
        config_id=parts[1].replace('Config Report.\n    Config ID = ','')
        config_nr=int(config_id[0:config_id.find('.')])
        config_id=config_id.replace('.\n    ','_')
        config_id=config_id.replace('.',',')
        config_id=config_id.replace(' ','')
        if config_nr<10:
            config_id='0'+config_id[0:-1]+'",,'
        else:
            config_id=config_id[0:-1]+'",,'
        tests=parts[2].split('\n')
        j=0
        for test in tests:
            if test.find('test')==-1:
                continue
            if test.find('passed')!=-1:
                test_case=test.split(' test ')[0]
                all_results.write(feature+config_id+test_case+',1,0,0,,,,,,\n')
            else:
                if j+1<len(tests):                
                    if test.find('failed')!=-1 and tests[j+1].find('not supported')==-1:
                        test_case=test.split(' test ')[0]
                        all_results.write(feature+config_id+test_case+',0,1,0,,,,,,\n')
                    else:
                        test_case=test.split(' test ')[0]
                        all_results.write(feature+config_id+test_case+',0,0,1,,,,,,\n')
                else:
                    test_case=test.split(' test ')[0]
                    all_results.write(feature+config_id+test_case+',0,1,0,,,,,,\n')                    
            j=j+1



def parse_file_gles2(file_name,test_suite):
    fisier=open(file_name).read()
    if fisier.find('#+')!=-1:
        failed.write('\n')
        failed.write('################################################################################\n')
        failed.write('\n')
        failed.write('Tests failed from the GLES2 suite with the setup:\n')
        fisier=open(file_name,'r')
        linii=fisier.readlines()
        for line in linii:
            if line.find('arg: ')!=-1:
                failed.write(line)
                failed.write('\n')
                break
        i=0
        for line in linii:
            if line.find('ConfigID:')!=-1 and linii[i+1].find('gtf_version')!=-1:
                failed.write('****************************************************\n')
                failed.write('\n')
                failed.write(linii[i])
                j=i+1
                while linii[j].find('gles2: Start')==-1 and linii[j].find('Regression FAILED ')==-1: 
                    failed.write(linii[j])
                    j=j+1
                failed.write('\n\n')

            
            if line.find(': Start')!=-1 and line.find('GTF:')==-1:
                start_test=i
                j=i
                for row in linii[i+1:]:
                    j=j+1                    
                    if row.find('failure = ')!=-1:
                        if row.find('#')!=-1:
                            failed.writelines(linii[i:j+1])
                            failed.write('\n')
                            failed.write('\n')
                        break

            i=i+1
            
    fisier=open(file_name,'r')
    text=fisier.read()

    all_results=open('qa_gles2_'+data+'_'+test_suite+'.csv','w')
    all_results.write('Feature,Case Id,Test Case,Pass,Fail,N/A,Comment,Measurement Name,Value,Unit,Target,Failure\n')

    feature='\"Khronos_Gles2\GTF_'+test_suite+'_'
    configs=text.split('ConfigID: ')
    for i in range(1,len(configs)):
        if configs[i].find('Sample Buffers: 1 Samples:')==-1:
            config=configs[i].split('\n')[0]
            config=config.replace(' RGBA ','_RGBA ')
            config=config.replace('( ','(')
            config=config.replace(', ',',')
            config=config.replace(') ',')_')
            config=config.replace(': ',':')
            config=config.replace(' Stencil','_Stencil')
            if i<10:
                config='ConfigID_0'+config+'",,'            
            else:
                config='ConfigID_'+config+'",,'
            tests=configs[i].split('gles2: ')
            test=tests[1].split(': Start\n')
            for j in range(1,len(test)):
                lines=test[j].split('\n')
                for line in lines:
                    if line.find('total = ')!=-1:
                        test_case=line[1:line.find(':')]
                        nr_of_tests=int(line[line.find('total = ')+8:line.find(',')])
                        if nr_of_tests==1:
                            if line.find('#')==-1:
                                all_results.write(feature+config+test_case+',1,0,0,,,,,,\n')
                            else:
                                all_results.write(feature+config+test_case+',0,1,0,,,,,,\n')
                        else:
                            for k in range(1,nr_of_tests+1):
                                last=line.split(', ')[-1]
                                if line.find('#'+str(k)+',')==-1 and last!=('#'+str(k)):
                                    if nr_of_tests<10:
                                        all_results.write(feature+config+test_case+'_'+str(k)+',1,0,0,,,,,,\n')
                                    if nr_of_tests>9 and nr_of_tests<100:
                                        if k<10:
                                            all_results.write(feature+config+test_case+'_0'+str(k)+',1,0,0,,,,,,\n')
                                        else:
                                            all_results.write(feature+config+test_case+'_'+str(k)+',1,0,0,,,,,,\n')
                                    if nr_of_tests>99:
                                        if k<10:
                                            all_results.write(feature+config+test_case+'_00'+str(k)+',1,0,0,,,,,,\n')
                                        else:
                                            if k<100:
                                                all_results.write(feature+config+test_case+'_0'+str(k)+',1,0,0,,,,,,\n')
                                            else:
                                                all_results.write(feature+config+test_case+'_'+str(k)+',1,0,0,,,,,,\n')
                                else:
                                    if nr_of_tests<10:
                                        all_results.write(feature+config+test_case+'_'+str(k)+',0,1,0,,,,,,\n')
                                    if nr_of_tests>9 and nr_of_tests<100:
                                        if k<10:
                                            all_results.write(feature+config+test_case+'_0'+str(k)+',0,1,0,,,,,,\n')
                                        else:
                                            all_results.write(feature+config+test_case+'_'+str(k)+',0,1,0,,,,,,\n')
                                    if nr_of_tests>99:
                                        if k<10:
                                            all_results.write(feature+config+test_case+'_00'+str(k)+',0,1,0,,,,,,\n')
                                        else:
                                            if k<100:
                                                all_results.write(feature+config+test_case+'_0'+str(k)+',0,1,0,,,,,,\n')
                                            else:
                                                all_results.write(feature+config+test_case+'_'+str(k)+',0,1,0,,,,,,\n')

    all_results.close()


def parse_file_gles3(file_name,test_suite):
    fisier=open(file_name).read()
    if fisier.find('Conformance FAILED')!=-1:
        failed.write('\n')
        failed.write('################################################################################\n')
        failed.write('\n')
        failed.write('Tests failed from the GLES3 suite with the setup:\n')
        fisier=open(file_name,'r')
        linii=fisier.readlines()
        #determine the argument passed to run the tests
        for line in linii:
            if line.find('arg: ')!=-1:
                failed.write(line)
                failed.write('\n')
                break
        i=0         #is the index for the lines
        for line in linii:
            #determine the ConfigID
            if line.find('ConfigID:')!=-1 and linii[i+1].find('gtf_version')!=-1:
                failed.write('****************************************************\n')
                failed.write('\n')
                failed.write(linii[i])
                j=i+1
                while linii[j].find('gles3: Start')==-1 and linii[j].find('Conformance FAILED ')==-1: 
                    failed.write(linii[j])
                    j=j+1
                failed.write('\n\n')


            if line.find(': Start')!=-1 and line.find('gles3: ')==-1:
                start_test=i
                j=i
                for row in linii[i+1:]:
                    j=j+1                    
                    if row.find('failure = ')!=-1:
                        if row.find('#')!=-1:
                            failed.writelines(linii[i:j+1])
                            failed.write('\n')
                            failed.write('\n')
                        break

            i=i+1

    fisier=open(file_name,'r')
    text=fisier.read()

    all_results=open('qa_gles3_'+data+'_'+test_suite+'.csv','w')
    all_results.write('Feature,Case Id,Test Case,Pass,Fail,N/A,Comment,Measurement Name,Value,Unit,Target,Failure\n')

    feature='\"Khronos_Gles3\GTF_'+test_suite+'_'
    configs=text.split('ConfigID: ')
    for i in range(1,len(configs)):
        if configs[i].find('Sample Buffers: 1 Samples:')==-1:
            config=configs[i].split('\n')[0]
            config=config.replace(' RGBA ','_RGBA ')
            config=config.replace('( ','(')
            config=config.replace(', ',',')
            config=config.replace(') ',')_')
            config=config.replace(': ',':')
            config=config.replace(' Stencil','_Stencil')
            if i<10:
                config='ConfigID_0'+config+'",,'            
            else:
                config='ConfigID_'+config+'",,'
            tests=configs[i].split('gles3: ')
            test=tests[1].split(': Start\n')
            for j in range(1,len(test)):
                lines=test[j].split('\n')
                for line in lines:
                    if line.find('total = ')!=-1:
                        test_case=line[1:line.find(':')]
                        nr_of_tests=int(line[line.find('total = ')+8:line.find(',')])
                        if nr_of_tests==1:
                            if line.find('#')==-1:
                                all_results.write(feature+config+test_case+',1,0,0,,,,,,\n')
                            else:
                                all_results.write(feature+config+test_case+',0,1,0,,,,,,\n')
                        else:
                            for k in range(1,nr_of_tests+1):
                                last=line.split(', ')[-1]
                                if line.find('#'+str(k)+',')==-1 and last!=('#'+str(k)):
                                    if nr_of_tests<10:
                                        all_results.write(feature+config+test_case+'_'+str(k)+',1,0,0,,,,,,\n')
                                    if nr_of_tests>9 and nr_of_tests<100:
                                        if k<10:
                                            all_results.write(feature+config+test_case+'_0'+str(k)+',1,0,0,,,,,,\n')
                                        else:
                                            all_results.write(feature+config+test_case+'_'+str(k)+',1,0,0,,,,,,\n')
                                    if nr_of_tests>99:
                                        if k<10:
                                            all_results.write(feature+config+test_case+'_00'+str(k)+',1,0,0,,,,,,\n')
                                        else:
                                            if k<100:
                                                all_results.write(feature+config+test_case+'_0'+str(k)+',1,0,0,,,,,,\n')
                                            else:
                                                all_results.write(feature+config+test_case+'_'+str(k)+',1,0,0,,,,,,\n')
                                else:
                                    if nr_of_tests<10:
                                        all_results.write(feature+config+test_case+'_'+str(k)+',0,1,0,,,,,,\n')
                                    if nr_of_tests>9 and nr_of_tests<100:
                                        if k<10:
                                            all_results.write(feature+config+test_case+'_0'+str(k)+',0,1,0,,,,,,\n')
                                        else:
                                            all_results.write(feature+config+test_case+'_'+str(k)+',0,1,0,,,,,,\n')
                                    if nr_of_tests>99:
                                        if k<10:
                                            all_results.write(feature+config+test_case+'_00'+str(k)+',0,1,0,,,,,,\n')
                                        else:
                                            if k<100:
                                                all_results.write(feature+config+test_case+'_0'+str(k)+',0,1,0,,,,,,\n')
                                            else:
                                                all_results.write(feature+config+test_case+'_'+str(k)+',0,1,0,,,,,,\n')

    all_results.close()

if sys.argv[1]=='gles1':
    data=exec_command_str('echo `date`')
    dates=data.split(' ')
    data=dates[5][0:-1]+'-'+dates[1]+'-'+dates[2]
    
    all_results=open('qa_gles1_'+data+'.csv','w')
    all_results.write('Feature,Case Id,Test Case,Pass,Fail,N/A,Comment,Measurement Name,Value,Unit,Target,Failure\n')
    
    failed=open('failures_gles1_'+data+'.txt','w')
    failed.write('Execution date: '+data+'\n')

    fisier=open('log_covegl.txt').read()
    if fisier.find('passed')!=-1:
        all_results.write('"Khronos_Gles1\Covegl",,Covegl,1,0,0,,,,,,\n')
    else:
        if fisier.find('failed')!=-1:
            failed.write('covegl failed\n')
            all_results.write('"Khronos_Gles1\Covegl",,Covegl,0,1,0,,,,,,\n')
        else:
            all_results.write('"Khronos_Gles1\Covegl",,Covegl,0,0,1,,,,,,\n')
    
    fisier=open('log_covgl.txt').read()
    if fisier.find('passed')!=-1:
        all_results.write('"Khronos_Gles1\Covgl",,Covgl,1,0,0,,,,,,\n')
    else:
        if fisier.find('failed')!=-1:
            failed.write('covgl failed\n')
            all_results.write('"Khronos_Gles1\Covgl",,Covgl,0,1,0,,,,,,\n')
        else:
            all_results.write('"Khronos_Gles1\Covgl",,Covgl,0,0,1,,,,,,\n')

    fisier=open('log_primtest.txt').read()
    if fisier.find('passed')!=-1:
        all_results.write('"Khronos_Gles1\Primtest",,Primtest,1,0,0,,,,,,\n')
    else:
        if fisier.find('failed')!=-1:
            failed.write('covgl failed\n')
            all_results.write('"Khronos_Gles1\Primtest",,Primtest,0,1,0,,,,,,\n')
        else:
            all_results.write('"Khronos_Gles1\Primtest",,Primtest,0,0,1,,,,,,\n')

    parse_file_gles1('log_conform_mustpass.txt','mustpass')
    parse_file_gles1('log_conform_TESTLIST_p_0.txt','Testlist')
    parse_file_gles1('log_conform_TESTLIST_p_1.txt','Testlist')
    parse_file_gles1('log_conform_TESTLIST_p_2.txt','Testlist')
    parse_file_gles1('log_conform_TESTLIST_p_3.txt','Testlist')
    failed.close()


if sys.argv[1]=='gles2':

    data=exec_command_str('echo `date`')
    dates=data.split(' ')
    data=dates[5][0:-1]+'-'+dates[1]+'-'+dates[2]

    failed=open('failures_gles2_'+data+'.txt','w')
    failed.write('Execution date: '+data+'\n')


    parse_file_gles2('log_width64.txt','w_64')
    parse_file_gles2('log_width113.txt','w_113')
    
    if sys.argv[2]=='snb':
        parse_file_gles2('log_width1366.txt','w_1366')
    if sys.argv[2]=='ivb':
        parse_file_gles2('log_width1600.txt','w_1600')
    if sys.argv[2]=='hsw' or sys.argv[2]=='acer':
        parse_file_gles2('log_width1920.txt','w_1920')
    failed.close()

if sys.argv[1]=="gles3":
    data=exec_command_str('echo `date`')
    dates=data.split(' ')
    data=dates[5][0:-1]+'-'+dates[1]+'-'+dates[2]

    failed=open('failures_gles3_'+data+'.txt','w')
    failed.write('Execution date: '+data+'\n')

    parse_file_gles3('log_width64.txt','w_64')
    parse_file_gles3('log_width113.txt','w_113')

    if sys.argv[2]=='ivb':
        parse_file_gles3('log_width1600.txt','w_1600')
        parse_file_gles3('log_width64_height_max.txt','h_900')
    if sys.argv[2]=='hsw' or sys.argv[2]=='acer':
        parse_file_gles3('log_width64_height_max.txt','h_1080')
        parse_file_gles3('log_width1920.txt','w_1920')
    failed.close()


if sys.argv[1]=='create_dir':
    data=exec_command_str('echo `date`')
    dates=data.split(' ')

    data=dates[5][0:-1]+'-'+dates[1]+'-'+dates[2]
    
    os.system('mkdir khronos')
    os.system('rm -rf khronos/'+data)
    os.system('mkdir khronos/'+data)

    director='khronos/'+data+'/'
    director_name='khronos/'+data

    os.system('mv -fu gles1/all_gles1/ '+director)
    os.system('mv -fu gles2/all_gles2/ '+director)
    os.system('mv -fu gles3/all_gles3/ '+director)
    os.system('mv -fu '+director+'all_gles1/failures_gles1_'+data+'.txt '+director)
    os.system('mv -fu '+director+'all_gles2/failures_gles2_'+data+'.txt '+director)
    os.system('mv -fu '+director+'all_gles3/failures_gles3_'+data+'.txt '+director)
    """
    #This part concatenates all the csv results in a single file

    os.system('mv '+director+'all_gles1/qa_gles1_'+data+'.txt '+director)
    os.system('mv '+director+'all_gles2/qa_gles2_'+data+'.txt '+director)
    report=open(director+'qa_report_'+data+'.csv','w')
    qa1=open(director+'qa_gles1_'+data+'.txt','r')
    g1List = qa1.readlines()
    report.writelines(g1List)
    qa2=open(director+'qa_gles2_'+data+'.txt','r')
    g2List = qa2.readlines()
    g2List=g2List[1:]
    report.writelines(g2List)
    report.close()
    qa1.close()
    qa2.close()
    os.system('mv '+director+'qa_report_'+data+'.csv khronos/')
    os.system('rm '+director+'qa_gles1_'+data+'.txt')
    os.system('rm '+director+'qa_gles2_'+data+'.txt ')
    """
    os.system('tar -zcvf '+director_name+'.tar.gz '+director)

