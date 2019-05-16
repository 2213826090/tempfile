#!/bin/bash

oldpath=`pwd`
if [[ "z"$1 == "z" ]]
    then
         echo "pls input device id"
         exit
    else
         echo "deviceid=$1"     
         b=$1       

fi
logPath=$oldpath/../../../noserunner/$b.googlefast.result.xml
if [[ -f $logPath ]]
then
rm $logPath
fi
../../../noserunner/runtests -s --plan-file ../../testplan/google_fast/google_fast --cycle 1  --client-config ../../../noserunner/$b.config --xml-report-file $logPath  --timeout 2000
logPath2=../../../noserunner/
logPathFile=../../../noserunner/$b.googlefast.result_logcat.zip
if [[ -f $logPathFile ]]
then 
rm $logPathFile
fi
cd $logPath2
zip $b.googlefast.result_logcat.zip $b.googlefast.result_logcat.log
cd $oldpath


