#!/bin/bash

oldpath=`pwd`
if [[ "z"$1 == "z" ]]
    then
         echo "pls input device id"
         exit
    else
         echo "deviceid=$1"         
         #cd /opt/noserunner/
         clientPath=../../../noserunner/client.config
         if [[ -f $clientPath ]]
         then
         b=$1
         sed -i "/deviceid/ s/=.*$/= $b/" $clientPath
         cp -f $clientPath ../../../noserunner/$b.config
         fi
fi
logPath=$oldpath/../../../noserunner/$b.camera.result.xml
if [[ -f $logPath ]]
then
rm $logPath
fi
../../../noserunner/runtests -s --plan-file ../../testplan/media/camera --cycle 1  --client-config ../../../noserunner/$b.config --xml-report-file $logPath  --timeout 2000
logPath2=../../../noserunner/
logPathFile=../../../noserunner/$b.camera.result_logcat.zip
if [[ -f $logPathFile ]]
then 
rm $logPathFile
fi
cd $logPath2
zip $b.camera.result_logcat.zip $b.camera.result_logcat.log
cd $oldpath


