#!/bin/bash

oldpath=`pwd`

cp -f /opt/oat-l/tests/tablet/bat/init/wifiap.conf ../../tests/tablet/media/init/wifiap.conf
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
#cd /opt/oat-l/tests/tablet/bat/init/
initPath=../../tests/tablet/media/init/
cd $initPath
timeout 600 python dut_init.py -s $b
cd $oldpath
logPath=$oldpath/../../../noserunner/$b.media.result.xml
if [[ -f $logPath ]]
then
rm $logPath
fi
../../../noserunner/runtests -s --plan-file ../../testplan/media/media --cycle 1  --client-config ../../../noserunner/$b.config --xml-report-file $logPath  --timeout 2000
logPath2=../../../noserunner/
logPathFile=../../../noserunner/$b.media.result_logcat.zip
if [[ -f $logPathFile ]]
then 
rm $logPathFile
fi
cd $logPath2
zip $b.media.result_logcat.zip $b.media.result_logcat.log
cd $oldpath


