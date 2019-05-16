#!/bin/bash

oldpath=`pwd`

if [[ "z"$1 == "z" ]]
    then
         echo "pls input device id"
         exit
    else
         echo "deviceid=$1"
         clientPath=../../../noserunner/client.config
         if [[ -f $clientPath ]]
         then
         b=$1
         sed -i "/deviceid/ s/=.*$/= $b/" $clientPath
         cp -f $clientPath ../../../noserunner/$b.config
         fi
fi
initPath=../../tests/tablet/bluetooth/init/
cd $initPath
python dut_init.py
cd $oldpath
logPath=$oldpath/../../../noserunner/$b.bluetooth.result.xml
if [[ -f $logPath ]]
then
rm $logPath
fi

export preferred_device=$1
../../../noserunner/runtests -s --plan-file ../../testplan/bluetooth/Communication_Bluetooth --cycle 1  --client-config ../../../noserunner/$b.config --xml-report-file $logPath  --timeout 2000
logPath2=../../../noserunner/
logPathFile=../../../noserunner/$b.bluetooth.result_logcat.zip
if [[ -f $logPathFile ]]
then 
rm $logPathFile
fi
cd $logPath2
zip $b.bluetooth.result_logcat.zip $b.bluetooth.result_logcat.log
cd $oldpath


