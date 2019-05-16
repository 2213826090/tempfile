#!/bin/bash
./time.sh
clear
LUCAS_LIB=$PWD/lib/
MSDK_LIB=$PWD/imports/mediasdk
MSDK_OLD_LIB=$PWD/imports/mediasdk_backup
MSDK_OLD_Version=`strings ${MSDK_OLD_LIB}/libmfxhw64.so | less | grep "mediasdk_product_version: "|cut -d " " -f 2|cut -c 10-`
DRIVER=`vainfo | grep "Driver version:" | awk '{print $(NF)}'`
CPU_Model=`cat /proc/cpuinfo | grep -m1 "model name" | cut -d' ' -f 6`
CPU_Freq=`cat /proc/cpuinfo | grep -m1 "MHz"|cut -d " " -f 3|cut -d "." -f 1`
GPU_Freq=`cat /sys/devices/pci0000:00/0000:00:02.0/drm/card0/gt_min_freq_mhz`
MSDK_Version=`strings /opt/intel/mediasdk/lib64/libmfxhw64.so | less | grep "mediasdk_product_version: "|cut -d " " -f 2|cut -c 10-`
Date=`date "+%Y_%m_%d-%H_%M_%S"`
KERNEL=`uname -r |cut -d "." -f 1-2`
TMP="temp"

function getPlatform()
{
        lspci | grep -iq 'Sky Lake' && PLATFORM=SKL && BASE=base_skl.csv && return
        lspci | grep -iq 'broadwell' && PLATFORM=BDW && BASE=base_bdw.csv && return
        lspci | grep -iq 'crystal' && PLATFORM=HSW && return
        PLATFORM=OTHER
}
getPlatform

OutputReport=Summary_${Date}_${PLATFORM}_${KERNEL}_${DRIVER}_${CPU_Model}_CPU_${CPU_Freq}_GPU_${GPU_Freq}_MSDK_${MSDK_Version}.csv

echo ------------------------------------------------------------------  >> $OutputReport
echo -e Test Start Time: $Date >> $OutputReport
echo -e Hostname:`hostname` >> $OutputReport
echo -e `more /proc/version`  >> $OutputReport
echo -e `grep -m 1 "model name" /proc/cpuinfo | cut -d: -f2 | sed -e 's/^ *//' | sed -e 's/$//'` >> $OutputReport
echo -e `grep MemTotal /proc/meminfo` >> $OutputReport
echo -e `vainfo | grep "Driver version:" ` >> $OutputReport
echo -e `strings  imports/mediasdk/libmfxhw64.so | less | grep "mediasdk_product_version: "` >> $OutputReport
echo -e GPU Frequency is `cat /sys/devices/pci0000:00/0000:00:02.0/drm/card0/gt_min_freq_mhz` >> $OutputReport
echo -e CPU Frequency is `cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_min_freq` >> $OutputReport
echo ------------------------------------------------------------------  >> $OutputReport
echo >> $OutputReport

printf "TEST_id,Sec,Frames,FPS,CPU_Usage,e,U,S,U+S\n"  >> $OutputReport

if [ $# -eq 0 ] ; then
        list=CVS_PPQ_LiteSuite_1to1_16.3_avc.csv
        list="$list CVS_PPQ_LiteSuite_1to1_16.3_avc_i.csv"
        list="$list CVS_PPQ_LiteSuite_1to1_16.3_avc_multiref.csv"
        list="$list CVS_PPQ_LiteSuite_1to1_16.3_mpeg2.csv"
        list="$list CVS_PPQ_LiteSuite_1to1_16.3_4k.csv"
        list="$list CVS_PPQ_LiteSuite_1N.csv"
        list="$list CVS_PPQ_LiteSuite_1N_16.3_mpeg2.csv"
        list="$list CVS_PPQ_LiteSuite_1N_16.3_4k.csv"
        list="$list CVS_PPQ_LiteSuite_NN.csv"
        list="$list CVS_PPQ_LiteSuite_NN_16.3_avc_i.csv"
        list="$list CVS_PPQ_LiteSuite_NN_16.3_mpeg2.csv"
        list="$list CVS_PPQ_LiteSuite_NN_16.3_4k.csv"
elif [ $# -eq 2 ] && [[ ! -f $2 ]] ; then
        list=$1
        CASE=$2
else
        list=$@
fi

if [ ! -d ${TMP} ]; then
        ln -s  /opt/local/temp $TMP
fi

function run_single_case(){
        csv=$1
        caseID=$2
        mkdir -p $TMP/${caseID}
        chmod -R 777 $TMP/${caseID}
        rm -rf $TMP/${caseID}/*
        LOG_OUT=$TMP/${caseID}/results_$Date.log
        touch $LOG_OUT
        secSum=0
        frameSum=0
        fpsSum=0
        CPUSum=0
        eSum=0
        USum=0
        SSum=0
        U_SSum=0

        for k in `seq 1 ${cycle}` ; do
                rm -rf $TMP/${caseID}/*
                echo Starting test ${caseID} run_${k} @ `date "+%Y_%m_%d-%H_%M_%S"` ....
                /usr/bin/time -f "%e---%U---%S---cpuusage%P" ./lucas -l info -o -r all --logfile $TMP/result.log -s ${csv} ${caseID} > $LOG_OUT 2>&1

                if [[ ! -z `cat $LOG_OUT | grep "imports/mediasdk/mfx_player"` ]] ; then
                        tempFile=`cat $LOG_OUT | grep "About to execute command:" | awk '{print $(NF)}'`
                        sec=`grep "Total time  " ${tempFile} | sed 's/Total time.*: \(.*\) sec.*/\1/'`
                        frame=`grep " number frames   "  ${tempFile} | awk '{print $(NF)}'`
                        fps=`grep "Total fps  " ${tempFile}  | awk '{print $(NF-1)}'`
                elif [[ ! -z `cat $LOG_OUT | grep "imports/mediasdk/sample_multi_transcode"` ]] ; then
                        sec=`grep "Common transcoding time is " ${LOG_OUT} | sed 's/Common transcoding time is //g' | sed 's/ sec//g'`
                        frame=`grep -m 1 "sec," ${LOG_OUT} | cut -d "," -f 2 |cut -d " " -f 2`
                        fps=`echo "scale=2;$frame/$sec" | bc`
                elif [[ ! -z `cat $LOG_OUT | grep "imports/mediasdk/mfx_transcoder"` ]] ; then
                        sec=`cat ${LOG_OUT} | grep -Po '(?<=Total execution time: ).*(?=s)'`
                        frame=`cat ${LOG_OUT} | grep "frames" | awk '{print $(NF)}' | sed -n '1p'`
                        fps=`grep "Total fps " ${LOG_OUT}|tail -n 1 | sed 's/Total fps.*: \(.*\) fps/\1/'`
                elif [[ ! -z `cat $LOG_OUT | grep "mv_encoder_adv -i "` ]] ; then
                        ms=`cat $LOG_OUT | grep -Po '(?<=Encode costs).*?(?= ms)'`
                        sec=`echo "scale=2;${ms}/1000" | bc`
                        frame=`cat $LOG_OUT | grep -Po '(?<=\-\-NumFrames ).*?(?= )'`
                        fps=`echo "scale=2;${frame}/${sec}" | bc `
                fi
                e=`grep "cpuusage" ${LOG_OUT} | awk -F'---' '{print $1}'`
                U=`grep "cpuusage" ${LOG_OUT} | awk -F'---' '{print $2}'`
                S=`grep "cpuusage" ${LOG_OUT} | awk -F'---' '{print $3}'`
                U_S=`echo "$U+$S" | bc`
                CPU=`echo "scale=1;${U_S}*100/${e}" | bc`%
                secSum=`echo "${secSum}+${sec}" | bc`
                frameSum=`echo "${frameSum}+${frame}" | bc`
                fpsSum=`echo "${fpsSum}+${fps}" | bc`
                CPUSum=`echo "scale=1;${CPUSum}+${CPU%%%}" | bc`
                eSum=`echo "${eSum}+${e}" | bc`
                USum=`echo "${USum}+${U}" | bc`
                SSum=`echo "${SSum}+${S}" | bc`
                U_SSum=`echo "${U_SSum}+${U_S}" | bc`
                echo "$caseID,$sec,$frame,$fps,$CPU,$e,$U,$S,$U_S"
                echo "$caseID,$sec,$frame,$fps,$CPU,$e,$U,$S,$U_S" >>  SKL_run_4_${DRIVER}.csv
        done
        secAvg=`echo "scale=2;${secSum}/${cycle}"   | bc`
        frameAvg=`echo "${frameSum}/${cycle}" | bc`
        fpsAvg=`echo "scale=3;${fpsSum}/${cycle}" | bc`
        CPUAvg=`echo "scale=1;${CPUSum}/${cycle}" | bc`%
        eAvg=`echo "scale=2;${eSum}/${cycle}" | bc`
        UAvg=`echo "scale=2;${USum}/${cycle}" | bc`
        SAvg=`echo "scale=2;${SSum}/${cycle}" | bc`
        U_SAvg=`echo "scale=2;${U_SSum}/${cycle}" | bc`
}


echo $list
#set -x
gpuHangCase=""
msdkRegression=""
export PATH=$PATH:$PWD/imports/tools
cycle=2
for i in $list ; do
        echo "cases in ${i}" >> $OutputReport
        if [[ ! -z ${CASE} ]] ; then
                sudo dmesg -C &>/dev/null
                run_single_case ${i} ${CASE}
                echo "$CASE,$secAvg,$frameAvg,$fpsAvg,$CPUAvg,$eAvg,$UAvg,$SAvg,$U_SAvg"  >>  $OutputReport
                echo "$CASE,$secAvg,$frameAvg,$fpsAvg,$CPUAvg,$eAvg,$UAvg,$SAvg,$U_SAvg"
                if [[ ! -z `dmesg | grep -i 'gpu'` ]] ; then
                        gpuHangCase="$gpuHangCase $CASE"
                fi
                echo End of test ${CASE} @ `date "+%Y_%m_%d-%H_%M_%S"` ...
                echo
        else
                for j in `cat ${i} | grep  "\.tpl.*\.par" |  cut -d, -f 1`  ; do
                        sudo dmesg -C &>/dev/null
                        echo Starting test ${j} @ `date "+%Y_%m_%d-%H_%M_%S"` ....
                        ./lucas -s ${i} ${j} &>/dev/null
                        exitState=$?
                        if [ ${exitState} -ne 0 ] ; then
                                echo fail to case ${j} with lucas
                                echo "${j},0,0,0,0,0,0,0,0"  >>  $OutputReport
                                echo "${j},0,0,0,0,0,0,0,0"
                        else
                                echo successfully run ${j} with lucas, will run ${cycle} times to get an avarage data
                                run_single_case ${i} ${j}
                                echo "${j},$secAvg,$frameAvg,$fpsAvg,$CPUAvg,$eAvg,$UAvg,$SAvg,$U_SAvg"  >>  $OutputReport
                                echo "${j},$secAvg,$frameAvg,$fpsAvg,$CPUAvg,$eAvg,$UAvg,$SAvg,$U_SAvg"
                        fi
                        if [[ ! -z `dmesg | grep -i 'gpu'` ]] ; then
                                gpuHangCase="$gpuHangCase $j"
                        fi
                        echo End of test ${j} @ `date "+%Y_%m_%d-%H_%M_%S"` ...
                        echo
                        sleep 2
                done
        fi
done

echo
echo GPU hang case: $gpuHangCase >> $OutputReport
echo End of $OutputReport
echo -e Test End Time: `date "+%Y_%m_%d-%H_%M_%S"` >> $OutputReport
exit 0
