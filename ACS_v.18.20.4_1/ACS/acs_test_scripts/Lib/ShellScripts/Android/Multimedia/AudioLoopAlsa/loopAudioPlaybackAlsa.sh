#!/system/bin/sh
dir=`dirname $0`
rm -r $dir/*.log $dir/*.txt AudioPlaybackAlsa.log audioplaybackalsa.log 2> /dev/null

if [ $# -lt 2 ]
then
	echo "$0: Audio Playback: Need test duration and Target Audio Device" | tee -a $dir/fail.log
    echo "$0: Usage: sh $dir/loopAudioPlaybackAlsa.sh <duration in minutes> <Audio Device:0 for AOB Speakers, 1 for headset>" | tee -a $dir/fail.log
	exit 1
else 
    test_time=$(($1*60))
    echo "$0: Audio Playback with Alsa: Test Duration : $1 minute(s)" | tee -a $dir/logfile.txt
fi

duration_minutes=$1
duration_seconds=`expr $duration_minutes \* 60`
base_time=`date +%s`
current_time=`date +%s`
end_time=`expr $base_time + $duration_seconds`

echo "$0: Current time in Seconds since beginning of the Time is $base_time" | tee -a $dir/logfile.txt
echo "$0: Test will complete when world moves past $end_time" | tee -a $dir/logfile.txt
	
while [ $current_time -le $end_time ]
do
    echo "$0: Starting Audio Playback "	| tee -a $dir/logfile.txt	
    if [ $2 -eq 0 ]		
    then
        export audio_out_device_cmd="sh spk_play.sh"
        sh spk_play.sh			
        sleep 30
        sh pb_end_seq_ihf.sh
    else
        export audio_out_device_cmd="sh hs_play.sh"
        sh hs_play.sh
        sleep 30
        sh pb_end_seq_hs.sh
    fi
    current_time=`date +%s`
done
    
echo "$0: Test completed" | tee -a $dir/logfile.txt
mv $dir/logfile.txt $dir/pass.log
exit 0