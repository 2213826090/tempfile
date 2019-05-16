#!/system/bin/sh

##################################################################################
# loopAudioPlayback.sh
#
# Description: an Android shell script that uses a custom music player app to repeatedly
#       play music located in /sdcard/Music.  The Anniedale concurrency team used this
#       player because it worked better in concurrency scenarios than the one provided by
#       Google.
#       It also periodically checks that the player is still running, and fails the test
#       if the player has died before the specified test duration.
#       A pass.log is created in the current working directory if the player did not crash.
#       A fail.log is created if it ended prematurely.
#
# Usage:
#       loopAudioPlayback.sh <duration_in_minutes>
#
# Author: Jongyoon Choi
# Organization: PEG-SVE-DSV
# Date: 27 Jan 2014
##################################################################################

check_audio_playback() {
	retval=$(dumpsys media.player | grep bitrate | sed 's/.*flags(//g' | sed 's/).*//g')
	if [ $retval != "0x00006015" ]
	then
		#Give 3 seconds of wait time
		sleep 3

		#Re-check
		retval=$(dumpsys media.player | grep bitrate | sed 's/.*flags(//g' | sed 's/).*//g')
		if [ $retval != "0x00006015" ]
		then
			echo "*************ERROR OCCURS****************" | tee -a $dir/logfile.txt
			echo "Failed to run Play Music" | tee -a $dir/logfile.txt
			echo "Recent Time : $(date)" | tee -a $dir/logfile.txt
			echo "*************ERROR OCCURS****************" | tee -a $dir/logfile.txt
			mv $dir/logfile.txt $dir/fail.log

			echo "Killing pre-launched Audio Playback App" | tee -a $dir/logfile.txt
			am force-stop fv.concurrency.audio

			exit
		else
			echo "Audio Playback App is alive at $(date)" | tee -a $dir/logfile.txt
		fi
	else
		echo "Audio Playback App is alive at $(date)" | tee -a $dir/logfile.txt
	fi
}

dir=`dirname $0`
rm -r $dir/*.log $dir/*.txt AudioPlayback.log audioplayback.log 2> /dev/null

if [ -z $1 ]; then
	echo "	Audio Playback: Need test duration" | tee -a $dir/fail.log
        echo "	Usage: sh $dir/loopAudioPlayback.sh <duration in minutes>" | tee -a $dir/fail.log
	exit 1
else
        test_time=$(($1*60))
        echo "Audio Playback: Test Duration : $1 minute(s)" | tee -a $dir/logfile.txt
fi

echo "Audio Playback test start time : $(date)" | tee -a $dir/logfile.txt
starting=$(date -u +%s)

echo "Killing pre-launched Audio Playback App" | tee -a $dir/logfile.txt
am force-stop fv.concurrency.audio

echo "Start Audio Playback App" | tee -a $dir/logfile.txt
am start -n fv.concurrency.audio/.MainActivity

if [ $? != 0 ]
then
	echo "*************ERROR OCCURS****************" | tee -a $dir/logfile.txt
	echo "Failed to invoke AudioPlayback Intent" | tee -a $dir/logfile.txt
	echo "Recent Time : $(date)" | tee -a $dir/logfile.txt
	echo "*************ERROR OCCURS****************" | tee -a $dir/logfile.txt
	mv $dir/logfile.txt $dir/fail.log
	exit
fi

while true
do
	sleep 20

	check_audio_playback $(($starting+$test_time))

	passed=$(date -u +%s)
	tested_time=$(($passed-$starting))
	if [ $tested_time -gt $test_time ]
	then
		echo "*****************************************" | tee -a $dir/logfile.txt
		echo "Audio Playback passed" | tee -a $dir/logfile.txt
		echo "End Time : $(date)" | tee -a $dir/logfile.txt
		echo "*****************************************" | tee -a $dir/logfile.txt
		mv $dir/logfile.txt $dir/pass.log

		echo "Killing pre-launched Audio Playback App" | tee -a $dir/logfile.txt
		am force-stop fv.concurrency.audio

		exit
	fi
done
