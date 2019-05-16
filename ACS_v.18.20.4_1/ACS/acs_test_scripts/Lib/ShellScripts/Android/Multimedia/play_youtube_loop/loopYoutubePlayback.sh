#!/system/bin/sh

# Script Parameters:
#   1: Run time in minutes
#   2: Video URL
#   3: Minimum play time in seconds
#   4: Maximum play time in seconds (including buffer time)
#   5: Capture screen recordings. 0=No, 1=Yes
#
# This test won't do extensive verification on the flags. It will log them for
#  a parent script to validate.

# Default values for options passed on the command line
ARG_RUN_MINUTES=5
ARG_VIDEO_URL="http://www.youtube.com/watch?v=r6sgrlaUOWM&hd=0"
ARG_MIN_PLAY_TIME_SECONDS=163
ARG_MAX_PLAY_TIME_SECONDS=600
ARG_CAPTURE_SCREEN_RECORD=0

# Static test variables
LOG_FILE=youtube_test.log
PASS_LOG_FILE=pass.log
FAIL_LOG_FILE=fail.log
FLAG_LOG_FILE=awesome_player_flags.log
TEMP_FLAG_LOG_FILE=current_loop_awesome_player_flags.log
YOUTUBE_START_CMD="am start -a android.intent.action.VIEW -d $ARG_VIDEO_URL"
YOUTUBE_STOP_CMD="am force-stop com.google.android.youtube"
FLAG_RETRIEVAL_CMD="dumpsys media.player | grep URI | sed 's/.*flags(//g' | sed 's/).*//g'"
HEARTBEAT_INTERVAL_SECONDS=10
HEARTBEAT_CHAR="."
MAX_START_WAIT_TIME=$(($ARG_MIN_PLAY_TIME_SECONDS-5))
SLEEP_TIME_SECONDS_BETWEEN_LOOPS=5
CAPTURE_SCREENSHOT_DURING_DEBUG=false
CAPTURE_SCREEN_RECORDING=false
SCREEN_RECORDING_BITRATE=4000000

# Variables used in the script
min_start_time=999999
max_start_time=0
total_start_time=0
min_play_time=$ARG_MAX_PLAY_TIME_SECONDS
max_play_time=0
total_play_time=0
test_passed=1
screen_recording_file=""
# Max recording time the screenrecording command will allow is 180 seconds.
screen_recording_time=180
debug_data_collected=false
# Failure Counts
fail_count_app_stop=0
fail_count_app_start=0
fail_count_exceeded_max_play_time=0
fail_count_flags_missing=0
fail_count_player_didnt_start=0
fail_count_player_finished_early=0

fail_test() {
    test_passed=0
    collect_debug_data $1
}
collect_debug_data() {
    echo "Collecting debug data." >> $LOG_FILE
    debug_data_collected=true
    if [ "$CAPTURE_SCREENSHOT_DURING_DEBUG" = true ]
    then
        # Take a screenshot and then dump the dumpsys output.
        screenshot_filename="screenshot_`date +%s`.png"
        echo "  Saving screenshot to $screenshot_filename." >> $LOG_FILE
        screencap -p $screenshot_filename
    fi
    echo "  Saving 'dumpsys media.player' output to log file." >> $LOG_FILE
    if [ ! -z "$1" ]
    then
        echo "$1" >> $LOG_FILE
    else
        dumpsys media.player >> $LOG_FILE
    fi
}

wait_for_recording_to_finish() {
    if [ "$CAPTURE_SCREEN_RECORDING" = true ]
    then
        # If we are capturing screen recordings, we need to make sure our loop lasted the recording time
        time_left_in_screen_recording=$(($screen_recording_time-(`date +%s`-$loop_start_time)))
        if [ $time_left_in_screen_recording -gt 0 ]; then
            echo "Waiting $time_left_in_screen_recording seconds for the screen recording to complete." >> $LOG_FILE
            sleep $time_left_in_screen_recording
        fi
    fi
}

# Remove any existing log files
# Clear out our log files
if [ -f $LOG_FILE ]; then rm $LOG_FILE; fi;
if [ -f $PASS_LOG_FILE ]; then rm $PASS_LOG_FILE; fi;
if [ -f $FAIL_LOG_FILE ]; then rm $FAIL_LOG_FILE; fi;
if [ -f $FLAG_LOG_FILE ]; then rm $FLAG_LOG_FILE; fi;
# Clear out any old screenshots. We don't care if an error is raised (such as if there are no screenshots).
rm screenshot_* 2>/dev/null

# If arguments were given, use them
if [ "$#" -ge 1 ]; then
    ARG_RUN_MINUTES=$1
fi
if [ "$#" -ge 2 ]; then
    ARG_VIDEO_URL=$2
fi
if [ "$#" -ge 3 ]; then
    ARG_MIN_PLAY_TIME_SECONDS=$3
fi
if [ "$#" -ge 4 ]; then
    ARG_MAX_PLAY_TIME_SECONDS=$4
fi
if [ "$#" -ge 5 ]; then
    if [ "$5" = "0" ]
    then
        CAPTURE_SCREEN_RECORDING=false
    else
        CAPTURE_SCREEN_RECORDING=true
    fi
fi

echo "YouTube Playback Test `date`" >> $LOG_FILE
echo "Test Parameters:" >> $LOG_FILE
echo "  Run Time: $ARG_RUN_MINUTES minutes" >> $LOG_FILE
echo "  Video URL: $ARG_VIDEO_URL" >> $LOG_FILE
echo "  Minimum Play Time: $ARG_MIN_PLAY_TIME_SECONDS seconds" >> $LOG_FILE
echo "  Maximum Play Time: $ARG_MAX_PLAY_TIME_SECONDS seconds" >> $LOG_FILE
echo "  YouTube Start Command: $YOUTUBE_START_CMD" >> $LOG_FILE
echo "  YouTube Stop Command: $YOUTUBE_STOP_CMD" >> $LOG_FILE
echo "  AwesomePlayer Flag Log File: $FLAG_LOG_FILE" >> $LOG_FILE
echo "  AwesomePlayer Flag Retrieval Command: $FLAG_RETRIEVAL_CMD" >> $LOG_FILE
echo "" >> $LOG_FILE

loop_number=0
start_time=`date +%s`
end_time=$(($start_time+($ARG_RUN_MINUTES*60)))
# Loop until our time runs out
while [ `date +%s` -lt $end_time ]
do
    # Each loop of the test will consist of the following steps:
    #  1. Stop the YouTube app
    #  2. Start the YouTube app
    #  3. Poll the AwesomePlayer flags until one of the following events occurs:
    #    a. 0x20 is set, indicating end of video
    #    b. No flags found, indicating something happened to the app
    #    c. Playing time exceeds ARG_MAX_PLAY_TIME_SECONDS

    # If our YouTube app is running, kill it
    echo "Killing YouTube app" >> $LOG_FILE
    am force-stop com.google.android.youtube
    kill_end_wait_time=$((`date +%s`+30))
    exceeded_kill_wait_time=0
    # Wait 30 seconds for it to stop
    while [ ! -z `ps | grep "youtube"` ]
    do
        if [ `date +%s` -gt $kill_end_wait_time ]
        then
            echo "ERROR: Timeout (30 seconds) waiting for YouTube app to exit." >> $LOG_FILE
            fail_test
            # We use the exceeded_kill_wait_time flag so we can break out of the bigger loop if we fail
            exceeded_kill_wait_time=1
            fail_count_app_stop=$(($fail_count_app_stop+1))
            break
        fi
    done
    if [ $exceeded_kill_wait_time -eq 1 ]; then continue; fi;
    sleep $SLEEP_TIME_SECONDS_BETWEEN_LOOPS
    if [ -f $TEMP_FLAG_LOG_FILE ]; then rm $TEMP_FLAG_LOG_FILE; fi;
    loop_number=$(($loop_number+1))
    echo ""
    echo "Loop $loop_number started at $(date)" >> $LOG_FILE
    if [ "$CAPTURE_SCREEN_RECORDING" = true ]
    then
        # Capture a screen recording of the first 180 seconds
        screen_recording_file="screen_recording_`date +%s`.mp4"
        echo "Capturing screen recording to $screen_recording_file." >> $LOG_FILE
        # We wouldn't typically have to specify the size. However, we must specify the
        #  size b/c of BZ187572 (http://shilc211.sh.intel.com:41006/show_bug.cgi?id=187572)
        screenrecord --size 1280x720 --time-limit $screen_recording_time --bit-rate $SCREEN_RECORDING_BITRATE $screen_recording_file &
    fi
    flags_started=0
    debug_data_collected=false
    loop_start_time=`date +%s`
    last_heartbeat_print=$loop_start_time
    max_loop_end_time=$(($loop_start_time+$ARG_MAX_PLAY_TIME_SECONDS))
    echo "Starting YouTube app." >> $LOG_FILE
    am start -a android.intent.action.VIEW -d $ARG_VIDEO_URL
    if [ $? -ne 0 ]
    then
        echo "ERROR: YouTube start command returned $?, indicating error. Restarting loop." >> $LOG_FILE
        fail_test
        fail_count_app_start=$(($fail_count_app_start+1))
        continue
    fi
    # Added for SAND browser mode. This should be parametrized. 360x330 is for 720p [Jong: 1/5/15]
    sleep 5
    input tap 360 330    
    echo "Collecting flag values while the video plays..." >> $LOG_FILE
    last_loop_pass=true
    while true
    do
        if [ `date +%s` -gt $max_loop_end_time ]
        then
            echo "ERROR: Video took longer than $ARG_MAX_PLAY_TIME_SECONDS seconds to finish playing." >> $LOG_FILE
            max_play_time=$ARG_MAX_PLAY_TIME_SECONDS
            total_play_time=$(($total_play_time+$ARG_MAX_PLAY_TIME_SECONDS))
            collect_debug_data
            fail_count_exceeded_max_play_time=$(($fail_count_exceeded_max_play_time+1))
            wait_for_recording_to_finish
            break
        fi
        if [ `date +%s` -gt $(($last_heartbeat_print+$HEARTBEAT_INTERVAL_SECONDS)) ]
        then
            echo -n $HEARTBEAT_CHAR >> $LOG_FILE
            last_heartbeat_print=`date +%s`
        fi
        # The definition for the flags can be found at:
        #  https://android.googlesource.com/platform/frameworks/av/+/master/media/libstagefright/include/AwesomePlayer.h
        dumpsys_output=`dumpsys media.player`
        awesome_player_flags=`echo $dumpsys_output | grep URI | sed 's/.*flags(//g' | sed 's/).*//g'`
        echo $awesome_player_flags >> $TEMP_FLAG_LOG_FILE
        if [ -z $awesome_player_flags ]
        then
            if [ $flags_started -eq 1 ]
            then
                # This if statement is required so we only collect debug data once on failure
                if [ "$last_loop_pass" = true ]; then
                    echo "ERROR: AwesomePlayer flags value was not found in the dumpsys output, indicating the video is not playing. Failing test and continuing to see if we recover." >> $LOG_FILE
                    collect_debug_data "$dumpsys_output"
                    fail_count_flags_missing=$(($fail_count_flags_missing+1))
                fi
                last_loop_pass=false
                continue
            else
                # If the player doesn't start within 5 seconds of when it could possibly finish,
                #  fail the test and collect some debug data. We want to fail at 5 seconds before so
                #  we can capture debug data while its possibly running and we just don't know it.
                if [ $((`date +%s`-$loop_start_time)) -gt $MAX_START_WAIT_TIME ]
                then
                    echo "WARNING: YouTube took longer than $MAX_START_WAIT_TIME seconds to start the player. Not failing test b/c no proof that the video isn't really playing has been found." >> $LOG_FILE
                    collect_debug_data
                    fail_count_player_didnt_start=$(($fail_count_player_didnt_start+1))
                    wait_for_recording_to_finish
                    break
                fi
            fi
        else
            if [ $flags_started -eq 0 ]
            then
                # Log some stats on how long it took to start
                video_start_time=$((`date +%s`-$loop_start_time))
                echo "Start Time: $video_start_time seconds" >> $LOG_FILE
                if [ $video_start_time -lt min_start_time ]; then
                    min_start_time=$video_start_time
                fi
                if [ $video_start_time -gt max_start_time ]; then
                    max_start_time=$video_start_time
                fi
                total_start_time=$(($total_start_time+$video_start_time))
            fi
            flags_started=1
            if [ $(($awesome_player_flags&0x20)) -eq $((0x20)) ]
            then
                end_play_time=`date +%s`
                play_time=$(($end_play_time-$loop_start_time))
                # Collect Stats
                if [ $play_time -lt min_play_time ]; then
                    min_play_time=$play_time
                fi
                if [ $play_time -gt max_play_time ]; then
                    max_play_time=$play_time
                fi
                total_play_time=$(($total_play_time+$play_time))
                # Check to make sure we didn't finish too early
                if [ $play_time -lt $ARG_MIN_PLAY_TIME_SECONDS ]
                then
                    if [ "$last_loop_pass" = true ]; then
                        echo "ERROR: YouTube finished quicker than the minimum play time. Should have taken at least $ARG_MIN_PLAY_TIME_SECONDS seconds, but we completed in $play_time seconds. Failing test and continuing to see if we recover." >> $LOG_FILE
                        fail_count_player_finished_early=$(($fail_count_player_finished_early+1))
                        last_loop_pass=false
                        collect_debug_data
                    fi
                    continue
                fi
                if [ $last_heartbeat_print -ne $loop_start_time ]; then
                    # We have printed a heartbeat and heartbeats don't have a newline at the end
                    echo ""
                fi
                echo "Video finished playing at $(date)" >> $LOG_FILE
                wait_for_recording_to_finish
                break
            fi
        fi
        last_loop_pass=true
    done
    if [ "$CAPTURE_SCREEN_RECORDING" = true ]
    then
        # If we got this far, then we must have passed. No need in keeping a screen recording
        #  of a passing run.
        if [ "$debug_data_collected" = false ]; then
            echo "Loop passed. Removing screen recording file." >> $LOG_FILE
            rm $screen_recording_file
        fi
    fi
    echo "AwesomePlayer Flag Summary:" >> $LOG_FILE
    echo "  (count flags_value: flags_decode)" >> $LOG_FILE
    echo "Loop $loop_number:" >> $FLAG_LOG_FILE
    # Print with an explanation of each flag
    IFS=$'\n'
    for x in `uniq -c $TEMP_FLAG_LOG_FILE`
    do
        count=`echo "$x" | awk '{printf $1}'`
        flags=`echo "$x" | awk '{printf $2}'`
        printf "% 7d %s: " $count $flags >> $LOG_FILE;
        if [ -z $flags ]; then
            echo "Player not started" >> $LOG_FILE;
            continue
        fi
        # The definition for the flags can be found at:
        #  https://android.googlesource.com/platform/frameworks/av/+/master/media/libstagefright/include/AwesomePlayer.h
        if [ $(($flags&0x01)) -eq $((0x01)) ]; then echo -n "PLAYING " >> $LOG_FILE; fi
        if [ $(($flags&0x02)) -eq $((0x02)) ]; then echo -n "LOOPING " >> $LOG_FILE; fi
        if [ $(($flags&0x04)) -eq $((0x04)) ]; then echo -n "FIRST_FRAME " >> $LOG_FILE; fi
        if [ $(($flags&0x08)) -eq $((0x08)) ]; then echo -n "PREPARING " >> $LOG_FILE; fi
        if [ $(($flags&0x10)) -eq $((0x10)) ]; then echo -n "PREPARED " >> $LOG_FILE; fi
        if [ $(($flags&0x20)) -eq $((0x20)) ]; then echo -n "AT_EOS " >> $LOG_FILE; fi
        if [ $(($flags&0x40)) -eq $((0x40)) ]; then echo -n "PREPARE_CANCELLED " >> $LOG_FILE; fi
        if [ $(($flags&0x80)) -eq $((0x80)) ]; then echo -n "CACHE_UNDERRUN " >> $LOG_FILE; fi
        if [ $(($flags&0x0100)) -eq $((0x0100)) ]; then echo -n "AUDIO_AT_EOS " >> $LOG_FILE; fi
        if [ $(($flags&0x0200)) -eq $((0x0200)) ]; then echo -n "VIDEO_AT_EOS " >> $LOG_FILE; fi
        if [ $(($flags&0x0400)) -eq $((0x0400)) ]; then echo -n "AUTO_LOOPING " >> $LOG_FILE; fi
        if [ $(($flags&0x0800)) -eq $((0x0800)) ]; then echo -n "PREPARING_CONNECTED " >> $LOG_FILE; fi
        if [ $(($flags&0x1000)) -eq $((0x1000)) ]; then echo -n "SEEK_PREVIEW " >> $LOG_FILE; fi
        if [ $(($flags&0x2000)) -eq $((0x2000)) ]; then echo -n "AUDIO_RUNNING " >> $LOG_FILE; fi
        if [ $(($flags&0x4000)) -eq $((0x4000)) ]; then echo -n "AUDIOPLAYER_STARTED " >> $LOG_FILE; fi
        if [ $(($flags&0x8000)) -eq $((0x8000)) ]; then echo -n "INCOGNITO " >> $LOG_FILE; fi
        if [ $(($flags&0x10000)) -eq $((0x10000)) ]; then echo -n "TEXT_RUNNING " >> $LOG_FILE; fi
        if [ $(($flags&0x20000)) -eq $((0x20000)) ]; then echo -n "TEXTPLAYER_INITIALIZED " >> $LOG_FILE; fi
        if [ $(($flags&0x40000)) -eq $((0x40000)) ]; then echo -n "SLOW_DECODER_HACK " >> $LOG_FILE; fi
        echo "" >> $LOG_FILE
    done
    uniq -c $TEMP_FLAG_LOG_FILE >> $FLAG_LOG_FILE
done

echo "Youtube Playback Test End Time : $(date)" >> $LOG_FILE
if [ $min_start_time -ne 999999 ] # Don't print if we didn't have any successful starts
then
    echo "Video Start Time Stats:" >> $LOG_FILE
    echo "  Minimum Start Time: $min_start_time seconds" >> $LOG_FILE
    echo "  Maximum Start Time: $max_start_time seconds" >> $LOG_FILE
    echo "  Average Start Time: $(($total_start_time/$loop_number)) seconds" >> $LOG_FILE
fi
if [ $max_play_time -ne 0 ] # Don't print if we didn't play anything
then
    echo "Video Play Time Stats:" >> $LOG_FILE
    echo "  Minimum Play Time: $min_play_time seconds" >> $LOG_FILE
    echo "  Maximum Play Time: $max_play_time seconds" >> $LOG_FILE
    echo "  Average Play Time: $(($total_play_time/$loop_number)) seconds" >> $LOG_FILE
fi

fail_totals=0
fail_count_summary=""
function append_fail_count {
    # Argument 1: String to append if count > 0
    # Argument 2: Failure count
    if [ $2 -gt 0 ]
    then
        fail_count_summary="$fail_count_summary\n  $1: $2"
        fail_totals=$((fail_totals+$2))
    fi
}
append_fail_count "AwesomePlayer flags missing from dumpsys output (PS400327)" $fail_count_flags_missing
append_fail_count "Player didn't start (no flags appeared in dumpsys output) (PS400328)" $fail_count_player_didnt_start
append_fail_count "Play time exceeded maximum play time" $fail_count_exceeded_max_play_time
append_fail_count "Play time was less than the minimum play time" $fail_count_player_finished_early
if [ $fail_totals -gt 0 ]
then
    echo -e "Documented Failures (Do not cause test to fail) Counts:$fail_count_summary" >> $LOG_FILE
fi
fail_totals=0
fail_count_summary=""
append_fail_count "App failed to stop" $fail_count_app_stop
append_fail_count "App failed to start" $fail_count_app_start
if [ $fail_totals -gt 0 ]
then
    echo -e "Undocumented Failure (Will cause test to fail) Counts:$fail_count_summary" >> $LOG_FILE
fi

if [ $test_passed -eq 1 ]
then
    mv $LOG_FILE $PASS_LOG_FILE
else
    mv $LOG_FILE $FAIL_LOG_FILE
fi
