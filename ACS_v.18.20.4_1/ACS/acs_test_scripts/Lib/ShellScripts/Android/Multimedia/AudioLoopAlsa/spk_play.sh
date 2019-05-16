echo "Playing on Speaker"
# configure codec mixer for speaker
sh speaker.sh
# Send AUD commands to LPE
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,1,1,1,0,0,8,0,0,0,255,255,85,0,0,0 > /dev/null
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,1,1,3,255,0,20,0,0,255,255,255,114,0,12,0,0,19,255,255,3,0,1,0,0,144,0,0 > /dev/null
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,1,1,3,255,0,10,0,0,19,255,255,119,0,2,0,1,0 > /dev/null
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,1,1,1,255,0,10,0,0,142,255,255,119,0,2,0,1,0 > /dev/null 
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,1,1,1,255,0,20,0,0,255,255,255,114,0,12,0,0,2,255,255,3,0,1,0,0,142,0,0 > /dev/null
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,1,1,1,0,0,26,0,255,255,255,255,117,0,18,0,3,0,3,0,24,9,15,255,15,255,3,0,1,1,1,0,0,0 > /dev/null

# ALSA play cmd
alsa_aplay -Dhw:wm8958audio,0,0 -f dat /sdcard/Music/s_48k_16.wav -F 24000 -B 96000 &

sleep 2 

alsa_amixer -c wm8958audio cset name='SST Byte control' 1,2,0,3,144,0,20,0,255,255,255,255,33,0,12,0,1,0,0,144,103,0,0,0,0,0,50,0 > /dev/null
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,2,0,1,142,0,20,0,255,255,255,255,33,0,12,0,1,0,0,142,103,0,0,0,0,0,50,0 > /dev/null
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,2,0,1,2,0,20,0,255,255,255,255,33,0,12,0,1,0,0,2,103,0,0,0,0,0,50,0 > /dev/null



