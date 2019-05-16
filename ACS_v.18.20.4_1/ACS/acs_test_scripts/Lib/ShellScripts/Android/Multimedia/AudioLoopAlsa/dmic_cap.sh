echo "Capture from On-board Mic"
# configure codec mixer for speaker
sh dmic_mixer.sh 

alsa_arecord -Dhw:wm8958audio,0 /sdcard/cap48k.wav -f S16_LE -r 48000 -c 2 -d 60 &

alsa_amixer -c wm8958audio cset name='SST Byte control' 1,1,1,1,0,0,8,0,0,0,255,255,85,0,0,0 > /dev/null
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,1,1,1,255,0,20,0,0,255,255,255,114,0,12,0,0,14,255,255,3,0,1,0,0,130,0,0 > /dev/null
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,1,1,1,255,0,10,0,0,14,255,255,119,0,2,0,1,0 > /dev/null
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,1,1,1,0,0,26,0,255,255,255,255,117,0,18,0,3,0,3,0,24,9,15,255,15,255,3,0,1,1,1,0,0,0 > /dev/null

sleep 2

alsa_amixer -c wm8958audio cset name='SST Byte control' 1,2,0,1,14,0,20,0,255,255,255,255,33,0,12,0,1,0,0,14,103,0,0,0,0,0,50,0 > /dev/null
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,2,0,1,130,0,20,0,255,255,255,255,33,0,12,0,1,0,0,130,103,0,0,0,0,0,50,0 > /dev/null


