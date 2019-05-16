#!/sbin/sh
# Script for playback End Sequence

# SBA_SET_SSP
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,1,1,1,0,0,24,0,0,0,255,255,117,0,16,0,3,0,0,0,24,9,15,0,15,0,3,0,1,1,1,0 > /dev/null		

# MMX_SET_SWM
# MEDIA1_OUT
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,1,1,3,255,0,20,0,0,255,255,255,114,0,12,0,0,19,0,0,0,0,1,0,0,144,0,0 > /dev/null

# MMX_SET_MEDIA_PATH
# MEDIA1_OUT
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,1,1,3,255,0,10,0,0,19,255,255,119,0,2,0,0,0 > /dev/null

# SBA_SET_SWM
#end_pb_sba_set_swm_CODEC_OUT0()
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,1,1,1,255,0,20,0,0,255,255,255,114,0,12,0,0,3,255,255,0,0,1,0,0,142,0,0 > /dev/null

# SBA_SET_MEDIA_PATH
# PB_PCM1_IN
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,1,1,1,255,0,10,0,0,142,255,255,119,0,2,0,0,0 > /dev/null

######## SBA_VB_STOP
alsa_amixer -c wm8958audio cset name='SST Byte control' 1,1,1,1,0,0,8,0,0,0,255,255,126,0,0,0 > /dev/null
