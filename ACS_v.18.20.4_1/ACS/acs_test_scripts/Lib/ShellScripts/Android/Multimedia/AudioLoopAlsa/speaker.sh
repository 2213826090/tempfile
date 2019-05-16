#!/sbin/sh

# Set down things
alsa_amixer -c wm8958audio cset name='Headphone Switch' off
alsa_amixer -c wm8958audio cset name='Speaker Switch' off
alsa_amixer -c wm8958audio cset name='Earpiece Switch' off

# Speaker settings
alsa_amixer -c wm8958audio cset name='Speaker Switch' on
alsa_amixer -c wm8958audio cset name='Speaker Mixer Volume' 3 3
alsa_amixer -c wm8958audio cset name='SPKL DAC1 Switch' on
alsa_amixer -c wm8958audio cset name='SPKR DAC1 Switch' on
alsa_amixer -c wm8958audio cset name='Speaker Volume' 63,63

# Core DAC settings
alsa_amixer -c wm8958audio cset name='DAC1L Mixer AIF1.1 Switch' on    # Change this to AIF1.2, in case to change the slot on codec side
alsa_amixer -c wm8958audio cset name='DAC1R Mixer AIF1.1 Switch' on
alsa_amixer -c wm8958audio cset name='DAC1 Switch' on

#From Jeeja
alsa_amixer -c wm8958audio cset name='SPKL Boost SPKL Switch' on
alsa_amixer -c wm8958audio cset name='Speaker ZC Switch' on on 
alsa_amixer -c wm8958audio cset name='Earpiece Mixer Left Output Switch' off
alsa_amixer -c wm8958audio cset name='Left Output Mixer DAC Switch' on     
