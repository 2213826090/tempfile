#!/sbin/sh
# Set down things
alsa_amixer -c wm8958audio cset name='Headphone Switch' off
alsa_amixer -c wm8958audio cset name='Speaker Switch' off
alsa_amixer -c wm8958audio cset name='Earpiece Switch' off

# Headset settings
alsa_amixer -c wm8958audio cset name='Headphone Switch' on

# Core DAC settings
alsa_amixer -c wm8958audio cset name='DAC1L Mixer AIF1.2 Switch' on
alsa_amixer -c wm8958audio cset name='DAC1R Mixer AIF1.2 Switch' on
alsa_amixer -c wm8958audio cset name='DAC1 Switch' on

# Headset MUX settings
alsa_amixer -c wm8958audio cset name='Left Output Mixer DAC Switch' on
alsa_amixer -c wm8958audio cset name='Left Headphone Mux' 'Mixer'
alsa_amixer -c wm8958audio cset name='Right Output Mixer DAC Switch' on
alsa_amixer -c wm8958audio cset name='Right Headphone Mux' 'Mixer'
