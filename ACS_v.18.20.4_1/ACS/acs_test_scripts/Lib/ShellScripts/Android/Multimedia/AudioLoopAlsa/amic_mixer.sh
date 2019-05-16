#!/sbin/sh

# AMIC input slot 0
# -----------------

alsa_amixer -c wm8958audio cset name='IN1L Switch' on
# add in 20dB analog gain.
alsa_amixer -c wm8958audio cset name='IN1L Volume' 25
alsa_amixer -c wm8958audio cset name='IN1L PGA IN1LN Switch' on
alsa_amixer -c wm8958audio cset name='IN1L PGA IN1LP Switch' on
alsa_amixer -c wm8958audio cset name='MIXINL IN1L Switch' on
alsa_amixer -c wm8958audio cset name='AIF1ADC1 Volume' 110 110
alsa_amixer -c wm8958audio cset name='MIXINL IN1L Switch' on
alsa_amixer -c wm8958audio cset name='ADCL Mux' 'ADC'
alsa_amixer -c wm8958audio cset name='ADCR Mux' 'ADC'
alsa_amixer -c wm8958audio cset name='AIF1ADC1L Mixer ADC/DMIC Switch' on
alsa_amixer -c wm8958audio cset name='AIF1ADC1R Mixer ADC/DMIC Switch' on


#!/sbin/sh

# AMIC input slot 0
# -----------------

#alsa_amixer -c wm8958audio cset name='IN1L Switch' on
# add in 20dB analog gain.
#alsa_amixer -c wm8958audio cset name='IN1L Volume' 25
#alsa_amixer -c wm8958audio cset name='IN1L PGA IN1LN Switch' on
#alsa_amixer -c wm8958audio cset name='MIXINL IN1L Switch' on

#alsa_amixer -c wm8958audio cset name='ADCL Mux' 'ADC'
#alsa_amixer -c wm8958audio cset name='AIF1ADC1L Mixer ADC/DMIC Switch' on
