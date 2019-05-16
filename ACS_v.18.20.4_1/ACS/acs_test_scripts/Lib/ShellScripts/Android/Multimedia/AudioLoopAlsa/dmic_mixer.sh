alsa_amixer -c wm8958audio cset name='ADCL Mux' 'DMIC'
alsa_amixer -c wm8958audio cset name='ADCR Mux' 'DMIC'

alsa_amixer -c wm8958audio cset name='AIF1ADC1L Mixer ADC/DMIC Switch' on
alsa_amixer -c wm8958audio cset name='AIF1ADC1R Mixer ADC/DMIC Switch' on
