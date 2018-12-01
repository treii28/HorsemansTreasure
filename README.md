# HorsemansTreasure
ESP32 project to bring Richard Gleave's Headless Horseman reliquary to life

The reliquary in the book(s) consisted of an antique, gilded lantern that contained the severed head of the headless horseman. Enchanted by witchcraft, the person possessing the lantern could speak magic words while making a blood sacrifice and it would become activated doing a number of supernatural effects such as glowing and speaking, pulsing blood across the outside through veins, and glowing letters would appear inscribed in the top.
The lantern would 'offer' a spike out of the top to help facilitate the sacrifice and when really empowered would glow exceptionally bright. Initially, the glass was cloudy in part due to age. But as the power would rise, the glass would become clearer revealing the head inside.

Hardware included thus far:

Main processor is an ESP32 Wemos D1 mini clone
Voice recognition will be handled by the EasyVR v3
LED fading and servos is via a PCA9685 16-channel I2C PWM expander 
audio support with PCM5102 I2S DAC
wav and mp3 playback is via a slightly modified version of the ESP8266Audio library
SD card support via generic microSD adapter

LED effects will include 370nm UV lighting which will illuminate UV reactive ink that is otherwise invisible.

Other effects to include frosted smart-material inside the outer glass which will magically 'clear' to reveal the contents, a mist-maker to emulate 'smoke', a fluid sensor that will activate 'dripping' red LEDs when water is added for the mister, UV leds inside the tower (to reveal the inscription on the top of the lantern housing) and along the sides (to show the veins on the skull) and 3W LEDs inside the lid for extra-bright flash and glow effects.

Interfacing may include a wifi web server or possibly a rest server and maybe even Blynk compatibility. The hope is to enable the ability to remotely update the firmware over the internet when needed, allow some level of modification to the audio files and various effects, possibly even alter/update the voice recognition settings, and if possible, include multiple modes. (one for the main book effects and a few others for interactivity or just a 'fun' version for laughs)


File info:
  Main program code:
    main.cpp
  Pin and global definitions:
    misc_defaults.h

  Modified version of the I2S output from ESP8266 audio to allow reading an envelope value representing a running average of the output on both channels. (used to get make the eyes glow and mouth move as the sounds play)
    AudioOutputI2SAve.cpp
    AudioOutputI2SAve.h

  Beginnings of importing the EasyVR voice command code:
    easyvr_defs.h

