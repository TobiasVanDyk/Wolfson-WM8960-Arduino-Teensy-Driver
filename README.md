# Wolfson-WM8960-Arduino-Teensy-Driver
Proto-driver (eventually as a library), and example applications for Arduino and Teensy MCUs using the Wolfson WM8960 Audio DAC, with separate stereo BTL 1W speaker and headphone amplifiers (common grounded) (Note 1). 

Note the difference in the headphone output circuit between the Waveshare Raspberry Pi Hat (which is also used in the Seeed Studio Re-Speaker Hat), and the Waveshare Audio CODEC module. That combination of the CODEC and Teensy sounds much better in the low range possibly due to the LC filter configuration at the headphone plug. The hat type of audio codec module needs an additional section of code in the main loop to enable manual switching between speaker and headphones (similar to the option in the Linux alsamixer) - this will be added soon. Currently both are enabled for the hat type board.

In addition the normal Teensy main volume control via two mono amplifiers will be added (currently the application in windows control the DAC volume), after further testing on setting the WM8960 analog output volume directly through i2c control.

Subjectively this 2007, 14 year old DAC (Notes 2 and 3) outperforms its specifications. Listening to the same source material through the headphone output on the waveshare wm8960 hat dac, matches the listening experience when using the same headphones with a Creative AE-5 (which has a dedicated headphone amp), setup.

Short interconnect wiring must be used - else 100 to 220 ohm resistors inline for all the i2s signal wiring can be used as was the case here. 3k9 pullup resistors was also used on the i2c SDA and SCL lines.

*Note 1: It should be possible to use the BTL speaker outputs isolated through 100uF capacitors, with common ground headphones - refer to the block diagram.*<br>
*Note 2: See the WM8960-ALSA-driver folder above, for the original 2007 WM8960 ALSA SoC Audio driver from Wolfson Microelectronics (PLC).*<br>
*Note 3: The latest datasheet from Cirrus Logic for the WM8960 is dated as recent as 2019: [WM8960_v4.4.pdf](WM8960_v4.4.pdf).*
 
[**Cirrus Logic Product WM8960**](https://www.cirrus.com/products/wm8960/) 

[**Waveshare WM8960 Stereo CODEC General purpose module**](https://www.waveshare.com/wm8960-audio-board.htm)
<p align="left">
<img src="images/module1.jpg" width="200" />  
<img src="images/module2.jpg" width="200" /> 
<img src="images/WM8960Schematic.jpg" width="300" /> 
<br>

[**Waveshare WM8960 Hi-Fi Sound Card HAT for Raspberry Pi**](https://www.waveshare.com/wm8960-audio-hat.htm)
<p align="left">
<img src="images/rpi1.jpg" width="200" />  
<img src="images/rpi2.jpg" width="200" /> 
<img src="images/WM8960HatSchematic.jpg" width="300" />   
<br

**Teensy 4.1 and Teensy 3.6 with the Waveshare WM8960 Stereo CODEC as USB Audio DAC:**
<p align="left">
<img src="images/wm8960.jpg" width="600" />  
<br>
  
**Teensy 4.0 and the Waveshare WM8960 Stereo CODEC as USB Audio DAC:**
<p align="left">
<img src="images/teensy40-wm8960.jpg" width="600" />  
<br>
  
**Teensy 4.1 and the Waveshare WM8960 Raspberry Pi Hat as USB Audio DAC:**

This combination of the CODEC and Teensy sounds much better in the low range end due to the LC filter at the headphone plug. It needs a new section of code to enable manual switching between speaker and headphones (similar to the option in the Linux alsamixer). Short interconnect wiring must be used - unlike the photo below.
<p align="left">
<img src="images/rpihat1.jpg" width="600" />  
<br>

**Raspberry Pi and Waveshare WM8960 as Audio DAC:**
<p align="left">
<img src="images/rpimodule.jpg" width="300" />  
<img src="images/rpirpi1.jpg" width="300" /> 
<br

The WM8960 CODEC is also used by the [**Seeed voicecard**](https://github.com/respeaker/seeed-voicecard) or [**ReSpeaker 2-Mics Pi HAT**](https://wiki.seeedstudio.com/ReSpeaker_2_Mics_Pi_HAT/).
<p align="left">
<img src="images/respeaker1.png" width="200" />  
<img src="images/respeaker2.png" width="300" /> 
<img src="images/SeedWM8960.png" width="300" />   
<br
