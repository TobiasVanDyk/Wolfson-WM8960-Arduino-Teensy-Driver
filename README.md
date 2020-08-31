# Wolfson-WM8960-Arduino-Teensy-Driver
Driver and example applications for Arduino and Teensy MCUs for the Wolfson WM8960 Audio DAC. 

Note the difference in the headphone output circuit between the Waveshare Raspberry Pi Hat (which is also used in the Seeed Studio Re-Speaker Hat), and the Waveshare Audio CODEC module. That combination of the CODEC and Teensy sounds much better in the low range end due to the LC filter at the headphone plug. Both type of audio codec modules need an additional section of code in the main loop to enable manual switching between speaker and headphones (similar to the option in the Linuc alsamixer) - this will be added soon. 

In addition a section for the windows main volume control will be added (currently the application in windows control the DAC volume).

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

This combination of the CODEC and Teensy sounds much better in the low range end due to the LC filter at the headphone plugs. It needs a new section of code to enable manual switching between speaker and headphones (similar to the option in the Linuc alsamixer).
<p align="left">
<img src="images/rpidacteensy41.jpg" width="600" />  
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
