# Wolfson-WM8960-Arduino-Teensy-Driver
<img src="SparkFun-WM8960/SparkFun_Audio_Codec_Breakout.jpg" width="16" height="16"/> January 2023: There is now a [**Sparkfun WM8960 Breakout Board**](https://www.sparkfun.com/products/21250) and a feature-complete [**Arduino driver library**](https://github.com/sparkfun/SparkFun_WM8960_Arduino_Library).

Proto-driver (eventually as a library similar to the STM32 library in the folder STM32-Open103Z), and example applications for Arduino and Teensy MCUs (Note 1), using the Wolfson WM8960 Audio DAC, with separate stereo BTL 1W speaker and headphone amplifiers (common grounded - Note 2), and a fractional-N PLL (which is missing from the the WM8731 DAC), enabling the use of one external crystal supporting most commonly-used audio-clock needs (Note 4 page 60). 

Note the difference in the headphone output circuit (Note 5), between the Waveshare Raspberry Pi Hat (which similar to the configuration in the Seeed Studio Re-Speaker Hat), and the Waveshare Audio CODEC module. That specific board type and the Teensy, has an better low end range, and also has an LC filter at the headphone plug. The hat type of audio codec module lacks the optional hardware configuration for headphone insert detection, and it needs an additional code to enable manual switching between speaker and headphones (similar to the option in the Linux alsamixer). Currently both outputs are enabled for the hat type board. The other type of audioboard has the headphone auto-detection circuitry enabled.

The standard Teensy main volume control via two mono amplifiers has been added for the hat type board (else only the current windows application control the DAC volume). Setting the WM8960 analog output volume directly through i2c control is also possible.

Subjectively this 2007, 14 year old DAC (Notes 3 and 4) outperforms its specifications. Listening to the same source material through the headphone output on the waveshare wm8960 hat dac, are similar to the listening experience when using the same source material and headphones with a Creative AE-5 (which has a dedicated headphone amplifier). Both module types do not need/have a MCLK connected.

Short interconnect wiring must be used - else 100 to 220 ohm resistors inline for all or some of the i2s signal wiring can be used, as was the case here. 3k9 pullup resistors was also used on the i2c SDA and SCL lines.

*Note 1: It is puzzling why a search for an Arduino driver was unsuccessful (August 2020), especially considering that this CODEC is widely used by the Raspberry Pi community, and is still in considerable demand, as seemingly confirmed by Note 4 below.*<br>
*Note 2: It should be possible to use the BTL speaker outputs isolated through 100uF capacitors, with common ground headphones - refer to the [block diagram](block-diagram.jpg).*<br>
*Note 3: See the WM8960-ALSA-driver folder above, for the original 2007-2011 WM8960 ALSA SoC Audio driver from Wolfson Microelectronics (PLC).*<br>
*Note 4: The latest datasheet from Cirrus Logic for the WM8960 is dated as recent as 2019: [WM8960_v4.4.pdf](WM8960_v4.4.pdf).*<br>
*Note 5: The headphone output is also used as a line output. On the General DAC type board there is also one input available on the line output jack.*

Table 1: Connections between WM8960 Raspberry Pi HAT and Teeensy 3x and 4x: 

| WM8960 RPi  | Teensy 4.x | Teensy 3.x  | 
|:------------|:-----------|:------------|
| 2,4 +5v     |	+5v        | +5v         |
| 6   GND     | GND 	      | GND 	       | 
| 3   SDA     |	18 SDA 	   | 18 SDA      | 
| 5   SCL     |	19 SCL     | 19 SCL      | 
| 12  PCM-CLK |	21 BCLK    | 9  BCLK     |
| 35  PCM-FS  | 20 LRCLK   | 23 LRCLK    |
| 38  PCM-IN  | 8  TX      | 13 SCK      |
| 40  PCM-OUT | 7  RX      | 22          |


Table 2: Connections between WM8960 General DAC and Teeensy 3x and 4x: 

| WM8960 DAC     | Teensy 4.x | Teensy 3.x  | 
|:---------------|:-----------|:------------|
| 1,2  +3v3      | +3v3       | +3v3        |
| 3,4   GND      | GND        | GND 	       | 
| 5,6   SDA      | 18 SDA     | 18 SDA      | 
| 7,8   SCL      | 19 SCL     | 19 SCL      | 
| 9,10  SCLK     | 21 BCLK    | 9  BCLK     |
| 11,12 WS       | 20 LRCLK   | 23 LRCLK    |
| 13    RXSDA    | 7  RX      | 22          |
| 14    TXSDA    | 8  TX      | 13 SCK      |
| 15,16 MCLK R/T |            |             |
 
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
  
**Teensy 4.0 and the Waveshare WM8960 Raspberry Pi Hat as USB Audio DAC:**

This combination of the CODEC and Teensy sounds much better in the low range end due to the LC filter at the headphone plug. It needs a new section of code to enable manual switching between speaker and headphones (similar to the option in the Linux alsamixer). Short interconnect wiring must be used - unlike the photo below.
<p align="left">
<img src="images/rpihat2.jpg" width="600" />  
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
