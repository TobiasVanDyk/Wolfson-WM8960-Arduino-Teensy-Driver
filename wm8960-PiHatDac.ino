////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPL3 license Tobias van Dyk Aug 2020
// Based on Wolfson WM8960 STM32 demo code from Waveshare
// Demo code BSD license from STMicroelectronics
// Also based on WM8960 ALSA SoC Audio driver from Wolfson Microelectronics
// 2007 GPL2 license Liam Girdwood
////////////////////////////////////////////////////////////////////////////////////////////////////////////
//       WM8960-DAC      Teensy 4.0 and 4.1    Teensy 3.6       Audioboard 3  WM8960 RPi GPIO Hat 
//                                                                    2,4 5v
//  1,2  VCC      Red    3V3                   3V3                            1   NC (3v3)
//  3,4  GND    Brown    GND                   GND (not AGnd)                 6   GND
//  5,6  SDA     Gray    18 SDA0               18 SDA0         18 SDA         3   SDA
//  7,8  SCL   Orange    19 SCL0               19 SCL0         19 SCL         5   SCL
//  9,10 CLK     Blue    21 BCLK1              9 BCK           9 BCLK        12  PCMCLK BCLK
// 11,12 WS     White    20 LRCLK1             23 LRCK         23 LRCLK       35  PCM_FS LRCLK i2s Frame clock input
// 13    RXSDA   Green    7 OUT1A              22 TX           22 TX          40  PCM_OUT I2S Data output
// 14    TXSDA    Blue    8 TX1                13 RX           13 RX          38  PCM_IN  I2S Data input
// 15    RXMCLK                                                                   I2S System Clock(Sending)
// 16    TXMCLK                                                                   I2SSystem Clock(Receive)
// WM8960_ADDRESS = 0x1a 
// MCLK signal transmitted (TX MCLK RX) MCLK signal transmitted   
//                                                           11 MCLK
//                                                            7 MOSI
//                                                           12 MISO
//                                                           14 SCLK
//                                                           6,10 CS
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>

int led = LED_BUILTIN;

#define WM8960_ADDRESS  0x1A

//register values
static uint16_t WM8960_REG_VAL[56] =
{  
  0x0097, 0x0097, 0x0000, 0x0000, 0x0000, 0x0008, 0x0000, 0x000A,
  0x01C0, 0x0000, 0x00FF, 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0000, 0x007B, 0x0100, 0x0032, 0x0000, 0x00C3, 0x00C3, 0x01C0,
  0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
  0x0100, 0x0100, 0x0050, 0x0050, 0x0050, 0x0050, 0x0000, 0x0000,
  0x0000, 0x0000, 0x0040, 0x0000, 0x0000, 0x0050, 0x0050, 0x0000,
  0x0000, 0x0037, 0x004D, 0x0080, 0x0008, 0x0031, 0x0026, 0x00ED
};

#include <Audio.h>
#include <SPI.h>

// GUItool: begin automatically generated code
AudioInputUSB            usb1;           //xy=121,162
AudioOutputI2S           i2s1;           //xy=559,166
AudioConnection          patchCord1(usb1, 0, i2s1, 0);
AudioConnection          patchCord2(usb1, 1, i2s1, 1);
// GUItool: end automatically generated code

 int delay1 = 10;
 int delay2 = 500;
 

//////////////////////////////////////////////////////////////////////////////////////////////////
// Registers of WM8960 are 9-bit. Thus, when we send data to it, 
// should split data to two bytes and add the ID of register to recognized before
// transmitting. The ID of register flagged by 7-bit.
//
// Byte0 = bit7 to bit1 = device address, bit0 = read/write
// Byte1 = bit7 to bit1 = register number 0-57, bit0 = MSB bit8 of register value
// Byte2 =                                                 bit7-bit0 register value
/////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////
// WM89060 Write RegisterNumber = Value
/////////////////////////////////////////
uint8_t WM8960_Write_Reg(uint8_t reg, uint16_t dat)  
{ 
  uint8_t res,I2C_Data[2];
  
  I2C_Data[0] = (reg<<1)|((uint8_t)((dat>>8)&0x0001));  //RegAddr
  I2C_Data[1] = (uint8_t)(dat&0x00FF);                  //RegValue

  //digitalWrite(led, HIGH);                              // briefly flash the LED

  Wire.beginTransmission(WM8960_ADDRESS);  // transmit to device lsb=0 => write
  //Wire.write(I2C_Data[0]);                  // buffer 1 byte reg1 in b7-b1
  //Wire.write(I2C_Data[1]);                  // buffer 1 byte lsb of val1
  Wire.write(I2C_Data, 2);                    // buffer 1 byte lsb of val1
  res = Wire.endTransmission();               // transmit buffer and then stop

  if(res == 0) { WM8960_REG_VAL[reg] = dat;
                 //digitalWrite(led, LOW); 
               }

  return res;
}

/////////////////////////////////////////
// WM89060 Inititialise
/////////////////////////////////////////
uint8_t WM89060_Init(void)  {

  uint8_t res;
  
  //////////////////////////////////////////////////////////  
  // Reset Device 0x0f, 0x0000
  // #define WM8960_RESET    0xf
  // reg<<1 register now in bit7-bit1
  // msb=b8 (ninth bit) of val now in bit0
  // uint8_t  reg1 = ((reg << 1) |(uint8_t)((val >> 8) & 1))
  // val = lsb b7-b0 val and with val msb b15-b18 all = 0
  ////////////////////////////////////////////////////////
  res = WM8960_Write_Reg(0x0f, 0x0000);
  //if (res == 0) Serial.println("WM8960 reset completed"); else return res;
  delay(delay1);
  
  // Set Power Source
  // #define WM8960_POWER1  0x19
  // #define WM8960_POWER2  0x1a bits 6,5,4,3 HP and Sp enable
  // #define WM8960_ADDCTL3 0x1b **************
  // #define WM8960_POWER3  0x2f
  res =  WM8960_Write_Reg(0x19, 1<<8 | 1<<7 | 1<<6);
  delayMicroseconds(delay2);
  res += WM8960_Write_Reg(0x1A, 1<<8 | 1<<7 | 1<<6 | 1<<5 | 1<<4 | 1<<3 | 1);
  delayMicroseconds(delay2);
  res += WM8960_Write_Reg(0x2F, 1<<3 | 1<<2);
  //if (res == 0) Serial.println("WM8960 power 1,2,3 completed"); else return res;
  delay(delay1);
  
  // Configure clock
  // MCLK->PLL->SYSCLK->DAC/ADC sample Freq = 24MHz/544.217 = 44.100kHz
  // See calculation pages 60 and table 44 in manual
  // f2 = 4 x 2 x 11.2896Hz = 90.3168MHz
  // R = 90.316 / 12 = 7.5264
  // PLLN = int R = 7
  // k = int ( 2^24 x (7.5264 – 7)) = 8831526
  // N = 7
  // Fractional 24bit value = 86C226h
  res = WM8960_Write_Reg(0x04, 0x0001); // Select PLL
  //if (res == 0) Serial.println("WM8960 Configure clock"); else return res;
  delay(delay1);

  // Configure PLL 1  0011 0111 = 37h
  //                 00010 0111   27h
  res = WM8960_Write_Reg(0x34, 0x0027); // Select PLL 1
  delay(delay1);

  // Configure PLL 2  bit 8 reserved 7-0 data
  res = WM8960_Write_Reg(0x35, 0x0086); // Select PLL 2
  delay(delay1);

  // Configure PLL 3   
  res = WM8960_Write_Reg(0x36, 0x00C2); // Select PLL 3
  delay(delay1);

  // Configure PLL 4   
  res = WM8960_Write_Reg(0x37, 0x0026); // Select PLL 4
  delay(delay1);

  // Configure ADC/DAC
  // bit0 = 1 ADC High Pass Filter Disable
  // bit1,2 De-emphasis 00 = No de-emphasis
  res = WM8960_Write_Reg(0x05, 0x0000);
  //if (res == 0) Serial.println("WM8960 Configure ADC/DAC"); else return res;
  delay(delay1);
  
  // Configure audio interface
  // I2S format 16 bits word length
  res = WM8960_Write_Reg(0x07, 0x0002);
  //if (res == 0) Serial.println("WM8960 Configure audio interface"); else return res;
  delay(delay1);
  
  // Configure HP_L and HP_R OUTPUTS was 0x006F | 0x0100
  res =  WM8960_Write_Reg(0x02, 0x007F | 0x0100);  //LOUT1 Volume Set
  delayMicroseconds(delay2);
  res += WM8960_Write_Reg(0x03, 0x007F | 0x0100);  //ROUT1 Volume Set
  //if (res == 0) Serial.println("WM8960 Configure HP_L and HP_R OUTPUTS"); else return res;
  delay(delay1);
  
  // Configure SPK_RP and SPK_RN
  res =  WM8960_Write_Reg(0x28, 0x007F | 0x0100); //Left Speaker Volume
  delayMicroseconds(delay2);
  res += WM8960_Write_Reg(0x29, 0x007F | 0x0100); //Right Speaker Volume
  //if (res == 0) Serial.println("WM8960 Configure SPK_RP and SPK_RN"); else return res;
  delay(delay1);

  // Enable the OUTPUTS 0x0037 default
  res = WM8960_Write_Reg(0x31, 0x00F7); //Enable Class D Speaker Outputs
  //if (res == 0) Serial.println("WM8960 Enable Class D Speaker Outputs"); else return res;
  delay(delay1);

  // Configure DAC volume
  res =  WM8960_Write_Reg(0x0a, 0x00FF | 0x0100);
  delayMicroseconds(delay2);
  res += WM8960_Write_Reg(0x0b, 0x00FF | 0x0100);
  //if (res == 0) Serial.println("WM8960 Configure DAC volume"); else return res;
  delay(delay1);

  // 3D
  // WM8960_Write_Reg(0x10, 0x001F);
  
  // Configure MIXER
  res =  WM8960_Write_Reg(0x22, 1<<8 | 1<<7);
  delayMicroseconds(delay2);
  res += WM8960_Write_Reg(0x25, 1<<8 | 1<<7);
  //if (res == 0) Serial.println("WM8960 Configure MIXER"); else return res;
  delay(delay1);

  // Jack Detect
  // res =  WM8960_Write_Reg(0x18, 1<<6 | 0<<5);
  res =  WM8960_Write_Reg(0x18, 0x0000);
  delayMicroseconds(delay2);
  //res += WM8960_Write_Reg(0x17, 0x01C3);
  delayMicroseconds(delay2);
  //res += WM8960_Write_Reg(0x30, 0x0009); //0x000D,0x0005
  //if (res == 0) Serial.println("WM8960 Jack Detect"); else return res;
  delay(delay1);

  return 0;
}

/////////////////////////////////////////
// Setup
/////////////////////////////////////////
void setup() 
{
  uint8_t res;
  pinMode(led, OUTPUT);
  AudioMemory(12);

  Wire.begin();           // join i2c bus (address optional for master)
  //Serial.begin(9600);
  
  res = WM89060_Init();
  //if (res != 0) Serial.println(res);

}
 
/////////////////////////////////////////
// Main
/////////////////////////////////////////
void loop() 
{
  
  delay(100);           // wait 5 seconds for next scan
 

}
