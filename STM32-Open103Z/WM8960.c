#include "WM8960.h"
#include "i2c.h"
#include "i2s.h"
#include "stdio.h"

#define WM8960_ADDRESS  0x1a

uint32_t AudioTotalSize;  /* This variable holds the total size of the audio file */
uint32_t AudioRemSize;    /* This variable holds the remaining data in audio file */
uint16_t *CurrentPos;     /* This variable holds the current position of audio pointer */

//resgister value
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

/**
  * @brief  Write register of WM8960.
  * @param  reg: The number of resigter which to be read.
  * @param  dat: The data which will be writeen to the register.
  * @retval The value of regsiter.
  */
uint8_t WM8960_Write_Reg(uint8_t reg, uint16_t dat)  {
  
  uint8_t res,I2C_Data[2];
  
  I2C_Data[0] = (reg<<1)|((uint8_t)((dat>>8)&0x0001));  //RegAddr
  I2C_Data[1] = (uint8_t)(dat&0x00FF);                  //RegValue
  
  res = HAL_I2C_Master_Transmit(&hi2c2,(WM8960_ADDRESS<<1),I2C_Data,2,10);
  if(res == HAL_OK)
    WM8960_REG_VAL[reg] = dat;
  
  return res;
}

/**
  * @brief  Read register of WM8960.
  * @param  reg: The number of resigter which to be read.
  * @retval The value of regsiter.
  */
uint16_t WM8960_Read_Reg(uint8_t reg) {
  
  return WM8960_REG_VAL[reg];
}


/**
  * @brief  Initialize WM8960 device.
  * @param  None
  * @retval None
  */
uint8_t WM89060_Init(void)  {

  uint8_t res;
  
  //Reset Device
  res = WM8960_Write_Reg(0x0f, 0x0000);
  if(res != 0)
    return res;
  else
    printf("WM8960 reset completed !!\r\n");
  
  //Set Power Source
  res =  WM8960_Write_Reg(0x19, 1<<8 | 1<<7 | 1<<6);
  res += WM8960_Write_Reg(0x1A, 1<<8 | 1<<7 | 1<<6 | 1<<5 | 1<<4 | 1<<3);
  res += WM8960_Write_Reg(0x2F, 1<<3 | 1<<2);
  if(res != 0)  {
    printf("Source set fail !!\r\n");
    printf("Error code: %d\r\n",res);
    return res;
  }
  
  //Configure clock
  //MCLK->div1->SYSCLK->DAC/ADC sample Freq = 25MHz(MCLK)/2*256 = 48.8kHz
  WM8960_Write_Reg(0x04, 0x0000);
  
  //Configure ADC/DAC
  WM8960_Write_Reg(0x05, 0x0000);
  
  //Configure audio interface
  //I2S format 16 bits word length
  WM8960_Write_Reg(0x07, 0x0002);
  
  //Configure HP_L and HP_R OUTPUTS
  WM8960_Write_Reg(0x02, 0x006F | 0x0100);  //LOUT1 Volume Set
  WM8960_Write_Reg(0x03, 0x006F | 0x0100);  //ROUT1 Volume Set
  
  //Configure SPK_RP and SPK_RN
  WM8960_Write_Reg(0x28, 0x007F | 0x0100); //Left Speaker Volume
  WM8960_Write_Reg(0x29, 0x007F | 0x0100); //Right Speaker Volume
  
  //Enable the OUTPUTS
  WM8960_Write_Reg(0x31, 0x00F7); //Enable Class D Speaker Outputs
  
  //Configure DAC volume
  WM8960_Write_Reg(0x0a, 0x00FF | 0x0100);
  WM8960_Write_Reg(0x0b, 0x00FF | 0x0100);
  
  //3D
//  WM8960_Write_Reg(0x10, 0x001F);
  
  //Configure MIXER
  WM8960_Write_Reg(0x22, 1<<8 | 1<<7);
  WM8960_Write_Reg(0x25, 1<<8 | 1<<7);
  
  //Jack Detect
  WM8960_Write_Reg(0x18, 1<<6 | 0<<5);
  WM8960_Write_Reg(0x17, 0x01C3);
  WM8960_Write_Reg(0x30, 0x0009);//0x000D,0x0005
  
  return 0;
}

/**
  * @brief  Starts playing audio stream from a data buffer for a determined size. 
  * @param  pBuffer: Pointer to the buffer 
  * @param  Size: Number of audio data BYTES.
  * @retval None
  */
uint32_t AudioFlashPlay(uint16_t* pBuffer, uint32_t FullSize, uint32_t StartAdd)
{ 
  AUDIO_Play((uint16_t*)pBuffer, (uint32_t)(FullSize - StartAdd));
  
  return 0;
}

/**
  * @brief  Starts playing audio stream from a data buffer for a determined size. 
  * @param  pBuffer: Pointer to the buffer 
  * @param  Size: Number of audio data BYTES.
  * @retval None
  */
uint32_t AUDIO_Play(uint16_t* pBuffer, uint32_t Size)
{
  uint8_t res;
  /* Set the total number of data to be played (count in half-word) */
  AudioTotalSize = Size;
  
  /* Update the Media layer and enable it for play */
  res = HAL_I2S_Transmit_DMA(&hi2s2,pBuffer, (uint32_t)(DMA_MAX(Size/2)));
  printf("DMA_MAX(Size/2): %d\r\n",DMA_MAX(Size/2));
  
  /* Update the remaining number of data to be played */
  AudioRemSize = (Size/2) - DMA_MAX(AudioTotalSize);
  printf("AudioRemSize = %d\r\n",AudioRemSize);
  
  /* Update the current audio pointer position */
  CurrentPos = pBuffer + DMA_MAX(AudioTotalSize);
  
  return res;
}

