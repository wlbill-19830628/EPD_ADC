/**
  ******************************************************************************
  * @file    gde021a1.c
  * @author  王亮
  * @brief   由ST官方驱动代码修改，HAL库改为LL库，
	*          SPI只使用MOSI,初始化代码由Cubemx生成，速度2MHz.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2014 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gde021a1.h"
#include "main.h"

/** @addtogroup BSP
  * @{
  */
/********************************* 连接 EPD ***********************************/

/**
  * @brief  初始化EPD相关接口引脚.
  * @param  None
  * @retval None
  */
void EPD_IO_Init(void)
{
    LL_GPIO_InitTypeDef GPIO_InitStruct;

    /* 使能相关GPIO时钟*/
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);

    /* 配置CS脚: ePD1_CS_Pin */
    GPIO_InitStruct.Pin = ePD1_CS_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    LL_GPIO_Init(ePD1_CS_GPIO_Port, &GPIO_InitStruct);

    /* 配置DC脚: ePD1_D_C_Pin */
    GPIO_InitStruct.Pin = ePD1_D_C_Pin;
    LL_GPIO_Init(ePD1_D_C_GPIO_Port, &GPIO_InitStruct);

    /* 配置RESET脚:ePD1_RESET_Pin*/
    GPIO_InitStruct.Pin = ePD1_RESET_Pin;
    LL_GPIO_Init(ePD1_RESET_GPIO_Port, &GPIO_InitStruct);

    /* 配置PWR脚: ePD1_PWR_ENn_Pin*/
    GPIO_InitStruct.Pin = ePD1_PWR_ENn_Pin;
    LL_GPIO_Init(ePD1_PWR_ENn_GPIO_Port, &GPIO_InitStruct);

    /* 配置BUSY脚: */
    GPIO_InitStruct.Pin = ePD1_BUSY_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
    LL_GPIO_Init(ePD1_BUSY_GPIO_Port, &GPIO_InitStruct);

    /* 使能显示 */
    EPD_PWR_LOW();

    /* 置位复位片选信号*/
    EPD_CS_LOW();
    EPD_CS_HIGH();

    /* EPD复位脚处理 */
    EPD_RESET_HIGH();
    LL_mDelay(10);
}


/**
  * @brief  SPI写1个字节到设备
  * @param  Value: 写入值
  * @retval None
  */
static void SPIx_Write(uint8_t Value)
{

    uint16_t wait_cnt=0;
	  //等待之前的数据发送完成
    while(!LL_SPI_IsActiveFlag_TXE(SPI1)) {};       
    
	  //发送当前数据
    LL_SPI_TransmitData8(SPI1,Value);
		
		//由于没有miso信号确认，发送完毕1字节数据后需要设置合理延时等待从机处理，大概20US左右，wait_cnt计数周期与主频有关。
    for(wait_cnt=0; wait_cnt<50; wait_cnt++)   
    {
        __NOP();
    }

}


///**
//  * @brief  SPI Read 4 bytes from device.
//  * @param  ReadSize Number of bytes to read (max 4 bytes)
//  * @retval Value read on the SPI
//  */
//static uint32_t SPIx_Read(void)
//{
//  HAL_StatusTypeDef status = HAL_OK;
//  uint32_t readvalue = 0;
//  uint32_t writevalue = 0xFFFFFFFF;
//
//  status = HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) &writevalue, (uint8_t*) &readvalue, 1, SPIx_TIMEOUT_MAX);

//  /* Check the communication status */
//  if(status != HAL_OK)
//  {
//    /* Re-Initiaize the BUS */
//    SPIx_Error();
//  }

//  return readvalue;
//}


/**
  * @brief  Write register value.
  * @param  None
  * @retval None
  */
void EPD_IO_WriteData(uint16_t RegValue)
{
    /* Reset EPD control line CS */
    EPD_CS_LOW();

    /* Set EPD data/command line DC to High */
    EPD_DC_HIGH();

    /* Send Data */
    SPIx_Write(RegValue);

    /* Deselect: Chip Select high */
    EPD_CS_HIGH();


}

/**
  * @brief  Writes command to selected EPD register.
  * @param  Reg: Address of the selected register.
  * @retval None
  */
void EPD_IO_WriteReg(uint8_t Reg)
{
    /* Reset EPD control line CS */
    EPD_CS_LOW();

    /* Set EPD data/command line DC to Low */
    EPD_DC_LOW();

    /* Send Command */
    SPIx_Write(Reg);

    /* Deselect: Chip Select high */
    EPD_CS_HIGH();
}

///**
//  * @brief  Reads data from EPD data register.
//  * @param  None
//  * @retval Read data.
//  */
//uint16_t EPD_IO_ReadData(void)
//{
//    /* Reset EPD control line CS */
//    EPD_CS_LOW();

//    /* Deselect: Chip Select high */
//    EPD_CS_HIGH();

//    /* Send Data */
//    return SPIx_Read();
//}
/** @addtogroup Components
  * @{
  */

/** @addtogroup GDE021A1
  * @brief      This file provides a set of functions needed to drive the
  *             GDE021A1 EPD (E Paper Display).
  * @{
  */

/** @defgroup GDE021A1_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */

/** @defgroup GDE021A1_Private_Defines
  * @{
  */
/**
  * @}
  */

/** @defgroup GDE021A1_Private_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup GDE021A1_Private_Variables
  * @{
  */

/* Look-up table for the epaper (90 bytes) */
const unsigned char WF_LUT[]= {
    0x82,0x00,0x00,0x00,0xAA,0x00,0x00,0x00,
    0xAA,0xAA,0x00,0x00,0xAA,0xAA,0xAA,0x00,
    0x55,0xAA,0xAA,0x00,0x55,0x55,0x55,0x55,
    0xAA,0xAA,0xAA,0xAA,0x55,0x55,0x55,0x55,
    0xAA,0xAA,0xAA,0xAA,0x15,0x15,0x15,0x15,
    0x05,0x05,0x05,0x05,0x01,0x01,0x01,0x01,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x41,0x45,0xF1,0xFF,0x5F,0x55,0x01,0x00,
    0x00,0x00,
};

EPD_DrvTypeDef   gde021a1_drv =
{
    gde021a1_Init,
    gde021a1_WritePixel,
    gde021a1_SetDisplayWindow,
    gde021a1_RefreshDisplay,
    gde021a1_CloseChargePump,
    gde021a1_GetEpdPixelWidth,
    gde021a1_GetEpdPixelHeight,
    gde021a1_DrawImage,
};

/**
* @}
*/

/** @defgroup GDE021A1_Private_FunctionPrototypes
  * @{
  */
/**
* @}
*/

/** @defgroup GDE021A1_Private_Functions
  * @{
  */

/**
  * @brief  Initialize the GDE021A1 EPD Component.
  * @param  None
  * @retval None
  */
void gde021a1_Init(void)
{
    uint8_t nb_bytes = 0;

    /* Initialize the GDE021A11 */
    EPD_IO_Init();

    EPD_IO_WriteReg(EPD_REG_16);  /* Deep sleep mode disable */
    EPD_IO_WriteData(0x00);
    EPD_IO_WriteReg(EPD_REG_17);  /* Data Entry Mode Setting */
    EPD_IO_WriteData(0x03);
    EPD_IO_WriteReg(EPD_REG_68);  /* Set the RAM X start/end address */
    EPD_IO_WriteData(0x00);       /* RAM X address start = 00h */
    EPD_IO_WriteData(0x11);       /* RAM X adress end = 11h (17 * 4pixels by address = 72 pixels) */
    EPD_IO_WriteReg(EPD_REG_69);  /* Set the RAM Y start/end address */
    EPD_IO_WriteData(0x00);       /* RAM Y address start = 0 */
    EPD_IO_WriteData(0xAB);       /* RAM Y adress end = 171 */
    EPD_IO_WriteReg(EPD_REG_78);  /* Set RAM X Address counter */
    EPD_IO_WriteData(0x00);
    EPD_IO_WriteReg(EPD_REG_79);  /* Set RAM Y Address counter */
    EPD_IO_WriteData(0x00);
    EPD_IO_WriteReg(EPD_REG_240); /* Booster Set Internal Feedback Selection */
    EPD_IO_WriteData(0x1F);
    EPD_IO_WriteReg(EPD_REG_33);  /* Disable RAM bypass and set GS transition to GSA = GS0 and GSB = GS3 */
    EPD_IO_WriteData(0x03);
    EPD_IO_WriteReg(EPD_REG_44);  /* Write VCOMregister */
    EPD_IO_WriteData(0xA0);
    EPD_IO_WriteReg(EPD_REG_60);  /* Border waveform */
    EPD_IO_WriteData(0x64);
    EPD_IO_WriteReg(EPD_REG_50);  /* Write LUT register */

    for (nb_bytes=0; nb_bytes<90; nb_bytes++)
    {
        EPD_IO_WriteData(WF_LUT[nb_bytes]);
    }
}

/**
  * @brief  Writes 4 dots.
  * @param  HEX_Code: specifies the Data to write.
  * @retval None
  */
void gde021a1_WritePixel(uint8_t HEX_Code)
{
    /* Prepare the register to write data on the RAM */
    EPD_IO_WriteReg(EPD_REG_36);

    /* Send the data to write */
    EPD_IO_WriteData(HEX_Code);
}

/**
  * @brief  Sets a display window.
  * @param  Xpos: specifies the X bottom left position.
  * @param  Ypos: specifies the Y bottom left position.
  * @param  Width: display window width.
  * @param  Height: display window height.
  * @retval None
*/
void gde021a1_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Width, uint16_t Height)
{
    /* Set Y position and the height */
    EPD_IO_WriteReg(EPD_REG_68);
    EPD_IO_WriteData(Ypos);
    EPD_IO_WriteData(Height);
    /* Set X position and the width */
    EPD_IO_WriteReg(EPD_REG_69);
    EPD_IO_WriteData(Xpos);
    EPD_IO_WriteData(Width);
    /* Set the height counter */
    EPD_IO_WriteReg(EPD_REG_78);
    EPD_IO_WriteData(Ypos);
    /* Set the width counter */
    EPD_IO_WriteReg(EPD_REG_79);
    EPD_IO_WriteData(Xpos);
}

/**
  * @brief  Gets the EPD pixel Width.
  * @param  None
  * @retval The EPD Pixel Width
  */
uint16_t gde021a1_GetEpdPixelWidth(void)
{
    return GDE021A1_EPD_PIXEL_WIDTH;
}

/**
  * @brief  Gets the EPD pixel Height.
  * @param  None
  * @retval The EPD Pixel Height
  */
uint16_t gde021a1_GetEpdPixelHeight(void)
{
    return GDE021A1_EPD_PIXEL_HEIGHT;
}

/**
  * @brief  Writes to the selected EPD register.
  * @param  EPD_Reg: Address of the selected register.
  * @param  EPD_RegValue: value to write to the selected register.
  * @retval None
  */
void gde021a1_WriteReg(uint8_t EPD_Reg, uint8_t EPD_RegValue)
{
    EPD_IO_WriteReg(EPD_Reg);

    EPD_IO_WriteData(EPD_RegValue);
}

///**
//  * @brief  Reads the selected EPD Register.
//  * @param  EPD_Reg: address of the selected register
//  * @retval EPD Register Value
//  */
//uint8_t gde021a1_ReadReg(uint8_t EPD_Reg)
//{
//    /* Write 8-bit Index (then Read Reg) */
//    EPD_IO_WriteReg(EPD_Reg);

//    /* Read 8-bit Reg */
//    return (EPD_IO_ReadData());
//}

/**
  * @brief  Activates display update sequence.
  * @param  None
  * @retval None
  */
void gde021a1_RefreshDisplay(void)
{
    /* Write on the Display update control register */
    EPD_IO_WriteReg(EPD_REG_34);

    /* Display update data sequence option */
    EPD_IO_WriteData(0xC4);

    /* Launching the update: Nothing should interrupt this sequence in order
       to avoid display corruption */
    EPD_IO_WriteReg(EPD_REG_32);
}

/**
  * @brief  Disables the clock and the charge pump.
  * @param  None
  * @retval None
  */
void gde021a1_CloseChargePump(void)
{
    /* Write on the Display update control register */
    EPD_IO_WriteReg(EPD_REG_34);

    /* Disable CP then Disable Clock signal */
    EPD_IO_WriteData(0x03);

    /* Launching the update: Nothing should interrupt this sequence in order
       to avoid display corruption */
    EPD_IO_WriteReg(EPD_REG_32);
}

/**
  * @brief  Displays picture..
  * @param  pdata: picture address.
  * @param  Xpos:  Image X position in the EPD
  * @param  Ypos:  Image Y position in the EPD
  * @param  Xsize: Image X size in the EPD
  * @note   Xsize have to be a multiple of 4
  * @param  Ysize: Image Y size in the EPD
  * @retval None
  */
void gde021a1_DrawImage(uint16_t Xpos, uint16_t Ypos, uint16_t Xsize, uint16_t Ysize, uint8_t *pdata)
{
    uint32_t i, j = 0;
    uint8_t pixels_4 = 0;
    uint8_t pixels_4_grey[4] = {0};
    uint8_t nb_4_pixels, data_res = 0;

    /* Prepare the register to write data on the RAM */
    EPD_IO_WriteReg(EPD_REG_36);

    /* X size is a multiple of 8 */
    if ((Xsize % 8) == 0)
    {
        for (i= 0; i< ((((Ysize) * (Xsize/4)))/2) ; i++)
        {
            /* Get the current data */
            pixels_4 = pdata[i];
            if (pixels_4 !=0)
            {
                /* One byte read codes 8 pixels in 1-bit bitmap */
                for (nb_4_pixels = 0; nb_4_pixels < 2; nb_4_pixels++)
                {
                    /* Processing 8 pixels */
                    /* Preparing the 4 pixels coded with 4 grey level per pixel
                       from a monochrome xbm file */
                    for (j= 0; j<4; j++)
                    {
                        if (((pixels_4) & 0x01) == 1)
                        {
                            /* Two LSB is coding black in 4 grey level */
                            pixels_4_grey[j] &= 0xFC;
                        }
                        else
                        {
                            /* Two LSB is coded white in 4 grey level */
                            pixels_4_grey[j] |= 0x03;
                        }
                        pixels_4 = pixels_4 >> 1;
                    }

                    /* Processing 4 pixels */
                    /* Format the data to have the Lower pixel number sent on the MSB for the SPI to fit with the RAM
                       EPD topology */
                    data_res = pixels_4_grey[0] << 6 | pixels_4_grey[1] << 4 | pixels_4_grey[2] << 2 | pixels_4_grey[3] << 0;

                    /* Send the data to the EPD's RAM through SPI */
                    EPD_IO_WriteData(data_res);
                }
            }
            else
            {
                /* 1 byte read from xbm files is equivalent to 8 pixels in the
                   other words 2 bytes to be transferred */
                EPD_IO_WriteData(0xFF);
                EPD_IO_WriteData(0xFF);
            }
        }
    }

    /* X size is a multiple of 4 */
    else
    {
        for (i= 0; i< ((((Ysize) * ((Xsize/4)+1))/2)) ; i++)
        {
            /* Get the current data */
            pixels_4 = pdata[i];
            if (((i+1) % (((Xsize/4)+1)/2)) != 0)
            {
                if (pixels_4 !=0)
                {
                    /* One byte read codes 8 pixels in 1-bit bitmap */
                    for (nb_4_pixels = 0; nb_4_pixels < 2; nb_4_pixels++)
                    {
                        /* Processing 8 pixels */
                        /* Preparing the 4 pixels coded with 4 grey level per pixel
                           from a monochrome xbm file */
                        for (j= 0; j<4; j++)
                        {
                            if (((pixels_4) & 0x01) == 1)
                            {
                                /* Two LSB is coding black in 4 grey level */
                                pixels_4_grey[j] &= 0xFC;
                            }
                            else
                            {
                                /* Two LSB is coded white in 4 grey level */
                                pixels_4_grey[j] |= 0x03;
                            }
                            pixels_4 = pixels_4 >> 1;
                        }

                        /* Processing 4 pixels */
                        /* Format the data to have the Lower pixel number sent on the MSB for the SPI to fit with the RAM
                           EPD topology */
                        data_res = pixels_4_grey[0] << 6 | pixels_4_grey[1] << 4 | pixels_4_grey[2] << 2 | pixels_4_grey[3] << 0;

                        /* Send the data to the EPD's RAM through SPI */
                        EPD_IO_WriteData(data_res);
                    }
                }
                else if (pixels_4 == 0)
                {
                    /* One byte read from xbm files is equivalent to 8 pixels in the
                       other words Two bytes to be transferred */
                    EPD_IO_WriteData(0xFF);
                    EPD_IO_WriteData(0xFF);
                }
            }

            else if (((i+1) % (((Xsize/4)+1)/2)) == 0)
            {
                if (pixels_4 !=0xf0)
                {
                    /* Processing 8 pixels */
                    /* Preparing the 4 pixels coded with 4 grey level per pixel
                       from a monochrome xbm file */
                    for (j= 0; j<4; j++)
                    {
                        if (((pixels_4) & 0x01) == 1)
                        {
                            /* 2 LSB is coding black in 4 grey level */
                            pixels_4_grey[j] &= 0xFC;
                        }
                        else
                        {
                            /* 2 LSB is coded white in 4 grey level */
                            pixels_4_grey[j] |= 0x03;
                        }
                        pixels_4 = pixels_4 >> 1;
                    }

                    /* Processing 4 pixels */
                    /* Format the data to have the Lower pixel number sent on the MSB for the SPI to fit with the RAM
                       EPD topology */
                    data_res = pixels_4_grey[0] << 6 | pixels_4_grey[1] << 4 | pixels_4_grey[2] << 2 | pixels_4_grey[3] << 0;

                    /* Send the data to the EPD's RAM through SPI */
                    EPD_IO_WriteData(data_res);
                }
                else if (pixels_4 == 0xf0)
                {
                    /* One byte to be transferred */
                    EPD_IO_WriteData(0xFF);
                }
            }
        }
    }
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
