/**
  ******************************************************************************
  * @file    picture.h
  * @author  MCD Application Team
  * @version V0.1.0
  * @date    18-June-2014
  * @brief   Pictures + Graphic extract from XBM files
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PICTURE_H
#define __PICTURE_H

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx.h"

/* Exported types ------------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/



/* External variables --------------------------------------------------------*/
/** 
 * @brief Avoids to stuck the program
 */
/*����ͼ��,�ߴ�16X16*/
const uint8_t picture_1[] = {
0xff, 0xff, 0x00, 
0x00, 0xff, 0xff,
0x00, 0x00, 0xff, 
0xff, 0x00, 0x00, 
0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 
0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 
}; 

const uint8_t picture_2[] = {0
 
}; 

const uint8_t picture_3[] = {0

}; 
 
const uint8_t picture_4[]= {0

}; 

#endif  /*__PICTURE_H*/

/******************* (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
