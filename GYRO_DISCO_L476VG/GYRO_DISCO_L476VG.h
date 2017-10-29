/* Copyright (c) 2010-2011 mbed.org, MIT License
*
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software
* and associated documentation files (the "Software"), to deal in the Software without
* restriction, including without limitation the rights to use, copy, modify, merge, publish,
* distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all copies or
* substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef __GYRO_DISCO_L476VG_H
#define __GYRO_DISCO_L476VG_H

#ifdef TARGET_DISCO_L476VG

#include "mbed.h"
#include "stm32l476g_discovery_gyroscope.h"

/*
  Class to drive the Gyroscope (L3GD20 device)
  present on DISCO_L476VG board.

  Usage:
 
  #include "mbed.h"
  #include "GYRO_DISCO_L476VG.h"
 
  GYRO_DISCO_L476VG gyro;
 
  int main()
  {
    float GyroBuffer[3];
    while(1) {
        // Read Gyroscope values
        gyro.GetXYZ(GyroBuffer);
        // Display values      
        printf("X = %f\n", GyroBuffer[0]);
        printf("Y = %f\n", GyroBuffer[1]);
        printf("Z = %f\n", GyroBuffer[2]);
        wait(1);
    }
  }
*/
class GYRO_DISCO_L476VG
{
  
public:
    //! Constructor
    GYRO_DISCO_L476VG();

    //! Destructor
    ~GYRO_DISCO_L476VG();

/**
  * @brief  Initialize Gyroscope.
  * @retval GYRO_OK or GYRO_ERROR
  */
    uint8_t Init(void);

/**
  * @brief  DeInitialize Gyroscope.
  * @retval None
  */
    void DeInit(void);

/**
  * @brief  Put Gyroscope in low power mode.
  * @retval None
  */
    void LowPower(void);
    
/**
  * @brief  Reboot memory content of Gyroscope.
  * @retval None
  */    
    void Reset(void);
    
/**
  * @brief  Read ID of Gyroscope component.
  * @retval ID
  */    
    uint8_t ReadID(void);
    
/**
  * @brief  Configure Gyroscope interrupts (INT1 or INT2).
  * @param  pIntConfig: pointer to a GYRO_InterruptConfigTypeDef 
  *         structure that contains the configuration setting for the L3GD20 Interrupt.
  * @retval None
  */    
    void ITConfig(GYRO_InterruptConfigTypeDef *pIntConfigStruct);
    
/**
  * @brief  Enable Gyroscope interrupts (INT1 or INT2).
  * @param  IntPin: Interrupt pin 
  *      This parameter can be: 
  *        @arg L3GD20_INT1
  *        @arg L3GD20_INT2
  * @retval None
  */    
    void EnableIT(uint8_t IntPin);
    
/**
  * @brief  Disable Gyroscope interrupts (INT1 or INT2).
  * @param  IntPin: Interrupt pin 
  *      This parameter can be: 
  *        @arg L3GD20_INT1
  *        @arg L3GD20_INT2
  * @retval None
  */    
    void DisableIT(uint8_t IntPin);
    
/**
  * @brief  Get XYZ angular acceleration from the Gyroscope.
  * @param  pfData: pointer on floating array         
  * @retval None
  */    
    void GetXYZ(float* pfData);

private:

};

#else
#error "This class must be used with DISCO_L476VG board only."
#endif // TARGET_DISCO_L476VG

#endif
