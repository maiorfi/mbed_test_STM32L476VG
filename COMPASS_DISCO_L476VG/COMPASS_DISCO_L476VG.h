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

#ifndef __COMPASS_DISCO_L476VG_H
#define __COMPASS_DISCO_L476VG_H

#ifdef TARGET_DISCO_L476VG

#include "mbed.h"
#include "stm32l476g_discovery_compass.h"

/*
  Class to drive the Compass (LSM303 device with Accelerometer and Magnetometer)
  present on DISCO_L476VG board.

  Usage:

  #include "mbed.h"
  #include "COMPASS_DISCO_L476VG.h"
 
  COMPASS_DISCO_L476VG compass;
 
  int main()
  {
     int16_t MagBuffer[3];
     int16_t AccBuffer[3];
     while(1)
     {
         // Read acceleremoter and magnetometer values
         compass.AccGetXYZ(AccBuffer);
         compass.MagGetXYZ(MagBuffer);
         // Display values      
         printf("Acc X = %d\n", AccBuffer[0]);
         printf("Acc Y = %d\n", AccBuffer[1]);
         printf("Acc Z = %d\n", AccBuffer[2]);
         printf("Mag X = %d\n", MagBuffer[0]);
         printf("Mag Y = %d\n", MagBuffer[1]);
         printf("Mag Z = %d\n", MagBuffer[2]); 
     }
  }
*/
class COMPASS_DISCO_L476VG
{
  
public:
    //! Constructor
    COMPASS_DISCO_L476VG();

    //! Destructor
    ~COMPASS_DISCO_L476VG();

/**
  * @brief  Initialize the COMPASS.
  * @retval COMPASS_OK or COMPASS_ERROR
  */
  COMPASS_StatusTypeDef Init(void);

/**
  * @brief  DeInitialize the COMPASS.
  * @retval None.
  */
  void DeInit(void);

/**
  * @brief  Put the COMPASS in low power mode.
  * @retval None
  */
  void LowPower(void);
    
/**
  * @brief  Get XYZ acceleration values.
  * @param  pDataXYZ Pointer on 3 angular accelerations table with  
  *                  pDataXYZ[0] = X axis, pDataXYZ[1] = Y axis, pDataXYZ[2] = Z axis
  * @retval None
  */
  void AccGetXYZ(int16_t *pDataXYZ);

/**
  * @brief  Get XYZ magnetometer values.
  * @param  pDataXYZ Pointer on 3 magnetometer values table with
  *                  pDataXYZ[0] = X axis, pDataXYZ[1] = Y axis, pDataXYZ[2] = Z axis 
  * @retval None
  */    
  void MagGetXYZ(int16_t *pDataXYZ);

private:

};

#else
#error "This class must be used with DISCO_L476VG board only."
#endif // TARGET_DISCO_L476VG

#endif
