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

#include "GYRO_DISCO_L476VG.h"

// Constructor
GYRO_DISCO_L476VG::GYRO_DISCO_L476VG()
{
    BSP_GYRO_Init();
}

// Destructor
GYRO_DISCO_L476VG::~GYRO_DISCO_L476VG()
{
    BSP_GYRO_DeInit();
}

//=================================================================================================================
// Public methods
//=================================================================================================================

uint8_t GYRO_DISCO_L476VG::Init(void)
{
    return BSP_GYRO_Init();
}

void GYRO_DISCO_L476VG::DeInit(void)
{
    BSP_GYRO_DeInit();
}

void GYRO_DISCO_L476VG::LowPower(void)
{
    BSP_GYRO_LowPower();
}

void GYRO_DISCO_L476VG::Reset(void)
{
    BSP_GYRO_Reset();
}

uint8_t GYRO_DISCO_L476VG::ReadID(void)
{
    return BSP_GYRO_ReadID();
}

void GYRO_DISCO_L476VG::ITConfig(GYRO_InterruptConfigTypeDef *pIntConfigStruct)
{
    BSP_GYRO_ITConfig(pIntConfigStruct);
}

void GYRO_DISCO_L476VG::EnableIT(uint8_t IntPin)
{
    BSP_GYRO_EnableIT(IntPin);
}

void GYRO_DISCO_L476VG::DisableIT(uint8_t IntPin)
{
    BSP_GYRO_DisableIT(IntPin);
}

void GYRO_DISCO_L476VG::GetXYZ(float* pfData)
{
    BSP_GYRO_GetXYZ(pfData);
}

//=================================================================================================================
// Private methods
//=================================================================================================================


