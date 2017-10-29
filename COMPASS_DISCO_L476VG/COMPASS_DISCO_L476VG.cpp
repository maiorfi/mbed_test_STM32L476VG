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

#include "COMPASS_DISCO_L476VG.h"

// Constructor
COMPASS_DISCO_L476VG::COMPASS_DISCO_L476VG()
{
    BSP_COMPASS_Init();
}

// Destructor
COMPASS_DISCO_L476VG::~COMPASS_DISCO_L476VG()
{
    BSP_COMPASS_DeInit();
}

//=================================================================================================================
// Public methods
//=================================================================================================================

COMPASS_StatusTypeDef COMPASS_DISCO_L476VG::Init(void)
{
    return BSP_COMPASS_Init();
}

void COMPASS_DISCO_L476VG::DeInit(void)
{
    BSP_COMPASS_DeInit();
}

void COMPASS_DISCO_L476VG::LowPower(void)
{
    BSP_COMPASS_LowPower();
}

void COMPASS_DISCO_L476VG::AccGetXYZ(int16_t *pDataXYZ)
{
    BSP_COMPASS_AccGetXYZ(pDataXYZ);
}

void COMPASS_DISCO_L476VG::MagGetXYZ(int16_t *pDataXYZ)
{
    BSP_COMPASS_MagGetXYZ(pDataXYZ);
}

//=================================================================================================================
// Private methods
//=================================================================================================================

