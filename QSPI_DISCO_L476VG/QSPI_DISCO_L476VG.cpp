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

#include "QSPI_DISCO_L476VG.h"

// Constructor
QSPI_DISCO_L476VG::QSPI_DISCO_L476VG()
{
  BSP_QSPI_Init();
}

// Destructor
QSPI_DISCO_L476VG::~QSPI_DISCO_L476VG()
{
  BSP_QSPI_DeInit();
}

//=================================================================================================================
// Public methods
//=================================================================================================================

uint8_t QSPI_DISCO_L476VG::Init(void)
{
  return BSP_QSPI_Init();
}

uint8_t QSPI_DISCO_L476VG::DeInit(void)
{
  return BSP_QSPI_DeInit();
}

uint8_t QSPI_DISCO_L476VG::Read(uint8_t* pData, uint32_t ReadAddr, uint32_t Size)
{
  return BSP_QSPI_Read(pData, ReadAddr, Size);
}

uint8_t QSPI_DISCO_L476VG::Write(uint8_t* pData, uint32_t WriteAddr, uint32_t Size)
{
  return BSP_QSPI_Write(pData, WriteAddr, Size);
}

uint8_t QSPI_DISCO_L476VG::Erase_Block(uint32_t BlockAddress)
{
  return BSP_QSPI_Erase_Block(BlockAddress);
}

uint8_t QSPI_DISCO_L476VG::Erase_Sector(uint32_t Sector)
{
  return BSP_QSPI_Erase_Sector(Sector);
}

uint8_t QSPI_DISCO_L476VG::Erase_Chip(void)
{
  return BSP_QSPI_Erase_Chip();
}

uint8_t QSPI_DISCO_L476VG::GetStatus(void)
{
  return BSP_QSPI_GetStatus();
}

uint8_t QSPI_DISCO_L476VG::GetInfo(QSPI_Info* pInfo)
{
  return BSP_QSPI_GetInfo(pInfo);
}

uint8_t QSPI_DISCO_L476VG::EnableMemoryMappedMode(void)
{
  return BSP_QSPI_EnableMemoryMappedMode();
}

uint8_t QSPI_DISCO_L476VG::SuspendErase(void)
{
  return BSP_QSPI_SuspendErase();
}

uint8_t QSPI_DISCO_L476VG::ResumeErase(void)
{
  return BSP_QSPI_ResumeErase();
}

//=================================================================================================================
// Private methods
//=================================================================================================================
