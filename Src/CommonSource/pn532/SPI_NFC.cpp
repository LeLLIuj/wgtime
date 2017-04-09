/*
 * \file API_NFC.cpp
 *
 *  Created on: Apr 27, 2016
 *  \author Remko Welling
 *          remko@rfsee.nl
 *
 *
 *	\version 0.8	Initial release for the HACOX project team for code review
 *
 *
 */

#include <stdint.h>
#include <string.h>

#include "spi.h"

#include "SPI_NFC.h"

SPI_NFC::SPI_NFC()
{

}

SPI_NFC::~SPI_NFC()
{

}


uint8_t SPI_NFC::getByte(void)
{
  return getByte_SPI2();
}


void SPI_NFC::sendByte(uint8_t byte)
{
  sendByte_SPI2(byte);
}


void SPI_NFC::setSelect(const bool state)
{
  setSelect_SPI2(state?1:0);
}


bool SPI_NFC::isIRQ(void)
{
  return isIRQ_SPI2();
}
