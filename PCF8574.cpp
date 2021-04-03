/*
 *  © 2021, Neil McKechnie. All rights reserved.
 *  
 *  This file is part of DCC++EX API
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "IODevice.h"
#include "DIAG.h"
#include "I2CManager.h"

// Constructor
PCF8574::PCF8574(VPIN firstID, int nPins, uint8_t I2CAddress) {
  _firstID = firstID;
  _nPins = max(nPins, 8);
  _I2CAddress = I2CAddress;
}

void PCF8574::create(VPIN firstID, int nPins, uint8_t I2CAddress) {
  addDevice(new PCF8574(firstID, nPins, I2CAddress));
}

void PCF8574::_begin() {
  I2CManager.begin();
  I2CManager.setClock(100000);  // Only supports slow clock
}

// Device-specific write function.
void PCF8574::_write(VPIN vpin, int value) {
  int pin = vpin -_firstID;
  DIAG(F("PCF8574 Write I2C:x%x Pin:%d Value:%d"), (int)_I2CAddress, (int)vpin, value);
  uint8_t mask = 1 << pin;
  if (value) 
    _currentPortState |= mask;
  else
    _currentPortState &= ~mask;
  I2CManager.write(_I2CAddress, 1, _currentPortState);
}

// Device-specific read function.
int PCF8574::_read(VPIN vpin) {
  byte inBuffer;
  int result;
  int pin = vpin-_firstID;
  uint8_t mask = 1 << pin;
  // To enable the pin to be read, write a '1' to it first.  The connected
  // equipment should pull the input down to ground.
  _currentPortState |= mask;
  I2CManager.read(_I2CAddress, &inBuffer, 1, 1, _currentPortState);
  if (inBuffer & mask) 
    result = 1;
  else
    result = 0;
  //DIAG(F("PCF8574 Read I2C:x%x Pin:%d Value:%d"), (int)_I2CAddress, (int)pin, result);
  return result;
}

void PCF8574::_display() {
  DIAG(F("PCF8574 Addr:x%x VPins:%d-%d"), (int)_I2CAddress, (int)_firstID, (int)_firstID+_nPins-1);
}

