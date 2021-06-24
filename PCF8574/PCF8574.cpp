/***********************************************************
Author: Bernard Borredon
Date: 27 december 2011
Version: 1.0
************************************************************/
#include "PCF8574.h"

// Local object address
static PCF8574 *pt_pcf8574;

// User callback function address
static void (*_ptf)(uint8_t data,PCF8574 *o)= NULL;

// Callback
static void _InterruptCB(void)
{
  uint8_t data = 0;
  bitset<8> data_p,val;
  bitset<8> changed;
  uint8_t i;
  
  // Read PCF8574 port (to acknowledge interrupt)
  data = pt_pcf8574->read();
  
  // Set bits that have changed
  pt_pcf8574->getIntrData(data_p,changed);
  changed = 0;
  val = data;
  for(i = 0;i < 8;i++) {
     if(val[i] != data_p[i])
       changed[i] = 1;
  }
  pt_pcf8574->setIntrData(val,changed);
  
  // Call user callback, if any with port value
  if(_ptf != NULL)
    _ptf(data,pt_pcf8574);
}

// Constructor
PCF8574::PCF8574(PinName sda,PinName scl,uint8_t address, bool typeA) : _i2c(sda, scl)
{
  _errnum = PCF8574_NoError;
  _intr = NULL;
  pt_pcf8574 = NULL;
  _intr_data = 0xFF;
  _intr_bits_changed = 0;
  _intr_flag = 0;
  
  // Set the address
  if(typeA) {  // PCF8574A
    _PCF8574_address = 0x70;
  }
  else {       // PCF8574
    _PCF8574_address = 0x40;
  }
  
  _address = address;
  
  // Check address validity (between 0 and 7)
  if(address > 7) {
    _errnum = PCF8574_BadAddress;
  }
  
  // Shift address
  _address = _address << 1;
  
  // Set I2C frequency (100 MHz)
  _i2c.frequency(100000);
}

// Read IO port
uint8_t PCF8574::read(void)
{
  uint8_t data;
  int ack;
  
  // Check error
  if(_errnum) 
    return(0);
  
  // No error
  _errnum = PCF8574_NoError;
    
  // Read port
  ack = _i2c.read(_PCF8574_address | _address,(char *)&data,sizeof(data));
  
  // Check error
  if(ack != 0) {
    _errnum = PCF8574_I2cError;
  }
  
  return(data);
}

// Write IO port
void PCF8574::write(uint8_t data)
{
  int ack;
  
  // Check error
  if(_errnum) 
    return;
 
  // No error   
  _errnum = PCF8574_NoError;
    
  // Write port
  ack = _i2c.write(_PCF8574_address | _address,(char *)&data,sizeof(data));
  
  // Check error
  if(ack != 0) {
    _errnum = PCF8574_I2cError;
  }
}

// Write IO pin
void PCF8574::write(uint8_t pin, uint8_t value)
{
  uint8_t data;
  
  // Read IO port value
  data = read();
  
  // Change bit pin
  if(value)
    BIT_SET(data,pin);
  else
    BIT_CLEAR(data,pin);
    
  // Write new IO port data
  write(data);
}

// Write to IO pins with a bitset and a bitset mask
void PCF8574::write(bitset<8> data, bitset<8> mask)
{
  uint8_t i;
  bitset<8> nd;
  
  nd = (bitset<8>) read();
  for(i = 0;i < 8;i++) {
     if(mask[i])
       nd[i] = data[i];
  }
  
  write((uint8_t) nd.to_ulong());
}

// Set pin and callback interrupt
void PCF8574::interrupt(PinName intr,void (*ftpr)(uint8_t, PCF8574 *))
{
  
  // Check error
  if(_errnum) 
    return;
    
  // No error
  _errnum = PCF8574_NoError;
    
  // Create InterruptIn instance if needed
  if(_intr == NULL) {
    _intr = new InterruptIn(intr);
    
    // Set InterruptIn callback
    if(_intr != NULL)
      _intr->fall(_InterruptCB);
    else
      _errnum = PCF8574_IntrError;
  }
  
  _ptf = ftpr; // User callback address
  pt_pcf8574 = this; // Object instance
  _intr_data = read(); // IO port value
  _intr_flag = 1; // Function called
}

// Get current error number
uint8_t PCF8574::getError(void)
{ 
  return(_errnum);
}

// Memorize IO port value and bits that have changed when interrupt
 void PCF8574::setIntrData(bitset<8> data, bitset<8> changed)
 {
   _intr_data = data;
   _intr_bits_changed = changed;
 }
 
 // Get IO port value and bits that have changed when interrupt
 void PCF8574::getIntrData(bitset<8>& data, bitset<8>& changed)
 {
   data = _intr_data;
   changed = _intr_bits_changed;
 }

// Redefine () operator
PCF8574::operator uint8_t() 
{ 
  return read(); 
}
 
// Redefine = operator
PCF8574 &PCF8574::operator=(uint8_t data)
{
  write((int)data);
  
  return(*this);
}