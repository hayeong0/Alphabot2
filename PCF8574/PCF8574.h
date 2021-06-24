#ifndef PCF8574__H_
#define PCF8574__H_

// Includes
#include <string> 
#include <bitset>

#include "mbed.h"

// Example
/*
#include <string>

#include "mbed.h"
#include "PCF8574.h"

#define PCF8574_ADDR 0    // I2c PCF8574 address is 0x00

static void myerror(std::string msg)
{
  printf("Error %s\n",msg.c_str());
  exit(1);
}

void pcf8574_it(uint8_t data, PCF8574 *o)
{
  printf("PCF8574 interrupt data = %02x\n",data);
}

DigitalOut led2(LED2);

int main() 
{
  PCF8574 pcf(p9,p10,PCF8574_ADDR,true);  // Declare PCF8574A i2c with sda = p9 and scl = p10
  uint8_t data;
  
  led2 = 0;
  
  // Set all IO port bits to 1 to enable inputs and test error
  data = 0xFF;
  pcf = data;
  if(pcf.getError() != 0)
    myerror(pcf.getErrorMessage());
  
  // Assign interrupt function to pin 17  
  pcf.interrupt(p17,&pcf8574_it);
  
  if(pcf.getError() != 0)
    myerror(pcf.getErrorMessage());
  
  // Get IO port (switch is used to flip bit 1)
  while(1) {
             wait(2.0);
             data = pcf;
             if(pcf.getError() != 0)
               myerror(pcf.getErrorMessage());
             led2 = !led2;
  }

  return(0);
}
*/

// Defines
#define PCF8574_NoError     0x00
#define PCF8574_BadAddress  0x01
#define PCF8574_I2cError    0x02
#define PCF8574_IntrError   0x03

#define PCF8574_MaxError       4

#ifndef BIT_SET
#define BIT_SET(x,n) (x=x | (0x01<<n))
#endif
#ifndef BIT_TEST
#define BIT_TEST(x,n) (x & (0x01<<n))
#endif
#ifndef BIT_CLEAR
#define BIT_CLEAR(x,n) (x=x & ~(0x01<<n))
#endif
#ifndef BIT_GET
#define BIT_GET(x,n) ((x >>n) & 0x1)
#endif

static std::string _ErrorMessagePCF8574[PCF8574_MaxError] = {
                                                              "",
                                                              "Bad chip address",
                                                              "I2C error (nack)",
                                                              "Unable to create InterruptIn instance"
                                                            };

// Class
class PCF8574 {
public:               
    /*
     * Constructor, initialize the PCF8574 on i2c interface.
     * @param sda : sda i2c pin (PinName)
     * @param scl : scl i2c pin (PinName)
     * @param address : PCF8574 address between 0 and 7 (uint8_t) 
     * @param typeA : PCF8574A if true, default false (bool)
     * @return none
    */
    PCF8574(PinName sda, PinName scl, uint8_t address, bool typeA = false);

    /*
     * Read the IO pins level
     * @param : none
     * @return port value (uint8_t)
    */
    uint8_t read(void);
    
    /*
     * Write to the IO pins
     * @param data : The 8 bits to write to the IO port (uint8_t)
     * @return none
    */
    void write(uint8_t data);
    
    /*
     * Write to individual IO pin
     * @param pin : The pin number between 0 and 7 (uint8_t)
     * @param value : The pin value (uint8_t)
     * @return none
    */
    void write(uint8_t pin, uint8_t value);
    
    /*
     * Write to IO pins with a bitset and a bitset mask (only bits set in mask are written)
     * @param data : The biset to write to the IO port (bitset<8>)
     * @param mask : The biset to write mask (bitset<8>)
     * @return none
    */
    void write(bitset<8> data, bitset<8> mask);
    
    /*
     * Interrupt IO callback called when PCF8574 IO pins changed (falling edge)
     * @param intr : pin name connected to PCF8574 int pin (PinName)
     * @param ftpr : user callback function (void ftpr(unit8_t data, PCF8574 *o)).
     *                 data : PCF8574 IO port value
     *                 o    : object instance address
     * @return none
    */
    void interrupt(PinName intr,void (*ftpr)(uint8_t data, PCF8574 *o));
    
    /*
     * Get current error message
     * @param  : none
     * @return current error message(std::string)
    */
    std::string getErrorMessage(void)
    { 
      if(_errnum < PCF8574_MaxError)
        return(_ErrorMessagePCF8574[_errnum]);
      else
        return("errnum out of range");
    }
    
    /*
     * Get the current error number (PCF8574_NoError if no error)
     * @param  : none
     * @return current error number (uint8_t)
    */
    uint8_t getError(void);
     
    /*
     * Memorize data IO port value and bits that have changed when interrupt
     * @param data : data IO port value when interrupt (bitset<8>)
     * @param changed : bits that have changed (bitset<8>) 
     * @return none
    */
     void setIntrData(bitset<8> data, bitset<8> changed);
     
    /*
     * Get data IO port value when and bits that have changed when interrupt 
     * @param data : data IO port value when interrupt (bitset<8>&) 
     * @param changed : bits that have changed (bitset<8>&)
     * @return none
    */
     void getIntrData(bitset<8>& data, bitset<8>& changed);
    
    /*
     *Operator () (read)
    */
    operator uint8_t();
    
    /* 
     *Operator = (write)
    */
    PCF8574 &operator=(uint8_t data);
    
private:
    I2C _i2c; // Local i2c communication interface instance
    int _address; // Local pcf8574 i2c address
    int _PCF8574_address; // PCF8574 address
    uint8_t _errnum; // Error number
    bitset<8> _intr_data; // IO port value when interrupt
    bitset<8> _intr_bits_changed; // Bits that have changed on interrupt 
    InterruptIn *_intr; // Internal InterruptIn object address
    uint8_t _intr_flag; // interrupt function has been called
};

#endif