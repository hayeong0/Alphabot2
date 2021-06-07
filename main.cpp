#include "mbed.h"
#include "HCSR04.h"
#include "RemoteIR.h"
#include "ReceiverIR.h"
#include "TB6612FNG.h"
#include "TRSensors.h"
#include "Adafruit_SSD1306.h"

#include "WS2812.h"
#include "PixelArray.h"

#define BUF 32
#define SENSOR 5
#define WS2812_BUF 100
#define WS2812_BUF2 4
#define NUM_COLORS 3
#define NUM_LEDS_PER_COLOR 4

Timer t;
TRSensors tr;           // TR sensotr 5개
I2C i2c(PB_9, PB_8);    // D14, 15
ReceiverIR IR(D4);      // user interface (IR receiver)
HCSR04 ultra(D3, D2);
TB6612FNG motorDriver(D6, A1, A0, D5, A2, A3);  // motor 구동
UnbufferedSerial pc(USBTX, USBRX, 115200);  // serial 통신 for 디버깅
Adafruit_SSD1306_I2c gOLED(i2c, D9, 0x7A, 32, 128); //32, 64

DigitalOut dc(D8, 1);    // oled 모듈 mode
DigitalOut rst(D9, 1);   // olde 모듈 reset
PixelArray px(WS2812_BUF);
WS2812 ws(D7, WS2812_BUF, 7, 15, 10, 15);

char buffer[80];
unsigned int sensor_values[SENSOR]; 
int color[NUM_COLORS] = {0xff0000, 0x00ff00, 0x0000ff};

int button = 0;
int pos = 0;
int sum = 0;

void display() {
    gOLED.clearDisplay();
    gOLED.setTextCursor(0, 0);
    gOLED.printf("Alphabot start\r\n");
    gOLED.printf("Timer: %.2f\r\n", sum);
    gOLED.display();
}
    
int main() { 
    int bitcount;
    uint8_t buf[BUF];
    RemoteIR::Format format;

    // oled     
    for (int i = 0; i < WS2812_BUF; i++) {
        px.Set(i, color[(i / NUM_LEDS_PER_COLOR) % NUM_COLORS]);
    }
    for (int j=0; j < WS2812_BUF; j++) {
        px.SetI(j%WS2812_BUF, 0xf+(0xf*4));
    }
    
    display();
    
    while(1) { 
        if (IR.getState() == ReceiverIR::Received) {
            bitcount = IR.getData(&format, buf, sizeof(buf)*8);
            sprintf(buffer, "button value: %d\r\n", buf[2]);
            pc.write(buffer, strlen(buffer));
            button = buf[2];
        }
        
        switch(button) {
            // Button >|| (Calibration): 0x00FF43BC
            case 0x43:
                sprintf(buffer, "[*] calibration start!\r\n");
                pc.write(buffer, strlen(buffer));
                tr.calibrate();
                sprintf(buffer, "[-] calibration done!\r\n");
                pc.write(buffer, strlen(buffer));
                ThisThread::sleep_for(200);
                button = 0x09;       
                break;

            // Button 2 (Forward): 0x00FF18E7   
            case 0x18:
                motorDriver.forward(0.4, 0.4);
                tr.calibrate(); 
                ThisThread::sleep_for(100);
                button = 0x09;       
                break;
            
            // Button 8 (Backward) : 0x00FF52AD
            case 0x52:
                motorDriver.backward(0.4, 0.4);
                tr.calibrate();
                ThisThread::sleep_for(100);
                button = 0x09;       
                break;
            
            // Button 4 (Turn left) : 0x00FF08F7
            case 0x08:
                motorDriver.turn_left(0.2, 0.4);
                ThisThread::sleep_for(100);
                button = 0x09;            
                break;

            // Button 6 (Turn right) : 0x00FF5AA5
            case 0x5A:
                motorDriver.turn_right(0.4, 0.2);
                ThisThread::sleep_for(100);
                button = 0x09;        
                break;
            
            // Button EQ (Stop) : 0x00FF09F6
            case 0x09:
                motorDriver.stop();
                break;
            
            // Button 5 (Auto Drive) : 0x00FF1CE3
            case 0x1C:
            // 곧 할 것임...

            
            // Button 100+ (Sensor value) : 0x00FF19E6
            case 0x19:
                pos = tr.readLine(sensor_values, 0);
                for (int i = 0; i < 5; i++) {
                  sprintf(buffer, "%d\r\n",sensor_values[i]);
                  pc.write(buffer, strlen(buffer));
                }
                sprintf(buffer, "[DONE]\r\n");
                pc.write(buffer, strlen(buffer));
                button = 0x09;       
                break;    

            // Button 200+ (Position value) : 0x00FF0DF2
            case 0x0D:
                int j = 100;
                while (j--) {
                    pos = tr.readLine(sensor_values, 0);
                    sprintf(buffer, "%d\r\n",pos);
                    pc.write(buffer, strlen(buffer));
                } 
                button = 0x09;       
                break;   
        }
    }
}
