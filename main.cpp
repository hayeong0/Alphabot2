#include "mbed.h"
#include "RemoteIR.h"
#include "ReceiverIR.h"
#include "TB6612FNG.h"
#include "TRSensors.h"
#include "Adafruit_SSD1306.h"
#include "hcsr04.h"

#include "WS2812.h"
#include "PixelArray.h"

#define BUF 32
#define SENSOR 5
#define WS2812_BUF 100
#define NUM_COLORS 6
#define NUM_LEDS_PER_COLOR 4

#define maximum 255
#define PWMA 0.3
#define PWMB 0.3

Timer t;
TRSensors tr;           // TR sensotr 5개
I2C i2c(D14, D15);    // D14, 15
ReceiverIR IR(D4);      // user interface (IR receiver)
HCSR04 ultra(D3, D2);
TB6612FNG motorDriver(D6, A1, A0, D5, A2, A3);  // motor 구동
UnbufferedSerial pc(USBTX, USBRX, 115200);  // serial 통신 for 디버깅
Adafruit_SSD1306_I2c gOLED(i2c, D9, 0x78, 64, 128); //32, 64

DigitalOut dc(D8, 1);    // oled 모듈 mode
DigitalOut rst(D9, 1);   // olde 모듈 reset
PixelArray px(WS2812_BUF);
WS2812 ws(D7, WS2812_BUF, 6, 17, 11, 14);

char buffer[80];
unsigned int sensor_values[SENSOR]; 
int colorbuf[NUM_COLORS] = {0x2f0000, 0x2f2f00, 0x002f00, 0x002f2f, 0x00002f, 0x2f002f};

int button = 0;
int pos = 0;
int sum = 0;
int last_proportional = 0;
int integral = 0;
int dist = 0;

void display() {
    gOLED.clearDisplay();
    gOLED.setTextCursor(0, 0);
    gOLED.printf("Alphabot\r\n");
    //gOLED.printf("Timer: %.2f\r\n", sum);
    gOLED.display();
}
    
int main() { 
    int bitcount;
    uint8_t buf[BUF];
    RemoteIR::Format format;

    // oled     
    for (int i = 0; i < WS2812_BUF; i++) {
        px.Set(i, colorbuf[(i / NUM_LEDS_PER_COLOR) % NUM_COLORS]);
    }
    for (int j = 0; j < WS2812_BUF; j++) {
        px.SetI(j%WS2812_BUF, 0x4B);
        ThisThread::sleep_for(1);
    }
    
   
    display();
    
    sprintf(buffer, "*=== Alphabot start! ===*\r\n");
    pc.write(buffer, strlen(buffer));

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
                //motorDriver.setspeed(0.6, 0.6);
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
                motorDriver.forward(0.3, 0.3);
                tr.calibrate(); 
                ThisThread::sleep_for(100);
                button = 0x09;       
                break;
            
            // Button 8 (Backward) : 0x00FF52AD
            case 0x52:
                motorDriver.backward(0.3, 0.3);
                tr.calibrate();
                ThisThread::sleep_for(100);
                button = 0x09;       
                break;
            
            // Button 4 (Turn left) : 0x00FF08F7
            case 0x08:
                motorDriver.turn_left(0.1, 0.3);
                ThisThread::sleep_for(100);
                button = 0x09;            
                break;

            // Button 6 (Turn right) : 0x00FF5AA5
            case 0x5A:
                motorDriver.turn_right(0.3, 0.1);
                ThisThread::sleep_for(100);
                button = 0x09;        
                break;
            
            // Button EQ (Stop) : 0x00FF09F6
            case 0x09:
                motorDriver.stop();
                break;
            
            // Button 5 (Auto Drive) : 0x00FF1CE3
            // 라인 위치 파악 + 모터 제어
            case 0x1C: {
              float kp = 0.0, ki = 0.0, kd = 0.0;
                float sum = 0;
                int start = 0, end = 0;
                // t.reset();
                // t.start();
                //ultra.init();        

                while(1) {  
                    start = t.elapsed_time().count();
                    unsigned int position = tr.readLine(sensor_values, 0);
                    ultra.start();
                    dist = ultra.get_dist_cm();
                  
                    if (dist <= 10) {
                      sprintf(buffer, "*=== bban zzak bban zzak! ===*\r\n");
                      pc.write(buffer, strlen(buffer));

                      // fade effect, intensitty scaling
                      for (int i = WS2812_BUF; i >= 0; i--) {
                        ws.write_offsets(px.getBuf(), i, i, i);
                      }
    
                      break;
                    }

                    if (position >= 1750 && position < 2500) {
                      motorDriver.forward(0.3, 0.3);
                    }
                    else if(position >= 2500 && position < 3000) {
                      motorDriver.turn_right(0.3, 0.3);
                    }
                    else if (position <= 3500 && position >= 3000) {
                      motorDriver.turn_right(0.3, 0.15);
                    }
                    else if (position <= 4000 && position > 3500) {
                      motorDriver.turn_right(0.3, 0.1);
                    }
                    else if (position < 1750 && position > 1000) {
                      motorDriver.turn_left(0.3, 0.3);
                    }
                    else if (position < 1000 && position > 0) {
                      motorDriver.turn_left(0.25, 0.3);
                    }
                    else if (position == 0){
                      motorDriver.turn_left(0.1, 0.3);
                    }
                    button = 0x09;   


                    int proportional = (int)position - 2000;
                    int derivative = proportional-last_proportional;
                    integral += proportional;
                    last_proportional = proportional;

                    int power_diff = proportional/kp + integral*ki + derivative*kd;

                    // if (power_diff > maximum) power_diff = maximum;
                    // if (power_diff < -maximum) power_diff = -maximum;

                    // const float std = 970.0
                    // // saturate
                    // if (power_diff < 0) { // 왼쪽으로 치우침 --> 왼쪽 바퀴 가속
                    //   motorDriver.forward(PWMA, maximum+power_diff/maximum);               
                    // }
                    // else if (power_diff == 0) {
                    //   motorDriver.forward(0.3, 0.3);
                    // }
                    // else { // 오른쪽으로 치우침 --> 오른쪽 바퀴 가속
                                   
                    // }
                }
                break;  
            }
                

            
            // Button 100+ (Sensor value) : 0x00FF19E6
            case 0x19: {
              tr.AnalogRead(sensor_values);
                for (int i = 0; i < 5; i++) {
                    sprintf(buffer, "IR[%d] %d\r\n", i+1, sensor_values[i]);
                    pc.write(buffer, strlen(buffer));
                }
              
                sprintf(buffer, "[DONE]\r\n");
                pc.write(buffer, strlen(buffer));
                button = 0x09;       
                break; 
            }
                   

            // Button 200+ (Position value) : 0x00FF0DF2
            case 0x0D: {
              tr.readCalibrated(sensor_values);
              for (int i = 0; i < 5; i++) {
                    sprintf(buffer, "Calib[%d] %d\r\n", i+1, sensor_values[i]);
                    pc.write(buffer, strlen(buffer));
                }
                button = 0x09;
                break;

              int j = 10;
                while (j--) {
                    pos = tr.readLine(sensor_values, 0);
                    sprintf(buffer, "%d\r\n", pos);
                    pc.write(buffer, strlen(buffer));
                } 
                button = 0x09;       
                break;  
            }
            
            default: {
                button = 0x09;
                break;
            }
              
      } //end of switch
    } // end of while
}
