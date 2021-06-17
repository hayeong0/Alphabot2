#include "mbed.h"

#include "RemoteIR.h"
#include "ReceiverIR.h"
#include "TRSensors.h"
#include "TB6612FNG.h"
#include "hcsr04.h"
#include "WS2812.h"
#include "PixelArray.h"
#include "Adafruit_SSD1306.h"
#include <string>
#include "PCF8574.h"

#define BUF 32
#define SENSOR 5
#define WS2812_BUF 100
#define NUM_COLORS 6
#define NUM_LEDS_PER_COLOR 4

#define maximum 550
#define PWMA 0.42
#define PWMB 0.4

Timer t;                
TRSensors tr;           // TR sensor 5개
I2C i2c(D14, D15);      // i2c 통신  SCL, SDA, P5
ReceiverIR IR(D4);      // user interface (IR receiver)
HCSR04 ultra(D3, D2);   // 초음파 센서
TB6612FNG motorDriver(D6, A1, A0, D5, A2, A3);  // motor driver
Adafruit_SSD1306_I2c gOLED(i2c, D9, 0x78, 64, 128); // oled 센서

UnbufferedSerial pc(USBTX, USBRX, 115200);  // 디버깅용 serial 통신 

PixelArray px(WS2812_BUF);                  // RGB LED   
WS2812 ws(D7, WS2812_BUF, 6, 17, 11, 14);


char buffer[80];  
unsigned int sensor_values[SENSOR]; 
int colorbuf[NUM_COLORS] = {0x2f0000, 0x2f2f00, 0x002f00, 0x002f2f, 0x00002f, 0x2f002f};
           
int button = 0;
int pos = 0;

int last_proportional = 0;
int integral = 0;
float sum = 0;
float kp = 3.0, ki = 0.03, kd = 2.1;
int flag = 0;

void RGB(int check) {
    int num = 3;
    if (check == 1) {
        for (int i = 0; i < WS2812_BUF; i++) {
            px.Set(i, colorbuf[(i / NUM_LEDS_PER_COLOR) % NUM_COLORS]);
        }
        for (int j = 0; j < WS2812_BUF; j++) {  // intensity
            px.SetI(j%WS2812_BUF, 0x4B);
        }
        while (num--) {
             // RGB setting
            for (int i = WS2812_BUF; i >= 0; i--) {
                ws.write_offsets(px.getBuf(), i, i, i);
                ThisThread::sleep_for(5);
            }
        }
        flag = 0;  
    } 
    //ws.useII(WS2812::PER_PIXEL); 
    px.SetAll(0); 
    ws.write_offsets(px.getBuf(), 0, 0, 0);
}

void display_init() {
    gOLED.clearDisplay();
    gOLED.setTextCursor(4, 2);
    gOLED.printf("\r== Alphabot ==\r\n\n");
    gOLED.printf("\n[*] Ready \r\n", sum);      
    gOLED.display();
}

void display_calibration() {
    gOLED.clearDisplay();
    gOLED.setTextCursor(0, 0);
    gOLED.printf("\n\n[*] Calibration Done! \r\n");       
    gOLED.display();
}

void display_obstacle() {
    gOLED.clearDisplay();
    gOLED.setTextCursor(0, 0);
    gOLED.printf("\n\n[*] STOP! \r\n");       
    gOLED.display();
}

void display_time() {
    gOLED.clearDisplay();
    gOLED.setTextCursor(5, 5);
    gOLED.printf("\r== Alphabot ==\r\n\n");
    gOLED.printf("\nTime: %f (sec) \r\n", sum);      
    gOLED.display();
}

int main() { 
    if (flag == 0) {
        px.SetAll(0); 
        ws.write_offsets(px.getBuf(), 0, 0, 0);
    }
    //px.SetAll(0); 
    uint8_t buf[BUF];
    RemoteIR::Format format;
  
    sprintf(buffer, "== Alphabot start! ==\r\n");
    pc.write(buffer, strlen(buffer));
  
    // OLED
    i2c.frequency(100000);      
    display_init();

    // Buzzer
    //int status = i2c.write((PCF8574_ADDR << 1), data_write, D10, 0);

    // IR remote
    while(1) { 
        if (IR.getState() == ReceiverIR::Received) {
            int bitcount = IR.getData(&format, buf, sizeof(buf)*8);
            sprintf(buffer, "button value: %d\r\n", buf[2]);
            pc.write(buffer, strlen(buffer));
            button = buf[2];
        }
        // TODO: IR 값 정밀 체크해주는 부분 추가하기
        switch(button) {
            // Button >|| (Calibration): 0x00FF43BC
            case 0x43: {
              //motorDriver.setspeed(0.6, 0.6);
              int i = 25;
              sprintf(buffer, "[*] calibration start!\r\n");
              pc.write(buffer, strlen(buffer));

              while (i--) {
                motorDriver.forward(0.3, 0.3);
                tr.calibrate(); 
                ThisThread::sleep_for(100);
                motorDriver.backward(0.3, 0.3);
                tr.calibrate();
                ThisThread::sleep_for(100);
                motorDriver.forward(0.3, 0.3);
                tr.calibrate();
                ThisThread::sleep_for(100);
                motorDriver.backward(0.3, 0.3);
                tr.calibrate();
                ThisThread::sleep_for(100);
              }
              sprintf(buffer, "[-] calibration done!\r\n");
              pc.write(buffer, strlen(buffer));
              display_calibration();

              ThisThread::sleep_for(200);
              button = 0x09;       
              break;
            }
                
            // Button 2 (Forward): 0x00FF18E7   
            case 0x18:
                motorDriver.forward(PWMA, PWMB);
                tr.calibrate(); 
                ThisThread::sleep_for(100);
                button = 0x09;       
                break;
            
            // Button 8 (Backward) : 0x00FF52AD
            case 0x52:
                motorDriver.backward(PWMA,PWMB);
                tr.calibrate();
                ThisThread::sleep_for(100);
                button = 0x09;       
                break;
            
            // Button 4 (Turn left) : 0x00FF08F7
            case 0x08:
                motorDriver.turn_left(0.1, 0.3);
                tr.calibrate();
                ThisThread::sleep_for(100);
                button = 0x09;            
                break;

            // Button 6 (Turn right) : 0x00FF5AA5
            case 0x5A:
                motorDriver.turn_right(0.3, 0.1);
                tr.calibrate();
                ThisThread::sleep_for(100);
                button = 0x09;        
                break;
            
            // Button EQ (Stop) : 0x00FF09F6
            case 0x09:
                flag = 0;
                motorDriver.stop();
                break;
            
            // Button 5 (Auto Drive) : 0x00FF1CE3
            // 라인 위치 파악 + 모터 제어
            case 0x1C: {
                
                int start = 0, end = 0, dist = 0;
                t.reset();
                t.start();
                start = t.elapsed_time().count();

                while(1) {  
                    flag = 0;
                    int position = tr.readLine(sensor_values, 0);
                    ultra.start();
                    dist = ultra.get_dist_cm();

                    if (dist <= 30) {
                        t.stop();
                        end = t.elapsed_time().count();
                        motorDriver.stop();
                        flag = 1;
                        
                        sum = end-start;
                        sum /= 1e+6;
                        if (flag == 1) {
                            RGB(flag);
                            display_time();
                        }          
                        button = 0x09;   
                        break;
                    }

                    int proportional = position - 2000;                 // 오차
                    int derivative = proportional-last_proportional;    // 현재 오차-이전 오차
                    integral += proportional;                           // 오차 누적
                    last_proportional = proportional;

                    int power_diff = proportional/kp + integral*ki + derivative*kd;  //pid 적용 후 제어 값

                    // 중앙보다 오른쪽에 위치 --> 오른쪽 바퀴 가속
                    if (power_diff < 0) { 
                        if (PWMA+(float)power_diff/maximum*PWMA < 0.15){
                            motorDriver.forward(0.16, PWMB);
                        }
                        else {
                            motorDriver.forward(PWMA+(float)power_diff/maximum*PWMA, PWMB);  
                        }             
                    }
                    // 중앙보다 왼쪽에 위치 --> 왼쪽 바퀴 가속
                    else if (power_diff > 0) { 
                        if (PWMB-(float)power_diff/maximum*PWMB < 0.15) { 
                            motorDriver.forward(PWMA, 0.16);
                        }
                        else {
                            motorDriver.forward(PWMA, PWMB-(float)power_diff/maximum*PWMB);
                        }
                    } 
                    else {  // 중앙
                        motorDriver.forward(PWMA, PWMB);
                    }
                    // debug
                    sprintf(buffer, "[+] Position: %d\r\n", position);
                    pc.write(buffer, strlen(buffer));
                    ThisThread::sleep_for(100);
                }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
                break;  
            }
                
            // Button 100+ (Sensor value) : 0x00FF19E6
            case 0x19: {
                tr.AnalogRead(sensor_values);
                for (int i = 0; i < 5; i++) {
                    gOLED.clearDisplay();
                    gOLED.setTextCursor(0, 0);
                    gOLED.printf("IR[1]: %d\r\n", sensor_values[0]);  
                    gOLED.printf("IR[2]: %d\r\n", sensor_values[1]);  
                    gOLED.printf("IR[3]: %d\r\n", sensor_values[2]);  
                    gOLED.printf("IR[4]: %d\r\n", sensor_values[3]);  
                    gOLED.printf("IR[5]: %d\r\n", sensor_values[4]);        
                    gOLED.display();

                    sprintf(buffer, "IR[%d] %d\r\n", i+1, sensor_values[i]);
                    pc.write(buffer, strlen(buffer));
                }
              
                sprintf(buffer, "[*] Done!\r\n");
                pc.write(buffer, strlen(buffer));
                button = 0x09;       
                break; 
            }
                   

            // Button 200+ (Position value) : 0x00FF0DF2
            case 0x0D: {
                tr.readCalibrated(sensor_values);
                // for (int i = 0; i < 5; i++) {
                //     gOLED.clearDisplay();
                //     gOLED.setTextCursor(0, 0);
                //     gOLED.printf("IR[1]: %d\r\n", sensor_values[0]);  
                //     gOLED.printf("IR[2]: %d\r\n", sensor_values[1]);  
                //     gOLED.printf("IR[3]: %d\r\n", sensor_values[2]);  
                //     gOLED.printf("IR[4]: %d\r\n", sensor_values[3]);  
                //     gOLED.printf("IR[5]: %d\r\n", sensor_values[4]);  
                //     gOLED.display();

                    // sprintf(buffer, "Calib[%d] %d\r\n", i+1, sensor_values[i]);
                    // pc.write(buffer, strlen(buffer));
                // }
                
                int j = 50;
                while (j--) {
                    pos = tr.readLine(sensor_values, 0);
                    gOLED.clearDisplay();
                    gOLED.setTextCursor(0, 0);
                    gOLED.printf("Position: %d\r\n", pos);  
                    gOLED.display();

                    // sprintf(buffer, "val: %d\r\n", pos);
                    // pc.write(buffer, strlen(buffer));
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


