#include "mbed.h"

#include "RemoteIR.h"
#include "ReceiverIR.h"
#include "TRSensors.h"
#include "TB6612FNG.h"
#include "hcsr04.h"
#include "WS2812.h"
#include "PixelArray.h"
#include "Adafruit_SSD1306.h"

#define BUF 32
#define SENSOR 5
#define WS2812_BUF 100
#define NUM_COLORS 6
#define NUM_LEDS_PER_COLOR 4

//125, 255, 300, 350
#define maximum 255
#define PWMA 0.3
#define PWMB 0.3

Timer t;                
TRSensors tr;          // TR sensor 5개
I2C i2c(D14, D15);      // i2c 통신  D14, D15
ReceiverIR IR(D4);      // user interface (IR receiver)
HCSR04 ultra(D3, D2);   // 초음파 센서
TB6612FNG motorDriver(D6, A1, A0, D5, A2, A3);  // motor driver
Adafruit_SSD1306_I2c gOLED(i2c, D9, 0x78, 64, 128); // oled 센서

UnbufferedSerial pc(USBTX, USBRX, 115200);  // 디버깅용 serial 통신 

PixelArray px(WS2812_BUF);  // RGB LED   
WS2812 ws(D7, WS2812_BUF, 6, 17, 11, 14);

char buffer[80];  
unsigned int sensor_values[SENSOR]; 
int colorbuf[NUM_COLORS] = {0x2f0000, 0x2f2f00, 0x002f00, 0x002f2f, 0x00002f, 0x2f002f};

int button = 0;
int pos = 0;

int position = 0;
//double power_diff = 0.0;
int prev_proportional = 0;
int integral = 0;
int dist = 0;
float sum = 0;
float kp = 3.0, ki = 0.0, kd = 0.0;
//float ptmp = 3.0, itmp = 0.0, dtmp = 0.0;

void RGB() {
  // RGB setting
    for (int i = 0; i < WS2812_BUF; i++) {
        px.Set(i, colorbuf[(i / NUM_LEDS_PER_COLOR) % NUM_COLORS]);
    }
    for (int j = 0; j < WS2812_BUF; j++) {
        px.SetI(j % WS2812_BUF, 0x4B);
        ThisThread::sleep_for(1);
    }
    for (int i = WS2812_BUF; i >= 0; i--) {
      ws.write_offsets(px.getBuf(), i, i, i);
    }
}

void display_time() {
  gOLED.clearDisplay();
  gOLED.setTextCursor(0, 0);
  gOLED.printf("** Alphabot **\r\n");
  gOLED.printf("Time: %f (sec) **\r\n", sum);      
  gOLED.display();
}

// void display_diff() {
//  gOLED.clearDisplay();
//  gOLED.setTextCursor(0, 0);
//  gOLED.printf("diff:  %ld \r\n", power_diff);      
//  gOLED.display();
// }

void display_val() {
  gOLED.clearDisplay();
  gOLED.setTextCursor(0, 0);
  gOLED.printf("[ %d ]\r\n", pos);      
  gOLED.display();
}

void display_position() {
  gOLED.clearDisplay();
  gOLED.setTextCursor(0, 0);
  gOLED.printf("[ %d ]\r\n", position);      
  gOLED.display();
}
void display_info() {
  gOLED.clearDisplay();
  gOLED.setTextCursor(2, 2);
  gOLED.printf("P : %f\r\n", ptmp);
  gOLED.printf("I : %f\r\n", itmp);  
  gOLED.printf("D : %f\r\n", dtmp);       
  gOLED.display();
}
    
int main() { 
    uint8_t buf[BUF];
    RemoteIR::Format format;
  
    sprintf(buffer, "*=== Alphabot start! ===*\r\n");
    pc.write(buffer, strlen(buffer));
  
    // OLEd
    i2c.frequency(100000);      
    display_time();

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
            // Button + (Increase P val)
            case 0x15: 
              ptmp += 0.02;
              display_info();
              button = 0x09;
              break;
            
            // Button - (Increase D val)
            case 0x07: 
              dtmp += 0.02;
              display_info();
              button = 0x09;
              break;
            
            // Button >|| (Calibration): 0x00FF43BC
            case 0x43: {
              //motorDriver.setspeed(0.6, 0.6);
              int i = 50;
              sprintf(buffer, "[*] calibration start!\r\n");
              pc.write(buffer, strlen(buffer));
          
              while (i--) {
                motorDriver.forward(0.5, 0.5);
                tr.calibrate(); 
                ThisThread::sleep_for(100);
                motorDriver.backward(0.5, 0.5);
                tr.calibrate();
                ThisThread::sleep_for(100);      
                // ThisThread::sleep_for(100);
                // motorDriver.turn_left(0.4, 0.5);
                // tr.calibrate();
                // ThisThread::sleep_for(100);
                // motorDriver.turn_right(0.5, 0.4);
                // tr.calibrate();
              }
              motorDriver.stop();
              sprintf(buffer, "[-] calibration done!\r\n");
              pc.write(buffer, strlen(buffer));

              gOLED.clearDisplay();
              gOLED.setTextCursor(0, 0);
              gOLED.printf("[*] Calibration Done! \r\n");       
              gOLED.display();

              ThisThread::sleep_for(200);
              button = 0x09;       
              break;
            }
                
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
                //motorDriver.turn_right(0.3, 0.1);
                tr.calibrate();
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
                int start = 0, end = 0;
                //ultra.init();        
                t.reset();
                t.start();
                start = t.elapsed_time().count();

                while(1) {       
                    position = tr.readLine(sensor_values, 0);
                    ultra.start();
                    dist = ultra.get_dist_cm();
                  
                    if (dist <= 20) {
                      sprintf(buffer, "*=== bbanzzak! bbanzzak! ===*\r\n");
                      pc.write(buffer, strlen(buffer));
                      // OLED
                      gOLED.clearDisplay();
                      gOLED.setTextCursor(0, 0);
                      gOLED.printf("[*] Done! \r\n");       
                      gOLED.display();
                      // RGB
                      //RGB();
                      break;
                    }

                    // 선이 IR1 왼쪽에 있는 상태
                    // if (position == 0) {
                    //   motorDriver.turn_left(0.1, 0.2);
                    // }
                    // else if (position > 0 && position < 650) {
                    //   motorDriver.turn_left(0.13, 0.2);
                    // }
                    // else if (position >= 650 && position < 1500) {
                    //   motorDriver.turn_left(0.17, 0.2);
                    // }
                    // else if (position >= 1500 && position < 2700) {
                    //   motorDriver.forward(0.2, 0.2);
                    // }
                    // else if(position >= 2700  && position < 3500) {
                    //   motorDriver.turn_right(0.2, 0.15);
                    // }
                    // else if (position >= 3500 && position < 4000) {
                    //   motorDriver.turn_right(0.25, 0.15);
                    // }
                    // else if (position > 4000) {
                    //   motorDriver.turn_right(0.3, 0.15);
                    // }

                    int proportional = position - 2000; // P: 오차
                    int derivative = proportional - prev_proportional;  // D: 현재 오차-이전 오차
                    integral += proportional; // I: 오차 누적
                    prev_proportional = proportional; 

                    int power_diff = proportional/kp + integral*ki + derivative*kd; //pid 적용 후 제어 값
                    
                    sprintf(buffer, "P : %f\r\n", kp);
                    pc.write(buffer, strlen(buffer));
                    sprintf(buffer, "I : %f\r\n", ki);
                    pc.write(buffer, strlen(buffer));
                    sprintf(buffer, "D : %f\r\n", kd);
                    pc.write(buffer, strlen(buffer));
                    sprintf(buffer, "[+] Power_diff: %d\r\n", power_diff);
                    pc.write(buffer, strlen(buffer));
                    
                    gOLED.clearDisplay();
                    gOLED.setTextCursor(2, 2);
                    gOLED.printf("P : %f\r\n", ptmp);
                    gOLED.printf("I : %f\r\n", itmp);  
                    gOLED.printf("D : %f\r\n", dtmp);     
                    gOLED.printf("P.D. : %d\r\n", power_diff);     
                    gOLED.display();


                    if (power_diff > maximum) power_diff = maximum;
                    if (power_diff < -maximum) power_diff = -maximum;

                    // // saturate
                    if (power_diff < 0) { // 오른쪽으로 치우침 --> 오른쪽 바퀴 가속
                      motorDriver.forward(PWMA+(float)power_diff/maximum*PWMA, PWMB); 
                      // if (power_diff == -maximum) motorDriver.forward(0.2, PWMB);      
                      // else motorDriver.forward(PWMA+(float)power_diff/maximum*PWMA, PWMB);           
                    }
                    else if (power_diff > 0) {  // 중앙보다 왼쪽에 위치 --> 왼쪽 바퀴 가속
                      motorDriver.forward(PWMA, PWMB-(float)power_diff/maximum*PWMB);
                      // if (power_diff == maximum) motorDriver.forward(PWMA, 0.2);
                      // else motorDriver.forward(PWMA, PWMB-(float)power_diff/maximum*PWMB);  
                    }
                    else {
                      motorDriver.forward(PWMA, PWMB);
                    }
                    
                    // debug
                    sprintf(buffer, "[+] Position: %d\r\n", position);
                    pc.write(buffer, strlen(buffer));
                    //display_position();
                    ThisThread::sleep_for(100);

                    // position이 왼쪽이나 오른쪽으로 치우쳐 있을 때
                    // if (sensor_values[0] > 800 && sensor_values[1] > 800 && sensor_values[2] > 800) {
                    //   motorDriver.forward(0, 0);
                    //   motorDriver.stop();
                     
                    //   end = t.elapsed_time().count();
                    //   sum = end-start;
                    //   display();
                    //   RGB();
                    //   ThisThread::sleep_for(1500);
                    //   break;
                    // }
                }
                end = t.elapsed_time().count();
                sum = end-start;
                sum /= 1e+6;

                display_time();
                button = 0x09;   
                break;  
            }
                
            // Button 100+ (Sensor value) : 0x00FF19E6
            case 0x19: {
              tr.AnalogRead(sensor_values);
              for (int i = 0; i < 5; i++) {
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
              for (int i = 0; i < 5; i++) {
                    sprintf(buffer, "Calib[%d] %d\r\n", i+1, sensor_values[i]);
                    pc.write(buffer, strlen(buffer));



                }

              int j = 50;
              while (j--) {
                  pos = tr.readLine(sensor_values, 0);
                  sprintf(buffer, "val: %d\r\n", pos);
                  pc.write(buffer, strlen(buffer));
              } 
              pos = tr.readLine(sensor_values, 0);
              display_val();
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
 
