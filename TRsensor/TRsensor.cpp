/*
Calibration
- Calibration은 특정조건에서 측정에 의하여 결정된 값을 표준에 의해 결정된 값 사이의 관계로 치환하는 작업을 의미한다.
- Calibration은 측정과 관측 시 발생하는 오차를 감소시키기 위해 필요하다.
- IR sensor는 빛이 반사되는 정도에 따라 voltage 값을 출력한다.
- 이 때 측정 상황에 따라 white & black의 측정 범위가 달라지기 때문에 센서의 위치와 상황을 고려하여 calibration을  
 */

#include "mbed.h"
#include "TRSensors.h"

#define NUMSENSORS 5

SPI spi(ARDUINO_UNO_D11, ARDUINO_UNO_D12, ARDUINO_UNO_D13);    
DigitalOut cs(ARDUINO_UNO_D10, 1); 

// Base class data member initialization (called by derived class init())
TRSensors::TRSensors() {
    spi.format(16, 0);          // 16bit 사용
    spi.frequency(2000000);     //  2MHz (2hz)

    _numSensors = NUMSENSORS;
    
    // calibrate() 함수 호출을 통해 얻게 된 값들 저장
    calibratedMin = (unsigned int*)malloc(sizeof(unsigned int) * _numSensors);
    calibratedMax = (unsigned int*)malloc(sizeof(unsigned int) * _numSensors);
    
    for (int i = 0; i < _numSensors; i++) {
        calibratedMin[i] = 1023;
        calibratedMax[i] = 0;
    }
}


// Reads the sensor values using TLC1543 ADC chip into an array. 
// The values returned are a measure of the reflectance in abstract units,
// with higher values corresponding to lower reflectance (e.g. a black
// surface or a void).
// [AnalogRead() 함수] 아날로그에서 값 읽어오는 함수; Read IR
void TRSensors::AnalogRead(unsigned int *sensor_values) {
    unsigned int channel = 0;
    unsigned int values[] = {0,0,0,0,0,0,0,0};

    for (channel = 0; channel < 6; channel++) {
        cs = 0;
        wait_us(2);
        values[channel] = spi.write(channel << 12);
        cs = 1;
        wait_us(21);
    }

    for (int i = 0; i < _numSensors; i++) {
        sensor_values[i] = values[i+1] >> 6;
    }

}

/*
 [calibrate() 함수]
 Reads the sensors 10 times and uses the results for calibration.
 The sensor values are not returned;
 instead, the maximum and minimum values found
 over time are stored internally and used for the readCalibrated() method.
 
 -> sensor 값이 반환되지 않고, max min 값을 저장 (시간이 지남에 따라 내부적으로 저장되고, readCalibrated()에 사용됨
 */

void TRSensors::calibrate() {
    int i = 0;
    // IR 센서로부터 받은 값을 저장
    unsigned int sensor_values[_numSensors];
    unsigned int max_sensor_values[_numSensors];    // max value
    unsigned int min_sensor_values[_numSensors];    // min value
    
    // [*] setting max and min sensor value;
    for (int j = 0; j < 10; j++) {
        // 10회에 걸쳐 센서 값을 받는다.
        AnalogRead(sensor_values);
        for(i = 0; i < _numSensors; i++) {
            // max 값
            if(j == 0 || max_sensor_values[i] < sensor_values[i])
                max_sensor_values[i] = sensor_values[i];

            // min 값
            if(j == 0 || min_sensor_values[i] > sensor_values[i])
                min_sensor_values[i] = sensor_values[i];
        }
    }
  
    /*
     min_sensor_value[0]-[4]: IR1~IR5의 10회 측정 값 중 가장 작은 값 저장
     max_sensor_value[0]-[4]: IR1~IR5의 10회 측정 값 중 가장 큰 값 저장
     */
    
    /*
     [*] record the min and max calibration values
     1) min_sensor_value의 저장된 값이 calibratedMax보다 큰 경우, 해당 값을 calibratedMax값으로 치환.
     2) max_sensor_value의 저장된 값이 calibratedMin보다 작은 경우, 해당 값을 calibrateMin값으로 치환.
     1), 2)를 통해 calibratedMin에는 x회 동안 calibrate()함수 호출을 하여 얻은
     max(white)값들 중 가장 minimum한 값이, calibrateMax에는 x회 동안 calibrate()함수
     호출을 하여 얻은 min(black)값들 중 가장 maximum한 값이 저장된다.
     */
    for (i = 0; i < _numSensors; i++) {
        if (min_sensor_values[i] > calibratedMax[i])
        calibratedMax[i] = min_sensor_values[i];
        if (max_sensor_values[i] < calibratedMin[i])
        calibratedMin[i] = max_sensor_values[i];
    }
}


// Returns values calibrated to a value between 0 and 1000, where
// 0 corresponds to the minimum value read by calibrate() and 1000
// corresponds to the maximum value.  Calibration values are
// stored separately for each sensor, so that differences in the
// sensors are accounted for automatically.
/*
 -> [0, 1000] 보정 된 값을 반환.
    0은 calibrate()에서 읽은 min, 1000은 max.
    보정 값은 각 센서에 대해 별도로 저장되므로 센서의 차이가 자동으로 고려됨.
*/

/*
 calibrate()를 마치면, calibratedMax에는 black의 범위에 속하는 값 중 가장 큰 값
                     calibratedMin에는 white의 범위에 속하는 값 중 가장 작은 값
 */
void TRSensors::readCalibrated(unsigned int *sensor_values) {
    // read the needed values
    AnalogRead(sensor_values);

    for (int i = 0; i < _numSensors; i++) {
        unsigned int calmin, calmax;
        unsigned int denominator;

        denominator = calibratedMax[i] - calibratedMin[i];

        signed int x = 0;
        
        // scaling 거쳐서 mapping; 수식 x = (sensor - caliMin) * 1000/denominator
        if (denominator != 0)
            x = (((signed long)sensor_values[i]) - calibratedMin[i]) * 1000 / denominator;
        
        // normalization
        if (x < 0)
            x = 0;          // white
        else if(x > 1000)
            x = 1000;       // black
        sensor_values[i] = x;
    }

}

/*
 Operates the same as read calibrated, but also returns an estimated position of the robot with respect to a line. The estimate is made using a weighted average of the sensor indices multiplied by 1000, so that a return value of 0 indicates that the line is directly below sensor 0, a return value of 1000 indicates that the line is directly below sensor 1, 2000  indicates that it's below sensor 2000, etc.  Intermediate values indicate that the line is between two sensors.

-> readCalibrate()와 동일하게 작동 + 라인에 대한 로봇의 예상 위치도 리턴.
  estimate는 센서 인덱스의 weighted avg에 1000을 곱하여 사용함.
  return value 0: 선이 센서 0 바로 아래 있음을 나타냄.
  return value 1000: 선이 센서 1 바로 아래 있음을 나타냄
  return value 2000: 두 센서 사이에 선이 있음
 */

/*
    The formula is:
    
        0*value0 + 1000*value1 + 2000*value2 + ...
        --------------------------------------------
            value0  +  value1  +  value2 + ...
    By default, this function assumes a dark line (high values)
    surrounded by white (low values).  If your line is light on
    black, set the optional second argument white_line to true.  In
    this case, each sensor value will be replaced by (1000-value)
    before the averaging.
 */

/*
 [+] 로봇 위치 파악할 수 있는 함수
 */
int TRSensors::readLine(unsigned int *sensor_values, unsigned char white_line) {
    unsigned char i, on_line = 0;
    unsigned long avg;      // this is for the weighted total, which is long
                            // before division
    unsigned int sum;       // this is for the denominator which is <= 64000
    static int last_value = 0; // assume initially that the line is left.
    
    // calibration한 센서 값을 얻음 (0~1000 사이)
    readCalibrated(sensor_values);

    avg = 0;
    sum = 0;
  
    for (i = 0; i < _numSensors; i++) {
        // calibration을 거친 sensor_value를 저장함.
        int value = sensor_values[i];
        
        if (!white_line)    // 0 값이 아니면 선에 있긴 있는거임
            value = 1000-value; //ex. 980 --> 20
        sensor_values[i] = value;   //20
        
        // robot이 라인 위에 있다고 판단하여 1 저장
        // (5개의 IR센서 중 1개의 센서라도 value값이 300보다 크면 on_line은 1)
        // 모든 IR센서 값이 300보다 작으면 로봇은 라인 위에 없다고 판단한다.
        if (value > 300) {
            on_line = 1;
        }
        
        // noise threshold보다 큰 값이 측정되어 있는 경우, sum에는 value값 누적됨.
        if (value > 50) {
            avg += (long)(value) * (i * 1000);
            sum += value;
        }
    }
    
    // last_value 변수에는 avg/sum의 계산이 진행되고 이를 리턴함 (위의 수식과 동일한 게산하는 중)
    if (!on_line) {
        // If it last read to the left of center, return 0.
         if(last_value < (_numSensors-1)*1000/2)
             return 0;
        
        // If it last read to the right of center, return the max.
         else
             return (_numSensors-1)*1000;
    }
    
    /*
     last_value < 2000: return 0    (IR1 sensor 아래 선이 있다고 판단 -> 선보다 오른쪽으로 치우친 상태)
     last_value > 2000: return 4000 (IR5 sensor 아래 선이 있다고 판단 -> 선보다 왼쪽으로 치우친 상태)
     어느 센서 아래에 선이 위치하는지에 대한 정보 파악 가능.
     
     last_value = 0:                            left on IR1
     last_value > 0 && last_value < 1000:       between IR1 and IR2
     last_value > 1000 && last_value < 2000:    between IR2 and IR3
     last_value > 2000 && last_value < 3000:    between IR3 and IR4
     last_value > 3000 && last_value < 4000:    between IR4 and IR5
     last_value > 4000 && last_value < 5000:    right on IR5
     
     */

    last_value = avg/sum;

    return last_value;
}
