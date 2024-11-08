
// #define DEBUG_BT

// #define SER_BT Serial1  // 115200
#define SER_BT Serial7  // 115200
#define SER_GPS Serial5 // 9600
#define SER_LD Serial2  // 115200
// #define SER_EX2 Serial7
#define MTR_1 3     // UL   M1       M2
#define MTR_2 4     // UR      . .
#define MTR_3 5     // DL      . .
#define MTR_4 6     // DR   M3       M4
#define LED_1 9  // battery level
#define LED_2 10 // arming
#define LED_3 11 //
#define LED_4 12 // gps status
#define LED_5 24 // transmitter conn status
#define ECHO A12
#define TRIG A13
#define BUZ 15
#define MNT 14
#define PPM_INT 40
#define PPM_CH 8
#define DIST_MAX 200*58.2

#include <Arduino.h>
#include <Servo.h>
#include "PPMReader.h"
#include "MPU9250.h"
#include "TinyGPSPlus.h"
#include "eeprom_utils.h"


/* #region Global variables */
PPMReader ppm(PPM_INT, PPM_CH); unsigned A=0, E=0, T=0, R=0, U1=0, U2=0, U3=0, U4=0, conn=0, armed=0, mode=0;
float r_tar=0, p_tar=0, y_tar=0;
float Balr=0, Balp=0, Baly=0, r_err=0, p_err=0, y_err=0;
MPU9250 mpu; float r_cur=0, p_cur=0, y_cur=0, r_rate=0, p_rate=0, y_rate=0; uint32_t prev_ms_imu=0;
uint32_t prev_ms_dbg=0;
uint64_t prev_us_loop=0, loop_time=0;
float alt_US=0; float alt_LD; byte arr[10] = {0,}; int idx=0;
TinyGPSPlus gps; float hdop=0, lat=0, lng=0, alt_GPS=0; int sats=0; uint32_t age=0, prev_ms_gps=0;
float vbat = 0;
Servo mtr1, mtr2, mtr3, mtr4; int mtr1_speed=0, mtr2_speed=0, mtr3_speed=0, mtr4_speed=0;
float Kp = 1.0; float Ki = 0.0; float Kd = 1.0; unsigned long dt_now=0, dt_prev=0; double dt=0;
/* #endregion*/


/* #region Function */
void update_ppm() {
    A = ppm.latestValidChannelValue(1, 0);
    E = ppm.latestValidChannelValue(2, 0);
    T = ppm.latestValidChannelValue(3, 0);
    R = ppm.latestValidChannelValue(4, 0);
    U1 = ppm.latestValidChannelValue(5, 0);
    U2 = ppm.latestValidChannelValue(6, 0);
    U3 = ppm.latestValidChannelValue(7, 0);
    U4 = ppm.latestValidChannelValue(8, 0);
    if (T == 0) {
        conn = 0;
    } else {
        conn = 1;
    }

    if (conn == 1) {
        r_tar = map(A, 1000, 2000, -20, 20);
        p_tar = map(E, 1000, 2000, -20, 20);
        y_tar = map(R, 1000, 2000, -20, 20);
        mode = map(U1, 1000, 2000, 3, 1);
        if (U3 < 1500) {
            armed = 1;
        } else {
            armed = 0;
        }
    } else {
        r_tar = 0;
        p_tar = 0;
        y_tar = y_cur;
        mode = 0;
        armed = 0;
    }
}

void update_gps() {
    sats = gps.satellites.value();   // default : 0
    hdop = gps.hdop.hdop();          // default : 99.99  // 99.99->poor, 0.01->nice
    lat = gps.location.lat();        // default : 0.0
    lng = gps.location.lng();        // default : 0.0
    age = gps.location.age();        // default : 4294967295 (ms)
    alt_GPS = gps.altitude.meters(); // default : 0.0
}

void calibration() {
    Serial.println("Accel Gyro calibration will start.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);

    digitalWrite(BUZ, HIGH);
    delay(100);
    digitalWrite(BUZ, LOW);
    delay(1000);
    digitalWrite(BUZ, HIGH);
    delay(100);
    digitalWrite(BUZ, LOW);
    delay(1000);
    digitalWrite(BUZ, HIGH);
    delay(100);
    digitalWrite(BUZ, LOW);
    delay(1000);
    mpu.calibrateAccelGyro();
    digitalWrite(BUZ, HIGH);
    delay(1000);
    digitalWrite(BUZ, LOW);
    delay(3000);

    Serial.println("Mag calibration will start.");
    Serial.println("Please Wave device in a figure eight until done.");

    digitalWrite(BUZ, HIGH);
    delay(50);
    digitalWrite(BUZ, LOW);
    delay(50);
    digitalWrite(BUZ, HIGH);
    delay(50);
    digitalWrite(BUZ, LOW);
    delay(50);
    mpu.calibrateMag();
    digitalWrite(BUZ, HIGH);
    delay(1000);
    digitalWrite(BUZ, LOW);

    // print_calibration();
    mpu.verbose(false);

    saveCalibration();
    loadCalibration();
}

void update_led() {
    // 1 : about battery
    // 2 : about 
    // 3 : about GPS
    // 4 : about arming state (armed)
    // 5 : about transmitter connection
    if (conn) {
        digitalWrite(LED_5, HIGH);
    } else {
        digitalWrite(LED_5, LOW);
    }
    if (armed) {
        digitalWrite(LED_4, HIGH);
    } else {
        digitalWrite(LED_4, LOW);
    }
    if(gps.location.age() < 2000){
        digitalWrite(LED_3, HIGH);
        if(millis() > prev_ms_gps + 100){
            prev_ms_gps = millis();
        }
    }else{
        digitalWrite(LED_3, LOW);
    }
}

void update_vbat(){
    vbat = ((float)analogRead(MNT)*11) * (3.3/1023.0);
}

void update_atti(){
    if(mpu.update()){ // 100Hz refresh
            r_cur = mpu.getEulerX();
            p_cur = -mpu.getEulerY();
            y_cur = mpu.getEulerZ();
            r_rate = mpu.getGyroX();
            p_rate = -mpu.getGyroY();
            y_rate = mpu.getGyroZ();
    }
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}

int T2180(int T){
    return (int)(((float)T-1000.0)/1000.0 * 180.0);
}

/* #endregion */

// ======== ======== ======== ========

void setup() {
    Serial.begin(115200);
    SER_BT.begin(115200); // 115200,1,0
    SER_LD.begin(115200);
    SER_GPS.begin(9600);

    mtr1.attach(MTR_1, 1000, 2000);mtr2.attach(MTR_2, 1000, 2000);
    mtr3.attach(MTR_3, 1000, 2000);mtr4.attach(MTR_4, 1000, 2000);
    mtr1.write(0);mtr2.write(0);mtr3.write(0);mtr4.write(0);

    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);
    pinMode(LED_4, OUTPUT);
    pinMode(LED_5, OUTPUT);
    pinMode(BUZ, OUTPUT);
    pinMode(MNT, INPUT);

    Wire.begin();
    delay(2000);
    if (!mpu.setup(0x68)) {
        while (1) {
            Serial.println("MPU connection failed");
            digitalWrite(BUZ, HIGH);
            delay(50);
            digitalWrite(BUZ, LOW);
            delay(50);
        }
    }
    loadCalibration();
    mpu.setFilterIterations(10);
}


const unsigned long interval_print = 10;
unsigned long previousMillis = 0;

void loop() {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval_print) {
        previousMillis = currentMillis;
        
        Serial.print(r_cur);
        Serial.print("\t");
        Serial.print(p_cur);
        Serial.print("\t");
        Serial.print(y_cur);
        Serial.print("\t");
        Serial.println();
        
        // Serial.print(sats);
        // Serial.print("\t");
        // Serial.print(lat, 6);
        // Serial.print("\t");
        // Serial.print(lng, 6);
        // Serial.print("\t");
        // Serial.print(alt_GPS);
        // Serial.print("\t");
        // Serial.println();
    }


    if (SER_BT.available()) {
        char tmp = SER_BT.read();
        if (tmp == 'c') { calibration(); }
        if (tmp == 'P') { Kp += 0.1; }
        if (tmp == 'p') { Kp -= 0.1; }
        if (tmp == 'I') { Ki += 0.1; }
        if (tmp == 'i') { Ki -= 0.1; }
        if (tmp == 'D') { Kd += 0.1; }
        if (tmp == 'd') { Kd -= 0.1; }
    }
    if(SER_LD.available()){
        byte t = SER_LD.read();
        if(t == 89){
            idx = 0;
        } else{
            arr[idx] = t;
            idx++;
        }
        if(idx == 7){
            alt_LD = (int)arr[0];
        }
    }
    while (SER_GPS.available()) { gps.encode(SER_GPS.read()); }
    update_gps();
    update_ppm();    // ppm update
    update_vbat();  // vbat update
    update_atti();  // attitude update
    update_led();   // LED ON/OFF

    // Controller start
    r_err = r_tar - r_cur;
    p_err = p_tar - p_cur;
    if(mode == 1){ y_err = 0; }
    else{ y_err = y_tar - y_cur; }

    dt_now = micros();
    dt = (dt_now - dt_prev) / 1000000.0;
    dt_prev = dt_now;

    Balr  = Kp * r_err;       Balp  = Kp * p_err;       Baly  = Kp * y_err;
    Balr += Kd * -r_rate;     Balp += Kd * -p_rate;     Baly += Kd * -y_rate;
    Balr += Ki * r_err * dt;  Balp += Ki * p_err * dt;  Baly += Ki * y_err * dt;
    
    if (armed){
        mtr1_speed = constrain(T2180(T) + Balp + Balr + Baly, 0, 180);
        mtr2_speed = constrain(T2180(T) + Balp - Balr - Baly, 0, 180);
        mtr3_speed = constrain(T2180(T) - Balp + Balr - Baly, 0, 180);
        mtr4_speed = constrain(T2180(T) - Balp - Balr + Baly, 0, 180);
    } else{
        mtr1_speed = 0;  mtr2_speed = 0;  mtr3_speed = 0;  mtr4_speed = 0;
    }

    // Motor update
    mtr1.write(mtr1_speed);  mtr2.write(mtr2_speed);  mtr3.write(mtr3_speed);  mtr4.write(mtr4_speed);

    // Loop Time Calc
    loop_time = micros() - prev_us_loop;
    prev_us_loop = micros();

#ifdef DEBUG_BT
    if (millis() > prev_ms_dbg + 50) {  // 10 : 100Hz, 20 : 50Hz, 50 : 20Hz, 100 : 10Hz,
        // Attitude
        SER_BT.print("r_cur"); SER_BT.print(':'); SER_BT.print(r_cur, 3); SER_BT.print(',');
        SER_BT.print("p_cur"); SER_BT.print(':'); SER_BT.print(p_cur, 3); SER_BT.print(',');
        SER_BT.print("y_cur"); SER_BT.print(':'); SER_BT.print(y_cur, 3); SER_BT.print(',');

        // Attitude rate
        SER_BT.print("r_rate"); SER_BT.print(':'); SER_BT.print(r_rate, 3); SER_BT.print(',');
        SER_BT.print("p_rate"); SER_BT.print(':'); SER_BT.print(p_rate, 3); SER_BT.print(',');
        SER_BT.print("y_rate"); SER_BT.print(':'); SER_BT.print(y_rate, 3); SER_BT.print(',');

        // Attitude target
        SER_BT.print("r_tar"); SER_BT.print(':'); SER_BT.print(r_tar, 3); SER_BT.print(',');
        SER_BT.print("p_tar"); SER_BT.print(':'); SER_BT.print(p_tar, 3); SER_BT.print(',');
        SER_BT.print("y_tar"); SER_BT.print(':'); SER_BT.print(y_tar, 3); SER_BT.print(',');

        // Motor speed (1000~2000)
        SER_BT.print("mtr1"); SER_BT.print(':'); SER_BT.print(mtr1_speed); SER_BT.print(',');
        SER_BT.print("mtr2"); SER_BT.print(':'); SER_BT.print(mtr2_speed); SER_BT.print(',');
        SER_BT.print("mtr3"); SER_BT.print(':'); SER_BT.print(mtr3_speed); SER_BT.print(',');
        SER_BT.print("mtr4"); SER_BT.print(':'); SER_BT.print(mtr4_speed); SER_BT.print(',');

        // Gain
        SER_BT.print("Kp"); SER_BT.print(':'); SER_BT.print(Kp); SER_BT.print(',');
        SER_BT.print("Ki"); SER_BT.print(':'); SER_BT.print(Ki); SER_BT.print(',');
        SER_BT.print("Kd"); SER_BT.print(':'); SER_BT.print(Kd); SER_BT.print(',');

        // Mode
        SER_BT.print("mode"); SER_BT.print(':'); SER_BT.print(mode); SER_BT.print(',');

        // PPM
        SER_BT.print("A");  SER_BT.print(':'); SER_BT.print(A);  SER_BT.print(',');
        SER_BT.print("E");  SER_BT.print(':'); SER_BT.print(E);  SER_BT.print(',');
        SER_BT.print("T");  SER_BT.print(':'); SER_BT.print(T);  SER_BT.print(',');
        SER_BT.print("R");  SER_BT.print(':'); SER_BT.print(R);  SER_BT.print(',');
        SER_BT.print("U1"); SER_BT.print(':'); SER_BT.print(U1); SER_BT.print(',');
        SER_BT.print("U2"); SER_BT.print(':'); SER_BT.print(U2); SER_BT.print(',');
        SER_BT.print("U3"); SER_BT.print(':'); SER_BT.print(U3); SER_BT.print(',');
        SER_BT.print("U4"); SER_BT.print(':'); SER_BT.print(U4); SER_BT.print(',');

        // Altitude
        SER_BT.print("alt_LD"); SER_BT.print(':'); SER_BT.print(alt_LD); SER_BT.print(',');

        // GPS
        SER_BT.print("sats");    SER_BT.print(':'); SER_BT.print(sats);    SER_BT.print(',');   // default : 0
        SER_BT.print("hdop");    SER_BT.print(':'); SER_BT.print(hdop);    SER_BT.print(',');  // default : 99.99  // 99.99->poor, 0.01->nice
        SER_BT.print("lat");     SER_BT.print(':'); SER_BT.print(lat,6);   SER_BT.print(',');   // default : 0.0
        SER_BT.print("lng");     SER_BT.print(':'); SER_BT.print(lng,6);   SER_BT.print(',');   // default : 0.0
        SER_BT.print("age");     SER_BT.print(':'); SER_BT.print(age);     SER_BT.print(',');   // default : 4294967295
        SER_BT.print("alt_GPS"); SER_BT.print(':'); SER_BT.print(alt_GPS); SER_BT.print(',');    // default : 0.0

        // Loop time
        SER_BT.print("loop_time"); SER_BT.print(':'); SER_BT.print(loop_time); SER_BT.print(',');

        // Battery Voltage
        SER_BT.print("vbat"); SER_BT.print(':'); SER_BT.print(vbat,3); SER_BT.print(',');

        // Arming check
        SER_BT.print("armed"); SER_BT.print(':'); SER_BT.print(armed); SER_BT.print(',');

        // END
        SER_BT.println();

        prev_ms_dbg = millis();
    }
#endif
}