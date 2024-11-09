#include <Arduino.h>
#include "GNC/nav.h"
#include "config.h"

#include "MPU9250.h"
#include "eeprom_utils.h"

#include "TinyGPSPlus.h"

MPU9250  mpu;
float    r_cur  = 0, p_cur  = 0, y_cur  = 0;
float    r_rate = 0, p_rate = 0, y_rate = 0;
uint32_t prev_ms_imu = 0;

TinyGPSPlus gps;
int         sats = 0;
float       hdop = 0, lat = 0, lng = 0;
uint32_t    age = 0;
float       alt_GPS = 0;
uint32_t    prev_ms_gps = 0;

void GNC_init_nav() {
    // GPS Communication Start
    SER_GPS.begin(9600);

    // IMU Communication Start
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
    // mpu.setFilterIterations(10);
}

void GNC_loop_nav() {
    uint32_t curr_ms = millis();

    if (curr_ms - prev_ms_imu >= 10) {
        update_atti();

        prev_ms_imu = curr_ms;
    }

    if (curr_ms - prev_ms_gps >= 100) {
        update_gps();
        prev_ms_gps = curr_ms;
    }

    while (SER_GPS.available()) {
        gps.encode(SER_GPS.read());
    }
}

void update_atti() {
    if (mpu.update()) {
        r_cur  =  mpu.getEulerX();
        p_cur  =  mpu.getEulerY();
        y_cur  =  mpu.getEulerZ();
        r_rate =  mpu.getGyroX();
        p_rate =  mpu.getGyroY();
        y_rate =  mpu.getGyroZ();
    }
}

void update_gps() {
    sats    = gps.satellites.value(); // default : 0
    hdop    = gps.hdop.hdop();        // default : 99.99  // 99.99->poor, 0.01->nice
    lat     = gps.location.lat();     // default : 0.0
    lng     = gps.location.lng();     // default : 0.0
    age     = gps.location.age();     // default : 4294967295 (ms)
    alt_GPS = gps.altitude.meters();  // default : 0.0
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
