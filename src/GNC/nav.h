#ifndef NAV_H
#define NAV_H

#include <Arduino.h>
#include "TinyGPSPlus.h"
#include "MPU9250.h"

// Global variables
extern MPU9250  mpu;
extern float    r_cur, p_cur, y_cur;
extern float    r_rate, p_rate, y_rate;
extern uint32_t prev_ms_imu;

extern TinyGPSPlus gps;
extern int         sats;
extern float       hdop, lat, lng;
extern uint32_t    age;
extern float       alt_GPS;
extern uint32_t    prev_ms_gps;

// Function declarations
void GNC_init_nav();
void GNC_loop_nav();
void update_atti();
void update_gps();
void calibration();
void print_calibration();

#endif
